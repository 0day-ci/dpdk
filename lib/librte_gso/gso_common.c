/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdbool.h>
#include <errno.h>

#include <rte_memcpy.h>
#include <rte_mempool.h>
#include <rte_ether.h>
#include <rte_gre.h>
#include <rte_ip.h>
#include <rte_tcp.h>
#include <rte_udp.h>

#include "gso_common.h"

static inline void
hdr_segment_init(struct rte_mbuf *hdr_segment, struct rte_mbuf *pkt,
		uint16_t pkt_hdr_offset)
{
	/* Copy MBUF metadata */
	hdr_segment->nb_segs = 1;
	hdr_segment->port = pkt->port;
	hdr_segment->ol_flags = pkt->ol_flags;
	hdr_segment->packet_type = pkt->packet_type;
	hdr_segment->pkt_len = pkt_hdr_offset;
	hdr_segment->data_len = pkt_hdr_offset;
	hdr_segment->tx_offload = pkt->tx_offload;

	/* Copy the packet header */
	rte_memcpy(rte_pktmbuf_mtod(hdr_segment, char *),
			rte_pktmbuf_mtod(pkt, char *),
			pkt_hdr_offset);
}

static inline void
free_gso_segment(struct rte_mbuf **pkts, uint16_t nb_pkts)
{
	uint16_t i;

	for (i = 0; i < nb_pkts; i++)
		rte_pktmbuf_free(pkts[i]);
}

int
gso_do_segment(struct rte_mbuf *pkt,
		uint16_t pkt_hdr_offset,
		uint16_t pyld_unit_size,
		struct rte_mempool *direct_pool,
		struct rte_mempool *indirect_pool,
		struct rte_mbuf **pkts_out,
		uint16_t nb_pkts_out)
{
	struct rte_mbuf *pkt_in;
	struct rte_mbuf *hdr_segment, *pyld_segment, *prev_segment;
	uint16_t pkt_in_data_pos, segment_bytes_remaining;
	uint16_t pyld_len, nb_segs;
	bool more_in_pkt, more_out_segs;

	pkt_in = pkt;
	nb_segs = 0;
	more_in_pkt = 1;
	pkt_in_data_pos = pkt_hdr_offset;

	while (more_in_pkt) {
		if (unlikely(nb_segs >= nb_pkts_out)) {
			free_gso_segment(pkts_out, nb_segs);
			return -EINVAL;
		}

		/* Allocate a direct MBUF */
		hdr_segment = rte_pktmbuf_alloc(direct_pool);
		if (unlikely(hdr_segment == NULL)) {
			free_gso_segment(pkts_out, nb_segs);
			return -ENOMEM;
		}
		/* Fill the packet header */
		hdr_segment_init(hdr_segment, pkt, pkt_hdr_offset);

		prev_segment = hdr_segment;
		segment_bytes_remaining = pyld_unit_size;
		more_out_segs = 1;

		while (more_out_segs && more_in_pkt) {
			/* Allocate an indirect MBUF */
			pyld_segment = rte_pktmbuf_alloc(indirect_pool);
			if (unlikely(pyld_segment == NULL)) {
				rte_pktmbuf_free(hdr_segment);
				free_gso_segment(pkts_out, nb_segs);
				return -ENOMEM;
			}
			/* Attach to current MBUF segment of pkt */
			rte_pktmbuf_attach(pyld_segment, pkt_in);

			prev_segment->next = pyld_segment;
			prev_segment = pyld_segment;

			pyld_len = segment_bytes_remaining;
			if (pyld_len + pkt_in_data_pos > pkt_in->data_len)
				pyld_len = pkt_in->data_len - pkt_in_data_pos;

			pyld_segment->data_off = pkt_in_data_pos +
				pkt_in->data_off;
			pyld_segment->data_len = pyld_len;

			/* Update header segment */
			hdr_segment->pkt_len += pyld_len;
			hdr_segment->nb_segs++;

			pkt_in_data_pos += pyld_len;
			segment_bytes_remaining -= pyld_len;

			/* Finish processing a MBUF segment of pkt */
			if (pkt_in_data_pos == pkt_in->data_len) {
				pkt_in = pkt_in->next;
				pkt_in_data_pos = 0;
				if (pkt_in == NULL)
					more_in_pkt = 0;
			}

			/* Finish generating a GSO segment */
			if (segment_bytes_remaining == 0)
				more_out_segs = 0;
		}
		pkts_out[nb_segs++] = hdr_segment;
	}
	return nb_segs;
}

static inline void
update_inner_tcp4_header(struct rte_mbuf *pkt, uint8_t ipid_delta,
		struct rte_mbuf **segs, uint16_t nb_segs)
{
	struct tcp_hdr *tcp_hdr;
	struct ipv4_hdr *ipv4_hdr;
	struct rte_mbuf *seg;
	uint32_t sent_seq;
	uint16_t inner_l2_offset;
	uint16_t id, i;

	inner_l2_offset = pkt->outer_l2_len + pkt->outer_l3_len + pkt->l2_len;
	ipv4_hdr = (struct ipv4_hdr *)(rte_pktmbuf_mtod(pkt, char *) +
			inner_l2_offset);
	tcp_hdr = (struct tcp_hdr *)((char *)ipv4_hdr + pkt->l3_len);
	id = rte_be_to_cpu_16(ipv4_hdr->packet_id);
	sent_seq = rte_be_to_cpu_32(tcp_hdr->sent_seq);

	for (i = 0; i < nb_segs; i++) {
		seg = segs[i];
		/* Update the inner IPv4 header */
		ipv4_hdr = (struct ipv4_hdr *)(rte_pktmbuf_mtod(seg, char *) +
				inner_l2_offset);
		ipv4_hdr->total_length = rte_cpu_to_be_16(seg->pkt_len -
				inner_l2_offset);
		ipv4_hdr->packet_id = rte_cpu_to_be_16(id);
		id += ipid_delta;

		/* Update the inner TCP header */
		tcp_hdr = (struct tcp_hdr *)((char *)ipv4_hdr + seg->l3_len);
		tcp_hdr->sent_seq = rte_cpu_to_be_32(sent_seq);
		if (likely(i < nb_segs - 1))
			tcp_hdr->tcp_flags &= (~(TCP_HDR_PSH_MASK |
						TCP_HDR_FIN_MASK));
		sent_seq += (seg->pkt_len - seg->data_len);
	}
}

static inline void
update_outer_ipv4_header(struct rte_mbuf *pkt, uint16_t id)
{
	struct ipv4_hdr *ipv4_hdr;

	ipv4_hdr = (struct ipv4_hdr *)(rte_pktmbuf_mtod(pkt, char *) +
			pkt->outer_l2_len);
	ipv4_hdr->total_length = rte_cpu_to_be_16(pkt->pkt_len -
			pkt->outer_l2_len);
	ipv4_hdr->packet_id = rte_cpu_to_be_16(id);
}

static inline void
update_outer_udp_header(struct rte_mbuf *pkt)
{
	struct udp_hdr *udp_hdr;
	uint16_t length;

	length = pkt->outer_l2_len + pkt->outer_l3_len;
	udp_hdr = (struct udp_hdr *)(rte_pktmbuf_mtod(pkt, char *) +
			length);
	udp_hdr->dgram_len = rte_cpu_to_be_16(pkt->pkt_len - length);
}

static inline void
update_ipv4_vxlan_tcp4_header(struct rte_mbuf *pkt, uint8_t ipid_delta,
		struct rte_mbuf **segs, uint16_t nb_segs)
{
	struct ipv4_hdr *ipv4_hdr;
	uint16_t i, id;

	ipv4_hdr = (struct ipv4_hdr *)(rte_pktmbuf_mtod(pkt, char *) +
			pkt->outer_l2_len);
	id = rte_be_to_cpu_16(ipv4_hdr->packet_id);
	for (i = 0; i < nb_segs; i++) {
		update_outer_ipv4_header(segs[i], id);
		id += ipid_delta;
		update_outer_udp_header(segs[i]);
	}
	/* Update inner TCP/IPv4 headers */
	update_inner_tcp4_header(pkt, ipid_delta, segs, nb_segs);
}

static inline void
update_ipv4_gre_tcp4_header(struct rte_mbuf *pkt, uint8_t ipid_delta,
		struct rte_mbuf **segs, uint16_t nb_segs)
{
	struct ipv4_hdr *ipv4_hdr;
	uint16_t i, id;

	ipv4_hdr = (struct ipv4_hdr *)(rte_pktmbuf_mtod(pkt, char *) +
			pkt->outer_l2_len);
	id = rte_be_to_cpu_16(ipv4_hdr->packet_id);
	for (i = 0; i < nb_segs; i++) {
		update_outer_ipv4_header(segs[i], id);
		id += ipid_delta;
	}

	/* Update inner TCP/IPv4 headers */
	update_inner_tcp4_header(pkt, ipid_delta, segs, nb_segs);
}

void
gso_update_pkt_headers(struct rte_mbuf *pkt, uint8_t ipid_delta,
		struct rte_mbuf **segs, uint16_t nb_segs)
{
	if (is_ipv4_vxlan_ipv4_tcp(pkt->packet_type))
		update_ipv4_vxlan_tcp4_header(pkt, ipid_delta, segs, nb_segs);
	else if (is_ipv4_gre_ipv4_tcp(pkt->packet_type))
		update_ipv4_gre_tcp4_header(pkt, ipid_delta, segs, nb_segs);
	else if (is_ipv4_tcp(pkt->packet_type))
		update_inner_tcp4_header(pkt, ipid_delta, segs, nb_segs);
}
