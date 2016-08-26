/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2016 Intel Corporation. All rights reserved.
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

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <inttypes.h>

#include <sys/queue.h>
#include <sys/stat.h>

#include <rte_common.h>
#include <rte_byteorder.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_cycles.h>
#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_launch.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_ring.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_interrupts.h>
#include <rte_pci.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_string_fns.h>
#include <rte_ip.h>
#include <rte_tcp.h>
#include <rte_udp.h>

#include "testpmd.h"

/* We cannot use rte_cpu_to_be_16() on a constant in a switch/case */
#if RTE_BYTE_ORDER == RTE_LITTLE_ENDIAN
#define _htons(x) ((uint16_t)((((x) & 0x00ffU) << 8) | (((x) & 0xff00U) >> 8)))
#else
#define _htons(x) (x)
#endif

/*
 * Helper function.
 * Performs actual copying.
 * Returns number of segments in the destination mbuf on success,
 * or negative error code on failure.
 */
static int
mbuf_copy_split(const struct rte_mbuf *ms, struct rte_mbuf *md[],
	uint16_t seglen[], uint8_t nb_seg)
{
	uint32_t dlen, slen, tlen;
	uint32_t i, len;
	const struct rte_mbuf *m;
	const uint8_t *src;
	uint8_t *dst;

	dlen = 0;
	slen = 0;
	tlen = 0;

	dst = NULL;
	src = NULL;

	m = ms;
	i = 0;
	while (ms != NULL && i != nb_seg) {

		if (slen == 0) {
			slen = rte_pktmbuf_data_len(ms);
			src = rte_pktmbuf_mtod(ms, const uint8_t *);
		}

		if (dlen == 0) {
			dlen = RTE_MIN(seglen[i], slen);
			md[i]->data_len = dlen;
			md[i]->next = (i + 1 == nb_seg) ? NULL : md[i + 1];
			dst = rte_pktmbuf_mtod(md[i], uint8_t *);
		}

		len = RTE_MIN(slen, dlen);
		memcpy(dst, src, len);
		tlen += len;
		slen -= len;
		dlen -= len;
		src += len;
		dst += len;

		if (slen == 0)
			ms = ms->next;
		if (dlen == 0)
			i++;
	}

	if (ms != NULL)
		return -ENOBUFS;
	else if (tlen != m->pkt_len)
		return -EINVAL;

	md[0]->nb_segs = nb_seg;
	md[0]->pkt_len = tlen;
	md[0]->vlan_tci = m->vlan_tci;
	md[0]->vlan_tci_outer = m->vlan_tci_outer;
	md[0]->ol_flags = m->ol_flags;
	md[0]->tx_offload = m->tx_offload;

	return nb_seg;
}

/*
 * Allocate a new mbuf with up to tx_pkt_nb_segs segments.
 * Copy packet contents and offload information into then new segmented mbuf.
 */
static struct rte_mbuf *
pkt_copy_split(const struct rte_mbuf *pkt)
{
	int32_t n, rc;
	uint32_t i, len, nb_seg;
	struct rte_mempool *mp;
	uint16_t seglen[RTE_MAX_SEGS_PER_PKT];
	struct rte_mbuf *p, *md[RTE_MAX_SEGS_PER_PKT];

	mp = current_fwd_lcore()->mbp;

	if (tx_pkt_split == TX_PKT_SPLIT_RND)
		nb_seg = random() % tx_pkt_nb_segs + 1;
	else
		nb_seg = tx_pkt_nb_segs;

	memcpy(seglen, tx_pkt_seg_lengths, nb_seg * sizeof(seglen[0]));

	/* calculate number of segments to use and their length. */
	len = 0;
	for (i = 0; i != nb_seg && len < pkt->pkt_len; i++) {
		len += seglen[i];
		md[i] = NULL;
	}

	n = pkt->pkt_len - len;

	/* update size of the last segment to fit rest of the packet */
	if (n >= 0) {
		seglen[i - 1] += n;
		len += n;
	}

	nb_seg = i;
	while (i != 0) {
		p = rte_pktmbuf_alloc(mp);
		if (p == NULL) {
			RTE_LOG(ERR, USER1,
				"failed to allocate %u-th of %u mbuf "
				"from mempool: %s\n",
				nb_seg - i, nb_seg, mp->name);
			break;
		}

		md[--i] = p;
		if (rte_pktmbuf_tailroom(md[i]) < seglen[i]) {
			RTE_LOG(ERR, USER1, "mempool %s, %u-th segment: "
				"expected seglen: %u, "
				"actual mbuf tailroom: %u\n",
				mp->name, i, seglen[i],
				rte_pktmbuf_tailroom(md[i]));
			break;
		}
	}

	/* all mbufs successfully allocated, do copy */
	if (i == 0) {
		rc = mbuf_copy_split(pkt, md, seglen, nb_seg);
		if (rc < 0)
			RTE_LOG(ERR, USER1,
				"mbuf_copy_split for %p(len=%u, nb_seg=%hhu) "
				"into %u segments failed with error code: %d\n",
				pkt, pkt->pkt_len, pkt->nb_segs, nb_seg, rc);

		/* figure out how many mbufs to free. */
		i = RTE_MAX(rc, 0);
	}

	/* free unused mbufs */
	for (; i != nb_seg; i++) {
		rte_pktmbuf_free_seg(md[i]);
		md[i] = NULL;
	}

	return md[0];
}

/*
 * Forwarding of packets in I/O mode.
 * Forward packets with tx_prep.
 * This is the fastest possible forwarding operation, as it does not access
 * to packets data.
 */
static void
pkt_burst_txprep_forward(struct fwd_stream *fs)
{
	struct rte_mbuf *pkts_burst[MAX_PKT_BURST];
	struct rte_mbuf *p;
	struct rte_port *txp;
	int i;
	uint16_t nb_rx;
	uint16_t nb_prep;
	uint16_t nb_tx;
#ifdef RTE_TEST_PMD_RECORD_CORE_CYCLES
	uint64_t start_tsc;
	uint64_t end_tsc;
	uint64_t core_cycles;
#endif
	uint16_t tso_segsz = 0;
	uint64_t ol_flags = 0;

	struct ether_hdr *eth_hdr;
	struct vlan_hdr *vlan_hdr;
	struct ipv4_hdr *ipv4_hdr;
	struct ipv6_hdr *ipv6_hdr;
	struct tcp_hdr *tcp_hdr;
	char *l3_hdr = NULL;

	uint8_t l4_proto = 0;

#ifdef RTE_TEST_PMD_RECORD_CORE_CYCLES
	start_tsc = rte_rdtsc();
#endif

	/*
	 * Receive a burst of packets and forward them.
	 */
	nb_rx = rte_eth_rx_burst(fs->rx_port, fs->rx_queue, pkts_burst,
			nb_pkt_per_burst);
	if (unlikely(nb_rx == 0))
		return;

	txp = &ports[fs->tx_port];
	tso_segsz = txp->tso_segsz;

	for (i = 0; i < nb_rx; i++) {

		eth_hdr = rte_pktmbuf_mtod(pkts_burst[i], struct ether_hdr *);
		ether_addr_copy(&peer_eth_addrs[fs->peer_addr],
				&eth_hdr->d_addr);
		ether_addr_copy(&ports[fs->tx_port].eth_addr,
				&eth_hdr->s_addr);

		uint16_t ether_type = eth_hdr->ether_type;

		pkts_burst[i]->l2_len = sizeof(struct ether_hdr);

		ol_flags = 0;

		if (tso_segsz > 0)
			ol_flags |= PKT_TX_TCP_SEG;

		if (ether_type == _htons(ETHER_TYPE_VLAN)) {
			ol_flags |= PKT_TX_VLAN_PKT;
			vlan_hdr = (struct vlan_hdr *)(eth_hdr + 1);
			pkts_burst[i]->l2_len += sizeof(struct vlan_hdr);
			ether_type = vlan_hdr->eth_proto;
		}

		switch (ether_type) {
		case _htons(ETHER_TYPE_IPv4):
			ol_flags |= (PKT_TX_IPV4 | PKT_TX_IP_CKSUM);
			pkts_burst[i]->l3_len = sizeof(struct ipv4_hdr);
			pkts_burst[i]->l4_len = sizeof(struct tcp_hdr);

			ipv4_hdr = (struct ipv4_hdr *)((char *)eth_hdr +
					pkts_burst[i]->l2_len);
			l3_hdr = (char *)ipv4_hdr;
			pkts_burst[i]->l3_len = (ipv4_hdr->version_ihl & 0x0f) * 4;
			l4_proto = ipv4_hdr->next_proto_id;

			break;
		case _htons(ETHER_TYPE_IPv6):
			ol_flags |= PKT_TX_IPV6;

			ipv6_hdr = (struct ipv6_hdr *)((char *)eth_hdr +
					pkts_burst[i]->l2_len);
			l3_hdr = (char *)ipv6_hdr;
			l4_proto = ipv6_hdr->proto;
			pkts_burst[i]->l3_len = sizeof(struct ipv6_hdr);
			break;
		default:
			printf("Unknown packet type\n");
			break;
		}

		if (l4_proto == IPPROTO_TCP) {
			ol_flags |= PKT_TX_TCP_CKSUM;
			tcp_hdr = (struct tcp_hdr *)(l3_hdr + pkts_burst[i]->l3_len);
			pkts_burst[i]->l4_len = (tcp_hdr->data_off & 0xf0) >> 2;
		} else if (l4_proto == IPPROTO_UDP) {
			ol_flags |= PKT_TX_UDP_CKSUM;
			pkts_burst[i]->l4_len = sizeof(struct udp_hdr);
		}

		pkts_burst[i]->tso_segsz = tso_segsz;
		pkts_burst[i]->ol_flags = ol_flags;

		/* Do split & copy for the packet. */
		if (tx_pkt_split != TX_PKT_SPLIT_OFF) {
			p = pkt_copy_split(pkts_burst[i]);
			if (p != NULL) {
				rte_pktmbuf_free(pkts_burst[i]);
				pkts_burst[i] = p;
			}
		}

		/* if verbose mode is enabled, dump debug info */
		if (verbose_level > 0) {
			printf("l2_len=%d, l3_len=%d, l4_len=%d, nb_segs=%d, tso_segz=%d\n",
					pkts_burst[i]->l2_len, pkts_burst[i]->l3_len,
					pkts_burst[i]->l4_len, pkts_burst[i]->nb_segs,
					pkts_burst[i]->tso_segsz);
		}
	}

	/*
	 * Prepare burst to transmit
	 */
	nb_prep = rte_eth_tx_prep(fs->tx_port, fs->tx_queue, pkts_burst, nb_rx);

	if (nb_prep < nb_rx)
		printf("Preparing packet burst to transmit failed: %s\n",
				rte_strerror(rte_errno));

#ifdef RTE_TEST_PMD_RECORD_BURST_STATS
	fs->rx_burst_stats.pkt_burst_spread[nb_rx]++;
#endif
	fs->rx_packets += nb_rx;
	nb_tx = rte_eth_tx_burst(fs->tx_port, fs->tx_queue, pkts_burst, nb_prep);
	fs->tx_packets += nb_tx;
#ifdef RTE_TEST_PMD_RECORD_BURST_STATS
	fs->tx_burst_stats.pkt_burst_spread[nb_tx]++;
#endif
	if (unlikely(nb_tx < nb_rx)) {
		fs->fwd_dropped += (nb_rx - nb_tx);
		do {
			rte_pktmbuf_free(pkts_burst[nb_tx]);
		} while (++nb_tx < nb_rx);
	}
#ifdef RTE_TEST_PMD_RECORD_CORE_CYCLES
	end_tsc = rte_rdtsc();
	core_cycles = (end_tsc - start_tsc);
	fs->core_cycles = (uint64_t) (fs->core_cycles + core_cycles);
#endif
}

static void
txprep_fwd_begin(portid_t pi)
{
	struct rte_eth_dev_info dev_info;

	rte_eth_dev_info_get(pi, &dev_info);
	printf("  nb_seg_max=%d, nb_mtu_seg_max=%d\n",
			dev_info.tx_desc_lim.nb_seg_max,
			dev_info.tx_desc_lim.nb_mtu_seg_max);
}

static void
txprep_fwd_end(portid_t pi __rte_unused)
{
	printf("txprep_fwd_end\n");
}

struct fwd_engine txprep_fwd_engine = {
	.fwd_mode_name  = "txprep",
	.port_fwd_begin = txprep_fwd_begin,
	.port_fwd_end   = txprep_fwd_end,
	.packet_fwd     = pkt_burst_txprep_forward,
};
