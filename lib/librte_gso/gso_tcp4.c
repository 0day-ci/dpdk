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


#include <rte_ether.h>
#include <rte_ip.h>

#include "gso_common.h"
#include "gso_tcp4.h"

int
gso_tcp4_segment(struct rte_mbuf *pkt,
		uint16_t gso_size,
		uint8_t ipid_delta,
		struct rte_mempool *direct_pool,
		struct rte_mempool *indirect_pool,
		struct rte_mbuf **pkts_out,
		uint16_t nb_pkts_out)
{
	struct ipv4_hdr *ipv4_hdr;
	uint16_t tcp_dl;
	uint16_t pyld_unit_size;
	uint16_t hdr_offset;
	int ret = 1;

	ipv4_hdr = (struct ipv4_hdr *)(rte_pktmbuf_mtod(pkt, char *) +
			pkt->l2_len);
	/* Don't process the fragmented packet */
	if (unlikely((ipv4_hdr->fragment_offset & rte_cpu_to_be_16(
						IPV4_HDR_DF_MASK)) == 0)) {
		pkts_out[0] = pkt;
		return ret;
	}

	tcp_dl = rte_be_to_cpu_16(ipv4_hdr->total_length) - pkt->l3_len -
		pkt->l4_len;
	/* Don't process the packet without data */
	if (unlikely(tcp_dl == 0)) {
		pkts_out[0] = pkt;
		return ret;
	}

	hdr_offset = pkt->l2_len + pkt->l3_len + pkt->l4_len;
	pyld_unit_size = gso_size - hdr_offset - ETHER_CRC_LEN;

	/* Segment the payload */
	ret = gso_do_segment(pkt, hdr_offset, pyld_unit_size, direct_pool,
			indirect_pool, pkts_out, nb_pkts_out);
	if (ret > 1)
		gso_update_pkt_headers(pkt, ipid_delta, pkts_out, ret);

	return ret;
}
