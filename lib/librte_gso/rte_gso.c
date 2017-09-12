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

#include <errno.h>

#include <rte_log.h>

#include "rte_gso.h"
#include "gso_common.h"
#include "gso_tcp4.h"
#include "gso_tunnel_tcp4.h"

int
rte_gso_segment(struct rte_mbuf *pkt,
		struct rte_gso_ctx gso_ctx,
		struct rte_mbuf **pkts_out,
		uint16_t nb_pkts_out)
{
	struct rte_mempool *direct_pool, *indirect_pool;
	struct rte_mbuf *pkt_seg;
	uint16_t gso_size;
	uint8_t ipid_delta;
	int ret = 1;

	if (pkt == NULL || pkts_out == NULL || nb_pkts_out < 1)
		return -EINVAL;

	if (gso_ctx.gso_size >= pkt->pkt_len ||
			(pkt->packet_type & gso_ctx.gso_types) !=
			pkt->packet_type) {
		pkts_out[0] = pkt;
		return ret;
	}

	direct_pool = gso_ctx.direct_pool;
	indirect_pool = gso_ctx.indirect_pool;
	gso_size = gso_ctx.gso_size;
	ipid_delta = gso_ctx.ipid_flag == RTE_GSO_IPID_INCREASE;

	if (is_ipv4_vxlan_ipv4_tcp(pkt->packet_type) ||
			is_ipv4_gre_ipv4_tcp(pkt->packet_type)) {
		ret = gso_tunnel_tcp4_segment(pkt, gso_size, ipid_delta,
				direct_pool, indirect_pool,
				pkts_out, nb_pkts_out);
	} else if (is_ipv4_tcp(pkt->packet_type)) {
		ret = gso_tcp4_segment(pkt, gso_size, ipid_delta,
				direct_pool, indirect_pool,
				pkts_out, nb_pkts_out);
	} else
		RTE_LOG(WARNING, GSO, "Unsupported packet type\n");

	if (ret > 1) {
		pkt_seg = pkt;
		while (pkt_seg) {
			rte_mbuf_refcnt_update(pkt_seg, -1);
			pkt_seg = pkt_seg->next;
		}
	}

	return ret;
}
