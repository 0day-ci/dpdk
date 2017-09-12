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

#ifndef _GSO_COMMON_H_
#define _GSO_COMMON_H_

#include <stdint.h>
#include <rte_mbuf.h>

#define IPV4_HDR_DF_SHIFT 14
#define IPV4_HDR_DF_MASK (1 << IPV4_HDR_DF_SHIFT)

#define TCP_HDR_PSH_MASK ((uint8_t)0x08)
#define TCP_HDR_FIN_MASK ((uint8_t)0x01)

#define ETHER_TCP_PKT (RTE_PTYPE_L2_ETHER | RTE_PTYPE_L4_TCP)
#define ETHER_VLAN_TCP_PKT (RTE_PTYPE_L2_ETHER_VLAN | RTE_PTYPE_L4_TCP)
static inline uint8_t is_ipv4_tcp(uint32_t ptype)
{
	switch (ptype & (~RTE_PTYPE_L3_MASK)) {
	case ETHER_VLAN_TCP_PKT:
	case ETHER_TCP_PKT:
		return RTE_ETH_IS_IPV4_HDR(ptype);
	default:
		return 0;
	}
}

/**
 * Internal function which updates relevant packet headers, following
 * segmentation. This is required to update, for example, the IPv4
 * 'total_length' field, to reflect the reduced length of the now-
 * segmented packet.
 *
 * @param pkt
 *  The original packet.
 * @param ipid_delta
 *  The increasing uint of IP ids.
 * @param segs
 *  Pointer array used for storing mbuf addresses for GSO segments.
 * @param nb_segs
 *  The number of GSO segments placed in segs.
 */
void gso_update_pkt_headers(struct rte_mbuf *pkt, uint8_t ipid_delta,
		struct rte_mbuf **segs, uint16_t nb_segs);

/**
 * Internal function which divides the input packet into small segments.
 * Each of the newly-created segments is organized as a two-segment MBUF,
 * where the first segment is a standard mbuf, which stores a copy of
 * packet header, and the second is an indirect mbuf which points to a
 * section of data in the input packet.
 *
 * @param pkt
 *  Packet to segment.
 * @param pkt_hdr_offset
 *  Packet header offset, measured in bytes.
 * @param pyld_unit_size
 *  The max payload length of a GSO segment.
 * @param direct_pool
 *  MBUF pool used for allocating direct buffers for output segments.
 * @param indirect_pool
 *  MBUF pool used for allocating indirect buffers for output segments.
 * @param pkts_out
 *  Pointer array used to keep the mbuf addresses of output segments. If
 *  the memory space in pkts_out is insufficient, gso_do_segment() fails
 *  and returns -EINVAL.
 * @param nb_pkts_out
 *  The max number of items that pkts_out can keep.
 *
 * @return
 *  - The number of segments created in the event of success.
 *  - Return -ENOMEM if run out of memory in MBUF pools.
 *  - Return -EINVAL for invalid parameters.
 */
int gso_do_segment(struct rte_mbuf *pkt,
		uint16_t pkt_hdr_offset,
		uint16_t pyld_unit_size,
		struct rte_mempool *direct_pool,
		struct rte_mempool *indirect_pool,
		struct rte_mbuf **pkts_out,
		uint16_t nb_pkts_out);
#endif
