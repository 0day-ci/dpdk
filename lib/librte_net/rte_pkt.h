/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Intel Corporation. All rights reserved.
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

#ifndef _RTE_PKT_H_
#define _RTE_PKT_H_

#include <rte_ip.h>
#include <rte_udp.h>
#include <rte_tcp.h>
#include <rte_sctp.h>

/**
 * Validate general requirements for tx offload in packet.
 */
static inline int
rte_validate_tx_offload(struct rte_mbuf *m)
{
	uint64_t ol_flags = m->ol_flags;

	/* Does packet set any of available offloads? */
	if (!(ol_flags & PKT_TX_OFFLOAD_MASK))
		return 0;

	/* IP checksum can be counted only for IPv4 packet */
	if ((ol_flags & PKT_TX_IP_CKSUM) && (ol_flags & PKT_TX_IPV6))
		return -EINVAL;

	if (ol_flags & (PKT_TX_L4_MASK | PKT_TX_TCP_SEG))
		/* IP type not set */
		if (!(ol_flags & (PKT_TX_IPV4 | PKT_TX_IPV6)))
			return -EINVAL;

	if (ol_flags & PKT_TX_TCP_SEG)
		/* PKT_TX_IP_CKSUM offload not set for IPv4 TSO packet */
		if ((m->tso_segsz == 0) ||
				((ol_flags & PKT_TX_IPV4) && !(ol_flags & PKT_TX_IP_CKSUM)))
			return -EINVAL;

	/* PKT_TX_OUTER_IP_CKSUM set for non outer IPv4 packet. */
	if ((ol_flags & PKT_TX_OUTER_IP_CKSUM) && !(ol_flags & PKT_TX_OUTER_IPV4))
		return -EINVAL;

	return 0;
}

/**
 * Fix pseudo header checksum for TSO and non-TSO tcp/udp packets before
 * hardware tx checksum.
 * For non-TSO tcp/udp packets full pseudo-header checksum is counted and set.
 * For TSO the IP payload length is not included.
 */
static inline int
rte_phdr_cksum_fix(struct rte_mbuf *m)
{
	struct ipv4_hdr *ipv4_hdr;
	struct ipv6_hdr *ipv6_hdr;
	struct tcp_hdr *tcp_hdr;
	struct udp_hdr *udp_hdr;
	uint64_t inner_l3_offset = m->l2_len;

	if (m->ol_flags & PKT_TX_OUTER_IP_CKSUM)
		inner_l3_offset += m->outer_l2_len + m->outer_l3_len;

	if (m->ol_flags & PKT_TX_IPV4) {
		ipv4_hdr = rte_pktmbuf_mtod_offset(m, struct ipv4_hdr *,
				inner_l3_offset);

		if (m->ol_flags & PKT_TX_IP_CKSUM)
			ipv4_hdr->hdr_checksum = 0;

		if ((m->ol_flags & PKT_TX_UDP_CKSUM) == PKT_TX_UDP_CKSUM) {
			/* non-TSO udp */
			udp_hdr = rte_pktmbuf_mtod_offset(m, struct udp_hdr *,
					inner_l3_offset + m->l3_len);
			udp_hdr->dgram_cksum = rte_ipv4_phdr_cksum(ipv4_hdr, m->ol_flags);
		} else if ((m->ol_flags & PKT_TX_TCP_CKSUM) ||
				(m->ol_flags & PKT_TX_TCP_SEG)) {
			/* non-TSO tcp or TSO */
			tcp_hdr = rte_pktmbuf_mtod_offset(m, struct tcp_hdr *,
					inner_l3_offset + m->l3_len);
			tcp_hdr->cksum = rte_ipv4_phdr_cksum(ipv4_hdr, m->ol_flags);
		}
	} else if (m->ol_flags & PKT_TX_IPV6) {
		ipv6_hdr = rte_pktmbuf_mtod_offset(m, struct ipv6_hdr *,
				inner_l3_offset);

		if ((m->ol_flags & PKT_TX_UDP_CKSUM) == PKT_TX_UDP_CKSUM) {
			/* non-TSO udp */
			udp_hdr = rte_pktmbuf_mtod_offset(m, struct udp_hdr *,
					inner_l3_offset + m->l3_len);
			udp_hdr->dgram_cksum = rte_ipv6_phdr_cksum(ipv6_hdr, m->ol_flags);
		} else if ((m->ol_flags & PKT_TX_TCP_CKSUM) ||
				(m->ol_flags & PKT_TX_TCP_SEG)) {
			/* non-TSO tcp or TSO */
			tcp_hdr = rte_pktmbuf_mtod_offset(m, struct tcp_hdr *,
					inner_l3_offset + m->l3_len);
			tcp_hdr->cksum = rte_ipv6_phdr_cksum(ipv6_hdr, m->ol_flags);
		}
	}
	return 0;
}

#endif /* _RTE_PKT_H_ */
