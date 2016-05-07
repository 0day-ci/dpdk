/*
 *   BSD LICENSE
 *
 *   Copyright (C) Cavium networks Ltd. 2016.
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
 *     * Neither the name of Cavium networks nor the names of its
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

#ifndef __THUNDERX_NICVF_RXTX_H__
#define __THUNDERX_NICVF_RXTX_H__

#include <rte_byteorder.h>
#include <rte_ethdev.h>

#define NICVF_TX_OFFLOAD_MASK (PKT_TX_IP_CKSUM | PKT_TX_L4_MASK)

#ifndef __hot
#define __hot	__attribute__((hot))
#endif

static inline uint16_t __attribute__((const))
nicvf_frag_num(uint16_t i)
{
#if RTE_BYTE_ORDER == RTE_BIG_ENDIAN
	return (i & ~3) + 3 - (i & 3);
#else
	return i;
#endif
}

uint16_t nicvf_recv_pkts(void *rxq, struct rte_mbuf **rx_pkts, uint16_t pkts);
uint16_t nicvf_recv_pkts_multiseg(void *rx_queue, struct rte_mbuf **rx_pkts,
				  uint16_t nb_pkts);

uint16_t nicvf_xmit_pkts(void *txq, struct rte_mbuf **tx_pkts, uint16_t pkts);
uint16_t nicvf_xmit_pkts_multiseg(void *txq, struct rte_mbuf **tx_pkts,
				  uint16_t pkts);

#endif /* __THUNDERX_NICVF_RXTX_H__  */
