/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation.
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

#ifndef _RTE_NET_CRC_H_
#define _RTE_NET_CRC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <rte_mbuf.h>

/** CRC types */
enum rte_net_crc_type {
	RTE_NET_CRC16_CCITT = 0,
	RTE_NET_CRC32_ETH,
	RTE_NET_CRC_REQS
};

/** CRC compute mode */
enum rte_net_crc_mode {
	RTE_NET_CRC_SCALAR = 0,
	RTE_NET_CRC_SSE42,
	RTE_NET_CRC_DEFAULT
};

/** CRC calc APIs params */
struct rte_net_crc_params {
	struct rte_mbuf *mbuf;		/**< packet mbuf */
	uint32_t data_offset;		/**< offset to the data */
	uint32_t data_len;			/**< length of the data */
	enum rte_net_crc_type type;	/**< crc type */
};

/**
 * CRC Initialisation API
 *
 *  This API should be called only once to initialise the internal crc
 *  data structue before using CRC compute API.
 *
 * @param crc_mode
 *   crc compute mode
 *
 * @return
 *   0 on success, -1 otherwise
 */

int
rte_net_crc_init(enum rte_net_crc_mode m);

/**
 * CRC compute API
 *
 * @param
 *  structure rte_net_crc_params
 *
 * @return
 *   crc value
 */

int
rte_net_crc_calc(struct rte_net_crc_params *p);

#ifdef __cplusplus
}
#endif


#endif /* _RTE_NET_CRC_H_ */
