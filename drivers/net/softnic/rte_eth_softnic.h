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

#ifndef __INCLUDE_RTE_ETH_SOFTNIC_H__
#define __INCLUDE_RTE_ETH_SOFTNIC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RTE_ETH_SOFTNIC_TXQ_ID_DEFAULT
#define RTE_ETH_SOFTNIC_TXQ_ID_DEFAULT				0
#endif

#ifndef RTE_ETH_SOFTNIC_DEQ_BSZ_MAX
#define RTE_ETH_SOFTNIC_DEQ_BSZ_MAX				256
#endif

#ifndef RTE_ETH_SOFTNIC_DEQ_BSZ_DEFAULT
#define RTE_ETH_SOFTNIC_DEQ_BSZ_DEFAULT				24
#endif

struct rte_eth_softnic_params {
	/**< Name of the overlay network interface (to be created) */
	const char *oname;

	 /**< Name for the underlay network interface (existing) */
	const char *uname;

	/**< TX queue ID for the underlay device */
	uint32_t txq_id;

	/**< Dequeue burst size */
	uint32_t deq_bsz;
};

/**
 * Create a new overlay device
 *
 * @param params
 *    a pointer to a structure rte_eth_softnic_params which contains
 *    all the arguments required for creating the overlay device.
 * @return
 *    0 if device is created successfully,  or -1 on error.
 */
int
rte_eth_softnic_create(struct rte_eth_softnic_params *params);

/**
 * Run the traffic management on the overlay device
 *
 * This function dequeues the scheduled packets from the HQoS scheduler
 * and transmit them onto underlay device interface.
 *
 * @param portid
 *    port id of the overlay device.
 * @return
 *    0.
 */
int
rte_eth_softnic_run(uint8_t port_id);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_RTE_ETH_SOFTNIC_H__ */
