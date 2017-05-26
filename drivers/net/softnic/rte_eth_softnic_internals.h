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

#ifndef __INCLUDE_RTE_ETH_SOFTNIC_INTERNALS_H__
#define __INCLUDE_RTE_ETH_SOFTNIC_INTERNALS_H__

#include <stdint.h>

#include <rte_mbuf.h>
#include <rte_ethdev.h>

#include "rte_eth_softnic.h"

struct pmd_internals {
	/* Devices */
	struct rte_eth_dev *odev;
	struct rte_eth_dev *udev;
	struct rte_eth_dev_data *odata;
	struct rte_eth_dev_data *udata;
	struct eth_dev_ops *odev_ops;
	const struct eth_dev_ops *udev_ops;
	uint8_t oport_id;
	uint8_t uport_id;

	/* Operation */
	struct rte_mbuf *pkts[RTE_ETH_SOFTNIC_DEQ_BSZ_MAX];
	uint32_t deq_bsz;
	uint32_t txq_id;
};

void
pmd_ops_inherit(struct eth_dev_ops *o, const struct eth_dev_ops *u);

void
pmd_ops_derive(struct eth_dev_ops *o, const struct eth_dev_ops *u);

#endif /* __INCLUDE_RTE_ETH_SOFTNIC_INTERNALS_H__ */
