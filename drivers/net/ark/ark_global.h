/*-
 * BSD LICENSE
 *
 * Copyright (c) 2015-2017 Atomic Rules LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * * Neither the name of copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ARK_GLOBAL_H_
#define _ARK_GLOBAL_H_

#include <time.h>
#include <assert.h>

#include <rte_mbuf.h>
#include <rte_ethdev.h>
#include <rte_malloc.h>
#include <rte_memcpy.h>
#include <rte_string_fns.h>
#include <rte_cycles.h>
#include <rte_kvargs.h>
#include <rte_dev.h>
#include <rte_version.h>

#include "ark_pktdir.h"
#include "ark_pktgen.h"
#include "ark_pktchkr.h"

#define ETH_ARK_ARG_MAXLEN	64
#define ARK_SYSCTRL_BASE  0x0
#define ARK_PKTGEN_BASE   0x10000
#define ARK_MPURx_BASE    0x20000
#define ARK_UDM_BASE      0x30000
#define ARK_MPUTx_BASE    0x40000
#define ARK_DDM_BASE      0x60000
#define ARK_CMAC_BASE     0x80000
#define ARK_PKTDIR_BASE   0xA0000
#define ARK_PKTCHKR_BASE  0x90000
#define ARK_RCPACING_BASE 0xB0000
#define ARK_EXTERNAL_BASE 0x100000
#define ARK_MPU_QOFFSET   0x00100
#define ARK_MAX_PORTS     8

#define Offset8(n)     n
#define Offset16(n)   (n/2)
#define Offset32(n)   (n/4)
#define Offset64(n)   (n/8)

/*
 * Structure to store private data for each PF/VF instance.
 */
#define DefPtr(type, name)	\
  union type {      \
	uint64_t *t64; \
	uint32_t *t32; \
	uint16_t *t16; \
	uint8_t  *t8;  \
	void     *v;  \
  } name

#define SetPtr(bar, ark, mem, off) {	    \
  ark->mem.t64 = (uint64_t *)&ark->bar[off]; \
  ark->mem.t32 = (uint32_t *)&ark->bar[off]; \
  ark->mem.t16 = (uint16_t *)&ark->bar[off]; \
  ark->mem.t8  = (uint8_t *)&ark->bar[off]; \
  }

struct ark_port {
	struct rte_eth_dev *eth_dev;
	int id;
};

struct ark_user_ext {
	void *(*dev_init) (struct rte_eth_dev *, void *abar, int port_id);
	void (*dev_uninit) (struct rte_eth_dev *, void *);
	int (*dev_get_port_count) (struct rte_eth_dev *, void *);
	int (*dev_configure) (struct rte_eth_dev *, void *);
	int (*dev_start) (struct rte_eth_dev *, void *);
	void (*dev_stop) (struct rte_eth_dev *, void *);
	void (*dev_close) (struct rte_eth_dev *, void *);
	int (*link_update) (struct rte_eth_dev *, int wait_to_complete, void *);
	int (*dev_set_link_up) (struct rte_eth_dev *, void *);
	int (*dev_set_link_down) (struct rte_eth_dev *, void *);
	void (*stats_get) (struct rte_eth_dev *, struct rte_eth_stats *, void *);
	void (*stats_reset) (struct rte_eth_dev *, void *);
	void (*mac_addr_add) (struct rte_eth_dev *,
	struct ether_addr *, uint32_t, uint32_t, void *);
	void (*mac_addr_remove) (struct rte_eth_dev *, uint32_t, void *);
	void (*mac_addr_set) (struct rte_eth_dev *, struct ether_addr *, void *);
};

struct ark_adapter {

	/* User extension private data */
	void *user_data;

	/* Pointers to packet generator and checker */
	int start_pg;
	ArkPktGen_t pg;
	ArkPktChkr_t pc;
	ArkPktDir_t pd;

	struct ark_port port[ARK_MAX_PORTS];
	int num_ports;

	/* Common for both PF and VF */
	struct rte_eth_dev *eth_dev;

	void *dHandle;
	struct ark_user_ext user_ext;

	/* Our Bar 0 */
	uint8_t *bar0;

	/* A Bar */
	uint8_t *Abar;

	/* Arkville demo block offsets */
	 DefPtr(SysCtrl, sysctrl);
	 DefPtr(PktGen, pktgen);
	 DefPtr(MpuRx, mpurx);
	 DefPtr(UDM, udm);
	 DefPtr(MpuTx, mputx);
	 DefPtr(DDM, ddm);
	 DefPtr(CMAC, cmac);
	 DefPtr(External, external);
	 DefPtr(PktDir, pktdir);
	 DefPtr(PktChkr, pktchkr);

	int started;
	uint16_t rxQueues;
	uint16_t txQueues;

	struct ark_rqpace_t *rqpacing;
};

typedef uint32_t *ark_t;

#endif
