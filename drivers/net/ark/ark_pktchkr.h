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

#ifndef _ARK_PKTCHKR_H_
#define _ARK_PKTCHKR_H_

#include <stdint.h>
#include <inttypes.h>

#include <rte_eal.h>

#include <rte_ethdev.h>
#include <rte_cycles.h>
#include <rte_lcore.h>
#include <rte_mbuf.h>
#include <rte_malloc.h>

#define ARK_PKTCHKR_BASE_ADR  0x90000

typedef void *ArkPktChkr_t;

struct ArkPktChkrStatRegs {
	uint32_t r0;
	uint32_t pktStartStop;
	uint32_t pktCtrl;
	uint32_t pktsRcvd;
	uint64_t bytesRcvd;
	uint32_t pktsOK;
	uint32_t pktsMismatch;
	uint32_t pktsErr;
	uint32_t firstMismatch;
	uint32_t resyncEvents;
	uint32_t pktsMissing;
	uint32_t minLatency;
	uint32_t maxLatency;
} __attribute__ ((packed));

struct ArkPktChkrCtlRegs {
	uint32_t pktCtrl;
	uint32_t pktPayload;
	uint32_t pktSizeMin;
	uint32_t pktSizeMax;
	uint32_t pktSizeIncr;
	uint32_t numPkts;
	uint32_t pktsSent;
	uint32_t srcMACAddrL;
	uint32_t srcMACAddrH;
	uint32_t dstMACAddrL;
	uint32_t dstMACAddrH;
	uint32_t ethType;
	uint32_t hdrDW[7];
} __attribute__ ((packed));

struct ArkPktChkrInst {
	struct rte_eth_dev_info *dev_info;
	volatile struct ArkPktChkrStatRegs *sregs;
	volatile struct ArkPktChkrCtlRegs *cregs;
	int l2_mode;
	int ordinal;
};

/*  packet checker functions */
ArkPktChkr_t ark_pmd_pktchkr_init(void *addr, int ord, int l2_mode);
void ark_pmd_pktchkr_uninit(ArkPktChkr_t handle);
void ark_pmd_pktchkr_run(ArkPktChkr_t handle);
int ark_pmd_pktchkr_stopped(ArkPktChkr_t handle);
void ark_pmd_pktchkr_stop(ArkPktChkr_t handle);
int ark_pmd_pktchkr_isRunning(ArkPktChkr_t handle);
int ark_pmd_pktchkr_getPktsSent(ArkPktChkr_t handle);
void ark_pmd_pktchkr_setPayloadByte(ArkPktChkr_t handle, uint32_t b);
void ark_pmd_pktchkr_setPktSizeMin(ArkPktChkr_t handle, uint32_t x);
void ark_pmd_pktchkr_setPktSizeMax(ArkPktChkr_t handle, uint32_t x);
void ark_pmd_pktchkr_setPktSizeIncr(ArkPktChkr_t handle, uint32_t x);
void ark_pmd_pktchkr_setNumPkts(ArkPktChkr_t handle, uint32_t x);
void ark_pmd_pktchkr_setSrcMACAddr(ArkPktChkr_t handle, uint64_t macAddr);
void ark_pmd_pktchkr_setDstMACAddr(ArkPktChkr_t handle, uint64_t macAddr);
void ark_pmd_pktchkr_setEthType(ArkPktChkr_t handle, uint32_t x);
void ark_pmd_pktchkr_setHdrDW(ArkPktChkr_t handle, uint32_t *hdr);
void ark_pmd_pktchkr_parse(char *args);
void ark_pmd_pktchkr_setup(ArkPktChkr_t handle);
void ark_pmd_pktchkr_dump_stats(ArkPktChkr_t handle);
int ark_pmd_pktchkr_waitDone(ArkPktChkr_t handle);

#endif
