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

#ifndef _ARK_PKTGEN_H_
#define _ARK_PKTGEN_H_

#include <stdint.h>
#include <inttypes.h>

#include <rte_eal.h>

#include <rte_ethdev.h>
#include <rte_cycles.h>
#include <rte_lcore.h>
#include <rte_mbuf.h>
#include <rte_malloc.h>

#define ARK_PKTGEN_BASE_ADR  0x10000

typedef void *ArkPktGen_t;

struct ArkPktGenRegs {
	uint32_t r0;
	volatile uint32_t pktStartStop;
	volatile uint32_t pktCtrl;
	uint32_t pktPayload;
	uint32_t pktSpacing;
	uint32_t pktSizeMin;
	uint32_t pktSizeMax;
	uint32_t pktSizeIncr;
	volatile uint32_t numPkts;
	volatile uint32_t pktsSent;
	uint32_t srcMACAddrL;
	uint32_t srcMACAddrH;
	uint32_t dstMACAddrL;
	uint32_t dstMACAddrH;
	uint32_t ethType;
	uint32_t hdrDW[7];
	uint32_t startOffset;
	uint32_t bytesPerCycle;
} __attribute__ ((packed));

struct ArkPktGenInst {
	struct rte_eth_dev_info *dev_info;
	struct ArkPktGenRegs *regs;
	int l2_mode;
	int ordinal;
};

/*  packet generator functions */
ArkPktGen_t ark_pmd_pktgen_init(void *, int ord, int l2_mode);
void ark_pmd_pktgen_uninit(ArkPktGen_t handle);
void ark_pmd_pktgen_run(ArkPktGen_t handle);
void ark_pmd_pktgen_pause(ArkPktGen_t handle);
uint32_t ark_pmd_pktgen_paused(ArkPktGen_t handle);
uint32_t ark_pmd_pktgen_isGenForever(ArkPktGen_t handle);
uint32_t ark_pmd_pktgen_isRunning(ArkPktGen_t handle);
uint32_t ark_pmd_pktgen_txDone(ArkPktGen_t handle);
void ark_pmd_pktgen_reset(ArkPktGen_t handle);
void ark_pmd_pktgen_waitDone(ArkPktGen_t handle);
uint32_t ark_pmd_pktgen_getPktsSent(ArkPktGen_t handle);
void ark_pmd_pktgen_setPayloadByte(ArkPktGen_t handle, uint32_t b);
void ark_pmd_pktgen_setPktSpacing(ArkPktGen_t handle, uint32_t x);
void ark_pmd_pktgen_setPktSizeMin(ArkPktGen_t handle, uint32_t x);
void ark_pmd_pktgen_setPktSizeMax(ArkPktGen_t handle, uint32_t x);
void ark_pmd_pktgen_setPktSizeIncr(ArkPktGen_t handle, uint32_t x);
void ark_pmd_pktgen_setNumPkts(ArkPktGen_t handle, uint32_t x);
void ark_pmd_pktgen_setSrcMACAddr(ArkPktGen_t handle, uint64_t macAddr);
void ark_pmd_pktgen_setDstMACAddr(ArkPktGen_t handle, uint64_t macAddr);
void ark_pmd_pktgen_setEthType(ArkPktGen_t handle, uint32_t x);
void ark_pmd_pktgen_setHdrDW(ArkPktGen_t handle, uint32_t *hdr);
void ark_pmd_pktgen_setStartOffset(ArkPktGen_t handle, uint32_t x);
void ark_pmd_pktgen_parse(char *argv);
void ark_pmd_pktgen_setup(ArkPktGen_t handle);

#endif
