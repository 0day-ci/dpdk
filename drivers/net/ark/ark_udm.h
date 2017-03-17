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

#ifndef _ARK_UDM_H_
#define _ARK_UDM_H_

#include <stdint.h>

#include <rte_memory.h>

/*
 * UDM hardware structures
 */

#define ARK_RX_WRITE_TIME_NS 2500
#define ARK_UDM_SETUP 0
#define ARK_UDM_CONST 0xBACECACE
struct ark_udm_setup_t {
	uint32_t r0;
	uint32_t r4;
	volatile uint32_t cycleCount;
	uint32_t const0;
};

#define ARK_UDM_CFG 0x010
struct ark_udm_cfg_t {
	volatile uint32_t stopFlushed;	/* RO */
	volatile uint32_t command;
	uint32_t dataroom;
	uint32_t headroom;
};

typedef enum {
	ARK_UDM_START = 0x1,
	ARK_UDM_STOP = 0x2,
	ARK_UDM_RESET = 0x3
} ArkUdmCommands;

#define ARK_UDM_STATS 0x020
struct ark_udm_stats_t {
	volatile uint64_t rxByteCount;
	volatile uint64_t rxPacketCount;
	volatile uint64_t rxMBufCount;
	volatile uint64_t rxSentPackets;
};

#define ARK_UDM_PQ 0x040
struct ark_udm_queue_stats_t {
	volatile uint64_t qByteCount;
	volatile uint64_t qPacketCount;	/* includes drops */
	volatile uint64_t qMbufCount;
	volatile uint64_t qFFPacketCount;
	volatile uint64_t qPktDrop;
	uint32_t qEnable;
};

#define ARK_UDM_TLP 0x0070
struct ark_udm_tlp_t {
	volatile uint64_t pkt_drop;	/* global */
	volatile uint32_t tlp_q1;
	volatile uint32_t tlp_q2;
	volatile uint32_t tlp_q3;
	volatile uint32_t tlp_q4;
	volatile uint32_t tlp_full;
};

#define ARK_UDM_PCIBP 0x00a0
struct ark_udm_pcibp_t {
	volatile uint32_t pci_clear;
	volatile uint32_t pci_empty;
	volatile uint32_t pci_q1;
	volatile uint32_t pci_q2;
	volatile uint32_t pci_q3;
	volatile uint32_t pci_q4;
	volatile uint32_t pci_full;
};

#define ARK_UDM_TLP_PS 0x00bc
struct ark_udm_tlp_ps_t {
	volatile uint32_t tlp_clear;
	volatile uint32_t tlp_ps_min;
	volatile uint32_t tlp_ps_max;
	volatile uint32_t tlp_full_ps_min;
	volatile uint32_t tlp_full_ps_max;
	volatile uint32_t tlp_dw_ps_min;
	volatile uint32_t tlp_dw_ps_max;
	volatile uint32_t tlp_pldw_ps_min;
	volatile uint32_t tlp_pldw_ps_max;
};

#define ARK_UDM_RT_CFG 0x00E0
struct ark_udm_rt_cfg_t {
	phys_addr_t hwProdAddr;
	uint32_t writeInterval;	/* 4ns cycles */
	volatile uint32_t prodIdx;	/* RO */
};

/*  Consolidated structure */
struct ark_udm_t {
	struct ark_udm_setup_t setup;
	struct ark_udm_cfg_t cfg;
	struct ark_udm_stats_t stats;
	struct ark_udm_queue_stats_t qstats;
	uint8_t reserved1[(ARK_UDM_TLP - ARK_UDM_PQ) -
					  sizeof(struct ark_udm_queue_stats_t)];
	struct ark_udm_tlp_t tlp;
	uint8_t reserved2[(ARK_UDM_PCIBP - ARK_UDM_TLP) -
					  sizeof(struct ark_udm_tlp_t)];
	struct ark_udm_pcibp_t pcibp;
	struct ark_udm_tlp_ps_t tlp_ps;
	struct ark_udm_rt_cfg_t rt_cfg;
	int8_t reserved3[(0x100 - ARK_UDM_RT_CFG) -
					  sizeof(struct ark_udm_rt_cfg_t)];
};

#define ARK_UDM_EXPECT_SIZE (0x00fc + 4)
#define ARK_UDM_QOFFSET ARK_UDM_EXPECT_SIZE

int ark_udm_verify(struct ark_udm_t *udm);
int ark_udm_stop(struct ark_udm_t *udm, int wait);
void ark_udm_start(struct ark_udm_t *udm);
int ark_udm_reset(struct ark_udm_t *udm);
void ark_udm_configure(struct ark_udm_t *udm,
					   uint32_t headroom,
					   uint32_t dataroom,
					   uint32_t write_interval_ns);
void ark_udm_write_addr(struct ark_udm_t *udm, phys_addr_t addr);
void ark_udm_stats_reset(struct ark_udm_t *udm);
void ark_udm_dump_stats(struct ark_udm_t *udm, const char *msg);
void ark_udm_dump_queue_stats(struct ark_udm_t *udm, const char *msg,
							  uint16_t qid);
void ark_udm_dump(struct ark_udm_t *udm, const char *msg);
void ark_udm_dump_perf(struct ark_udm_t *udm, const char *msg);
void ark_udm_dump_setup(struct ark_udm_t *udm, uint16_t qId);
int ark_udm_is_flushed(struct ark_udm_t *udm);

/* Per queue data */
uint64_t ark_udm_dropped(struct ark_udm_t *udm);
uint64_t ark_udm_bytes(struct ark_udm_t *udm);
uint64_t ark_udm_packets(struct ark_udm_t *udm);

void ark_udm_queue_stats_reset(struct ark_udm_t *udm);
void ark_udm_queue_enable(struct ark_udm_t *udm, int enable);

#endif
