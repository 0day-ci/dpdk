/*-
 *   BSD LICENSE
 *
 *   Copyright 2017 6WIND S.A.
 *   Copyright 2017 Mellanox.
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
 *     * Neither the name of 6WIND S.A. nor the names of its
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

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/queue.h>
#include <sys/mount.h>

#include <rte_byteorder.h>
#include <rte_jhash.h>
#include <rte_malloc.h>
#include <rte_eth_tap.h>
#include <tap_flow.h>
#include <tap_autoconf.h>
#include <tap_tcmsgs.h>
#include <tap_bpf.h>

#define ERROR                   0

/*
 * The queue number is offset by 1, to distinguish packets that have
 * gone through this rule (skb->cb[1] != 0) from others.
 */
#define QUEUE_OFFSET 1

/* bpf_insn array matching cls_q section. See tap_bpf_program.c file */
static struct bpf_insn cls_q_insns[] = {
	{0x61,  1,  1,  52, 0x00000000},
	{0x18,  2,  0,   0, 0xdeadbeef},
	{0x00,  0,  0,   0, 0x00000000},
	{0x63, 10,  2,  -4, 0x00000000},
	{0x61,  2, 10,  -4, 0x00000000},
	{0x07,  2,  0,   0, 0x00000001},
	{0x67,  2,  0,   0, 0x00000020},
	{0x77,  2,  0,   0, 0x00000020},
	{0xb7,  0,  0,   0, 0xffffffff},
	{0x1d,  1,  2,   1, 0x00000000},
	{0xb7,  0,  0,   0, 0x00000000},
	{0x95,  0,  0,   0, 0x00000000},
};

/**
 * Load BPF program (section cls_q) into the kernel and return a bpf fd
 *
 * @param queue_idx
 *   Queue index matching packet cb
 *
 * @return
 *   -1 if the BPF program couldn't be loaded. An fd (int) otherwise.
 */
int tap_flow_bpf_cls_q(__u32 queue_idx)
{
	cls_q_insns[1].imm = queue_idx;

	return bpf_load(BPF_PROG_TYPE_SCHED_CLS,
		(struct bpf_insn *)cls_q_insns,
		ARRAY_SIZE(cls_q_insns),
		"Dual BSD/GPL");
}

/* bpf_insn array matching l3_l4 section. see tap_bpf_program.c file */
static struct bpf_insn l3_l4_hash_insns[] = {
	{0xbf,    6,    1,        0, 0x00000000},
	{0x61,    8,    6,       16, 0x00000000},
	{0x61,    7,    6,       76, 0x00000000},
	{0x61,    9,    6,       80, 0x00000000},
	{0x18,    1,    0,        0, 0xdeadbeef},
	{0x00,    0,    0,        0, 0x00000000},
	{0x63,   10,    1,       -4, 0x00000000},
	{0xbf,    2,   10,        0, 0x00000000},
	{0x07,    2,    0,        0, 0xfffffffc},
	{0x18,    1,    1,        0, 0x00000005},
	{0x00,    0,    0,        0, 0x00000000},
	{0x85,    0,    0,        0, 0x00000001},
	{0x55,    0,    0,       21, 0x00000000},
	{0xb7,    1,    0,        0, 0x00000a64},
	{0x6b,   10,    1,      -16, 0x00000000},
	{0x18,    1,    0,        0, 0x69666e6f},
	{0x00,    0,    0,        0, 0x65727567},
	{0x7b,   10,    1,      -24, 0x00000000},
	{0x18,    1,    0,        0, 0x6e207369},
	{0x00,    0,    0,        0, 0x6320746f},
	{0x7b,   10,    1,      -32, 0x00000000},
	{0x18,    1,    0,        0, 0x20737372},
	{0x00,    0,    0,        0, 0x2079656b},
	{0x7b,   10,    1,      -40, 0x00000000},
	{0x18,    1,    0,        0, 0x68736168},
	{0x00,    0,    0,        0, 0x203a2928},
	{0x7b,   10,    1,      -48, 0x00000000},
	{0xb7,    7,    0,        0, 0x00000000},
	{0x73,   10,    7,      -14, 0x00000000},
	{0xbf,    1,   10,        0, 0x00000000},
	{0x07,    1,    0,        0, 0xffffffd0},
	{0xb7,    2,    0,        0, 0x00000023},
	{0x85,    0,    0,        0, 0x00000006},
	{0x05,    0,    0,      457, 0x00000000},
	{0xbf,    3,    7,        0, 0x00000000},
	{0xb7,    1,    0,        0, 0x0000000e},
	{0x61,    2,    6,       20, 0x00000000},
	{0x15,    2,    0,        8, 0x00000000},
	{0x61,    2,    6,       28, 0x00000000},
	{0x55,    2,    0,        6, 0x0000a888},
	{0xb7,    7,    0,        0, 0x00000000},
	{0xbf,    1,    3,        0, 0x00000000},
	{0x07,    1,    0,        0, 0x00000012},
	{0x2d,    1,    9,      447, 0x00000000},
	{0xb7,    1,    0,        0, 0x00000012},
	{0x69,    8,    3,       16, 0x00000000},
	{0xb7,    7,    0,        0, 0x00000003},
	{0x57,    8,    0,        0, 0x0000ffff},
	{0x55,    8,    0,      442, 0x00000008},
	{0x0f,    3,    1,        0, 0x00000000},
	{0xb7,    7,    0,        0, 0x00000000},
	{0xbf,    1,    3,        0, 0x00000000},
	{0xbf,    2,    3,        0, 0x00000000},
	{0x07,    1,    0,        0, 0x00000018},
	{0x2d,    1,    9,      436, 0x00000000},
	{0xbf,    7,    2,        0, 0x00000000},
	{0x71,    2,    7,       12, 0x00000000},
	{0xbf,    3,    2,        0, 0x00000000},
	{0x67,    3,    0,        0, 0x00000038},
	{0xc7,    3,    0,        0, 0x00000020},
	{0x77,    3,    0,        0, 0x0000001f},
	{0x57,    3,    0,        0, 0x2cc681d1},
	{0x67,    2,    0,        0, 0x00000018},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x40000000},
	{0xb7,    1,    0,        0, 0x00000000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x598d03a2},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x20000000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xb31a0745},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x10000000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x66340e8a},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x08000000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xcc681d15},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x04000000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x98d03a2b},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x02000000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x31a07456},
	{0x57,    2,    0,        0, 0x01000000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x6340e8ad},
	{0x71,    2,    7,       13, 0x00000000},
	{0x67,    2,    0,        0, 0x00000010},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00800000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xc681d15b},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00400000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x8d03a2b7},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00200000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x1a07456f},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00100000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x340e8ade},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00080000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x681d15bd},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00040000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xd03a2b7b},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00020000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xa07456f6},
	{0x57,    2,    0,        0, 0x00010000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x40e8aded},
	{0x71,    2,    7,       14, 0x00000000},
	{0x67,    2,    0,        0, 0x00000008},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00008000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x81d15bdb},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00004000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x03a2b7b7},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00002000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x07456f6f},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00001000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x0e8adedf},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000800},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x1d15bdbf},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000400},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x3a2b7b7e},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000200},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x7456f6fd},
	{0x57,    2,    0,        0, 0x00000100},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xe8adedfa},
	{0x71,    2,    7,       15, 0x00000000},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000080},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xd15bdbf4},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000040},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xa2b7b7e9},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000020},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x456f6fd3},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000010},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x8adedfa7},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000008},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x15bdbf4f},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000004},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x2b7b7e9e},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000002},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x56f6fd3d},
	{0x57,    2,    0,        0, 0x00000001},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xadedfa7b},
	{0x71,    4,    7,       16, 0x00000000},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x67,    5,    0,        0, 0x00000038},
	{0xc7,    5,    0,        0, 0x00000020},
	{0xb7,    2,    0,        0, 0xffffffff},
	{0x6d,    5,    2,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x5bdbf4f7},
	{0x67,    4,    0,        0, 0x00000018},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x40000000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xb7b7e9ef},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x20000000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x6f6fd3df},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x10000000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xdedfa7bf},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x08000000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xbdbf4f7f},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x04000000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x7b7e9eff},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x02000000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xf6fd3dff},
	{0x57,    4,    0,        0, 0x01000000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xedfa7bfe},
	{0x71,    4,    7,       17, 0x00000000},
	{0x67,    4,    0,        0, 0x00000010},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00800000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xdbf4f7fc},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00400000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xb7e9eff9},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00200000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x6fd3dff2},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00100000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xdfa7bfe5},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00080000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xbf4f7fca},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00040000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x7e9eff94},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00020000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xfd3dff28},
	{0x57,    4,    0,        0, 0x00010000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xfa7bfe51},
	{0x71,    4,    7,       18, 0x00000000},
	{0x67,    4,    0,        0, 0x00000008},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00008000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xf4f7fca2},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00004000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xe9eff945},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00002000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xd3dff28a},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00001000},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xa7bfe514},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00000800},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x4f7fca28},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00000400},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x9eff9450},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00000200},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x3dff28a0},
	{0x57,    4,    0,        0, 0x00000100},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x7bfe5141},
	{0x71,    4,    7,       19, 0x00000000},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00000080},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xf7fca283},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00000040},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xeff94506},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00000020},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xdff28a0c},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00000010},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xbfe51418},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00000008},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x7fca2831},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00000004},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xff945063},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x57,    5,    0,        0, 0x00000002},
	{0x1d,    5,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xff28a0c6},
	{0x57,    4,    0,        0, 0x00000001},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xfe51418c},
	{0x71,    4,    7,       20, 0x00000000},
	{0x67,    4,    0,        0, 0x00000008},
	{0x71,    5,    7,       21, 0x00000000},
	{0x4f,    4,    5,        0, 0x00000000},
	{0xbf,    5,    4,        0, 0x00000000},
	{0x67,    5,    0,        0, 0x00000030},
	{0xc7,    5,    0,        0, 0x00000020},
	{0x6d,    5,    2,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xfca28319},
	{0x67,    4,    0,        0, 0x00000010},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x40000000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xf9450633},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x20000000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xf28a0c67},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x10000000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xe51418ce},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x08000000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xca28319d},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x04000000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x9450633b},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x02000000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x28a0c676},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x01000000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x51418ced},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x00800000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xa28319db},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x00400000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x450633b6},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x00200000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x8a0c676c},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x00100000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x1418ced8},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x00080000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x28319db1},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x00040000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x50633b63},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x00020000},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xa0c676c6},
	{0x57,    4,    0,        0, 0x00010000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x418ced8d},
	{0x71,    2,    7,       22, 0x00000000},
	{0x67,    2,    0,        0, 0x00000008},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00008000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x8319db1a},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00004000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x0633b634},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00002000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x0c676c68},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00001000},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x18ced8d1},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000800},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x319db1a3},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000400},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x633b6347},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000200},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xc676c68f},
	{0x57,    2,    0,        0, 0x00000100},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x8ced8d1f},
	{0x71,    2,    7,       23, 0x00000000},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000080},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x19db1a3e},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000040},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x33b6347d},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000020},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x676c68fa},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000010},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xced8d1f4},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000008},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x9db1a3e9},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000004},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x3b6347d2},
	{0xbf,    4,    2,        0, 0x00000000},
	{0x57,    4,    0,        0, 0x00000002},
	{0x1d,    4,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0x76c68fa5},
	{0x57,    2,    0,        0, 0x00000001},
	{0x1d,    2,    1,        1, 0x00000000},
	{0xa7,    3,    0,        0, 0xed8d1f4a},
	{0xbf,    2,    3,        0, 0x00000000},
	{0x67,    2,    0,        0, 0x00000020},
	{0x77,    2,    0,        0, 0x00000020},
	{0x61,    4,    0,      200, 0x00000000},
	{0xbf,    5,    2,        0, 0x00000000},
	{0x3f,    5,    4,        0, 0x00000000},
	{0x2f,    5,    4,        0, 0x00000000},
	{0x1f,    2,    5,        0, 0x00000000},
	{0x57,    2,    0,        0, 0x0000000f},
	{0x67,    2,    0,        0, 0x00000002},
	{0x0f,    0,    2,        0, 0x00000000},
	{0x61,    4,    0,      136, 0x00000000},
	{0xbf,    2,    4,        0, 0x00000000},
	{0x07,    2,    0,        0, 0x00000001},
	{0x63,    6,    2,       52, 0x00000000},
	{0x73,   10,    1,      -12, 0x00000000},
	{0xb7,    1,    0,        0, 0x0a75253d},
	{0x63,   10,    1,      -16, 0x00000000},
	{0x18,    1,    0,        0, 0x71207825},
	{0x00,    0,    0,        0, 0x65756575},
	{0x7b,   10,    1,      -24, 0x00000000},
	{0x18,    1,    0,        0, 0x73616820},
	{0x00,    0,    0,        0, 0x78303d68},
	{0x7b,   10,    1,      -32, 0x00000000},
	{0x18,    1,    0,        0, 0x6c5f3476},
	{0x00,    0,    0,        0, 0x346c5f33},
	{0x7b,   10,    1,      -40, 0x00000000},
	{0x18,    1,    0,        0, 0x3e3e3e3e},
	{0x00,    0,    0,        0, 0x7069203e},
	{0x7b,   10,    1,      -48, 0x00000000},
	{0xbf,    1,   10,        0, 0x00000000},
	{0x07,    1,    0,        0, 0xffffffd0},
	{0xb7,    2,    0,        0, 0x00000025},
	{0x85,    0,    0,        0, 0x00000006},
	{0xb7,    7,    0,        0, 0x00000001},
	{0xbf,    0,    7,        0, 0x00000000},
	{0x95,    0,    0,        0, 0x00000000},
};

/**
 * Load BPF program (section l3_l4) into the kernel and return a bpf fd.
 *
 * @param[in] key_idx
 *   RSS MAP key index
 *
 * @param[in] map_fd
 *   BPF RSS map file descriptor
 *
 * @return
 *   -1 if the BPF program couldn't be loaded. An fd (int) otherwise.
 */
int tap_flow_bpf_calc_l3_l4_hash(__u32 key_idx, int map_fd)
{
	l3_l4_hash_insns[4].imm = key_idx;
	l3_l4_hash_insns[9].imm = map_fd;

	return bpf_load(BPF_PROG_TYPE_SCHED_ACT,
		(struct bpf_insn *)l3_l4_hash_insns,
		ARRAY_SIZE(l3_l4_hash_insns),
		"Dual BSD/GPL");
}


#ifndef __NR_bpf
# if defined(__i386__)
#  define __NR_bpf 357
# elif defined(__x86_64__)
#  define __NR_bpf 321
# elif defined(__aarch64__)
#  define __NR_bpf 280
# elif defined(__sparc__)
#  define __NR_bpf 349
# elif defined(__s390__)
#  define __NR_bpf 351
# else
#  error __NR_bpf not defined. libbpf does not support your arch.
# endif
#endif

/**
 * Helper function to convert a pointer to unsigned 64 bits
 *
 * @param[in] ptr
 *   pointer to address
 *
 * @return
 *   64 bit unsigned long type of pointer address
 */
static inline __u64 ptr_to_u64(const void *ptr)
{
	return (__u64)(unsigned long)ptr;
}

/**
 * Call BPF system call
 *
 * @param[in] cmd
 *   BPF command for program loading, map creation, map entry update, etc
 *
 * @param[in] attr
 *   System call attributes relevant to system call command
 *
 * @param[in] size
 *   size of attr parameter
 *
 * @return
 *   -1 if BPF system call failed, 0 otherwise
 */
static inline int sys_bpf(enum bpf_cmd cmd, union bpf_attr *attr,
			unsigned int size)
{
	return syscall(__NR_bpf, cmd, attr, size);
}

/**
 * Load BPF instructions to kernel
 *
 * @param[in] type
 *   BPF program type: classifieir or action
 *
 * @param[in] insns
 *   Array of BPF instructions (equivalent to BPF instructions)
 *
 * @param[in] insns_cnt
 *   Number of BPF instructions (size of array)
 *
 * @param[in] lincense
 *   License string that must be acknowledged by the kernel
 *
 * @return
 *   -1 if the BPF program couldn't be loaded, fd (file descriptor) otherwise
 */
static int bpf_load(enum bpf_prog_type type,
		  const struct bpf_insn *insns,
		  size_t insns_cnt,
		  const char *license)
{
	union bpf_attr attr;

	bzero(&attr, sizeof(attr));
	attr.prog_type = type;
	attr.insn_cnt = (__u32)insns_cnt;
	attr.insns = ptr_to_u64(insns);
	attr.license = ptr_to_u64(license);
	attr.log_buf = ptr_to_u64(NULL);
	attr.log_level = 0;
	attr.kern_version = 0;

	return sys_bpf(BPF_PROG_LOAD, &attr, sizeof(attr));
}
