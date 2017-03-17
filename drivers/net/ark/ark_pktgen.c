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

#include <getopt.h>
#include <sys/time.h>
#include <locale.h>
#include <unistd.h>

#include "ark_pktgen.h"
#include "ark_debug.h"

#define ARK_MAX_STR_LEN 64
union OptV {
	int Int;
	int Bool;
	uint64_t Long;
	char Str[ARK_MAX_STR_LEN];
};

enum OPType {
	OTInt,
	OTLong,
	OTBool,
	OTString
};

struct Options {
	char opt[ARK_MAX_STR_LEN];
	enum OPType t;
	union OptV v;
};

static struct Options toptions[] = {
	{{"configure"}, OTBool, {1} },
	{{"dg-mode"}, OTBool, {1} },
	{{"run"}, OTBool, {0} },
	{{"pause"}, OTBool, {0} },
	{{"reset"}, OTBool, {0} },
	{{"dump"}, OTBool, {0} },
	{{"genForever"}, OTBool, {0} },
	{{"enSlavedStart"}, OTBool, {0} },
	{{"varyLength"}, OTBool, {0} },
	{{"incrPayload"}, OTBool, {0} },
	{{"incrFirstByte"}, OTBool, {0} },
	{{"insSeqNum"}, OTBool, {0} },
	{{"insTimeStamp"}, OTBool, {1} },
	{{"insUDPHdr"}, OTBool, {0} },
	{{"numPkts"}, OTLong, .v.Long = 100000000},
	{{"payloadByte"}, OTInt, {0x55} },
	{{"pktSpacing"}, OTInt, {130} },
	{{"pktSizeMin"}, OTInt, {2006} },
	{{"pktSizeMax"}, OTInt, {1514} },
	{{"pktSizeIncr"}, OTInt, {1} },
	{{"ethType"}, OTInt, {0x0800} },
	{{"srcMACAddr"}, OTLong, .v.Long = 0xDC3CF6425060L},
	{{"dstMACAddr"}, OTLong, .v.Long = 0x112233445566L},
	{{"hdrDW0"}, OTInt, {0x0016e319} },
	{{"hdrDW1"}, OTInt, {0x27150004} },
	{{"hdrDW2"}, OTInt, {0x76967bda} },
	{{"hdrDW3"}, OTInt, {0x08004500} },
	{{"hdrDW4"}, OTInt, {0x005276ed} },
	{{"hdrDW5"}, OTInt, {0x40004006} },
	{{"hdrDW6"}, OTInt, {0x56cfc0a8} },
	{{"startOffset"}, OTInt, {0} },
	{{"bytesPerCycle"}, OTInt, {10} },
	{{"shaping"}, OTBool, {0} },
	{{"dstIP"}, OTString, .v.Str = "169.254.10.240"},
	{{"dstPort"}, OTInt, {65536} },
	{{"srcPort"}, OTInt, {65536} },
};

ArkPktGen_t
ark_pmd_pktgen_init(void *adr, int ord, int l2_mode)
{
	struct ArkPktGenInst *inst =
		rte_malloc("ArkPktGenInstPMD", sizeof(struct ArkPktGenInst), 0);
	inst->regs = (struct ArkPktGenRegs *) adr;
	inst->ordinal = ord;
	inst->l2_mode = l2_mode;
	return inst;
}

void
ark_pmd_pktgen_uninit(ArkPktGen_t handle)
{
	rte_free(handle);
}

void
ark_pmd_pktgen_run(ArkPktGen_t handle)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	inst->regs->pktStartStop = 1;
}

uint32_t
ark_pmd_pktgen_paused(ArkPktGen_t handle)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;
	uint32_t r = inst->regs->pktStartStop;

	return (((r >> 16) & 1) == 1);
}

void
ark_pmd_pktgen_pause(ArkPktGen_t handle)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;
	int cnt = 0;

	inst->regs->pktStartStop = 0;

	while (!ark_pmd_pktgen_paused(handle)) {
		usleep(1000);
		if (cnt++ > 100) {
			PMD_DRV_LOG(ERR, "pktgen %d failed to pause.\n", inst->ordinal);
			break;
		}
	}
	ARK_DEBUG_TRACE("pktgen %d paused.\n", inst->ordinal);
}

void
ark_pmd_pktgen_reset(ArkPktGen_t handle)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	if (!ark_pmd_pktgen_isRunning(handle) && !ark_pmd_pktgen_paused(handle)) {
		ARK_DEBUG_TRACE
			("pktgen %d is not running and is not paused. No need to reset.\n",
			 inst->ordinal);
		return;
	}

	if (ark_pmd_pktgen_isRunning(handle) && !ark_pmd_pktgen_paused(handle)) {
		ARK_DEBUG_TRACE("pktgen %d is not paused. Pausing first.\n",
						inst->ordinal);
		ark_pmd_pktgen_pause(handle);
	}

	ARK_DEBUG_TRACE("Resetting pktgen %d.\n", inst->ordinal);
	inst->regs->pktStartStop = (1 << 8);
}

uint32_t
ark_pmd_pktgen_txDone(ArkPktGen_t handle)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;
	uint32_t r = inst->regs->pktStartStop;

	return (((r >> 24) & 1) == 1);
}

uint32_t
ark_pmd_pktgen_isRunning(ArkPktGen_t handle)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;
	uint32_t r = inst->regs->pktStartStop;

	return ((r & 1) == 1);
}

uint32_t
ark_pmd_pktgen_isGenForever(ArkPktGen_t handle)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;
	uint32_t r = inst->regs->pktCtrl;

	return (((r >> 24) & 1) == 1);
}

void
ark_pmd_pktgen_waitDone(ArkPktGen_t handle)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	if (ark_pmd_pktgen_isGenForever(handle)) {
	PMD_DRV_LOG(ERR, "waitDone will not terminate because genForever=1\n");
	}
	int waitCycle = 10;

	while (!ark_pmd_pktgen_txDone(handle) && (waitCycle > 0)) {
	usleep(1000);
	waitCycle--;
	ARK_DEBUG_TRACE("Waiting for pktgen %d to finish sending...\n",
		inst->ordinal);
	}
	ARK_DEBUG_TRACE("pktgen %d done.\n", inst->ordinal);
}

uint32_t
ark_pmd_pktgen_getPktsSent(ArkPktGen_t handle)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	return inst->regs->pktsSent;
}

void
ark_pmd_pktgen_setPayloadByte(ArkPktGen_t handle, uint32_t b)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	inst->regs->pktPayload = b;
}

void
ark_pmd_pktgen_setPktSpacing(ArkPktGen_t handle, uint32_t x)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	inst->regs->pktSpacing = x;
}

void
ark_pmd_pktgen_setPktSizeMin(ArkPktGen_t handle, uint32_t x)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	inst->regs->pktSizeMin = x;
}

void
ark_pmd_pktgen_setPktSizeMax(ArkPktGen_t handle, uint32_t x)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	inst->regs->pktSizeMax = x;
}

void
ark_pmd_pktgen_setPktSizeIncr(ArkPktGen_t handle, uint32_t x)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	inst->regs->pktSizeIncr = x;
}

void
ark_pmd_pktgen_setNumPkts(ArkPktGen_t handle, uint32_t x)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	inst->regs->numPkts = x;
}

void
ark_pmd_pktgen_setSrcMACAddr(ArkPktGen_t handle, uint64_t macAddr)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	inst->regs->srcMACAddrH = (macAddr >> 32) & 0xffff;
	inst->regs->srcMACAddrL = macAddr & 0xffffffff;
}

void
ark_pmd_pktgen_setDstMACAddr(ArkPktGen_t handle, uint64_t macAddr)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	inst->regs->dstMACAddrH = (macAddr >> 32) & 0xffff;
	inst->regs->dstMACAddrL = macAddr & 0xffffffff;
}

void
ark_pmd_pktgen_setEthType(ArkPktGen_t handle, uint32_t x)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	inst->regs->ethType = x;
}

void
ark_pmd_pktgen_setHdrDW(ArkPktGen_t handle, uint32_t *hdr)
{
	uint32_t i;
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	for (i = 0; i < 7; i++)
		inst->regs->hdrDW[i] = hdr[i];
}

void
ark_pmd_pktgen_setStartOffset(ArkPktGen_t handle, uint32_t x)
{
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	inst->regs->startOffset = x;
}

static struct Options *
OPTIONS(const char *id)
{
	unsigned i;

	for (i = 0; i < sizeof(toptions) / sizeof(struct Options); i++) {
		if (strcmp(id, toptions[i].opt) == 0)
			return &toptions[i];
	}

	PMD_DRV_LOG
		(ERR,
		 "pktgen:  Could not find requested option !!, option = %s\n", id
		 );
	return NULL;
}

static int pmd_setArg(char *arg, char *val);
static int
pmd_setArg(char *arg, char *val)
{
	struct Options *o = OPTIONS(arg);

	if (o) {
	switch (o->t) {
	case OTInt:
	case OTBool:
		o->v.Int = atoi(val);
		break;
	case OTLong:
		o->v.Int = atoll(val);
		break;
	case OTString:
		strncpy(o->v.Str, val, ARK_MAX_STR_LEN);
		break;
	}
	return 1;
	}
	return 0;
}

/******
 * Arg format = "opt0=v,optN=v ..."
 ******/
void
ark_pmd_pktgen_parse(char *args)
{
	char *argv, *v;
	const char toks[] = " =\n\t\v\f\r";

	argv = strtok(args, toks);
	v = strtok(NULL, toks);
	pmd_setArg(argv, v);
	while (argv && v) {
	argv = strtok(NULL, toks);
	v = strtok(NULL, toks);
	if (argv && v)
		pmd_setArg(argv, v);
	}
}

static int32_t parseIPV4string(char const *ipAddress);
static int32_t
parseIPV4string(char const *ipAddress)
{
	unsigned int ip[4];

	if (4 != sscanf(ipAddress, "%u.%u.%u.%u", &ip[0], &ip[1], &ip[2], &ip[3]))
	return 0;
	return ip[3] + ip[2] * 0x100 + ip[1] * 0x10000ul + ip[0] * 0x1000000ul;
}

static void
ark_pmd_pktgen_setPktCtrl(ArkPktGen_t handle, uint32_t genForever,
	uint32_t enSlavedStart, uint32_t varyLength, uint32_t incrPayload,
	uint32_t incrFirstByte, uint32_t insSeqNum, uint32_t insUDPHdr,
	uint32_t insTimeStamp)
{
	uint32_t r;
	struct ArkPktGenInst *inst = (struct ArkPktGenInst *) handle;

	if (!inst->l2_mode)
		insUDPHdr = 0;

	r = (genForever << 24) | (enSlavedStart << 20) | (varyLength << 16) |
	(incrPayload << 12) | (incrFirstByte << 8) |
	(insTimeStamp << 5) | (insSeqNum << 4) | insUDPHdr;

	inst->regs->bytesPerCycle = OPTIONS("bytesPerCycle")->v.Int;
	if (OPTIONS("shaping")->v.Bool)
		r = r | (1 << 28);	/* enable shaping */


	inst->regs->pktCtrl = r;
}

void
ark_pmd_pktgen_setup(ArkPktGen_t handle)
{
	uint32_t hdr[7];
	int32_t dstIp = parseIPV4string(OPTIONS("dstIP")->v.Str);

	if (!OPTIONS("pause")->v.Bool && (!OPTIONS("reset")->v.Bool
		&& (OPTIONS("configure")->v.Bool))) {

	ark_pmd_pktgen_setPayloadByte(handle, OPTIONS("payloadByte")->v.Int);
	ark_pmd_pktgen_setSrcMACAddr(handle, OPTIONS("srcMACAddr")->v.Int);
	ark_pmd_pktgen_setDstMACAddr(handle, OPTIONS("dstMACAddr")->v.Long);
	ark_pmd_pktgen_setEthType(handle, OPTIONS("ethType")->v.Int);

	if (OPTIONS("dg-mode")->v.Bool) {
		hdr[0] = OPTIONS("hdrDW0")->v.Int;
		hdr[1] = OPTIONS("hdrDW1")->v.Int;
		hdr[2] = OPTIONS("hdrDW2")->v.Int;
		hdr[3] = OPTIONS("hdrDW3")->v.Int;
		hdr[4] = OPTIONS("hdrDW4")->v.Int;
		hdr[5] = OPTIONS("hdrDW5")->v.Int;
		hdr[6] = OPTIONS("hdrDW6")->v.Int;
	} else {
		hdr[0] = dstIp;
		hdr[1] = OPTIONS("dstPort")->v.Int;
		hdr[2] = OPTIONS("srcPort")->v.Int;
		hdr[3] = 0;
		hdr[4] = 0;
		hdr[5] = 0;
		hdr[6] = 0;
	}
	ark_pmd_pktgen_setHdrDW(handle, hdr);
	ark_pmd_pktgen_setNumPkts(handle, OPTIONS("numPkts")->v.Int);
	ark_pmd_pktgen_setPktSizeMin(handle, OPTIONS("pktSizeMin")->v.Int);
	ark_pmd_pktgen_setPktSizeMax(handle, OPTIONS("pktSizeMax")->v.Int);
	ark_pmd_pktgen_setPktSizeIncr(handle, OPTIONS("pktSizeIncr")->v.Int);
	ark_pmd_pktgen_setPktSpacing(handle, OPTIONS("pktSpacing")->v.Int);
	ark_pmd_pktgen_setStartOffset(handle, OPTIONS("startOffset")->v.Int);
	ark_pmd_pktgen_setPktCtrl(handle,
		OPTIONS("genForever")->v.Bool,
		OPTIONS("enSlavedStart")->v.Bool,
		OPTIONS("varyLength")->v.Bool,
		OPTIONS("incrPayload")->v.Bool,
		OPTIONS("incrFirstByte")->v.Bool,
		OPTIONS("insSeqNum")->v.Int,
		OPTIONS("insUDPHdr")->v.Bool, OPTIONS("insTimeStamp")->v.Int);
	}

	if (OPTIONS("pause")->v.Bool)
	ark_pmd_pktgen_pause(handle);

	if (OPTIONS("reset")->v.Bool)
	ark_pmd_pktgen_reset(handle);

	if (OPTIONS("run")->v.Bool) {
	ARK_DEBUG_TRACE("Starting packet generator on port %d\n",
		OPTIONS("port")->v.Int);
	ark_pmd_pktgen_run(handle);
	}
}
