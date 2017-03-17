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

#include "ark_pktchkr.h"
#include "ark_debug.h"

static int setArg(char *arg, char *val);
static int ark_pmd_pktchkr_isGenForever(ArkPktChkr_t handle);

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
	{{"port"}, OTInt, {0} },
	{{"mac-dump"}, OTBool, {0} },
	{{"dg-mode"}, OTBool, {1} },
	{{"run"}, OTBool, {0} },
	{{"stop"}, OTBool, {0} },
	{{"dump"}, OTBool, {0} },
	{{"enResync"}, OTBool, {0} },
	{{"tuserErrVal"}, OTInt, {1} },
	{{"genForever"}, OTBool, {0} },
	{{"enSlavedStart"}, OTBool, {0} },
	{{"varyLength"}, OTBool, {0} },
	{{"incrPayload"}, OTInt, {0} },
	{{"incrFirstByte"}, OTBool, {0} },
	{{"insSeqNum"}, OTBool, {0} },
	{{"insTimeStamp"}, OTBool, {1} },
	{{"insUDPHdr"}, OTBool, {0} },
	{{"numPkts"}, OTLong, .v.Long = 10000000000000L},
	{{"payloadByte"}, OTInt, {0x55} },
	{{"pktSpacing"}, OTInt, {60} },
	{{"pktSizeMin"}, OTInt, {2005} },
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
	{{"dstIP"}, OTString, .v.Str = "169.254.10.240"},
	{{"dstPort"}, OTInt, {65536} },
	{{"srcPort"}, OTInt, {65536} },
};

ArkPktChkr_t
ark_pmd_pktchkr_init(void *addr, int ord, int l2_mode)
{
	struct ArkPktChkrInst *inst =
		rte_malloc("ArkPktChkrInst", sizeof(struct ArkPktChkrInst), 0);
	inst->sregs = (struct ArkPktChkrStatRegs *) addr;
	inst->cregs = (struct ArkPktChkrCtlRegs *) (((uint8_t *) addr) + 0x100);
	inst->ordinal = ord;
	inst->l2_mode = l2_mode;
	return inst;
}

void
ark_pmd_pktchkr_uninit(ArkPktChkr_t handle)
{
	rte_free(handle);
}

void
ark_pmd_pktchkr_run(ArkPktChkr_t handle)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	inst->sregs->pktStartStop = 0;
	inst->sregs->pktStartStop = 0x1;
}

int
ark_pmd_pktchkr_stopped(ArkPktChkr_t handle)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;
	uint32_t r = inst->sregs->pktStartStop;

	return (((r >> 16) & 1) == 1);
}

void
ark_pmd_pktchkr_stop(ArkPktChkr_t handle)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;
	int waitCycle = 10;

	inst->sregs->pktStartStop = 0;
	while (!ark_pmd_pktchkr_stopped(handle) && (waitCycle > 0)) {
	usleep(1000);
	waitCycle--;
	ARK_DEBUG_TRACE("Waiting for pktchk %d to stop...\n", inst->ordinal);
	}
	ARK_DEBUG_TRACE("pktchk %d stopped.\n", inst->ordinal);
}

int
ark_pmd_pktchkr_isRunning(ArkPktChkr_t handle)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;
	uint32_t r = inst->sregs->pktStartStop;

	return ((r & 1) == 1);
}

static void
ark_pmd_pktchkr_setPktCtrl(ArkPktChkr_t handle, uint32_t genForever,
	uint32_t varyLength, uint32_t incrPayload, uint32_t incrFirstByte,
	uint32_t insSeqNum, uint32_t insUDPHdr, uint32_t enResync,
	uint32_t tuserErrVal, uint32_t insTimeStamp)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;
	uint32_t r = (tuserErrVal << 16) | (enResync << 0);

	inst->sregs->pktCtrl = r;
	if (!inst->l2_mode) {
	insUDPHdr = 0;
	}
	r = (genForever << 24) | (varyLength << 16) |
	(incrPayload << 12) | (incrFirstByte << 8) |
	(insTimeStamp << 5) | (insSeqNum << 4) | insUDPHdr;
	inst->cregs->pktCtrl = r;
}

static
	int
ark_pmd_pktchkr_isGenForever(ArkPktChkr_t handle)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;
	uint32_t r = inst->cregs->pktCtrl;

	return (((r >> 24) & 1) == 1);
}

int
ark_pmd_pktchkr_waitDone(ArkPktChkr_t handle)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	if (ark_pmd_pktchkr_isGenForever(handle)) {
	ARK_DEBUG_TRACE
		("Error: waitDone will not terminate because genForever=1\n");
	return -1;
	}
	int waitCycle = 10;

	while (!ark_pmd_pktchkr_stopped(handle) && (waitCycle > 0)) {
	usleep(1000);
	waitCycle--;
	ARK_DEBUG_TRACE
		("Waiting for packet checker %d's internal pktgen to finish sending...\n",
		inst->ordinal);
	ARK_DEBUG_TRACE("pktchk %d's pktgen done.\n", inst->ordinal);
	}
	return 0;
}

int
ark_pmd_pktchkr_getPktsSent(ArkPktChkr_t handle)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	return inst->cregs->pktsSent;
}

void
ark_pmd_pktchkr_setPayloadByte(ArkPktChkr_t handle, uint32_t b)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	inst->cregs->pktPayload = b;
}

void
ark_pmd_pktchkr_setPktSizeMin(ArkPktChkr_t handle, uint32_t x)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	inst->cregs->pktSizeMin = x;
}

void
ark_pmd_pktchkr_setPktSizeMax(ArkPktChkr_t handle, uint32_t x)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	inst->cregs->pktSizeMax = x;
}

void
ark_pmd_pktchkr_setPktSizeIncr(ArkPktChkr_t handle, uint32_t x)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	inst->cregs->pktSizeIncr = x;
}

void
ark_pmd_pktchkr_setNumPkts(ArkPktChkr_t handle, uint32_t x)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	inst->cregs->numPkts = x;
}

void
ark_pmd_pktchkr_setSrcMACAddr(ArkPktChkr_t handle, uint64_t macAddr)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	inst->cregs->srcMACAddrH = (macAddr >> 32) & 0xffff;
	inst->cregs->srcMACAddrL = macAddr & 0xffffffff;
}

void
ark_pmd_pktchkr_setDstMACAddr(ArkPktChkr_t handle, uint64_t macAddr)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	inst->cregs->dstMACAddrH = (macAddr >> 32) & 0xffff;
	inst->cregs->dstMACAddrL = macAddr & 0xffffffff;
}

void
ark_pmd_pktchkr_setEthType(ArkPktChkr_t handle, uint32_t x)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	inst->cregs->ethType = x;
}

void
ark_pmd_pktchkr_setHdrDW(ArkPktChkr_t handle, uint32_t *hdr)
{
	uint32_t i;
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	for (i = 0; i < 7; i++) {
	inst->cregs->hdrDW[i] = hdr[i];
	}
}

void
ark_pmd_pktchkr_dump_stats(ArkPktChkr_t handle)
{
	struct ArkPktChkrInst *inst = (struct ArkPktChkrInst *) handle;

	fprintf(stderr, "pktsRcvd      = (%'u)\n", inst->sregs->pktsRcvd);
	fprintf(stderr, "bytesRcvd     = (%'" PRIu64 ")\n",
	inst->sregs->bytesRcvd);
	fprintf(stderr, "pktsOK        = (%'u)\n", inst->sregs->pktsOK);
	fprintf(stderr, "pktsMismatch  = (%'u)\n", inst->sregs->pktsMismatch);
	fprintf(stderr, "pktsErr       = (%'u)\n", inst->sregs->pktsErr);
	fprintf(stderr, "firstMismatch = (%'u)\n", inst->sregs->firstMismatch);
	fprintf(stderr, "resyncEvents  = (%'u)\n", inst->sregs->resyncEvents);
	fprintf(stderr, "pktsMissing   = (%'u)\n", inst->sregs->pktsMissing);
	fprintf(stderr, "minLatency    = (%'u)\n", inst->sregs->minLatency);
	fprintf(stderr, "maxLatency    = (%'u)\n", inst->sregs->maxLatency);
}

static struct Options *
OPTIONS(const char *id)
{
	unsigned i;

	for (i = 0; i < sizeof(toptions) / sizeof(struct Options); i++) {
	if (strcmp(id, toptions[i].opt) == 0) {
		return &toptions[i];
	}
	}
	PMD_DRV_LOG(ERR,
	"pktgen: Could not find requested option !!, option = %s\n", id);
	return NULL;
}

static int
setArg(char *arg, char *val)
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
ark_pmd_pktchkr_parse(char *args)
{
	char *argv, *v;
	const char toks[] = "= \n\t\v\f\r";

	argv = strtok(args, toks);
	v = strtok(NULL, toks);
	setArg(argv, v);
	while (argv && v) {
	argv = strtok(NULL, toks);
	v = strtok(NULL, toks);
	if (argv && v)
		setArg(argv, v);
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

void
ark_pmd_pktchkr_setup(ArkPktChkr_t handle)
{
	uint32_t hdr[7];
	int32_t dstIp = parseIPV4string(OPTIONS("dstIP")->v.Str);

	if (!OPTIONS("stop")->v.Bool && OPTIONS("configure")->v.Bool) {

	ark_pmd_pktchkr_setPayloadByte(handle, OPTIONS("payloadByte")->v.Int);
	ark_pmd_pktchkr_setSrcMACAddr(handle, OPTIONS("srcMACAddr")->v.Int);
	ark_pmd_pktchkr_setDstMACAddr(handle, OPTIONS("dstMACAddr")->v.Long);

	ark_pmd_pktchkr_setEthType(handle, OPTIONS("ethType")->v.Int);
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
	ark_pmd_pktchkr_setHdrDW(handle, hdr);
	ark_pmd_pktchkr_setNumPkts(handle, OPTIONS("numPkts")->v.Int);
	ark_pmd_pktchkr_setPktSizeMin(handle, OPTIONS("pktSizeMin")->v.Int);
	ark_pmd_pktchkr_setPktSizeMax(handle, OPTIONS("pktSizeMax")->v.Int);
	ark_pmd_pktchkr_setPktSizeIncr(handle, OPTIONS("pktSizeIncr")->v.Int);
	ark_pmd_pktchkr_setPktCtrl(handle,
		OPTIONS("genForever")->v.Bool,
		OPTIONS("varyLength")->v.Bool,
		OPTIONS("incrPayload")->v.Bool,
		OPTIONS("incrFirstByte")->v.Bool,
		OPTIONS("insSeqNum")->v.Int,
		OPTIONS("insUDPHdr")->v.Bool,
		OPTIONS("enResync")->v.Bool,
		OPTIONS("tuserErrVal")->v.Int, OPTIONS("insTimeStamp")->v.Int);
	}

	if (OPTIONS("stop")->v.Bool)
	ark_pmd_pktchkr_stop(handle);

	if (OPTIONS("run")->v.Bool) {
	ARK_DEBUG_TRACE("Starting packet checker on port %d\n",
		OPTIONS("port")->v.Int);
	ark_pmd_pktchkr_run(handle);
	}

}
