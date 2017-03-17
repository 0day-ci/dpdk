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

#include <unistd.h>

#include "ark_debug.h"
#include "ark_mpu.h"

uint16_t
ark_api_num_queues(struct ark_mpu_t *mpu)
{
	return mpu->hw.numQueues;
}

uint16_t
ark_api_num_queues_per_port(struct ark_mpu_t *mpu, uint16_t ark_ports)
{
	return mpu->hw.numQueues / ark_ports;
}

int
ark_mpu_verify(struct ark_mpu_t *mpu, uint32_t objSize)
{
	uint32_t version;

	version = mpu->id.vernum & 0x0000FF00;
	if ((mpu->id.idnum != 0x2055504d) || (mpu->hw.objSize != objSize)
	|| version != 0x00003100) {
	fprintf(stderr,
		"   MPU module not found as expected %08x \"%c%c%c%c"
		"%c%c%c%c\"\n", mpu->id.idnum, mpu->id.id[0], mpu->id.id[1],
		mpu->id.id[2], mpu->id.id[3], mpu->id.ver[0], mpu->id.ver[1],
		mpu->id.ver[2], mpu->id.ver[3]);
	fprintf(stderr,
		"   MPU HW numQueues: %u hwDepth %u, objSize: %u, objPerMRR: %u Expected size %u\n",
		mpu->hw.numQueues, mpu->hw.hwDepth, mpu->hw.objSize,
		mpu->hw.objPerMRR, objSize);
	return -1;
	}
	return 0;
}

void
ark_mpu_stop(struct ark_mpu_t *mpu)
{
	mpu->cfg.command = MPU_CMD_Stop;
}

void
ark_mpu_start(struct ark_mpu_t *mpu)
{
	mpu->cfg.command = MPU_CMD_Run;	/* run state */
}

int
ark_mpu_reset(struct ark_mpu_t *mpu)
{

	int cnt = 0;

	mpu->cfg.command = MPU_CMD_Reset;	/* reset */

	while (mpu->cfg.command != MPU_CMD_Idle) {
	if (cnt++ > 1000)
		break;
	usleep(10);
	}
	if (mpu->cfg.command != MPU_CMD_Idle) {
	mpu->cfg.command = MPU_CMD_ForceReset;	/* forced reset */
	usleep(10);
	}
	ark_mpu_reset_stats(mpu);
	return mpu->cfg.command != MPU_CMD_Idle;
}

void
ark_mpu_reset_stats(struct ark_mpu_t *mpu)
{
	mpu->stats.pciRequest = 1;	/* reset stats */
}

int
ark_mpu_configure(struct ark_mpu_t *mpu, phys_addr_t ring, uint32_t ringSize,
	int isTx)
{
	ark_mpu_reset(mpu);

	if (!rte_is_power_of_2(ringSize)) {
	fprintf(stderr, "ARKP Invalid ring size for MPU %d\n", ringSize);
	return -1;
	}

	mpu->cfg.ringBase = ring;
	mpu->cfg.ringSize = ringSize;
	mpu->cfg.ringMask = ringSize - 1;
	mpu->cfg.minHostMove = isTx ? 1 : mpu->hw.objPerMRR;
	mpu->cfg.minHWMove = mpu->hw.objPerMRR;
	mpu->cfg.swProdIndex = 0;
	mpu->cfg.hwConsIndex = 0;
	return 0;
}

void
ark_mpu_dump(struct ark_mpu_t *mpu, const char *code, uint16_t qid)
{
	/* DUMP to see that we have started */
	ARK_DEBUG_TRACE
		("ARKP MPU: %s Q: %3u swProd %u, hwCons: %u\n", code, qid,
		 mpu->cfg.swProdIndex, mpu->cfg.hwConsIndex);
	ARK_DEBUG_TRACE
		("ARKP MPU: %s state: %d count %d, reserved %d data 0x%08x_%08x 0x%08x_%08x\n",
		 code, mpu->debug.state, mpu->debug.count, mpu->debug.reserved,
		 mpu->debug.peek[1], mpu->debug.peek[0], mpu->debug.peek[3],
		 mpu->debug.peek[2]
		 );
	ARK_DEBUG_STATS
		("ARKP MPU: %s Q: %3u" FMT_SU64 FMT_SU64 FMT_SU64 FMT_SU64
		 FMT_SU64 FMT_SU64 FMT_SU64 "\n", code, qid,
		 "PCI Request:", mpu->stats.pciRequest,
		 "QueueEmpty", mpu->stats.qEmpty,
		 "QueueQ1", mpu->stats.qQ1,
		 "QueueQ2", mpu->stats.qQ2,
		 "QueueQ3", mpu->stats.qQ3,
		 "QueueQ4", mpu->stats.qQ4,
		 "QueueFull", mpu->stats.qFull
		 );
}

void
ark_mpu_dump_setup(struct ark_mpu_t *mpu, uint16_t qId)
{
	ARK_DEBUG_TRACE
		("MPU Setup Q: %u"
		 FMT_SPTR "\n", qId,
		 "ringBase", (void *) mpu->cfg.ringBase
		 );

}
