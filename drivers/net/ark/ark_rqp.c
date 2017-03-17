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

#include "ark_rqp.h"
#include "ark_debug.h"

/* ************************************************************************* */
void
ark_rqp_stats_reset(struct ark_rqpace_t *rqp)
{
	rqp->statsClear = 1;
	/* POR 992 */
	/* rqp->cpld_max = 992; */
	/* POR 64 */
	/* rqp->cplh_max = 64; */

}

/* ************************************************************************* */
void
ark_rqp_dump(struct ark_rqpace_t *rqp)
{
	if (rqp->errCountOther != 0)
	fprintf
		(stderr,
		"ARKP RQP Errors noted: ctrl: %d cplh_hmax %d cpld_max %d"
		 FMT_SU32
		FMT_SU32 "\n",
		 rqp->ctrl, rqp->cplh_max, rqp->cpld_max,
		"Error Count", rqp->errCnt,
		 "Error General", rqp->errCountOther);

	ARK_DEBUG_STATS
		("ARKP RQP Dump: ctrl: %d cplh_hmax %d cpld_max %d" FMT_SU32
		 FMT_SU32 FMT_SU32 FMT_SU32 FMT_SU32 FMT_SU32 FMT_SU32
		 FMT_SU32 FMT_SU32 FMT_SU32 FMT_SU32 FMT_SU32 FMT_SU32
		 FMT_SU32 FMT_SU32 FMT_SU32
		 FMT_SU32 FMT_SU32 FMT_SU32 FMT_SU32 FMT_SU32 "\n",
		 rqp->ctrl, rqp->cplh_max, rqp->cpld_max,
		 "Error Count", rqp->errCnt,
		 "Error General", rqp->errCountOther,
		 "stallPS", rqp->stallPS,
		 "stallPS Min", rqp->stallPSMin,
		 "stallPS Max", rqp->stallPSMax,
		 "reqPS", rqp->reqPS,
		 "reqPS Min", rqp->reqPSMin,
		 "reqPS Max", rqp->reqPSMax,
		 "reqDWPS", rqp->reqDWPS,
		 "reqDWPS Min", rqp->reqDWPSMin,
		 "reqDWPS Max", rqp->reqDWPSMax,
		 "cplPS", rqp->cplPS,
		 "cplPS Min", rqp->cplPSMin,
		 "cplPS Max", rqp->cplPSMax,
		 "cplDWPS", rqp->cplDWPS,
		 "cplDWPS Min", rqp->cplDWPSMin,
		 "cplDWPS Max", rqp->cplDWPSMax,
		 "cplh pending", rqp->cplh_pending,
		 "cpld pending", rqp->cpld_pending,
		 "cplh pending max", rqp->cplh_pending_max,
		 "cpld pending max", rqp->cpld_pending_max);
}
