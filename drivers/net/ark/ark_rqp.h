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

#ifndef _ARK_RQP_H_
#define _ARK_RQP_H_

#include <stdint.h>

#include <rte_memory.h>

/*
 * RQ Pacing core hardware structure
 */
struct ark_rqpace_t {
	volatile uint32_t ctrl;
	volatile uint32_t statsClear;
	volatile uint32_t cplh_max;
	volatile uint32_t cpld_max;
	volatile uint32_t errCnt;
	volatile uint32_t stallPS;
	volatile uint32_t stallPSMin;
	volatile uint32_t stallPSMax;
	volatile uint32_t reqPS;
	volatile uint32_t reqPSMin;
	volatile uint32_t reqPSMax;
	volatile uint32_t reqDWPS;
	volatile uint32_t reqDWPSMin;
	volatile uint32_t reqDWPSMax;
	volatile uint32_t cplPS;
	volatile uint32_t cplPSMin;
	volatile uint32_t cplPSMax;
	volatile uint32_t cplDWPS;
	volatile uint32_t cplDWPSMin;
	volatile uint32_t cplDWPSMax;
	volatile uint32_t cplh_pending;
	volatile uint32_t cpld_pending;
	volatile uint32_t cplh_pending_max;
	volatile uint32_t cpld_pending_max;
	volatile uint32_t errCountOther;
};

void ark_rqp_dump(struct ark_rqpace_t *rqp);
void ark_rqp_stats_reset(struct ark_rqpace_t *rqp);

#endif
