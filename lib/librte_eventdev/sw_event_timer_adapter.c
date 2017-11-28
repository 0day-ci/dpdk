/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
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
 *     * Neither the name of Intel Corporation nor the names of its
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

#include "rte_event_timer_adapter.h"
#include "rte_event_timer_adapter_driver.h"

static int
sw_event_timer_adapter_init(struct rte_event_timer_adapter *adapter)
{
	RTE_SET_USED(adapter);

	return 0;
}

static int
sw_event_timer_adapter_uninit(struct rte_event_timer_adapter *adapter)
{
	RTE_SET_USED(adapter);

	return 0;
}

static int
sw_event_timer_adapter_start(const struct rte_event_timer_adapter *adapter)
{
	RTE_SET_USED(adapter);

	return 0;
}

static int
sw_event_timer_adapter_stop(const struct rte_event_timer_adapter *adapter)
{
	RTE_SET_USED(adapter);

	return 0;
}

static void
sw_event_timer_adapter_get_info(const struct rte_event_timer_adapter *adapter,
			struct rte_event_timer_adapter_info *adapter_info)
{
	RTE_SET_USED(adapter);
	RTE_SET_USED(adapter_info);
}

static int
sw_event_timer_arm_burst(const struct rte_event_timer_adapter *adapter,
			 struct rte_event_timer **evtims,
			 uint16_t nb_evtims)
{
	RTE_SET_USED(adapter);
	RTE_SET_USED(evtims);
	RTE_SET_USED(nb_evtims);

	return 0;
}

static int
sw_event_timer_cancel_burst(const struct rte_event_timer_adapter *adapter,
			    struct rte_event_timer **evtims,
			    uint16_t nb_evtims)
{
	RTE_SET_USED(adapter);
	RTE_SET_USED(evtims);
	RTE_SET_USED(nb_evtims);

	return 0;
}

static int
sw_event_timer_arm_tmo_tick_burst(const struct rte_event_timer_adapter *adapter,
				  struct rte_event_timer **tims,
				  uint64_t timeout_tick,
				  uint16_t nb_tims)
{
	RTE_SET_USED(adapter);
	RTE_SET_USED(tims);
	RTE_SET_USED(timeout_tick);
	RTE_SET_USED(nb_tims);

	return 0;
}

const struct rte_event_timer_adapter_ops sw_event_adapter_timer_ops = {
	.init = sw_event_timer_adapter_init,
	.uninit = sw_event_timer_adapter_uninit,
	.start = sw_event_timer_adapter_start,
	.stop = sw_event_timer_adapter_stop,
	.get_info = sw_event_timer_adapter_get_info,
	.arm_burst = sw_event_timer_arm_burst,
	.arm_tmo_tick_burst = sw_event_timer_arm_tmo_tick_burst,
	.cancel_burst = sw_event_timer_cancel_burst,
};
