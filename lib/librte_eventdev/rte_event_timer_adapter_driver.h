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

#ifndef __RTE_EVENT_TIMER_ADAPTER_DRIVER_H__
#define __RTE_EVENT_TIMER_ADAPTER_DRIVER_H__

/**
 * @file
 *
 * Description
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "rte_event_timer_adapter.h"

/*
 * Definitions of functions exported by an event timer adapter implementation
 * through *rte_event_timer_adapter_ops* structure supplied in the
 * *rte_event_timer_adapter* structure associated with an event timer adapter.
 */

typedef int (*rte_event_timer_adapter_init_t)(
		struct rte_event_timer_adapter *adapter);
/**< @internal Event timer adapter implementation setup */
typedef int (*rte_event_timer_adapter_uninit_t)(
		struct rte_event_timer_adapter *adapter);
/**< @internal Event timer adapter implementation teardown */
typedef int (*rte_event_timer_adapter_start_t)(
		const struct rte_event_timer_adapter *adapter);
/**< @internal Start running event timer adapter */
typedef int (*rte_event_timer_adapter_stop_t)(
		const struct rte_event_timer_adapter *adapter);
/**< @internal Stop running event timer adapter */
typedef void (*rte_event_timer_adapter_get_info_t)(
		const struct rte_event_timer_adapter *adapter,
		struct rte_event_timer_adapter_info *adapter_info);
/**< @internal Get contextual information for event timer adapter */
typedef int (*rte_event_timer_arm_burst_t)(
		const struct rte_event_timer_adapter *adapter,
		struct rte_event_timer **tims,
		uint16_t nb_tims);
/**< @internal Enable event timers to enqueue timer events upon expiry */
typedef int (*rte_event_timer_arm_tmo_tick_burst_t)(
		const struct rte_event_timer_adapter *adapter,
		struct rte_event_timer **tims,
		uint64_t timeout_tick,
		uint16_t nb_tims);
/**< @internal Enable event timers with common expiration time */
typedef int (*rte_event_timer_cancel_burst_t)(
		const struct rte_event_timer_adapter *adapter,
		struct rte_event_timer **tims,
		uint16_t nb_tims);
/**< @internal Prevent event timers from enqueuing timer events */

/**
 * @internal Structure containing the functions exported by an event timer
 * adapter implementation.
 */
struct rte_event_timer_adapter_ops {
	rte_event_timer_adapter_init_t		init;  /**< Set up adapter */
	rte_event_timer_adapter_uninit_t	uninit;/**< Tear down adapter */
	rte_event_timer_adapter_start_t		start; /**< Start adapter */
	rte_event_timer_adapter_stop_t		stop;  /**< Stop adapter */
	rte_event_timer_adapter_get_info_t	get_info;
	/**< Get info from driver */
	rte_event_timer_arm_burst_t		arm_burst;
	/**< Arm one or more event timers */
	rte_event_timer_arm_tmo_tick_burst_t	arm_tmo_tick_burst;
	/**< Arm event timers with same expiration time */
	rte_event_timer_cancel_burst_t		cancel_burst;
	/**< Cancel one or more event timers */
};

/**
 * @internal Adapter data; structure to be placed in shared memory to be
 * accessible by various processes in a multi-process configuration.
 */
struct rte_event_timer_adapter_data {
	uint8_t id;
	/**< Event timer adapter ID */
	uint8_t event_dev_id;
	/**< Event device ID */
	uint32_t socket_id;
	/**< Socket ID where memory is allocated */
	uint8_t event_port_id;
	/**< Optional: event port ID used when the inbuilt port is absent */
	const struct rte_memzone *mz;
	/**< Event timer adapter memzone pointer */
	struct rte_event_timer_adapter_conf conf;
	/**< Configuration used to configure the adapter. */
	uint32_t caps;
	/**< Adapter capabilities */
	void *adapter_priv;
	/**< Timer adapter private data*/

	RTE_STD_C11
	uint8_t started : 1;
	/**< Flag to indicate adapter started. */
} __rte_cache_aligned;

/**
 * @internal Data structure associated with each event timer adapter.
 */
struct rte_event_timer_adapter {
	rte_event_timer_arm_burst_t arm_burst;
	/**< Pointer to driver arm_burst function. */
	rte_event_timer_arm_tmo_tick_burst_t arm_tmo_tick_burst;
	/**< Pointer to driver arm_tmo_tick_burst function. */
	rte_event_timer_cancel_burst_t cancel_burst;
	/**< Pointer to driver cancel function. */

	struct rte_event_timer_adapter_data *data;
	/**< Pointer to shared adapter data */
	const struct rte_event_timer_adapter_ops *ops;
	/**< Functions exported by adapter driver */

	RTE_STD_C11
	uint8_t allocated : 1;
	/**< Flag to indicate that this adapter has been allocated */
} __rte_cache_aligned;

#ifdef __cplusplus
}
#endif

#endif /* __RTE_EVENT_TIMER_ADAPTER_DRIVER_H__ */
