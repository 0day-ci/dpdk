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
#ifndef __RTE_EVENT_TIMER_ADAPTER_H__
#define __RTE_EVENT_TIMER_ADAPTER_H__

/**
 * @file
 *
 * RTE Event Timer Adapter
 *
 * TODO: description
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <rte_spinlock.h>
#include <rte_memory.h>

#include "rte_eventdev.h"

#define RTE_EVENT_TIMER_ADAPTER_NUM_MAX 64

/**
 * Timer adapter clock source
 */
enum rte_event_timer_adapter_clk_src {
	RTE_EVENT_TIMER_ADAPTER_CPU_CLK,
	/**< Use CPU clock as the clock source. */
	RTE_EVENT_TIMER_ADAPTER_EXT_CLK0,
	/**< Platform dependent external clock source 0. */
	RTE_EVENT_TIMER_ADAPTER_EXT_CLK1,
	/**< Platform dependent external clock source 1. */
	RTE_EVENT_TIMER_ADAPTER_EXT_CLK2,
	/**< Platform dependent external clock source 2. */
	RTE_EVENT_TIMER_ADAPTER_EXT_CLK3,
	/**< Platform dependent external clock source 3. */
};

#define RTE_EVENT_TIMER_ADAPTER_F_ADJUST_RES	(1ULL << 0)
/**< The event timer adapter implementation may have constraints on the
 * resolution (timer_tick_ns) and maximum timer expiry timeout(max_tmo_ns)
 * based on the given timer adapter or system.  If this flag is set, the
 * implementation adjusts the resolution and maximum timeout to the best
 * possible configuration. On successful timer adapter creation, the
 * application can get the configured resolution and max timeout with
 * ``rte_event_timer_adapter_get_info()``.
 */
#define RTE_EVENT_TIMER_ADAPTER_F_SP_PUT	(1ULL << 1)
/**< ``rte_event_timer_arm_burst()`` API to be used in single producer mode.
 *
 * @see struct rte_event_timer_adapter_conf::flags
 */

/*
 * Timer adapter configuration structure
 */
struct rte_event_timer_adapter_conf {
	uint8_t event_dev_id;
	/**< Event device identifier */
	uint16_t timer_adapter_id;
	/**< Event timer adapter identifier */
	uint32_t socket_id;
	/**< Identifer of socket from which to allocate memory for adapter */
	enum rte_event_timer_adapter_clk_src clk_src;
	/**< Clock source for timer adapter */
	uint64_t timer_tick_ns;
	/**< Timer adapter resolution in ns */
	uint64_t max_tmo_ns;
	/**< Maximum timer timeout(expiry) in ns */
	uint64_t nb_timers;
	/**< Total number of timers per adapter */
	uint64_t flags;
	/**< Timer adapter config flags (RTE_EVENT_TIMER_ADAPTER_F_*) */
};

struct rte_event_timer_adapter;

/*
 * Callback function type for producer port creation.
 */
typedef int (*rte_event_timer_adapter_port_conf_cb_t)(uint16_t id,
						      uint8_t event_dev_id,
						      uint8_t *event_port_id,
						      void *conf_arg);

/*
 * Create an event timer adapter.
 *
 * This function must be invoked first before any other function in the API.
 *
 * @param conf
 *   The event timer adapter configuration structure.
 *
 * @return
 *   A pointer to the new allocated event timer adapter on success.
 *   NULL on error with rte_errno set appropriately.
 *   Possible rte_errno values include:
 *   - ERANGE: timer_tick_ns is not in supported range.
 */
struct rte_event_timer_adapter *rte_event_timer_adapter_create(
			const struct rte_event_timer_adapter_conf *conf);

/*
 * Create a timer adapter with the supplied callback.
 *
 * This function can be used to have a more granular control over the timer
 * adapter creation.  If a built-in port is absent, then the function uses the
 * callback provided to create and get the port id to be used as a producer
 * port.
 *
 * @param conf
 *   The timer adapter configuration structure
 * @param conf_cb
 *   The port config callback function.
 * @param conf_arg
 *   Opaque pointer to the argument for the callback function
 * @param id_ptr[out]
 *   Address of variable to store adapter identifier in
 *
 * @return
 *   A pointer to the new allocated event timer adapter on success.
 *   NULL on error with rte_errno set appropriately.
 *   Possible rte_errno values include:
 *   - ERANGE: timer_tick_ns is not in supported range.
 *   - ENOMEM: unable to allocate sufficient memory for adapter instances
 *   - EINVAL: invalid event device identifier specified in config
 *   - ENOSPC: maximum number of adapters already created
 */
struct rte_event_timer_adapter *rte_event_timer_adapter_create_ext(
			const struct rte_event_timer_adapter_conf *conf,
			rte_event_timer_adapter_port_conf_cb_t conf_cb,
			void *conf_arg);

/*
 * Timer adapter info structure.
 */
struct rte_event_timer_adapter_info {
	uint64_t min_resolution_ns;
	/**< Minimum timer adapter resolution in ns */
	uint64_t max_tmo_ns;
	/**< Maximum timer timeout(expire) in ns */
	struct rte_event_timer_adapter_conf conf;
	/**< Configured timer adapter attributes */
	uint32_t caps;
	/**< Event timer adapter capabilities */
	int16_t event_dev_port_id;
	/**< Event device port ID, if applicable */
	int32_t service_id;
	/**< Service ID, if applicable */
};

/**
 * Retrieve the contextual information of an event timer adapter.
 *
 * @param adapter
 *   A pointer to the event timer adapter structure.
 *
 * @param[out] adapter_info
 *   A pointer to a structure of type *rte_event_timer_adapter_info* to be
 *   filled with the contextual information of the adapter.
 *
 * @return
 *   - 0: Success, driver updates the contextual information of the
 *   timer adapter
 *   - <0: Error code returned by the driver info get function.
 *   - -EINVAL if adapter identifier invalid
 *
 * @see RTE_EVENT_TIMER_ADAPTER_F_ADJUST_RES,
 *   struct rte_event_timer_adapter_info
 *
 */
int rte_event_timer_adapter_get_info(
		const struct rte_event_timer_adapter *adapter,
		struct rte_event_timer_adapter_info *adapter_info);

/**
 * Start a timer adapter.
 *
 * The adapter start step is the last one and consists of setting the timer
 * adapter to start accepting the timers and schedules to event queues.
 *
 * On success, all basic functions exported by the API (timer arm,
 * timer cancel and so on) can be invoked.
 *
 * @param adapter
 *   A pointer to the event timer adapter structure.
 *
 * @return
 *   - 0: Success, adapter started.
 *   - <0: Error code returned by the driver start function.
 *   - -EINVAL if adapter identifier invalid
 */
int rte_event_timer_adapter_start(
		const struct rte_event_timer_adapter *adapter);

/**
 * Stop an event timer adapter.
 *
 * The adapter can be restarted with a call to
 * ``rte_event_timer_adapter_start()``.
 *
 * @param adapter
 *   A pointer to the event timer adapter structure.
 *
 * @return
 *   - 0: Success, adapter stopped.
 *   - <0: Error code returned by the driver stop function.
 *   - -EINVAL if adapter identifier invalid
 */
int rte_event_timer_adapter_stop(const struct rte_event_timer_adapter *adapter);

/*
 * Lookup an event timer adapter using its identifier.
 *
 * If an event timer adapter was created in another process with the same
 * identifier, this function will locate its state and set up access to it
 * so that it can be used in this process.
 *
 * @param adapter_id
 *  The event timer adapter identifier.
 *
 * @return
 *  A pointer to the event timer adapter matching the identifier on success.
 *  NULL on error with rte_errno set appropriately.
 *  Possible rte_errno values include:
 *   - ENOENT - required entry not available to return.
 */
struct rte_event_timer_adapter *rte_event_timer_adapter_lookup(
							uint16_t adapter_id);

/*
 * Free an event timer adapter.
 *
 * Destroy an event timer adapter, freeing all resources.
 *
 * Before invoking this function, the application must wait for all the
 * armed timers to expire or cancel the outstanding armed timers.
 *
 * @param adapter
 *   A pointer to an event timer adapter structure.
 *
 * @return
 *   - 0: Successfully freed the event timer adapter resources.
 *   - <0: Failed to free the event timer adapter resources.
 *   - -EAGAIN:  adapter is busy; timers outstanding
 *   - -EBUSY: stop hasn't been called for this adapter yet
 *   - -EINVAL: adapter id invalid, or adapter invalid
 */
int rte_event_timer_adapter_free(struct rte_event_timer_adapter *adapter);

/**
 * Event timer state.
 */
enum rte_event_timer_state {
	RTE_EVENT_TIMER_NOT_ARMED = 0,
	/**< Event timer is in not armed state.*/
	RTE_EVENT_TIMER_ARMED = 1,
	/**< Event timer successfully armed.*/
	RTE_EVENT_TIMER_ERROR = -1,
	/**< Generic event timer error.*/
	RTE_EVENT_TIMER_ERROR_TOOEARLY = -2,
	/**< Event timer timeout tick is too little to add to the adapter. */
	RTE_EVENT_TIMER_ERROR_TOOLATE = -3,
	/**< Event timer timeout tick is greater than the maximum timeout.*/
};

/**
 * The generic *rte_event_timer* structure to hold the event timer attributes
 * for arm and cancel operations.
 */
RTE_STD_C11
struct rte_event_timer {
	struct rte_event ev;
	/**<
	 * Expiry event attributes.  On successful event timer timeout,
	 * the following attributes will be used to inject the expiry event to
	 * the eventdev:
	 *  - event_queue_id: Targeted event queue id for expiry events.
	 *  - event_priority: Event priority of the event expiry event in the
	 *  event queue relative to other events.
	 *  - sched_type: Scheduling type of the expiry event.
	 *  - flow_id: Flow id of the expiry event.
	 *  - op: RTE_EVENT_OP_NEW
	 *  - event_type: RTE_EVENT_TYPE_TIMER
	 */
	enum rte_event_timer_state state;
	/**< State of the event timer. */
	uint64_t timeout_ticks;
	/**< Expiry timer ticks expressed in number of *timer_ticks_ns* from
	 * now.
	 * @see struct rte_event_timer_adapter_info::adapter_conf::timer_tick_ns
	 */
	uint64_t impl_opaque[2];
	/**< Implementation-specific opaque data.
	 * An event timer adapter implementation use this field to hold
	 * implementation specific values to share between the arm and cancel
	 * operations.  The application should not modify this field.
	 */
	uint8_t user_meta[];
	/**< Memory to store user specific metadata.
	 * The event timer adapter implementation should not modify this area.
	 */
} __rte_cache_aligned;

/**
 * Arm a burst of event timers with separate expiration timeout tick for each
 * event timer.
 *
 * Before calling this function, the application allocates
 * ``struct rte_event_timer`` objects from mempool or huge page backed
 * application buffers of desired size. On successful allocation,
 * application updates the `struct rte_event_timer`` attributes such as
 * expiry event attributes, timeout ticks from now.
 * This function submits the event timer arm requests to the event timer adapter
 * and on expiry, the events will be injected to designated event queue.
 *
 * @param adapter
 *   A pointer to an event timer adapter structure.
 * @param event_timers
 *   Pointer to an array of objects of type *rte_event_timer* structure.
 * @param nb_event_timers
 *   Number of event timers in the supplied array.
 *
 * @return
 *   The number of successfully armed event timers. The return value can be less
 *   than the value of the *nb_timers* parameter. If the return value is less
 *   than *nb_events*, the remaining event timers at the end of *tim*
 *   are not consumed, and the caller has to take care of them, and rte_errno
 *   is set accordingly. Possible errno values include:
 *   - -EINVAL  Invalid timer adapter identifier, expiry event queue ID is
 *   invalid, or an expiry event's sched type doesn't match the capabilities of
 *   the destination event queue.
 *   - -EAGAIN Specified timer adapter is not running
 */
int rte_event_timer_arm_burst(const struct rte_event_timer_adapter *adapter,
			      struct rte_event_timer **event_timers,
			      uint16_t nb_event_timers);

/**
 * Arm a burst of event timers with same expiration timeout tick.
 *
 * Provides the same functionality as ``rte_event_timer_arm_burst()``, except
 * that application can use this API when all the event timers have the
 * same timeout expiration tick. This specialized function can provide the
 * additional hint to the adapter implementation and optimize if possible.
 *
 * @param adapter
 *   A pointer to an event timer adapter structure.
 * @param event_timers
 *   Points to an array of objects of type *rte_event_timer* structure.
 * @param timeout_ticks
 *   The number of ticks in which the timers should expire.
 * @param nb_event_timers
 *   Number of event timers in the supplied array.
 *
 * @return
 *   The number of successfully armed event timers. The return value can be less
 *   than the value of the *nb_timers* parameter. If the return value is less
 *   than *nb_events*, the remaining event timers at the end of *tim*
 *   are not consumed, and the caller has to take care of them, and rte_errno
 *   is set accordingly. Possible errno values include:
 *   - -EINVAL  Invalid timer adapter identifier, expiry event queue ID is
 *   invalid, or an expiry event's sched type doesn't match the capabilities of
 *   the destination event queue.
 *   - -EAGAIN Specified event timer adapter is not running
 */
int rte_event_timer_arm_tmo_tick_burst(
			const struct rte_event_timer_adapter *adapter,
			struct rte_event_timer **event_timers,
			const uint64_t timeout_ticks,
			const uint16_t nb_event_timers);

/**
 * Cancel a burst of event timer from being scheduled to the event device.
 *
 * @param adapter
 *   A pointer to an event timer adapter structure.
 * @param event_timers
 *   Points to an array of objects of type *rte_event_timer* structure
 * @param nb_event_timers
 *   Number of event timer instances in the supplied array.
 *
 * @return
 *   The number of successfully canceled event timers. The return value can be
 *   less than the value of the *nb_timers* parameter. If the return value is
 *   less than *nb_events*, the remaining event timers at the end of *tim*
 *   are not consumed, and the caller has to take care of them, and rte_errno
 *   is set accordingly. Possible errno values include:
 *   - -EINVAL  Invalid timer adapter identifier
 *   - -EAGAIN  Specified timer adapter is not running
 */
int rte_event_timer_cancel_burst(const struct rte_event_timer_adapter *adapter,
				 struct rte_event_timer **event_timers,
				 uint16_t nb_event_timers);

#endif /* __RTE_EVENT_TIMER_ADAPTER_H__ */
