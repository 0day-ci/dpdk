/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2017 Intel Corporation. All rights reserved.
 *   All rights reserved.
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

#include "rte_ethdev_profile.h"

/**
 * This conditional block is responsible for RX queues profiling using the
 * Instrumentation and Tracing Technology (ITT) API, employed by the
 * Intel VTune TM Amplifier.
 */
#ifdef RTE_ETHDEV_PROFILE_ITT_WASTED_RX_ITERATIONS

#include <rte_config.h>
#include <ittnotify.h>

#define ITT_MAX_NAME_LEN (100)

/**
 * Auxiliary ITT structure belonging to port and using to:
 *   -  track queue state to determine whether it is wasting loop iterations
 *   -  begin or end ITT task using task domain and name
 */
struct rte_eth_itt_aux_data {
	/**
	 * ITT domains for each queue.
	 */
	__itt_domain *wasted_iter_domains[RTE_MAX_QUEUES_PER_PORT];
	/**
	 * ITT task names for each queue.
	 */
	__itt_string_handle *wasted_iter_handles[RTE_MAX_QUEUES_PER_PORT];
	/**
	 * Flags indicating the queues state. Possible values:
	 * 1 - queue is wasting iterations, 0 - otherwise.
	 */
	uint8_t queue_is_wasting_iters[RTE_MAX_QUEUES_PER_PORT];
};

/**
 * The pool of *rte_eth_itt_aux_data* structures.
 */
struct rte_eth_itt_aux_data itt_aux_data[RTE_MAX_ETHPORTS];


/**
 * This callback function manages ITT tasks collection on given port and queue.
 * It must be registered with rte_eth_add_rx_callback() to be called from
 * rte_eth_rx_burst(). To find more comments see rte_rx_callback_fn function
 * type declaration.
 */
static uint16_t
collect_itt_rx_burst_cb(uint8_t port_id, uint16_t queue_id,
	__rte_unused struct rte_mbuf *pkts[], uint16_t nb_pkts,
	__rte_unused uint16_t max_pkts, __rte_unused void *user_param)
{
	if (unlikely(nb_pkts == 0)) {
		if (!itt_aux_data[port_id].queue_is_wasting_iters[queue_id]) {
			__itt_task_begin(
				itt_aux_data[port_id].wasted_iter_domains[queue_id],
				__itt_null, __itt_null,
				itt_aux_data[port_id].wasted_iter_handles[queue_id]);
			itt_aux_data[port_id].queue_is_wasting_iters[queue_id] = 1;
		}
	} else {
		if (unlikely(itt_aux_data[port_id].queue_is_wasting_iters[queue_id])) {
			__itt_task_end(
				itt_aux_data[port_id].wasted_iter_domains[queue_id]);
			itt_aux_data[port_id].queue_is_wasting_iters[queue_id] = 0;
		}
	}
	return nb_pkts;
}

/**
 * Initialization of rte_eth_itt_aux_data for a given port.
 * This function must be invoked when ethernet device is being configured.
 * Result will be stored in the global array *itt_aux_data*.
 *
 * @param port_id
 *  The port identifier of the Ethernet device.
 * @param port_name
 *  The name of the Ethernet device.
 * @param rx_queue_num
 *  The number of RX queues on specified port.
 */
static inline void
rte_eth_init_itt(uint8_t port_id, char *port_name, uint8_t rx_queue_num)
{
	uint16_t q_id;
	for (q_id = 0; q_id < rx_queue_num; ++q_id) {
		char domain_name[ITT_MAX_NAME_LEN];
		snprintf(domain_name, sizeof(domain_name),
			"RXBurst.WastedIterations.Port_%s.Queue_%d",
			port_name, q_id);
		itt_aux_data[port_id].wasted_iter_domains[q_id]
			= __itt_domain_create(domain_name);

		char task_name[ITT_MAX_NAME_LEN];
		snprintf(task_name, sizeof(task_name),
			"port id: %d; queue id: %d",
			port_id, q_id);
		itt_aux_data[port_id].wasted_iter_handles[q_id]
			= __itt_string_handle_create(task_name);

		itt_aux_data[port_id].queue_is_wasting_iters[q_id] = 0;

		rte_eth_add_rx_callback(
			port_id, q_id, collect_itt_rx_burst_cb, NULL);
	}
}
#endif /* RTE_ETHDEV_PROFILE_ITT_WASTED_RX_ITERATIONS */

void
rte_eth_rx_profile_init(__rte_unused uint8_t port_id,
	__rte_unused struct rte_eth_dev *dev)
{
#ifdef RTE_ETHDEV_PROFILE_ITT_WASTED_RX_ITERATIONS
	rte_eth_init_itt(port_id, dev->data->name, dev->data->nb_rx_queues);
#endif
}
