#ifndef _RTE_ETH_ITT_H_
#define _RTE_ETH_ITT_H_

#include <ittnotify.h>
#include <rte_config.h>

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
	__itt_domain *wasted_iteration_itt_domains[RTE_MAX_QUEUES_PER_PORT];
	/**
	 * ITT task names for each queue.
	 */
	__itt_string_handle *wasted_iteration_itt_handles[RTE_MAX_QUEUES_PER_PORT];
	/**
	 * Flags indicating the queues state. Possible values:
	 * 1 - queue is wasting iterations, 0 - otherwise.
	 */
	uint8_t queue_is_wasting_iterations[RTE_MAX_QUEUES_PER_PORT];
};

/**
 * The pool of *rte_eth_itt_aux_data* structures.
 */
struct rte_eth_itt_aux_data itt_aux_data[RTE_MAX_ETHPORTS];

/**
 * Initialization of rte_eth_itt_aux_data for a given port.
 * This function must be invoked when ethernet device is being configured.
 * Result will be stored in the global array *itt_aux_data*.
 *
 * @param port_id
 *  The port identifier of the Ethernet device.
 * @param port_name
 *  The name of the Ethernet device.
 * @param queue_num
 *  The number of queues on specified port.
 */
static inline void
rte_eth_init_itt(uint8_t port_id, char *port_name, uint8_t queue_num) {
	uint16_t q_id;
	for (q_id = 0; q_id < queue_num; ++q_id) {
		char domain_name[ITT_MAX_NAME_LEN];
		snprintf(domain_name, sizeof(domain_name),
			"RXBurst.WastedIterations.Port_%s.Queue_%d",
			port_name, q_id);
		itt_aux_data[port_id].wasted_iteration_itt_domains[q_id]
			= __itt_domain_create(domain_name);

		char task_name[ITT_MAX_NAME_LEN];
		snprintf(task_name, sizeof(task_name),
			"port id: %d; queue id: %d",
			port_id, q_id);
		itt_aux_data[port_id].wasted_iteration_itt_handles[q_id]
			= __itt_string_handle_create(task_name);

		itt_aux_data[port_id].queue_is_wasting_iterations[q_id] = 0;
	}
}

#endif
