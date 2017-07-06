/*
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
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

#ifndef _RTE_EVENT_ETH_RX_ADAPTER_
#define _RTE_EVENT_ETH_RX_ADAPTER_

/**
 * @file
 *
 * RTE Event Ethernet Rx Adapter
 *
 * An eventdev-based packet processing application enqueues/dequeues mbufs
 * to/from the event device. The ethernet Rx event adapter's role is to transfer
 * mbufs from the ethernet receive queues managed by DPDK to an event device.
 * The application uses the adapter APIs to configure the packet flow between
 * the ethernet devices and event devices. The adapter is designed to work with
 * the EAL service cores. The adapter's work can be parallelized by dividing the
 * NIC Rx queues among multiple adapter services that run in parallel.
 *
 * Before using the adapter, the application needs to enumerate and configure
 * the ethernet devices that it wishes to use. This is typically done using the
 * following DPDK ethdev functions:
 *  - rte_eth_dev_configure()
 *  - rte_eth_tx_queue_setup()
 *  - rte_eth_rx_queue_setup()
 *  - rte_eth_dev_start()
 *
 * The application also configures an event device and creates event ports
 * to interface with the event device. In addition to the event ports used by
 * its packet processing functions, the application creates an event port
 * to be used by this adapter.
 *
 * The ethernet Rx event adapter's functions are:
 *  - rte_event_eth_rx_adapter_create()
 *  - rte_event_eth_rx_adapter_free()
 *  - rte_event_eth_rx_adapter_queue_add()
 *  - rte_event_eth_rx_adapter_queue_del()
 *  - rte_event_eth_rx_adapter_stats_get()
 *  - rte_event_eth_rx_adapter_stats_reset()
 *
 * The applicaton creates an event to ethernet adapter using
 * rte_event_eth_rx_adapter_create(). The event device and event port
 * identifiers are passed to this function.
 *
 * The adapter needs to know which ethernet rx queues to poll for mbufs as well
 * as event device parameters such as the event queue identifier, event
 * priority and scheduling type that the adapter should use when constructing
 * events. The rte_event_eth_rx_adapter_queue_add() function is provided for
 * this purpose.
 *
 * At the time of adding an ethernet device receive queue, the application can
 * also specify a static event flow id and set the
 * RTE_ETH_RX_EVENT_ADAPTER_QUEUE_FLOW_ID_VALID bit of the rx_queue_flags
 * member of the rte_event_eth_rx_adapter_queue_conf structure. If the
 * RTE_ETH_RX_EVENT_ADAPTER_QUEUE_FLOW_ID_VALID isn't set, the flow id is
 * assigned the value of the RSS hash. The adapter generates the RSS hash if it
 * hasn't been already computed by the NIC, based on source and destination
 * IPv4/6 addresses, using the rte_softrss_be() routine included in the DPDK.
 *
 * The servicing weight parameter in the rte_event_eth_rx_adapter_queue_conf
 * intended to provide application control of the polling frequency of ethernet
 * device receive queues, for example, the application may want to poll higher
 * priority queues with a higher frequency but at the same time not starve
 * lower priority queues completely. If this parameter is zero and the receive
 * interrupt is enabled when configuring the device, the receive queue is
 * interrupt driven; else, the queue is assigned a servicing weight of one.
 *
 * Note: Interrupt driven receive queues are currentely unimplemented.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <rte_service.h>

#include "rte_eventdev.h"


#define RTE_MAX_EVENT_ETH_RX_ADAPTER_INSTANCE 32

/* struct rte_event_eth_rx_adapter_queue_conf flags definitions */
#define RTE_EVENT_ETH_RX_ADAPTER_QUEUE_FLOW_ID_VALID	0x1
/**< This flag indicates the flow identifier is valid
 * @see rte_event_eth_rx_adapter_queue_conf
 */

struct rte_event_eth_rx_adapter_conf {
	char service_name[RTE_SERVICE_NAME_MAX];
	/**< Service name of the adapter instance, the rx adapter
	 * launches a service core function with this name
	 */
	uint8_t eventdev_id;
	/**< Event device identifier */
	uint8_t rx_event_port_id;
	/**< Event port identifier, the adapter enqueues mbuf events to this
	 * port
	 */
	uint32_t max_nb_rx;
	/**< The adapter can return early if it has processed at least
	 * max_nb_rx mbufs. This isn't treated as a requirement; batching may
	 * cause the adapter to process more than max_nb_rx mbufs
	 */
	int socket_id;
	/**< Identifier of the NUMA socket on which the service
	 * functions are invoked
	 */
};

/** Rx queue configuration structure */
struct rte_event_eth_rx_adapter_queue_conf {
	uint32_t rx_queue_flags;
	 /**< Flags for handling received packets
	  * @see RTE_EVENT_ETH_RX_ADAPTER_QUEUE_FLOW_ID_VALID
	  */
	uint16_t servicing_weight;
	/**< Relative polling frequency of ethernet receive queue, if this
	 * is set to zero, the Rx queue is interrupt driven (unless rx queue
	 * interrupts are not enabled for the ethernet device)
	 */
	struct rte_event ev;
	/**<
	 *  The values from the following event fields will be used when
	 *  enqueuing mbuf events:
	 *   - event_queue_id: Targeted event queue ID for received packets.
	 *   - event_priority: Event priority of packets from this Rx queue in
	 *                     the event queue relative to other events.
	 *   - sched_type: Scheduling type for packets from this Rx queue.
	 *   - flow_id: If the RTE_ETH_RX_EVENT_ADAPTER_QUEUE_FLOW_ID_VALID bit
	 *		is set in rx_queue_flags, this flow_id is used for all
	 *		packets received from this queue. Otherwise the flow ID
	 *		is set to the RSS hash of the src and dst IPv4/6
	 *		address.
	 *
	 * The event adapter sets ev.event_type to RTE_EVENT_TYPE_ETHDEV in the
	 * enqueued event
	 */
};

struct rte_event_eth_rx_adapter_stats {
	uint64_t rx_poll_count;
	/**< Receive queue poll count */
	uint64_t rx_packets;
	/**< Received packet count */
	uint64_t rx_enq_count;
	/**< Eventdev enqueue count */
	uint64_t rx_enq_retry;
	/**< Eventdev enqueue retry count */
	uint64_t rx_enq_start_ts;
	/**< Rx enqueue start timestamp */
	uint64_t rx_enq_block_cycles;
	/**< Cycles for which the service is blocked by the event device,
	 * i.e, the service fails to enqueue to the event device.
	 */
	uint64_t rx_enq_end_ts;
	/**< Latest timestamp at which the service is unblocked
	 * by the event device. The start, end timestamps and
	 * block cycles can be used to compute the percentage of
	 * cycles the service is blocked by the event device.
	 */
};

/**
 * Initializes the event adapter for ethernet
 * receive. Applications are required to call this function
 * before calling any eth receive event adapter functions
 *
 * @return
 *   - 0: Success
 *   - <0: Error code on failure
 */
int rte_event_eth_rx_adapter_init(void);


/**
 * Create a new ethernet Rx event adapter with the specified identifier.
 * Internally this API registers a service function using the
 * rte_service_register() API.
 *
 * @param adapter_id
 *   Event adapter identifier.
 * @param conf
 *   Event adapter config parameters.
 * @return
 *   - 0: Success
 *   - <0: Error code on failure
 */
int rte_event_eth_rx_adapter_create(uint8_t id,
				    const struct rte_event_eth_rx_adapter_conf *conf);

/**
 * Free an event adapter
 *
 * @param id
 *   Adapter identifier.
 * @return
 *   - 0: Success
 *   - <0: Error code on failure
 */
int rte_event_eth_rx_adapter_free(uint8_t id);

/**
 * Add receive queue to an event adapter. After a queue has been
 * added to the event adapter, the result of the application calling
 * rte_eth_rx_burst(eth_dev_id, rx_queue_id, ..) is undefined.
 *
 * @param id
 *   Adapter identifier.
 * @param eth_dev_id
 *  Port identifier of Ethernet device.
 * @param rx_queue_id
 *  Ethernet device receive queue index.
 *  If rx_queue_id is -1, then all Rx queues configured for
 *  the device are added.
 * @param conf
 *  Additonal configuration structure.
 * @return
 *  - 0: Success, Receive queue added correctly.
 *  - <0: Error code on failure.
 */
int rte_event_eth_rx_adapter_queue_add(uint8_t id,
				       uint8_t eth_dev_id,
				       int32_t rx_queue_id,
				       const struct rte_event_eth_rx_adapter_queue_conf *conf);

/**
 * Delete receive queue from an event adapter.
 *
 * @param id
 *   Adapter identifier.
 * @param eth_dev_id
 *  Port identifier of Ethernet device.
 * @param rx_queue_id
 *  Ethernet device receive queue index.
 *  If rx_queue_id is -1, then all Rx queues configured for
 *  the device are deleted.
 * @return
 *  - 0: Success, Receive queue deleted correctly.
 *  - <0: Error code on failure.
 */
int rte_event_eth_rx_adapter_queue_del(uint8_t id, uint8_t eth_dev_id,
				       int32_t rx_queue_id);

/**
 * Retrieve statistics for an adapter
 *
 * @param id
 *   Adapter identifier.
 * @param stats
 *  A pointer to structure used to retrieve statistics for an adapter.
 * @return
 *  - 0: Success, retrieved successfully.
 *  - <0: Error code on failure.
 */
int rte_event_eth_rx_adapter_stats_get(uint8_t id,
				       struct rte_event_eth_rx_adapter_stats *stats);

/**
 * Reset statistics for an adapter
 *
 * @param id
 *   Adapter identifier.
 * @return
 *  - 0: Success, statistics reset successfully.
 *  - <0: Error code on failure.
 */
int rte_event_eth_rx_adapter_stats_reset(uint8_t id);

#ifdef __cplusplus
}
#endif
#endif	/* _RTE_EVENT_ETH_RX_ADAPTER_ */
