/*-
 *   BSD LICENSE
 *
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

#ifndef _RTE_FLOW_CLASSIFY_H_
#define _RTE_FLOW_CLASSIFY_H_

/**
 * @file
 *
 * RTE Flow Classify Library
 *
 * This library provides flow record information with some measured properties.
 *
 * Application can select variety of flow types based on various flow keys.
 *
 * Library only maintains flow records between rte_flow_classify_stats_get()
 * calls and with a maximum limit.
 *
 * Provided flow record will be linked list rte_flow_classify_stat_xxx
 * structure.
 *
 * Library is responsible from allocating and freeing memory for flow record
 * table. Previous table freed with next rte_flow_classify_stats_get() call and
 * all tables are freed with rte_flow_classify_type_reset() or
 * rte_flow_classify_type_set(x, 0). Memory for table allocated on the fly while
 * creating records.
 *
 * A rte_flow_classify_type_set() with a valid type will register Rx/Tx
 * callbacks and start filling flow record table.
 * With rte_flow_classify_stats_get(), pointer sent to caller and meanwhile
 * library continues collecting records.
 *
 *  Usage:
 *  - application calls rte_flow_classify_type_set() for a device
 *  - library creates Rx/Tx callbacks for packets and start filling flow table
 *    for that type of flow (currently only one flow type supported)
 *  - application calls rte_flow_classify_stats_get() to get pointer to linked
 *    listed flow table. Library assigns this pointer to another value and keeps
 *    collecting flow data. In next rte_flow_classify_stats_get(), library first
 *    free the previous table, and pass current table to the application, keep
 *    collecting data.
 *  - application calls rte_flow_classify_type_reset(), library unregisters the
 *    callbacks and free all flow table data.
 *
 */

#include <rte_ethdev.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_tcp.h>
#include <rte_udp.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Types of classification supported.
 */
enum rte_flow_classify_type {
	RTE_FLOW_CLASSIFY_TYPE_GENERIC = (1 << 0),
	RTE_FLOW_CLASSIFY_TYPE_MAX,
};

#define RTE_FLOW_CLASSIFY_TYPE_MASK = (((RTE_FLOW_CLASSIFY_TYPE_MAX - 1) << 1) - 1)

/**
 * Global configuration struct
 */
struct rte_flow_classify_config {
	uint32_t type; /* bitwise enum rte_flow_classify_type values */
	void *flow_table_prev;
	uint32_t flow_table_prev_item_count;
	void *flow_table_current;
	uint32_t flow_table_current_item_count;
} rte_flow_classify_config[RTE_MAX_ETHPORTS];

#define RTE_FLOW_CLASSIFY_STAT_MAX UINT16_MAX

/**
 * Classification stats data struct
 */
struct rte_flow_classify_stat_generic {
	struct rte_flow_classify_stat_generic *next;
	uint32_t id;
	uint64_t timestamp;

	struct ether_addr src_mac;
	struct ether_addr dst_mac;
	uint32_t src_ipv4;
	uint32_t dst_ipv4;
	uint8_t l3_protocol_id;
	uint16_t src_port;
	uint16_t dst_port;

	uint64_t packet_count;
	uint64_t packet_size; /* bytes */
};

/**
* Get flow types to flow_classify
*
* @param port_id
*   Ethernet device port id to get classification.
* @param type
*   bitmap of enum rte_flow_classify_type values enabled for classification
* @return
*   - (0) if successful.
*   - (-EINVAL) on failure.
*/
int
rte_flow_classify_type_get(uint8_t port_id, uint32_t *type);

/**
* Set flow types to flow_classify
*
* If the type list is zero, no classification done.
*
* @param port_id
*   Ethernet device port_id to set classification.
* @param type
*   bitmap of enum rte_flow_classify_type values to enable classification
* @return
*   - (0) if successful.
*   - (-EINVAL) on failure.
*/
int
rte_flow_classify_type_set(uint8_t port_id, uint32_t type);

/**
* Disable flow classification for device
*
* @param port_id
*   Ethernet device port id to reset classification.
* @return
*   - (0) if successful.
*   - (-EINVAL) on failure.
*/
int
rte_flow_classify_type_reset(uint8_t port_id);

/**
* Get classified results
*
* @param port_id
*  Ethernet device port id to get flow stats
* @param stats
*  void * to linked list flow data
* @return
*   - (0) if successful.
*   - (-EINVAL) on failure.
*/
int
rte_flow_classify_stats_get(uint8_t port_id, void *stats);

/**
* Reset classified results
*
* @param port_id
*  Ethernet device port id to reset flow stats
* @return
*   - (0) if successful.
*   - (-EINVAL) on failure.
*/
int
rte_flow_classify_stats_reset(uint8_t port_id);

#ifdef __cplusplus
}
#endif

#endif /* _RTE_FLOW_CLASSIFY_H_ */
