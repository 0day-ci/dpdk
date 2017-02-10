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

#ifndef __INCLUDE_RTE_SCHEDDEV_DRIVER_H__
#define __INCLUDE_RTE_SCHEDDEV_DRIVER_H__

/**
 * @file
 * RTE Generic Hierarchical Scheduler API (Driver Side)
 *
 * This file provides implementation helpers for internal use by PMDs, they
 * are not intended to be exposed to applications and are not subject to ABI
 * versioning.
 */

#include <stdint.h>

#include <rte_errno.h>
#include "rte_ethdev.h"
#include "rte_scheddev.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*rte_scheddev_capabilities_get_t)(struct rte_eth_dev *dev,
	struct rte_scheddev_capabilities *cap,
	struct rte_scheddev_error *error);
/**< @internal Scheduler capabilities get */

typedef int (*rte_scheddev_node_capabilities_get_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	struct rte_scheddev_node_capabilities *cap,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node capabilities get */

typedef int (*rte_scheddev_wred_profile_add_t)(struct rte_eth_dev *dev,
	uint32_t wred_profile_id,
	struct rte_scheddev_wred_params *profile,
	struct rte_scheddev_error *error);
/**< @internal Scheduler WRED profile add */

typedef int (*rte_scheddev_wred_profile_delete_t)(struct rte_eth_dev *dev,
	uint32_t wred_profile_id,
	struct rte_scheddev_error *error);
/**< @internal Scheduler WRED profile delete */

typedef int (*rte_scheddev_shared_wred_context_add_update_t)(
	struct rte_eth_dev *dev,
	uint32_t shared_wred_context_id,
	uint32_t wred_profile_id,
	struct rte_scheddev_error *error);
/**< @internal Scheduler shared WRED context add */

typedef int (*rte_scheddev_shared_wred_context_delete_t)(
	struct rte_eth_dev *dev,
	uint32_t shared_wred_context_id,
	struct rte_scheddev_error *error);
/**< @internal Scheduler shared WRED context delete */

typedef int (*rte_scheddev_shaper_profile_add_t)(struct rte_eth_dev *dev,
	uint32_t shaper_profile_id,
	struct rte_scheddev_shaper_params *profile,
	struct rte_scheddev_error *error);
/**< @internal Scheduler shaper profile add */

typedef int (*rte_scheddev_shaper_profile_delete_t)(struct rte_eth_dev *dev,
	uint32_t shaper_profile_id,
	struct rte_scheddev_error *error);
/**< @internal Scheduler shaper profile delete */

typedef int (*rte_scheddev_shared_shaper_add_update_t)(struct rte_eth_dev *dev,
	uint32_t shared_shaper_id,
	uint32_t shaper_profile_id,
	struct rte_scheddev_error *error);
/**< @internal Scheduler shared shaper add/update */

typedef int (*rte_scheddev_shared_shaper_delete_t)(struct rte_eth_dev *dev,
	uint32_t shared_shaper_id,
	struct rte_scheddev_error *error);
/**< @internal Scheduler shared shaper delete */

typedef int (*rte_scheddev_node_add_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	uint32_t parent_node_id,
	uint32_t priority,
	uint32_t weight,
	struct rte_scheddev_node_params *params,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node add */

typedef int (*rte_scheddev_node_delete_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node delete */

typedef int (*rte_scheddev_node_suspend_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node suspend */

typedef int (*rte_scheddev_node_resume_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node resume */

typedef int (*rte_scheddev_hierarchy_set_t)(struct rte_eth_dev *dev,
	int clear_on_fail,
	struct rte_scheddev_error *error);
/**< @internal Scheduler hierarchy set */

typedef int (*rte_scheddev_node_parent_update_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	uint32_t parent_node_id,
	uint32_t priority,
	uint32_t weight,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node parent update */

typedef int (*rte_scheddev_node_shaper_update_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	uint32_t shaper_profile_id,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node shaper update */

typedef int (*rte_scheddev_node_shared_shaper_update_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	uint32_t shared_shaper_id,
	int32_t add,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node shaper update */

typedef int (*rte_scheddev_node_scheduling_mode_update_t)(
	struct rte_eth_dev *dev,
	uint32_t node_id,
	int *scheduling_mode_per_priority,
	uint32_t n_priorities,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node scheduling mode update */

typedef int (*rte_scheddev_node_cman_update_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	enum rte_scheddev_cman_mode cman,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node congestion management mode update */

typedef int (*rte_scheddev_node_wred_context_update_t)(
	struct rte_eth_dev *dev,
	uint32_t node_id,
	uint32_t wred_profile_id,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node WRED context update */

typedef int (*rte_scheddev_node_shared_wred_context_update_t)(
	struct rte_eth_dev *dev,
	uint32_t node_id,
	uint32_t shared_wred_context_id,
	int add,
	struct rte_scheddev_error *error);
/**< @internal Scheduler node WRED context update */

typedef int (*rte_scheddev_mark_vlan_dei_t)(struct rte_eth_dev *dev,
	int mark_green,
	int mark_yellow,
	int mark_red,
	struct rte_scheddev_error *error);
/**< @internal Scheduler packet marking - VLAN DEI */

typedef int (*rte_scheddev_mark_ip_ecn_t)(struct rte_eth_dev *dev,
	int mark_green,
	int mark_yellow,
	int mark_red,
	struct rte_scheddev_error *error);
/**< @internal Scheduler packet marking - IPv4/IPv6 ECN */

typedef int (*rte_scheddev_mark_ip_dscp_t)(struct rte_eth_dev *dev,
	int mark_green,
	int mark_yellow,
	int mark_red,
	struct rte_scheddev_error *error);
/**< @internal Scheduler packet marking - IPv4/IPv6 DSCP */

typedef int (*rte_scheddev_stats_get_enabled_t)(struct rte_eth_dev *dev,
	uint64_t *nonleaf_node_capability_stats_mask,
	uint64_t *nonleaf_node_enabled_stats_mask,
	uint64_t *leaf_node_capability_stats_mask,
	uint64_t *leaf_node_enabled_stats_mask,
	struct rte_scheddev_error *error);
/**< @internal Scheduler get set of stats counters enabled for all nodes */

typedef int (*rte_scheddev_stats_enable_t)(struct rte_eth_dev *dev,
	uint64_t nonleaf_node_enabled_stats_mask,
	uint64_t leaf_node_enabled_stats_mask,
	struct rte_scheddev_error *error);
/**< @internal Scheduler enable selected stats counters for all nodes */

typedef int (*rte_scheddev_node_stats_get_enabled_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	uint64_t *capability_stats_mask,
	uint64_t *enabled_stats_mask,
	struct rte_scheddev_error *error);
/**< @internal Scheduler get set of stats counters enabled for specific node */

typedef int (*rte_scheddev_node_stats_enable_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	uint64_t enabled_stats_mask,
	struct rte_scheddev_error *error);
/**< @internal Scheduler enable selected stats counters for specific node */

typedef int (*rte_scheddev_node_stats_read_t)(struct rte_eth_dev *dev,
	uint32_t node_id,
	struct rte_scheddev_node_stats *stats,
	int clear,
	struct rte_scheddev_error *error);
/**< @internal Scheduler read stats counters for specific node */

struct rte_scheddev_ops {
	/** Scheduler capabilities_get */
	rte_scheddev_capabilities_get_t capabilities_get;
	/** Scheduler node capabilities get */
	rte_scheddev_node_capabilities_get_t node_capabilities_get;

	/** Scheduler WRED profile add */
	rte_scheddev_wred_profile_add_t wred_profile_add;
	/** Scheduler WRED profile delete */
	rte_scheddev_wred_profile_delete_t wred_profile_delete;
	/** Scheduler shared WRED context add/update */
	rte_scheddev_shared_wred_context_add_update_t
		shared_wred_context_add_update;
	/** Scheduler shared WRED context delete */
	rte_scheddev_shared_wred_context_delete_t
		shared_wred_context_delete;
	/** Scheduler shaper profile add */
	rte_scheddev_shaper_profile_add_t shaper_profile_add;
	/** Scheduler shaper profile delete */
	rte_scheddev_shaper_profile_delete_t shaper_profile_delete;
	/** Scheduler shared shaper add/update */
	rte_scheddev_shared_shaper_add_update_t shared_shaper_add_update;
	/** Scheduler shared shaper delete */
	rte_scheddev_shared_shaper_delete_t shared_shaper_delete;

	/** Scheduler node add */
	rte_scheddev_node_add_t node_add;
	/** Scheduler node delete */
	rte_scheddev_node_delete_t node_delete;
	/** Scheduler node suspend */
	rte_scheddev_node_suspend_t node_suspend;
	/** Scheduler node resume */
	rte_scheddev_node_resume_t node_resume;
	/** Scheduler hierarchy set */
	rte_scheddev_hierarchy_set_t hierarchy_set;

	/** Scheduler node parent update */
	rte_scheddev_node_parent_update_t node_parent_update;
	/** Scheduler node shaper update */
	rte_scheddev_node_shaper_update_t node_shaper_update;
	/** Scheduler node shared shaper update */
	rte_scheddev_node_shared_shaper_update_t node_shared_shaper_update;
	/** Scheduler node scheduling mode update */
	rte_scheddev_node_scheduling_mode_update_t node_scheduling_mode_update;
	/** Scheduler node congestion management mode update */
	rte_scheddev_node_cman_update_t node_cman_update;
	/** Scheduler node WRED context update */
	rte_scheddev_node_wred_context_update_t node_wred_context_update;
	/** Scheduler node shared WRED context update */
	rte_scheddev_node_shared_wred_context_update_t
		node_shared_wred_context_update;

	/** Scheduler packet marking - VLAN DEI */
	rte_scheddev_mark_vlan_dei_t mark_vlan_dei;
	/** Scheduler packet marking - IPv4/IPv6 ECN */
	rte_scheddev_mark_ip_ecn_t mark_ip_ecn;
	/** Scheduler packet marking - IPv4/IPv6 DSCP */
	rte_scheddev_mark_ip_dscp_t mark_ip_dscp;

	/** Scheduler get statistics counter type enabled for all nodes */
	rte_scheddev_stats_get_enabled_t stats_get_enabled;
	/** Scheduler enable selected statistics counters for all nodes */
	rte_scheddev_stats_enable_t stats_enable;
	/** Scheduler get statistics counter type enabled for current node */
	rte_scheddev_node_stats_get_enabled_t node_stats_get_enabled;
	/** Scheduler enable selected statistics counters for current node */
	rte_scheddev_node_stats_enable_t node_stats_enable;
	/** Scheduler read statistics counters for current node */
	rte_scheddev_node_stats_read_t node_stats_read;
};

/**
 * Initialize generic error structure.
 *
 * This function also sets rte_errno to a given value.
 *
 * @param error
 *   Pointer to error structure (may be NULL).
 * @param code
 *   Related error code (rte_errno).
 * @param type
 *   Cause field and error type.
 * @param cause
 *   Object responsible for the error.
 * @param message
 *   Human-readable error message.
 *
 * @return
 *   Error code.
 */
static inline int
rte_scheddev_error_set(struct rte_scheddev_error *error,
		   int code,
		   enum rte_scheddev_error_type type,
		   const void *cause,
		   const char *message)
{
	if (error) {
		*error = (struct rte_scheddev_error){
			.type = type,
			.cause = cause,
			.message = message,
		};
	}
	rte_errno = code;
	return code;
}

/**
 * Get generic hierarchical scheduler operations structure from a port
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param error
 *   Error details
 *
 * @return
 *   The hierarchical scheduler operations structure associated with port_id on
 *   success, NULL otherwise.
 */
const struct rte_scheddev_ops *
rte_scheddev_ops_get(uint8_t port_id, struct rte_scheddev_error *error);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_RTE_SCHEDDEV_DRIVER_H__ */
