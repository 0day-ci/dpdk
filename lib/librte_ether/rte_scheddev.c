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

#include <stdint.h>

#include <rte_errno.h>
#include <rte_branch_prediction.h>
#include "rte_ethdev.h"
#include "rte_scheddev_driver.h"
#include "rte_scheddev.h"

/* Get generic scheduler operations structure from a port. */
const struct rte_scheddev_ops *
rte_scheddev_ops_get(uint8_t port_id, struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops;

	if (!rte_eth_dev_is_valid_port(port_id)) {
		rte_scheddev_error_set(error,
			ENODEV,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENODEV));
		return NULL;
	}

	if ((dev->dev_ops->cap_ctrl == NULL) ||
		dev->dev_ops->cap_ctrl(dev, RTE_ETH_CAPABILITY_SCHED, &ops) ||
		(ops == NULL)) {
		rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));
		return NULL;
	}

	return ops;
}

/* Get capabilities */
int rte_scheddev_capabilities_get(uint8_t port_id,
	struct rte_scheddev_capabilities *cap,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->capabilities_get == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->capabilities_get(dev, cap, error);
}

/* Get node capabilities */
int rte_scheddev_node_capabilities_get(uint8_t port_id,
	uint32_t node_id,
	struct rte_scheddev_node_capabilities *cap,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_capabilities_get == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_capabilities_get(dev, node_id, cap, error);
}

/* Add WRED profile */
int rte_scheddev_wred_profile_add(uint8_t port_id,
	uint32_t wred_profile_id,
	struct rte_scheddev_wred_params *profile,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->wred_profile_add == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->wred_profile_add(dev, wred_profile_id, profile, error);
}

/* Delete WRED profile */
int rte_scheddev_wred_profile_delete(uint8_t port_id,
	uint32_t wred_profile_id,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->wred_profile_delete == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->wred_profile_delete(dev, wred_profile_id, error);
}

/* Add/update shared WRED context */
int rte_scheddev_shared_wred_context_add_update(uint8_t port_id,
	uint32_t shared_wred_context_id,
	uint32_t wred_profile_id,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->shared_wred_context_add_update == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->shared_wred_context_add_update(dev, shared_wred_context_id,
		wred_profile_id, error);
}

/* Delete shared WRED context */
int rte_scheddev_shared_wred_context_delete(uint8_t port_id,
	uint32_t shared_wred_context_id,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->shared_wred_context_delete == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->shared_wred_context_delete(dev, shared_wred_context_id,
		error);
}

/* Add shaper profile */
int rte_scheddev_shaper_profile_add(uint8_t port_id,
	uint32_t shaper_profile_id,
	struct rte_scheddev_shaper_params *profile,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->shaper_profile_add == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->shaper_profile_add(dev, shaper_profile_id, profile, error);
}

/* Delete WRED profile */
int rte_scheddev_shaper_profile_delete(uint8_t port_id,
	uint32_t shaper_profile_id,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->shaper_profile_delete == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->shaper_profile_delete(dev, shaper_profile_id, error);
}

/* Add shared shaper */
int rte_scheddev_shared_shaper_add_update(uint8_t port_id,
	uint32_t shared_shaper_id,
	uint32_t shaper_profile_id,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->shared_shaper_add_update == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->shared_shaper_add_update(dev, shared_shaper_id,
		shaper_profile_id, error);
}

/* Delete shared shaper */
int rte_scheddev_shared_shaper_delete(uint8_t port_id,
	uint32_t shared_shaper_id,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->shared_shaper_delete == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->shared_shaper_delete(dev, shared_shaper_id, error);
}

/* Add node to port scheduler hierarchy */
int rte_scheddev_node_add(uint8_t port_id,
	uint32_t node_id,
	uint32_t parent_node_id,
	uint32_t priority,
	uint32_t weight,
	struct rte_scheddev_node_params *params,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_add == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_add(dev, node_id, parent_node_id, priority, weight,
		params, error);
}

/* Delete node from scheduler hierarchy */
int rte_scheddev_node_delete(uint8_t port_id,
	uint32_t node_id,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_delete == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_delete(dev, node_id, error);
}

/* Suspend node */
int rte_scheddev_node_suspend(uint8_t port_id,
	uint32_t node_id,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_suspend == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_suspend(dev, node_id, error);
}

/* Resume node */
int rte_scheddev_node_resume(uint8_t port_id,
	uint32_t node_id,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_resume == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_resume(dev, node_id, error);
}

/* Set the initial port scheduler hierarchy */
int rte_scheddev_hierarchy_set(uint8_t port_id,
	int clear_on_fail,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->hierarchy_set == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->hierarchy_set(dev, clear_on_fail, error);
}

/* Update node parent  */
int rte_scheddev_node_parent_update(uint8_t port_id,
	uint32_t node_id,
	uint32_t parent_node_id,
	uint32_t priority,
	uint32_t weight,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_parent_update == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_parent_update(dev, node_id, parent_node_id, priority,
		weight, error);
}

/* Update node private shaper */
int rte_scheddev_node_shaper_update(uint8_t port_id,
	uint32_t node_id,
	uint32_t shaper_profile_id,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_shaper_update == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_shaper_update(dev, node_id, shaper_profile_id,
		error);
}

/* Update node shared shapers */
int rte_scheddev_node_shared_shaper_update(uint8_t port_id,
	uint32_t node_id,
	uint32_t shared_shaper_id,
	int add,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_shared_shaper_update == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_shared_shaper_update(dev, node_id, shared_shaper_id,
		add, error);
}

/* Update scheduling mode */
int rte_scheddev_node_scheduling_mode_update(uint8_t port_id,
	uint32_t node_id,
	int *scheduling_mode_per_priority,
	uint32_t n_priorities,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_scheduling_mode_update == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_scheduling_mode_update(dev, node_id,
		scheduling_mode_per_priority, n_priorities, error);
}

/* Update node congestion management mode */
int rte_scheddev_node_cman_update(uint8_t port_id,
	uint32_t node_id,
	enum rte_scheddev_cman_mode cman,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_cman_update == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_cman_update(dev, node_id, cman, error);
}

/* Update node private WRED context */
int rte_scheddev_node_wred_context_update(uint8_t port_id,
	uint32_t node_id,
	uint32_t wred_profile_id,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_wred_context_update == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_wred_context_update(dev, node_id, wred_profile_id,
		error);
}

/* Update node shared WRED context */
int rte_scheddev_node_shared_wred_context_update(uint8_t port_id,
	uint32_t node_id,
	uint32_t shared_wred_context_id,
	int add,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_shared_wred_context_update == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_shared_wred_context_update(dev, node_id,
		shared_wred_context_id, add, error);
}

/* Packet marking - VLAN DEI */
int rte_scheddev_mark_vlan_dei(uint8_t port_id,
	int mark_green,
	int mark_yellow,
	int mark_red,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->mark_vlan_dei == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->mark_vlan_dei(dev, mark_green, mark_yellow, mark_red,
		error);
}

/* Packet marking - IPv4/IPv6 ECN */
int rte_scheddev_mark_ip_ecn(uint8_t port_id,
	int mark_green,
	int mark_yellow,
	int mark_red,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->mark_ip_ecn == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->mark_ip_ecn(dev, mark_green, mark_yellow, mark_red, error);
}

/* Packet marking - IPv4/IPv6 DSCP */
int rte_scheddev_mark_ip_dscp(uint8_t port_id,
	int mark_green,
	int mark_yellow,
	int mark_red,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->mark_ip_dscp == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->mark_ip_dscp(dev, mark_green, mark_yellow, mark_red,
		error);
}

/* Get set of stats counter types currently enabled for all nodes */
int rte_scheddev_stats_get_enabled(uint8_t port_id,
	uint64_t *nonleaf_node_capability_stats_mask,
	uint64_t *nonleaf_node_enabled_stats_mask,
	uint64_t *leaf_node_capability_stats_mask,
	uint64_t *leaf_node_enabled_stats_mask,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->stats_get_enabled == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->stats_get_enabled(dev,
		nonleaf_node_capability_stats_mask,
		nonleaf_node_enabled_stats_mask,
		leaf_node_capability_stats_mask,
		leaf_node_enabled_stats_mask,
		error);
}

/* Enable specified set of stats counter types for all nodes */
int rte_scheddev_stats_enable(uint8_t port_id,
	uint64_t nonleaf_node_enabled_stats_mask,
	uint64_t leaf_node_enabled_stats_mask,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->stats_enable == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->stats_enable(dev,
		nonleaf_node_enabled_stats_mask,
		leaf_node_enabled_stats_mask,
		error);
}

/* Get set of stats counter types currently enabled for specific node */
int rte_scheddev_node_stats_get_enabled(uint8_t port_id,
	uint32_t node_id,
	uint64_t *capability_stats_mask,
	uint64_t *enabled_stats_mask,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_stats_get_enabled == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_stats_get_enabled(dev,
		node_id,
		capability_stats_mask,
		enabled_stats_mask,
		error);
}

/* Enable specified set of stats counter types for specific node */
int rte_scheddev_node_stats_enable(uint8_t port_id,
	uint32_t node_id,
	uint64_t enabled_stats_mask,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_stats_enable == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_stats_enable(dev, node_id, enabled_stats_mask, error);
}

/* Read and/or clear stats counters for specific node */
int rte_scheddev_node_stats_read(uint8_t port_id,
	uint32_t node_id,
	struct rte_scheddev_node_stats *stats,
	int clear,
	struct rte_scheddev_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_scheddev_ops *ops =
		rte_scheddev_ops_get(port_id, error);

	if (ops == NULL)
		return -rte_errno;

	if (ops->node_stats_read == NULL)
		return -rte_scheddev_error_set(error,
			ENOSYS,
			RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));

	return ops->node_stats_read(dev, node_id, stats, clear, error);
}
