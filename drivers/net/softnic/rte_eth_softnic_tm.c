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

#include <rte_tm_driver.h>
#include <rte_sched.h>

#include "rte_eth_softnic_internals.h"
#include "rte_eth_softnic.h"

int
tm_init(struct pmd_internals *p)
{
	struct tm_params *t = &p->tm_params;
	uint32_t n_subports, subport_id;
	int status;

	/* Port */
	t->port_params.name = p->odev->data->name;
	t->port_params.socket = p->udev->data->numa_node;
	t->port_params.rate = p->udev->data->dev_link.link_speed;

	p->sched = rte_sched_port_config(&t->port_params);
	if (p->sched == NULL)
		return -1;

	/* Subport */
	n_subports = t->port_params.n_subports_per_port;
	for (subport_id = 0; subport_id < n_subports; subport_id++) {
		uint32_t n_pipes_per_subport = t->port_params.n_pipes_per_subport;
		uint32_t pipe_id;

		status = rte_sched_subport_config(p->sched,
			subport_id,
			&t->subport_params[subport_id]);
		if (status) {
			rte_sched_port_free(p->sched);
			return -1;
		}

		/* Pipe */
		n_pipes_per_subport = t->port_params.n_pipes_per_subport;
		for (pipe_id = 0; pipe_id < n_pipes_per_subport; pipe_id++) {
			int pos = subport_id * TM_MAX_PIPES_PER_SUBPORT + pipe_id;
			int profile_id = t->pipe_to_profile[pos];

			if (profile_id < 0)
				continue;

			status = rte_sched_pipe_config(p->sched,
				subport_id,
				pipe_id,
				profile_id);
			if (status) {
				rte_sched_port_free(p->sched);
				return -1;
			}
		}
	}

	return 0;
}

void
tm_free(struct pmd_internals *p)
{
	if (p->sched)
		rte_sched_port_free(p->sched);
}

/* Traffic manager node type get */
static int
pmd_tm_node_type_get(struct rte_eth_dev *dev __rte_unused,
	uint32_t node_id __rte_unused,
	int *is_leaf __rte_unused,
	struct rte_tm_error *error __rte_unused)
{
	return 0;
}

/* Traffic manager capabilities get */
static int
pmd_tm_capabilities_get(struct rte_eth_dev *dev __rte_unused,
	struct rte_tm_capabilities *cap __rte_unused,
	struct rte_tm_error *error __rte_unused)
{
	return 0;
}

/* Traffic manager level capabilities get */
static int
pmd_tm_level_capabilities_get(struct rte_eth_dev *dev __rte_unused,
	uint32_t level_id __rte_unused,
	struct rte_tm_level_capabilities *cap __rte_unused,
	struct rte_tm_error *error __rte_unused)
{
	return 0;
}

/* Traffic manager node capabilities get */
static int
pmd_tm_node_capabilities_get(struct rte_eth_dev *dev __rte_unused,
	uint32_t node_id __rte_unused,
	struct rte_tm_node_capabilities *cap __rte_unused,
	struct rte_tm_error *error __rte_unused)
{
	return 0;
}

/* Traffic manager shaper profile add */
static int
pmd_tm_shaper_profile_add(struct rte_eth_dev *dev __rte_unused,
	uint32_t shaper_profile_id __rte_unused,
	struct rte_tm_shaper_params *profile __rte_unused,
	struct rte_tm_error *error __rte_unused)
{
	return 0;
}

/* Traffic manager shaper profile delete */
static int
pmd_tm_shaper_profile_delete(struct rte_eth_dev *dev __rte_unused,
	uint32_t shaper_profile_id __rte_unused,
	struct rte_tm_error *error __rte_unused)
{
	return 0;
}

/* Traffic manager node add */
static int
pmd_tm_node_add(struct rte_eth_dev *dev __rte_unused,
	uint32_t node_id __rte_unused,
	uint32_t parent_node_id __rte_unused,
	uint32_t priority __rte_unused,
	uint32_t weight __rte_unused,
	struct rte_tm_node_params *params __rte_unused,
	struct rte_tm_error *error __rte_unused)
{
	return 0;
}

/* Traffic manager node delete */
static int
pmd_tm_node_delete(struct rte_eth_dev *dev __rte_unused,
	uint32_t node_id __rte_unused,
	struct rte_tm_error *error __rte_unused)
{
	return 0;
}

/* Traffic manager hierarchy commit */
static int
pmd_tm_hierarchy_commit(struct rte_eth_dev *dev __rte_unused,
	int clear_on_fail __rte_unused,
	struct rte_tm_error *error __rte_unused)
{
	return 0;
}

/* Traffic manager read stats counters for specific node */
static int
pmd_tm_node_stats_read(struct rte_eth_dev *dev __rte_unused,
	uint32_t node_id __rte_unused,
	struct rte_tm_node_stats *stats __rte_unused,
	uint64_t *stats_mask __rte_unused,
	int clear __rte_unused,
	struct rte_tm_error *error __rte_unused)
{
	return 0;
}

const struct rte_tm_ops pmd_tm_ops = {
	.node_type_get = pmd_tm_node_type_get,
	.capabilities_get = pmd_tm_capabilities_get,
	.level_capabilities_get = pmd_tm_level_capabilities_get,
	.node_capabilities_get = pmd_tm_node_capabilities_get,

	.wred_profile_add = NULL,
	.wred_profile_delete = NULL,
	.shared_wred_context_add_update = NULL,
	.shared_wred_context_delete = NULL,

	.shaper_profile_add = pmd_tm_shaper_profile_add,
	.shaper_profile_delete = pmd_tm_shaper_profile_delete,
	.shared_shaper_add_update = NULL,
	.shared_shaper_delete = NULL,

	.node_add = pmd_tm_node_add,
	.node_delete = pmd_tm_node_delete,
	.node_suspend = NULL,
	.node_resume = NULL,
	.hierarchy_commit = pmd_tm_hierarchy_commit,

	.node_parent_update = NULL,
	.node_shaper_update = NULL,
	.node_shared_shaper_update = NULL,
	.node_stats_update = NULL,
	.node_wfq_weight_mode_update = NULL,
	.node_cman_update = NULL,
	.node_wred_context_update = NULL,
	.node_shared_wred_context_update = NULL,

	.node_stats_read = pmd_tm_node_stats_read,
};
