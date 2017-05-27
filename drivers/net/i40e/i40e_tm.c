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

#include <rte_malloc.h>

#include "base/i40e_prototype.h"
#include "i40e_ethdev.h"

static int i40e_tm_capabilities_get(struct rte_eth_dev *dev,
				    struct rte_tm_capabilities *cap,
				    struct rte_tm_error *error);
static int i40e_shaper_profile_add(struct rte_eth_dev *dev,
				   uint32_t shaper_profile_id,
				   struct rte_tm_shaper_params *profile,
				   struct rte_tm_error *error);
static int i40e_shaper_profile_del(struct rte_eth_dev *dev,
				   uint32_t shaper_profile_id,
				   struct rte_tm_error *error);
static int i40e_node_add(struct rte_eth_dev *dev, uint32_t node_id,
			 uint32_t parent_node_id, uint32_t priority,
			 uint32_t weight, struct rte_tm_node_params *params,
			 struct rte_tm_error *error);

const struct rte_tm_ops i40e_tm_ops = {
	.capabilities_get = i40e_tm_capabilities_get,
	.shaper_profile_add = i40e_shaper_profile_add,
	.shaper_profile_delete = i40e_shaper_profile_del,
	.node_add = i40e_node_add,
};

int
i40e_tm_ops_get(struct rte_eth_dev *dev __rte_unused,
		void *arg)
{
	if (!arg)
		return -EINVAL;

	*(const void **)arg = &i40e_tm_ops;

	return 0;
}

void
i40e_tm_conf_init(struct rte_eth_dev *dev)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);

	/* initialize shaper profile list */
	TAILQ_INIT(&pf->tm_conf.shaper_profile_list);

	/* initialize node configuration */
	pf->tm_conf.root = NULL;
	TAILQ_INIT(&pf->tm_conf.tc_list);
	pf->tm_conf.nb_tc_node = 0;
}

void
i40e_tm_conf_uninit(struct rte_eth_dev *dev)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_tm_shaper_profile *shaper_profile;
	struct i40e_tm_node *tc;

	/* clear node configuration */
	while ((tc = TAILQ_FIRST(&pf->tm_conf.tc_list))) {
		TAILQ_REMOVE(&pf->tm_conf.tc_list, tc, node);
		rte_free(tc);
	}
	pf->tm_conf.nb_tc_node = 0;
	if (pf->tm_conf.root) {
		rte_free(pf->tm_conf.root);
		pf->tm_conf.root = NULL;
	}

	/* Remove all shaper profiles */
	while ((shaper_profile =
	       TAILQ_FIRST(&pf->tm_conf.shaper_profile_list))) {
		TAILQ_REMOVE(&pf->tm_conf.shaper_profile_list,
			     shaper_profile, node);
		rte_free(shaper_profile);
	}
}

static inline uint16_t
i40e_tc_nb_get(struct rte_eth_dev *dev)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_vsi *main_vsi = pf->main_vsi;
	uint16_t sum = 0;
	int i;

	for (i = 0; i < I40E_MAX_TRAFFIC_CLASS; i++) {
		if (main_vsi->enabled_tc & BIT_ULL(i))
			sum++;
	}

	return sum;
}

static int
i40e_tm_capabilities_get(struct rte_eth_dev *dev,
			 struct rte_tm_capabilities *cap,
			 struct rte_tm_error *error)
{
	uint16_t tc_nb = 0;

	if (!cap || !error)
		return -EINVAL;

	error->type = RTE_TM_ERROR_TYPE_NONE;

	/* set all the parameters to 0 first. */
	memset(cap, 0, sizeof(struct rte_tm_capabilities));

	/* only support port + TCs */
	tc_nb = i40e_tc_nb_get(dev);
	cap->n_nodes_max = tc_nb + 1;
	cap->n_levels_max = 2;
	cap->non_leaf_nodes_identical = 0;
	cap->leaf_nodes_identical = 0;
	cap->shaper_n_max = cap->n_nodes_max;
	cap->shaper_private_n_max = cap->n_nodes_max;
	cap->shaper_private_dual_rate_n_max = 0;
	cap->shaper_private_rate_min = 0;
	/* 40Gbps -> 5GBps */
	cap->shaper_private_rate_max = 5000000000ull;
	cap->shaper_shared_n_max = 0;
	cap->shaper_shared_n_nodes_per_shaper_max = 0;
	cap->shaper_shared_n_shapers_per_node_max = 0;
	cap->shaper_shared_dual_rate_n_max = 0;
	cap->shaper_shared_rate_min = 0;
	cap->shaper_shared_rate_max = 0;
	cap->sched_n_children_max = tc_nb;
	cap->sched_sp_n_priorities_max = 0;
	cap->sched_wfq_n_children_per_group_max = 0;
	cap->sched_wfq_n_groups_max = 0;
	cap->sched_wfq_weight_max = 0;
	cap->cman_head_drop_supported = 0;
	cap->dynamic_update_mask = 0;

	/**
	 * not supported parameters are 0, below,
	 * shaper_pkt_length_adjust_min
	 * shaper_pkt_length_adjust_max
	 * cman_wred_context_n_max
	 * cman_wred_context_private_n_max
	 * cman_wred_context_shared_n_max
	 * cman_wred_context_shared_n_nodes_per_context_max
	 * cman_wred_context_shared_n_contexts_per_node_max
	 * mark_vlan_dei_supported
	 * mark_ip_ecn_tcp_supported
	 * mark_ip_ecn_sctp_supported
	 * mark_ip_dscp_supported
	 * stats_mask
	 */

	return 0;
}

static inline struct i40e_tm_shaper_profile *
i40e_shaper_profile_search(struct rte_eth_dev *dev,
			   uint32_t shaper_profile_id)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_shaper_profile_list *shaper_profile_list =
		&pf->tm_conf.shaper_profile_list;
	struct i40e_tm_shaper_profile *shaper_profile;

	TAILQ_FOREACH(shaper_profile, shaper_profile_list, node) {
		if (shaper_profile_id == shaper_profile->shaper_profile_id)
			return shaper_profile;
	}

	return NULL;
}

static int
i40e_shaper_profile_add(struct rte_eth_dev *dev,
			uint32_t shaper_profile_id,
			struct rte_tm_shaper_params *profile,
			struct rte_tm_error *error)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_tm_shaper_profile *shaper_profile;

	if (!profile || !error)
		return -EINVAL;

	/* min rate not supported */
	if (profile->committed.rate) {
		error->type = RTE_TM_ERROR_TYPE_SHAPER_PROFILE_COMMITTED_RATE;
		error->message = "committed rate not supported";
		return -EINVAL;
	}
	/* min bucket size not supported */
	if (profile->committed.size) {
		error->type = RTE_TM_ERROR_TYPE_SHAPER_PROFILE_COMMITTED_SIZE;
		error->message = "committed bucket size not supported";
		return -EINVAL;
	}
	/* max bucket size not supported */
	if (profile->peak.size) {
		error->type = RTE_TM_ERROR_TYPE_SHAPER_PROFILE_PEAK_SIZE;
		error->message = "peak bucket size not supported";
		return -EINVAL;
	}
	/* length adjustment not supported */
	if (profile->pkt_length_adjust) {
		error->type = RTE_TM_ERROR_TYPE_SHAPER_PROFILE_PKT_ADJUST_LEN;
		error->message = "packet length adjustment not supported";
		return -EINVAL;
	}

	shaper_profile = i40e_shaper_profile_search(dev, shaper_profile_id);

	if (shaper_profile) {
		error->type = RTE_TM_ERROR_TYPE_SHAPER_PROFILE_ID;
		error->message = "profile ID exist";
		return -EINVAL;
	}

	shaper_profile = rte_zmalloc("i40e_tm_shaper_profile",
				     sizeof(struct i40e_tm_shaper_profile),
				     0);
	if (!shaper_profile)
		return -ENOMEM;
	shaper_profile->shaper_profile_id = shaper_profile_id;
	(void)rte_memcpy(&shaper_profile->profile, profile,
			 sizeof(struct rte_tm_shaper_params));
	TAILQ_INSERT_TAIL(&pf->tm_conf.shaper_profile_list,
			  shaper_profile, node);

	return 0;
}

static int
i40e_shaper_profile_del(struct rte_eth_dev *dev,
			uint32_t shaper_profile_id,
			struct rte_tm_error *error)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_tm_shaper_profile *shaper_profile;

	if (!error)
		return -EINVAL;

	shaper_profile = i40e_shaper_profile_search(dev, shaper_profile_id);

	if (!shaper_profile) {
		error->type = RTE_TM_ERROR_TYPE_SHAPER_PROFILE_ID;
		error->message = "profile ID not exist";
		return -EINVAL;
	}

	/* don't delete a profile if it's used by one or several nodes */
	if (shaper_profile->reference_count) {
		error->type = RTE_TM_ERROR_TYPE_SHAPER_PROFILE;
		error->message = "profile in use";
		return -EINVAL;
	}

	TAILQ_REMOVE(&pf->tm_conf.shaper_profile_list, shaper_profile, node);
	rte_free(shaper_profile);

	return 0;
}

static inline struct i40e_tm_node *
i40e_tm_node_search(struct rte_eth_dev *dev,
		    uint32_t node_id, enum i40e_tm_node_type *node_type)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	struct i40e_tm_node_list *tc_list = &pf->tm_conf.tc_list;
	struct i40e_tm_node *tm_node;

	if (pf->tm_conf.root && pf->tm_conf.root->id == node_id) {
		*node_type = I40E_TM_NODE_TYPE_PORT;
		return pf->tm_conf.root;
	}

	TAILQ_FOREACH(tm_node, tc_list, node) {
		if (tm_node->id == node_id) {
			*node_type = I40E_TM_NODE_TYPE_TC;
			return tm_node;
		}
	}

	return NULL;
}

static int
i40e_node_add(struct rte_eth_dev *dev, uint32_t node_id,
	      uint32_t parent_node_id, uint32_t priority,
	      uint32_t weight, struct rte_tm_node_params *params,
	      struct rte_tm_error *error)
{
	struct i40e_pf *pf = I40E_DEV_PRIVATE_TO_PF(dev->data->dev_private);
	enum i40e_tm_node_type node_type = I40E_TM_NODE_TYPE_MAX;
	struct i40e_tm_shaper_profile *shaper_profile;
	struct i40e_tm_node *tm_node;
	uint16_t tc_nb = 0;

	if (!params || !error)
		return -EINVAL;

	if (node_id == RTE_TM_NODE_ID_NULL) {
		error->type = RTE_TM_ERROR_TYPE_NODE_ID;
		error->message = "invalid node id";
		return -EINVAL;
	}

	if (priority) {
		error->type = RTE_TM_ERROR_TYPE_NODE_PRIORITY;
		error->message = "priority not supported";
		return -EINVAL;
	}

	if (weight) {
		error->type = RTE_TM_ERROR_TYPE_NODE_WEIGHT;
		error->message = "weight not supported";
		return -EINVAL;
	}

	/* check if the node ID is already used */
	if (i40e_tm_node_search(dev, node_id, &node_type)) {
		error->type = RTE_TM_ERROR_TYPE_NODE_ID;
		error->message = "node id already used";
		return -EINVAL;
	}

	/* check the shaper profile id */
	shaper_profile = i40e_shaper_profile_search(dev,
						    params->shaper_profile_id);
	if (!shaper_profile) {
		error->type = RTE_TM_ERROR_TYPE_NODE_PARAMS_SHAPER_PROFILE_ID;
		error->message = "shaper profile not exist";
		return -EINVAL;
	}

	/* not support shared shaper */
	if (params->shared_shaper_id) {
		error->type = RTE_TM_ERROR_TYPE_NODE_PARAMS_SHARED_SHAPER_ID;
		error->message = "shared shaper not supported";
		return -EINVAL;
	}
	if (params->n_shared_shapers) {
		error->type = RTE_TM_ERROR_TYPE_NODE_PARAMS_N_SHARED_SHAPERS;
		error->message = "shared shaper not supported";
		return -EINVAL;
	}

	/* root node if not have a parent */
	if (parent_node_id == RTE_TM_NODE_ID_NULL) {
		/* check the unsupported parameters */
		if (params->nonleaf.wfq_weight_mode) {
			error->type =
				RTE_TM_ERROR_TYPE_NODE_PARAMS_WFQ_WEIGHT_MODE;
			error->message = "WFQ not supported";
			return -EINVAL;
		}
		if (params->nonleaf.n_sp_priorities) {
			error->type =
				RTE_TM_ERROR_TYPE_NODE_PARAMS_N_SP_PRIORITIES;
			error->message = "SP priority not supported";
			return -EINVAL;
		}

		/* obviously no more than one root */
		if (pf->tm_conf.root) {
			error->type = RTE_TM_ERROR_TYPE_NODE_PARENT_NODE_ID;
			error->message = "already have a root";
			return -EINVAL;
		}

		/* add the root node */
		tm_node = rte_zmalloc("i40e_tm_node",
				      sizeof(struct i40e_tm_node),
				      0);
		if (!tm_node)
			return -ENOMEM;
		tm_node->id = node_id;
		tm_node->priority = priority;
		tm_node->weight = weight;
		tm_node->reference_count = 0;
		tm_node->parent = NULL;
		tm_node->shaper_profile = shaper_profile;
		(void)rte_memcpy(&tm_node->params, params,
				 sizeof(struct rte_tm_node_params));
		pf->tm_conf.root = tm_node;

		/* increase the reference counter of the shaper profile */
		shaper_profile->reference_count++;

		return 0;
	}

	/* TC node */
	/* check the unsupported parameters */
	if (params->leaf.cman) {
		error->type = RTE_TM_ERROR_TYPE_NODE_PARAMS_CMAN;
		error->message = "Congestion management not supported";
		return -EINVAL;
	}
	if (params->leaf.wred.wred_profile_id !=
	    RTE_TM_WRED_PROFILE_ID_NONE) {
		error->type =
			RTE_TM_ERROR_TYPE_NODE_PARAMS_WRED_PROFILE_ID;
		error->message = "WRED not supported";
		return -EINVAL;
	}
	if (params->leaf.wred.shared_wred_context_id) {
		error->type =
			RTE_TM_ERROR_TYPE_NODE_PARAMS_SHARED_WRED_CONTEXT_ID;
		error->message = "WRED not supported";
		return -EINVAL;
	}
	if (params->leaf.wred.n_shared_wred_contexts) {
		error->type =
			RTE_TM_ERROR_TYPE_NODE_PARAMS_N_SHARED_WRED_CONTEXTS;
		error->message = "WRED not supported";
		return -EINVAL;
	}

	/* should have a root first */
	if (!pf->tm_conf.root) {
		error->type = RTE_TM_ERROR_TYPE_NODE_PARENT_NODE_ID;
		error->message = "no root yet";
		return -EINVAL;
	}
	if (pf->tm_conf.root->id != parent_node_id) {
		error->type = RTE_TM_ERROR_TYPE_NODE_PARENT_NODE_ID;
		error->message = "parent id doesn't belong to the root";
		return -EINVAL;
	}

	/* check the TC number */
	tc_nb = i40e_tc_nb_get(dev);
	if (pf->tm_conf.nb_tc_node >= tc_nb) {
		error->type = RTE_TM_ERROR_TYPE_NODE_ID;
		error->message = "too much TC";
		return -EINVAL;
	}

	/* add the TC node */
	tm_node = rte_zmalloc("i40e_tm_node",
			      sizeof(struct i40e_tm_node),
			      0);
	if (!tm_node)
		return -ENOMEM;
	tm_node->id = node_id;
	tm_node->priority = priority;
	tm_node->weight = weight;
	tm_node->reference_count = 0;
	tm_node->parent = pf->tm_conf.root;
	tm_node->shaper_profile = shaper_profile;
	(void)rte_memcpy(&tm_node->params, params,
			 sizeof(struct rte_tm_node_params));
	TAILQ_INSERT_TAIL(&pf->tm_conf.tc_list,
			  tm_node, node);
	tm_node->parent->reference_count++;
	pf->tm_conf.nb_tc_node++;

	/* increase the reference counter of the shaper profile */
	shaper_profile->reference_count++;

	return 0;
}
