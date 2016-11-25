/*-
 *   BSD LICENSE
 *
 *   Copyright 2016 6WIND S.A.
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
 *     * Neither the name of 6WIND S.A. nor the names of its
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

#include <sys/queue.h>

#include <rte_ethdev.h>
#include <rte_flow.h>
#include <rte_flow_driver.h>
#include <rte_malloc.h>

#include "mlx5.h"

struct rte_flow {
	LIST_ENTRY(rte_flow) next;
};

/**
 * Validate a flow supported by the NIC.
 *
 * @param priv
 *   Pointer to private structure.
 * @param[in] attr
 *   Flow rule attributes.
 * @param[in] pattern
 *   Pattern specification (list terminated by the END pattern item).
 * @param[in] actions
 *   Associated actions (list terminated by the END action).
 * @param[out] error
 *   Perform verbose error reporting if not NULL.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
static int
priv_flow_validate(struct priv *priv,
		   const struct rte_flow_attr *attr,
		   const struct rte_flow_item items[],
		   const struct rte_flow_action actions[],
		   struct rte_flow_error *error)
{
	(void)priv;
	const struct rte_flow_item *ilast = NULL;
	const struct rte_flow_action *alast = NULL;

	if (attr->group) {
		rte_flow_error_set(error, ENOTSUP,
				   RTE_FLOW_ERROR_TYPE_ATTR_GROUP,
				   NULL,
				   "groups are not supported");
		return -rte_errno;
	}
	if (attr->priority) {
		rte_flow_error_set(error, ENOTSUP,
				   RTE_FLOW_ERROR_TYPE_ATTR_PRIORITY,
				   NULL,
				   "priorities are not supported");
		return -rte_errno;
	}
	if (attr->egress) {
		rte_flow_error_set(error, ENOTSUP,
				   RTE_FLOW_ERROR_TYPE_ATTR_EGRESS,
				   NULL,
				   "egress is not supported");
		return -rte_errno;
	}
	if (!attr->ingress) {
		rte_flow_error_set(error, ENOTSUP,
				   RTE_FLOW_ERROR_TYPE_ATTR_INGRESS,
				   NULL,
				   "only ingress is supported");
		return -rte_errno;
	}
	for (; items->type != RTE_FLOW_ITEM_TYPE_END; ++items) {
		if (items->type == RTE_FLOW_ITEM_TYPE_VOID) {
			continue;
		} else if (items->type == RTE_FLOW_ITEM_TYPE_ETH) {
			if (ilast)
				goto exit_item_not_supported;
			ilast = items;
		} else if ((items->type == RTE_FLOW_ITEM_TYPE_IPV4) ||
			   (items->type == RTE_FLOW_ITEM_TYPE_IPV6)) {
			if (!ilast)
				goto exit_item_not_supported;
			else if (ilast->type != RTE_FLOW_ITEM_TYPE_ETH)
				goto exit_item_not_supported;
			ilast = items;
		} else if ((items->type == RTE_FLOW_ITEM_TYPE_UDP) ||
			   (items->type == RTE_FLOW_ITEM_TYPE_TCP)) {
			if (!ilast)
				goto exit_item_not_supported;
			else if ((ilast->type != RTE_FLOW_ITEM_TYPE_IPV4) &&
				 (ilast->type != RTE_FLOW_ITEM_TYPE_IPV6))
				goto exit_item_not_supported;
			ilast = items;
		} else {
			goto exit_item_not_supported;
		}
	}
	for (; actions->type != RTE_FLOW_ACTION_TYPE_END; ++actions) {
		if (actions->type == RTE_FLOW_ACTION_TYPE_VOID) {
			continue;
		} else if ((actions->type == RTE_FLOW_ACTION_TYPE_QUEUE) ||
			   (actions->type == RTE_FLOW_ACTION_TYPE_DROP)) {
			if (alast &&
			    alast->type != actions->type)
				goto exit_action_not_supported;
			alast = actions;
		} else {
			goto exit_action_not_supported;
		}
	}
	return 0;
exit_item_not_supported:
	rte_flow_error_set(error, ENOTSUP, RTE_FLOW_ERROR_TYPE_ITEM,
			   items, "item not supported");
	return -rte_errno;
exit_action_not_supported:
	rte_flow_error_set(error, ENOTSUP, RTE_FLOW_ERROR_TYPE_ACTION,
			   actions, "action not supported");
	return -rte_errno;
}

/**
 * Validate a flow supported by the NIC.
 *
 * @see rte_flow_validate()
 * @see rte_flow_ops
 */
int
mlx5_flow_validate(struct rte_eth_dev *dev,
		   const struct rte_flow_attr *attr,
		   const struct rte_flow_item items[],
		   const struct rte_flow_action actions[],
		   struct rte_flow_error *error)
{
	struct priv *priv = dev->data->dev_private;
	int ret;

	priv_lock(priv);
	ret = priv_flow_validate(priv, attr, items, actions, error);
	priv_unlock(priv);
	return ret;
}

/**
 * Create a flow.
 *
 * @see rte_flow_create()
 * @see rte_flow_ops
 */
struct rte_flow *
mlx5_flow_create(struct rte_eth_dev *dev,
		 const struct rte_flow_attr *attr,
		 const struct rte_flow_item items[],
		 const struct rte_flow_action actions[],
		 struct rte_flow_error *error)
{
	struct priv *priv = dev->data->dev_private;
	struct rte_flow *flow;

	priv_lock(priv);
	if (priv_flow_validate(priv, attr, items, actions, error)) {
		priv_unlock(priv);
		return NULL;
	}
	flow = rte_malloc(__func__, sizeof(struct rte_flow), 0);
	LIST_INSERT_HEAD(&priv->flows, flow, next);
	priv_unlock(priv);
	return flow;
}

/**
 * Destroy a flow.
 *
 * @param  priv
 *   Pointer to private structure.
 * @param[in] flow
 *   Pointer to the flow to destroy.
 */
static void
priv_flow_destroy(struct priv *priv,
		  struct rte_flow *flow)
{
	(void)priv;
	LIST_REMOVE(flow, next);
	rte_free(flow);
}

/**
 * Destroy a flow.
 *
 * @see rte_flow_destroy()
 * @see rte_flow_ops
 */
int
mlx5_flow_destroy(struct rte_eth_dev *dev,
		  struct rte_flow *flow,
		  struct rte_flow_error *error)
{
	struct priv *priv = dev->data->dev_private;

	(void)error;
	priv_lock(priv);
	priv_flow_destroy(priv, flow);
	priv_unlock(priv);
	return 0;
}

/**
 * Destroy all flows.
 *
 * @see rte_flow_flush()
 * @see rte_flow_ops
 */
int
mlx5_flow_flush(struct rte_eth_dev *dev,
		struct rte_flow_error *error)
{
	struct priv *priv = dev->data->dev_private;

	(void)error;
	priv_lock(priv);
	while (!LIST_EMPTY(&priv->flows)) {
		struct rte_flow *flow;

		flow = LIST_FIRST(&priv->flows);
		priv_flow_destroy(priv, flow);
	}
	priv_unlock(priv);
	return 0;
}
