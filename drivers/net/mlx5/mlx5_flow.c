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
#include <string.h>

/* Verbs header. */
/* ISO C doesn't support unnamed structs/unions, disabling -pedantic. */
#ifdef PEDANTIC
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <infiniband/verbs.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-Wpedantic"
#endif

#include <rte_ethdev.h>
#include <rte_flow.h>
#include <rte_flow_driver.h>
#include <rte_malloc.h>

#include "mlx5.h"

/** Define a value to use as index for the drop queue. */
#define MLX5_FLOW_DROP_QUEUE ((uint32_t)-1)

struct rte_flow {
	LIST_ENTRY(rte_flow) next;
	struct ibv_exp_flow_attr *ibv_attr;
	struct ibv_exp_rwq_ind_table *ind_table;
	struct ibv_qp *qp;
	struct ibv_exp_flow *ibv_flow;
	struct ibv_exp_wq *wq;
	struct ibv_cq *cq;
	uint8_t drop;
};

/**
 * Check support for a given item.
 *
 * @param item[in]
 *   Item specification.
 * @param mask[in]
 *   Bit-mask covering supported fields to compare with spec, last and mask in
 *   \item.
 * @param size
 *   Bit-Mask size in bytes.
 *
 * @return
 *   0 on success.
 */
static int
mlx5_flow_item_validate(const struct rte_flow_item *item,
			const uint8_t *mask, unsigned int size)
{
	int ret = 0;

	if (item->spec && !item->mask) {
		unsigned int i;
		const uint8_t *spec = item->spec;

		for (i = 0; i < size; ++i)
			if ((spec[i] | mask[i]) != mask[i])
				return -1;
	}
	if (item->last && !item->mask) {
		unsigned int i;
		const uint8_t *spec = item->last;

		for (i = 0; i < size; ++i)
			if ((spec[i] | mask[i]) != mask[i])
				return -1;
	}
	if (item->mask) {
		unsigned int i;
		const uint8_t *spec = item->mask;

		for (i = 0; i < size; ++i)
			if ((spec[i] | mask[i]) != mask[i])
				return -1;
	}
	if (item->spec && item->last) {
		uint8_t spec[size];
		uint8_t last[size];
		const uint8_t *apply = mask;
		unsigned int i;

		if (item->mask)
			apply = item->mask;
		for (i = 0; i < size; ++i) {
			spec[i] = ((const uint8_t *)item->spec)[i] & apply[i];
			last[i] = ((const uint8_t *)item->last)[i] & apply[i];
		}
		ret = memcmp(spec, last, size);
	}
	return ret;
}

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
	const struct rte_flow_item *ilast = NULL;
	const struct rte_flow_action *alast = NULL;
	/* Supported mask. */
	const struct rte_flow_item_eth eth_mask = {
		.dst.addr_bytes = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff },
		.src.addr_bytes = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff },
	};
	const struct rte_flow_item_ipv4 ipv4_mask = {
		.hdr = {
			.src_addr = -1,
			.dst_addr = -1,
		},
	};
	const struct rte_flow_item_ipv6 ipv6_mask = {
		.hdr = {
			.src_addr = {
				0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
			},
			.dst_addr = {
				0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
			},
		},
	};
	const struct rte_flow_item_udp udp_mask = {
		.hdr = {
			.src_port = -1,
			.dst_port = -1,
		},
	};
	const struct rte_flow_item_tcp tcp_mask = {
		.hdr = {
			.src_port = -1,
			.dst_port = -1,
		},
	};

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
		int err = 0;

		if (items->type == RTE_FLOW_ITEM_TYPE_VOID) {
			continue;
		} else if (items->type == RTE_FLOW_ITEM_TYPE_ETH) {
			if (ilast)
				goto exit_item_not_supported;
			ilast = items;
			err = mlx5_flow_item_validate(
					items,
					(const uint8_t *)&eth_mask,
					sizeof(eth_mask));
			if (err)
				goto exit_item_not_supported;
		} else if (items->type == RTE_FLOW_ITEM_TYPE_IPV4) {
			if (!ilast)
				goto exit_item_not_supported;
			else if (ilast->type != RTE_FLOW_ITEM_TYPE_ETH)
				goto exit_item_not_supported;
			ilast = items;
			err = mlx5_flow_item_validate(
					items,
					(const uint8_t *)&ipv4_mask,
					sizeof(ipv4_mask));
			if (err)
				goto exit_item_not_supported;
		} else if (items->type == RTE_FLOW_ITEM_TYPE_IPV6) {
			if (!ilast)
				goto exit_item_not_supported;
			else if (ilast->type != RTE_FLOW_ITEM_TYPE_ETH)
				goto exit_item_not_supported;
			ilast = items;
			err = mlx5_flow_item_validate(
					items,
					(const uint8_t *)&ipv6_mask,
					sizeof(ipv6_mask));
			if (err)
				goto exit_item_not_supported;
		} else if (items->type == RTE_FLOW_ITEM_TYPE_UDP) {
			if (!ilast)
				goto exit_item_not_supported;
			else if ((ilast->type != RTE_FLOW_ITEM_TYPE_IPV4) &&
				 (ilast->type != RTE_FLOW_ITEM_TYPE_IPV6))
				goto exit_item_not_supported;
			ilast = items;
			err = mlx5_flow_item_validate(
					items,
					(const uint8_t *)&udp_mask,
					sizeof(udp_mask));
			if (err)
				goto exit_item_not_supported;
		} else if (items->type == RTE_FLOW_ITEM_TYPE_TCP) {
			if (!ilast)
				goto exit_item_not_supported;
			else if ((ilast->type != RTE_FLOW_ITEM_TYPE_IPV4) &&
				 (ilast->type != RTE_FLOW_ITEM_TYPE_IPV6))
				goto exit_item_not_supported;
			ilast = items;
			err = mlx5_flow_item_validate(
					items,
					(const uint8_t *)&tcp_mask,
					sizeof(tcp_mask));
			if (err)
				goto exit_item_not_supported;
		} else {
			goto exit_item_not_supported;
		}
	}
	for (; actions->type != RTE_FLOW_ACTION_TYPE_END; ++actions) {
		if (actions->type == RTE_FLOW_ACTION_TYPE_VOID) {
			continue;
		} else if (actions->type == RTE_FLOW_ACTION_TYPE_QUEUE) {
			const struct rte_flow_action_queue *queue =
				(const struct rte_flow_action_queue *)
				actions->conf;

			if (alast &&
			    alast->type != actions->type)
				goto exit_action_not_supported;
			if (queue->index > (priv->rxqs_n - 1)) {
				rte_flow_error_set(error, EINVAL,
						   RTE_FLOW_ERROR_TYPE_ACTION,
						   actions,
						   "queue index error");
				goto exit;
			}
			alast = actions;
		} else if (actions->type == RTE_FLOW_ACTION_TYPE_DROP) {
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
exit:
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
 * Convert Ethernet item to Verbs specification.
 *
 * @param item[in]
 *   Item specification.
 * @param eth[in, out]
 *   Verbs Ethernet specification structure.
 */
static void
mlx5_flow_create_eth(const struct rte_flow_item *item,
		     struct ibv_exp_flow_spec_eth *eth)
{
	const struct rte_flow_item_eth *spec = item->spec;
	const struct rte_flow_item_eth *mask = item->mask;
	unsigned int i;

	memset(eth, 0, sizeof(struct ibv_exp_flow_spec_eth));
	*eth = (struct ibv_exp_flow_spec_eth) {
		.type = IBV_EXP_FLOW_SPEC_ETH,
		.size = sizeof(struct ibv_exp_flow_spec_eth),
	};
	if (spec) {
		memcpy(eth->val.dst_mac, spec->dst.addr_bytes, ETHER_ADDR_LEN);
		memcpy(eth->val.src_mac, spec->src.addr_bytes, ETHER_ADDR_LEN);
	}
	if (mask) {
		memcpy(eth->mask.dst_mac, mask->dst.addr_bytes, ETHER_ADDR_LEN);
		memcpy(eth->mask.src_mac, mask->src.addr_bytes, ETHER_ADDR_LEN);
	}
	/* Remove unwanted bits from values. */
	for (i = 0; i < ETHER_ADDR_LEN; ++i) {
		eth->val.dst_mac[i] &= eth->mask.dst_mac[i];
		eth->val.src_mac[i] &= eth->mask.src_mac[i];
	}
	eth->val.ether_type &= eth->mask.ether_type;
	eth->val.vlan_tag &= eth->mask.vlan_tag;
}

/**
 * Convert IPv4 item to Verbs specification.
 *
 * @param item[in]
 *   Item specification.
 * @param ipv4[in, out]
 *   Verbs IPv4 specification structure.
 */
static void
mlx5_flow_create_ipv4(const struct rte_flow_item *item,
		      struct ibv_exp_flow_spec_ipv4 *ipv4)
{
	const struct rte_flow_item_ipv4 *spec = item->spec;
	const struct rte_flow_item_ipv4 *mask = item->mask;

	memset(ipv4, 0, sizeof(struct ibv_exp_flow_spec_ipv4));
	*ipv4 = (struct ibv_exp_flow_spec_ipv4) {
		.type = IBV_EXP_FLOW_SPEC_IPV4,
		.size = sizeof(struct ibv_exp_flow_spec_ipv4),
	};
	if (spec) {
		ipv4->val = (struct ibv_exp_flow_ipv4_filter){
			.src_ip = spec->hdr.src_addr,
			.dst_ip = spec->hdr.dst_addr,
		};
	}
	if (mask) {
		ipv4->mask = (struct ibv_exp_flow_ipv4_filter){
			.src_ip = mask->hdr.src_addr,
			.dst_ip = mask->hdr.dst_addr,
		};
	}
	/* Remove unwanted bits from values. */
	ipv4->val.src_ip &= ipv4->mask.src_ip;
	ipv4->val.dst_ip &= ipv4->mask.dst_ip;
}

/**
 * Convert IPv6 item to Verbs specification.
 *
 * @param item[in]
 *   Item specification.
 * @param ipv6[in, out]
 *   Verbs IPv6 specification structure.
 */
static void
mlx5_flow_create_ipv6(const struct rte_flow_item *item,
		      struct ibv_exp_flow_spec_ipv6 *ipv6)
{
	const struct rte_flow_item_ipv6 *spec = item->spec;
	const struct rte_flow_item_ipv6 *mask = item->mask;
	unsigned int i;

	memset(ipv6, 0, sizeof(struct ibv_exp_flow_spec_ipv6));
	ipv6->type = IBV_EXP_FLOW_SPEC_IPV6;
	ipv6->size = sizeof(struct ibv_exp_flow_spec_ipv6);
	if (spec) {
		memcpy(ipv6->val.src_ip, spec->hdr.src_addr,
		       RTE_DIM(ipv6->val.src_ip));
		memcpy(ipv6->val.dst_ip, spec->hdr.dst_addr,
		       RTE_DIM(ipv6->val.dst_ip));
	}
	if (mask) {
		memcpy(ipv6->mask.src_ip, mask->hdr.src_addr,
		       RTE_DIM(ipv6->mask.src_ip));
		memcpy(ipv6->mask.dst_ip, mask->hdr.dst_addr,
		       RTE_DIM(ipv6->mask.dst_ip));
	}
	/* Remove unwanted bits from values. */
	for (i = 0; i < RTE_DIM(ipv6->val.src_ip); ++i) {
		ipv6->val.src_ip[i] &= ipv6->mask.src_ip[i];
		ipv6->val.dst_ip[i] &= ipv6->mask.dst_ip[i];
	}
}

/**
 * Convert UDP item to Verbs specification.
 *
 * @param item[in]
 *   Item specification.
 * @param udp[in, out]
 *   Verbs UDP specification structure.
 */
static void
mlx5_flow_create_udp(const struct rte_flow_item *item,
		     struct ibv_exp_flow_spec_tcp_udp *udp)
{
	const struct rte_flow_item_udp *spec = item->spec;
	const struct rte_flow_item_udp *mask = item->mask;

	memset(udp, 0, sizeof(struct ibv_exp_flow_spec_tcp_udp));
	*udp = (struct ibv_exp_flow_spec_tcp_udp) {
		.type = IBV_EXP_FLOW_SPEC_UDP,
		.size = sizeof(struct ibv_exp_flow_spec_tcp_udp),
	};
	udp->type = IBV_EXP_FLOW_SPEC_UDP;
	if (spec) {
		udp->val.dst_port = spec->hdr.dst_port;
		udp->val.src_port = spec->hdr.src_port;
	}
	if (mask) {
		udp->mask.dst_port = mask->hdr.dst_port;
		udp->mask.src_port = mask->hdr.src_port;
	}
	/* Remove unwanted bits from values. */
	udp->val.src_port &= udp->mask.src_port;
	udp->val.dst_port &= udp->mask.dst_port;
}

/**
 * Convert TCP item to Verbs specification.
 *
 * @param item[in]
 *   Item specification.
 * @param tcp[in, out]
 *   Verbs TCP specification structure.
 */
static void
mlx5_flow_create_tcp(const struct rte_flow_item *item,
		     struct ibv_exp_flow_spec_tcp_udp *tcp)
{
	const struct rte_flow_item_tcp *spec = item->spec;
	const struct rte_flow_item_tcp *mask = item->mask;

	memset(tcp, 0, sizeof(struct ibv_exp_flow_spec_tcp_udp));
	*tcp = (struct ibv_exp_flow_spec_tcp_udp) {
		.type = IBV_EXP_FLOW_SPEC_TCP,
		.size = sizeof(struct ibv_exp_flow_spec_tcp_udp),
	};
	tcp->type = IBV_EXP_FLOW_SPEC_TCP;
	if (spec) {
		tcp->val.dst_port = spec->hdr.dst_port;
		tcp->val.src_port = spec->hdr.src_port;
	}
	if (mask) {
		tcp->mask.dst_port = mask->hdr.dst_port;
		tcp->mask.src_port = mask->hdr.src_port;
	}
	/* Remove unwanted bits from values. */
	tcp->val.src_port &= tcp->mask.src_port;
	tcp->val.dst_port &= tcp->mask.dst_port;
}

/**
 * Complete flow rule creation.
 *
 * @param  priv
 *   Pointer to private structure.
 * @param  ibv_attr
 *   Verbs flow attributes.
 * @param  queue
 *   Destination queue.
 * @param[out] error
 *   Perform verbose error reporting if not NULL.
 *
 * @return
 *   A flow if the rule could be created.
 */
static struct rte_flow *
priv_flow_create_action_queue(struct priv *priv,
			      struct ibv_exp_flow_attr *ibv_attr,
			      uint32_t queue,
			      struct rte_flow_error *error)
{
	struct rxq_ctrl *rxq;
	struct rte_flow *rte_flow;

	assert(priv->pd);
	assert(priv->ctx);
	rte_flow = rte_calloc(__func__, 1, sizeof(*rte_flow), 0);
	if (!rte_flow) {
		rte_flow_error_set(error, ENOMEM, RTE_FLOW_ERROR_TYPE_ACTION,
				   NULL, "cannot allocate flow memory");
		return NULL;
	}
	if (queue == MLX5_FLOW_DROP_QUEUE) {
		rte_flow->drop = 1;
		rte_flow->cq =
			ibv_exp_create_cq(priv->ctx, 1, NULL, NULL, 0,
					  &(struct ibv_exp_cq_init_attr){
						  .comp_mask = 0,
					  });
		if (!rte_flow->cq) {
			rte_flow_error_set(error, ENOMEM,
					   RTE_FLOW_ERROR_TYPE_ACTION,
					   NULL, "cannot allocate CQ");
			goto error;
		}
		rte_flow->wq = ibv_exp_create_wq(
			priv->ctx,
			&(struct ibv_exp_wq_init_attr){
				.wq_type = IBV_EXP_WQT_RQ,
				.max_recv_wr = 1,
				.max_recv_sge = 1,
				.pd = priv->pd,
				.cq = rte_flow->cq,
			});
	} else {
		rxq = container_of((*priv->rxqs)[queue], struct rxq_ctrl, rxq);
		rte_flow->drop = 0;
		rte_flow->wq = rxq->wq;
	}
	rte_flow->ibv_attr = ibv_attr;
	rte_flow->ind_table = ibv_exp_create_rwq_ind_table(
		priv->ctx,
		&(struct ibv_exp_rwq_ind_table_init_attr){
			.pd = priv->pd,
			.log_ind_tbl_size = 0,
			.ind_tbl = &rte_flow->wq,
			.comp_mask = 0,
		});
	if (!rte_flow->ind_table) {
		rte_flow_error_set(error, ENOMEM, RTE_FLOW_ERROR_TYPE_ACTION,
				   NULL, "cannot allocate indirection table");
		goto error;
	}
	rte_flow->qp = ibv_exp_create_qp(
		priv->ctx,
		&(struct ibv_exp_qp_init_attr){
			.qp_type = IBV_QPT_RAW_PACKET,
			.comp_mask =
				IBV_EXP_QP_INIT_ATTR_PD |
				IBV_EXP_QP_INIT_ATTR_PORT |
				IBV_EXP_QP_INIT_ATTR_RX_HASH,
			.pd = priv->pd,
			.rx_hash_conf = &(struct ibv_exp_rx_hash_conf){
				.rx_hash_function =
					IBV_EXP_RX_HASH_FUNC_TOEPLITZ,
				.rx_hash_key_len = rss_hash_default_key_len,
				.rx_hash_key = rss_hash_default_key,
				.rx_hash_fields_mask = 0,
				.rwq_ind_tbl = rte_flow->ind_table,
			},
			.port_num = priv->port,
		});
	if (!rte_flow->qp) {
		rte_flow_error_set(error, ENOMEM, RTE_FLOW_ERROR_TYPE_ACTION,
				   NULL, "cannot allocate QP");
		goto error;
	}
	rte_flow->ibv_flow = ibv_exp_create_flow(rte_flow->qp,
						 rte_flow->ibv_attr);
	if (!rte_flow->ibv_flow) {
		rte_flow_error_set(error, ENOMEM, RTE_FLOW_ERROR_TYPE_ACTION,
				   NULL, "flow rule creation failure");
		goto error;
	}
	if (LIST_EMPTY(&priv->flows))
		LIST_INIT(&priv->flows);
	LIST_INSERT_HEAD(&priv->flows, rte_flow, next);
	return rte_flow;
error:
	assert(rte_flow);
	if (rte_flow->qp)
		ibv_destroy_qp(rte_flow->qp);
	if (rte_flow->ind_table)
		ibv_exp_destroy_rwq_ind_table(rte_flow->ind_table);
	if (rte_flow->drop && rte_flow->wq)
		ibv_exp_destroy_wq(rte_flow->wq);
	if (rte_flow->drop && rte_flow->cq)
		ibv_destroy_cq(rte_flow->cq);
	rte_free(rte_flow->ibv_attr);
	rte_free(rte_flow);
	return NULL;
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
	struct rte_flow *rte_flow = NULL;
	struct ibv_exp_flow_attr *ibv_attr;
	unsigned int flow_size = sizeof(struct ibv_exp_flow_attr);

	priv_lock(priv);
	if (priv_flow_validate(priv, attr, items, actions, error))
		goto exit;
	ibv_attr = rte_malloc(__func__, flow_size, 0);
	if (!ibv_attr) {
		rte_flow_error_set(error, ENOMEM, RTE_FLOW_ERROR_TYPE_HANDLE,
				   NULL, "cannot allocate ibv_attr memory");
		goto exit;
	}
	*ibv_attr = (struct ibv_exp_flow_attr){
		.type = IBV_EXP_FLOW_ATTR_NORMAL,
		.size = sizeof(struct ibv_exp_flow_attr),
		.priority = attr->priority,
		.num_of_specs = 0,
		.port = 0,
		.flags = 0,
		.reserved = 0,
	};
	/* Update ibv_flow_spec. */
	for (; items->type != RTE_FLOW_ITEM_TYPE_END; ++items) {
		if (items->type == RTE_FLOW_ITEM_TYPE_VOID) {
			continue;
		} else if (items->type == RTE_FLOW_ITEM_TYPE_ETH) {
			struct ibv_exp_flow_spec_eth *eth;
			unsigned int eth_size =
				sizeof(struct ibv_exp_flow_spec_eth);

			ibv_attr = rte_realloc(ibv_attr,
					       flow_size + eth_size, 0);
			if (!ibv_attr)
				goto error_no_memory;
			eth = (void *)((uintptr_t)ibv_attr + flow_size);
			mlx5_flow_create_eth(items, eth);
			flow_size += eth_size;
			++ibv_attr->num_of_specs;
			ibv_attr->priority = 2;
		} else if (items->type == RTE_FLOW_ITEM_TYPE_IPV4) {
			struct ibv_exp_flow_spec_ipv4 *ipv4;
			unsigned int ipv4_size =
				sizeof(struct ibv_exp_flow_spec_ipv4);

			ibv_attr = rte_realloc(ibv_attr,
					       flow_size + ipv4_size, 0);
			if (!ibv_attr)
				goto error_no_memory;
			ipv4 = (void *)((uintptr_t)ibv_attr + flow_size);
			mlx5_flow_create_ipv4(items, ipv4);
			flow_size += ipv4_size;
			++ibv_attr->num_of_specs;
			ibv_attr->priority = 1;
		} else if (items->type == RTE_FLOW_ITEM_TYPE_IPV6) {
			struct ibv_exp_flow_spec_ipv6 *ipv6;
			unsigned int ipv6_size =
				sizeof(struct ibv_exp_flow_spec_ipv6);

			ibv_attr = rte_realloc(ibv_attr,
					       flow_size + ipv6_size, 0);
			if (!ibv_attr)
				goto error_no_memory;
			ipv6 = (void *)((uintptr_t)ibv_attr + flow_size);
			mlx5_flow_create_ipv6(items, ipv6);
			flow_size += ipv6_size;
			++ibv_attr->num_of_specs;
			ibv_attr->priority = 1;
		} else if (items->type == RTE_FLOW_ITEM_TYPE_UDP) {
			struct ibv_exp_flow_spec_tcp_udp *udp;
			unsigned int udp_size =
				sizeof(struct ibv_exp_flow_spec_tcp_udp);

			ibv_attr = rte_realloc(ibv_attr,
					       flow_size + udp_size, 0);
			if (!ibv_attr)
				goto error_no_memory;
			udp = (void *)((uintptr_t)ibv_attr + flow_size);
			mlx5_flow_create_udp(items, udp);
			flow_size += udp_size;
			++ibv_attr->num_of_specs;
			ibv_attr->priority = 0;
		} else if (items->type == RTE_FLOW_ITEM_TYPE_TCP) {
			struct ibv_exp_flow_spec_tcp_udp *tcp;
			unsigned int tcp_size =
				sizeof(struct ibv_exp_flow_spec_tcp_udp);

			ibv_attr = rte_realloc(ibv_attr,
					       flow_size + tcp_size, 0);
			if (!ibv_attr)
				goto error_no_memory;
			tcp = (void *)((uintptr_t)ibv_attr + flow_size);
			mlx5_flow_create_tcp(items, tcp);
			flow_size += tcp_size;
			++ibv_attr->num_of_specs;
			ibv_attr->priority = 0;
		} else {
			/* This default rule should not happen. */
			rte_free(ibv_attr);
			rte_flow_error_set(
				error, ENOTSUP, RTE_FLOW_ERROR_TYPE_ITEM,
				items, "unsupported item");
			goto exit;
		}
	}
	for (; actions->type != RTE_FLOW_ACTION_TYPE_END; ++actions) {
		if (actions->type == RTE_FLOW_ACTION_TYPE_VOID) {
			continue;
		} else if (actions->type == RTE_FLOW_ACTION_TYPE_QUEUE) {
			const struct rte_flow_action_queue *queue =
				(const struct rte_flow_action_queue *)
				actions->conf;

			rte_flow = priv_flow_create_action_queue(
					priv, ibv_attr,
					queue->index, error);
		} else if (actions->type == RTE_FLOW_ACTION_TYPE_DROP) {
			rte_flow = priv_flow_create_action_queue(
					priv, ibv_attr,
					MLX5_FLOW_DROP_QUEUE, error);
		} else {
			rte_flow_error_set(error, ENOTSUP,
					   RTE_FLOW_ERROR_TYPE_ACTION,
					   actions, "unsupported action");
			goto exit;
		}
	}
	priv_unlock(priv);
	return rte_flow;
error_no_memory:
	rte_flow_error_set(error, ENOMEM,
			   RTE_FLOW_ERROR_TYPE_ITEM,
			   items,
			   "cannot allocate memory");
exit:
	priv_unlock(priv);
	return NULL;
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
	claim_zero(ibv_exp_destroy_flow(flow->ibv_flow));
	if (flow->qp)
		claim_zero(ibv_destroy_qp(flow->qp));
	if (flow->ind_table)
		claim_zero(
			ibv_exp_destroy_rwq_ind_table(
				flow->ind_table));
	if (flow->drop && flow->wq)
		claim_zero(ibv_exp_destroy_wq(flow->wq));
	if (flow->drop && flow->cq)
		claim_zero(ibv_destroy_cq(flow->cq));
	rte_free(flow->ibv_attr);
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
