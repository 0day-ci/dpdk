/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
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

#include <rte_cryptodev.h>
#include <rte_malloc.h>

#include "rte_cryptodev_scheduler_operations.h"
#include "scheduler_pmd_private.h"

#define NB_PKT_SIZE_SLAVES			(2)
#define PKT_SIZE_THRESHOLD			(0xff80)

struct psd_scheduler_qp_ctx {
	struct scheduler_slave slaves[NB_PKT_SIZE_SLAVES];
	uint8_t last_deq_index;
	uint16_t threshold;
} __rte_cache_aligned;

static uint16_t
schedule_enqueue(void *qp, struct rte_crypto_op **ops, uint16_t nb_ops)
{
	struct psd_scheduler_qp_ctx *qp_ctx =
			((struct scheduler_qp_ctx *)qp)->private_qp_ctx;
	struct rte_crypto_op *sched_ops[NB_PKT_SIZE_SLAVES][nb_ops];
	struct scheduler_slave *slave;
	uint16_t nb_sched_ops[NB_PKT_SIZE_SLAVES] = {0};
	uint16_t i, j, processed_ops, total_ops = 0;
	struct rte_cryptodev_sym_session *sessions[NB_PKT_SIZE_SLAVES][nb_ops];
	struct scheduler_session *sess0, *sess1, *sess2, *sess3;
	struct rte_mbuf *mbuf0, *mbuf1, *mbuf2, *mbuf3;
	uint8_t index0, index1, index2, index3;

	if (unlikely(nb_ops == 0))
		return 0;

	for (i = 0; i < nb_ops && i < 4; i++) {
		rte_prefetch0(ops[i]->sym->session);
		rte_prefetch0(ops[i]->sym->m_src);
	}

	for (i = 0; (i < (nb_ops - 8)) && (nb_ops > 8); i += 4) {
		rte_prefetch0(ops[i + 4]->sym->session);
		rte_prefetch0(ops[i + 5]->sym->session);
		rte_prefetch0(ops[i + 6]->sym->session);
		rte_prefetch0(ops[i + 7]->sym->session);
		rte_prefetch0(ops[i + 4]->sym->m_src);
		rte_prefetch0(ops[i + 5]->sym->m_src);
		rte_prefetch0(ops[i + 6]->sym->m_src);
		rte_prefetch0(ops[i + 7]->sym->m_src);

		sess0 = (struct scheduler_session *)
				ops[i]->sym->session->_private;
		sess1 = (struct scheduler_session *)
				ops[i+1]->sym->session->_private;
		sess2 = (struct scheduler_session *)
				ops[i+2]->sym->session->_private;
		sess3 = (struct scheduler_session *)
				ops[i+3]->sym->session->_private;

		mbuf0 = ops[i]->sym->m_src;
		mbuf1 = ops[i + 1]->sym->m_src;
		mbuf2 = ops[i + 2]->sym->m_src;
		mbuf3 = ops[i + 3]->sym->m_src;

		index0 = mbuf0->data_len & qp_ctx->threshold ? 0 : 1;
		index1 = mbuf1->data_len & qp_ctx->threshold ? 0 : 1;
		index2 = mbuf2->data_len & qp_ctx->threshold ? 0 : 1;
		index3 = mbuf3->data_len & qp_ctx->threshold ? 0 : 1;

		sched_ops[index0][nb_sched_ops[index0]] = ops[i];
		sessions[index0][nb_sched_ops[index0]++] =
				ops[i]->sym->session;
		sched_ops[index1][nb_sched_ops[index1]] = ops[i + 1];
		sessions[index1][nb_sched_ops[index1]++] =
				ops[i + 1]->sym->session;
		sched_ops[index2][nb_sched_ops[index2]] = ops[i + 2];
		sessions[index2][nb_sched_ops[index2]++] =
				ops[i + 2]->sym->session;
		sched_ops[index3][nb_sched_ops[index3]] = ops[i + 3];
		sessions[index3][nb_sched_ops[index3]++] =
				ops[i + 3]->sym->session;

		ops[i]->sym->session = sess0->sessions[index0];
		ops[i + 1]->sym->session = sess1->sessions[index1];
		ops[i + 2]->sym->session = sess2->sessions[index2];
		ops[i + 3]->sym->session = sess3->sessions[index3];
	}

	for (; i < nb_ops; i++) {
		sess0 = (struct scheduler_session *)
				ops[i]->sym->session->_private;
		mbuf0 = ops[i]->sym->m_src;
		index0 = mbuf0->data_len & qp_ctx->threshold ? 0 : 1;
		sched_ops[index0][nb_sched_ops[index0]] = ops[i];
		sessions[index0][nb_sched_ops[index0]++] =
				ops[i]->sym->session;
		ops[i]->sym->session = sess0->sessions[index0];
	}

	for (i = 0; i < NB_PKT_SIZE_SLAVES; i++) {
		slave = &qp_ctx->slaves[i];
		processed_ops = rte_cryptodev_enqueue_burst(slave->dev_id,
				slave->qp_id, sched_ops[i], nb_sched_ops[i]);
		slave->nb_inflight_cops += processed_ops;

		if (unlikely(processed_ops < nb_sched_ops[i]))
			for (j = processed_ops; j < nb_sched_ops[i]; j++)
				sched_ops[i][j]->sym->session = sessions[i][j];
		total_ops += processed_ops;
	}

	return total_ops;
}

static uint16_t
schedule_enqueue_ordering(void *qp, struct rte_crypto_op **ops,
		uint16_t nb_ops)
{
	struct scheduler_qp_ctx *qp_ctx = qp;
	struct psd_scheduler_qp_ctx *ps_qp_ctx = qp_ctx->private_qp_ctx;
	struct rte_crypto_op *sched_ops[NB_PKT_SIZE_SLAVES][nb_ops];
	struct scheduler_slave *slave;
	uint16_t nb_sched_ops[NB_PKT_SIZE_SLAVES] = {0};
	uint16_t i, j, processed_ops, total_ops = 0;
	struct rte_cryptodev_sym_session *sessions[NB_PKT_SIZE_SLAVES][nb_ops];
	struct scheduler_session *sess0, *sess1, *sess2, *sess3;
	struct rte_mbuf *mbuf0, *mbuf1, *mbuf2, *mbuf3;
	uint8_t index0, index1, index2, index3;

	if (unlikely(nb_ops == 0))
		return 0;

	for (i = 0; i < nb_ops && i < 4; i++) {
		rte_prefetch0(ops[i]->sym->session);
		rte_prefetch0(ops[i]->sym->m_src);
	}

	for (i = 0; (i < (nb_ops - 8)) && (nb_ops > 8); i += 4) {
		rte_prefetch0(ops[i + 4]->sym->session);
		rte_prefetch0(ops[i + 5]->sym->session);
		rte_prefetch0(ops[i + 6]->sym->session);
		rte_prefetch0(ops[i + 7]->sym->session);
		rte_prefetch0(ops[i + 4]->sym->m_src);
		rte_prefetch0(ops[i + 5]->sym->m_src);
		rte_prefetch0(ops[i + 6]->sym->m_src);
		rte_prefetch0(ops[i + 7]->sym->m_src);

		sess0 = (struct scheduler_session *)
				ops[i]->sym->session->_private;
		sess1 = (struct scheduler_session *)
				ops[i+1]->sym->session->_private;
		sess2 = (struct scheduler_session *)
				ops[i+2]->sym->session->_private;
		sess3 = (struct scheduler_session *)
				ops[i+3]->sym->session->_private;

		mbuf0 = ops[i]->sym->m_src;
		mbuf1 = ops[i + 1]->sym->m_src;
		mbuf2 = ops[i + 2]->sym->m_src;
		mbuf3 = ops[i + 3]->sym->m_src;

		index0 = mbuf0->data_len & ps_qp_ctx->threshold ? 0 : 1;
		index1 = mbuf1->data_len & ps_qp_ctx->threshold ? 0 : 1;
		index2 = mbuf2->data_len & ps_qp_ctx->threshold ? 0 : 1;
		index3 = mbuf3->data_len & ps_qp_ctx->threshold ? 0 : 1;

		sched_ops[index0][nb_sched_ops[index0]] = ops[i];
		sessions[index0][nb_sched_ops[index0]++] =
				ops[i]->sym->session;
		mbuf0->seqn = qp_ctx->seqn++;
		sched_ops[index1][nb_sched_ops[index1]] = ops[i + 1];
		sessions[index1][nb_sched_ops[index1]++] =
				ops[i + 1]->sym->session;
		mbuf1->seqn = qp_ctx->seqn++;
		sched_ops[index2][nb_sched_ops[index2]] = ops[i + 2];
		sessions[index2][nb_sched_ops[index2]++] =
				ops[i + 2]->sym->session;
		mbuf2->seqn = qp_ctx->seqn++;
		sched_ops[index3][nb_sched_ops[index3]] = ops[i + 3];
		sessions[index3][nb_sched_ops[index3]++] =
				ops[i + 3]->sym->session;
		mbuf3->seqn = qp_ctx->seqn++;

		ops[i]->sym->session = sess0->sessions[index0];
		ops[i + 1]->sym->session = sess1->sessions[index1];
		ops[i + 2]->sym->session = sess2->sessions[index2];
		ops[i + 3]->sym->session = sess3->sessions[index3];
	}

	for (; i < nb_ops; i++) {
		sess0 = (struct scheduler_session *)
				ops[i]->sym->session->_private;
		mbuf0 = ops[i]->sym->m_src;
		index0 = mbuf0->data_len & ps_qp_ctx->threshold ? 0 : 1;
		sched_ops[index0][nb_sched_ops[index0]] = ops[i];
		sessions[index0][nb_sched_ops[index0]++] =
				ops[i]->sym->session;
		ops[i]->sym->session = sess0->sessions[index0];
	}

	for (i = 0; i < NB_PKT_SIZE_SLAVES; i++) {
		slave = &ps_qp_ctx->slaves[i];
		processed_ops = rte_cryptodev_enqueue_burst(slave->dev_id,
				slave->qp_id, sched_ops[i], nb_sched_ops[i]);
		slave->nb_inflight_cops += processed_ops;

		if (unlikely(processed_ops < nb_sched_ops[i]))
			for (j = processed_ops; j < nb_sched_ops[i]; j++)
				sched_ops[i][j]->sym->session = sessions[i][j];
		total_ops += processed_ops;
	}

	if (total_ops < nb_ops)
		qp_ctx->seqn -= nb_ops - total_ops;

	return total_ops;
}

static uint16_t
schedule_dequeue(void *qp, struct rte_crypto_op **ops, uint16_t nb_ops)
{
	struct psd_scheduler_qp_ctx *qp_ctx =
			((struct scheduler_qp_ctx *)qp)->private_qp_ctx;
	struct scheduler_slave *slave;
	uint16_t nb_deq_ops = 0, nb_2nd_deq_ops = 0;

	/* rotating between the 2 slaves */
	slave = &qp_ctx->slaves[qp_ctx->last_deq_index];
	if (slave->nb_inflight_cops) {
		nb_deq_ops = rte_cryptodev_dequeue_burst(slave->dev_id,
			slave->qp_id, ops, nb_ops);
		slave->nb_inflight_cops -= nb_deq_ops;
	}

	qp_ctx->last_deq_index = ~qp_ctx->last_deq_index & 0x01;

	if (nb_deq_ops < nb_ops) {
		slave = &qp_ctx->slaves[qp_ctx->last_deq_index];

		if (slave->nb_inflight_cops) {
			nb_2nd_deq_ops = rte_cryptodev_dequeue_burst(
					slave->dev_id, slave->qp_id,
					&ops[nb_deq_ops],
					nb_ops - nb_deq_ops);

			slave->nb_inflight_cops -= nb_2nd_deq_ops;
			nb_deq_ops += nb_2nd_deq_ops;
		}
	}

	return nb_deq_ops;
}

static uint16_t
schedule_dequeue_ordering(void *qp, struct rte_crypto_op **ops,
		uint16_t nb_ops)
{
	struct scheduler_qp_ctx *qp_ctx = (struct scheduler_qp_ctx *)qp;
	struct psd_scheduler_qp_ctx *ps_qp_ctx = qp_ctx->private_qp_ctx;
	struct scheduler_slave *slave;
	struct rte_reorder_buffer *reorder_buff = qp_ctx->reorder_buf;
	struct rte_mbuf *mbuf0, *mbuf1, *mbuf2, *mbuf3;
	uint16_t nb_deq_ops = 0, nb_2nd_deq_ops = 0, nb_drained_mbufs;
	const uint16_t nb_op_ops = nb_ops;
	struct rte_crypto_op *op_ops[nb_op_ops];
	struct rte_mbuf *reorder_mbufs[nb_op_ops];
	uint16_t i;

	/* check which slave has more inflight ops */
	slave = &ps_qp_ctx->slaves[ps_qp_ctx->last_deq_index];
	if (!slave->nb_inflight_cops) {
		nb_deq_ops = rte_cryptodev_dequeue_burst(slave->dev_id,
			slave->qp_id, ops, nb_ops);
		slave->nb_inflight_cops -= nb_deq_ops;
	}

	ps_qp_ctx->last_deq_index = ~ps_qp_ctx->last_deq_index & 0x01;

	if (nb_deq_ops < nb_ops) {
		slave = &ps_qp_ctx->slaves[ps_qp_ctx->last_deq_index];

		if (slave->nb_inflight_cops) {
			nb_2nd_deq_ops = rte_cryptodev_dequeue_burst(
					slave->dev_id, slave->qp_id,
					&ops[nb_deq_ops],
					nb_ops - nb_deq_ops);

			slave->nb_inflight_cops -= nb_2nd_deq_ops;
			nb_deq_ops += nb_2nd_deq_ops;
		}
	}

	for (i = 0; i < nb_deq_ops && i < 4; i++)
		rte_prefetch0(op_ops[i]->sym->m_src);

	for (i = 0; (i < (nb_deq_ops - 8)) && (nb_deq_ops > 8); i += 4) {
		mbuf0 = op_ops[i]->sym->m_src;
		mbuf1 = op_ops[i + 1]->sym->m_src;
		mbuf2 = op_ops[i + 2]->sym->m_src;
		mbuf3 = op_ops[i + 3]->sym->m_src;

		mbuf0->userdata = op_ops[i];
		mbuf1->userdata = op_ops[i + 1];
		mbuf2->userdata = op_ops[i + 2];
		mbuf3->userdata = op_ops[i + 3];

		rte_reorder_insert(reorder_buff, mbuf0);
		rte_reorder_insert(reorder_buff, mbuf1);
		rte_reorder_insert(reorder_buff, mbuf2);
		rte_reorder_insert(reorder_buff, mbuf3);

		rte_prefetch0(op_ops[i + 4]->sym->m_src);
		rte_prefetch0(op_ops[i + 5]->sym->m_src);
		rte_prefetch0(op_ops[i + 6]->sym->m_src);
		rte_prefetch0(op_ops[i + 7]->sym->m_src);
	}

	for (; i < nb_deq_ops; i++) {
		mbuf0 = op_ops[i]->sym->m_src;
		mbuf0->userdata = op_ops[i];
		rte_reorder_insert(reorder_buff, mbuf0);
	}

	nb_drained_mbufs = rte_reorder_drain(reorder_buff, reorder_mbufs,
			nb_ops);
	for (i = 0; i < nb_drained_mbufs && i < 4; i++)
		rte_prefetch0(reorder_mbufs[i]);

	for (i = 0; (i < (nb_drained_mbufs - 8)) && (nb_drained_mbufs > 8);
			i += 4) {
		ops[i] = *(struct rte_crypto_op **)reorder_mbufs[i]->userdata;
		ops[i + 1] = *(struct rte_crypto_op **)
			reorder_mbufs[i + 1]->userdata;
		ops[i + 2] = *(struct rte_crypto_op **)
			reorder_mbufs[i + 2]->userdata;
		ops[i + 3] = *(struct rte_crypto_op **)
			reorder_mbufs[i + 3]->userdata;

		reorder_mbufs[i]->userdata = NULL;
		reorder_mbufs[i + 1]->userdata = NULL;
		reorder_mbufs[i + 2]->userdata = NULL;
		reorder_mbufs[i + 3]->userdata = NULL;

		rte_prefetch0(reorder_mbufs[i + 4]);
		rte_prefetch0(reorder_mbufs[i + 5]);
		rte_prefetch0(reorder_mbufs[i + 6]);
		rte_prefetch0(reorder_mbufs[i + 7]);
	}

	for (; i < nb_drained_mbufs; i++) {
		ops[i] = *(struct rte_crypto_op **)
			reorder_mbufs[i]->userdata;
		reorder_mbufs[i]->userdata = NULL;
	}

	return nb_drained_mbufs;
}

static int
slave_attach(__rte_unused struct rte_cryptodev *dev,
		__rte_unused uint8_t slave_id)
{
	return 0;
}

static int
slave_detach(__rte_unused struct rte_cryptodev *dev,
		__rte_unused uint8_t slave_id)
{
	return 0;
}

static int
scheduler_start(struct rte_cryptodev *dev)
{
	struct scheduler_ctx *sched_ctx = dev->data->dev_private;
	uint16_t i;

	/* for packet size based scheduler, nb_slaves have to >= 2 */
	if (sched_ctx->nb_slaves < NB_PKT_SIZE_SLAVES)
		return -1;

	for (i = 0; i < dev->data->nb_queue_pairs; i++) {
		struct scheduler_qp_ctx *qp_ctx = dev->data->queue_pairs[i];
		struct psd_scheduler_qp_ctx *ps_qp_ctx =
				qp_ctx->private_qp_ctx;
		uint32_t j;

		memset(ps_qp_ctx->slaves, 0, NB_PKT_SIZE_SLAVES *
				sizeof(struct scheduler_slave));
		for (j = 0; j < NB_PKT_SIZE_SLAVES; j++) {
			ps_qp_ctx->slaves[j].dev_id =
					sched_ctx->slaves[j].dev_id;
			ps_qp_ctx->slaves[j].qp_id = i;
		}

		ps_qp_ctx->threshold = PKT_SIZE_THRESHOLD;
	}

	if (sched_ctx->reordering_enabled) {
		dev->enqueue_burst = &schedule_enqueue_ordering;
		dev->dequeue_burst = &schedule_dequeue_ordering;
	} else {
		dev->enqueue_burst = &schedule_enqueue;
		dev->dequeue_burst = &schedule_dequeue;
	}

	return 0;
}

static int
scheduler_stop(__rte_unused struct rte_cryptodev *dev)
{
	return 0;
}

static int
scheduler_config_qp(struct rte_cryptodev *dev, uint16_t qp_id)
{
	struct scheduler_qp_ctx *qp_ctx = dev->data->queue_pairs[qp_id];
	struct psd_scheduler_qp_ctx *ps_qp_ctx;

	ps_qp_ctx = rte_zmalloc_socket(NULL, sizeof(*ps_qp_ctx), 0,
			rte_socket_id());
	if (!ps_qp_ctx) {
		CS_LOG_ERR("failed allocate memory for private queue pair");
		return -ENOMEM;
	}

	qp_ctx->private_qp_ctx = (void *)ps_qp_ctx;

	return 0;
}

static int
scheduler_create_private_ctx(__rte_unused struct rte_cryptodev *dev)
{
	return 0;
}

struct rte_cryptodev_scheduler_ops scheduler_ps_ops = {
	slave_attach,
	slave_detach,
	scheduler_start,
	scheduler_stop,
	scheduler_config_qp,
	scheduler_create_private_ctx
};

struct rte_cryptodev_scheduler psd_scheduler = {
		.name = "packet-size-based-scheduler",
		.description = "scheduler which will distribute crypto op "
				"burst based on the packet size",
		.mode = CDEV_SCHED_MODE_PKT_SIZE_DISTRBUTE,
		.ops = &scheduler_ps_ops
};

struct rte_cryptodev_scheduler *packet_size_based_scheduler = &psd_scheduler;
