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

#ifndef _SCHEDULER_PMD_PRIVATE_H
#define _SCHEDULER_PMD_PRIVATE_H

#include <rte_hash.h>
#include <rte_reorder.h>
#include <rte_cryptodev_scheduler.h>

/**< Maximum number of bonded devices per devices */
#ifndef MAX_SLAVES_NUM
#define MAX_SLAVES_NUM				(8)
#endif

#define PER_SLAVE_BUFF_SIZE			(256)

#define CS_LOG_ERR(fmt, args...)					\
	RTE_LOG(ERR, CRYPTODEV, "[%s] %s() line %u: " fmt "\n",		\
		RTE_STR(CRYPTODEV_NAME_SCHEDULER_PMD),			\
		__func__, __LINE__, ## args)

#ifdef RTE_LIBRTE_CRYPTO_SCHEDULER_DEBUG
#define CS_LOG_INFO(fmt, args...)					\
	RTE_LOG(INFO, CRYPTODEV, "[%s] %s() line %u: " fmt "\n",	\
		RTE_STR(CRYPTODEV_NAME_SCHEDULER_PMD),			\
		__func__, __LINE__, ## args)

#define CS_LOG_DBG(fmt, args...)					\
	RTE_LOG(DEBUG, CRYPTODEV, "[%s] %s() line %u: " fmt "\n",	\
		RTE_STR(CRYPTODEV_NAME_SCHEDULER_PMD),			\
		__func__, __LINE__, ## args)
#else
#define CS_LOG_INFO(fmt, args...)
#define CS_LOG_DBG(fmt, args...)
#endif

struct scheduler_slave {
	uint8_t dev_id;
	uint16_t qp_id;
	uint32_t nb_inflight_cops;

	enum rte_cryptodev_type dev_type;
};

struct scheduler_ctx {
	void *private_ctx;
	/**< private scheduler context pointer */

	struct rte_cryptodev_capabilities *capabilities;
	uint32_t nb_capabilities;

	uint32_t max_nb_queue_pairs;

	struct scheduler_slave slaves[MAX_SLAVES_NUM];
	uint32_t nb_slaves;

	enum rte_cryptodev_scheduler_mode mode;

	struct rte_cryptodev_scheduler_ops ops;

	uint8_t reordering_enabled;

	char name[RTE_CRYPTODEV_SCHEDULER_NAME_MAX_LEN];
	char description[RTE_CRYPTODEV_SCHEDULER_DESC_MAX_LEN];
} __rte_cache_aligned;

struct scheduler_qp_ctx {
	void *private_qp_ctx;

	rte_cryptodev_scheduler_burst_enqueue_t schedule_enqueue;
	rte_cryptodev_scheduler_burst_dequeue_t schedule_dequeue;

	struct rte_reorder_buffer *reorder_buf;
	uint32_t nb_empty_bufs;
	uint32_t seqn;
} __rte_cache_aligned;

struct scheduler_session {
	struct rte_cryptodev_sym_session *sessions[MAX_SLAVES_NUM];
};

/** device specific operations function pointer structure */
extern struct rte_cryptodev_ops *rte_crypto_scheduler_pmd_ops;

static inline void
scheduler_reorder_prepare(void *qp, struct rte_crypto_op **ops,
		uint16_t nb_ops)
{
	struct scheduler_qp_ctx *qp_ctx = (struct scheduler_qp_ctx *)qp;
	uint16_t i;

	if (unlikely(nb_ops == 0))
		return;

	for (i = 0; i < nb_ops && i < 4; i++)
		rte_prefetch0(ops[i]->sym->m_src);

	for (i = 0; (i < (nb_ops - 8)) && (nb_ops > 8); i += 4) {
		rte_prefetch0(ops[i + 4]->sym->m_src);
		rte_prefetch0(ops[i + 5]->sym->m_src);
		rte_prefetch0(ops[i + 6]->sym->m_src);
		rte_prefetch0(ops[i + 7]->sym->m_src);

		ops[i]->sym->m_src->seqn = qp_ctx->seqn++;
		ops[i + 1]->sym->m_src->seqn = qp_ctx->seqn++;
		ops[i + 2]->sym->m_src->seqn = qp_ctx->seqn++;
		ops[i + 3]->sym->m_src->seqn = qp_ctx->seqn++;
	}

	for (; i < nb_ops; i++)
		ops[i]->sym->m_src->seqn = qp_ctx->seqn++;
}

static inline void
scheduler_reorder_revert(void *qp, uint16_t nb_revert_ops)
{
	struct scheduler_qp_ctx *qp_ctx = (struct scheduler_qp_ctx *)qp;

	qp_ctx->seqn -= nb_revert_ops;
}

static inline uint16_t
scheduler_reorder_drain(void *qp, struct rte_crypto_op **ops,
		uint16_t nb_ops, uint16_t nb_drain_ops)
{
	struct scheduler_qp_ctx *qp_ctx = (struct scheduler_qp_ctx *)qp;
	struct rte_reorder_buffer *reorder_buff = qp_ctx->reorder_buf;
	struct rte_mbuf *mbuf0, *mbuf1, *mbuf2, *mbuf3;
	struct rte_mbuf *reorder_mbufs[nb_ops];
	uint16_t nb_drained_mbufs, i;

	for (i = 0; i < nb_ops && i < 4; i++)
		rte_prefetch0(ops[i]->sym->m_src);

	for (i = 0; (i < (nb_ops - 8)) && (nb_ops > 8);
			i += 4) {
		rte_prefetch0(ops[i + 4]->sym->m_src);
		rte_prefetch0(ops[i + 5]->sym->m_src);
		rte_prefetch0(ops[i + 6]->sym->m_src);
		rte_prefetch0(ops[i + 7]->sym->m_src);

		mbuf0 = ops[i]->sym->m_src;
		mbuf1 = ops[i + 1]->sym->m_src;
		mbuf2 = ops[i + 2]->sym->m_src;
		mbuf3 = ops[i + 3]->sym->m_src;

		mbuf0->userdata = ops[i];
		mbuf1->userdata = ops[i + 1];
		mbuf2->userdata = ops[i + 2];
		mbuf3->userdata = ops[i + 3];

		rte_reorder_insert(reorder_buff, mbuf0);
		rte_reorder_insert(reorder_buff, mbuf1);
		rte_reorder_insert(reorder_buff, mbuf2);
		rte_reorder_insert(reorder_buff, mbuf3);
	}

	for (; i < nb_ops; i++) {
		mbuf0 = ops[i]->sym->m_src;
		mbuf0->userdata = ops[i];
		rte_reorder_insert(reorder_buff, mbuf0);
	}

	nb_drained_mbufs = rte_reorder_drain(reorder_buff, reorder_mbufs,
			nb_drain_ops);
	for (i = 0; i < nb_drained_mbufs && i < 4; i++)
		rte_prefetch0(reorder_mbufs[i]);

	for (i = 0; (i < (nb_drained_mbufs - 8)) && (nb_drained_mbufs > 8);
			i += 4) {
		ops[i] = *(struct rte_crypto_op **)
				reorder_mbufs[i]->userdata;
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

	qp_ctx->nb_empty_bufs -= (nb_ops - nb_drained_mbufs);

	return nb_drained_mbufs;
}

#endif /* _SCHEDULER_PMD_PRIVATE_H */
