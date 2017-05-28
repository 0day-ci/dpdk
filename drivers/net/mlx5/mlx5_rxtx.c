/*-
 *   BSD LICENSE
 *
 *   Copyright 2015 6WIND S.A.
 *   Copyright 2015 Mellanox.
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

#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* Verbs header. */
/* ISO C doesn't support unnamed structs/unions, disabling -pedantic. */
#ifdef PEDANTIC
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <infiniband/verbs.h>
#include <infiniband/mlx5_hw.h>
#include <infiniband/arch.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-Wpedantic"
#endif

/* DPDK headers don't like -pedantic. */
#ifdef PEDANTIC
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <rte_mbuf.h>
#include <rte_mempool.h>
#include <rte_prefetch.h>
#include <rte_common.h>
#include <rte_branch_prediction.h>
#include <rte_ether.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-Wpedantic"
#endif

#include "mlx5.h"
#include "mlx5_utils.h"
#include "mlx5_rxtx.h"
#include "mlx5_autoconf.h"
#include "mlx5_defs.h"
#include "mlx5_prm.h"

static inline int
check_cqe(volatile struct mlx5_cqe *cqe,
	  unsigned int cqes_n, const uint16_t ci)
	  __attribute__((always_inline));

static inline void
txq_complete(struct txq *txq) __attribute__((always_inline));

static inline uint32_t
txq_mp2mr(struct txq *txq, struct rte_mempool *mp)
	__attribute__((always_inline));

static inline void
mlx5_tx_dbrec(struct txq *txq, volatile struct mlx5_wqe *wqe)
	__attribute__((always_inline));

static inline uint32_t
rxq_cq_to_pkt_type(volatile struct mlx5_cqe *cqe)
	__attribute__((always_inline));

static inline int
mlx5_rx_poll_len(struct rxq *rxq, volatile struct mlx5_cqe *cqe,
		 uint16_t cqe_cnt, uint32_t *rss_hash)
		 __attribute__((always_inline));

static inline uint32_t
rxq_cq_to_ol_flags(struct rxq *rxq, volatile struct mlx5_cqe *cqe)
		   __attribute__((always_inline));

#ifndef NDEBUG

/**
 * Verify or set magic value in CQE.
 *
 * @param cqe
 *   Pointer to CQE.
 *
 * @return
 *   0 the first time.
 */
static inline int
check_cqe_seen(volatile struct mlx5_cqe *cqe)
{
	static const uint8_t magic[] = "seen";
	volatile uint8_t (*buf)[sizeof(cqe->rsvd0)] = &cqe->rsvd0;
	int ret = 1;
	unsigned int i;

	for (i = 0; i < sizeof(magic) && i < sizeof(*buf); ++i)
		if (!ret || (*buf)[i] != magic[i]) {
			ret = 0;
			(*buf)[i] = magic[i];
		}
	return ret;
}

#endif /* NDEBUG */

/**
 * Check whether CQE is valid.
 *
 * @param cqe
 *   Pointer to CQE.
 * @param cqes_n
 *   Size of completion queue.
 * @param ci
 *   Consumer index.
 *
 * @return
 *   0 on success, 1 on failure.
 */
static inline int
check_cqe(volatile struct mlx5_cqe *cqe,
	  unsigned int cqes_n, const uint16_t ci)
{
	uint16_t idx = ci & cqes_n;
	uint8_t op_own = cqe->op_own;
	uint8_t op_owner = MLX5_CQE_OWNER(op_own);
	uint8_t op_code = MLX5_CQE_OPCODE(op_own);

	if (unlikely((op_owner != (!!(idx))) || (op_code == MLX5_CQE_INVALID)))
		return 1; /* No CQE. */
#ifndef NDEBUG
	if ((op_code == MLX5_CQE_RESP_ERR) ||
	    (op_code == MLX5_CQE_REQ_ERR)) {
		volatile struct mlx5_err_cqe *err_cqe = (volatile void *)cqe;
		uint8_t syndrome = err_cqe->syndrome;

		if ((syndrome == MLX5_CQE_SYNDROME_LOCAL_LENGTH_ERR) ||
		    (syndrome == MLX5_CQE_SYNDROME_REMOTE_ABORTED_ERR))
			return 0;
		if (!check_cqe_seen(cqe))
			ERROR("unexpected CQE error %u (0x%02x)"
			      " syndrome 0x%02x",
			      op_code, op_code, syndrome);
		return 1;
	} else if ((op_code != MLX5_CQE_RESP_SEND) &&
		   (op_code != MLX5_CQE_REQ)) {
		if (!check_cqe_seen(cqe))
			ERROR("unexpected CQE opcode %u (0x%02x)",
			      op_code, op_code);
		return 1;
	}
#endif /* NDEBUG */
	return 0;
}

/**
 * Return the address of the WQE.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param  wqe_ci
 *   WQE consumer index.
 *
 * @return
 *   WQE address.
 */
static inline uintptr_t *
tx_mlx5_wqe(struct txq *txq, uint16_t ci)
{
	ci &= ((1 << txq->wqe_n) - 1);
	return (uintptr_t *)((uintptr_t)txq->wqes + ci * MLX5_WQE_SIZE);
}

/**
 * Return the size of tailroom of WQ.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param addr
 *   Pointer to tail of WQ.
 *
 * @return
 *   Size of tailroom.
 */
static inline size_t
tx_mlx5_wq_tailroom(struct txq *txq, void *addr)
{
	size_t tailroom;
	tailroom = (uintptr_t)(txq->wqes) +
		   (1 << txq->wqe_n) * MLX5_WQE_SIZE -
		   (uintptr_t)addr;
	return tailroom;
}

/**
 * Copy data to tailroom of circular queue.
 *
 * @param dst
 *   Pointer to destination.
 * @param src
 *   Pointer to source.
 * @param n
 *   Number of bytes to copy.
 * @param base
 *   Pointer to head of queue.
 * @param tailroom
 *   Size of tailroom from dst.
 *
 * @return
 *   Pointer after copied data.
 */
static inline void *
mlx5_copy_to_wq(void *dst, const void *src, size_t n,
		void *base, size_t tailroom)
{
	void *ret;

	if (n > tailroom) {
		rte_memcpy(dst, src, tailroom);
		rte_memcpy(base, (void *)((uintptr_t)src + tailroom),
			   n - tailroom);
		ret = (uint8_t *)base + n - tailroom;
	} else {
		rte_memcpy(dst, src, n);
		ret = (n == tailroom) ? base : (uint8_t *)dst + n;
	}
	return ret;
}

/**
 * Manage TX completions.
 *
 * When sending a burst, mlx5_tx_burst() posts several WRs.
 *
 * @param txq
 *   Pointer to TX queue structure.
 */
static inline void
txq_complete(struct txq *txq)
{
	const unsigned int elts_n = 1 << txq->elts_n;
	const unsigned int cqe_n = 1 << txq->cqe_n;
	const unsigned int cqe_cnt = cqe_n - 1;
	uint16_t elts_free = txq->elts_tail;
	uint16_t elts_tail;
	uint16_t cq_ci = txq->cq_ci;
	volatile struct mlx5_cqe *cqe = NULL;
	volatile struct mlx5_wqe_ctrl *ctrl;

	do {
		volatile struct mlx5_cqe *tmp;

		tmp = &(*txq->cqes)[cq_ci & cqe_cnt];
		if (check_cqe(tmp, cqe_n, cq_ci))
			break;
		cqe = tmp;
#ifndef NDEBUG
		if (MLX5_CQE_FORMAT(cqe->op_own) == MLX5_COMPRESSED) {
			if (!check_cqe_seen(cqe))
				ERROR("unexpected compressed CQE, TX stopped");
			return;
		}
		if ((MLX5_CQE_OPCODE(cqe->op_own) == MLX5_CQE_RESP_ERR) ||
		    (MLX5_CQE_OPCODE(cqe->op_own) == MLX5_CQE_REQ_ERR)) {
			if (!check_cqe_seen(cqe))
				ERROR("unexpected error CQE, TX stopped");
			return;
		}
#endif /* NDEBUG */
		++cq_ci;
	} while (1);
	if (unlikely(cqe == NULL))
		return;
	txq->wqe_pi = ntohs(cqe->wqe_counter);
	ctrl = (volatile struct mlx5_wqe_ctrl *)
		tx_mlx5_wqe(txq, txq->wqe_pi);
	elts_tail = ctrl->ctrl3;
	assert(elts_tail < (1 << txq->wqe_n));
	/* Free buffers. */
	while (elts_free != elts_tail) {
		struct rte_mbuf *elt = (*txq->elts)[elts_free];
		unsigned int elts_free_next =
			(elts_free + 1) & (elts_n - 1);
		struct rte_mbuf *elt_next = (*txq->elts)[elts_free_next];

#ifndef NDEBUG
		/* Poisoning. */
		memset(&(*txq->elts)[elts_free],
		       0x66,
		       sizeof((*txq->elts)[elts_free]));
#endif
		RTE_MBUF_PREFETCH_TO_FREE(elt_next);
		/* Only one segment needs to be freed. */
		rte_pktmbuf_free_seg(elt);
		elts_free = elts_free_next;
	}
	txq->cq_ci = cq_ci;
	txq->elts_tail = elts_tail;
	/* Update the consumer index. */
	rte_wmb();
	*txq->cq_db = htonl(cq_ci);
}

/**
 * Get Memory Pool (MP) from mbuf. If mbuf is indirect, the pool from which
 * the cloned mbuf is allocated is returned instead.
 *
 * @param buf
 *   Pointer to mbuf.
 *
 * @return
 *   Memory pool where data is located for given mbuf.
 */
static struct rte_mempool *
txq_mb2mp(struct rte_mbuf *buf)
{
	if (unlikely(RTE_MBUF_INDIRECT(buf)))
		return rte_mbuf_from_indirect(buf)->pool;
	return buf->pool;
}

/**
 * Get Memory Region (MR) <-> Memory Pool (MP) association from txq->mp2mr[].
 * Add MP to txq->mp2mr[] if it's not registered yet. If mp2mr[] is full,
 * remove an entry first.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param[in] mp
 *   Memory Pool for which a Memory Region lkey must be returned.
 *
 * @return
 *   mr->lkey on success, (uint32_t)-1 on failure.
 */
static inline uint32_t
txq_mp2mr(struct txq *txq, struct rte_mempool *mp)
{
	unsigned int i;
	uint32_t lkey = (uint32_t)-1;

	for (i = 0; (i != RTE_DIM(txq->mp2mr)); ++i) {
		if (unlikely(txq->mp2mr[i].mp == NULL)) {
			/* Unknown MP, add a new MR for it. */
			break;
		}
		if (txq->mp2mr[i].mp == mp) {
			assert(txq->mp2mr[i].lkey != (uint32_t)-1);
			assert(htonl(txq->mp2mr[i].mr->lkey) ==
			       txq->mp2mr[i].lkey);
			lkey = txq->mp2mr[i].lkey;
			break;
		}
	}
	if (unlikely(lkey == (uint32_t)-1))
		lkey = txq_mp2mr_reg(txq, mp, i);
	return lkey;
}

/**
 * Ring TX queue doorbell.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param wqe
 *   Pointer to the last WQE posted in the NIC.
 */
static inline void
mlx5_tx_dbrec(struct txq *txq, volatile struct mlx5_wqe *wqe)
{
	uint64_t *dst = (uint64_t *)((uintptr_t)txq->bf_reg);
	volatile uint64_t *src = ((volatile uint64_t *)wqe);

	rte_wmb();
	*txq->qp_db = htonl(txq->wqe_ci);
	/* Ensure ordering between DB record and BF copy. */
	rte_wmb();
	*dst = *src;
}

/**
 * DPDK callback to check the status of a tx descriptor.
 *
 * @param tx_queue
 *   The tx queue.
 * @param[in] offset
 *   The index of the descriptor in the ring.
 *
 * @return
 *   The status of the tx descriptor.
 */
int
mlx5_tx_descriptor_status(void *tx_queue, uint16_t offset)
{
	struct txq *txq = tx_queue;
	const unsigned int elts_n = 1 << txq->elts_n;
	const unsigned int elts_cnt = elts_n - 1;
	unsigned int used;

	txq_complete(txq);
	used = (txq->elts_head - txq->elts_tail) & elts_cnt;
	if (offset < used)
		return RTE_ETH_TX_DESC_FULL;
	return RTE_ETH_TX_DESC_DONE;
}

/**
 * DPDK callback to check the status of a rx descriptor.
 *
 * @param rx_queue
 *   The rx queue.
 * @param[in] offset
 *   The index of the descriptor in the ring.
 *
 * @return
 *   The status of the tx descriptor.
 */
int
mlx5_rx_descriptor_status(void *rx_queue, uint16_t offset)
{
	struct rxq *rxq = rx_queue;
	struct rxq_zip *zip = &rxq->zip;
	volatile struct mlx5_cqe *cqe;
	const unsigned int cqe_n = (1 << rxq->cqe_n);
	const unsigned int cqe_cnt = cqe_n - 1;
	unsigned int cq_ci;
	unsigned int used;

	/* if we are processing a compressed cqe */
	if (zip->ai) {
		used = zip->cqe_cnt - zip->ca;
		cq_ci = zip->cq_ci;
	} else {
		used = 0;
		cq_ci = rxq->cq_ci;
	}
	cqe = &(*rxq->cqes)[cq_ci & cqe_cnt];
	while (check_cqe(cqe, cqe_n, cq_ci) == 0) {
		int8_t op_own;
		unsigned int n;

		op_own = cqe->op_own;
		if (MLX5_CQE_FORMAT(op_own) == MLX5_COMPRESSED)
			n = ntohl(cqe->byte_cnt);
		else
			n = 1;
		cq_ci += n;
		used += n;
		cqe = &(*rxq->cqes)[cq_ci & cqe_cnt];
	}
	used = RTE_MIN(used, (1U << rxq->elts_n) - 1);
	if (offset < used)
		return RTE_ETH_RX_DESC_DONE;
	return RTE_ETH_RX_DESC_AVAIL;
}

/**
 * DPDK callback for TX.
 *
 * @param dpdk_txq
 *   Generic pointer to TX queue structure.
 * @param[in] pkts
 *   Packets to transmit.
 * @param pkts_n
 *   Number of packets in array.
 *
 * @return
 *   Number of packets successfully transmitted (<= pkts_n).
 */
uint16_t
mlx5_tx_burst(void *dpdk_txq, struct rte_mbuf **pkts, uint16_t pkts_n)
{
	struct txq *txq = (struct txq *)dpdk_txq;
	uint16_t elts_head = txq->elts_head;
	const unsigned int elts_n = 1 << txq->elts_n;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int k = 0;
	unsigned int max;
	unsigned int max_inline = txq->max_inline;
	const unsigned int inline_en = !!max_inline && txq->inline_en;
	uint16_t max_wqe;
	unsigned int comp;
	volatile struct mlx5_wqe_v *wqe = NULL;
	volatile struct mlx5_wqe_ctrl *last_wqe = NULL;
	unsigned int segs_n = 0;
	struct rte_mbuf *buf = NULL;
	uint8_t *raw;

	if (unlikely(!pkts_n))
		return 0;
	/* Prefetch first packet cacheline. */
	rte_prefetch0(*pkts);
	/* Start processing. */
	txq_complete(txq);
	max = (elts_n - (elts_head - txq->elts_tail));
	if (max > elts_n)
		max -= elts_n;
	max_wqe = (1u << txq->wqe_n) - (txq->wqe_ci - txq->wqe_pi);
	if (unlikely(!max_wqe))
		return 0;
	do {
		volatile rte_v128u32_t *dseg = NULL;
		uint32_t length;
		unsigned int ds = 0;
		unsigned int sg = 0; /* counter of additional segs attached. */
		uintptr_t addr;
		uint64_t naddr;
		uint16_t pkt_inline_sz = MLX5_WQE_DWORD_SIZE + 2;
		uint16_t tso_header_sz = 0;
		uint16_t ehdr;
		uint8_t cs_flags = 0;
		uint64_t tso = 0;
#ifdef MLX5_PMD_SOFT_COUNTERS
		uint32_t total_length = 0;
#endif

		/* first_seg */
		buf = *pkts;
		segs_n = buf->nb_segs;
		/*
		 * Make sure there is enough room to store this packet and
		 * that one ring entry remains unused.
		 */
		assert(segs_n);
		if (max < segs_n + 1)
			break;
		max -= segs_n;
		--segs_n;
		if (unlikely(--max_wqe == 0))
			break;
		wqe = (volatile struct mlx5_wqe_v *)
			tx_mlx5_wqe(txq, txq->wqe_ci);
		rte_prefetch0(tx_mlx5_wqe(txq, txq->wqe_ci + 1));
		if (pkts_n - i > 1)
			rte_prefetch0(*(pkts + 1));
		addr = rte_pktmbuf_mtod(buf, uintptr_t);
		length = DATA_LEN(buf);
		ehdr = (((uint8_t *)addr)[1] << 8) |
		       ((uint8_t *)addr)[0];
#ifdef MLX5_PMD_SOFT_COUNTERS
		total_length = length;
#endif
		if (length < (MLX5_WQE_DWORD_SIZE + 2))
			break;
		/* Update element. */
		(*txq->elts)[elts_head] = buf;
		/* Prefetch next buffer data. */
		if (pkts_n - i > 1)
			rte_prefetch0(
			    rte_pktmbuf_mtod(*(pkts + 1), volatile void *));
		/* Should we enable HW CKSUM offload */
		if (buf->ol_flags &
		    (PKT_TX_IP_CKSUM | PKT_TX_TCP_CKSUM | PKT_TX_UDP_CKSUM)) {
			const uint64_t is_tunneled = buf->ol_flags &
						     (PKT_TX_TUNNEL_GRE |
						      PKT_TX_TUNNEL_VXLAN);

			if (is_tunneled && txq->tunnel_en) {
				cs_flags = MLX5_ETH_WQE_L3_INNER_CSUM |
					   MLX5_ETH_WQE_L4_INNER_CSUM;
				if (buf->ol_flags & PKT_TX_OUTER_IP_CKSUM)
					cs_flags |= MLX5_ETH_WQE_L3_CSUM;
			} else {
				cs_flags = MLX5_ETH_WQE_L3_CSUM |
					   MLX5_ETH_WQE_L4_CSUM;
			}
		}
		raw = ((uint8_t *)(uintptr_t)wqe) + 2 * MLX5_WQE_DWORD_SIZE;
		/* Replace the Ethernet type by the VLAN if necessary. */
		if (buf->ol_flags & PKT_TX_VLAN_PKT) {
			uint32_t vlan = htonl(0x81000000 | buf->vlan_tci);
			unsigned int len = 2 * ETHER_ADDR_LEN - 2;

			addr += 2;
			length -= 2;
			/* Copy Destination and source mac address. */
			memcpy((uint8_t *)raw, ((uint8_t *)addr), len);
			/* Copy VLAN. */
			memcpy((uint8_t *)raw + len, &vlan, sizeof(vlan));
			/* Copy missing two bytes to end the DSeg. */
			memcpy((uint8_t *)raw + len + sizeof(vlan),
			       ((uint8_t *)addr) + len, 2);
			addr += len + 2;
			length -= (len + 2);
		} else {
			memcpy((uint8_t *)raw, ((uint8_t *)addr) + 2,
			       MLX5_WQE_DWORD_SIZE);
			length -= pkt_inline_sz;
			addr += pkt_inline_sz;
		}
		if (txq->tso_en) {
			tso = buf->ol_flags & PKT_TX_TCP_SEG;
			if (tso) {
				uintptr_t end = (uintptr_t)
						(((uintptr_t)txq->wqes) +
						(1 << txq->wqe_n) *
						MLX5_WQE_SIZE);
				unsigned int copy_b;
				uint8_t vlan_sz = (buf->ol_flags &
						  PKT_TX_VLAN_PKT) ? 4 : 0;
				const uint64_t is_tunneled =
							buf->ol_flags &
							(PKT_TX_TUNNEL_GRE |
							 PKT_TX_TUNNEL_VXLAN);

				tso_header_sz = buf->l2_len + vlan_sz +
						buf->l3_len + buf->l4_len;

				if (is_tunneled	&& txq->tunnel_en) {
					tso_header_sz += buf->outer_l2_len +
							 buf->outer_l3_len;
					cs_flags |= MLX5_ETH_WQE_L4_INNER_CSUM;
				} else {
					cs_flags |= MLX5_ETH_WQE_L4_CSUM;
				}
				if (unlikely(tso_header_sz >
					     MLX5_MAX_TSO_HEADER))
					break;
				copy_b = tso_header_sz - pkt_inline_sz;
				/* First seg must contain all headers. */
				assert(copy_b <= length);
				raw += MLX5_WQE_DWORD_SIZE;
				if (copy_b &&
				   ((end - (uintptr_t)raw) > copy_b)) {
					uint16_t n = (MLX5_WQE_DS(copy_b) -
						      1 + 3) / 4;

					if (unlikely(max_wqe < n))
						break;
					max_wqe -= n;
					rte_memcpy((void *)raw,
						   (void *)addr, copy_b);
					addr += copy_b;
					length -= copy_b;
					pkt_inline_sz += copy_b;
					/*
					 * Another DWORD will be added
					 * in the inline part.
					 */
					raw += MLX5_WQE_DS(copy_b) *
					       MLX5_WQE_DWORD_SIZE -
					       MLX5_WQE_DWORD_SIZE;
				} else {
					/* NOP WQE. */
					wqe->ctrl = (rte_v128u32_t){
						     htonl(txq->wqe_ci << 8),
						     htonl(txq->qp_num_8s | 1),
						     0,
						     0,
					};
					ds = 1;
					total_length = 0;
					k++;
					goto next_wqe;
				}
			}
		}
		/* Inline if enough room. */
		if (inline_en || tso) {
			uintptr_t end = (uintptr_t)
				(((uintptr_t)txq->wqes) +
				 (1 << txq->wqe_n) * MLX5_WQE_SIZE);
			unsigned int inline_room = max_inline *
						   RTE_CACHE_LINE_SIZE -
						   (pkt_inline_sz - 2);
			uintptr_t addr_end = (addr + inline_room) &
					     ~(RTE_CACHE_LINE_SIZE - 1);
			unsigned int copy_b = (addr_end > addr) ?
				RTE_MIN((addr_end - addr), length) :
				0;

			raw += MLX5_WQE_DWORD_SIZE;
			if (copy_b && ((end - (uintptr_t)raw) > copy_b)) {
				/*
				 * One Dseg remains in the current WQE.  To
				 * keep the computation positive, it is
				 * removed after the bytes to Dseg conversion.
				 */
				uint16_t n = (MLX5_WQE_DS(copy_b) - 1 + 3) / 4;

				if (unlikely(max_wqe < n))
					break;
				max_wqe -= n;
				if (tso) {
					uint32_t inl =
						htonl(copy_b | MLX5_INLINE_SEG);

					pkt_inline_sz =
						MLX5_WQE_DS(tso_header_sz) *
						MLX5_WQE_DWORD_SIZE;
					rte_memcpy((void *)raw,
						   (void *)&inl, sizeof(inl));
					raw += sizeof(inl);
					pkt_inline_sz += sizeof(inl);
				}
				rte_memcpy((void *)raw, (void *)addr, copy_b);
				addr += copy_b;
				length -= copy_b;
				pkt_inline_sz += copy_b;
			}
			/*
			 * 2 DWORDs consumed by the WQE header + ETH segment +
			 * the size of the inline part of the packet.
			 */
			ds = 2 + MLX5_WQE_DS(pkt_inline_sz - 2);
			if (length > 0) {
				if (ds % (MLX5_WQE_SIZE /
					  MLX5_WQE_DWORD_SIZE) == 0) {
					if (unlikely(--max_wqe == 0))
						break;
					dseg = (volatile rte_v128u32_t *)
					       tx_mlx5_wqe(txq, txq->wqe_ci +
							   ds / 4);
				} else {
					dseg = (volatile rte_v128u32_t *)
						((uintptr_t)wqe +
						 (ds * MLX5_WQE_DWORD_SIZE));
				}
				goto use_dseg;
			} else if (!segs_n) {
				goto next_pkt;
			} else {
				/* dseg will be advance as part of next_seg */
				dseg = (volatile rte_v128u32_t *)
					((uintptr_t)wqe +
					 ((ds - 1) * MLX5_WQE_DWORD_SIZE));
				goto next_seg;
			}
		} else {
			/*
			 * No inline has been done in the packet, only the
			 * Ethernet Header as been stored.
			 */
			dseg = (volatile rte_v128u32_t *)
				((uintptr_t)wqe + (3 * MLX5_WQE_DWORD_SIZE));
			ds = 3;
use_dseg:
			/* Add the remaining packet as a simple ds. */
			naddr = htonll(addr);
			*dseg = (rte_v128u32_t){
				htonl(length),
				txq_mp2mr(txq, txq_mb2mp(buf)),
				naddr,
				naddr >> 32,
			};
			++ds;
			if (!segs_n)
				goto next_pkt;
		}
next_seg:
		assert(buf);
		assert(ds);
		assert(wqe);
		/*
		 * Spill on next WQE when the current one does not have
		 * enough room left. Size of WQE must a be a multiple
		 * of data segment size.
		 */
		assert(!(MLX5_WQE_SIZE % MLX5_WQE_DWORD_SIZE));
		if (!(ds % (MLX5_WQE_SIZE / MLX5_WQE_DWORD_SIZE))) {
			if (unlikely(--max_wqe == 0))
				break;
			dseg = (volatile rte_v128u32_t *)
			       tx_mlx5_wqe(txq, txq->wqe_ci + ds / 4);
			rte_prefetch0(tx_mlx5_wqe(txq,
						  txq->wqe_ci + ds / 4 + 1));
		} else {
			++dseg;
		}
		++ds;
		buf = buf->next;
		assert(buf);
		length = DATA_LEN(buf);
#ifdef MLX5_PMD_SOFT_COUNTERS
		total_length += length;
#endif
		/* Store segment information. */
		naddr = htonll(rte_pktmbuf_mtod(buf, uintptr_t));
		*dseg = (rte_v128u32_t){
			htonl(length),
			txq_mp2mr(txq, txq_mb2mp(buf)),
			naddr,
			naddr >> 32,
		};
		elts_head = (elts_head + 1) & (elts_n - 1);
		(*txq->elts)[elts_head] = buf;
		++sg;
		/* Advance counter only if all segs are successfully posted. */
		if (sg < segs_n)
			goto next_seg;
		else
			j += sg;
next_pkt:
		elts_head = (elts_head + 1) & (elts_n - 1);
		++pkts;
		++i;
		/* Initialize known and common part of the WQE structure. */
		if (tso) {
			wqe->ctrl = (rte_v128u32_t){
				htonl((txq->wqe_ci << 8) | MLX5_OPCODE_TSO),
				htonl(txq->qp_num_8s | ds),
				0,
				0,
			};
			wqe->eseg = (rte_v128u32_t){
				0,
				cs_flags | (htons(buf->tso_segsz) << 16),
				0,
				(ehdr << 16) | htons(tso_header_sz),
			};
		} else {
			wqe->ctrl = (rte_v128u32_t){
				htonl((txq->wqe_ci << 8) | MLX5_OPCODE_SEND),
				htonl(txq->qp_num_8s | ds),
				0,
				0,
			};
			wqe->eseg = (rte_v128u32_t){
				0,
				cs_flags,
				0,
				(ehdr << 16) | htons(pkt_inline_sz),
			};
		}
next_wqe:
		txq->wqe_ci += (ds + 3) / 4;
		/* Save the last successful WQE for completion request */
		last_wqe = (volatile struct mlx5_wqe_ctrl *)wqe;
#ifdef MLX5_PMD_SOFT_COUNTERS
		/* Increment sent bytes counter. */
		txq->stats.obytes += total_length;
#endif
	} while (i < pkts_n);
	/* Take a shortcut if nothing must be sent. */
	if (unlikely((i + k) == 0))
		return 0;
	txq->elts_head = (txq->elts_head + i + j) & (elts_n - 1);
	/* Check whether completion threshold has been reached. */
	comp = txq->elts_comp + i + j + k;
	if (comp >= MLX5_TX_COMP_THRESH) {
		/* Request completion on last WQE. */
		last_wqe->ctrl2 = htonl(8);
		/* Save elts_head in unused "immediate" field of WQE. */
		last_wqe->ctrl3 = txq->elts_head;
		txq->elts_comp = 0;
	} else {
		txq->elts_comp = comp;
	}
#ifdef MLX5_PMD_SOFT_COUNTERS
	/* Increment sent packets counter. */
	txq->stats.opackets += i;
#endif
	/* Ring QP doorbell. */
	mlx5_tx_dbrec(txq, (volatile struct mlx5_wqe *)last_wqe);
	return i;
}

/**
 * Open a MPW session.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param mpw
 *   Pointer to MPW session structure.
 * @param length
 *   Packet length.
 */
static inline void
mlx5_mpw_new(struct txq *txq, struct mlx5_mpw *mpw, uint32_t length)
{
	uint16_t idx = txq->wqe_ci & ((1 << txq->wqe_n) - 1);
	volatile struct mlx5_wqe_data_seg (*dseg)[MLX5_MPW_DSEG_MAX] =
		(volatile struct mlx5_wqe_data_seg (*)[])
		tx_mlx5_wqe(txq, idx + 1);

	mpw->state = MLX5_MPW_STATE_OPENED;
	mpw->pkts_n = 0;
	mpw->len = length;
	mpw->total_len = 0;
	mpw->wqe = (volatile struct mlx5_wqe *)tx_mlx5_wqe(txq, idx);
	mpw->wqe->eseg.mss = htons(length);
	mpw->wqe->eseg.inline_hdr_sz = 0;
	mpw->wqe->eseg.rsvd0 = 0;
	mpw->wqe->eseg.rsvd1 = 0;
	mpw->wqe->eseg.rsvd2 = 0;
	mpw->wqe->ctrl[0] = htonl((MLX5_OPC_MOD_MPW << 24) |
				  (txq->wqe_ci << 8) | MLX5_OPCODE_TSO);
	mpw->wqe->ctrl[2] = 0;
	mpw->wqe->ctrl[3] = 0;
	mpw->data.dseg[0] = (volatile struct mlx5_wqe_data_seg *)
		(((uintptr_t)mpw->wqe) + (2 * MLX5_WQE_DWORD_SIZE));
	mpw->data.dseg[1] = (volatile struct mlx5_wqe_data_seg *)
		(((uintptr_t)mpw->wqe) + (3 * MLX5_WQE_DWORD_SIZE));
	mpw->data.dseg[2] = &(*dseg)[0];
	mpw->data.dseg[3] = &(*dseg)[1];
	mpw->data.dseg[4] = &(*dseg)[2];
}

/**
 * Close a MPW session.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param mpw
 *   Pointer to MPW session structure.
 */
static inline void
mlx5_mpw_close(struct txq *txq, struct mlx5_mpw *mpw)
{
	unsigned int num = mpw->pkts_n;

	/*
	 * Store size in multiple of 16 bytes. Control and Ethernet segments
	 * count as 2.
	 */
	mpw->wqe->ctrl[1] = htonl(txq->qp_num_8s | (2 + num));
	mpw->state = MLX5_MPW_STATE_CLOSED;
	if (num < 3)
		++txq->wqe_ci;
	else
		txq->wqe_ci += 2;
	rte_prefetch0(tx_mlx5_wqe(txq, txq->wqe_ci));
	rte_prefetch0(tx_mlx5_wqe(txq, txq->wqe_ci + 1));
}

/**
 * DPDK callback for TX with MPW support.
 *
 * @param dpdk_txq
 *   Generic pointer to TX queue structure.
 * @param[in] pkts
 *   Packets to transmit.
 * @param pkts_n
 *   Number of packets in array.
 *
 * @return
 *   Number of packets successfully transmitted (<= pkts_n).
 */
uint16_t
mlx5_tx_burst_mpw(void *dpdk_txq, struct rte_mbuf **pkts, uint16_t pkts_n)
{
	struct txq *txq = (struct txq *)dpdk_txq;
	uint16_t elts_head = txq->elts_head;
	const unsigned int elts_n = 1 << txq->elts_n;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int max;
	uint16_t max_wqe;
	unsigned int comp;
	struct mlx5_mpw mpw = {
		.state = MLX5_MPW_STATE_CLOSED,
	};

	if (unlikely(!pkts_n))
		return 0;
	/* Prefetch first packet cacheline. */
	rte_prefetch0(tx_mlx5_wqe(txq, txq->wqe_ci));
	rte_prefetch0(tx_mlx5_wqe(txq, txq->wqe_ci + 1));
	/* Start processing. */
	txq_complete(txq);
	max = (elts_n - (elts_head - txq->elts_tail));
	if (max > elts_n)
		max -= elts_n;
	max_wqe = (1u << txq->wqe_n) - (txq->wqe_ci - txq->wqe_pi);
	if (unlikely(!max_wqe))
		return 0;
	do {
		struct rte_mbuf *buf = *(pkts++);
		unsigned int elts_head_next;
		uint32_t length;
		unsigned int segs_n = buf->nb_segs;
		uint32_t cs_flags = 0;

		/*
		 * Make sure there is enough room to store this packet and
		 * that one ring entry remains unused.
		 */
		assert(segs_n);
		if (max < segs_n + 1)
			break;
		/* Do not bother with large packets MPW cannot handle. */
		if (segs_n > MLX5_MPW_DSEG_MAX)
			break;
		max -= segs_n;
		--pkts_n;
		/* Should we enable HW CKSUM offload */
		if (buf->ol_flags &
		    (PKT_TX_IP_CKSUM | PKT_TX_TCP_CKSUM | PKT_TX_UDP_CKSUM))
			cs_flags = MLX5_ETH_WQE_L3_CSUM | MLX5_ETH_WQE_L4_CSUM;
		/* Retrieve packet information. */
		length = PKT_LEN(buf);
		assert(length);
		/* Start new session if packet differs. */
		if ((mpw.state == MLX5_MPW_STATE_OPENED) &&
		    ((mpw.len != length) ||
		     (segs_n != 1) ||
		     (mpw.wqe->eseg.cs_flags != cs_flags)))
			mlx5_mpw_close(txq, &mpw);
		if (mpw.state == MLX5_MPW_STATE_CLOSED) {
			/*
			 * Multi-Packet WQE consumes at most two WQE.
			 * mlx5_mpw_new() expects to be able to use such
			 * resources.
			 */
			if (unlikely(max_wqe < 2))
				break;
			max_wqe -= 2;
			mlx5_mpw_new(txq, &mpw, length);
			mpw.wqe->eseg.cs_flags = cs_flags;
		}
		/* Multi-segment packets must be alone in their MPW. */
		assert((segs_n == 1) || (mpw.pkts_n == 0));
#if defined(MLX5_PMD_SOFT_COUNTERS) || !defined(NDEBUG)
		length = 0;
#endif
		do {
			volatile struct mlx5_wqe_data_seg *dseg;
			uintptr_t addr;

			elts_head_next = (elts_head + 1) & (elts_n - 1);
			assert(buf);
			(*txq->elts)[elts_head] = buf;
			dseg = mpw.data.dseg[mpw.pkts_n];
			addr = rte_pktmbuf_mtod(buf, uintptr_t);
			*dseg = (struct mlx5_wqe_data_seg){
				.byte_count = htonl(DATA_LEN(buf)),
				.lkey = txq_mp2mr(txq, txq_mb2mp(buf)),
				.addr = htonll(addr),
			};
			elts_head = elts_head_next;
#if defined(MLX5_PMD_SOFT_COUNTERS) || !defined(NDEBUG)
			length += DATA_LEN(buf);
#endif
			buf = buf->next;
			++mpw.pkts_n;
			++j;
		} while (--segs_n);
		assert(length == mpw.len);
		if (mpw.pkts_n == MLX5_MPW_DSEG_MAX)
			mlx5_mpw_close(txq, &mpw);
		elts_head = elts_head_next;
#ifdef MLX5_PMD_SOFT_COUNTERS
		/* Increment sent bytes counter. */
		txq->stats.obytes += length;
#endif
		++i;
	} while (pkts_n);
	/* Take a shortcut if nothing must be sent. */
	if (unlikely(i == 0))
		return 0;
	/* Check whether completion threshold has been reached. */
	/* "j" includes both packets and segments. */
	comp = txq->elts_comp + j;
	if (comp >= MLX5_TX_COMP_THRESH) {
		volatile struct mlx5_wqe *wqe = mpw.wqe;

		/* Request completion on last WQE. */
		wqe->ctrl[2] = htonl(8);
		/* Save elts_head in unused "immediate" field of WQE. */
		wqe->ctrl[3] = elts_head;
		txq->elts_comp = 0;
	} else {
		txq->elts_comp = comp;
	}
#ifdef MLX5_PMD_SOFT_COUNTERS
	/* Increment sent packets counter. */
	txq->stats.opackets += i;
#endif
	/* Ring QP doorbell. */
	if (mpw.state == MLX5_MPW_STATE_OPENED)
		mlx5_mpw_close(txq, &mpw);
	mlx5_tx_dbrec(txq, mpw.wqe);
	txq->elts_head = elts_head;
	return i;
}

/**
 * Open a MPW inline session.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param mpw
 *   Pointer to MPW session structure.
 * @param length
 *   Packet length.
 */
static inline void
mlx5_mpw_inline_new(struct txq *txq, struct mlx5_mpw *mpw, uint32_t length)
{
	uint16_t idx = txq->wqe_ci & ((1 << txq->wqe_n) - 1);
	struct mlx5_wqe_inl_small *inl;

	mpw->state = MLX5_MPW_INL_STATE_OPENED;
	mpw->pkts_n = 0;
	mpw->len = length;
	mpw->total_len = 0;
	mpw->wqe = (volatile struct mlx5_wqe *)tx_mlx5_wqe(txq, idx);
	mpw->wqe->ctrl[0] = htonl((MLX5_OPC_MOD_MPW << 24) |
				  (txq->wqe_ci << 8) |
				  MLX5_OPCODE_TSO);
	mpw->wqe->ctrl[2] = 0;
	mpw->wqe->ctrl[3] = 0;
	mpw->wqe->eseg.mss = htons(length);
	mpw->wqe->eseg.inline_hdr_sz = 0;
	mpw->wqe->eseg.cs_flags = 0;
	mpw->wqe->eseg.rsvd0 = 0;
	mpw->wqe->eseg.rsvd1 = 0;
	mpw->wqe->eseg.rsvd2 = 0;
	inl = (struct mlx5_wqe_inl_small *)
		(((uintptr_t)mpw->wqe) + 2 * MLX5_WQE_DWORD_SIZE);
	mpw->data.raw = (uint8_t *)&inl->raw;
}

/**
 * Close a MPW inline session.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param mpw
 *   Pointer to MPW session structure.
 */
static inline void
mlx5_mpw_inline_close(struct txq *txq, struct mlx5_mpw *mpw)
{
	unsigned int size;
	struct mlx5_wqe_inl_small *inl = (struct mlx5_wqe_inl_small *)
		(((uintptr_t)mpw->wqe) + (2 * MLX5_WQE_DWORD_SIZE));

	size = MLX5_WQE_SIZE - MLX5_MWQE64_INL_DATA + mpw->total_len;
	/*
	 * Store size in multiple of 16 bytes. Control and Ethernet segments
	 * count as 2.
	 */
	mpw->wqe->ctrl[1] = htonl(txq->qp_num_8s | MLX5_WQE_DS(size));
	mpw->state = MLX5_MPW_STATE_CLOSED;
	inl->byte_cnt = htonl(mpw->total_len | MLX5_INLINE_SEG);
	txq->wqe_ci += (size + (MLX5_WQE_SIZE - 1)) / MLX5_WQE_SIZE;
}

/**
 * DPDK callback for TX with MPW inline support.
 *
 * @param dpdk_txq
 *   Generic pointer to TX queue structure.
 * @param[in] pkts
 *   Packets to transmit.
 * @param pkts_n
 *   Number of packets in array.
 *
 * @return
 *   Number of packets successfully transmitted (<= pkts_n).
 */
uint16_t
mlx5_tx_burst_mpw_inline(void *dpdk_txq, struct rte_mbuf **pkts,
			 uint16_t pkts_n)
{
	struct txq *txq = (struct txq *)dpdk_txq;
	uint16_t elts_head = txq->elts_head;
	const unsigned int elts_n = 1 << txq->elts_n;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int max;
	uint16_t max_wqe;
	unsigned int comp;
	unsigned int inline_room = txq->max_inline * RTE_CACHE_LINE_SIZE;
	struct mlx5_mpw mpw = {
		.state = MLX5_MPW_STATE_CLOSED,
	};
	/*
	 * Compute the maximum number of WQE which can be consumed by inline
	 * code.
	 * - 2 DSEG for:
	 *   - 1 control segment,
	 *   - 1 Ethernet segment,
	 * - N Dseg from the inline request.
	 */
	const unsigned int wqe_inl_n =
		((2 * MLX5_WQE_DWORD_SIZE +
		  txq->max_inline * RTE_CACHE_LINE_SIZE) +
		 RTE_CACHE_LINE_SIZE - 1) / RTE_CACHE_LINE_SIZE;

	if (unlikely(!pkts_n))
		return 0;
	/* Prefetch first packet cacheline. */
	rte_prefetch0(tx_mlx5_wqe(txq, txq->wqe_ci));
	rte_prefetch0(tx_mlx5_wqe(txq, txq->wqe_ci + 1));
	/* Start processing. */
	txq_complete(txq);
	max = (elts_n - (elts_head - txq->elts_tail));
	if (max > elts_n)
		max -= elts_n;
	do {
		struct rte_mbuf *buf = *(pkts++);
		unsigned int elts_head_next;
		uintptr_t addr;
		uint32_t length;
		unsigned int segs_n = buf->nb_segs;
		uint32_t cs_flags = 0;

		/*
		 * Make sure there is enough room to store this packet and
		 * that one ring entry remains unused.
		 */
		assert(segs_n);
		if (max < segs_n + 1)
			break;
		/* Do not bother with large packets MPW cannot handle. */
		if (segs_n > MLX5_MPW_DSEG_MAX)
			break;
		max -= segs_n;
		--pkts_n;
		/*
		 * Compute max_wqe in case less WQE were consumed in previous
		 * iteration.
		 */
		max_wqe = (1u << txq->wqe_n) - (txq->wqe_ci - txq->wqe_pi);
		/* Should we enable HW CKSUM offload */
		if (buf->ol_flags &
		    (PKT_TX_IP_CKSUM | PKT_TX_TCP_CKSUM | PKT_TX_UDP_CKSUM))
			cs_flags = MLX5_ETH_WQE_L3_CSUM | MLX5_ETH_WQE_L4_CSUM;
		/* Retrieve packet information. */
		length = PKT_LEN(buf);
		/* Start new session if packet differs. */
		if (mpw.state == MLX5_MPW_STATE_OPENED) {
			if ((mpw.len != length) ||
			    (segs_n != 1) ||
			    (mpw.wqe->eseg.cs_flags != cs_flags))
				mlx5_mpw_close(txq, &mpw);
		} else if (mpw.state == MLX5_MPW_INL_STATE_OPENED) {
			if ((mpw.len != length) ||
			    (segs_n != 1) ||
			    (length > inline_room) ||
			    (mpw.wqe->eseg.cs_flags != cs_flags)) {
				mlx5_mpw_inline_close(txq, &mpw);
				inline_room =
					txq->max_inline * RTE_CACHE_LINE_SIZE;
			}
		}
		if (mpw.state == MLX5_MPW_STATE_CLOSED) {
			if ((segs_n != 1) ||
			    (length > inline_room)) {
				/*
				 * Multi-Packet WQE consumes at most two WQE.
				 * mlx5_mpw_new() expects to be able to use
				 * such resources.
				 */
				if (unlikely(max_wqe < 2))
					break;
				max_wqe -= 2;
				mlx5_mpw_new(txq, &mpw, length);
				mpw.wqe->eseg.cs_flags = cs_flags;
			} else {
				if (unlikely(max_wqe < wqe_inl_n))
					break;
				max_wqe -= wqe_inl_n;
				mlx5_mpw_inline_new(txq, &mpw, length);
				mpw.wqe->eseg.cs_flags = cs_flags;
			}
		}
		/* Multi-segment packets must be alone in their MPW. */
		assert((segs_n == 1) || (mpw.pkts_n == 0));
		if (mpw.state == MLX5_MPW_STATE_OPENED) {
			assert(inline_room ==
			       txq->max_inline * RTE_CACHE_LINE_SIZE);
#if defined(MLX5_PMD_SOFT_COUNTERS) || !defined(NDEBUG)
			length = 0;
#endif
			do {
				volatile struct mlx5_wqe_data_seg *dseg;

				elts_head_next =
					(elts_head + 1) & (elts_n - 1);
				assert(buf);
				(*txq->elts)[elts_head] = buf;
				dseg = mpw.data.dseg[mpw.pkts_n];
				addr = rte_pktmbuf_mtod(buf, uintptr_t);
				*dseg = (struct mlx5_wqe_data_seg){
					.byte_count = htonl(DATA_LEN(buf)),
					.lkey = txq_mp2mr(txq, txq_mb2mp(buf)),
					.addr = htonll(addr),
				};
				elts_head = elts_head_next;
#if defined(MLX5_PMD_SOFT_COUNTERS) || !defined(NDEBUG)
				length += DATA_LEN(buf);
#endif
				buf = buf->next;
				++mpw.pkts_n;
				++j;
			} while (--segs_n);
			assert(length == mpw.len);
			if (mpw.pkts_n == MLX5_MPW_DSEG_MAX)
				mlx5_mpw_close(txq, &mpw);
		} else {
			unsigned int max;

			assert(mpw.state == MLX5_MPW_INL_STATE_OPENED);
			assert(length <= inline_room);
			assert(length == DATA_LEN(buf));
			elts_head_next = (elts_head + 1) & (elts_n - 1);
			addr = rte_pktmbuf_mtod(buf, uintptr_t);
			(*txq->elts)[elts_head] = buf;
			/* Maximum number of bytes before wrapping. */
			max = ((((uintptr_t)(txq->wqes)) +
				(1 << txq->wqe_n) *
				MLX5_WQE_SIZE) -
			       (uintptr_t)mpw.data.raw);
			if (length > max) {
				rte_memcpy((void *)(uintptr_t)mpw.data.raw,
					   (void *)addr,
					   max);
				mpw.data.raw = (volatile void *)txq->wqes;
				rte_memcpy((void *)(uintptr_t)mpw.data.raw,
					   (void *)(addr + max),
					   length - max);
				mpw.data.raw += length - max;
			} else {
				rte_memcpy((void *)(uintptr_t)mpw.data.raw,
					   (void *)addr,
					   length);

				if (length == max)
					mpw.data.raw =
						(volatile void *)txq->wqes;
				else
					mpw.data.raw += length;
			}
			++mpw.pkts_n;
			mpw.total_len += length;
			++j;
			if (mpw.pkts_n == MLX5_MPW_DSEG_MAX) {
				mlx5_mpw_inline_close(txq, &mpw);
				inline_room =
					txq->max_inline * RTE_CACHE_LINE_SIZE;
			} else {
				inline_room -= length;
			}
		}
		elts_head = elts_head_next;
#ifdef MLX5_PMD_SOFT_COUNTERS
		/* Increment sent bytes counter. */
		txq->stats.obytes += length;
#endif
		++i;
	} while (pkts_n);
	/* Take a shortcut if nothing must be sent. */
	if (unlikely(i == 0))
		return 0;
	/* Check whether completion threshold has been reached. */
	/* "j" includes both packets and segments. */
	comp = txq->elts_comp + j;
	if (comp >= MLX5_TX_COMP_THRESH) {
		volatile struct mlx5_wqe *wqe = mpw.wqe;

		/* Request completion on last WQE. */
		wqe->ctrl[2] = htonl(8);
		/* Save elts_head in unused "immediate" field of WQE. */
		wqe->ctrl[3] = elts_head;
		txq->elts_comp = 0;
	} else {
		txq->elts_comp = comp;
	}
#ifdef MLX5_PMD_SOFT_COUNTERS
	/* Increment sent packets counter. */
	txq->stats.opackets += i;
#endif
	/* Ring QP doorbell. */
	if (mpw.state == MLX5_MPW_INL_STATE_OPENED)
		mlx5_mpw_inline_close(txq, &mpw);
	else if (mpw.state == MLX5_MPW_STATE_OPENED)
		mlx5_mpw_close(txq, &mpw);
	mlx5_tx_dbrec(txq, mpw.wqe);
	txq->elts_head = elts_head;
	return i;
}

/**
 * Open an Enhanced MPW session.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param mpw
 *   Pointer to MPW session structure.
 * @param length
 *   Packet length.
 */
static inline void
mlx5_empw_new(struct txq *txq, struct mlx5_mpw *mpw, int padding)
{
	uint16_t idx = txq->wqe_ci & ((1 << txq->wqe_n) - 1);

	mpw->state = MLX5_MPW_ENHANCED_STATE_OPENED;
	mpw->pkts_n = 0;
	mpw->total_len = sizeof(struct mlx5_wqe);
	mpw->wqe = (volatile struct mlx5_wqe *)tx_mlx5_wqe(txq, idx);
	mpw->wqe->ctrl[0] = htonl((MLX5_OPC_MOD_ENHANCED_MPSW << 24) |
				  (txq->wqe_ci << 8) |
				  MLX5_OPCODE_ENHANCED_MPSW);
	mpw->wqe->ctrl[2] = 0;
	mpw->wqe->ctrl[3] = 0;
	memset((void *)(uintptr_t)&mpw->wqe->eseg, 0, MLX5_WQE_DWORD_SIZE);
	if (unlikely(padding)) {
		uintptr_t addr = (uintptr_t)(mpw->wqe + 1);

		/* Pad the first 2 DWORDs with zero-length inline header. */
		*(volatile uint32_t *)addr = htonl(MLX5_INLINE_SEG);
		*(volatile uint32_t *)(addr + MLX5_WQE_DWORD_SIZE) =
			htonl(MLX5_INLINE_SEG);
		mpw->total_len += 2 * MLX5_WQE_DWORD_SIZE;
		/* Start from the next WQEBB. */
		mpw->data.raw = (volatile void *)(tx_mlx5_wqe(txq, idx + 1));
	} else {
		mpw->data.raw = (volatile void *)(mpw->wqe + 1);
	}
}

/**
 * Close an Enhanced MPW session.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param mpw
 *   Pointer to MPW session structure.
 *
 * @return
 *   Number of consumed WQEs.
 */
static inline uint16_t
mlx5_empw_close(struct txq *txq, struct mlx5_mpw *mpw)
{
	uint16_t ret;

	/* Store size in multiple of 16 bytes. Control and Ethernet segments
	 * count as 2.
	 */
	mpw->wqe->ctrl[1] = htonl(txq->qp_num_8s | MLX5_WQE_DS(mpw->total_len));
	mpw->state = MLX5_MPW_STATE_CLOSED;
	ret = (mpw->total_len + (MLX5_WQE_SIZE - 1)) / MLX5_WQE_SIZE;
	txq->wqe_ci += ret;
	return ret;
}

/**
 * DPDK callback for TX with Enhanced MPW support.
 *
 * @param dpdk_txq
 *   Generic pointer to TX queue structure.
 * @param[in] pkts
 *   Packets to transmit.
 * @param pkts_n
 *   Number of packets in array.
 *
 * @return
 *   Number of packets successfully transmitted (<= pkts_n).
 */
uint16_t
mlx5_tx_burst_empw(void *dpdk_txq, struct rte_mbuf **pkts, uint16_t pkts_n)
{
	struct txq *txq = (struct txq *)dpdk_txq;
	uint16_t elts_head = txq->elts_head;
	const unsigned int elts_n = 1 << txq->elts_n;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int max_elts;
	uint16_t max_wqe;
	unsigned int max_inline = txq->max_inline * RTE_CACHE_LINE_SIZE;
	unsigned int mpw_room = 0;
	unsigned int inl_pad = 0;
	uint32_t inl_hdr;
	struct mlx5_mpw mpw = {
		.state = MLX5_MPW_STATE_CLOSED,
	};

	if (unlikely(!pkts_n))
		return 0;
	/* Start processing. */
	txq_complete(txq);
	max_elts = (elts_n - (elts_head - txq->elts_tail));
	if (max_elts > elts_n)
		max_elts -= elts_n;
	/* A CQE slot must always be available. */
	assert((1u << txq->cqe_n) - (txq->cq_pi - txq->cq_ci));
	max_wqe = (1u << txq->wqe_n) - (txq->wqe_ci - txq->wqe_pi);
	if (unlikely(!max_wqe))
		return 0;
	do {
		struct rte_mbuf *buf = *(pkts++);
		unsigned int elts_head_next;
		uintptr_t addr;
		uint64_t naddr;
		unsigned int n;
		unsigned int do_inline = 0; /* Whether inline is possible. */
		uint32_t length;
		unsigned int segs_n = buf->nb_segs;
		uint32_t cs_flags = 0;

		/*
		 * Make sure there is enough room to store this packet and
		 * that one ring entry remains unused.
		 */
		assert(segs_n);
		if (max_elts - j < segs_n + 1)
			break;
		/* Do not bother with large packets MPW cannot handle. */
		if (segs_n > MLX5_MPW_DSEG_MAX)
			break;
		/* Should we enable HW CKSUM offload. */
		if (buf->ol_flags &
		    (PKT_TX_IP_CKSUM | PKT_TX_TCP_CKSUM | PKT_TX_UDP_CKSUM))
			cs_flags = MLX5_ETH_WQE_L3_CSUM | MLX5_ETH_WQE_L4_CSUM;
		/* Retrieve packet information. */
		length = PKT_LEN(buf);
		/* Start new session if:
		 * - multi-segment packet
		 * - no space left even for a dseg
		 * - next packet can be inlined with a new WQE
		 * - cs_flag differs
		 * It can't be MLX5_MPW_STATE_OPENED as always have a single
		 * segmented packet.
		 */
		if (mpw.state == MLX5_MPW_ENHANCED_STATE_OPENED) {
			if ((segs_n != 1) ||
			    (inl_pad + sizeof(struct mlx5_wqe_data_seg) >
			      mpw_room) ||
			    (length <= txq->inline_max_packet_sz &&
			     inl_pad + sizeof(inl_hdr) + length >
			      mpw_room) ||
			    (mpw.wqe->eseg.cs_flags != cs_flags))
				max_wqe -= mlx5_empw_close(txq, &mpw);
		}
		if (unlikely(mpw.state == MLX5_MPW_STATE_CLOSED)) {
			if (unlikely(segs_n != 1)) {
				/* Fall back to legacy MPW.
				 * A MPW session consumes 2 WQEs at most to
				 * include MLX5_MPW_DSEG_MAX pointers.
				 */
				if (unlikely(max_wqe < 2))
					break;
				mlx5_mpw_new(txq, &mpw, length);
			} else {
				/* In Enhanced MPW, inline as much as the budget
				 * is allowed. The remaining space is to be
				 * filled with dsegs. If the title WQEBB isn't
				 * padded, it will have 2 dsegs there.
				 */
				mpw_room = RTE_MIN(MLX5_WQE_SIZE_MAX,
					    (max_inline ? max_inline :
					     pkts_n * MLX5_WQE_DWORD_SIZE) +
					    MLX5_WQE_SIZE);
				if (unlikely(max_wqe * MLX5_WQE_SIZE <
					      mpw_room))
					break;
				/* Don't pad the title WQEBB to not waste WQ. */
				mlx5_empw_new(txq, &mpw, 0);
				mpw_room -= mpw.total_len;
				inl_pad = 0;
				do_inline =
					length <= txq->inline_max_packet_sz &&
					sizeof(inl_hdr) + length <= mpw_room &&
					!txq->mpw_hdr_dseg;
			}
			mpw.wqe->eseg.cs_flags = cs_flags;
		} else {
			/* Evaluate whether the next packet can be inlined.
			 * Inlininig is possible when:
			 * - length is less than configured value
			 * - length fits for remaining space
			 * - not required to fill the title WQEBB with dsegs
			 */
			do_inline =
				length <= txq->inline_max_packet_sz &&
				inl_pad + sizeof(inl_hdr) + length <=
				 mpw_room &&
				(!txq->mpw_hdr_dseg ||
				 mpw.total_len >= MLX5_WQE_SIZE);
		}
		/* Multi-segment packets must be alone in their MPW. */
		assert((segs_n == 1) || (mpw.pkts_n == 0));
		if (unlikely(mpw.state == MLX5_MPW_STATE_OPENED)) {
#if defined(MLX5_PMD_SOFT_COUNTERS) || !defined(NDEBUG)
			length = 0;
#endif
			do {
				volatile struct mlx5_wqe_data_seg *dseg;

				elts_head_next =
					(elts_head + 1) & (elts_n - 1);
				assert(buf);
				(*txq->elts)[elts_head] = buf;
				dseg = mpw.data.dseg[mpw.pkts_n];
				addr = rte_pktmbuf_mtod(buf, uintptr_t);
				*dseg = (struct mlx5_wqe_data_seg){
					.byte_count = htonl(DATA_LEN(buf)),
					.lkey = txq_mp2mr(txq, txq_mb2mp(buf)),
					.addr = htonll(addr),
				};
				elts_head = elts_head_next;
#if defined(MLX5_PMD_SOFT_COUNTERS) || !defined(NDEBUG)
				length += DATA_LEN(buf);
#endif
				buf = buf->next;
				++j;
				++mpw.pkts_n;
			} while (--segs_n);
			/* A multi-segmented packet takes one MPW session.
			 * TODO: Pack more multi-segmented packets if possible.
			 */
			mlx5_mpw_close(txq, &mpw);
			if (mpw.pkts_n < 3)
				max_wqe--;
			else
				max_wqe -= 2;
		} else if (do_inline) {
			/* Inline packet into WQE. */
			unsigned int max;

			assert(mpw.state == MLX5_MPW_ENHANCED_STATE_OPENED);
			assert(length == DATA_LEN(buf));
			inl_hdr = htonl(length | MLX5_INLINE_SEG);
			addr = rte_pktmbuf_mtod(buf, uintptr_t);
			mpw.data.raw = (volatile void *)
				((uintptr_t)mpw.data.raw + inl_pad);
			max = tx_mlx5_wq_tailroom(txq,
					(void *)(uintptr_t)mpw.data.raw);
			/* Copy inline header. */
			mpw.data.raw = (volatile void *)
				mlx5_copy_to_wq(
					  (void *)(uintptr_t)mpw.data.raw,
					  &inl_hdr,
					  sizeof(inl_hdr),
					  (void *)(uintptr_t)txq->wqes,
					  max);
			max = tx_mlx5_wq_tailroom(txq,
					(void *)(uintptr_t)mpw.data.raw);
			/* Copy packet data. */
			mpw.data.raw = (volatile void *)
				mlx5_copy_to_wq(
					  (void *)(uintptr_t)mpw.data.raw,
					  (void *)addr,
					  length,
					  (void *)(uintptr_t)txq->wqes,
					  max);
			++mpw.pkts_n;
			mpw.total_len += (inl_pad + sizeof(inl_hdr) + length);
			/* No need to get completion as the entire packet is
			 * copied to WQ. Free the buf right away.
			 */
			elts_head_next = elts_head;
			rte_pktmbuf_free_seg(buf);
			mpw_room -= (inl_pad + sizeof(inl_hdr) + length);
			/* Add pad in the next packet if any. */
			inl_pad = (((uintptr_t)mpw.data.raw +
					(MLX5_WQE_DWORD_SIZE - 1)) &
					~(MLX5_WQE_DWORD_SIZE - 1)) -
				  (uintptr_t)mpw.data.raw;
		} else {
			/* No inline. Load a dseg of packet pointer. */
			volatile rte_v128u32_t *dseg;

			assert(mpw.state == MLX5_MPW_ENHANCED_STATE_OPENED);
			assert((inl_pad + sizeof(*dseg)) <= mpw_room);
			assert(length == DATA_LEN(buf));
			if (!tx_mlx5_wq_tailroom(txq,
					(void *)((uintptr_t)mpw.data.raw
						+ inl_pad)))
				dseg = (volatile void *)txq->wqes;
			else
				dseg = (volatile void *)
					((uintptr_t)mpw.data.raw +
					 inl_pad);
			elts_head_next = (elts_head + 1) & (elts_n - 1);
			(*txq->elts)[elts_head] = buf;
			addr = rte_pktmbuf_mtod(buf, uintptr_t);
			for (n = 0; n * RTE_CACHE_LINE_SIZE < length; n++)
				rte_prefetch2((void *)(addr +
						n * RTE_CACHE_LINE_SIZE));
			naddr = htonll(addr);
			*dseg = (rte_v128u32_t) {
				htonl(length),
				txq_mp2mr(txq, txq_mb2mp(buf)),
				naddr,
				naddr >> 32,
			};
			mpw.data.raw = (volatile void *)(dseg + 1);
			mpw.total_len += (inl_pad + sizeof(*dseg));
			++j;
			++mpw.pkts_n;
			mpw_room -= (inl_pad + sizeof(*dseg));
			inl_pad = 0;
		}
		elts_head = elts_head_next;
#ifdef MLX5_PMD_SOFT_COUNTERS
		/* Increment sent bytes counter. */
		txq->stats.obytes += length;
#endif
		++i;
	} while (i < pkts_n);
	/* Take a shortcut if nothing must be sent. */
	if (unlikely(i == 0))
		return 0;
	/* Check whether completion threshold has been reached. */
	if (txq->elts_comp + j >= MLX5_TX_COMP_THRESH ||
			(uint16_t)(txq->wqe_ci - txq->mpw_comp) >=
			 (1 << txq->wqe_n) / MLX5_TX_COMP_THRESH_INLINE_DIV) {
		volatile struct mlx5_wqe *wqe = mpw.wqe;

		/* Request completion on last WQE. */
		wqe->ctrl[2] = htonl(8);
		/* Save elts_head in unused "immediate" field of WQE. */
		wqe->ctrl[3] = elts_head;
		txq->elts_comp = 0;
		txq->mpw_comp = txq->wqe_ci;
		txq->cq_pi++;
	} else {
		txq->elts_comp += j;
	}
#ifdef MLX5_PMD_SOFT_COUNTERS
	/* Increment sent packets counter. */
	txq->stats.opackets += i;
#endif
	if (mpw.state == MLX5_MPW_ENHANCED_STATE_OPENED)
		mlx5_empw_close(txq, &mpw);
	else if (mpw.state == MLX5_MPW_STATE_OPENED)
		mlx5_mpw_close(txq, &mpw);
	/* Ring QP doorbell. */
	mlx5_tx_dbrec(txq, mpw.wqe);
	txq->elts_head = elts_head;
	return i;
}

/**
 * Translate RX completion flags to packet type.
 *
 * @param[in] cqe
 *   Pointer to CQE.
 *
 * @note: fix mlx5_dev_supported_ptypes_get() if any change here.
 *
 * @return
 *   Packet type for struct rte_mbuf.
 */
static inline uint32_t
rxq_cq_to_pkt_type(volatile struct mlx5_cqe *cqe)
{
	uint32_t pkt_type;
	uint16_t flags = ntohs(cqe->hdr_type_etc);

	if (cqe->pkt_info & MLX5_CQE_RX_TUNNEL_PACKET) {
		pkt_type =
			TRANSPOSE(flags,
				  MLX5_CQE_RX_IPV4_PACKET,
				  RTE_PTYPE_INNER_L3_IPV4_EXT_UNKNOWN) |
			TRANSPOSE(flags,
				  MLX5_CQE_RX_IPV6_PACKET,
				  RTE_PTYPE_INNER_L3_IPV6_EXT_UNKNOWN);
		pkt_type |= ((cqe->pkt_info & MLX5_CQE_RX_OUTER_PACKET) ?
			     RTE_PTYPE_L3_IPV6_EXT_UNKNOWN :
			     RTE_PTYPE_L3_IPV4_EXT_UNKNOWN);
	} else {
		pkt_type =
			TRANSPOSE(flags,
				  MLX5_CQE_L3_HDR_TYPE_IPV6,
				  RTE_PTYPE_L3_IPV6_EXT_UNKNOWN) |
			TRANSPOSE(flags,
				  MLX5_CQE_L3_HDR_TYPE_IPV4,
				  RTE_PTYPE_L3_IPV4_EXT_UNKNOWN);
	}
	return pkt_type;
}

/**
 * Get size of the next packet for a given CQE. For compressed CQEs, the
 * consumer index is updated only once all packets of the current one have
 * been processed.
 *
 * @param rxq
 *   Pointer to RX queue.
 * @param cqe
 *   CQE to process.
 * @param[out] rss_hash
 *   Packet RSS Hash result.
 *
 * @return
 *   Packet size in bytes (0 if there is none), -1 in case of completion
 *   with error.
 */
static inline int
mlx5_rx_poll_len(struct rxq *rxq, volatile struct mlx5_cqe *cqe,
		 uint16_t cqe_cnt, uint32_t *rss_hash)
{
	struct rxq_zip *zip = &rxq->zip;
	uint16_t cqe_n = cqe_cnt + 1;
	int len = 0;
	uint16_t idx, end;

	/* Process compressed data in the CQE and mini arrays. */
	if (zip->ai) {
		volatile struct mlx5_mini_cqe8 (*mc)[8] =
			(volatile struct mlx5_mini_cqe8 (*)[8])
			(uintptr_t)(&(*rxq->cqes)[zip->ca & cqe_cnt]);

		len = ntohl((*mc)[zip->ai & 7].byte_cnt);
		*rss_hash = ntohl((*mc)[zip->ai & 7].rx_hash_result);
		if ((++zip->ai & 7) == 0) {
			/* Invalidate consumed CQEs */
			idx = zip->ca;
			end = zip->na;
			while (idx != end) {
				(*rxq->cqes)[idx & cqe_cnt].op_own =
					MLX5_CQE_INVALIDATE;
				++idx;
			}
			/*
			 * Increment consumer index to skip the number of
			 * CQEs consumed. Hardware leaves holes in the CQ
			 * ring for software use.
			 */
			zip->ca = zip->na;
			zip->na += 8;
		}
		if (unlikely(rxq->zip.ai == rxq->zip.cqe_cnt)) {
			/* Invalidate the rest */
			idx = zip->ca;
			end = zip->cq_ci;

			while (idx != end) {
				(*rxq->cqes)[idx & cqe_cnt].op_own =
					MLX5_CQE_INVALIDATE;
				++idx;
			}
			rxq->cq_ci = zip->cq_ci;
			zip->ai = 0;
		}
	/* No compressed data, get next CQE and verify if it is compressed. */
	} else {
		int ret;
		int8_t op_own;

		ret = check_cqe(cqe, cqe_n, rxq->cq_ci);
		if (unlikely(ret == 1))
			return 0;
		++rxq->cq_ci;
		op_own = cqe->op_own;
		if (MLX5_CQE_FORMAT(op_own) == MLX5_COMPRESSED) {
			volatile struct mlx5_mini_cqe8 (*mc)[8] =
				(volatile struct mlx5_mini_cqe8 (*)[8])
				(uintptr_t)(&(*rxq->cqes)[rxq->cq_ci &
							  cqe_cnt]);

			/* Fix endianness. */
			zip->cqe_cnt = ntohl(cqe->byte_cnt);
			/*
			 * Current mini array position is the one returned by
			 * check_cqe64().
			 *
			 * If completion comprises several mini arrays, as a
			 * special case the second one is located 7 CQEs after
			 * the initial CQE instead of 8 for subsequent ones.
			 */
			zip->ca = rxq->cq_ci;
			zip->na = zip->ca + 7;
			/* Compute the next non compressed CQE. */
			--rxq->cq_ci;
			zip->cq_ci = rxq->cq_ci + zip->cqe_cnt;
			/* Get packet size to return. */
			len = ntohl((*mc)[0].byte_cnt);
			*rss_hash = ntohl((*mc)[0].rx_hash_result);
			zip->ai = 1;
			/* Prefetch all the entries to be invalidated */
			idx = zip->ca;
			end = zip->cq_ci;
			while (idx != end) {
				rte_prefetch0(&(*rxq->cqes)[(idx) & cqe_cnt]);
				++idx;
			}
		} else {
			len = ntohl(cqe->byte_cnt);
			*rss_hash = ntohl(cqe->rx_hash_res);
		}
		/* Error while receiving packet. */
		if (unlikely(MLX5_CQE_OPCODE(op_own) == MLX5_CQE_RESP_ERR))
			return -1;
	}
	return len;
}

/**
 * Translate RX completion flags to offload flags.
 *
 * @param[in] rxq
 *   Pointer to RX queue structure.
 * @param[in] cqe
 *   Pointer to CQE.
 *
 * @return
 *   Offload flags (ol_flags) for struct rte_mbuf.
 */
static inline uint32_t
rxq_cq_to_ol_flags(struct rxq *rxq, volatile struct mlx5_cqe *cqe)
{
	uint32_t ol_flags = 0;
	uint16_t flags = ntohs(cqe->hdr_type_etc);

	ol_flags =
		TRANSPOSE(flags,
			  MLX5_CQE_RX_L3_HDR_VALID,
			  PKT_RX_IP_CKSUM_GOOD) |
		TRANSPOSE(flags,
			  MLX5_CQE_RX_L4_HDR_VALID,
			  PKT_RX_L4_CKSUM_GOOD);
	if ((cqe->pkt_info & MLX5_CQE_RX_TUNNEL_PACKET) && (rxq->csum_l2tun))
		ol_flags |=
			TRANSPOSE(flags,
				  MLX5_CQE_RX_L3_HDR_VALID,
				  PKT_RX_IP_CKSUM_GOOD) |
			TRANSPOSE(flags,
				  MLX5_CQE_RX_L4_HDR_VALID,
				  PKT_RX_L4_CKSUM_GOOD);
	return ol_flags;
}

#ifdef MLX5_VECTORIZED_RX
#if !defined (RTE_ARCH_X86_64)
#error Currently supports only 64bit architectures
#endif

#ifndef __INTEL_COMPILER
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

static inline void
rxq_copy_mbuf_v(struct rxq *rxq, struct rte_mbuf **pkts, uint16_t n)
{
	const uint16_t q_mask = (1 << rxq->elts_n) - 1;
	struct rte_mbuf **elts = &(*rxq->elts)[rxq->rq_pi & q_mask];
	unsigned int pos;
	uint16_t p = n & -2;

	for (pos = 0; pos < p; pos += 2) {
		__m128i mbp;
		mbp = _mm_loadu_si128((__m128i *)&elts[pos]);
		_mm_storeu_si128((__m128i *)&pkts[pos], mbp);
	}
	if (n & 1)
		pkts[pos] = elts[pos];
}

static inline void
rxq_replenish_bulk_mbuf(struct rxq *rxq)
{
	const unsigned int q_mask = (1 << rxq->elts_n) - 1;
	unsigned int elts_idx = rxq->rq_ci & q_mask;
	struct rte_mbuf **elts = &(*rxq->elts)[elts_idx];
	volatile struct mlx5_wqe_data_seg *wq = &(*rxq->wqes)[elts_idx];
	unsigned int i;

	if (rte_mempool_get_bulk(rxq->mp, (void *)elts,
				 MLX5_VPMD_RXQ_RPLNSH_THRESH) < 0) {
		/* TODO: exception handling by fake_mbuf? */
		rxq->stats.rx_nombuf += MLX5_VPMD_RXQ_RPLNSH_THRESH;
		return;
	}
	for (i = 0; i < MLX5_VPMD_RXQ_RPLNSH_THRESH; i++)
		wq[i].addr = htonll(rte_pktmbuf_mtod(elts[i], uintptr_t));
	rxq->rq_ci += MLX5_VPMD_RXQ_RPLNSH_THRESH;
	rte_wmb();
	*rxq->rq_db = htonl(rxq->rq_ci);
}

static inline void
rxq_cq_uncompress_v(struct rxq *rxq,
		    volatile struct mlx5_cqe *cq,
		    struct rte_mbuf **elts)
{
	volatile struct mlx5_mini_cqe8 *mcq = (void *)(cq + 1);
	struct rte_mbuf *t_pkt = elts[0]; /* Title packet */
	unsigned int pos;
	uint16_t mcqe_n;
	__m128i shuf_mask1, shuf_mask2;
	__m128i rearm, rxdf;
	const __m128i crc_adjust = _mm_set_epi16(
				0, 0, 0,
				rxq->crc_present * (-ETHER_CRC_LEN),
				0,
				rxq->crc_present * (-ETHER_CRC_LEN),
				0, 0
				);

	rearm = _mm_loadu_si128((__m128i *)&t_pkt->rearm_data);
	rxdf = _mm_loadu_si128((__m128i *)&t_pkt->rx_descriptor_fields1);
	/* mask to shuffle from extracted mini CQE to mbuf */
	shuf_mask1 = _mm_set_epi8(
			0, 1, 2, 3,            /* rss, bswap32 */
			0xFF, 0xFF,            /* skip vlan_tci */
			6, 7,                  /* data_len, bswap16 */
			0xFF, 0xFF, 6, 7,      /* pkt_len, bswap16 */
			0xFF, 0xFF, 0xFF, 0xFF /* skip packet_type */
			);
	shuf_mask2 = _mm_set_epi8(
			8, 9, 10, 11,          /* rss, bswap32 */
			0xFF, 0xFF,            /* skip vlan_tci */
			14, 15,                /* data_len, bswap16 */
			0xFF, 0xFF, 14, 15,    /* pkt_len, bswap16 */
			0xFF, 0xFF, 0xFF, 0xFF /* skip packet_type */
			);
	/* Restore the compressed count. Must be 16 bits. */
	mcqe_n = t_pkt->data_len + (rxq->crc_present * ETHER_CRC_LEN);
	/*
	 * A. load mCQEs into a 128bit register.
	 * B. store rearm data to mbuf.
	 * C. combine data from mCQEs with rx_descriptor_fields1.
	 * D. store rx_descriptor_fields1.
	 */
	for (pos = 0; pos < mcqe_n; ) {
		__m128i mcqe1, mcqe2;
		__m128i rxdf1, rxdf2;

		/* A.1 load mCQEs into a 128bit register. */
		mcqe1 = _mm_loadu_si128((__m128i *)&mcq[pos % 8]);
		mcqe2 = _mm_loadu_si128((__m128i *)&mcq[pos % 8 + 2]);
		/* B.1 store rearm data to mbuf. */
		_mm_storeu_si128((__m128i *)&elts[pos]->rearm_data, rearm);
		_mm_storeu_si128((__m128i *)&elts[pos + 1]->rearm_data, rearm);
		_mm_storeu_si128((__m128i *)&elts[pos + 2]->rearm_data, rearm);
		_mm_storeu_si128((__m128i *)&elts[pos + 3]->rearm_data, rearm);
		/* C.1 combine data from mCQEs with rx_descriptor_fields1. */
		rxdf1 = _mm_shuffle_epi8(mcqe1, shuf_mask1);
		rxdf2 = _mm_shuffle_epi8(mcqe1, shuf_mask2);
		rxdf1 = _mm_add_epi16(rxdf1, crc_adjust);
		rxdf2 = _mm_add_epi16(rxdf2, crc_adjust);
		rxdf1 = _mm_blend_epi16(rxdf1, rxdf, 0x23);
		rxdf2 = _mm_blend_epi16(rxdf2, rxdf, 0x23);
		/* D.1 store rx_descriptor_fields1. */
		_mm_storeu_si128((__m128i *)&elts[pos]->rx_descriptor_fields1,
				 rxdf1);
		_mm_storeu_si128((__m128i *)&elts[pos + 1]->rx_descriptor_fields1,
				 rxdf2);
		/* C.1 combine data from mCQEs with rx_descriptor_fields1. */
		rxdf1 = _mm_shuffle_epi8(mcqe2, shuf_mask1);
		rxdf2 = _mm_shuffle_epi8(mcqe2, shuf_mask2);
		rxdf1 = _mm_add_epi16(rxdf1, crc_adjust);
		rxdf2 = _mm_add_epi16(rxdf2, crc_adjust);
		rxdf1 = _mm_blend_epi16(rxdf1, rxdf, 0x23);
		rxdf2 = _mm_blend_epi16(rxdf2, rxdf, 0x23);
		/* D.1 store rx_descriptor_fields1. */
		_mm_storeu_si128((__m128i *)&elts[pos + 2]->rx_descriptor_fields1,
				 rxdf1);
		_mm_storeu_si128((__m128i *)&elts[pos + 3]->rx_descriptor_fields1,
				 rxdf2);
		/* TODO: need to copy hash.fdir.hi ?? */
		pos += MLX5_VPMD_DESCS_PER_LOOP;
		/* Move to next CQE */
		if (!(pos & 0x7))
			mcq = (void *)(cq + pos);
	}
	/* Invalidate consumed CQEs. */
	for (pos = 1; pos < mcqe_n; ++pos)
		cq[pos].op_own = MLX5_CQE_INVALIDATE;
	rxq->cq_ci += mcqe_n;
}

static uint32_t mlx5_ptype_table[16] = {
	RTE_PTYPE_UNKNOWN,
	RTE_PTYPE_L3_IPV6_EXT_UNKNOWN, /* b0001 */
	RTE_PTYPE_L3_IPV4_EXT_UNKNOWN, /* b0010 */
	RTE_PTYPE_UNKNOWN, RTE_PTYPE_UNKNOWN,
	RTE_PTYPE_L3_IPV4_EXT_UNKNOWN |
		RTE_PTYPE_INNER_L3_IPV6_EXT_UNKNOWN, /* b0101 */
	RTE_PTYPE_L3_IPV4_EXT_UNKNOWN |
		RTE_PTYPE_INNER_L3_IPV4_EXT_UNKNOWN, /* b0110 */
	RTE_PTYPE_UNKNOWN, RTE_PTYPE_UNKNOWN,
	RTE_PTYPE_L3_IPV6_EXT_UNKNOWN, /* b1001 */
	RTE_PTYPE_L3_IPV4_EXT_UNKNOWN, /* b1010 */
	RTE_PTYPE_UNKNOWN, RTE_PTYPE_UNKNOWN,
	RTE_PTYPE_L3_IPV6_EXT_UNKNOWN |
		RTE_PTYPE_INNER_L3_IPV6_EXT_UNKNOWN, /* b1101 */
	RTE_PTYPE_L3_IPV6_EXT_UNKNOWN |
		RTE_PTYPE_INNER_L3_IPV4_EXT_UNKNOWN, /* b1110 */
	RTE_PTYPE_UNKNOWN
};


static inline void
rxq_cq_to_ptype_oflags_v(struct rxq *rxq, __m128i cqes[4], struct rte_mbuf **pkts)
{
	__m128i pinfo0;
	__m128i pinfo1;
	__m128i pinfo, ptype;
	const __m128i ptype_mask = _mm_set_epi32(
			0xd07, 0xd07, 0xd07, 0xd07);
	const __m128i pinfo_mask = _mm_set_epi32(
			0x3, 0x3, 0x3, 0x3);
	const __m128i mbuf_init = _mm_loadl_epi64((__m128i *)&rxq->mbuf_initializer);
	__m128i rearm0, rearm1, rearm2, rearm3;

	/* Extract pkt_info field */
	pinfo0 = _mm_unpacklo_epi32(cqes[0], cqes[1]);
	pinfo1 = _mm_unpacklo_epi32(cqes[2], cqes[3]);
	pinfo = _mm_unpacklo_epi64(pinfo0, pinfo1);
	/* Extract hdr_type_etc field */
	pinfo0 = _mm_unpackhi_epi32(cqes[0], cqes[1]);
	pinfo1 = _mm_unpackhi_epi32(cqes[2], cqes[3]);
	ptype = _mm_unpacklo_epi64(pinfo0, pinfo1);
	/*
	 * Merge the two fields to generate the following
	 * bit[0]  = l2_ok,    bit[1]     = l3_ok
	 * bit[2]  = l4_ok,
	 * bit[8]  = cv,       bit[11:10] = l3_hdr_type
	 * bit[12] = tunneled, bit[13]    = outer_l3_type
	 */
	ptype = _mm_and_si128(ptype, ptype_mask);
	pinfo = _mm_and_si128(pinfo, pinfo_mask);
	pinfo = _mm_slli_epi32(pinfo, 12);
	ptype = _mm_or_si128(ptype, pinfo);
	pinfo = _mm_srli_epi32(ptype, 10);
	pkts[0]->packet_type = mlx5_ptype_table[_mm_extract_epi8(ptype, 0)];
	pkts[1]->packet_type = mlx5_ptype_table[_mm_extract_epi8(ptype, 4)];
	pkts[2]->packet_type = mlx5_ptype_table[_mm_extract_epi8(ptype, 8)];
	pkts[3]->packet_type = mlx5_ptype_table[_mm_extract_epi8(ptype, 12)];
	/* TODO: Fill ol_flags */
	rearm0 = _mm_blend_epi16(mbuf_init, mbuf_init, 0x01);
	rearm1 = _mm_blend_epi16(mbuf_init, mbuf_init, 0x02);
	rearm2 = _mm_blend_epi16(mbuf_init, mbuf_init, 0x04);
	rearm3 = _mm_blend_epi16(mbuf_init, mbuf_init, 0x08);
	_mm_store_si128((__m128i *)&pkts[0]->rearm_data, rearm0);
	_mm_store_si128((__m128i *)&pkts[1]->rearm_data, rearm1);
	_mm_store_si128((__m128i *)&pkts[2]->rearm_data, rearm2);
	_mm_store_si128((__m128i *)&pkts[3]->rearm_data, rearm3);
}

/**
 * DPDK callback for vectorized RX.
 *
 * @param dpdk_rxq
 *   Generic pointer to RX queue structure.
 * @param[out] pkts
 *   Array to store received packets.
 * @param pkts_n
 *   Maximum number of packets in array.
 *
 * @return
 *   Number of packets successfully received (<= pkts_n).
 */
uint16_t
mlx5_rx_burst_vec(void *dpdk_rxq, struct rte_mbuf **pkts, uint16_t pkts_n)
{
	struct rxq *rxq = dpdk_rxq;
	const uint16_t q_n = 1 << rxq->cqe_n;
	const uint16_t q_mask = q_n - 1;
	volatile struct mlx5_cqe *cq;
	struct rte_mbuf **elts;
	unsigned int pos;
	uint64_t comp_idx = MLX5_VPMD_DESCS_PER_LOOP;
	uint16_t nocmp_n = 0;
	uint16_t rcvd_pkt = 0;
	unsigned int cq_idx = rxq->cq_ci & q_mask;
	unsigned int elts_idx;
	unsigned int ownership = !!(rxq->cq_ci & (q_mask + 1));
	__m128i owner_check, opcode_check, format_check;
	__m128i shuf_mask, blend_mask;
	const __m128i zero = _mm_setzero_si128();
	const __m128i ones = _mm_cmpeq_epi32(zero, zero);
	const __m128i crc_adjust = _mm_set_epi16(
				0, 0, 0, 0, 0,
				rxq->crc_present * (-ETHER_CRC_LEN),
				0,
				rxq->crc_present * (-ETHER_CRC_LEN)
				);

	/* TODO: Consider striding for multi-seg packet? */
	assert(rxq->sges_n == 0);
	assert(rxq->cqe_n == rxq->elts_n);
	cq = &(*rxq->cqes)[cq_idx];
	rte_prefetch0(cq);
	rte_prefetch0(cq + 1);
	rte_prefetch0(cq + 2);
	rte_prefetch0(cq + 3);
	pkts_n = RTE_MIN(pkts_n, MLX5_VPMD_MAX_RX_BURST);
	/*
	 * Order of indexes:
	 *   rq_ci >= cq_ci >= rq_pi
	 * Definition of indexes:
	 *   rq_ci - cq_ci := # of buffers owned by HW (posted).
	 *   cq_ci - rq_pi := # of buffers not returned to app (uncompressed).
	 *   N - (rq_ci - rq_pi) := # of buffers consumed (to be replenished).
	 */
	if ((uint16_t)(q_n - (rxq->rq_ci - rxq->rq_pi)) >=
			MLX5_VPMD_RXQ_RPLNSH_THRESH)
		rxq_replenish_bulk_mbuf(rxq);
	/* See if there're unreturned mbufs from compressed CQE. */
	rcvd_pkt = rxq->cq_ci - rxq->rq_pi;
	if (rcvd_pkt > 0) {
		rcvd_pkt = RTE_MIN(rcvd_pkt, pkts_n);
		rxq_copy_mbuf_v(rxq, pkts, rcvd_pkt);
		rxq->rq_pi += rcvd_pkt;
		pkts += rcvd_pkt;
	}
	elts_idx = rxq->rq_pi & q_mask;
	elts = &(*rxq->elts)[elts_idx];
	pkts_n = RTE_MIN((uint16_t)(pkts_n - rcvd_pkt), q_n - elts_idx);
	/*
	 * TODO; Need to make pkts not overflow. May need to reserve a few NULL
	 * CQEs at the end of CQ.
	 *
	 * pkts_n = RTE_ALIGN_FLOOR(pkts_n, MLX5_VPMD_DESCS_PER_LOOP);
	 */
	if (!pkts_n) {
#ifdef MLX5_PMD_SOFT_COUNTERS
		/* Increment packets counter. */
		rxq->stats.ipackets += rcvd_pkt;
#endif
		return rcvd_pkt;
	}
	/* At this point, there shouldn't be any remained packets */
	assert(rxq->rq_pi == rxq->cq_ci);
	owner_check =
		_mm_set_epi64x(0x0100000001000000LL, 0x0100000001000000LL);
	opcode_check =
		_mm_set_epi64x(0xf0000000f0000000LL, 0xf0000000f0000000LL);
	format_check =
		_mm_set_epi64x(0x0c0000000c000000LL, 0x0c0000000c000000LL);
	/* Mask to blend from the last Qword to the first DQword */
	blend_mask = _mm_set_epi8(
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x80
		);
	/* Mask to shuffle from extracted CQE to mbuf */
	shuf_mask = _mm_set_epi8(
			0xFF, 1, 2, 3,  /* fdir.hi, bswap32 */
			12, 13, 14, 15, /* rss, bswap32 */
			10, 11,         /* vlan_tci, bswap16 */
			4, 5,           /* data_len, bswap16 */
			0xFF, 0xFF,     /* zero out 2nd half of pkt_len */
			4, 5            /* pkt_len, bswap16 */
			);
	/*
	 * A. load first Qword (8bytes) in one loop.
	 * B. copy 4 mbuf pointers from elts ring to returing pkts.
	 * C. load remained CQE data and extract necessary fields.
	 * D. fill in mbuf.
	 * E. get vaild CQEs.
	 * F. find compressed CQE.
	 */
	for (pos = 0; pos < pkts_n;
			pos += MLX5_VPMD_DESCS_PER_LOOP) {
		__m128i cqes[MLX5_VPMD_DESCS_PER_LOOP];
		__m128i cqe_tmp1, cqe_tmp2;
		__m128i pkt_mb0, pkt_mb1, pkt_mb2, pkt_mb3;
		__m128i op_own, op_own_tmp1, op_own_tmp2;
		__m128i owner_mask, invalid_mask, comp_mask;
		__m128i mbp1, mbp2;
		uint64_t n;

		/* Prefetch next 4 CQEs */
		rte_prefetch0(&cq[pos + MLX5_VPMD_DESCS_PER_LOOP]);
		rte_prefetch0(&cq[pos + MLX5_VPMD_DESCS_PER_LOOP + 1]);
		rte_prefetch0(&cq[pos + MLX5_VPMD_DESCS_PER_LOOP + 2]);
		rte_prefetch0(&cq[pos + MLX5_VPMD_DESCS_PER_LOOP + 3]);
		/* A.1 load cqes. */
		cqes[3] = _mm_loadl_epi64((__m128i *)&cq[pos + 3].sop_drop_qpn);
		rte_compiler_barrier();
		cqes[2] = _mm_loadl_epi64((__m128i *)&cq[pos + 2].sop_drop_qpn);
		rte_compiler_barrier();
		/* B.1 load mbuf pointers. */
		mbp1 = _mm_loadu_si128((__m128i *)&elts[pos]);
		mbp2 = _mm_loadu_si128((__m128i *)&elts[pos + 2]);
		/* A.1 load cqes. */
		cqes[1] = _mm_loadl_epi64((__m128i *)&cq[pos + 1].sop_drop_qpn);
		rte_compiler_barrier();
		cqes[0] = _mm_loadl_epi64((__m128i *)&cq[pos].sop_drop_qpn);
		/* B.2 copy mbuf pointers. */
		_mm_storeu_si128((__m128i *)&pkts[pos], mbp1);
		_mm_storeu_si128((__m128i *)&pkts[pos + 2], mbp2);
		rte_compiler_barrier();
		/* C.1 load remained CQE data and extract necessary fields. */
		cqe_tmp2 = _mm_loadu_si128((__m128i *)&cq[pos + 3]);
		cqe_tmp1 = _mm_loadu_si128((__m128i *)&cq[pos + 2]);
		cqes[3] = _mm_blendv_epi8(cqes[3], cqe_tmp2, blend_mask);
		cqes[2] = _mm_blendv_epi8(cqes[2], cqe_tmp1, blend_mask);
		cqe_tmp2 = _mm_loadu_si128((__m128i *)&cq[pos + 3].rsvd1[3]);
		cqe_tmp1 = _mm_loadu_si128((__m128i *)&cq[pos + 2].rsvd1[3]);
		cqes[3] = _mm_blend_epi16(cqes[3], cqe_tmp2, 0x30);
		cqes[2] = _mm_blend_epi16(cqes[2], cqe_tmp1, 0x30);
		cqe_tmp2 = _mm_loadl_epi64((__m128i *)&cq[pos + 3].rsvd2[10]);
		cqe_tmp1 = _mm_loadl_epi64((__m128i *)&cq[pos + 2].rsvd2[10]);
		cqes[3] = _mm_blend_epi16(cqes[3], cqe_tmp2, 0x04);
		cqes[2] = _mm_blend_epi16(cqes[2], cqe_tmp1, 0x04);
		/* C.2 generate final structure for mbuf with swapping bytes */
		pkt_mb3 = _mm_shuffle_epi8(cqes[3], shuf_mask);
		pkt_mb2 = _mm_shuffle_epi8(cqes[2], shuf_mask);
		/* C.3 adjust CRC length */
		pkt_mb3 = _mm_add_epi16(pkt_mb3, crc_adjust);
		pkt_mb2 = _mm_add_epi16(pkt_mb2, crc_adjust);
		/* D.1 fill in mbuf - rx_descriptor_fields1. */
		_mm_storeu_si128((void *)&pkts[pos + 3]->pkt_len, pkt_mb3);
		_mm_storeu_si128((void *)&pkts[pos + 2]->pkt_len, pkt_mb2);
		/* E.1 extract op_own field. */
		op_own_tmp2 = _mm_unpacklo_epi32(cqes[2], cqes[3]);
		/* C.1 load remained CQE data and extract necessary fields. */
		cqe_tmp2 = _mm_loadu_si128((__m128i *)&cq[pos + 1]);
		cqe_tmp1 = _mm_loadu_si128((__m128i *)&cq[pos]);
		cqes[1] = _mm_blendv_epi8(cqes[1], cqe_tmp2, blend_mask);
		cqes[0] = _mm_blendv_epi8(cqes[0], cqe_tmp1, blend_mask);
		cqe_tmp2 = _mm_loadu_si128((__m128i *)&cq[pos + 1].rsvd1[3]);
		cqe_tmp1 = _mm_loadu_si128((__m128i *)&cq[pos].rsvd1[3]);
		cqes[1] = _mm_blend_epi16(cqes[1], cqe_tmp2, 0x30);
		cqes[0] = _mm_blend_epi16(cqes[0], cqe_tmp1, 0x30);
		cqe_tmp2 = _mm_loadl_epi64((__m128i *)&cq[pos + 1].rsvd2[10]);
		cqe_tmp1 = _mm_loadl_epi64((__m128i *)&cq[pos].rsvd2[10]);
		cqes[1] = _mm_blend_epi16(cqes[1], cqe_tmp2, 0x04);
		cqes[0] = _mm_blend_epi16(cqes[0], cqe_tmp1, 0x04);
		/* C.2 generate final structure for mbuf with swapping bytes */
		pkt_mb1 = _mm_shuffle_epi8(cqes[1], shuf_mask);
		pkt_mb0 = _mm_shuffle_epi8(cqes[0], shuf_mask);
		/* C.3 adjust CRC length */
		pkt_mb1 = _mm_add_epi16(pkt_mb1, crc_adjust);
		pkt_mb0 = _mm_add_epi16(pkt_mb0, crc_adjust);
		/* E.1 extract op_own byte. */
		op_own_tmp1 = _mm_unpacklo_epi32(cqes[0], cqes[1]);
		op_own = _mm_unpackhi_epi64(op_own_tmp1, op_own_tmp2);
		/* D.1 fill in mbuf - rx_descriptor_fields1. */
		_mm_storeu_si128((void *)&pkts[pos + 1]->pkt_len, pkt_mb1);
		_mm_storeu_si128((void *)&pkts[pos]->pkt_len, pkt_mb0);
		/* D.1 fill in mbuf - rearm_data and packet_type. */
		rxq_cq_to_ptype_oflags_v(rxq, cqes, &pkts[pos]);
		/* E.2 flip owner bit to mark invalid CQEs. */
		owner_mask = _mm_and_si128(op_own, owner_check);
		if (ownership)
			owner_mask = _mm_xor_si128(owner_mask, owner_check);
		owner_mask = _mm_cmpeq_epi32(owner_mask, owner_check);
		owner_mask = _mm_packs_epi32(owner_mask, zero);
		/* E.3 get mask for invalidated CQEs */
		invalid_mask = _mm_cmpeq_epi32(opcode_check,
				_mm_and_si128(op_own, opcode_check));
		invalid_mask = _mm_packs_epi32(invalid_mask, zero);
		/* E.4 mask out beyond boundary */
		invalid_mask = _mm_or_si128(
				invalid_mask,
				ones <<
				 ((pkts_n - pos) * sizeof(uint16_t) * 8));
		/* E.5 merge invalid_mask with invalid owner */
		invalid_mask = _mm_or_si128(invalid_mask, owner_mask);
		/* F.1 find compressed CQE format. */
		comp_mask = _mm_and_si128(op_own, format_check);
		comp_mask = _mm_cmpeq_epi32(comp_mask, format_check);
		comp_mask = _mm_packs_epi32(comp_mask, zero);
		/* F.2 mask out invalid entries */
		comp_mask = _mm_andnot_si128(invalid_mask, comp_mask);
		comp_idx = _mm_cvtsi128_si64(comp_mask);
		/* F.3 get the first compressed CQE */
		comp_idx = comp_idx ?
				__builtin_ctzll(comp_idx) /
					(sizeof(uint16_t) * 8) :
				MLX5_VPMD_DESCS_PER_LOOP;
		/* E.6 mask out entries after the compressed CQE */
		invalid_mask = _mm_or_si128(
				invalid_mask,
				ones <<
				 (comp_idx * sizeof(uint16_t) * 8));
		/* E.7 count non-compressed valid CQEs. */
		n = _mm_cvtsi128_si64(invalid_mask);
		n = n ? __builtin_ctzll(n) / (sizeof(uint16_t) * 8) :
			MLX5_VPMD_DESCS_PER_LOOP;
		nocmp_n += n;
		if (likely(n != MLX5_VPMD_DESCS_PER_LOOP))
			break;
	}
	/* If no new CQE seen, return without updating cq_db. */
	if (unlikely(!nocmp_n && comp_idx == MLX5_VPMD_DESCS_PER_LOOP))
		return rcvd_pkt;
	/* Update the consumer indexes for non-compressed CQEs. */
	assert(nocmp_n <= pkts_n);
	rxq->cq_ci += nocmp_n;
	rxq->rq_pi += nocmp_n;
	rcvd_pkt += nocmp_n;
	/* Uncompress the last CQE if compressed. */
	if (comp_idx < MLX5_VPMD_DESCS_PER_LOOP) {
		assert(comp_idx == (nocmp_n % MLX5_VPMD_DESCS_PER_LOOP));
		rxq_cq_uncompress_v(rxq, &cq[nocmp_n], &elts[nocmp_n]);
		/* Return more packets if needed */
		if (nocmp_n < pkts_n) {
			uint16_t n = rxq->cq_ci - rxq->rq_pi;

			n = RTE_MIN(n, pkts_n - nocmp_n);
			rxq_copy_mbuf_v(rxq, &pkts[nocmp_n], n);
			rxq->rq_pi += n;
			rcvd_pkt += n;
		}
	}
	rte_wmb();
	*rxq->cq_db = htonl(rxq->cq_ci);
#ifdef MLX5_PMD_SOFT_COUNTERS
	/* Increment packets counter. */
	rxq->stats.ipackets += rcvd_pkt;
#endif
	return rcvd_pkt;
}

#ifndef __INTEL_COMPILER
#pragma GCC diagnostic pop
#endif
#endif	/* MLX5_VECTORIZED_RX */

/**
 * DPDK callback for RX.
 *
 * @param dpdk_rxq
 *   Generic pointer to RX queue structure.
 * @param[out] pkts
 *   Array to store received packets.
 * @param pkts_n
 *   Maximum number of packets in array.
 *
 * @return
 *   Number of packets successfully received (<= pkts_n).
 */
uint16_t
mlx5_rx_burst(void *dpdk_rxq, struct rte_mbuf **pkts, uint16_t pkts_n)
{
	struct rxq *rxq = dpdk_rxq;
	const unsigned int wqe_cnt = (1 << rxq->elts_n) - 1;
	const unsigned int cqe_cnt = (1 << rxq->cqe_n) - 1;
	const unsigned int sges_n = rxq->sges_n;
	struct rte_mbuf *pkt = NULL;
	struct rte_mbuf *seg = NULL;
	volatile struct mlx5_cqe *cqe =
		&(*rxq->cqes)[rxq->cq_ci & cqe_cnt];
	unsigned int i = 0;
	unsigned int rq_ci = rxq->rq_ci << sges_n;
	int len = 0; /* keep its value across iterations. */

	while (pkts_n) {
		unsigned int idx = rq_ci & wqe_cnt;
		volatile struct mlx5_wqe_data_seg *wqe = &(*rxq->wqes)[idx];
		struct rte_mbuf *rep = (*rxq->elts)[idx];
		uint32_t rss_hash_res = 0;

		if (pkt)
			NEXT(seg) = rep;
		seg = rep;
		rte_prefetch0(seg);
		rte_prefetch0(cqe);
		rte_prefetch0(wqe);
		rep = rte_mbuf_raw_alloc(rxq->mp);
		if (unlikely(rep == NULL)) {
			++rxq->stats.rx_nombuf;
			if (!pkt) {
				/*
				 * no buffers before we even started,
				 * bail out silently.
				 */
				break;
			}
			while (pkt != seg) {
				assert(pkt != (*rxq->elts)[idx]);
				rep = NEXT(pkt);
				NEXT(pkt) = NULL;
				NB_SEGS(pkt) = 1;
				rte_mbuf_raw_free(pkt);
				pkt = rep;
			}
			break;
		}
		if (!pkt) {
			cqe = &(*rxq->cqes)[rxq->cq_ci & cqe_cnt];
			len = mlx5_rx_poll_len(rxq, cqe, cqe_cnt,
					       &rss_hash_res);
			if (!len) {
				rte_mbuf_raw_free(rep);
				break;
			}
			if (unlikely(len == -1)) {
				/* RX error, packet is likely too large. */
				rte_mbuf_raw_free(rep);
				++rxq->stats.idropped;
				goto skip;
			}
			pkt = seg;
			assert(len >= (rxq->crc_present << 2));
			/* Update packet information. */
			pkt->packet_type = 0;
			pkt->ol_flags = 0;
			if (rss_hash_res && rxq->rss_hash) {
				pkt->hash.rss = rss_hash_res;
				pkt->ol_flags = PKT_RX_RSS_HASH;
			}
			if (rxq->mark &&
			    MLX5_FLOW_MARK_IS_VALID(cqe->sop_drop_qpn)) {
				pkt->ol_flags |= PKT_RX_FDIR;
				if (cqe->sop_drop_qpn !=
				    htonl(MLX5_FLOW_MARK_DEFAULT)) {
					uint32_t mark = cqe->sop_drop_qpn;

					pkt->ol_flags |= PKT_RX_FDIR_ID;
					pkt->hash.fdir.hi =
						mlx5_flow_mark_get(mark);
				}
			}
			if (rxq->csum | rxq->csum_l2tun) {
				pkt->packet_type = rxq_cq_to_pkt_type(cqe);
				pkt->ol_flags |= rxq_cq_to_ol_flags(rxq, cqe);
			}
			if (rxq->vlan_strip &&
			    (cqe->hdr_type_etc &
			     htons(MLX5_CQE_VLAN_STRIPPED))) {
				pkt->ol_flags |= PKT_RX_VLAN_PKT |
					PKT_RX_VLAN_STRIPPED;
				pkt->vlan_tci = ntohs(cqe->vlan_info);
			}
			if (rxq->crc_present)
				len -= ETHER_CRC_LEN;
			PKT_LEN(pkt) = len;
		}
		DATA_LEN(rep) = DATA_LEN(seg);
		PKT_LEN(rep) = PKT_LEN(seg);
		SET_DATA_OFF(rep, DATA_OFF(seg));
		NB_SEGS(rep) = NB_SEGS(seg);
		PORT(rep) = PORT(seg);
		NEXT(rep) = NULL;
		(*rxq->elts)[idx] = rep;
		/*
		 * Fill NIC descriptor with the new buffer.  The lkey and size
		 * of the buffers are already known, only the buffer address
		 * changes.
		 */
		wqe->addr = htonll(rte_pktmbuf_mtod(rep, uintptr_t));
		if (len > DATA_LEN(seg)) {
			len -= DATA_LEN(seg);
			++NB_SEGS(pkt);
			++rq_ci;
			continue;
		}
		DATA_LEN(seg) = len;
#ifdef MLX5_PMD_SOFT_COUNTERS
		/* Increment bytes counter. */
		rxq->stats.ibytes += PKT_LEN(pkt);
#endif
		/* Return packet. */
		*(pkts++) = pkt;
		pkt = NULL;
		--pkts_n;
		++i;
skip:
		/* Align consumer index to the next stride. */
		rq_ci >>= sges_n;
		++rq_ci;
		rq_ci <<= sges_n;
	}
	if (unlikely((i == 0) && ((rq_ci >> sges_n) == rxq->rq_ci)))
		return 0;
	/* Update the consumer index. */
	rxq->rq_ci = rq_ci >> sges_n;
	rte_wmb();
	*rxq->cq_db = htonl(rxq->cq_ci);
	rte_wmb();
	*rxq->rq_db = htonl(rxq->rq_ci);
#ifdef MLX5_PMD_SOFT_COUNTERS
	/* Increment packets counter. */
	rxq->stats.ipackets += i;
#endif
	return i;
}

/**
 * Dummy DPDK callback for TX.
 *
 * This function is used to temporarily replace the real callback during
 * unsafe control operations on the queue, or in case of error.
 *
 * @param dpdk_txq
 *   Generic pointer to TX queue structure.
 * @param[in] pkts
 *   Packets to transmit.
 * @param pkts_n
 *   Number of packets in array.
 *
 * @return
 *   Number of packets successfully transmitted (<= pkts_n).
 */
uint16_t
removed_tx_burst(void *dpdk_txq, struct rte_mbuf **pkts, uint16_t pkts_n)
{
	(void)dpdk_txq;
	(void)pkts;
	(void)pkts_n;
	return 0;
}

/**
 * Dummy DPDK callback for RX.
 *
 * This function is used to temporarily replace the real callback during
 * unsafe control operations on the queue, or in case of error.
 *
 * @param dpdk_rxq
 *   Generic pointer to RX queue structure.
 * @param[out] pkts
 *   Array to store received packets.
 * @param pkts_n
 *   Maximum number of packets in array.
 *
 * @return
 *   Number of packets successfully received (<= pkts_n).
 */
uint16_t
removed_rx_burst(void *dpdk_rxq, struct rte_mbuf **pkts, uint16_t pkts_n)
{
	(void)dpdk_rxq;
	(void)pkts;
	(void)pkts_n;
	return 0;
}

/**
 * DPDK callback for rx queue interrupt enable.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param rx_queue_id
 *   RX queue number
 *
 * @return
 *   0 on success, negative on failure.
 */
int
mlx5_rx_intr_enable(struct rte_eth_dev *dev, uint16_t rx_queue_id)
{
#ifdef HAVE_UPDATE_CQ_CI
	struct priv *priv = mlx5_get_priv(dev);
	struct rxq *rxq = (*priv->rxqs)[rx_queue_id];
	struct rxq_ctrl *rxq_ctrl = container_of(rxq, struct rxq_ctrl, rxq);
	struct ibv_cq *cq = rxq_ctrl->cq;
	uint16_t ci = rxq->cq_ci;
	int ret = 0;

	ibv_mlx5_exp_update_cq_ci(cq, ci);
	ret = ibv_req_notify_cq(cq, 0);
#else
	int ret = -1;
	(void)dev;
	(void)rx_queue_id;
#endif
	if (ret)
		WARN("unable to arm interrupt on rx queue %d", rx_queue_id);
	return ret;
}

/**
 * DPDK callback for rx queue interrupt disable.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param rx_queue_id
 *   RX queue number
 *
 * @return
 *   0 on success, negative on failure.
 */
int
mlx5_rx_intr_disable(struct rte_eth_dev *dev, uint16_t rx_queue_id)
{
#ifdef HAVE_UPDATE_CQ_CI
	struct priv *priv = mlx5_get_priv(dev);
	struct rxq *rxq = (*priv->rxqs)[rx_queue_id];
	struct rxq_ctrl *rxq_ctrl = container_of(rxq, struct rxq_ctrl, rxq);
	struct ibv_cq *cq = rxq_ctrl->cq;
	struct ibv_cq *ev_cq;
	void *ev_ctx;
	int ret = 0;

	ret = ibv_get_cq_event(cq->channel, &ev_cq, &ev_ctx);
	if (ret || ev_cq != cq)
		ret = -1;
	else
		ibv_ack_cq_events(cq, 1);
#else
	int ret = -1;
	(void)dev;
	(void)rx_queue_id;
#endif
	if (ret)
		WARN("unable to disable interrupt on rx queue %d",
		     rx_queue_id);
	return ret;
}
