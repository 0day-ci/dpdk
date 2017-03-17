/*-
 * BSD LICENSE
 *
 * Copyright (c) 2015-2017 Atomic Rules LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * * Neither the name of copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <unistd.h>

#include "ark_global.h"
#include "ark_debug.h"
#include "ark_ethdev.h"
#include "ark_mpu.h"
#include "ark_udm.h"

#define ARK_RX_META_SIZE 32
#define ARK_RX_META_OFFSET (RTE_PKTMBUF_HEADROOM - ARK_RX_META_SIZE)
#define ARK_RX_MAX_NOCHAIN (RTE_MBUF_DEFAULT_DATAROOM)

#ifdef RTE_LIBRTE_ARK_DEBUG_RX
#define ARK_RX_DEBUG 1
#define ARK_FULL_DEBUG 1
#else
#define ARK_RX_DEBUG 0
#define ARK_FULL_DEBUG 0
#endif

/* Forward declarations */
struct ark_rx_queue;
struct ark_rx_meta;

static void dump_mbuf_data(struct rte_mbuf *mbuf, uint16_t lo, uint16_t hi);
static void ark_ethdev_rx_dump(const char *name, struct ark_rx_queue *queue);
static uint32_t eth_ark_rx_jumbo(struct ark_rx_queue *queue,
	struct ark_rx_meta *meta, struct rte_mbuf *mbuf0, uint32_t consIndex);
static inline int eth_ark_rx_seed_mbufs(struct ark_rx_queue *queue);

/* ************************************************************************* */
struct ark_rx_queue {

	/* array of mbufs to populate */
	struct rte_mbuf **reserveQ;
	/* array of physical addrresses of the mbuf data pointer */
	/* This point is a virtual address */
	phys_addr_t *paddressQ;
	struct rte_mempool *mb_pool;

	struct ark_udm_t *udm;
	struct ark_mpu_t *mpu;

	uint32_t queueSize;
	uint32_t queueMask;

	uint32_t seedIndex;		/* 1 set with an empty mbuf */
	uint32_t consIndex;		/* 3 consumed by the driver */

	/* The queue Id is used to identify the HW Q */
	uint16_t phys_qid;

	/* The queue Index is used within the dpdk device structures */
	uint16_t queueIndex;

	uint32_t pad1;

	/* separate cache line */
	/* second cache line - fields only used in slow path */
	MARKER cacheline1 __rte_cache_min_aligned;

	volatile uint32_t prodIndex;	/* 2 filled by the HW */

} __rte_cache_aligned;

/* ************************************************************************* */

/* MATCHES struct in UDMDefines.bsv */

/* TODO move to ark_udm.h */
struct ark_rx_meta {
	uint64_t timestamp;
	uint64_t userData;
	uint8_t port;
	uint8_t dstQueue;
	uint16_t pktLen;
};

/* ************************************************************************* */

/* TODO  pick a better function name */
static int
eth_ark_rx_queue_setup(struct rte_eth_dev *dev,
	struct ark_rx_queue *queue,
	uint16_t rx_queue_id __rte_unused, uint16_t rx_queue_idx)
{
	phys_addr_t queueBase;
	phys_addr_t physAddrQBase;
	phys_addr_t physAddrProdIndex;

	queueBase = rte_malloc_virt2phy(queue);
	physAddrProdIndex = queueBase +
		offsetof(struct ark_rx_queue, prodIndex);

	physAddrQBase = rte_malloc_virt2phy(queue->paddressQ);

	/* Verify HW */
	if (ark_mpu_verify(queue->mpu, sizeof(phys_addr_t))) {
	PMD_DRV_LOG(ERR, "ARKP: Illegal configuration rx queue\n");
	return -1;
	}

	/* Stop and Reset and configure MPU */
	ark_mpu_configure(queue->mpu, physAddrQBase, queue->queueSize, 0);

	ark_udm_write_addr(queue->udm, physAddrProdIndex);

	/* advance the valid pointer, but don't start until the queue starts */
	ark_mpu_reset_stats(queue->mpu);

	/* The seed is the producer index for the HW */
	ark_mpu_set_producer(queue->mpu, queue->seedIndex);
	dev->data->rx_queue_state[rx_queue_idx] = RTE_ETH_QUEUE_STATE_STOPPED;

	return 0;
}

static inline void
eth_ark_rx_update_consIndex(struct ark_rx_queue *queue, uint32_t consIndex)
{
	queue->consIndex = consIndex;
	eth_ark_rx_seed_mbufs(queue);
	ark_mpu_set_producer(queue->mpu, queue->seedIndex);
}

/* ************************************************************************* */
int
eth_ark_dev_rx_queue_setup(struct rte_eth_dev *dev,
	uint16_t queue_idx,
	uint16_t nb_desc,
	unsigned int socket_id,
	const struct rte_eth_rxconf *rx_conf, struct rte_mempool *mb_pool)
{
	struct ark_adapter *ark = (struct ark_adapter *) dev->data->dev_private;
	static int warning1;		/* = 0 */

	struct ark_rx_queue *queue;
	uint32_t i;
	int status;

	int port = ark_get_port_id(dev, ark);
	int qidx = port + queue_idx;	/* TODO FIXME */

	// TODO: We may already be setup, check here if there is nothing to do
	/* Free memory prior to re-allocation if needed */
	if (dev->data->rx_queues[queue_idx] != NULL) {
	// TODO: release any allocated queues
	dev->data->rx_queues[queue_idx] = NULL;
	}

	if (rx_conf != NULL && warning1 == 0) {
	warning1 = 1;
	PMD_DRV_LOG(INFO,
		"ARKP: Arkville PMD ignores  rte_eth_rxconf argument.\n");
	}

	if (RTE_PKTMBUF_HEADROOM < ARK_RX_META_SIZE) {
	PMD_DRV_LOG(ERR,
		"Error: DPDK Arkville requires head room > %d bytes (%s)\n",
		ARK_RX_META_SIZE, __func__);
	return -1;		/* ERROR CODE */
	}

	if (!rte_is_power_of_2(nb_desc)) {
	PMD_DRV_LOG(ERR,
		"DPDK Arkville configuration queue size must be power of two %u (%s)\n",
		nb_desc, __func__);
	return -1;		/* ERROR CODE */
	}

	/* Allocate queue struct */
	queue =
	rte_zmalloc_socket("ArkRXQueue", sizeof(struct ark_rx_queue), 64,
	socket_id);
	if (queue == 0) {
	PMD_DRV_LOG(ERR, "Failed to allocate memory in %s\n", __func__);
	return -ENOMEM;
	}

	/* NOTE zmalloc is used, no need to 0 indexes, etc. */
	queue->mb_pool = mb_pool;
	queue->phys_qid = qidx;
	queue->queueIndex = queue_idx;
	queue->queueSize = nb_desc;
	queue->queueMask = nb_desc - 1;

	queue->reserveQ =
	rte_zmalloc_socket("ArkRXQueue mbuf",
	nb_desc * sizeof(struct rte_mbuf *), 64, socket_id);
	queue->paddressQ =
	rte_zmalloc_socket("ArkRXQueue paddr", nb_desc * sizeof(phys_addr_t),
	64, socket_id);
	if (queue->reserveQ == 0 || queue->paddressQ == 0) {
	PMD_DRV_LOG(ERR, "Failed to allocate queue memory in %s\n", __func__);
	rte_free(queue->reserveQ);
	rte_free(queue->paddressQ);
	rte_free(queue);
	return -ENOMEM;
	}

	dev->data->rx_queues[queue_idx] = queue;
	queue->udm = RTE_PTR_ADD(ark->udm.v, qidx * ARK_UDM_QOFFSET);
	queue->mpu = RTE_PTR_ADD(ark->mpurx.v, qidx * ARK_MPU_QOFFSET);

	/* populate mbuf reserve */
	status = eth_ark_rx_seed_mbufs(queue);

	/* MPU Setup */
	if (status == 0)
		status = eth_ark_rx_queue_setup(dev, queue, qidx, queue_idx);

	if (unlikely(status != 0)) {
	struct rte_mbuf *mbuf;

	PMD_DRV_LOG(ERR, "ARKP Failed to initialize RX queue %d %s\n", qidx,
		__func__);
	/* Free the mbufs allocated */
	for (i = 0, mbuf = queue->reserveQ[0]; i < nb_desc; ++i, mbuf++) {
		if (mbuf != 0)
			rte_pktmbuf_free(mbuf);
	}
	rte_free(queue->reserveQ);
	rte_free(queue->paddressQ);
	rte_free(queue);
	return -1;		/* ERROR CODE */
	}

	return 0;
}

/* ************************************************************************* */
uint16_t
eth_ark_recv_pkts_noop(void *rx_queue __rte_unused,
	struct rte_mbuf **rx_pkts __rte_unused, uint16_t nb_pkts __rte_unused)
{
	return 0;
}

/* ************************************************************************* */
uint16_t
eth_ark_recv_pkts(void *rx_queue, struct rte_mbuf **rx_pkts, uint16_t nb_pkts)
{
	struct ark_rx_queue *queue;
	register uint32_t consIndex, prodIndex;
	uint16_t nb;
	uint64_t rx_bytes = 0;
	struct rte_mbuf *mbuf;
	struct ark_rx_meta *meta;

	queue = (struct ark_rx_queue *) rx_queue;
	if (unlikely(queue == 0))
	return 0;
	if (unlikely(nb_pkts == 0))
	return 0;
	prodIndex = queue->prodIndex;
	consIndex = queue->consIndex;
	nb = 0;

	while (prodIndex != consIndex) {
		mbuf = queue->reserveQ[consIndex & queue->queueMask];
		/* prefetch mbuf ? */
		rte_mbuf_prefetch_part1(mbuf);
		rte_mbuf_prefetch_part2(mbuf);

		/* META DATA burried in buffer */
		meta = RTE_PTR_ADD(mbuf->buf_addr, ARK_RX_META_OFFSET);

		mbuf->port = meta->port;
		mbuf->pkt_len = meta->pktLen;
		mbuf->data_len = meta->pktLen;
		mbuf->data_off = RTE_PKTMBUF_HEADROOM;
		mbuf->udata64 = meta->userData;
		if (ARK_RX_DEBUG) {	/* debug use */
			if ((meta->pktLen > (1024 * 16)) ||
				(meta->pktLen == 0)) {
				PMD_DRV_LOG(INFO,
						"ARKP RX: Bad Meta Q: %u cons: %u prod: %u\n",
						queue->phys_qid,
						consIndex,
						queue->prodIndex);

				PMD_DRV_LOG(INFO, "       :  cons: %u prod: %u seedIndex %u\n",
						consIndex,
						queue->prodIndex,
						queue->seedIndex);

				PMD_DRV_LOG(INFO, "       :  UDM prod: %u  len: %u\n",
						queue->udm->rt_cfg.prodIdx,
						meta->pktLen);
				ark_mpu_dump(queue->mpu,
							 "    ",
							 queue->phys_qid);

				dump_mbuf_data(mbuf, 0, 256);
				/* its FUBAR so fix it */
				mbuf->pkt_len = 63;
				meta->pktLen = 63;
			}
			mbuf->seqn = consIndex;
		}

		rx_bytes += meta->pktLen;	/* TEMP stats */

		if (unlikely(meta->pktLen > ARK_RX_MAX_NOCHAIN))
			consIndex = eth_ark_rx_jumbo
				(queue, meta, mbuf, consIndex + 1);
		else
			consIndex += 1;

		rx_pkts[nb] = mbuf;
		nb++;
		if (nb >= nb_pkts)
			break;
	}

	if (unlikely(nb != 0))
		/* report next free to FPGA */
		eth_ark_rx_update_consIndex(queue, consIndex);

	return nb;
}

/* ************************************************************************* */
static uint32_t
eth_ark_rx_jumbo(struct ark_rx_queue *queue,
	struct ark_rx_meta *meta, struct rte_mbuf *mbuf0, uint32_t consIndex)
{
	struct rte_mbuf *mbuf_prev;
	struct rte_mbuf *mbuf;

	uint16_t remaining;
	uint16_t data_len;
	uint8_t segments;

	/* first buf populated by called */
	mbuf_prev = mbuf0;
	segments = 1;
	data_len = RTE_MIN(meta->pktLen, RTE_MBUF_DEFAULT_DATAROOM);
	remaining = meta->pktLen - data_len;
	mbuf0->data_len = data_len;

	/* TODO check that the data does not exceed prodIndex! */
	while (remaining != 0) {
		data_len =
			RTE_MIN(remaining,
					RTE_MBUF_DEFAULT_DATAROOM +
					RTE_PKTMBUF_HEADROOM);

		remaining -= data_len;
		segments += 1;

		mbuf = queue->reserveQ[consIndex & queue->queueMask];
		mbuf_prev->next = mbuf;
		mbuf_prev = mbuf;
		mbuf->data_len = data_len;
		mbuf->data_off = 0;
		if (ARK_RX_DEBUG)
			mbuf->seqn = consIndex;	/* for debug only */

		consIndex += 1;
	}

	mbuf0->nb_segs = segments;
	return consIndex;
}

/* Drain the internal queue allowing hw to clear out. */
static void
eth_ark_rx_queue_drain(struct ark_rx_queue *queue)
{
	register uint32_t consIndex;
	struct rte_mbuf *mbuf;

	consIndex = queue->consIndex;

	/* NOT performance optimized, since this is a one-shot call */
	while ((consIndex ^ queue->prodIndex) & queue->queueMask) {
		mbuf = queue->reserveQ[consIndex & queue->queueMask];
		rte_pktmbuf_free(mbuf);
		consIndex++;
		eth_ark_rx_update_consIndex(queue, consIndex);
	}
}

uint32_t
eth_ark_dev_rx_queue_count(struct rte_eth_dev *dev, uint16_t queue_id)
{
	struct ark_rx_queue *queue;

	queue = dev->data->rx_queues[queue_id];
	return (queue->prodIndex - queue->consIndex);	/* mod arith */
}

/* ************************************************************************* */
int
eth_ark_rx_start_queue(struct rte_eth_dev *dev, uint16_t queue_id)
{
	struct ark_rx_queue *queue;

	queue = dev->data->rx_queues[queue_id];
	if (queue == 0)
	return -1;

	dev->data->rx_queue_state[queue_id] = RTE_ETH_QUEUE_STATE_STARTED;

	ark_mpu_set_producer(queue->mpu, queue->seedIndex);
	ark_mpu_start(queue->mpu);

	ark_udm_queue_enable(queue->udm, 1);

	return 0;
}

/* ************************************************************************* */

/* Queue can be restarted.   data remains
 */
int
eth_ark_rx_stop_queue(struct rte_eth_dev *dev, uint16_t queue_id)
{
	struct ark_rx_queue *queue;

	queue = dev->data->rx_queues[queue_id];
	if (queue == 0)
	return -1;

	ark_udm_queue_enable(queue->udm, 0);

	dev->data->rx_queue_state[queue_id] = RTE_ETH_QUEUE_STATE_STOPPED;

	return 0;
}

/* ************************************************************************* */
static inline int
eth_ark_rx_seed_mbufs(struct ark_rx_queue *queue)
{
	uint32_t limit = queue->consIndex + queue->queueSize;
	uint32_t seedIndex = queue->seedIndex;

	uint32_t count = 0;
	uint32_t seedM = queue->seedIndex & queue->queueMask;

	uint32_t nb = limit - seedIndex;

	/* Handle wrap around -- remainder is filled on the next call */
	if (unlikely(seedM + nb > queue->queueSize))
		nb = queue->queueSize - seedM;

	struct rte_mbuf **mbufs = &queue->reserveQ[seedM];
	int status = rte_pktmbuf_alloc_bulk(queue->mb_pool, mbufs, nb);

	if (unlikely(status != 0))
		return -1;

	if (ARK_RX_DEBUG) {		/* DEBUG */
		while (count != nb) {
			struct rte_mbuf *mbuf_init =
				queue->reserveQ[seedM + count];

			memset(mbuf_init->buf_addr, -1, 512);
			*((uint32_t *) mbuf_init->buf_addr) = seedIndex + count;
			*(uint16_t *) RTE_PTR_ADD(mbuf_init->buf_addr, 4) =
				queue->phys_qid;
			count++;
		}
		count = 0;
	}
	/* DEBUG */
	queue->seedIndex += nb;

	/* Duff's device https://en.wikipedia.org/wiki/Duff's_device */
	switch (nb % 4) {
	case 0:
	while (count != nb) {
		queue->paddressQ[seedM++] = (*mbufs++)->buf_physaddr;
		count++;
		/* FALLTHROUGH */
	case 3:
		queue->paddressQ[seedM++] = (*mbufs++)->buf_physaddr;
		count++;
		/* FALLTHROUGH */
	case 2:
		queue->paddressQ[seedM++] = (*mbufs++)->buf_physaddr;
		count++;
		/* FALLTHROUGH */
	case 1:
		queue->paddressQ[seedM++] = (*mbufs++)->buf_physaddr;
		count++;
		/* FALLTHROUGH */

	} /* while (count != nb) */
	} /* switch */

	return 0;
}

void
eth_ark_rx_dump_queue(struct rte_eth_dev *dev, uint16_t queue_id,
	const char *msg)
{
	struct ark_rx_queue *queue;

	queue = dev->data->rx_queues[queue_id];

	ark_ethdev_rx_dump(msg, queue);
}

/* ************************************************************************* */

/* Call on device closed no user API, queue is stopped */
void
eth_ark_dev_rx_queue_release(void *vqueue)
{
	struct ark_rx_queue *queue;
	uint32_t i;

	queue = (struct ark_rx_queue *) vqueue;
	if (queue == 0)
		return;

	ark_udm_queue_enable(queue->udm, 0);
	/* Stop the MPU since pointer are going away */
	ark_mpu_stop(queue->mpu);

	/* Need to clear out mbufs here, dropping packets along the way */
	eth_ark_rx_queue_drain(queue);

	for (i = 0; i < queue->queueSize; ++i)
		rte_pktmbuf_free(queue->reserveQ[i]);

	rte_free(queue->reserveQ);
	rte_free(queue->paddressQ);
	rte_free(queue);
}

void
eth_rx_queue_stats_get(void *vqueue, struct rte_eth_stats *stats)
{
	struct ark_rx_queue *queue;
	struct ark_udm_t *udm;

	queue = vqueue;
	if (queue == 0)
	return;
	udm = queue->udm;

	uint64_t ibytes = ark_udm_bytes(udm);
	uint64_t ipackets = ark_udm_packets(udm);
	uint64_t idropped = ark_udm_dropped(queue->udm);

	stats->q_ipackets[queue->queueIndex] = ipackets;
	stats->q_ibytes[queue->queueIndex] = ibytes;
	stats->q_errors[queue->queueIndex] = idropped;
	stats->ipackets += ipackets;
	stats->ibytes += ibytes;
	stats->imissed += idropped;
}

void
eth_rx_queue_stats_reset(void *vqueue)
{
	struct ark_rx_queue *queue;

	queue = vqueue;
	if (queue == 0)
		return;

	ark_mpu_reset_stats(queue->mpu);
	ark_udm_queue_stats_reset(queue->udm);
}

void
eth_ark_udm_force_close(struct rte_eth_dev *dev)
{
	struct ark_adapter *ark = (struct ark_adapter *) dev->data->dev_private;
	struct ark_rx_queue *queue;
	uint32_t index;
	uint16_t i;

	if (!ark_udm_is_flushed(ark->udm.v)) {
	/* restart the MPUs */
	fprintf(stderr, "ARK: %s UDM not flushed\n", __func__);
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		queue = (struct ark_rx_queue *) dev->data->rx_queues[i];
		if (queue == 0)
		continue;

		ark_mpu_start(queue->mpu);
		/* Add some buffers */
		index = 100000 + queue->seedIndex;
		ark_mpu_set_producer(queue->mpu, index);
	}
	/* Wait to allow data to pass */
	usleep(100);

	ARK_DEBUG_TRACE("UDM forced flush attempt, stopped = %d\n",
		ark_udm_is_flushed(ark->udm.v));
	}
	ark_udm_reset(ark->udm.v);

}

static void
ark_ethdev_rx_dump(const char *name, struct ark_rx_queue *queue)
{
	if (queue == NULL)
	return;
	ARK_DEBUG_TRACE("RX QUEUE %d -- %s", queue->phys_qid, name);
	ARK_DEBUG_TRACE(FMT_SU32 FMT_SU32 FMT_SU32 FMT_SU32 "\n",
	"queueSize", queue->queueSize,
	"seedIndex", queue->seedIndex,
	"prodIndex", queue->prodIndex, "consIndex", queue->consIndex);

	ark_mpu_dump(queue->mpu, name, queue->phys_qid);
	ark_mpu_dump_setup(queue->mpu, queue->phys_qid);
	ark_udm_dump(queue->udm, name);
	ark_udm_dump_setup(queue->udm, queue->phys_qid);

}

static void
dump_mbuf_data(struct rte_mbuf *mbuf, uint16_t lo, uint16_t hi)
{
	uint16_t i, j;

	fprintf(stderr, " MBUF: %p len %d, off: %d, seq: %u\n", mbuf,
	mbuf->pkt_len, mbuf->data_off, mbuf->seqn);
	for (i = lo; i < hi; i += 16) {
		uint8_t *dp = RTE_PTR_ADD(mbuf->buf_addr, i);

		fprintf(stderr, "  %6d:  ", i);
		for (j = 0; j < 16; j++)
			fprintf(stderr, " %02x", dp[j]);

		fprintf(stderr, "\n");
	}
}
