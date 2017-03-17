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
#include "ark_mpu.h"
#include "ark_ddm.h"
#include "ark_ethdev.h"
#include "ark_debug.h"

#define ARK_TX_META_SIZE   32
#define ARK_TX_META_OFFSET (RTE_PKTMBUF_HEADROOM - ARK_TX_META_SIZE)
#define ARK_TX_MAX_NOCHAIN (RTE_MBUF_DEFAULT_DATAROOM)
#define ARK_TX_PAD_TO_60   1

#ifdef RTE_LIBRTE_ARK_DEBUG_TX
#define ARK_TX_DEBUG       1
#define ARK_TX_DEBUG_JUMBO 1
#else
#define ARK_TX_DEBUG       0
#define ARK_TX_DEBUG_JUMBO 0
#endif

/* ************************************************************************* */

/* struct fixed in FPGA -- 16 bytes */

/* TODO move to ark_ddm.h */
struct ark_tx_meta {
	uint64_t physaddr;
	uint32_t delta_ns;
	uint16_t data_len;		/* of this MBUF */
#define   ARK_DDM_EOP   0x01
#define   ARK_DDM_SOP   0x02
	uint8_t flags;		/* bit 0 indicates last mbuf in chain. */
	uint8_t reserved[1];
};

/* ************************************************************************* */
struct ark_tx_queue {

	struct ark_tx_meta *metaQ;
	struct rte_mbuf **bufs;

	/* handles for hw objects */
	struct ark_mpu_t *mpu;
	struct ark_ddm_t *ddm;

	/* Stats HW tracks bytes and packets, need to count send errors */
	uint64_t tx_errors;

	uint32_t queueSize;
	uint32_t queueMask;

	/* 3 indexs to the paired data rings. */
	uint32_t prodIndex;		/* where to put the next one */
	uint32_t freeIndex;		/* mbuf has been freed */

	// The queue Id is used to identify the HW Q
	uint16_t phys_qid;
	/* The queue Index within the dpdk device structures */
	uint16_t queueIndex;

	uint32_t pad[1];

	/* second cache line - fields only used in slow path */
	MARKER cacheline1 __rte_cache_min_aligned;
	uint32_t consIndex;		/* hw is done, can be freed */
} __rte_cache_aligned;

/* Forward declarations */
static uint32_t eth_ark_tx_jumbo(struct ark_tx_queue *queue,
	struct rte_mbuf *mbuf);
static int eth_ark_tx_hw_queue_config(struct ark_tx_queue *queue);
static void free_completed_tx(struct ark_tx_queue *queue);

static inline void
ark_tx_hw_queue_stop(struct ark_tx_queue *queue)
{
	ark_mpu_stop(queue->mpu);
}

/* ************************************************************************* */
static inline void
eth_ark_tx_meta_from_mbuf(struct ark_tx_meta *meta,
	const struct rte_mbuf *mbuf, uint8_t flags)
{
	meta->physaddr = rte_mbuf_data_dma_addr(mbuf);
	meta->delta_ns = 0;
	meta->data_len = rte_pktmbuf_data_len(mbuf);
	meta->flags = flags;
}

/* ************************************************************************* */
uint16_t
eth_ark_xmit_pkts_noop(void *vtxq __rte_unused,
	struct rte_mbuf **tx_pkts __rte_unused, uint16_t nb_pkts __rte_unused)
{
	return 0;
}

/* ************************************************************************* */
uint16_t
eth_ark_xmit_pkts(void *vtxq, struct rte_mbuf **tx_pkts, uint16_t nb_pkts)
{
	struct ark_tx_queue *queue;
	struct rte_mbuf *mbuf;
	struct ark_tx_meta *meta;

	uint32_t idx;
	uint32_t prodIndexLimit;
	int stat;
	uint16_t nb;

	queue = (struct ark_tx_queue *) vtxq;

	/* free any packets after the HW is done with them */
	free_completed_tx(queue);

	prodIndexLimit = queue->queueSize + queue->freeIndex;

	for (nb = 0;
		 (nb < nb_pkts) && (queue->prodIndex != prodIndexLimit);
		 ++nb) {
		mbuf = tx_pkts[nb];

		if (ARK_TX_PAD_TO_60) {
			if (unlikely(rte_pktmbuf_pkt_len(mbuf) < 60)) {
				/* this packet even if it is small can be split,
				 * be sure to add to the end
				 */
				uint16_t toAdd = 60 - rte_pktmbuf_pkt_len(mbuf);
				char *appended = rte_pktmbuf_append(mbuf, toAdd);

				if (appended == 0) {
					/* This packet is in error, we cannot send it so just
					 * count it and delete it.
					 */
					queue->tx_errors += 1;
					rte_pktmbuf_free(mbuf);
					continue;
				}
				memset(appended, 0, toAdd);
			}
		}

		if (unlikely(mbuf->nb_segs != 1)) {
			stat = eth_ark_tx_jumbo(queue, mbuf);
			if (unlikely(stat != 0))
				break;		/* Queue is full */
		} else {
			idx = queue->prodIndex & queue->queueMask;
			queue->bufs[idx] = mbuf;
			meta = &queue->metaQ[idx];
			eth_ark_tx_meta_from_mbuf(meta, mbuf,
									  ARK_DDM_SOP | ARK_DDM_EOP);
			queue->prodIndex++;
		}
	}

	if (ARK_TX_DEBUG) {
		if (nb != nb_pkts) {
			PMD_DRV_LOG(ERR,
						"ARKP TX: Failure to send: req: %u sent: %u prod: %u cons: %u free: %u\n",
						nb_pkts, nb, queue->prodIndex, queue->consIndex,
						queue->freeIndex);
			ark_mpu_dump(queue->mpu, "TX Failure MPU: ", queue->phys_qid);
		}
	}

	/* let fpga know producer index.  */
	if (likely(nb != 0))
		ark_mpu_set_producer(queue->mpu, queue->prodIndex);

	return nb;
}

/* ************************************************************************* */
static uint32_t
eth_ark_tx_jumbo(struct ark_tx_queue *queue, struct rte_mbuf *mbuf)
{
	struct rte_mbuf *next;
	struct ark_tx_meta *meta;
	uint32_t freeQueueSpace;
	uint32_t idx;
	uint8_t flags = ARK_DDM_SOP;

	freeQueueSpace = queue->queueMask - (queue->prodIndex - queue->freeIndex);
	if (unlikely(freeQueueSpace < mbuf->nb_segs)) {
	return -1;
	}

	if (ARK_TX_DEBUG_JUMBO) {
	PMD_DRV_LOG(ERR,
		"ARKP  JUMBO TX len: %u segs: %u prod: %u cons: %u free: %u freeSpace: %u\n",
		mbuf->pkt_len, mbuf->nb_segs, queue->prodIndex, queue->consIndex,
		queue->freeIndex, freeQueueSpace);
	}

	while (mbuf != NULL) {
	next = mbuf->next;

	idx = queue->prodIndex & queue->queueMask;
	queue->bufs[idx] = mbuf;
	meta = &queue->metaQ[idx];

	flags |= (next == NULL) ? ARK_DDM_EOP : 0;
	eth_ark_tx_meta_from_mbuf(meta, mbuf, flags);
	queue->prodIndex++;

	flags &= ~ARK_DDM_SOP;	/* drop SOP flags */
	mbuf = next;
	}

	return 0;
}

/* ************************************************************************* */
int
eth_ark_tx_queue_setup(struct rte_eth_dev *dev,
	uint16_t queue_idx,
	uint16_t nb_desc,
	unsigned int socket_id, const struct rte_eth_txconf *tx_conf __rte_unused)
{
	struct ark_adapter *ark = (struct ark_adapter *) dev->data->dev_private;
	struct ark_tx_queue *queue;
	int status;

	/* TODO: divide the Q's evenly with the Vports */
	int port = ark_get_port_id(dev, ark);
	int qidx = port + queue_idx;	/* FIXME for multi queue */

	if (!rte_is_power_of_2(nb_desc)) {
		PMD_DRV_LOG(ERR,
					"DPDK Arkville configuration queue size must be power of two %u (%s)\n",
					nb_desc, __func__);
		return -1;
	}

	/* TODO: We may already be setup, check here if there is to do return */
	/* /\* Free memory prior to re-allocation if needed *\/ */
	/* if (dev->data->tx_queues[queue_idx] != NULL) { */
	/* 	dev->data->tx_queues[queue_idx] = NULL; */
	/* } */

	/* Allocate queue struct */
	queue =
		rte_zmalloc_socket("ArkTXQueue", sizeof(struct ark_tx_queue), 64,
	socket_id);
	if (queue == 0) {
		PMD_DRV_LOG(ERR, "ARKP Failed to allocate tx queue memory in %s\n",
					__func__);
		return -ENOMEM;
	}

	/* we use zmalloc no need to initialize fields */
	queue->queueSize = nb_desc;
	queue->queueMask = nb_desc - 1;
	queue->phys_qid = qidx;
	queue->queueIndex = queue_idx;
	dev->data->tx_queues[queue_idx] = queue;

	queue->metaQ =
	rte_zmalloc_socket("ArkTXQueue meta",
	nb_desc * sizeof(struct ark_tx_meta), 64, socket_id);
	queue->bufs =
	rte_zmalloc_socket("ArkTXQueue bufs",
	nb_desc * sizeof(struct rte_mbuf *), 64, socket_id);

	if (queue->metaQ == 0 || queue->bufs == 0) {
		PMD_DRV_LOG(ERR, "Failed to allocate queue memory in %s\n", __func__);
		rte_free(queue->metaQ);
		rte_free(queue->bufs);
		rte_free(queue);
		return -ENOMEM;
	}

	queue->ddm = RTE_PTR_ADD(ark->ddm.v, qidx * ARK_DDM_QOFFSET);
	queue->mpu = RTE_PTR_ADD(ark->mputx.v, qidx * ARK_MPU_QOFFSET);

	status = eth_ark_tx_hw_queue_config(queue);

	if (unlikely(status != 0)) {
		rte_free(queue->metaQ);
		rte_free(queue->bufs);
		rte_free(queue);
		return -1;		/* ERROR CODE */
	}

	return 0;
}

/* ************************************************************************* */
static int
eth_ark_tx_hw_queue_config(struct ark_tx_queue *queue)
{
	phys_addr_t queueBase, ringBase, prodIndexAddr;
	uint32_t writeInterval_ns;

	/* Verify HW -- MPU */
	if (ark_mpu_verify(queue->mpu, sizeof(struct ark_tx_meta)))
		return -1;

	queueBase = rte_malloc_virt2phy(queue);
	ringBase = rte_malloc_virt2phy(queue->metaQ);
	prodIndexAddr = queueBase + offsetof(struct ark_tx_queue, consIndex);

	ark_mpu_stop(queue->mpu);
	ark_mpu_reset(queue->mpu);

	/* Stop and Reset and configure MPU */
	ark_mpu_configure(queue->mpu, ringBase, queue->queueSize, 1);

	/* Adjust the write interval based on queue size -- increase pcie traffic
	 * when low mbuf count */
	switch (queue->queueSize) {
	case 128:
	writeInterval_ns = 500;
	break;
	case 256:
	writeInterval_ns = 500;
	break;
	case 512:
	writeInterval_ns = 1000;
	break;
	default:
	writeInterval_ns = 2000;
	break;
	}

	// Completion address in UDM
	ark_ddm_setup(queue->ddm, prodIndexAddr, writeInterval_ns);

	return 0;
}

/* ************************************************************************* */
void
eth_ark_tx_queue_release(void *vtx_queue)
{
	struct ark_tx_queue *queue;

	queue = (struct ark_tx_queue *) vtx_queue;

	ark_tx_hw_queue_stop(queue);

	queue->consIndex = queue->prodIndex;
	free_completed_tx(queue);

	rte_free(queue->metaQ);
	rte_free(queue->bufs);
	rte_free(queue);

}

/* ************************************************************************* */
int
eth_ark_tx_queue_stop(struct rte_eth_dev *dev, uint16_t queue_id)
{
	struct ark_tx_queue *queue;
	int cnt = 0;

	queue = dev->data->tx_queues[queue_id];

	/* Wait for DDM to send out all packets. */
	while (queue->consIndex != queue->prodIndex) {
		usleep(100);
		if (cnt++ > 10000)
			return -1;
	}

	ark_mpu_stop(queue->mpu);
	free_completed_tx(queue);

	dev->data->tx_queue_state[queue_id] = RTE_ETH_QUEUE_STATE_STOPPED;

	return 0;
}

int
eth_ark_tx_queue_start(struct rte_eth_dev *dev, uint16_t queue_id)
{
	struct ark_tx_queue *queue;

	queue = dev->data->tx_queues[queue_id];
	if (dev->data->tx_queue_state[queue_id] == RTE_ETH_QUEUE_STATE_STARTED)
		return 0;

	ark_mpu_start(queue->mpu);
	dev->data->tx_queue_state[queue_id] = RTE_ETH_QUEUE_STATE_STARTED;

	return 0;
}

/* ************************************************************************* */
static void
free_completed_tx(struct ark_tx_queue *queue)
{
	struct rte_mbuf *mbuf;
	struct ark_tx_meta *meta;
	uint32_t topIndex;

	topIndex = queue->consIndex;	/* read once */
	while (queue->freeIndex != topIndex) {
		meta = &queue->metaQ[queue->freeIndex & queue->queueMask];
		mbuf = queue->bufs[queue->freeIndex & queue->queueMask];

		if (likely((meta->flags & ARK_DDM_SOP) != 0)) {
			/* ref count of the mbuf is checked in this call. */
			rte_pktmbuf_free(mbuf);
		}
		queue->freeIndex++;
	}
}

/* ************************************************************************* */
void
eth_tx_queue_stats_get(void *vqueue, struct rte_eth_stats *stats)
{
	struct ark_tx_queue *queue;
	struct ark_ddm_t *ddm;
	uint64_t bytes, pkts;

	queue = vqueue;
	ddm = queue->ddm;

	bytes = ark_ddm_queue_byte_count(ddm);
	pkts = ark_ddm_queue_pkt_count(ddm);

	stats->q_opackets[queue->queueIndex] = pkts;
	stats->q_obytes[queue->queueIndex] = bytes;
	stats->opackets += pkts;
	stats->obytes += bytes;
	stats->oerrors += queue->tx_errors;
}

void
eth_tx_queue_stats_reset(void *vqueue)
{
	struct ark_tx_queue *queue;
	struct ark_ddm_t *ddm;

	queue = vqueue;
	ddm = queue->ddm;

	ark_ddm_queue_reset_stats(ddm);
	queue->tx_errors = 0;
}
