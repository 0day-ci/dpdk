/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Intel Corporation. All rights reserved.
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

#ifndef _SW_EVDEV_H_
#define _SW_EVDEV_H_

#include <rte_eventdev.h>
#include <rte_eventdev_pmd.h>
#include "event_ring.h"

#define PMD_NAME "evdev_sw"

#define SW_QIDS_MAX 128
#define SW_QID_NUM_FIDS 16384
#define SW_IQS_MAX 4
#define SW_Q_PRIORITY_MAX 255
#define SW_PORTS_MAX 128
#define MAX_SW_CONS_Q_DEPTH 255

/* allow for lots of over-provisioning */
#define MAX_SW_PROD_Q_DEPTH 4096

#define SW_FRAGMENTS_MAX 16
#define PORT_DEQUEUE_BURST_SIZE 16
#define SW_PORT_HIST_LIST (MAX_SW_PROD_Q_DEPTH + (MAX_SW_CONS_Q_DEPTH*2))

#define SW_PORT_OVERLOAD_THRES (512)

#define RTE_SCHED_TYPE_DIRECT (RTE_SCHED_TYPE_PARALLEL + 1)

#ifdef RTE_LIBRTE_PMD_EVDEV_SW_DEBUG
#define SW_LOG_INFO(fmt, args...) \
	RTE_LOG(INFO, PMD, "[%s] %s() line %u: " fmt "\n", \
			PMD_NAME, \
			__func__, __LINE__, ## args)

#define SW_LOG_DBG(fmt, args...) \
	RTE_LOG(DEBUG, PMD, "[%s] %s() line %u: " fmt "\n", \
			PMD_NAME, \
			__func__, __LINE__, ## args)
#else
#define SW_LOG_INFO(fmt, args...)
#define SW_LOG_DBG(fmt, args...)
#endif

enum {
	QE_FLAG_VALID_SHIFT = 0,
	QE_FLAG_COMPLETE_SHIFT,
	QE_FLAG_NOT_EOP_SHIFT,
	_QE_FLAG_COUNT
};

#define QE_FLAG_VALID    (1 << QE_FLAG_VALID_SHIFT)  /* set for NEW, FWD, FRAG */
#define QE_FLAG_COMPLETE (1 << QE_FLAG_COMPLETE_SHIFT)  /* set for FWD, DROP */
#define QE_FLAG_NOT_EOP  (1 << QE_FLAG_NOT_EOP_SHIFT)  /* set for FRAG only */

static const uint8_t sw_qe_flag_map[] = {
		QE_FLAG_VALID /* RTE_QEENT_OP_NEW */,
		QE_FLAG_VALID | QE_FLAG_COMPLETE /* RTE_QEENT_OP_FWD */,
		QE_FLAG_COMPLETE /* RTE_QEENT_OP_DROP */,
		QE_FLAG_VALID | QE_FLAG_COMPLETE | QE_FLAG_NOT_EOP,
};

/* Records basic event stats at a given point. Used in port and qid structs */
struct sw_point_stats {
	uint64_t rx_pkts;
	uint64_t rx_dropped;
	uint64_t tx_pkts;
};

struct reorder_buffer_entry {
	uint16_t num_fragments;		/**< Number of packet fragments */
	uint16_t fragment_index;	/**< Points to the oldest valid frag */
	uint8_t ready;			/**< Entry is ready to be reordered */
	struct rte_event fragments[SW_FRAGMENTS_MAX];
};

struct sw_hist_list_entry {
	int32_t qid;
	int32_t fid;
	struct reorder_buffer_entry *rob_entry;
};

struct sw_port {
	/* A numeric ID for the port. This should be used to access the
	 * statistics as returned by *rte_event_dev_stats_get*, and in other
	 * places where the API requires accessing a port by integer. It is not
	 * valid to assume that ports will be allocated in a linear sequence.
	 */
	uint8_t id;

	/** Indicates if this port is overloaded, and we need to throttle input */
	uint8_t overloaded;
	uint8_t overload_threshold;

	int16_t is_directed; /** Takes from a single directed QID */
	int16_t num_ordered_qids; /** For loadbalanced we can optimise pulling
			    packets from producers if there is no reordering
			    involved */

	/* track packets in and out of this port */
	struct sw_point_stats stats;

	/** Ring and buffer for pulling events from workers for scheduling */
	struct qe_ring *rx_worker_ring __rte_cache_aligned;
	uint32_t pp_buf_start;
	uint32_t pp_buf_count;
	struct rte_event pp_buf[PORT_DEQUEUE_BURST_SIZE];


	/** Ring and buffer for pushing packets to workers after scheduling */
	struct qe_ring *cq_worker_ring __rte_cache_aligned;
	uint16_t cq_buf_count;
	uint16_t outstanding_releases; /* num releases yet to be completed */
	struct rte_event cq_buf[MAX_SW_CONS_Q_DEPTH];

	/* History list structs, containing info on pkts egressed to worker */
	uint16_t hist_head __rte_cache_aligned;
	uint16_t hist_tail;
	uint16_t inflights;
	struct sw_hist_list_entry hist_list[SW_PORT_HIST_LIST];

	uint8_t num_qids_mapped;
};

struct sw_fid_t {
	/* which CQ this FID is currently pinned to */
	uint32_t cq;
	/* number of packets gone to the CQ with this FID */
	uint32_t count;
};

struct sw_qid {
	/* The type of this QID */
	int type;
	/* Integer ID representing the queue. This is used in history lists,
	 * to identify the stage of processing. */
	uint32_t id;
	struct sw_point_stats stats;

	/* Internal priority rings for packets */
	struct iq_ring *iq[SW_IQS_MAX];
	uint32_t iq_pkt_mask; 	/* A mask to indicate packets in an IQ */
	uint64_t iq_pkt_count[SW_IQS_MAX];

	/* Information on what CQs are polling this IQ */
	uint32_t cq_num_mapped_cqs;
	uint32_t cq_next_tx; /* cq to write next (non-atomic) packet */
	uint32_t cq_map[SW_PORTS_MAX];

	/* Track flow ids for atomic load balancing */
	struct sw_fid_t fids[SW_QID_NUM_FIDS];

	/* Track packet order for reordering when needed */
	struct reorder_buffer_entry *reorder_buffer; /* packets awaiting reordering */
	struct rte_ring *reorder_buffer_freelist; /* available reorder slots */
	uint32_t reorder_buffer_index; /* oldest valid reorder buffer entry */
	uint32_t window_size;          /* Used to wrap reorder_buffer_index */

	uint8_t priority;
};

struct sw_evdev {
	/* must be the first item in the private dev struct */
	struct rte_event_dev dev;

	const struct rte_memzone *mz;

	/* Contains all ports - load balanced and directed */
	struct sw_port ports[SW_PORTS_MAX];
	uint32_t port_count;
	uint16_t cq_ring_space[SW_PORTS_MAX]; /* How many packets are in the cq */

	/* All qids - allocated in one slab for vectorization */
	struct sw_qid qids[SW_QIDS_MAX];
	uint32_t qid_count;

	/* Array of pointers to load-balanced QIDs sorted by priority level */
	struct sw_qid *qids_prioritized[SW_QIDS_MAX];

	/* Stats */
	struct sw_point_stats stats __rte_cache_aligned;
	uint64_t sched_called;
	uint64_t sched_no_iq_enqueues;
	uint64_t sched_no_cq_enqueues;
	uint64_t sched_cq_qid_called;
	uint64_t sched_overload_counter;

	uint8_t started;

	uint32_t overloaded __rte_cache_aligned;
};

int  sw_event_enqueue(struct rte_event_dev *dev, uint8_t port_id,
		     struct rte_event *ev, bool pin_event);
int  sw_event_enqueue_burst(struct rte_event_dev *dev, uint8_t port_id,
			   struct rte_event ev[], int num, bool pin_event);
bool sw_event_dequeue(struct rte_event_dev *dev, uint8_t port_id,
		      struct rte_event *ev, uint64_t wait);
int  sw_event_dequeue_burst(struct rte_event_dev *dev, uint8_t port_id,
			   struct rte_event *ev, int num, uint64_t wait);
void sw_event_release(struct rte_event_dev *dev, uint8_t port_id, uint8_t index);
int  sw_event_schedule(struct rte_event_dev *dev);

#endif /* _SW_EVDEV_H_ */
