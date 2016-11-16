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

#include "sw_evdev.h"

#include <rte_atomic.h>
#include <rte_hash_crc.h>

#define FLOWID_MASK (SW_QID_NUM_FIDS-1)

static inline void
sw_overload_check_and_set(struct sw_evdev *sw, struct sw_port *p,
			  uint16_t free_count)
{
	if (!p->overloaded &&
			free_count < MAX_SW_PROD_Q_DEPTH - p->overload_threshold) {
		p->overloaded = 1;
		rte_atomic32_inc((void *)&sw->overloaded);
	}
}

int
sw_event_enqueue(struct rte_event_dev *dev, uint8_t port_id, struct rte_event *ev,
		  bool pin_event)
{
	RTE_SET_USED(pin_event);
	uint16_t free_count;
	struct sw_evdev *sw = (void *)dev;

	if(port_id >= sw->port_count)
		return -1;

	struct sw_port *p = &sw->ports[port_id];
	/* TODO: Concider optimization: keep port overloaded in flat array in
	 * sw instance, do a lookup and just one return branch together with
	 * port_id check above */
	if(sw->overloaded && ev->operation == RTE_EVENT_OP_NEW)
		return -ENOSPC;

	ev->operation = sw_qe_flag_map[ev->operation];
	const uint8_t invalid_qid = (ev[0].queue_id >= sw->qid_count);
	ev[0].operation &= ~(invalid_qid << QE_FLAG_VALID_SHIFT);
	/* mask flowID to valid range after a crc to jumble bits */
	ev[0].flow_id = FLOWID_MASK & rte_hash_crc_4byte(ev[0].flow_id, -1);

	if(invalid_qid) {
		p->stats.rx_dropped++;
	}

	unsigned int num_enq = qe_ring_enqueue_burst(p->rx_worker_ring,
						     ev, 1, &free_count);

	sw_overload_check_and_set(sw, p, free_count);

	/* TODO: Discuss on ML and fix this inconsistency in API:
	 * num_enq is the number of packet enqueued, so
	 * 0 = no packets
	 * 1 = got a packet
	 * This is different to how currently documented in API.
	 */
	return num_enq;
}

int
sw_event_enqueue_burst(struct rte_event_dev *dev, uint8_t port_id,
			struct rte_event ev[], int num, bool pin_event)
{
	/* TODO: change enqueue API to uint32_t for num? */
	int32_t i;
	uint16_t free_count;
	struct sw_evdev *sw = (void *)dev;

	if(port_id >= sw->port_count)
		return 0;

	struct sw_port *p = &sw->ports[port_id];
	RTE_SET_USED(pin_event);

	for (i = 0; i < num; i++) {
		/* optimize to two loops, with and without overload */
		if(sw->overloaded && ev[i].operation == RTE_EVENT_OP_NEW)
			return -ENOSPC;

		ev[i].operation = sw_qe_flag_map[ev[i].operation];
		const uint8_t invalid_qid = (ev[i].queue_id >= sw->qid_count);
		ev[i].operation &= ~(invalid_qid << QE_FLAG_VALID_SHIFT);
		ev[i].flow_id = FLOWID_MASK & rte_hash_crc_4byte(ev[i].flow_id, -1);

		if(invalid_qid) {
			p->stats.rx_dropped++;
		}
	}

	/* returns number of events actually enqueued */
	uint32_t deq = qe_ring_enqueue_burst(p->rx_worker_ring, ev, num,
					     &free_count);
	sw_overload_check_and_set(sw, p, free_count);
	return deq;
}

bool
sw_event_dequeue(struct rte_event_dev *dev, uint8_t port_id,
		  struct rte_event *ev, uint64_t wait)
{
	RTE_SET_USED(wait);
	struct sw_evdev *sw = (void *)dev;

	if(port_id >= sw->port_count)
		return 0;

	struct sw_port *p = &sw->ports[port_id];
	struct qe_ring *ring = p->cq_worker_ring;

	/* check that all previous dequeus have been released */
	uint16_t out_rels = p->outstanding_releases;
	uint16_t i;
	for(i = 0; i < out_rels; i++) {
		sw_event_release(dev, port_id, i);
	}

	/* Intel modification: may not be in final API */
	if(ev == 0)
		return 0;

	/* returns number of events actually dequeued, after storing */
	uint32_t ndeq = qe_ring_dequeue_burst(ring, ev, 1);
	p->outstanding_releases = ndeq;
	return ndeq;
}

int
sw_event_dequeue_burst(struct rte_event_dev *dev, uint8_t port_id,
			struct rte_event *ev, int num, uint64_t wait)
{
	RTE_SET_USED(wait);
	struct sw_evdev *sw = (void *)dev;

	if(port_id >= sw->port_count)
		return 0;

	struct sw_port *p = &sw->ports[port_id];
	struct qe_ring *ring = p->cq_worker_ring;

	/* check that all previous dequeus have been released */
	if (!p->is_directed) {
		uint16_t out_rels = p->outstanding_releases;
		uint16_t i;
		for(i = 0; i < out_rels; i++) {
			sw_event_release(dev, port_id, i);
		}
	}

	/* Intel modification: may not be in final API */
	if(ev == 0)
		return 0;

	/* returns number of events actually dequeued */
	uint32_t ndeq = qe_ring_dequeue_burst(ring, ev, num);
	p->outstanding_releases = ndeq;
	return ndeq;
}

void
sw_event_release(struct rte_event_dev *dev, uint8_t port_id, uint8_t index)
{
	struct sw_evdev *sw = (void *)dev;
	struct sw_port *p = &sw->ports[port_id];
	RTE_SET_USED(p);
	RTE_SET_USED(index);

	/* This function "hints" the scheduler that packet *index* of the
	 * previous burst:
	 * (Atomic)  has completed is critical section
	 * (Ordered) is ready for egress
	 *
	 * It is not mandatory to implement this functionality, but it may
	 * improve load-balancing / parallelism in the packet flows.
	 */

	/* create drop message */
	struct rte_event ev = {
		.operation = sw_qe_flag_map[RTE_EVENT_OP_DROP],
	};

	uint16_t free_count;
	qe_ring_enqueue_burst(p->rx_worker_ring, &ev, 1, &free_count);

	p->outstanding_releases--;
}
