/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Intel Corporation. All rights reserved.
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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <sys/queue.h>

#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_launch.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_debug.h>
#include <rte_ethdev.h>
#include <rte_cycles.h>

#include <rte_eventdev.h>
#include "test.h"

#define MAX_PORTS 16
#define MAX_QIDS 16
#define NUM_PACKETS (1<<18)

struct test {
	struct rte_mempool *mbuf_pool;
	int ev;
	int port[MAX_PORTS];
	int qid[MAX_QIDS];
	int nb_qids;
};

static inline struct rte_mbuf *
rte_gen_arp(int portid, struct rte_mempool *mp)
{
	/*
	* len = 14 + 46
	* ARP, Request who-has 10.0.0.1 tell 10.0.0.2, length 46
	*/
	static const uint8_t arp_request[] = {
		/*0x0000:*/ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xec, 0xa8,
		0x6b, 0xfd, 0x02, 0x29, 0x08, 0x06, 0x00, 0x01,
		/*0x0010:*/ 0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xec, 0xa8,
		0x6b, 0xfd, 0x02, 0x29, 0x0a, 0x00, 0x00, 0x01,
		/*0x0020:*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x00,
		0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		/*0x0030:*/ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00
	};
	struct rte_mbuf *m;
	int pkt_len = sizeof(arp_request) - 1;

	m = rte_pktmbuf_alloc(mp);
	if (!m)
		return 0;

	memcpy((void *)((uint64_t)m->buf_addr + m->data_off),
		arp_request, pkt_len);
	rte_pktmbuf_pkt_len(m) = pkt_len;
	rte_pktmbuf_data_len(m) = pkt_len;

	RTE_SET_USED(portid);
	/*
	 * Ignore MAC address for super-simple testing
	 * struct ether_addr mac_addr;
	 * rte_eth_macaddr_get(portid, &mac_addr);
	 * memcpy((void *)((uint64_t)m->buf_addr + m->data_off + 6),
	 * &mac_addr, 6);
	 */

	return m;
}

/* initialization and config */
static inline int
init(struct test *t, int nb_queues, int nb_ports)
{
	struct rte_event_dev_config config = {
			.nb_event_queues = nb_queues,
			.nb_event_ports = nb_ports,
	};
	int ret, nevdevs = rte_event_dev_count();

	void *temp = t->mbuf_pool; /* save and restore mbuf pool */

	memset(t, 0, sizeof(*t));
	t->mbuf_pool = temp;

	if (nevdevs < 1) {
		printf("%d: No Eventdev Devices Found\n", __LINE__);
		return -1;
	}

	const char *eventdev_name = "evdev_sw0";

	t->ev = rte_event_dev_get_dev_id(eventdev_name);
	if (t->ev < 0) {
		printf("%d: Eventdev %s not found - quitting.\n", __LINE__, eventdev_name);
		return -1;
	}

	ret = rte_event_dev_configure(t->ev, &config);
	if (ret < 0)
		printf("%d: Error configuring device\n", __LINE__);
	return ret;
};

static inline int
create_ports(struct test *t, int num_ports)
{
	int i;
	static const struct rte_event_port_conf conf = {
			.dequeue_queue_depth = 32,
			.enqueue_queue_depth = 64,
	};

	for (i = 0; i < num_ports; i++) {
		if (rte_event_port_setup(t->ev, i, &conf) < 0) {
			printf("Error setting up port %d\n", i);
			return -1;
		}
		t->port[i] = i;
	}

	return 0;
}

static inline int
create_atomic_qids(struct test *t, int num_qids)
{
	int i;

	/* Q creation */
	static const struct rte_event_queue_conf conf = {
			.priority = RTE_EVENT_QUEUE_PRIORITY_NORMAL,
			.nb_atomic_flows = 1024,
	};

	for (i = t->nb_qids; i < t->nb_qids + num_qids; i++) {
		if (rte_event_queue_setup(t->ev, i, &conf) < 0) {
			printf("%d: error creating qid %d\n", __LINE__, i);
			return -1;
		}
		t->qid[i] = i;
	}
	t->nb_qids += num_qids;

	return 0;
}

static inline int
create_ordered_qids(struct test *t, int num_qids)
{
	int i;

	/* Q creation */
	static const struct rte_event_queue_conf conf = {
			.priority = RTE_EVENT_QUEUE_PRIORITY_NORMAL,
			.nb_atomic_order_sequences = 1024,
	};

	for (i = t->nb_qids; i < t->nb_qids + num_qids; i++) {
		if (rte_event_queue_setup(t->ev, i, &conf) < 0) {
			printf("%d: error creating qid %d\n", __LINE__, i);
			return -1;
		}
		t->qid[i] = i;
	}
	t->nb_qids += num_qids;

	return 0;
}

static inline int
create_unordered_qids(struct test *t, int num_qids)
{
	int i;

	/* Q creation */
	static const struct rte_event_queue_conf conf = {
			.priority = RTE_EVENT_QUEUE_PRIORITY_NORMAL,
	};

	for (i = t->nb_qids; i < t->nb_qids + num_qids; i++) {
		if (rte_event_queue_setup(t->ev, i, &conf) < 0) {
			printf("%d: error creating qid %d\n", __LINE__, i);
			return -1;
		}
		t->qid[i] = i;
	}
	t->nb_qids += num_qids;

	return 0;
}

static inline int
create_directed_qids(struct test *t, int num_qids, int ports[])
{
	int i;

	/* Q creation */
	static const struct rte_event_queue_conf conf = {
			.priority = RTE_EVENT_QUEUE_PRIORITY_NORMAL,
			.event_queue_cfg = RTE_EVENT_QUEUE_CFG_SINGLE_CONSUMER,
	};

	for (i = t->nb_qids; i < t->nb_qids + num_qids; i++) {
		struct rte_event_queue_link link;

		if (rte_event_queue_setup(t->ev, i, &conf) < 0) {
			printf("%d: error creating qid %d\n", __LINE__, i);
			return -1;
		}
		t->qid[i] = i;

		link = (struct rte_event_queue_link){
			t->qid[i],
			RTE_EVENT_QUEUE_SERVICE_PRIORITY_NORMAL
		};
		if (rte_event_port_link(t->ev, ports[i - t->nb_qids], &link, 1) != 1) {
			printf("%d: error creating link for qid %d\n",
					__LINE__, i);
			return -1;
		}
	}
	t->nb_qids += num_qids;

	return 0;
}

/* destruction */
static inline int
cleanup(struct test *t)
{
	rte_event_dev_stop(t->ev);
	rte_event_dev_close(t->ev);
	return 0;
};

/* run_prio_packet_test
 * This performs a basic packet priority check on the test instance passed in.
 * It is factored out of the main priority tests as the same tests must be
 * performed to ensure prioritization of each type of QID.
 *
 * Requirements:
 *  - An initialized test structure, including mempool
 *  - t->port[0] is initialized for both Enq / Deq of packets to the QID
 *  - t->qid[0] is the QID to be tested
 *  - if LB QID, the CQ must be mapped to the QID.
 */
static int
run_prio_packet_test(struct test *t)
{
	int err;
	const uint32_t MAGIC_SEQN[] = {4711, 1234};
	const uint32_t PRIORITY[] = {3, 0};
	unsigned i;
	for(i = 0; i < RTE_DIM(MAGIC_SEQN); i++) {
		/* generate pkt and enqueue */
		struct rte_event ev;
		struct rte_mbuf *arp = rte_gen_arp(0, t->mbuf_pool);
		if (!arp) {
			printf("%d: gen of pkt failed\n", __LINE__);
			return -1;
		}
		arp->seqn = MAGIC_SEQN[i];

		ev = (struct rte_event){
			.priority = PRIORITY[i],
			.operation = RTE_EVENT_OP_NEW,
			.queue_id = t->qid[0],
			.mbuf = arp
		};
		err = rte_event_enqueue(t->ev, t->port[0], &ev, 0);
		if (err < 0) {
			printf("%d: error failed to enqueue\n", __LINE__);
			return -1;
		}
	}

	rte_event_schedule(t->ev);

	struct rte_event_dev_stats stats;
	err = rte_event_dev_stats_get(t->ev, &stats);
	if (err) {
		printf("%d: error failed to get stats\n", __LINE__);
		return -1;
	}

	if (stats.port_rx_pkts[t->port[0]] != 2) {
		printf("%d: error stats incorrect for directed port\n", __LINE__);
		rte_event_dev_dump(stdout, t->ev);
		return -1;
	}

	struct rte_event ev, ev2;
	uint32_t deq_pkts;
	deq_pkts = rte_event_dequeue(t->ev, t->port[0], &ev, 0);
	if (deq_pkts != 1) {
		printf("%d: error failed to deq\n", __LINE__);
		rte_event_dev_dump(stdout, t->ev);
		return -1;
	}
	if(ev.mbuf->seqn != MAGIC_SEQN[1]) {
		printf("%d: first packet out not highest priority\n", __LINE__);
		rte_event_dev_dump(stdout, t->ev);
		return -1;
	}
	rte_pktmbuf_free(ev.mbuf);

	deq_pkts = rte_event_dequeue(t->ev, t->port[0], &ev2, 0);
	if (deq_pkts != 1) {
		printf("%d: error failed to deq\n", __LINE__);
		rte_event_dev_dump(stdout, t->ev);
		return -1;
	}
	if(ev2.mbuf->seqn != MAGIC_SEQN[0]) {
		printf("%d: second packet out not lower priority\n", __LINE__);
		rte_event_dev_dump(stdout, t->ev);
		return -1;
	}
	rte_pktmbuf_free(ev2.mbuf);

	cleanup(t);
	return 0;
}

static int
test_single_directed_packet(struct test *t)
{
	const int rx_enq = 0;
	const int wrk_enq = 2;
	int err;

	/* Create instance with 3 directed QIDs going to 3 ports */
	if (init(t, 3, 3) < 0 ||
			create_ports(t, 3) < 0 ||
			create_directed_qids(t, 3, t->port) < 0)
		return -1;

	if (rte_event_dev_start(t->ev) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	/************** FORWARD ****************/
	struct rte_mbuf *arp = rte_gen_arp(0, t->mbuf_pool);
	struct rte_event ev = {
			.operation = RTE_EVENT_OP_NEW,
			.queue_id = wrk_enq,
			.mbuf = arp,
	};

	if (!arp) {
		printf("%d: gen of pkt failed\n", __LINE__);
		return -1;
	}

	const uint32_t MAGIC_SEQN = 4711;
	arp->seqn = MAGIC_SEQN;

	/* generate pkt and enqueue */
	err = rte_event_enqueue(t->ev, rx_enq, &ev, 0);
	if (err < 0) {
		printf("%d: error failed to enqueue\n", __LINE__);
		return -1;
	}

	/* Run schedule() as dir packets may need to be re-ordered */
	if (rte_event_schedule(t->ev) < 0) {
		printf("%d: Error with schedule call\n", __LINE__);
		return -1;
	}

	struct rte_event_dev_stats stats;
	err = rte_event_dev_stats_get(t->ev, &stats);
	if (err) {
		printf("%d: error failed to get stats\n", __LINE__);
		return -1;
	}

	if (stats.port_rx_pkts[rx_enq] != 1) {
		printf("%d: error stats incorrect for directed port\n", __LINE__);
		return -1;
	}

	uint32_t deq_pkts;
	deq_pkts = rte_event_dequeue(t->ev, wrk_enq, &ev, 1);
	if (deq_pkts != 1) {
		printf("%d: error failed to deq\n", __LINE__);
		return -1;
	}

	err = rte_event_dev_stats_get(t->ev, &stats);
	if (stats.port_rx_pkts[wrk_enq] != 0 &&
			stats.port_rx_pkts[wrk_enq] != 1) {
		printf("%d: error directed stats post-dequeue\n", __LINE__);
		return -1;
	}

	if (ev.mbuf->seqn != MAGIC_SEQN) {
		printf("%d: error magic sequence number not dequeued\n", __LINE__);
		return -1;
	}

	rte_pktmbuf_free(ev.mbuf);
	cleanup(t);
	return 0;
}

static int
test_overload_trip(struct test *t)
{
	int err;

	/* Create instance with 3 directed QIDs going to 3 ports */
	if (init(t, 1, 1) < 0 ||
			create_ports(t, 1) < 0 ||
			create_atomic_qids(t, 1) < 0)
		return -1;

	struct rte_event_queue_link link = {t->qid[0],
			RTE_EVENT_QUEUE_SERVICE_PRIORITY_NORMAL };
	int ret = rte_event_port_link(t->ev, t->port[0], &link, 1);
	if (ret != 1) {
		printf("%d: error mapping lb qid0\n", __LINE__);
		return -1;
	}

	if (rte_event_dev_start(t->ev) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	struct rte_mbuf *arp = rte_gen_arp(0, t->mbuf_pool);
	if (!arp) {
		printf("%d: gen of pkt failed\n", __LINE__);
		return -1;
	}

	/* 512 packets is threshold
	 * iters 0 - 511 is 512 packets, then overload will be flagged
	 * iter 512 (the 513th pkt) is the first refused NEW packet */
	const uint32_t THRES = (256+1);
	uint32_t i;
	for (i = 0; i < THRES; i++) {
		struct rte_event ev = {
				.operation = RTE_EVENT_OP_NEW,
				.queue_id = t->qid[0],
				.mbuf = arp,
		};
		err = rte_event_enqueue(t->ev, 0, &ev, 0);
		if(i == THRES-1) {
			if(err != -ENOSPC) {
				printf("%d: overload trip didn't cause NEW pkt enq fail\n", __LINE__);
				return -1;
			}
			else {
				//printf("iter %d -ENOSPC returned for new enq as expected.\n", i);
			}
		} else {
			if (err < 0) {
				printf("%d: error failed to enqueue\n", __LINE__);
				return -1;
			}
		}
	}

	for (i = 0; i < THRES; i++) {
		if (rte_event_schedule(t->ev) < 0) {
			printf("%d: Error with schedule call\n", __LINE__);
			return -1;
		}

		uint32_t deq_pkts;
		struct rte_event ev;
		deq_pkts = rte_event_dequeue(t->ev, 0, &ev, 1);

		/* i == THRES-1 *should* fail to deq, due to NEW pkt rejection
		 * when enqueue is attempted in overload mode */
		if (i == (THRES-1) && deq_pkts == 0)
			break;

		if (deq_pkts != 1) {
			printf("%d: warning failed to deq event i = %d\n",
					__LINE__, i);
			//return -1;
		}
	}

	rte_pktmbuf_free(arp);
	cleanup(t);
	return 0;
}

static int
test_directed_overload(struct test *t)
{
	int err;

	/* Create instance with 3 directed QIDs going to 3 ports */
	if (init(t, 1, 1) < 0 ||
			create_ports(t, 1) < 0 ||
			create_directed_qids(t, 1, t->port) < 0)
		return -1;

	if (rte_event_dev_start(t->ev) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	/* 512 packets is threshold
	 * iters 0 - 511 is 512 packets, then overload will be flagged
	 * iter 512 (the 513th pkt) is the first refused NEW packet */
	const uint32_t THRES = (256+1);
	uint32_t i;
	for (i = 0; i < THRES; i++) {
		struct rte_event ev = {
				.operation = RTE_EVENT_OP_NEW,
				.queue_id = t->qid[0],
				.event = (uintptr_t)i,
		};
		err = rte_event_enqueue(t->ev, 0, &ev, 0);
		if(i == THRES-1) {
			if(err != -ENOSPC) {
				printf("%d: overload trip didn't cause NEW pkt enq fail\n", __LINE__);
				//return -1;
			}
			else {
				//printf("iter %d -ENOSPC returned for new enq as expected.\n", i);
			}
		} else {
			if (err < 0) {
				printf("%d: error failed to enqueue\n", __LINE__);
				return -1;
			}
		}
	}

	if (rte_event_schedule(t->ev) < 0) {
		printf("%d: Error with schedule call\n", __LINE__);
		return -1;
	}

	uint32_t pkt_deq_cntr = 0;
	for (i = 0; i < THRES; i++) {
		if (rte_event_schedule(t->ev) < 0) {
			printf("%d: Error with schedule call\n", __LINE__);
			return -1;
		}

		int32_t deq_pkts;
		struct rte_event ev;
		deq_pkts = rte_event_dequeue(t->ev, 0, &ev, 1);

		/* i == THRES-1 *should* fail to deq, due to NEW pkt rejection
		 * when enqueue is attempted in overload mode */
		if (i == (THRES-1) && deq_pkts == 0)
			break;

		if (deq_pkts != 1) {
			printf("%d: warning failed to deq (iter = %d), ret %d. Dumping stats\n",
					__LINE__, i, deq_pkts);
			rte_event_dev_dump(stdout, t->ev);
			return -1;
		}
		pkt_deq_cntr += deq_pkts;
	}

	cleanup(t);
	return 0;
}


static int
test_priority_directed(struct test *t)
{
	if (init(t, 1, 1) < 0 ||
			create_ports(t, 1) < 0 ||
			create_directed_qids(t, 1, t->port) < 0) {
		printf("%d: Error initialising device\n", __LINE__);
		return -1;
	}

	if (rte_event_dev_start(t->ev) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	return run_prio_packet_test(t);
}

static int
test_priority_atomic(struct test *t)
{
	if (init(t, 1, 1) < 0 ||
			create_ports(t, 1) < 0 ||
			create_atomic_qids(t, 1) < 0) {
		printf("%d: Error initialising device\n", __LINE__);
		return -1;
	}

	/* map the QID */
	struct rte_event_queue_link link = {t->qid[0],
			RTE_EVENT_QUEUE_SERVICE_PRIORITY_NORMAL };
	if (rte_event_port_link(t->ev, t->port[0], &link, 1) != 1) {
		printf("%d: error mapping qid to port\n", __LINE__);
		return -1;
	}
	if (rte_event_dev_start(t->ev) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	return run_prio_packet_test(t);
}

static int
test_priority_ordered(struct test *t)
{
	if (init(t, 1, 1) < 0 ||
			create_ports(t, 1) < 0 ||
			create_ordered_qids(t, 1) < 0) {
		printf("%d: Error initialising device\n", __LINE__);
		return -1;
	}

	/* map the QID */
	struct rte_event_queue_link link = {t->qid[0],
			RTE_EVENT_QUEUE_SERVICE_PRIORITY_NORMAL };
	if (rte_event_port_link(t->ev, t->port[0], &link, 1) != 1) {
		printf("%d: error mapping qid to port\n", __LINE__);
		return -1;
	}
	if (rte_event_dev_start(t->ev) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	return run_prio_packet_test(t);
}

static int
test_priority_unordered(struct test *t)
{
	if (init(t, 1, 1) < 0 ||
			create_ports(t, 1) < 0 ||
			create_unordered_qids(t, 1) < 0) {
		printf("%d: Error initialising device\n", __LINE__);
		return -1;
	}

	/* map the QID */
	struct rte_event_queue_link link = {t->qid[0],
			RTE_EVENT_QUEUE_SERVICE_PRIORITY_NORMAL };
	if (rte_event_port_link(t->ev, t->port[0], &link, 1) != 1) {
		printf("%d: error mapping qid to port\n", __LINE__);
		return -1;
	}
	if (rte_event_dev_start(t->ev) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	return run_prio_packet_test(t);
}

static int
burst_packets(struct test *t)
{
	/************** CONFIG ****************/
	uint32_t i;
	int err;
	int ret;

	/* Create instance with 4 ports and 2 queues */
	if (init(t, 2, 2) < 0 ||
			create_ports(t, 2) < 0 ||
			create_atomic_qids(t, 2) < 0) {
		printf("%d: Error initialising device\n", __LINE__);
		return -1;
	}

	/* CQ mapping to QID */
	struct rte_event_queue_link link = {t->qid[0],
			RTE_EVENT_QUEUE_SERVICE_PRIORITY_NORMAL };
	ret = rte_event_port_link(t->ev, t->port[0], &link, 1);
	if (ret != 1) {
		printf("%d: error mapping lb qid0\n", __LINE__);
		return -1;
	}
	link.queue_id = t->qid[1];
	ret = rte_event_port_link(t->ev, t->port[1], &link, 1);
	if (ret != 1) {
		printf("%d: error mapping lb qid1\n", __LINE__);
		return -1;
	}

	if (rte_event_dev_start(t->ev) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	/************** FORWARD ****************/
	const uint32_t rx_port = 0;
	const uint32_t NUM_PKTS = 2;

	for (i = 0; i < NUM_PKTS; i++) {
		struct rte_mbuf *arp = rte_gen_arp(0, t->mbuf_pool);
		if (!arp) {
			printf("%d: error generating pkt\n" , __LINE__);
			return -1;
		}

		struct rte_event ev = {
				.operation = RTE_EVENT_OP_NEW,
				.queue_id = i % 2,
				.flow_id = i % 3,
				.mbuf = arp,
		};
		/* generate pkt and enqueue */
		err = rte_event_enqueue(t->ev, t->port[rx_port], &ev, 0);
		if (err < 1) {
			printf("%d: Failed to enqueue\n", __LINE__);
			return -1;
		}
	}
	int16_t pkts = rte_event_schedule(t->ev);

	RTE_SET_USED(pkts);

	/* Check stats for all NUM_PKTS arrived to sched core */
	struct rte_event_dev_stats stats;

	err = rte_event_dev_stats_get(t->ev, &stats);
	if (err) {
		printf("%d: failed to get stats\n", __LINE__);
		return -1;
	}
	if (stats.rx_pkts != NUM_PKTS || stats.tx_pkts != NUM_PKTS) {
		printf("%d: Sched core didn't receive all %d pkts\n", __LINE__, NUM_PKTS);
		rte_event_dev_dump(stdout, t->ev);
		return -1;
	}

	uint32_t deq_pkts;
	int p;

	deq_pkts = 0;
	/******** DEQ QID 1 *******/
	do {
		struct rte_event ev;
		p = rte_event_dequeue(t->ev, t->port[0], &ev, 0);
		deq_pkts += p;
		rte_pktmbuf_free(ev.mbuf);
	} while (p);

	if (deq_pkts != NUM_PKTS/2) {
		printf("%d: Half of NUM_PKTS didn't arrive at port 1\n", __LINE__);
		return -1;
	}

	/******** DEQ QID 2 *******/
	deq_pkts = 0;
	do {
		struct rte_event ev;
		p = rte_event_dequeue(t->ev, t->port[1], &ev, 0);
		deq_pkts += p;
		rte_pktmbuf_free(ev.mbuf);
	} while (p);
	if (deq_pkts != NUM_PKTS/2) {
		printf("%d: Half of NUM_PKTS didn't arrive at port 2\n", __LINE__);
		return -1;
	}

	cleanup(t);
	return 0;
}

static int
load_balancing(struct test *t)
{
	const int rx_enq = 0;
	int err;
	uint32_t i;

	if (init(t, 1, 4) < 0 ||
			create_ports(t, 4) < 0 ||
			create_atomic_qids(t, 1) < 0) {
		printf("%d: Error initialising device\n", __LINE__);
		return -1;
	}

	struct rte_event_queue_link link = {t->qid[0],
			RTE_EVENT_QUEUE_SERVICE_PRIORITY_NORMAL };
	for (i = 0; i < 3; i++) {
		/* map port 1 - 3 inclusive */
		if (rte_event_port_link(t->ev, t->port[i+1], &link, 1) != 1) {
			printf("%d: error mapping qid to port %d\n", __LINE__, i);
			return -1;
		}
	}

	if (rte_event_dev_start(t->ev) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	/************** FORWARD ****************/
	/*
	 * Create a set of flows that test the load-balancing operation of the
	 * implementation. Fill CQ 0 and 1 with flows 0 and 1, and test
	 * with a new flow, which should be sent to the 3rd mapped CQ
	 */
	static uint32_t flows[] = {0, 1, 1, 0, 0, 2, 2, 0, 2};
#define PKT_NUM (sizeof(flows) / sizeof(flows[0]))
	for (i = 0; i < PKT_NUM; i++) {
		struct rte_mbuf *arp = rte_gen_arp(0, t->mbuf_pool);
		if (!arp) {
			printf("%d: gen of pkt failed\n", __LINE__);
			return -1;
		}

		struct rte_event ev = {
				.operation = RTE_EVENT_OP_NEW,
				.queue_id = t->qid[0],
				.flow_id = flows[i],
				.mbuf = arp,
		};
		/* generate pkt and enqueue */
		err = rte_event_enqueue(t->ev, t->port[rx_enq], &ev, 0);
		if (err < 1) {
			printf("%d: Failed to enqueue\n", __LINE__);
			return -1;
		}
	}

	rte_event_schedule(t->ev);

	struct rte_event_dev_stats stats;
	err = rte_event_dev_stats_get(t->ev, &stats);
	if (err) {
		printf("%d: failed to get stats\n", __LINE__);
		return -1;
	}

	if (stats.port_inflight[1] != 4) {
		printf("%d:%s: port 1 inflight not correct\n", __LINE__, __func__);
		return -1;
	}
	if (stats.port_inflight[2] != 2) {
		printf("%d:%s: port 2 inflight not correct\n", __LINE__, __func__);
		return -1;
	}
	if (stats.port_inflight[3] != 3) {
		printf("%d:%s: port 3 inflight not correct\n", __LINE__, __func__);
		return -1;
	}

	cleanup(t);
	return 0;
}

static int
invalid_qid(struct test *t)
{
	struct rte_event_dev_stats stats;
	const int rx_enq = 0;
	int err;
	uint32_t i;

	if (init(t, 1, 4) < 0 ||
			create_ports(t, 4) < 0 ||
			create_atomic_qids(t, 1) < 0) {
		printf("%d: Error initialising device\n", __LINE__);
		return -1;
	}

	/* CQ mapping to QID */
	for(i = 0; i < 4; i++) {
		struct rte_event_queue_link link = {t->qid[0],
				RTE_EVENT_QUEUE_SERVICE_PRIORITY_NORMAL };
		err = rte_event_port_link(t->ev, t->port[i], &link, 1);
		if (err != 1) {
			printf("%d: error mapping port 1 qid\n", __LINE__);
			return -1;
		}
	}

	if (rte_event_dev_start(t->ev) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	/*
	 * Send in a packet with an invalid qid to the scheduler.
	 * We should see the packed enqueued OK, but the inflights for
	 * that packet should not be incremented, and the rx_dropped
	 * should be incremented.
	 */
	static uint32_t flows1[] = {20};

#define PKT_NUM1 (sizeof(flows1) / sizeof(flows1[0]))

	for (i = 0; i < PKT_NUM1; i++) {
		struct rte_mbuf *arp = rte_gen_arp(0, t->mbuf_pool);
		if (!arp) {
			printf("%d: gen of pkt failed\n", __LINE__);
			return -1;
		}

		struct rte_event ev = {
				.operation = RTE_EVENT_OP_NEW,
				.queue_id = t->qid[0] + flows1[i],
				.flow_id = i,
				.mbuf = arp,
		};
		/* generate pkt and enqueue */
		err = rte_event_enqueue(t->ev, t->port[rx_enq], &ev, 0);
		if (err < 1) {
			printf("%d: Failed to enqueue\n", __LINE__);
			return -1;
		}
	}

	/* call the scheduler */
	int16_t pkts = rte_event_schedule(t->ev);
	RTE_SET_USED(pkts);

	err = rte_event_dev_stats_get(t->ev, &stats);
	if (err) {
		printf("%d: failed to get stats\n", __LINE__);
		return -1;
	}

	/*
	 * Now check the resulting inflights on the port, and the rx_dropped.
	 */
	if (stats.port_inflight[0] != 0) {
		printf("%d:%s: port 1 inflight count not correct\n", __LINE__, __func__);
		rte_event_dev_dump(stdout, 0);
		return -1;
	}
	if (stats.port_rx_dropped[0] != 1) {
		printf("%d:%s: port 1 drops\n", __LINE__, __func__);
		rte_event_dev_dump(stdout, 0);
		return -1;
	}
	/* each packet drop should only be counted in one place - port or dev */
	if (stats.rx_dropped != 0) {
		printf("%d:%s: port 1 dropped count not correct\n", __LINE__, __func__);
		rte_event_dev_dump(stdout, 0);
		return -1;
	}

	cleanup(t);
	return 0;
}

static int
worker_loopback_worker_fn(void *arg)
{
	struct test *t = arg;
	uint8_t port = t->port[1];
	int count = 0;
	int err;

	/*
	 * Takes packets from the input port and then loops them back through
	 * the Queue Manager. Each packet gets looped through QIDs 0-8, 16 times,
	 * so each packet goes through 8*16 = 128 times.
	 */
	printf("%d: \tWorker function started\n", __LINE__);
	while (count < NUM_PACKETS) {
#define BURST_SIZE 32
		struct rte_event ev[BURST_SIZE];
		uint16_t i, nb_rx = rte_event_dequeue(t->ev, port, ev, BURST_SIZE);
		if (nb_rx == 0) {
			rte_pause();
			continue;
		}

		for (i = 0; i < nb_rx; i++) {
			ev[i].queue_id++;
			if (ev[i].queue_id != 8) {
				ev[i].operation = RTE_EVENT_OP_FORWARD;
				err = rte_event_enqueue(t->ev, port, &ev[i], 0);
				if (err <= 0) {
					printf("%d: Can't enqueue FWD!!\n", __LINE__);
					return -1;
				}
				continue;
			}

			ev[i].queue_id = 0;
			ev[i].mbuf->udata64++;
			if (ev[i].mbuf->udata64 != 16) {
				ev[i].operation = RTE_EVENT_OP_FORWARD;
				err = rte_event_enqueue(t->ev, port, &ev[i], 0);
				if (err <= 0) {
					printf("%d: Can't enqueue FWD!!\n", __LINE__);
					return -1;
				}
				continue;
			}
			/* we have hit 16 iterations through system - drop */
			rte_pktmbuf_free(ev[i].mbuf);
			count++;
			ev[i].operation = RTE_EVENT_OP_DROP;
			err = rte_event_enqueue(t->ev, port, &ev[i], 0);
			if(err != 1) {
				printf("%d drop enqueue failed\n", __LINE__);
				return -1;
			}
		}
	}

	return 0;
}

static int
worker_loopback_producer_fn(void *arg)
{
	struct test *t = arg;
	uint8_t port = t->port[0];
	uint64_t count = 0;

	printf("%d: \tProducer function started\n", __LINE__);
	while (count < NUM_PACKETS) {
		struct rte_mbuf *m = rte_pktmbuf_alloc(t->mbuf_pool);
		if (m == NULL) {
			printf("%d: Error allocating mbuf\n", __LINE__);
			return -1;
		}
		m->udata64 = 0;

		struct rte_event ev = {
				.operation = RTE_EVENT_OP_NEW,
				.queue_id = t->qid[0],
				.flow_id = (uintptr_t)m & 0xFFFF,
				.mbuf = m,
		};

		while (rte_event_enqueue(t->ev, port, &ev, 0) != 1)
			rte_pause();

		count++;
	}

	return 0;
}

static int
worker_loopback(struct test *t)
{
	/* use a single producer core, and a worker core to see what happens
	 * if the worker loops packets back multiple times
	 */
	struct rte_event_dev_stats stats;
	uint64_t print_cycles = 0, cycles = 0;
	uint64_t tx_pkts = 0;
	int err;
	int w_lcore, p_lcore;
	uint32_t i;

	if (init(t, 8, 2) < 0 ||
			create_ports(t, 2) < 0 ||
			create_atomic_qids(t, 8) < 0) {
		printf("%d: Error initialising device\n", __LINE__);
		return -1;
	}

	/* CQ mapping to QID */
	for(i = 0; i < 8; i++) {
		struct rte_event_queue_link link = {t->qid[i],
				RTE_EVENT_QUEUE_SERVICE_PRIORITY_NORMAL };
		err = rte_event_port_link(t->ev, t->port[1], &link, 1);
		if (err != 1) {
			printf("%d: error mapping port 2 qid %d\n", __LINE__, i);
			return -1;
		}
	}

	if (rte_event_dev_start(t->ev) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	p_lcore = rte_get_next_lcore(
			/* start core */ -1,
			/* skip master */ 1,
			/* wrap */ 0);
	w_lcore = rte_get_next_lcore(p_lcore, 1, 0);

	rte_eal_remote_launch(worker_loopback_producer_fn, t, p_lcore);
	rte_eal_remote_launch(worker_loopback_worker_fn, t, w_lcore);

	print_cycles = cycles = rte_get_timer_cycles();
	while (rte_eal_get_lcore_state(p_lcore) != FINISHED ||
			rte_eal_get_lcore_state(w_lcore) != FINISHED) {

		rte_event_schedule(t->ev);

		uint64_t new_cycles = rte_get_timer_cycles();

		if (new_cycles - print_cycles > rte_get_timer_hz()) {
			rte_event_dev_stats_get(t->ev, &stats);
			printf("%d: \tSched Rx = %" PRIu64 ", Tx = %" PRIu64 "\n",
					__LINE__, stats.rx_pkts, stats.tx_pkts);

			print_cycles = new_cycles;
		}
		if (new_cycles - cycles > rte_get_timer_hz() * 3) {
			rte_event_dev_stats_get(t->ev, &stats);
			if (stats.tx_pkts == tx_pkts) {
				rte_event_dev_dump(stdout, t->ev);
				printf("%d: \nNo schedules for seconds, deadlock\n", __LINE__);
				return -1;
			}
			tx_pkts = stats.tx_pkts;
			cycles = new_cycles;
		}
	}

	rte_eal_mp_wait_lcore();

	//rte_event_dev_dump(stdout, 0);

	cleanup(t);
	return 0;
}

static struct rte_mempool *eventdev_func_mempool;

static int
test_eventdev(void)
{
	struct test *t = malloc(sizeof(struct test));
	int ret;

	/* Only create mbuf pool once, reuse for each test run */
	if (!eventdev_func_mempool) {
		eventdev_func_mempool = rte_pktmbuf_pool_create("EVDEV_SA_MBUF_POOL",
				(1<<16), /* size */
				32 /*MBUF_CACHE_SIZE*/,
				0,
				RTE_MBUF_DEFAULT_BUF_SIZE,
				rte_socket_id());
		if (!eventdev_func_mempool) {
			printf("ERROR creating mempool\n");
			return -1;
		}
	}
	t->mbuf_pool = eventdev_func_mempool;

	printf("*** Running Single Directed Packet test...\n");
	ret = test_single_directed_packet(t);
	if (ret != 0) {
		printf("ERROR - Single Directed Packet test FAILED.\n");
		return ret;
	}
	printf("*** Running Overload Trip test...\n");
	ret = test_overload_trip(t);
	if (ret != 0) {
		printf("ERROR - Overload Trip test FAILED.\n");
		return ret;
	}
	printf("*** Running Directed Overload test...\n");
	ret = test_directed_overload(t);
	if (ret != 0) {
		printf("ERROR - Directed Overload test FAILED.\n");
		return ret;
	}
	printf("*** Running Prioritized Directed test...\n");
	ret = test_priority_directed(t);
	if (ret != 0) {
		printf("ERROR - Prioritized Directed test FAILED.\n");
		return ret;
	}
	printf("*** Running Prioritized Atomic test...\n");
	ret = test_priority_atomic(t);
	if (ret != 0) {
		printf("ERROR - Prioritized Atomic test FAILED.\n");
		return ret;
	}

	printf("*** Running Prioritized Ordered test...\n");
	ret = test_priority_ordered(t);
	if (ret != 0) {
		printf("ERROR - Prioritized Ordered test FAILED.\n");
		return ret;
	}
	printf("*** Running Prioritized Unordered test...\n");
	ret = test_priority_unordered(t);
	if (ret != 0) {
		printf("ERROR - Prioritized Unordered test FAILED.\n");
		return ret;
	}
	printf("*** Running Burst Packets test...\n");
	ret = burst_packets(t);
	if (ret != 0) {
		printf("ERROR - Burst Packets test FAILED.\n");
		return ret;
	}
	printf("*** Running Load Balancing test...\n");
	ret = load_balancing(t);
	if (ret != 0) {
		printf("ERROR - Load Balancing test FAILED.\n");
		return ret;
	}
	printf("*** Running Invalid QID test...\n");
	ret = invalid_qid(t);
	if (ret != 0) {
		printf("ERROR - Invalid QID test FAILED.\n");
		return ret;
	}
	if (rte_lcore_count() >= 3) {
		printf("*** Running Worker loopback test...\n");
		ret = worker_loopback(t);
		if (ret != 0) {
			printf("ERROR - Worker loopback test FAILED.\n");
			return ret;
		}
	} else {
		printf("### Not enough cores for worker loopback test. \n");
		printf("### Need at least 3 cores for test.\n");
	}
	/* Free test instance, leaving mempool initialized, and a pointer to it
	 * in the static eventdev_func_mempool variable. It is re-used on re-runs */
	free(t);

	return 0;
}

REGISTER_TEST_COMMAND(eventdev_func_autotest, test_eventdev);
