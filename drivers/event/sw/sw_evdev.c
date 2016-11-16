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

#include <string.h>

#include <rte_vdev.h>
#include <rte_memzone.h>
#include <rte_kvargs.h>
#include <rte_ring.h>
#include <rte_eventdev_pmd.h>

#include "sw_evdev.h"
#include "iq_ring.h"

#define NUMA_NODE_ARG "numa_node"

static int
sw_dev_stats_get(const struct rte_event_dev *dev,
		struct rte_event_dev_stats *stats)
{
	const struct sw_evdev *sw = (const void *)dev;
	unsigned int i;

	if (dev == NULL || stats == NULL)
		return -EINVAL;

	memset(stats, 0, sizeof(*stats));

	stats->rx_pkts = sw->stats.rx_pkts;
	stats->rx_dropped = sw->stats.rx_dropped;
	stats->tx_pkts = sw->stats.tx_pkts;

	for (i = 0; i < sw->port_count; i++) {
		stats->port_rx_pkts[i] = sw->ports[i].stats.rx_pkts;
		stats->port_rx_dropped[i] = sw->ports[i].stats.rx_dropped;
		stats->port_inflight[i] = sw->ports[i].inflights;
		stats->port_tx_pkts[i] = sw->ports[i].stats.tx_pkts;
	}

	for (i = 0; i < sw->qid_count; i++) {
		stats->queue_rx_pkts[i] = sw->qids[i].stats.rx_pkts;
		stats->queue_rx_dropped[i] = sw->qids[i].stats.rx_dropped;
		stats->queue_tx_pkts[i] = sw->qids[i].stats.tx_pkts;
	}
	return 0;
}

static int
sw_port_link(struct rte_event_dev *dev, uint8_t port_id,
		struct rte_event_queue_link link[], int num)
{
	struct sw_evdev *sw = (void *)dev;
	struct sw_port *p = &sw->ports[port_id];
	int i;

	if (link == NULL) {
		/* TODO: map all queues */
		rte_errno = -EDQUOT;
		return 0;
	}
	if (port_id > sw->port_count) {
		rte_errno = -EINVAL;
		return 0;
	}

	for (i = 0; i < num; i++) {
		struct sw_qid *q;
		uint32_t qid = link[i].queue_id;
		if (qid >= sw->qid_count) {
			break; /* error - invalid QIDs */
		}
		q = &sw->qids[qid];

		/* check for qid map overflow */
		if (q->cq_num_mapped_cqs >= RTE_DIM(q->cq_map))
			break;

		if (p->is_directed && p->num_qids_mapped > 0)
			break;

		if (q->type == RTE_SCHED_TYPE_DIRECT) {
			/* check directed qids only map to one port */
			if (p->num_qids_mapped > 0)
				break;
			/* check port only takes a directed flow */
			if (num > 1)
				break;

			p->is_directed = 1;
			p->num_qids_mapped = 1;
		} else if (q->type == RTE_SCHED_TYPE_ORDERED) {
			p->num_ordered_qids++;
			p->num_qids_mapped++;
		} else if (q->type == RTE_SCHED_TYPE_ATOMIC) {
			p->num_qids_mapped++;
		}

		q->cq_map[q->cq_num_mapped_cqs++] = port_id;
	}
	return i;
}

static void
sw_dump(FILE *f, const struct rte_event_dev *dev)
{
	static const char *q_type_strings[] = {"Ordered" , "Atomic",
			"Parallel", "Directed"
	};
	uint32_t i;
	const struct sw_evdev *sw = (const void *)dev;
	fprintf(f, "EventDev %s: ports %d, qids %d\n", sw->dev.name,
			sw->port_count, sw->qid_count);

	fprintf(f, "\trx   %"PRIu64"\n\tdrop %"PRIu64"\n\ttx   %"PRIu64"\n",
		sw->stats.rx_pkts, sw->stats.rx_dropped, sw->stats.tx_pkts);
	fprintf(f, "\tsched calls: %"PRIu64"\n", sw->sched_called);
	fprintf(f, "\tsched cq/qid call: %"PRIu64"\n", sw->sched_cq_qid_called);
	fprintf(f, "\tsched no IQ enq: %"PRIu64"\n", sw->sched_no_iq_enqueues);
	fprintf(f, "\tsched no CQ enq: %"PRIu64"\n", sw->sched_no_cq_enqueues);
	fprintf(f, "\toverloads %"PRIu64"\t%s\n", sw->sched_overload_counter,
			sw->overloaded ? " [OVERLOADED NOW]" : "");

#define COL_RED "\x1b[31m"
#define COL_RESET "\x1b[0m"

	for (i = 0; i < sw->port_count; i++) {
		const struct sw_port *p = &sw->ports[i];
		fprintf(f, "  Port %d %s %s\n", i,
				p->is_directed ? " (SingleCons)" : "",
				p->overloaded ? " ["COL_RED"OVERLOAD"COL_RESET"]" : "");
		fprintf(f, "\trx   %"PRIu64"\n\tdrop %"PRIu64"\n\ttx   %"PRIu64"\n"
			"\tinf %d\n", sw->ports[i].stats.rx_pkts,
			sw->ports[i].stats.rx_dropped,
			sw->ports[i].stats.tx_pkts, sw->ports[i].inflights);

		uint64_t rx_used = qe_ring_count(p->rx_worker_ring);
		uint64_t rx_free = qe_ring_free_count(p->rx_worker_ring);
		const char *rxcol = (rx_free == 0) ? COL_RED : COL_RESET;
		fprintf(f, "\t%srx ring used: %ld\tfree: %ld"COL_RESET"\n",
				rxcol, rx_used, rx_free);

		uint64_t tx_used = qe_ring_count(p->cq_worker_ring);
		uint64_t tx_free = qe_ring_free_count(p->cq_worker_ring);
		const char *txcol = (tx_free == 0) ? COL_RED : COL_RESET;
		fprintf(f, "\t%scq ring used: %ld\tfree: %ld"COL_RESET"\n",
				txcol, tx_used, tx_free);
	}

	for (i = 0; i < sw->qid_count; i++) {
		fprintf(f, "  Queue %d (%s)\n", i, q_type_strings[sw->qids[i].type]);
		fprintf(f, "\trx   %"PRIu64"\n\tdrop %"PRIu64"\n\ttx   %"PRIu64"\n",
			sw->qids[i].stats.rx_pkts, sw->qids[i].stats.rx_dropped,
			sw->qids[i].stats.tx_pkts);
		uint32_t iq;
		for(iq = 0; iq < SW_IQS_MAX; iq++) {
			uint32_t used = iq_ring_count(sw->qids[i].iq[iq]);
			uint32_t free = iq_ring_free_count(sw->qids[i].iq[iq]);
			const char *col = (free == 0) ? COL_RED : COL_RESET;
			fprintf(f, "\t%siq %d: Used %d\tFree %d"COL_RESET"\n",
					col, iq, used, free);
		}
	}
}

static int
sw_port_setup(struct rte_event_dev *dev, uint8_t port_id,
		const struct rte_event_port_conf *conf)
{
	struct sw_evdev *sw = (void *)dev;
	struct sw_port *p = &sw->ports[port_id];
	char buf[QE_RING_NAMESIZE];
	unsigned i;

	if (conf->enqueue_queue_depth >
				dev->info.max_event_port_enqueue_queue_depth ||
			conf->dequeue_queue_depth >
				dev->info.max_event_port_dequeue_queue_depth){
		rte_errno = EINVAL;
		return -1;
	}

	*p = (struct sw_port){0}; /* zero entire structure */
	p->id = port_id;

	/* TODO: how do we work with an overload scheme here?
	 * For now, still use a huge buffer, with per-port thresholds.
	 * When it fills beyond the configured max size, we throttle.
	 */
	snprintf(buf, sizeof(buf), "%s_%s", dev->name, "rx_worker_ring");
	p->rx_worker_ring = qe_ring_create(buf, MAX_SW_PROD_Q_DEPTH,
			dev->socket_id);
	if (p->rx_worker_ring == NULL)
		return -1;

	/* threshold is number of free spaces that are left in ring
	 * before overload should kick in. QE ring returns free_count,
	 * so storing this way makes more sense than actual depth
	 */
	uint32_t requested = MAX_SW_PROD_Q_DEPTH - conf->new_event_threshold;
	p->overload_threshold = requested > 255 ? 255 : requested;

	snprintf(buf, sizeof(buf), "%s_%s", dev->name, "cq_worker_ring");
	p->cq_worker_ring = qe_ring_create(buf, conf->dequeue_queue_depth,
			dev->socket_id);
	if (p->cq_worker_ring == NULL) {
		qe_ring_destroy(p->rx_worker_ring);
		return -1;
	}
	sw->cq_ring_space[port_id] = conf->dequeue_queue_depth;

	/* set hist list contents to empty */
	for (i = 0; i < SW_PORT_HIST_LIST; i++) {
		p->hist_list[i].fid = -1;
		p->hist_list[i].qid = -1;
	}

	return 0;
}

static int
sw_port_cleanup(struct sw_evdev *sw, uint8_t port_id)
{
	struct sw_port *p = &sw->ports[port_id];

	qe_ring_destroy(p->rx_worker_ring);
	qe_ring_destroy(p->cq_worker_ring);
	memset(p, 0, sizeof(*p));

	return 0;
}

static uint8_t
sw_port_count(struct rte_event_dev *dev)
{
	struct sw_evdev *sw = (void *)dev;
	return sw->port_count;
}


static uint16_t
sw_queue_count(struct rte_event_dev *dev)
{
	struct sw_evdev *sw = (void *)dev;
	return sw->qid_count;
}

static int32_t
qid_cleanup(struct sw_evdev *sw, uint32_t idx)
{
	struct sw_qid *qid = &sw->qids[idx];
	uint32_t i;

	for (i = 0; i < SW_IQS_MAX; i++) {
		iq_ring_destroy(qid->iq[i]);
	}

	if (qid->type == RTE_SCHED_TYPE_ORDERED) {
		rte_free(qid->reorder_buffer);
		rte_ring_free(qid->reorder_buffer_freelist);
	}
	memset(qid, 0, sizeof(*qid));

	return 0;
}

static int32_t
qid_init(struct sw_evdev *sw, unsigned idx, int type,
		const struct rte_event_queue_conf *queue_conf)
{
	int i;
	int socket_id = sw->dev.socket_id;
	char buf[IQ_RING_NAMESIZE];
	struct sw_qid *qid = &sw->qids[idx];

	for (i = 0; i < SW_IQS_MAX; i++) {
		snprintf(buf, sizeof(buf), "q_%u_iq_%d", idx, i);
		qid->iq[i] = iq_ring_create(buf, socket_id);
		if (!qid->iq[i]) {
			SW_LOG_DBG("ring create failed");
			goto cleanup;
		}
	}

	/* Initialize the iq packet mask to 1, as __builtin_clz() is undefined
	 * if the value passed in is zero.
	 */
	qid->iq_pkt_mask = 1;

	/* Initialize the FID structures to no pinning (-1), and zero packets */
	struct sw_fid_t fid = {.cq = -1, .count = 0};
	for (i = 0; i < SW_QID_NUM_FIDS; i++)
		qid->fids[i] = fid;

	qid->id = idx;
	qid->type = type;
	qid->priority = queue_conf->priority;

	if (qid->type == RTE_SCHED_TYPE_ORDERED) {
		uint32_t window_size;

		/* rte_ring and window_size_mask require require window_size to
		 * be a power-of-2.
		 */
		window_size = rte_align32pow2(
				queue_conf->nb_atomic_order_sequences);

		qid->window_size = window_size - 1;

		if (!window_size) {
			SW_LOG_DBG("invalid reorder_window_size for ordered queue\n");
			goto cleanup;
		}

		snprintf(buf, sizeof(buf), "%s_iq_%d_rob", sw->dev.name, i);
		qid->reorder_buffer = rte_zmalloc_socket(buf,
				window_size * sizeof(qid->reorder_buffer[0]),
				0, socket_id);
		if (!qid->reorder_buffer) {
			SW_LOG_DBG("reorder_buffer malloc failed\n");
			goto cleanup;
		}

		memset(&qid->reorder_buffer[0],
		       0,
		       window_size * sizeof(qid->reorder_buffer[0]));

		snprintf(buf, sizeof(buf), "%s_iq_%d_freelist", sw->dev.name, i);
		qid->reorder_buffer_freelist = rte_ring_create(buf,
				window_size,
				socket_id,
				RING_F_SP_ENQ | RING_F_SC_DEQ);
		if (!qid->reorder_buffer_freelist) {
			SW_LOG_DBG("freelist ring create failed");
			goto cleanup;
		}

		/* Populate the freelist with reorder buffer entries. Enqueue
		 * 'window_size - 1' entries because the rte_ring holds only
		 * that many.
		 */
		for (i = 0; i < (int) window_size - 1; i++) {
			if (rte_ring_sp_enqueue(qid->reorder_buffer_freelist,
						&qid->reorder_buffer[i]) < 0)
				goto cleanup;
		}

		qid->reorder_buffer_index = 0;
		qid->cq_next_tx = 0;
	}

	return 0;

cleanup:
	for (i = 0; i < SW_IQS_MAX; i++) {
		if (qid->iq[i])
			iq_ring_destroy(qid->iq[i]);
	}

	if (qid->reorder_buffer) {
		rte_free(qid->reorder_buffer);
		qid->reorder_buffer = NULL;
	}

	if (qid->reorder_buffer_freelist) {
		rte_ring_free(qid->reorder_buffer_freelist);
		qid->reorder_buffer_freelist = NULL;
	}

	return -EINVAL;
}

static int
sw_queue_setup(struct rte_event_dev *dev,
		uint8_t queue_id,
		const struct rte_event_queue_conf *conf)
{
	int type;
	if (conf->nb_atomic_flows > 0 &&
			conf ->nb_atomic_order_sequences > 0)
		return -1;

	if (conf->event_queue_cfg & RTE_EVENT_QUEUE_CFG_SINGLE_CONSUMER)
		type = RTE_SCHED_TYPE_DIRECT;
	else if (conf->nb_atomic_flows > 0)
		type = RTE_SCHED_TYPE_ATOMIC;
	else if (conf->nb_atomic_order_sequences > 0)
		type = RTE_SCHED_TYPE_ORDERED;
	else
		type = RTE_SCHED_TYPE_PARALLEL;

	return qid_init((void *)dev, queue_id, type, conf);
}

static int
sw_dev_configure(struct rte_event_dev *dev,
			struct rte_event_dev_config *config)
{
	struct sw_evdev *se = (void *)dev;

	if (config->nb_event_queues > dev->info.max_event_queues ||
			config->nb_event_ports > dev->info.max_event_ports)
		return -1;

	se->qid_count = config->nb_event_queues;
	se->port_count = config->nb_event_ports;
	return 0;
}

static int
assign_numa_node(const char *key __rte_unused, const char *value, void *opaque)
{
	int *socket_id = opaque;
	*socket_id = atoi(value);
	if (*socket_id > RTE_MAX_NUMA_NODES)
		return -1;
	return 0;
}

static inline void
swap_ptr(void *a, void *b)
{
	void *tmp = a;
	a = b;
	b= tmp;
}

static int
sw_start(struct rte_event_dev *dev)
{
	unsigned int i, j;
	struct sw_evdev *sw = (void *)dev;
	/* check all ports are set up */
	for (i = 0; i < sw->port_count; i++)
		if (sw->ports[i].rx_worker_ring == NULL)
			return -1;

	/* check all queues are configured and mapped to ports*/
	for (i = 0; i < sw->qid_count; i++)
		if (sw->qids[i].iq[0] == NULL ||
				sw->qids[i].cq_num_mapped_cqs == 0)
			return -1;

	/* build up our prioritized array of qids */
	/* We don't use qsort here, as if all/multiple entries have the same
	 * priority, the result is non-deterministic. From "man 3 qsort":
	 * "If two members compare as equal, their order in the sorted
	 * array is undefined."
	 */
	for (i = 0; i < sw->qid_count; i++) {
		sw->qids_prioritized[i] = &sw->qids[i];
		for (j = i; j > 0; j--)
			if (sw->qids_prioritized[j]->priority <
					sw->qids_prioritized[j-1]->priority)
				swap_ptr(sw->qids_prioritized[j],
						sw->qids_prioritized[j-1]);
	}
	sw->started = 1;
	return 0;
}

static void
sw_stop(struct rte_event_dev *dev)
{
	struct sw_evdev *sw = (void *)dev;
	sw->started = 0;
}
static int
sw_close(struct rte_event_dev *dev)
{
	struct sw_evdev *sw = (void *)dev;
	uint32_t i;

	for(i = 0; i < sw->qid_count; i++) {
		qid_cleanup(sw, i);
	}
	sw->qid_count = 0;

	for (i = 0; i < sw->port_count; i++) {
		sw_port_cleanup(sw, i);
	}
	sw->port_count = 0;

	memset(&sw->stats, 0, sizeof(sw->stats));

	return 0;
}

static int
sw_probe(const char *name, const char *params)
{
	static const struct rte_event_dev_ops evdev_sw_ops = {
			.configure = sw_dev_configure,
			.queue_setup = sw_queue_setup,
			.queue_count = sw_queue_count,
			.port_setup = sw_port_setup,
			.port_link = sw_port_link,
			.port_count = sw_port_count,
			.start = sw_start,
			.stop = sw_stop,
			.close = sw_close,
			.stats_get = sw_dev_stats_get,
			.dump = sw_dump,

			.enqueue = sw_event_enqueue,
			.enqueue_burst = sw_event_enqueue_burst,
			.dequeue = sw_event_dequeue,
			.dequeue_burst = sw_event_dequeue_burst,
			.release = sw_event_release,
			.schedule = sw_event_schedule,
	};
	static const char *args[] = { NUMA_NODE_ARG, NULL };
	const struct rte_memzone *mz;
	struct sw_evdev *se;
	struct rte_event_dev_info evdev_sw_info = {
			.driver_name = PMD_NAME,
			.max_event_queues = SW_QIDS_MAX,
			.max_event_queue_flows = SW_QID_NUM_FIDS,
			.max_event_queue_priority_levels = SW_Q_PRIORITY_MAX,
			.max_event_priority_levels = SW_IQS_MAX,
			.max_event_ports = SW_PORTS_MAX,
			.max_event_port_dequeue_queue_depth = MAX_SW_CONS_Q_DEPTH,
			.max_event_port_enqueue_queue_depth = MAX_SW_PROD_Q_DEPTH,
			/* for event limits, there is no hard limit, but it
			 * depends on number of Queues configured and depth of
			 * producer/consumer queues
			 */
			.max_num_events = -1,
			.event_dev_cap = (RTE_EVENT_DEV_CAP_QUEUE_QOS |
					RTE_EVENT_DEV_CAP_EVENT_QOS),
	};
	int socket_id = 0;

	if (params != NULL && params[0] != '\0') {
		struct rte_kvargs *kvlist = rte_kvargs_parse(params, args);

		if (!kvlist) {
			RTE_LOG(INFO, PMD,
				"Ignoring unsupported parameters when creating device '%s'\n",
				name);
		} else {
			int ret = rte_kvargs_process(kvlist, NUMA_NODE_ARG,
					assign_numa_node, &socket_id);
			rte_kvargs_free(kvlist);

			if (ret != 0) {
				RTE_LOG(ERR, PMD,
					"%s: Error parsing numa node parameter",
					name);
				return ret;
			}
		}
	}

	RTE_LOG(INFO, PMD, "Creating eventdev sw device %s, on numa node %d\n",
			name, socket_id);

	mz = rte_memzone_reserve(name, sizeof(*se), socket_id, 0);
	if (mz == NULL)
		return -1; /* memzone_reserve sets rte_errno on error */

	se = mz->addr;
	se->mz = mz;
	snprintf(se->dev.name, sizeof(se->dev.name), "%s", name);
	se->dev.configured = false;
	se->dev.info = evdev_sw_info;
	se->dev.ops = &evdev_sw_ops;
	se->dev.socket_id = socket_id;

	return rte_event_dev_register(&se->dev);
}

static int
sw_remove(const char *name)
{
	if (name == NULL)
		return -EINVAL;

	RTE_LOG(INFO, PMD, "Closing eventdev sw device %s\n", name);
	/* TODO unregister eventdev and release memzone */

	return 0;
}

static struct rte_vdev_driver evdev_sw_pmd_drv = {
	.probe = sw_probe,
	.remove = sw_remove
};

RTE_PMD_REGISTER_VDEV(evdev_sw, evdev_sw_pmd_drv);
RTE_PMD_REGISTER_PARAM_STRING(evdev_sw,"numa_node=<int>");
