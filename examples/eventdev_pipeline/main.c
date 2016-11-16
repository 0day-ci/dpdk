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

#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <signal.h>

#include <rte_eal.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_launch.h>
#include <rte_malloc.h>
#include <rte_cycles.h>
#include <rte_ethdev.h>
#include <rte_eventdev.h>

#define BATCH_SIZE 32

static unsigned int num_workers = 4;
static unsigned long num_packets = (1L << 25); /* do ~32M packets */
static unsigned int num_fids = 16;
static unsigned int num_priorities = 1;
static int sched_type = RTE_SCHED_TYPE_ATOMIC;

struct prod_data {
	uint8_t event_dev_id;
	uint8_t event_port_id;
	int32_t qid;
	unsigned num_ports;
};

struct cons_data {
	uint8_t event_dev_id;
	uint8_t event_port_id;
};

struct worker_data {
	uint8_t event_dev_id;
	int event_port_id;
	int32_t qid;
};

static volatile int done = 0;
static int quiet = 0;
struct rte_mempool *mp;

static int
worker(void *arg)
{
	struct rte_event rcv_events[BATCH_SIZE];

	struct worker_data *data = (struct worker_data *)arg;
	uint8_t event_dev_id = data->event_dev_id;
	uint8_t event_port_id = data->event_port_id;
	int32_t qid = data->qid;
	size_t sent = 0, received = 0;

	while (!done) {
		uint16_t i;

		uint16_t n = rte_event_dequeue_burst(event_dev_id,
						     event_port_id,
						     rcv_events,
						     RTE_DIM(rcv_events),
						     false);
		if (n == 0){
			rte_pause();
			/* Flush any buffered events */
			rte_event_dequeue(event_dev_id,
					  event_port_id,
					  NULL,
					  false);
			continue;
		}
		received += n;

		for (i = 0; i < n; i++) {
			struct ether_hdr *eth;
			struct ether_addr addr;
			struct rte_event *ev = &rcv_events[i];

			ev->queue_id = qid;
			ev->flow_id = 0;
			ev->priority = 0;
			ev->sched_type = RTE_SCHED_TYPE_ATOMIC;
			ev->operation = RTE_EVENT_OP_FORWARD;

			uint64_t now = rte_rdtsc();
			while(now + 750 > rte_rdtsc()) {}

			/* change mac addresses on packet */
			eth = rte_pktmbuf_mtod(ev->mbuf, struct ether_hdr *);
			ether_addr_copy(&eth->d_addr, &addr);
			ether_addr_copy(&eth->s_addr, &eth->d_addr);
			ether_addr_copy(&addr, &eth->s_addr);
		}
		int ret = rte_event_enqueue_burst(event_dev_id, event_port_id,
					rcv_events, n, false);
		if (ret != n)
			rte_panic("worker %u thread failed to enqueue event\n",
				rte_lcore_id());
	}

	/* Flush the buffered events */
	rte_event_dequeue(event_dev_id, event_port_id, NULL, false);

	if (!quiet)
		printf("  worker %u thread done. RX=%zu TX=%zu\n",
				rte_lcore_id(), received, sent);

	return 0;
}

static int
scheduler(void *arg)
{
	RTE_SET_USED(arg);
	size_t loops = 0;

	while (!done) {
		/* Assumes an event dev ID of 0 */
		rte_event_schedule(0);
		loops++;
	}

	printf("  scheduler thread done. loops=%zu\n", loops);

	return 0;
}

static int
consumer(void *arg)
{
	struct rte_event events[BATCH_SIZE];

	struct cons_data *data = (struct cons_data *)arg;
	uint8_t event_dev_id = data->event_dev_id;
	uint8_t event_port_id = data->event_port_id;
	struct rte_eth_dev_tx_buffer *tx_buf[RTE_MAX_ETHPORTS];
	size_t npackets = num_packets;
	size_t received = 0;
	size_t received_printed = 0; /* tracks when we last printed receive count */
	uint64_t start_time = 0;
	uint64_t freq_khz = rte_get_timer_hz() / 1000;
	uint64_t dropped = 0;
	unsigned i;

	for (i = 0; i < rte_eth_dev_count(); i++) {
		tx_buf[i] = rte_malloc(NULL, RTE_ETH_TX_BUFFER_SIZE(32), 0);
		if (tx_buf[i] == NULL)
			rte_panic("Out of memory\n");
		rte_eth_tx_buffer_init(tx_buf[i], 32);
		rte_eth_tx_buffer_set_err_callback(tx_buf[i],
				rte_eth_tx_buffer_count_callback, &dropped);
	}

	while (!done) {
		uint16_t i;
		uint16_t n = rte_event_dequeue_burst(event_dev_id,
						     event_port_id,
						     events,
						     RTE_DIM(events),
						     false);

		if (n == 0){
			rte_pause();
			continue;
		}
		if (start_time == 0)
			start_time = rte_get_timer_cycles();

		received += n;
		for (i = 0; i < n; i++) {
			uint8_t outport = events[i].mbuf->port;
			rte_eth_tx_buffer(outport, 0, tx_buf[outport], events[i].mbuf);
		}

		if (!quiet && received >= received_printed + (1<<22)) {
			printf("# consumer RX=%zu, time %"PRIu64"ms\n",
					received,
					(rte_get_timer_cycles() - start_time) / freq_khz);
			received_printed = received;
		}

		if (num_packets > 0 && npackets > 0) {
			npackets -= n;
			if (npackets == 0)
				done = 1;
		}
	}

	for (i = 0; i < rte_eth_dev_count(); i++)
		rte_eth_tx_buffer_flush(i, 0, tx_buf[i]);

	printf("  consumer done! RX=%zu, time %"PRIu64"ms\n",
			received,
			(rte_get_timer_cycles() - start_time) / freq_khz);

	return 0;
}

static int
producer(void *arg)
{

	struct prod_data *data = (struct prod_data *)arg;
	size_t npackets = num_packets;
	unsigned i;
	uint64_t mbuf_seqno = 0;
	size_t sent = 0;
	uint8_t eth_port = 0;
	uint8_t event_dev_id = data->event_dev_id;
	uint8_t event_port_id = data->event_port_id;
	int fid_counter = 0;

	while (!done) {
		int ret;
		unsigned num_ports = data->num_ports;
		int32_t qid = data->qid;
		struct rte_event events[BATCH_SIZE];
		struct rte_mbuf *mbufs[BATCH_SIZE];

		uint16_t nb_rx = rte_eth_rx_burst(eth_port, 0, mbufs, BATCH_SIZE);
		if (++eth_port == num_ports)
			eth_port = 0;
		if (nb_rx == 0) {
			rte_pause();
			/* Flush any buffered events */
			rte_event_dequeue(event_dev_id,
					  event_port_id,
					  NULL,
					  false);
			continue;
		}

		for (i = 0; i < nb_rx; i++) {
			struct rte_mbuf *m = mbufs[i];
			struct rte_event *ev = &events[i];

			ev->queue_id = qid;
			ev->flow_id = fid_counter++ % 6;
			ev->priority = 0;
			m->udata64 = mbuf_seqno++;
			ev->mbuf = m;
			ev->sched_type = sched_type;
			ev->operation = RTE_EVENT_OP_NEW;
		}

		do {
			ret = rte_event_enqueue_burst(event_dev_id,
							event_port_id,
							events,
							nb_rx,
							false);
		} while (ret == -ENOSPC);
		if (ret != nb_rx)
			rte_panic("producer thread failed to enqueue *all* events\n");

		sent += nb_rx;

		if (num_packets > 0 && npackets > 0) {
			npackets -= nb_rx;
			if (npackets == 0)
				break;
		}
	}

	/* Flush any buffered events */
	while (!done)
		rte_event_dequeue(event_dev_id, event_port_id, NULL, false);

	printf("  prod thread done! TX=%zu across %u flows\n", sent, num_fids);

	return 0;
}

static struct option long_options[] = {
	{"workers", required_argument, 0, 'w'},
	{"packets", required_argument, 0, 'n'},
	{"atomic-flows", required_argument, 0, 'f'},
	{"priority", required_argument, 0, 'p'},
	{"ordered", no_argument, 0, 'o'},
	{"quiet", no_argument, 0, 'q'},
	{0, 0, 0, 0}
};

static void
usage(void)
{
	const char *usage_str =
		"  Usage: eventdev_pipeline [options]\n"
		"  Options:\n"
		"  -w, --workers=N       Use N workers (default 4)\n"
		"  -n, --packets=N       Send N packets (default ~32M), 0 implies no limit\n"
		"  -f, --atomic-flows=N  Use N random flows from 1 to N (default 16)\n"
		"  -p, --priority=N      Use N number of priorities (default 1)\n"
		"  -o, --ordered         Use ordered scheduling\n"
		"  -q, --quiet           Minimize printed output\n"
		"\n";

	fprintf(stderr, "%s", usage_str);
	exit(1);
}

static void
parse_app_args(int argc, char** argv)
{
	/* Parse cli options*/
	int option_index;
	int c;
	opterr = 0;

	for (;;) {
		c = getopt_long(argc, argv, "w:n:f:p:oq", long_options,
				&option_index);
		if (c == -1)
			break;

		switch (c) {
			case 'w':
				num_workers = (unsigned int)atoi(optarg);
				break;
			case 'n':
				num_packets = (unsigned long )atol(optarg);
				break;
			case 'f':
				num_fids = (unsigned int)atoi(optarg);
				break;
			case 'p':
				num_priorities = (unsigned int)atoi(optarg);
				break;
			case 'o':
				sched_type = RTE_SCHED_TYPE_ORDERED;
				break;
			case 'q':
				quiet = 1;
				break;
			default:
				usage();
		}
	}
	if (num_workers == 0)
		usage();
}

/*
 * Initializes a given port using global settings and with the RX buffers
 * coming from the mbuf_pool passed as a parameter.
 */
static inline int
port_init(uint8_t port, struct rte_mempool *mbuf_pool)
{
	static const struct rte_eth_conf port_conf_default = {
		.rxmode = { .max_rx_pkt_len = ETHER_MAX_LEN }
	};
	const uint16_t rx_rings = 1, tx_rings = 1;
	const uint16_t rx_ring_size = 512, tx_ring_size = 512;
	struct rte_eth_conf port_conf = port_conf_default;
	int retval;
	uint16_t q;

	if (port >= rte_eth_dev_count())
		return -1;

	/* Configure the Ethernet device. */
	retval = rte_eth_dev_configure(port, rx_rings, tx_rings, &port_conf);
	if (retval != 0)
		return retval;

	/* Allocate and set up 1 RX queue per Ethernet port. */
	for (q = 0; q < rx_rings; q++) {
		retval = rte_eth_rx_queue_setup(port, q, rx_ring_size,
				rte_eth_dev_socket_id(port), NULL, mbuf_pool);
		if (retval < 0)
			return retval;
	}

	/* Allocate and set up 1 TX queue per Ethernet port. */
	for (q = 0; q < tx_rings; q++) {
		retval = rte_eth_tx_queue_setup(port, q, tx_ring_size,
				rte_eth_dev_socket_id(port), NULL);
		if (retval < 0)
			return retval;
	}

	/* Start the Ethernet port. */
	retval = rte_eth_dev_start(port);
	if (retval < 0)
		return retval;

	/* Display the port MAC address. */
	struct ether_addr addr;
	rte_eth_macaddr_get(port, &addr);
	printf("Port %u MAC: %02" PRIx8 " %02" PRIx8 " %02" PRIx8
			   " %02" PRIx8 " %02" PRIx8 " %02" PRIx8 "\n",
			(unsigned)port,
			addr.addr_bytes[0], addr.addr_bytes[1],
			addr.addr_bytes[2], addr.addr_bytes[3],
			addr.addr_bytes[4], addr.addr_bytes[5]);

	/* Enable RX in promiscuous mode for the Ethernet device. */
	rte_eth_promiscuous_enable(port);

	return 0;
}

static int
init_ports(unsigned num_ports)
{
	uint8_t portid;

	mp = rte_pktmbuf_pool_create("packet_pool",
			/* mbufs */ 16384 * num_ports,
			/* cache_size */ 512,
			/* priv_size*/ 0,
			/* data_room_size */ RTE_MBUF_DEFAULT_BUF_SIZE,
			rte_socket_id());

	for (portid = 0; portid < num_ports; portid++)
		if (port_init(portid, mp) != 0)
			rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu8 "\n",
					portid);
	return 0;
}

static uint8_t
setup_event_dev(struct prod_data *prod_data,
		struct cons_data *cons_data,
		struct worker_data *worker_data)
{
	struct rte_event_dev_config config;
	struct rte_event_queue_conf queue_config;
	struct rte_event_port_conf port_config;
	struct rte_event_queue_link link;
	int prod_port;
	int cons_port;
	int qid0;
	int cons_qid;
	int prod_qid;
	unsigned i;
	int ret;
	int8_t id;

	const char *dev_name = "evdev_sw0";
	id = rte_event_dev_get_dev_id(dev_name);
	if (id < 0)
		rte_panic("Failed to get %s device ID\n", dev_name);

	config.nb_event_queues = 3;
	config.nb_event_ports = num_workers + 2;
	config.nb_events_limit = 256;
	config.dequeue_wait_ns = 0;

	ret = rte_event_dev_configure(id, &config);
	if (ret)
		rte_panic("Failed to configure the event dev\n");

	/* Create queues */
	queue_config.event_queue_cfg = RTE_EVENT_QUEUE_CFG_ATOMIC_ONLY;
	queue_config.priority = 0;

	qid0 = 0;
	ret = rte_event_queue_setup(id, qid0, &queue_config);
	if (ret < 0)
		rte_panic("Failed to create the scheduled QID\n");

	queue_config.event_queue_cfg = RTE_EVENT_QUEUE_CFG_SINGLE_CONSUMER;
	queue_config.priority = 0;

	cons_qid = 1;
	ret = rte_event_queue_setup(id, cons_qid, &queue_config);
	if (ret < 0)
		rte_panic("Failed to create the cons directed QID\n");

	queue_config.event_queue_cfg = RTE_EVENT_QUEUE_CFG_SINGLE_CONSUMER;
	queue_config.priority = 0;

	prod_qid = 2;
	ret = rte_event_queue_setup(id, prod_qid, &queue_config);
	if (ret < 0)
		rte_panic("Failed to create the prod directed QID\n");

	/* Create ports */
#define LB_PORT_DEPTH 16
#define DIR_PORT_DEPTH 32
	port_config.enqueue_queue_depth = LB_PORT_DEPTH;
	port_config.dequeue_queue_depth = LB_PORT_DEPTH;
	port_config.new_event_threshold = 255;

	prod_port = 0;
	ret = rte_event_port_setup(id, prod_port, &port_config);
	if (ret < 0)
		rte_panic("Failed to create the producer port\n");

	cons_port = 1;
	port_config.enqueue_queue_depth = DIR_PORT_DEPTH;
	port_config.dequeue_queue_depth = DIR_PORT_DEPTH;
	ret = rte_event_port_setup(id, cons_port, &port_config);
	if (ret < 0)
		rte_panic("Failed to create the consumer port\n");

	port_config.enqueue_queue_depth = LB_PORT_DEPTH;
	port_config.dequeue_queue_depth = LB_PORT_DEPTH;
	for (i = 0; i < num_workers; i++) {
		worker_data[i].event_port_id = i + 2;
		ret = rte_event_port_setup(id, worker_data[i].event_port_id, &port_config);
		if (ret < 0)
			rte_panic("Failed to create worker port #%d\n", i);
	}

	/* Map ports/qids */
	for (i = 0; i < num_workers; i++) {
		link.queue_id = qid0;
		link.priority = 0;

		ret = rte_event_port_link(id, worker_data[i].event_port_id, &link, 1);
		if (ret != 1)
			rte_panic("Failed to map worker%d port to qid0\n", i);
	}

	/* Link consumer port to its QID */
	link.queue_id = cons_qid;
	link.priority = 0;

	ret = rte_event_port_link(id, cons_port, &link, 1);
	if (ret != 1)
		rte_panic("Failed to map consumer port to cons_qid\n");

	/* Link producer port to its QID */
	link.queue_id = prod_qid;
	link.priority = 0;

	ret = rte_event_port_link(id, prod_port, &link, 1);
	if (ret != 1)
		rte_panic("Failed to map producer port to prod_qid\n");

	/* Dispatch to slaves */
	*prod_data = (struct prod_data){.event_dev_id = id,
					.event_port_id = prod_port,
					.qid = qid0};
	*cons_data = (struct cons_data){.event_dev_id = id,
					.event_port_id = cons_port};

	for (i = 0; i < num_workers; i++) {
		struct worker_data *w = &worker_data[i];
		w->event_dev_id = id;
		w->qid = cons_qid;
	}

	if (rte_event_dev_start(id) < 0) {
		printf("%d: Error with start call\n", __LINE__);
		return -1;
	}

	return (uint8_t) id;
}

static void sighndlr(int sig)
{
	/* Ctlr-Z to dump stats */
	if(sig == SIGTSTP) {
		rte_mempool_dump(stdout, mp);
		rte_event_dev_dump(stdout, 0);
	}
	/* Ctlr-C to exit */
	if(sig == SIGINT)
		rte_exit(0, "sigint arrived, quitting\n");
}

int
main(int argc, char **argv)
{
	signal(SIGINT , sighndlr);
	signal(SIGTSTP, sighndlr);

	struct prod_data prod_data = {0};
	struct cons_data cons_data = {0};
	struct worker_data *worker_data;
	unsigned nworkers = 0;
	unsigned num_ports;
	int lcore_id;
	int err;
	int has_prod = 0;
	int has_cons = 0;
	int has_scheduler = 0;

	err = rte_eal_init(argc, argv);
	if (err < 0)
		rte_panic("Invalid EAL arguments\n");

	argc -= err;
	argv += err;

	/* Parse cli options*/
	parse_app_args(argc, argv);

	num_ports = rte_eth_dev_count();
	if (num_ports == 0)
		rte_panic("No ethernet ports found\n");

	if (!quiet) {
		printf("  Config:\n");
		printf("\tports: %u\n", num_ports);
		printf("\tworkers: %u\n", num_workers);
		printf("\tpackets: %lu\n", num_packets);
		printf("\tflows: %u\n", num_fids);
		printf("\tpriorities: %u\n", num_priorities);
		if (sched_type == RTE_SCHED_TYPE_ORDERED)
			printf("\tqid0 type: ordered\n");
		if (sched_type == RTE_SCHED_TYPE_ATOMIC)
			printf("\tqid0 type: atomic\n");
		printf("\n");
	}

	const unsigned cores_needed = num_workers +
			/*main*/1 +
			/*sched*/1 +
			/*TX*/1 +
			/*RX*/1;

	if (!quiet) {
		printf("Number of cores available: %u\n", rte_lcore_count());
		printf("Number of cores to be used: %u\n", cores_needed);
	}

	if (rte_lcore_count() < cores_needed)
		rte_panic("Too few cores\n");

	const uint8_t ndevs = rte_event_dev_count();
	if (ndevs == 0)
		rte_panic("No event devs found. Do you need to pass in a --vdev flag?\n");
	if (ndevs > 1)
		fprintf(stderr, "Warning: More than one event dev, using idx 0");

	worker_data = rte_calloc(0, num_workers, sizeof(worker_data[0]), 0);
	if (worker_data == NULL)
		rte_panic("rte_calloc failed\n");

	uint8_t id = setup_event_dev(&prod_data, &cons_data, worker_data);
	RTE_SET_USED(id);

	prod_data.num_ports = num_ports;
	init_ports(num_ports);

	RTE_LCORE_FOREACH_SLAVE(lcore_id) {
		if (has_prod && has_cons && has_scheduler && nworkers == num_workers)
			break;

		if (!has_scheduler) {
			err = rte_eal_remote_launch(scheduler, NULL, lcore_id);
			if (err)
				rte_panic("Failed to launch scheduler\n");

			has_scheduler = 1;
			continue;
		}

		if (nworkers < num_workers) {
			err = rte_eal_remote_launch(worker, &worker_data[nworkers], lcore_id);
			if (err)
				rte_panic("Failed to launch worker%d\n", nworkers);
			nworkers++;
			continue;
		}

		if (!has_cons) {
			err = rte_eal_remote_launch(consumer, &cons_data, lcore_id);
			if (err)
				rte_panic("Failed to launch consumer\n");
			has_cons = 1;
			continue;
		}

		if (!has_prod) {
			err = rte_eal_remote_launch(producer, &prod_data, lcore_id);
			if (err)
				rte_panic("Failed to launch producer\n");
			has_prod = 1;
			continue;
		}
	}

	rte_eal_mp_wait_lcore();

	/* Cleanup done automatically by kernel on app exit */

	return 0;
}
