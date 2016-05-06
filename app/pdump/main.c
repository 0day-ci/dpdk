/*
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
#include <stdarg.h>
#include <inttypes.h>
#include <sys/queue.h>
#include <stdlib.h>
#include <getopt.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <net/if.h>

/* sys/un.h with __USE_MISC uses strlen, which is unsafe */
#ifdef __USE_MISC
#define REMOVED_USE_MISC
#undef __USE_MISC
#endif
#include <sys/un.h>
/* make sure we redefine __USE_MISC only if it was previously undefined */
#ifdef REMOVED_USE_MISC
#define __USE_MISC
#undef REMOVED_USE_MISC
#endif

#include <rte_eal.h>
#include <rte_common.h>
#include <rte_debug.h>
#include <rte_ethdev.h>
#include <rte_malloc.h>
#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_launch.h>
#include <rte_tailq.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_debug.h>
#include <rte_log.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_string_fns.h>
#include <rte_errno.h>
#include <rte_dev.h>
#include <rte_pci.h>
#include <rte_kvargs.h>
#include <rte_mempool.h>
#include <rte_ring.h>
#include <rte_pdump.h>

#define PDUMP_PORT_ARG "port"
#define PDUMP_PCI_ARG "device_id"
#define PDUMP_QUEUE_ARG "queue"
#define PDUMP_DIR_ARG "dir"
#define PDUMP_RX_DEV_ARG "rx-dev"
#define PDUMP_TX_DEV_ARG "tx-dev"
#define PDUMP_RXTX_DEV_ARG "rxtx-dev"
#define PDUMP_RING_SIZE_ARG "ring-size"
#define PDUMP_MBUF_SIZE_ARG "mbuf-size"
#define PDUMP_NUM_MBUFS_ARG "total-num-mbufs"

#define VDEV_NAME "eth_pcap_%s_%d"
#define VDEV_TX_PCAP "tx_pcap=%s"
#define VDEV_TX_IFACE "tx_iface=%s"
#define TX_STREAM_SIZE 64

#define MP_NAME "pdump_pool_%d"

#define RXTX_RING "rxtx_ring_%d"
#define RX_RING "rx_ring_%d"
#define TX_RING "tx_ring_%d"

#define RXTX_STR "rxtx"
#define RX_STR "rx"
#define TX_STR "tx"

/* Maximum long option length for option parsing. */
#define APP_ARG_TCPDUMP_MAX_TUPLES 54
#define MBUF_POOL_CACHE_SIZE 250
#define TX_DESC_PER_QUEUE 512
#define RX_DESC_PER_QUEUE 128
#define MBUFS_PER_POOL 65535
#define MAX_LONG_OPT_SZ 64
#define RING_SIZE 16384
#define ARRAY_SIZE 256
#define BURST_SIZE 32
#define NUM_VDEVS 2

enum pdump_en_dis {
	DISABLE = 1,
	ENABLE = 2
};

enum pcap_stream {
	IFACE = 1,
	PCAP = 2
};

enum pdump_by {
	PORT_ID = 1,
	DEVICE_ID = 2
};

const char *valid_pdump_arguments[] = {
	PDUMP_PORT_ARG,
	PDUMP_PCI_ARG,
	PDUMP_QUEUE_ARG,
	PDUMP_DIR_ARG,
	PDUMP_RX_DEV_ARG,
	PDUMP_TX_DEV_ARG,
	PDUMP_RXTX_DEV_ARG,
	PDUMP_RING_SIZE_ARG,
	PDUMP_MBUF_SIZE_ARG,
	PDUMP_NUM_MBUFS_ARG,
	NULL
};

struct pdump_stats {
	uint64_t dequeue_pkts;
	uint64_t tx_pkts;
	uint64_t freed_pkts;
};

struct pdump_tuples {
	/* cli params */
	uint8_t port;
	char *device_id;
	uint16_t queue;
	char rx_dev[TX_STREAM_SIZE];
	char tx_dev[TX_STREAM_SIZE];
	char rxtx_dev[TX_STREAM_SIZE];
	uint32_t ring_size;
	uint16_t mbuf_data_size;
	uint32_t total_num_mbufs;

	/* params for library API call */
	uint32_t dir;
	struct rte_mempool *mp;
	struct rte_ring *rx_ring;
	struct rte_ring *tx_ring;
	struct rte_ring *rxtx_ring;

	/* params for packet dumping */
	enum pdump_by dump_by_type;
	int rx_vdev_id;
	int tx_vdev_id;
	int rxtx_vdev_id;
	enum pcap_stream rx_vdev_stream_type;
	enum pcap_stream tx_vdev_stream_type;
	enum pcap_stream rxtx_vdev_stream_type;
	bool single_pdump_dev;

	/* stats */
	struct pdump_stats stats;
} __rte_cache_aligned;
static struct pdump_tuples pdump_t[APP_ARG_TCPDUMP_MAX_TUPLES];

int num_tuples;
static struct rte_eth_conf port_conf_default;
volatile uint8_t quit_signal;

/**< display usage */
static void
pdump_usage(const char *prgname)
{
	printf("%s [EAL options] -- --pdump='("
	"port=<port_id>|device_id=<pci address or device name>),"
	"(queue=2),"
	"(rx-dev=<iface/path to pcap file>|"
	"tx-dev=<iface/path to pcap file>"
	"|rxtx-dev=<iface/path to pcap file>),"
	"[ring-size=<size of ring>default:16384],"
	"[mbuf-size=<mbuf data room size>default:2176],"
	"[total-num-mbufs=<number mbufs>default:65535]"
	")'\n",
	prgname);
}

static int
parse_port(const char *key __rte_unused, const char *value, void *extra_args)
{
	int n;
	struct pdump_tuples *pt = extra_args;

	n = atoi(value);
	if (n >= RTE_MAX_ETHPORTS) {
		printf("port %d >= RTE_MAX_ETHPORTS(%d)\n", n, RTE_MAX_ETHPORTS);
		return -1;
	}
	pt->port = (uint8_t) n;
	pt->dump_by_type = PORT_ID;

	return 0;
}

static int
parse_device_id(const char *key __rte_unused, const char *value, void *extra_args)
{
	struct pdump_tuples *pt = extra_args;

	pt->device_id = strdup(value);
	pt->dump_by_type = DEVICE_ID;

	return 0;
}

static int
parse_queue(const char *key __rte_unused, const char *value, void *extra_args)
{
	int n;
	struct pdump_tuples *pt = extra_args;

	if (!strcmp(value, "*"))
		pt->queue = RTE_PDUMP_ALL_QUEUES;
	else {
		n = atoi(value);
		if (n >= 0)
			pt->queue = (uint16_t) n;
		else {
			printf("queue id %d invalid - must be >= 0\n", n);
			return -1;
		}
	}
	return 0;
}

static int
parse_rxtxdev(const char *key, const char *value, void *extra_args)
{

	struct pdump_tuples *pt = extra_args;

	if (!strcmp(key, PDUMP_RX_DEV_ARG)) {
		strncpy(pt->rx_dev, value, strlen(value));
		/* identify the tx stream type for pcap vdev */
		if (if_nametoindex(pt->rx_dev))
			pt->rx_vdev_stream_type = IFACE;
	} else if (!strcmp(key, PDUMP_TX_DEV_ARG)) {
		strncpy(pt->tx_dev, value, strlen(value));
		/* identify the tx stream type for pcap vdev */
		if (if_nametoindex(pt->tx_dev))
			pt->tx_vdev_stream_type = IFACE;
	} else if (!strcmp(key, PDUMP_RXTX_DEV_ARG)) {
		strncpy(pt->rxtx_dev, value, strlen(value));
		/* identify the tx stream type for pcap vdev */
		if (if_nametoindex(pt->rxtx_dev))
			pt->rxtx_vdev_stream_type = IFACE;
	} else {
		printf("dev type %s invalid must be rx|tx|rxtx\n", value);
		return -1;
	}

	return 0;
}

static int
parse_ring_size(const char *key __rte_unused, const char *value, void *extra_args)
{
	int n;
	struct pdump_tuples *pt = extra_args;

	n = atoi(value);
	if (n >= 0)
		pt->ring_size = (uint32_t) n;
	else {
		printf("ring_size %d invalid - must be >= 0\n", n);
		return -1;
	}

	return 0;
}

static int
parse_mbuf_data_size(const char *key __rte_unused, const char *value, void *extra_args)
{
	int n;
	struct pdump_tuples *pt = extra_args;

	n = atoi(value);
	if (n > 0 && n <= 0xFFFF)
		pt->mbuf_data_size = (uint16_t) n;
	else {
		printf("mbuf_data_size %d invalid - must be > 0 and < 65536\n", n);
		return -1;
	}

	return 0;
}

static int
parse_num_mbufs(const char *key __rte_unused, const char *value, void *extra_args)
{
	int n;
	struct pdump_tuples *pt = extra_args;

	n = atoi(value);
	if (n > 1024)
		pt->total_num_mbufs = (uint16_t) n;
	else {
		printf("total-num-mbufs %d invalid - must be > 1024\n", n);
		return -1;
	}

	return 0;
}

static int
parse_pdump(const char *optarg)
{
	struct rte_kvargs *kvlist;
	int ret = 0, cnt1, cnt2, cnt3;
	struct pdump_tuples *pt;

	pt = &pdump_t[num_tuples];

	/* initial check for invalid arguments */
	kvlist = rte_kvargs_parse(optarg, valid_pdump_arguments);
	if (kvlist == NULL) {
		printf("invalid arguments passed in --pdump parameter\n");
		return -1;
	}

	/* port/device_id parsing and validation */
	cnt1 = rte_kvargs_count(kvlist, PDUMP_PORT_ARG);
	cnt2 = rte_kvargs_count(kvlist, PDUMP_PCI_ARG);
	if (!((cnt1 == 1 && cnt2 == 0) || (cnt1 == 0 && cnt2 == 1))) {
		printf("--pdump parameter must have either port id or device_id "
			"( i.e.pci or dev name) address argument\n");
		ret = -1;
		goto free_kvlist;
	} else if (cnt1 == 1)
		ret = rte_kvargs_process(kvlist, PDUMP_PORT_ARG,
				&parse_port, pt);
	else if (cnt2 == 1)
		ret = rte_kvargs_process(kvlist, PDUMP_PCI_ARG,
				&parse_device_id, pt);
	if (ret < 0)
		goto free_kvlist;

	/* queue parsing and validation */
	cnt1 = rte_kvargs_count(kvlist, PDUMP_QUEUE_ARG);
	if (cnt1 != 1) {
		printf("--pdump parameter must have queue argument\n");
		ret = -1;
		goto free_kvlist;
	}
	ret = rte_kvargs_process(kvlist, PDUMP_QUEUE_ARG, &parse_queue, pt);
	if (ret < 0)
		goto free_kvlist;

	cnt1 = rte_kvargs_count(kvlist, PDUMP_RXTX_DEV_ARG);
	cnt2 = rte_kvargs_count(kvlist, PDUMP_RX_DEV_ARG);
	cnt3 = rte_kvargs_count(kvlist, PDUMP_TX_DEV_ARG);
	if (cnt1 == 0 && cnt2 == 0 && cnt3 == 0) {
		printf("--pdump must have either rx/tx/rxtx device for packet capturing\n");
		ret = -1;
		goto free_kvlist;
	} else if (cnt1 == 1 && (cnt2 >= 1 || cnt3 >= 1)) {
		printf("--pdump must have either rx and/or tx devices or"
			" rxtx device only for packet capturing\n");
		ret = -1;
		goto free_kvlist;
	} else if (cnt2 == 1 && cnt3 == 1) {
		ret = rte_kvargs_process(kvlist, PDUMP_RX_DEV_ARG, &parse_rxtxdev, pt);
		if (ret < 0)
			goto free_kvlist;
		ret = rte_kvargs_process(kvlist, PDUMP_TX_DEV_ARG, &parse_rxtxdev, pt);
		if (ret < 0)
			goto free_kvlist;
		pt->dir = RTE_PDUMP_FLAG_RXTX;
	} else if (cnt1 ==  1) {
		ret = rte_kvargs_process(kvlist, PDUMP_RXTX_DEV_ARG, &parse_rxtxdev, pt);
		if (ret < 0)
			goto free_kvlist;
		pt->single_pdump_dev = true;
		pt->dir = RTE_PDUMP_FLAG_RXTX;
	} else if (cnt2 == 1) {
		ret = rte_kvargs_process(kvlist, PDUMP_RX_DEV_ARG, &parse_rxtxdev, pt);
		if (ret < 0)
			goto free_kvlist;
		pt->dir = RTE_PDUMP_FLAG_RX;
	} else if (cnt3 == 1) {
		ret = rte_kvargs_process(kvlist, PDUMP_TX_DEV_ARG, &parse_rxtxdev, pt);
		if (ret < 0)
			goto free_kvlist;
		pt->dir = RTE_PDUMP_FLAG_TX;
	}

	/* optional */
	/* ring_size parsing and validation */
	cnt1 = rte_kvargs_count(kvlist, PDUMP_RING_SIZE_ARG);
	if (cnt1 == 1) {
		ret = rte_kvargs_process(kvlist, PDUMP_RING_SIZE_ARG,
						&parse_ring_size, pt);
		if (ret < 0)
			goto free_kvlist;
	} else
		pt->ring_size = RING_SIZE;

	/* mbuf_data_size parsing and validation */
	cnt1 = rte_kvargs_count(kvlist, PDUMP_MBUF_SIZE_ARG);
	if (cnt1 == 1) {
		ret = rte_kvargs_process(kvlist, PDUMP_MBUF_SIZE_ARG,
						&parse_mbuf_data_size, pt);
		if (ret < 0)
			goto free_kvlist;
	} else
		pt->mbuf_data_size = RTE_MBUF_DEFAULT_BUF_SIZE;

	/* total_num_mbufs parsing and validation */
	cnt1 = rte_kvargs_count(kvlist, PDUMP_NUM_MBUFS_ARG);
	if (cnt1 == 1) {
		ret = rte_kvargs_process(kvlist, PDUMP_NUM_MBUFS_ARG,
						&parse_num_mbufs, pt);
		if (ret < 0)
			goto free_kvlist;
	} else
		pt->total_num_mbufs = MBUFS_PER_POOL;

	num_tuples++;
	return 0;

free_kvlist:
		rte_kvargs_free(kvlist);
		return ret;
}

/* Parse the argument given in the command line of the application */
static int
launch_args_parse(int argc, char **argv)
{
	int opt, ret;
	int option_index;
	char *prgname = argv[0];
	static struct option long_option[] = {
		{"pdump", 1, 0, 0},
		{NULL, 0, 0, 0}
	};

	if (argc == 1)
		pdump_usage(prgname);

	/* Parse command line */
	while ((opt = getopt_long(argc, argv, " ",
			long_option, &option_index)) != EOF) {
		switch (opt) {
		case 0:
			if (!strncmp(long_option[option_index].name, "pdump",
					MAX_LONG_OPT_SZ)) {
				ret = parse_pdump(optarg);
				if (ret) {
					printf("invalid pdump\n");
					pdump_usage(prgname);
					return -1;
				}
			}
			break;
		default:
			pdump_usage(prgname);
			return -1;
		}
	}

	return 0;
}

static void
print_pdump_stats(void)
{
	int i;
	struct pdump_tuples *pt;

	for (i = 0; i < num_tuples; i++) {
		printf("##### PDUMP DEBUG STATS #####\n");
		pt = &pdump_t[i];
		printf(" -packets dequeued:			%"PRIu64"\n",
							pt->stats.dequeue_pkts);
		printf(" -packets transmitted to vdev:		%"PRIu64"\n",
							pt->stats.tx_pkts);
		printf(" -packets freed:			%"PRIu64"\n",
							pt->stats.freed_pkts);
	}
}

static inline void
disable_pdump(struct pdump_tuples *pt)
{
	if (pt->dump_by_type == DEVICE_ID)
		rte_pdump_disable_by_deviceid(pt->device_id, pt->queue, pt->dir);
	else if (pt->dump_by_type == PORT_ID)
		rte_pdump_disable(pt->port, pt->queue, pt->dir);
}

static void
free_ring_data(struct rte_ring *ring, uint8_t vdev_id, struct pdump_stats *stats)
{
	while (rte_ring_count(ring)) {
		/* write input packets of port to vdev for pdump */
		struct rte_mbuf *rxtx_bufs[BURST_SIZE];

		/* first dequeue packets from ring of primary process */
		const uint16_t nb_in_deq = rte_ring_dequeue_burst(ring,
				(void *)rxtx_bufs, BURST_SIZE);
		stats->dequeue_pkts += nb_in_deq;

		if (nb_in_deq) {
			/* then sent on vdev */
			uint16_t nb_in_txd = rte_eth_tx_burst(
					vdev_id,
					0, rxtx_bufs, nb_in_deq);
			stats->tx_pkts += nb_in_txd;

			if (unlikely(nb_in_txd < nb_in_deq)) {
				do {
					rte_pktmbuf_free(rxtx_bufs[nb_in_txd]);
					stats->freed_pkts++;
				} while (++nb_in_txd < nb_in_deq);
			}
		}
	}
}

static void
cleanup_pdump_resources(void)
{
	int i;
	struct pdump_tuples *pt;

	/* disable pdump and free the pdump_tuple resources */
	for (i = 0; i < num_tuples; i++) {
		pt = &pdump_t[i];

		/* remove callbacks */
		disable_pdump(pt);

		/*
		* transmit rest enqueued packets of the rings to vdev,
		* in order to release mbufs to the mepool
		**/
		if (pt->single_pdump_dev && pt->dir == RTE_PDUMP_FLAG_RXTX)
			free_ring_data(pt->rxtx_ring, pt->rxtx_vdev_id, &pt->stats);
		else if (pt->dir == RTE_PDUMP_FLAG_RXTX) {
			free_ring_data(pt->rx_ring, pt->rx_vdev_id, &pt->stats);
			free_ring_data(pt->tx_ring, pt->tx_vdev_id, &pt->stats);
		} else if (pt->dir == RTE_PDUMP_FLAG_RX)
			free_ring_data(pt->rx_ring, pt->rx_vdev_id, &pt->stats);
		else if (pt->dir == RTE_PDUMP_FLAG_TX)
			free_ring_data(pt->tx_ring, pt->tx_vdev_id, &pt->stats);

		if (pt->device_id)
			free(pt->device_id);

		/* free the rings */
		if (pt->rx_ring)
			rte_ring_free(pt->rx_ring);
		if (pt->tx_ring)
			rte_ring_free(pt->tx_ring);
		if (pt->rxtx_ring)
			rte_ring_free(pt->rxtx_ring);
	}
}

static void
signal_handler(int sig_num)
{

	if (sig_num == SIGINT) {
		printf("\n\nSignal %d received, preparing to exit...\n",
				sig_num);
		quit_signal = 1;
	}

	cleanup_pdump_resources();
	/* dump debug stats */
	print_pdump_stats();
}

static inline int
configure_vdev(uint8_t port_id)
{
	struct ether_addr addr;
	const uint16_t rxRings = 0, txRings = 1;
	const uint8_t nb_ports = rte_eth_dev_count();
	int ret;
	uint16_t q;

	if (port_id > nb_ports)
		return -1;

	ret = rte_eth_dev_configure(port_id, rxRings, txRings, &port_conf_default);
	if (ret != 0)
		rte_exit(EXIT_FAILURE, "dev config failed\n");

	 for (q = 0; q < txRings; q++) {
		ret = rte_eth_tx_queue_setup(port_id, q, TX_DESC_PER_QUEUE,
				rte_eth_dev_socket_id(port_id), NULL);
		if (ret < 0)
			rte_exit(EXIT_FAILURE, "queue setup failed\n");
	}

	ret = rte_eth_dev_start(port_id);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "dev start failed\n");

	rte_eth_macaddr_get(port_id, &addr);
	printf("Port %u MAC: %02"PRIx8" %02"PRIx8" %02"PRIx8
			" %02"PRIx8" %02"PRIx8" %02"PRIx8"\n",
			(unsigned)port_id,
			addr.addr_bytes[0], addr.addr_bytes[1],
			addr.addr_bytes[2], addr.addr_bytes[3],
			addr.addr_bytes[4], addr.addr_bytes[5]);

	rte_eth_promiscuous_enable(port_id);

	return 0;
}

static void
create_mp_ring_vdev(void)
{
	int i;
	struct pdump_tuples *pt = NULL;
	struct rte_mempool *mbuf_pool = NULL;
	char vdev_name[ARRAY_SIZE];
	char vdev_params[ARRAY_SIZE];
	char ring_name[ARRAY_SIZE];
	char mempool_name[ARRAY_SIZE];
	uint8_t nb_ports = rte_eth_dev_count();

	for (i = 0; i < num_tuples; i++) {
		pt = &pdump_t[i];
		/* create rxtx_ring */
		snprintf(mempool_name, ARRAY_SIZE, MP_NAME, i);
		mbuf_pool = rte_mempool_lookup(mempool_name);
		if (mbuf_pool == NULL) {
			/* create mempool */
			mbuf_pool = rte_pktmbuf_pool_create(mempool_name,
					pt->total_num_mbufs,
					MBUF_POOL_CACHE_SIZE, 0, pt->mbuf_data_size,
					rte_socket_id());
			if (mbuf_pool == NULL)
				rte_exit(EXIT_FAILURE, "%s\n", rte_strerror(rte_errno));
		}
		pt->mp = mbuf_pool;

		if (pt->dir == RTE_PDUMP_FLAG_RXTX && pt->single_pdump_dev) {
			/* create rxtx_ring */
			snprintf(ring_name, ARRAY_SIZE, RXTX_RING, i);
			pt->rxtx_ring = rte_ring_create(ring_name, pt->ring_size,
							rte_socket_id(), 0);
			if (pt->rxtx_ring == NULL)
				rte_exit(EXIT_FAILURE, "%s:%s:%d\n",
					rte_strerror(rte_errno), __func__, __LINE__);

			/* create vdevs */
			snprintf(vdev_name, ARRAY_SIZE, VDEV_NAME, RXTX_STR, i);
			if (pt->rxtx_vdev_stream_type == IFACE)
				snprintf(vdev_params, ARRAY_SIZE, VDEV_TX_IFACE,
						pt->rxtx_dev);
			else
				snprintf(vdev_params, ARRAY_SIZE, VDEV_TX_PCAP,
						pt->rxtx_dev);
			if (rte_eal_vdev_init(vdev_name, vdev_params) < 0)
				rte_exit(EXIT_FAILURE, "vdev creation failed:%s:%d\n",
						__func__, __LINE__);
			pt->rxtx_vdev_id = nb_ports++;

			/* configure vdev */
			configure_vdev(pt->rxtx_vdev_id);

		} else if (pt->dir == RTE_PDUMP_FLAG_RXTX) {

			/* create rx_ring */
			snprintf(ring_name, ARRAY_SIZE, RX_RING, i);
			pt->rx_ring = rte_ring_create(ring_name, pt->ring_size,
							rte_socket_id(), 0);
			if (pt->rx_ring == NULL)
				rte_exit(EXIT_FAILURE, "%s:%s:%d\n",
						rte_strerror(rte_errno),
						__func__, __LINE__);

			/* create tx_ring */
			snprintf(ring_name, ARRAY_SIZE, TX_RING, i);
			pt->tx_ring = rte_ring_create(ring_name, pt->ring_size,
							rte_socket_id(), 0);
			if (pt->tx_ring == NULL)
				rte_exit(EXIT_FAILURE, "%s:%s:%d\n",
					rte_strerror(rte_errno),
					__func__, __LINE__);

			/* create vdevs */
			snprintf(vdev_name, ARRAY_SIZE, VDEV_NAME, RX_STR, i);
			if (pt->rx_vdev_stream_type == IFACE)
				snprintf(vdev_params, ARRAY_SIZE, VDEV_TX_IFACE,
						pt->rx_dev);
			else
				snprintf(vdev_params, ARRAY_SIZE, VDEV_TX_PCAP,
						pt->rx_dev);
			if (rte_eal_vdev_init(vdev_name, vdev_params) < 0)
				rte_exit(EXIT_FAILURE, "vdev creation failed:%s:%d\n",
					__func__, __LINE__);
			pt->rx_vdev_id = nb_ports++;
			/* configure vdev */
			configure_vdev(pt->rx_vdev_id);

			snprintf(vdev_name, ARRAY_SIZE, VDEV_NAME, TX_STR, i);
			if (pt->tx_vdev_stream_type == IFACE)
				snprintf(vdev_params, ARRAY_SIZE, VDEV_TX_IFACE,
						pt->tx_dev);
			else
				snprintf(vdev_params, ARRAY_SIZE, VDEV_TX_PCAP,
						pt->tx_dev);
			if (rte_eal_vdev_init(vdev_name, vdev_params) < 0)
				rte_exit(EXIT_FAILURE, "vdev creation failed:%s:%d\n",
						__func__, __LINE__);
			pt->tx_vdev_id = nb_ports++;
			/* configure vdev */
			configure_vdev(pt->tx_vdev_id);

		} else if (pt->dir == RTE_PDUMP_FLAG_RX) {

			/* create rx_ring */
			snprintf(ring_name, ARRAY_SIZE, RX_RING, i);
			pt->rx_ring = rte_ring_create(ring_name, pt->ring_size,
							rte_socket_id(), 0);
			if (pt->rx_ring == NULL)
				rte_exit(EXIT_FAILURE, "%s\n", rte_strerror(rte_errno));

			/* create vdevs */
			snprintf(vdev_name, ARRAY_SIZE, VDEV_NAME, RX_STR, i);
			if (pt->rx_vdev_stream_type == IFACE)
				snprintf(vdev_params, ARRAY_SIZE, VDEV_TX_IFACE,
						pt->rx_dev);
			else
				snprintf(vdev_params, ARRAY_SIZE, VDEV_TX_PCAP,
						pt->rx_dev);
			if (rte_eal_vdev_init(vdev_name, vdev_params) < 0)
				rte_exit(EXIT_FAILURE, "vdev creation failed:%s:%d\n",
						__func__, __LINE__);
			pt->rx_vdev_id = nb_ports++;
			/* configure vdev */
			configure_vdev(pt->rx_vdev_id);

		} else if (pt->dir == RTE_PDUMP_FLAG_TX) {

			/* create tx_ring */
			snprintf(ring_name, ARRAY_SIZE, TX_RING, i);
			pt->tx_ring = rte_ring_create(ring_name, pt->ring_size,
							rte_socket_id(), 0);
			if (pt->tx_ring == NULL)
				rte_exit(EXIT_FAILURE, "%s\n", rte_strerror(rte_errno));

			/* create vdevs */
			snprintf(vdev_name, ARRAY_SIZE, VDEV_NAME, TX_STR, i);
			if (pt->tx_vdev_stream_type == IFACE)
				snprintf(vdev_params, ARRAY_SIZE, VDEV_TX_IFACE,
						pt->tx_dev);
			else
				snprintf(vdev_params, ARRAY_SIZE, VDEV_TX_PCAP,
						pt->tx_dev);
			if (rte_eal_vdev_init(vdev_name, vdev_params) < 0)
				rte_exit(EXIT_FAILURE, "vdev creation failed\n");
			pt->tx_vdev_id = nb_ports++;
			/* configure vdev */
			configure_vdev(pt->tx_vdev_id);

		}
	}
}

static void
enable_pdump(void)
{
	int i;
	struct pdump_tuples *pt;
	int ret = 0, ret1 = 0;

	for (i = 0; i < num_tuples; i++) {
		pt = &pdump_t[i];
		if (pt->single_pdump_dev && pt->dir == RTE_PDUMP_FLAG_RXTX) {
			if (pt->dump_by_type == DEVICE_ID)
				ret = rte_pdump_enable_by_deviceid(pt->device_id,
						pt->queue, pt->dir,
						pt->rxtx_ring,
						pt->mp, NULL);
			else if (pt->dump_by_type == PORT_ID)
				ret = rte_pdump_enable(pt->port, pt->queue, pt->dir,
						pt->rxtx_ring, pt->mp, NULL);
		} else if (pt->dir == RTE_PDUMP_FLAG_RXTX) {
			if (pt->dump_by_type == DEVICE_ID) {
				ret = rte_pdump_enable_by_deviceid(pt->device_id,
								pt->queue,
								RTE_PDUMP_FLAG_RX,
								pt->rx_ring,
								pt->mp, NULL);
				ret = rte_pdump_enable_by_deviceid(pt->device_id,
								pt->queue,
								RTE_PDUMP_FLAG_TX,
								pt->tx_ring,
								pt->mp, NULL);
			} else if (pt->dump_by_type == PORT_ID) {
				ret = rte_pdump_enable(pt->port, pt->queue,
							RTE_PDUMP_FLAG_RX,
							pt->rx_ring, pt->mp, NULL);
				ret1 = rte_pdump_enable(pt->port, pt->queue,
							RTE_PDUMP_FLAG_TX,
							pt->tx_ring, pt->mp, NULL);
			}
		} else if (pt->dir == RTE_PDUMP_FLAG_RX) {
			if (pt->dump_by_type == DEVICE_ID)
				ret = rte_pdump_enable_by_deviceid(pt->device_id,
								pt->queue,
								pt->dir, pt->rx_ring,
								pt->mp, NULL);
			else if (pt->dump_by_type == PORT_ID)
				ret = rte_pdump_enable(pt->port, pt->queue, pt->dir,
						pt->rx_ring, pt->mp, NULL);
		} else if (pt->dir == RTE_PDUMP_FLAG_TX) {
			if (pt->dump_by_type == DEVICE_ID)
				ret = rte_pdump_enable_by_deviceid(pt->device_id,
								pt->queue,
								pt->dir,
						pt->tx_ring, pt->mp, NULL);
			else if (pt->dump_by_type == PORT_ID)
				ret = rte_pdump_enable(pt->port, pt->queue, pt->dir,
						pt->tx_ring, pt->mp, NULL);
		}
		if (ret < 0 || ret1 < 0) {
			cleanup_pdump_resources();
			rte_exit(EXIT_FAILURE, "%s\n", rte_strerror(rte_errno));
		}
	}
}

static inline void
pdump_rxtx(struct rte_ring *ring, uint8_t vdev_id, struct pdump_stats *stats)
{
	/* write input packets of port to vdev for pdump */
	struct rte_mbuf *rxtx_bufs[BURST_SIZE];

	/* first dequeue packets from ring of primary process */
	const uint16_t nb_in_deq = rte_ring_dequeue_burst(ring,
			(void *)rxtx_bufs, BURST_SIZE);
	stats->dequeue_pkts += nb_in_deq;

	if (nb_in_deq) {
		/* then sent on vdev */
		uint16_t nb_in_txd = rte_eth_tx_burst(
				vdev_id,
				0, rxtx_bufs, nb_in_deq);
		stats->tx_pkts += nb_in_txd;

		if (unlikely(nb_in_txd < nb_in_deq)) {
			do {
				rte_pktmbuf_free(rxtx_bufs[nb_in_txd]);
				stats->freed_pkts++;
			} while (++nb_in_txd < nb_in_deq);
		}
	}
}

static inline void
dump_packets(void)
{
	int i;
	struct pdump_tuples *pt;

	while (!quit_signal) {
		for (i = 0; i < num_tuples; i++) {
			pt = &pdump_t[i];
			if (pt->single_pdump_dev && pt->dir == RTE_PDUMP_FLAG_RXTX)
				pdump_rxtx(pt->rxtx_ring, pt->rxtx_vdev_id, &pt->stats);
			else if (pt->dir == RTE_PDUMP_FLAG_RXTX) {
				pdump_rxtx(pt->rx_ring, pt->rx_vdev_id, &pt->stats);
				pdump_rxtx(pt->tx_ring, pt->tx_vdev_id, &pt->stats);
			} else if (pt->dir == RTE_PDUMP_FLAG_RX)
				pdump_rxtx(pt->rx_ring, pt->rx_vdev_id, &pt->stats);
			else if (pt->dir == RTE_PDUMP_FLAG_TX)
				pdump_rxtx(pt->tx_ring, pt->tx_vdev_id, &pt->stats);
		}
	}
}

int
main(int argc, char **argv)
{
	int diag;
	int ret;
	int i;

	char c_flag[] = "-c1";
	char n_flag[] = "-n4";
	char mp_flag[] = "--proc-type=secondary";
	char *argp[argc + 3];

	/* catch ctrl-c so we can print on exit */
	signal(SIGINT, signal_handler);

	argp[0] = argv[0];
	argp[1] = c_flag;
	argp[2] = n_flag;
	argp[3] = mp_flag;

	for (i = 1; i < argc; i++)
		argp[i + 3] = argv[i];

	argc += 3;

	diag = rte_eal_init(argc, argp);
	if (diag < 0)
		rte_panic("Cannot init EAL\n");

	argc -= diag;
	argv += (diag - 3);

	/* parse app arguments */
	if (argc > 1) {
		ret = launch_args_parse(argc, argv);
		if (ret < 0)
			rte_exit(EXIT_FAILURE, "Invalid argument\n");
	}

	/* create mempool, ring and vdevs info */
	create_mp_ring_vdev();
	enable_pdump();
	dump_packets();

	return 0;
}
