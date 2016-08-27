/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2016 Intel Corporation. All rights reserved.
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

#include <arpa/inet.h>
#include <getopt.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/virtio_net.h>
#include <linux/virtio_ring.h>
#include <signal.h>
#include <stdint.h>
#include <sys/eventfd.h>
#include <sys/param.h>
#include <unistd.h>

#include <rte_atomic.h>
#include <rte_cycles.h>
#include <rte_ethdev.h>
#include <rte_log.h>
#include <rte_string_fns.h>
#include <rte_malloc.h>
#include <rte_virtio_net.h>
#include <rte_ip.h>
#include <rte_tcp.h>

#include "main.h"
#include "vswitch_common.h"

#ifndef MAX_QUEUES
#define MAX_QUEUES 128
#endif

/* the maximum number of external ports supported */
#define MAX_SUP_PORTS 1

#define MBUF_CACHE_SIZE	128
#define MBUF_DATA_SIZE	RTE_MBUF_DEFAULT_BUF_SIZE

#define BURST_TX_DRAIN_US 100	/* TX drain every ~100us */

#define BURST_RX_WAIT_US 15	/* Defines how long we wait between retries on RX */
#define BURST_RX_RETRIES 4		/* Number of retries on RX. */

#define JUMBO_FRAME_MAX_SIZE    0x2600

/* State of virtio device. */
#define DEVICE_MAC_LEARNING 0
#define DEVICE_RX			1
#define DEVICE_SAFE_REMOVE	2

/* Configurable number of RX/TX ring descriptors */
#define RTE_TEST_RX_DESC_DEFAULT 1024
#define RTE_TEST_TX_DESC_DEFAULT 512

#define INVALID_PORT_ID 0xFF

/* Max number of devices. Limited by vmdq. */
#define MAX_DEVICES 64

/* Size of buffers used for snprintfs. */
#define MAX_PRINT_BUFF 6072

/* Maximum character device basename size. */
#define MAX_BASENAME_SZ 10

/* Maximum long option length for option parsing. */
#define MAX_LONG_OPT_SZ 64

/* mask of enabled ports */
static uint32_t enabled_port_mask = 0;

/* Promiscuous mode */
static uint32_t promiscuous;

/* number of devices/queues to support*/
static uint32_t num_devices;

static struct rte_mempool *mbuf_pool;
static int mergeable;

/* Do vlan strip on host, enabled on default */
static uint32_t vlan_strip = 1;

static uint32_t jumbo_frame_en = 0;

/* Enable VM2VM communications. If this is disabled then the MAC address compare is skipped. */
typedef enum {
	VM2VM_DISABLED = 0,
	VM2VM_SOFTWARE = 1,
	VM2VM_HARDWARE = 2,
	VM2VM_LAST
} vm2vm_type;
static vm2vm_type vm2vm_mode = VM2VM_SOFTWARE;

/* Enable stats. */
static uint32_t enable_stats = 0;
/* Enable retries on RX. */
static uint32_t enable_retry = 1;

/* Disable TX checksum offload */
static uint32_t enable_tx_csum;

/* Disable TSO offload */
static uint32_t enable_tso;

static int client_mode;

/* Specify timeout (in useconds) between retries on RX. */
static uint32_t burst_rx_delay_time = BURST_RX_WAIT_US;
/* Specify the number of retries on RX. */
static uint32_t burst_rx_retry_num = BURST_RX_RETRIES;

/* Character device basename. Can be set by user. */
static char dev_basename[MAX_BASENAME_SZ] = "vhost-net";

/* vswitch device name and maximum number of ports */
static char switch_dev[MAX_BASENAME_SZ] = "vmdq";
static uint32_t switch_max_ports = MAX_DEVICES;

struct vswitch_dev *vswitch_dev_g;
static unsigned lcore_ids[RTE_MAX_LCORE];
static uint8_t ports[RTE_MAX_ETHPORTS];
static unsigned num_ports = 0; /**< The number of ports specified in command line */

static struct vhost_dev_tailq_list vhost_dev_list =
	TAILQ_HEAD_INITIALIZER(vhost_dev_list);

static struct lcore_info lcore_info[RTE_MAX_LCORE];


/* TX queue for each data core. */
struct mbuf_table lcore_tx_queue[RTE_MAX_LCORE];

#define MBUF_TABLE_DRAIN_TSC	((rte_get_tsc_hz() + US_PER_S - 1) \
				 / US_PER_S * BURST_TX_DRAIN_US)
#define VLAN_HLEN       4

/*
 * Validate the device number according to the max pool number gotten form
 * dev_info. If the device number is invalid, give the error message and
 * return -1. Each device must have its own pool.
 */
static inline int
validate_num_devices(uint32_t max_nb_devices)
{
	if (num_devices > max_nb_devices) {
		RTE_LOG(ERR, VHOST_PORT, "invalid number of devices\n");
		return -1;
	}
	return 0;
}

/*
 * Initialises a given port using global settings and with the rx buffers
 * coming from the mbuf_pool passed as parameter
 */
static inline int
port_init(uint8_t port)
{
	struct rte_eth_dev_info dev_info;
	struct rte_eth_rxconf *rxconf;
	struct rte_eth_txconf *txconf;
	int16_t rx_rings, tx_rings;
	uint16_t rx_ring_size, tx_ring_size;
	struct vswitch_port *vs_port;
	int retval;
	uint16_t q;

	if (port >= rte_eth_dev_count()) return -1;

	vs_port = vs_add_port(vswitch_dev_g, port, VSWITCH_PTYPE_PHYS, NULL);

	if (!vs_port) {
		rte_exit(EXIT_FAILURE, "Failed to add port [%d] to vsdev %s\n",
			 port, vswitch_dev_g->name);
	}

	/* The max pool number from dev_info will be used to validate the pool number specified in cmd line */
	rte_eth_dev_info_get (vs_port->port_id, &dev_info);

	if (dev_info.max_rx_queues > MAX_QUEUES) {
		rte_exit(EXIT_FAILURE,
			"please define MAX_QUEUES no less than %u in %s\n",
			dev_info.max_rx_queues, __FILE__);
	}

	rxconf = &dev_info.default_rxconf;
	txconf = &dev_info.default_txconf;
	rxconf->rx_drop_en = 1;

	/* Enable vlan offload */
	txconf->txq_flags &= ~ETH_TXQ_FLAGS_NOVLANOFFL;

	rx_ring_size = RTE_TEST_RX_DESC_DEFAULT;
	tx_ring_size = RTE_TEST_TX_DESC_DEFAULT;
	tx_rings = (uint16_t)rte_lcore_count();

	if (enable_tx_csum == 0)
		rte_vhost_feature_disable(1ULL << VIRTIO_NET_F_CSUM);

	if (enable_tso == 0) {
		rte_vhost_feature_disable(1ULL << VIRTIO_NET_F_HOST_TSO4);
		rte_vhost_feature_disable(1ULL << VIRTIO_NET_F_HOST_TSO6);
		rte_vhost_feature_disable(1ULL << VIRTIO_NET_F_GUEST_TSO4);
		rte_vhost_feature_disable(1ULL << VIRTIO_NET_F_GUEST_TSO6);
	}

	rx_rings = (uint16_t)dev_info.max_rx_queues;
	/* Configure ethernet device. */
	retval = rte_eth_dev_configure(vs_port->port_id, rx_rings, tx_rings,
				       &vs_port->port_conf);
	if (retval != 0) {
		RTE_LOG(ERR, VHOST_PORT, "Failed to configure port %u: %s.\n",
			vs_port->port_id, strerror(-retval));
		return retval;
	}

	/* Setup the queues. */
	for (q = 0; q < rx_rings; q ++) {
		retval = rte_eth_rx_queue_setup(port, q, rx_ring_size,
						rte_eth_dev_socket_id(port),
						rxconf,
						mbuf_pool);
		if (retval < 0) {
			RTE_LOG(ERR, VHOST_PORT,
				"Failed to setup rx queue %u of port %u: %s.\n",
				q, vs_port->port_id, strerror(-retval));
			return retval;
		}
	}
	for (q = 0; q < tx_rings; q ++) {
		retval = rte_eth_tx_queue_setup(port, q, tx_ring_size,
						rte_eth_dev_socket_id(port),
						txconf);
		if (retval < 0) {
			RTE_LOG(ERR, VHOST_PORT,
				"Failed to setup tx queue %u of port %u: %s.\n",
				q, port, strerror(-retval));
			return retval;
		}
	}

	if (promiscuous)
		rte_eth_promiscuous_enable(port);

	vs_port_start(vs_port);

	return 0;
}

/*
 * Set character device basename.
 */
static int
us_vhost_parse_basename(const char *q_arg)
{
	/* parse number string */

	if (strnlen(q_arg, MAX_BASENAME_SZ) > MAX_BASENAME_SZ)
		return -1;
	else
		snprintf((char*)&dev_basename, MAX_BASENAME_SZ, "%s", q_arg);

	return 0;
}

/*
 * Set switch device name.
 */
static int
us_vhost_parse_switch_name(const char *q_arg)
{
	/* parse number string */

	if (strnlen(q_arg, MAX_BASENAME_SZ) > MAX_BASENAME_SZ)
		return -1;
	else
		snprintf((char*)&switch_dev, MAX_BASENAME_SZ, "%s", q_arg);

	return 0;
}

/*
 * Parse the portmask provided at run time.
 */
static int
parse_portmask(const char *portmask)
{
	char *end = NULL;
	unsigned long pm;

	errno = 0;

	/* parse hexadecimal string */
	pm = strtoul(portmask, &end, 16);
	if ((portmask[0] == '\0') || (end == NULL) || (*end != '\0') || (errno != 0))
		return -1;

	if (pm == 0)
		return -1;

	return pm;

}

/*
 * Parse num options at run time.
 */
static int
parse_num_opt(const char *q_arg, uint32_t max_valid_value)
{
	char *end = NULL;
	unsigned long num;

	errno = 0;

	/* parse unsigned int string */
	num = strtoul(q_arg, &end, 10);
	if ((q_arg[0] == '\0') || (end == NULL) || (*end != '\0') || (errno != 0))
		return -1;

	if (num > max_valid_value)
		return -1;

	return num;

}

/*
 * Display usage
 */
static void
us_vhost_usage(const char *prgname)
{
	RTE_LOG(INFO, VHOST_CONFIG, "%s [EAL options] -- -p PORTMASK\n"
	"		--vm2vm [0|1|2]\n"
	"		--rx_retry [0|1] --mergeable [0|1] --stats [0-N]\n"
	"		--dev-basename <name>\n"
	"		--nb-devices ND\n"
	"		-p PORTMASK: Set mask for ports to be used by application\n"
	"		--vm2vm [0|1|2]: disable/software(default)/hardware vm2vm comms\n"
	"		--rx-retry [0|1]: disable/enable(default) retries on rx. Enable retry if destintation queue is full\n"
	"		--rx-retry-delay [0-N]: timeout(in usecond) between retries on RX. This makes effect only if retries on rx enabled\n"
	"		--rx-retry-num [0-N]: the number of retries on rx. This makes effect only if retries on rx enabled\n"
	"		--mergeable [0|1]: disable(default)/enable RX mergeable buffers\n"
	"		--vlan-strip [0|1]: disable/enable(default) RX VLAN strip on host\n"
	"		--stats [0-N]: 0: Disable stats, N: Time in seconds to print stats\n"
	"		--dev-basename: The basename to be used for the character device.\n"
	"		--tx-csum [0|1] disable/enable TX checksum offload.\n"
	"		--tso [0|1] disable/enable TCP segment offload.\n"
	"		--client register a vhost-user socket as client mode.\n",
	       prgname);
}

/*
 * Parse the arguments given in the command line of the application.
 */
static int
us_vhost_parse_args(int argc, char **argv)
{
	int opt, ret;
	int option_index;
	unsigned i;
	const char *prgname = argv[0];
	static struct option long_option[] = {
		{"vm2vm", required_argument, NULL, 0},
		{"rx-retry", required_argument, NULL, 0},
		{"rx-retry-delay", required_argument, NULL, 0},
		{"rx-retry-num", required_argument, NULL, 0},
		{"mergeable", required_argument, NULL, 0},
		{"vlan-strip", required_argument, NULL, 0},
		{"stats", required_argument, NULL, 0},
		{"dev-basename", required_argument, NULL, 0},
		{"tx-csum", required_argument, NULL, 0},
		{"tso", required_argument, NULL, 0},
		{"client", no_argument, &client_mode, 1},
		{"switch", required_argument, NULL, 0},
		{"max-ports", required_argument, NULL, 0},
		{NULL, 0, 0, 0},
	};

	/* Parse command line */
	while ((opt = getopt_long(argc, argv, "p:P",
			long_option, &option_index)) != EOF) {
		switch (opt) {
		/* Portmask */
		case 'p':
			enabled_port_mask = parse_portmask(optarg);
			if (enabled_port_mask == 0) {
				RTE_LOG(INFO, VHOST_CONFIG, "Invalid portmask\n");
				us_vhost_usage(prgname);
				return -1;
			}
			break;

		case 'P':
			promiscuous = 1;
			rte_vhost_feature_enable(1ULL << VIRTIO_NET_F_CTRL_RX);

			break;

		case 0:
			/* Enable/disable vm2vm comms. */
			if (!strncmp(long_option[option_index].name, "vm2vm",
				MAX_LONG_OPT_SZ)) {
				ret = parse_num_opt(optarg, (VM2VM_LAST - 1));
				if (ret == -1) {
					RTE_LOG(INFO, VHOST_CONFIG,
						"Invalid argument for "
						"vm2vm [0|1|2]\n");
					us_vhost_usage(prgname);
					return -1;
				} else {
					vm2vm_mode = (vm2vm_type)ret;
				}
			}

			/* Enable/disable retries on RX. */
			if (!strncmp(long_option[option_index].name, "rx-retry", MAX_LONG_OPT_SZ)) {
				ret = parse_num_opt(optarg, 1);
				if (ret == -1) {
					RTE_LOG(INFO, VHOST_CONFIG, "Invalid argument for rx-retry [0|1]\n");
					us_vhost_usage(prgname);
					return -1;
				} else {
					enable_retry = ret;
				}
			}

			/* Enable/disable TX checksum offload. */
			if (!strncmp(long_option[option_index].name, "tx-csum", MAX_LONG_OPT_SZ)) {
				ret = parse_num_opt(optarg, 1);
				if (ret == -1) {
					RTE_LOG(INFO, VHOST_CONFIG, "Invalid argument for tx-csum [0|1]\n");
					us_vhost_usage(prgname);
					return -1;
				} else
					enable_tx_csum = ret;
			}

			/* Enable/disable TSO offload. */
			if (!strncmp(long_option[option_index].name, "tso", MAX_LONG_OPT_SZ)) {
				ret = parse_num_opt(optarg, 1);
				if (ret == -1) {
					RTE_LOG(INFO, VHOST_CONFIG, "Invalid argument for tso [0|1]\n");
					us_vhost_usage(prgname);
					return -1;
				} else
					enable_tso = ret;
			}

			/* Specify the retries delay time (in useconds) on RX. */
			if (!strncmp(long_option[option_index].name, "rx-retry-delay", MAX_LONG_OPT_SZ)) {
				ret = parse_num_opt(optarg, INT32_MAX);
				if (ret == -1) {
					RTE_LOG(INFO, VHOST_CONFIG, "Invalid argument for rx-retry-delay [0-N]\n");
					us_vhost_usage(prgname);
					return -1;
				} else {
					burst_rx_delay_time = ret;
				}
			}

			/* Specify the retries number on RX. */
			if (!strncmp(long_option[option_index].name, "rx-retry-num", MAX_LONG_OPT_SZ)) {
				ret = parse_num_opt(optarg, INT32_MAX);
				if (ret == -1) {
					RTE_LOG(INFO, VHOST_CONFIG, "Invalid argument for rx-retry-num [0-N]\n");
					us_vhost_usage(prgname);
					return -1;
				} else {
					burst_rx_retry_num = ret;
				}
			}

			/* Enable/disable RX mergeable buffers. */
			if (!strncmp(long_option[option_index].name, "mergeable", MAX_LONG_OPT_SZ)) {
				ret = parse_num_opt(optarg, 1);
				if (ret == -1) {
					RTE_LOG(INFO, VHOST_CONFIG, "Invalid argument for mergeable [0|1]\n");
					us_vhost_usage(prgname);
					return -1;
				} else {
					mergeable = !!ret;
					if (ret)
						jumbo_frame_en = 1;
				}
			}

			/* Enable/disable RX VLAN strip on host. */
			if (!strncmp(long_option[option_index].name,
				"vlan-strip", MAX_LONG_OPT_SZ)) {
				ret = parse_num_opt(optarg, 1);
				if (ret == -1) {
					RTE_LOG(INFO, VHOST_CONFIG,
						"Invalid argument for VLAN strip [0|1]\n");
					us_vhost_usage(prgname);
					return -1;
				} else {
					vlan_strip = !!ret;
				}
			}

			/* Enable/disable stats. */
			if (!strncmp(long_option[option_index].name, "stats", MAX_LONG_OPT_SZ)) {
				ret = parse_num_opt(optarg, INT32_MAX);
				if (ret == -1) {
					RTE_LOG(INFO, VHOST_CONFIG, "Invalid argument for stats [0..N]\n");
					us_vhost_usage(prgname);
					return -1;
				} else {
					enable_stats = ret;
				}
			}

			/* Set character device basename. */
			if (!strncmp(long_option[option_index].name, "dev-basename", MAX_LONG_OPT_SZ)) {
				if (us_vhost_parse_basename(optarg) == -1) {
					RTE_LOG(INFO, VHOST_CONFIG, "Invalid argument for character device basename (Max %d characters)\n", MAX_BASENAME_SZ);
					us_vhost_usage(prgname);
					return -1;
				}
			}

			/* Set vswitch_driver name */
			if (!strncmp(long_option[option_index].name, "switch", MAX_LONG_OPT_SZ)) {
				if (us_vhost_parse_switch_name(optarg) == -1) {
					RTE_LOG(INFO, VHOST_CONFIG, "Invalid argument for character switch dev (Max %d characters)\n", MAX_BASENAME_SZ);
					us_vhost_usage(prgname);
					return -1;
				}
			}

			/* Specify Max ports in vswitch. */
			if (!strncmp(long_option[option_index].name, "max-ports", MAX_LONG_OPT_SZ)) {
				ret = parse_num_opt(optarg, INT32_MAX);
				if (ret == -1) {
					RTE_LOG(INFO, VHOST_CONFIG, "Invalid argument for switch max ports [0-N]\n");
					us_vhost_usage(prgname);
					return -1;
				} else {
					switch_max_ports = ret;
				}
			}

			break;

			/* Invalid option - print options. */
		default:
			us_vhost_usage(prgname);
			return -1;
		}
	}

	for (i = 0; i < RTE_MAX_ETHPORTS; i++) {
		if (enabled_port_mask & (1 << i))
			ports[num_ports++] = (uint8_t)i;
	}

	if ((num_ports ==  0) || (num_ports > MAX_SUP_PORTS)) {
		RTE_LOG(INFO, VHOST_PORT, "Current enabled port number is %u,"
			"but only %u port can be enabled\n",num_ports, MAX_SUP_PORTS);
		return -1;
	}

	return 0;
}

/*
 * Update the global var NUM_PORTS and array PORTS according to system ports number
 * and return valid ports number
 */
static unsigned check_ports_num(unsigned nb_ports)
{
	unsigned valid_num_ports = num_ports;
	unsigned portid;

	if (num_ports > nb_ports) {
		RTE_LOG(INFO, VHOST_PORT, "\nSpecified port number(%u) exceeds total system port number(%u)\n",
			num_ports, nb_ports);
		num_ports = nb_ports;
	}

	for (portid = 0; portid < num_ports; portid ++) {
		if (ports[portid] >= nb_ports) {
			RTE_LOG(INFO, VHOST_PORT, "\nSpecified port ID(%u) exceeds max system port ID(%u)\n",
				ports[portid], (nb_ports - 1));
			ports[portid] = INVALID_PORT_ID;
			valid_num_ports--;
		}
	}
	return valid_num_ports;
}

struct vhost_dev *find_vhost_dev(struct ether_addr *mac)
{
	struct vhost_dev *vdev;

	TAILQ_FOREACH(vdev, &vhost_dev_list, global_vdev_entry) {
		if (vdev->ready == DEVICE_RX &&
		    is_same_ether_addr(mac, &vdev->mac_address))
			return vdev;
	}

	return NULL;
}

static inline void __attribute__((always_inline))
virtio_xmit(struct vhost_dev *dst_vdev, struct vhost_dev *src_vdev,
	    struct rte_mbuf *m)
{
	uint16_t ret;

	ret = rte_vhost_enqueue_burst(dst_vdev->vid, VIRTIO_RXQ, &m, 1);
	if (enable_stats) {
		rte_atomic64_inc(&dst_vdev->stats.rx_total_atomic);
		rte_atomic64_add(&dst_vdev->stats.rx_atomic, ret);
		src_vdev->stats.tx_total++;
		src_vdev->stats.tx += ret;
	}
}

/*
 * Check if the packet destination MAC address is for a local device. If so then put
 * the packet on that devices RX queue. If not then return.
 */
int virtio_tx_local(struct vhost_dev *vdev, struct rte_mbuf *m)
{
	struct ether_hdr *pkt_hdr;
	struct vhost_dev *dst_vdev;

	pkt_hdr = rte_pktmbuf_mtod(m, struct ether_hdr *);

	dst_vdev = find_vhost_dev(&pkt_hdr->d_addr);
	if (!dst_vdev)
		return -1;

	if (vdev->vid == dst_vdev->vid) {
		RTE_LOG(DEBUG, VHOST_DATA,
			"(%d) TX: src and dst MAC is same. Dropping packet.\n",
			vdev->vid);
		return 0;
	}

	RTE_LOG(DEBUG, VHOST_DATA,
		"(%d) TX: MAC address is local\n", dst_vdev->vid);

	if (unlikely(dst_vdev->remove)) {
		RTE_LOG(DEBUG, VHOST_DATA,
			"(%d) device is marked for removal\n", dst_vdev->vid);
		return 0;
	}

	virtio_xmit(dst_vdev, vdev, m);
	return 0;
}

struct mbuf_table *vhost_switch_get_txq(uint16_t core_id)
{
	return &lcore_tx_queue[core_id];
}

static inline void
free_pkts(struct rte_mbuf **pkts, uint16_t n)
{
	while (n--)
		rte_pktmbuf_free(pkts[n]);
}

void do_drain_mbuf_table(struct mbuf_table *tx_q)
{
	uint16_t count;
	struct vswitch_port *tx_port;

	/* Let switch implmentation decide which physical port to do tx to.
	 * Every switch implmentation may have it's own strategy, for example
	 * VMDQ does tx to only one Physical port. Having a scheduler function
	 * which is switch specefic give flexibility to have another strategy
	 * for a switch
	 */
	tx_port = vs_sched_tx_port(vswitch_dev_g, VSWITCH_PTYPE_PHYS,
				     rte_lcore_id());
	if (unlikely(!tx_port))
	    goto out;

	count = tx_port->do_tx(tx_port, rte_lcore_id(), NULL, tx_q->m_table,
				  tx_q->len);

	if (unlikely(count < tx_q->len))
		free_pkts(&tx_q->m_table[count], tx_q->len - count);

	tx_q->len = 0;
out:
	return;
}

static inline void __attribute__((always_inline))
drain_mbuf_table(struct mbuf_table *tx_q)
{
	static uint64_t prev_tsc;
	uint64_t cur_tsc;

	if (tx_q->len == 0)
		return;

	cur_tsc = rte_rdtsc();
	if (unlikely(cur_tsc - prev_tsc > MBUF_TABLE_DRAIN_TSC)) {
		prev_tsc = cur_tsc;

		RTE_LOG(DEBUG, VHOST_DATA,
			"TX queue drained after timeout with burst size %u\n",
			tx_q->len);
		do_drain_mbuf_table(tx_q);
	}
}

static inline void __attribute__((always_inline))
drain_eth_rx(struct vhost_dev *vdev)
{
	uint16_t rx_count;
	struct rte_mbuf *pkts[MAX_PKT_BURST];
	uint16_t rxq = vdev->vs_port->phys_port_rxq;
	struct vswitch_port *rx_port;

	/* Let switch implmentation decide which physical port to do rx from.
	 * Every switch implmentation may have it's own strategy, for example
	 * VMDQ does rx from only one Physical port. Having a scheduler function
	 * which is switch specefic give flexibility to have another strategy
	 * for a switch
	 */
	rx_port = vs_sched_rx_port(vswitch_dev_g, VSWITCH_PTYPE_PHYS,
				     rte_lcore_id());
	if (unlikely(!rx_port))
	    goto out;

	rx_count = rx_port->do_rx(rx_port, rxq, NULL, pkts, MAX_PKT_BURST);

	if (!rx_count)
		return;

	vs_lookup_n_fwd(rx_port, pkts, rx_count, rxq);

	if (enable_stats) {
		rte_atomic64_add(&vdev->stats.rx_total_atomic, rx_count);
	}

	free_pkts(pkts, rx_count);
out:
	return;
}

static inline void __attribute__((always_inline))
drain_virtio_tx(struct vhost_dev *vdev)
{
	struct rte_mbuf *pkts[MAX_PKT_BURST];
	struct vswitch_port *vs_port = vdev->vs_port;
	uint16_t count;

	count = rte_vhost_dequeue_burst(vdev->vid, VIRTIO_RXQ, mbuf_pool,
					pkts, MAX_PKT_BURST);

	if (unlikely(vdev->ready == DEVICE_MAC_LEARNING)) {
		if (vdev->remove || vs_learn_port(vs_port, pkts, count))
			free_pkts(pkts, count);
	}

	vs_lookup_n_fwd(vs_port, pkts, count, VIRTIO_RXQ);
}

/*
 * Main function of vhost-switch. It basically does:
 *
 * for each vhost device {
 *    - drain_eth_rx()
 *
 *      Which drains the host eth Rx queue linked to the vhost device,
 *      and deliver all of them to guest virito Rx ring associated with
 *      this vhost device.
 *
 *    - drain_virtio_tx()
 *
 *      Which drains the guest virtio Tx queue and deliver all of them
 *      to the target, which could be another vhost device, or the
 *      physical eth dev. The route is done in function "virtio_tx_route".
 * }
 */
static int
switch_worker(void *arg __rte_unused)
{
	unsigned i;
	unsigned lcore_id = rte_lcore_id();
	struct vhost_dev *vdev;
	struct mbuf_table *tx_q;

	RTE_LOG(INFO, VHOST_DATA, "Procesing on Core %u started\n", lcore_id);

	tx_q = &lcore_tx_queue[lcore_id];
	for (i = 0; i < rte_lcore_count(); i++) {
		if (lcore_ids[i] == lcore_id) {
			tx_q->txq_id = i;
			break;
		}
	}

	while(1) {
		drain_mbuf_table(tx_q);

		/*
		 * Inform the configuration core that we have exited the
		 * linked list and that no devices are in use if requested.
		 */
		if (lcore_info[lcore_id].dev_removal_flag == REQUEST_DEV_REMOVAL)
			lcore_info[lcore_id].dev_removal_flag = ACK_DEV_REMOVAL;

		/*
		 * Process vhost devices
		 */
		TAILQ_FOREACH(vdev, &lcore_info[lcore_id].vdev_list,
			      lcore_vdev_entry) {
			if (unlikely(vdev->remove)) {
				vs_unlearn_port(vdev->vs_port);
				vdev->ready = DEVICE_SAFE_REMOVE;
				continue;
			}

			if (likely(vdev->ready == DEVICE_RX))
				drain_eth_rx(vdev);

			if (likely(!vdev->remove))
				drain_virtio_tx(vdev);
		}
	}

	return 0;
}

/*
 * Remove a device from the specific data core linked list and from the
 * main linked list. Synchonization  occurs through the use of the
 * lcore dev_removal_flag. Device is made volatile here to avoid re-ordering
 * of dev->remove=1 which can cause an infinite loop in the rte_pause loop.
 */
static void
destroy_device(int vid)
{
	struct vhost_dev *vdev = NULL;
	struct vswitch_port *vs_port;
	int lcore;

	TAILQ_FOREACH(vdev, &vhost_dev_list, global_vdev_entry) {
		if (vdev->vid == vid)
			break;
	}
	if (!vdev)
		return;
	/*set the remove flag. */
	vdev->remove = 1;
	while(vdev->ready != DEVICE_SAFE_REMOVE) {
		rte_pause();
	}

	TAILQ_REMOVE(&lcore_info[vdev->coreid].vdev_list, vdev,
		     lcore_vdev_entry);
	TAILQ_REMOVE(&vhost_dev_list, vdev, global_vdev_entry);


	/* Set the dev_removal_flag on each lcore. */
	RTE_LCORE_FOREACH_SLAVE(lcore)
		lcore_info[lcore].dev_removal_flag = REQUEST_DEV_REMOVAL;

	/*
	 * Once each core has set the dev_removal_flag to ACK_DEV_REMOVAL
	 * we can be sure that they can no longer access the device removed
	 * from the linked lists and that the devices are no longer in use.
	 */
	RTE_LCORE_FOREACH_SLAVE(lcore) {
		while (lcore_info[lcore].dev_removal_flag != ACK_DEV_REMOVAL)
			rte_pause();
	}

	lcore_info[vdev->coreid].device_num--;

	vs_port = vdev->vs_port;
	vs_port_stop(vs_port);
	vs_del_port(vs_port);

	RTE_LOG(INFO, VHOST_DATA,
		"(%d) device has been removed from data core\n",
		vdev->vid);

	rte_free(vdev);
}

/*
 * A new device is added to a data core. First the device is added to the main linked list
 * and the allocated to a specific data core.
 */
static int
new_device(int vid)
{
	int lcore, core_add = 0;
	uint32_t device_num_min = num_devices;
	struct vhost_dev *vdev;
	struct vswitch_port *vs_port;

	vdev = rte_zmalloc("vhost device", sizeof(*vdev), RTE_CACHE_LINE_SIZE);
	if (vdev == NULL) {
		RTE_LOG(INFO, VHOST_DATA,
			"(%d) couldn't allocate memory for vhost dev\n",
			vid);
		return -1;
	}
	vdev->vid = vid;

	TAILQ_INSERT_TAIL(&vhost_dev_list, vdev, global_vdev_entry);

	/*reset ready flag*/
	vdev->ready = DEVICE_MAC_LEARNING;
	vdev->remove = 0;

	/* Find a suitable lcore to add the device. */
	RTE_LCORE_FOREACH_SLAVE(lcore) {
		if (lcore_info[lcore].device_num < device_num_min) {
			device_num_min = lcore_info[lcore].device_num;
			core_add = lcore;
		}
	}
	vdev->coreid = core_add;

	TAILQ_INSERT_TAIL(&lcore_info[vdev->coreid].vdev_list, vdev,
			  lcore_vdev_entry);
	lcore_info[vdev->coreid].device_num++;

	/* Disable notifications. */
	rte_vhost_enable_guest_notification(vid, VIRTIO_RXQ, 0);
	rte_vhost_enable_guest_notification(vid, VIRTIO_TXQ, 0);

	vs_port = vs_add_port(vswitch_dev_g, vid, VSWITCH_PTYPE_VIRTIO, vdev);

	if (!vs_port) {
		rte_exit(EXIT_FAILURE, "Failed to add port [%d] to vsdev %s\n",
			 vs_port->port_id, vswitch_dev_g->name);
	}

	vdev->vs_port = vs_port;
	vs_port_start(vs_port);

	RTE_LOG(INFO, VHOST_DATA,
		"(%d) device has been added to data core %d\n",
		vid, vdev->coreid);

	return 0;
}

/*
 * These callback allow devices to be added to the data core when configuration
 * has been fully complete.
 */
static const struct virtio_net_device_ops virtio_net_device_ops =
{
	.new_device =  new_device,
	.destroy_device = destroy_device,
};

/*
 * This is a thread will wake up after a period to print stats if the user has
 * enabled them.
 */
static void
print_stats(void)
{
	struct vhost_dev *vdev;
	uint64_t tx_dropped, rx_dropped;
	uint64_t tx, tx_total, rx, rx_total;
	const char clr[] = { 27, '[', '2', 'J', '\0' };
	const char top_left[] = { 27, '[', '1', ';', '1', 'H','\0' };

	while(1) {
		sleep(enable_stats);

		/* Clear screen and move to top left */
		printf("%s%s\n", clr, top_left);
		printf("Device statistics =================================\n");

		TAILQ_FOREACH(vdev, &vhost_dev_list, global_vdev_entry) {
			tx_total   = vdev->stats.tx_total;
			tx         = vdev->stats.tx;
			tx_dropped = tx_total - tx;

			rx_total   = rte_atomic64_read(&vdev->stats.rx_total_atomic);
			rx         = rte_atomic64_read(&vdev->stats.rx_atomic);
			rx_dropped = rx_total - rx;

			printf("Statistics for device %d\n"
				"-----------------------\n"
				"TX total:              %" PRIu64 "\n"
				"TX dropped:            %" PRIu64 "\n"
				"TX successful:         %" PRIu64 "\n"
				"RX total:              %" PRIu64 "\n"
				"RX dropped:            %" PRIu64 "\n"
				"RX successful:         %" PRIu64 "\n",
				vdev->vid,
				tx_total, tx_dropped, tx,
				rx_total, rx_dropped, rx);
		}

		printf("===================================================\n");
	}
}

/* When we receive a INT signal, unregister vhost driver */
static void
sigint_handler(__rte_unused int signum)
{
	/* Unregister vhost driver. */
	int ret = rte_vhost_driver_unregister((char *)&dev_basename);
	if (ret != 0)
		rte_exit(EXIT_FAILURE, "vhost driver unregister failure.\n");
	exit(0);
}

/*
 * While creating an mbuf pool, one key thing is to figure out how
 * many mbuf entries is enough for our use. FYI, here are some
 * guidelines:
 *
 * - Each rx queue would reserve @nr_rx_desc mbufs at queue setup stage
 *
 * - For each switch core (A CPU core does the packet switch), we need
 *   also make some reservation for receiving the packets from virtio
 *   Tx queue. How many is enough depends on the usage. It's normally
 *   a simple calculation like following:
 *
 *       MAX_PKT_BURST * max packet size / mbuf size
 *
 *   So, we definitely need allocate more mbufs when TSO is enabled.
 *
 * - Similarly, for each switching core, we should serve @nr_rx_desc
 *   mbufs for receiving the packets from physical NIC device.
 *
 * - We also need make sure, for each switch core, we have allocated
 *   enough mbufs to fill up the mbuf cache.
 */
static void
create_mbuf_pool(uint16_t nr_port, uint32_t nr_switch_core, uint32_t mbuf_size,
	uint32_t nr_queues, uint32_t nr_rx_desc, uint32_t nr_mbuf_cache)
{
	uint32_t nr_mbufs;
	uint32_t nr_mbufs_per_core;
	uint32_t mtu = 1500;

	if (mergeable)
		mtu = 9000;
	if (enable_tso)
		mtu = 64 * 1024;

	nr_mbufs_per_core  = (mtu + mbuf_size) * MAX_PKT_BURST /
			(mbuf_size - RTE_PKTMBUF_HEADROOM) * MAX_PKT_BURST;
	nr_mbufs_per_core += nr_rx_desc;
	nr_mbufs_per_core  = RTE_MAX(nr_mbufs_per_core, nr_mbuf_cache);

	nr_mbufs  = nr_queues * nr_rx_desc;
	nr_mbufs += nr_mbufs_per_core * nr_switch_core;
	nr_mbufs *= nr_port;

	mbuf_pool = rte_pktmbuf_pool_create("MBUF_POOL", nr_mbufs,
					    nr_mbuf_cache, 0, mbuf_size,
					    rte_socket_id());
	if (mbuf_pool == NULL)
		rte_exit(EXIT_FAILURE, "Cannot create mbuf pool\n");
}

static uint64_t get_vswitch_conf_flags(void)
{
	uint64_t vs_conf_flags = 0;

	if (vm2vm_mode == VM2VM_HARDWARE)
		vs_conf_flags |= VS_CNF_FLG_VM2VM_HARDWARE;

	if (vm2vm_mode == VM2VM_SOFTWARE)
		vs_conf_flags |= VS_CNF_FLG_VM2VM_SOFTWARE;

	if (promiscuous)
		vs_conf_flags |= VS_CNF_FLG_PROMISCOUS_EN;

	if (jumbo_frame_en)
		vs_conf_flags |= VS_CNF_FLG_JUMBO_EN;

	if (enable_stats)
		vs_conf_flags |= VS_CNF_FLG_STATS_EN;

	return vs_conf_flags;
}

/*
 * Main function, does initialisation and calls the per-lcore functions. The CUSE
 * device is also registered here to handle the IOCTLs.
 */
int
main(int argc, char *argv[])
{
	unsigned lcore_id, core_id = 0;
	unsigned nb_ports, valid_num_ports;
	int ret;
	uint8_t portid;
	static pthread_t tid;
	char thread_name[RTE_MAX_THREAD_NAME_LEN];
	uint64_t flags = 0, vswitch_conf_flags;

	signal(SIGINT, sigint_handler);

	/* init EAL */
	ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");
	argc -= ret;
	argv += ret;

	vs_vswitch_init();

	/* TBD:XXX: This needs to be removed here, when constructor mechanism
	 * for registering swittches is in place
	 */
	vmdq_switch_impl_init();

	/* parse app arguments */
	ret = us_vhost_parse_args(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Invalid argument\n");

	/*TBD:XXX: vdev list or the vdev ports */
	for (lcore_id = 0; lcore_id < RTE_MAX_LCORE; lcore_id ++)
		TAILQ_INIT(&lcore_info[lcore_id].vdev_list);

		if (rte_lcore_is_enabled(lcore_id))
			lcore_ids[core_id ++] = lcore_id;

	if (rte_lcore_count() > RTE_MAX_LCORE)
		rte_exit(EXIT_FAILURE,"Not enough cores\n");

	/* Get the number of physical ports. */
	nb_ports = rte_eth_dev_count();

	/*
	 * Update the global var NUM_PORTS and global array PORTS
	 * and get value of var VALID_NUM_PORTS according to system ports number
	 */
	valid_num_ports = check_ports_num(nb_ports);

	if ((valid_num_ports ==  0) || (valid_num_ports > MAX_SUP_PORTS)) {
		RTE_LOG(INFO, VHOST_PORT, "Current enabled port number is %u,"
			"but only %u port can be enabled\n",num_ports, MAX_SUP_PORTS);
		return -1;
	}

	vswitch_dev_g = vs_get_vswitch_dev(switch_dev);
	if (!vswitch_dev_g) {
		RTE_LOG(INFO, VHOST_CONFIG, "switch dev %s not supported\n",
			switch_dev);
		return -1;
	}

	/*
	 * FIXME: here we are trying to allocate mbufs big enough for
	 * @MAX_QUEUES, but the truth is we're never going to use that
	 * many queues here. We probably should only do allocation for
	 * those queues we are going to use.
	 */
	create_mbuf_pool(valid_num_ports, rte_lcore_count() - 1, MBUF_DATA_SIZE,
			 MAX_QUEUES, RTE_TEST_RX_DESC_DEFAULT, MBUF_CACHE_SIZE);

	vswitch_conf_flags = get_vswitch_conf_flags();
	vs_switch_dev_init(vswitch_dev_g, vswitch_conf_flags);

	/* initialize all ports */
	for (portid = 0; portid < nb_ports; portid++) {
		/* skip ports that are not enabled */
		if ((enabled_port_mask & (1 << portid)) == 0) {
			RTE_LOG(INFO, VHOST_PORT,
				"Skipping disabled port %d\n", portid);
			continue;
		}
		if (port_init(portid) != 0)
			rte_exit(EXIT_FAILURE,
				"Cannot initialize network ports\n");
	}

	/* Enable stats if the user option is set. */
	if (enable_stats) {
		ret = pthread_create(&tid, NULL, (void *)print_stats, NULL);
		if (ret != 0)
			rte_exit(EXIT_FAILURE,
				"Cannot create print-stats thread\n");

		/* Set thread_name for aid in debugging.  */
		snprintf(thread_name, RTE_MAX_THREAD_NAME_LEN, "print-stats");
		ret = rte_thread_setname(tid, thread_name);
		if (ret != 0)
			RTE_LOG(DEBUG, VHOST_CONFIG,
				"Cannot set print-stats name\n");
	}

	/* Launch all data cores. */
	RTE_LCORE_FOREACH_SLAVE(lcore_id)
		rte_eal_remote_launch(switch_worker, NULL, lcore_id);

	if (mergeable == 0)
		rte_vhost_feature_disable(1ULL << VIRTIO_NET_F_MRG_RXBUF);

	if (client_mode)
		flags |= RTE_VHOST_USER_CLIENT;

	/* Register vhost(cuse or user) driver to handle vhost messages. */
	ret = rte_vhost_driver_register(dev_basename, flags);
	if (ret != 0)
		rte_exit(EXIT_FAILURE, "vhost driver register failure.\n");

	rte_vhost_driver_callback_register(&virtio_net_device_ops);

	/* Start CUSE session. */
	rte_vhost_driver_session_start();
	return 0;

}
