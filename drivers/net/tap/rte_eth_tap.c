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

#include <rte_mbuf.h>
#include <rte_ethdev.h>
#include <rte_malloc.h>
#include <rte_kvargs.h>
#include <rte_dev.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <poll.h>
#include <arpa/inet.h>
#include <net/if.h>
#ifdef __linux__
#include <linux/if.h>
#include <linux/if_tun.h>
#include <linux/if_ether.h>
#else
#include <netinet/if_ether.h>
#endif
#include <fcntl.h>

#include <poll.h>

/* Linux based path to the TUN device */
#define TUN_TAP_DEV_PATH        "/dev/net/tun"

#define ETH_TAP_IFACE_ARG       "iface"
#define ETH_TAP_SPEED_ARG       "speed"

static const char *valid_arguments[] = {
	ETH_TAP_IFACE_ARG,
	ETH_TAP_SPEED_ARG,
	NULL
};

static const char *drivername = "Tap PMD";
static int tap_unit = 0;

static struct rte_eth_link pmd_link = {
	.link_speed = ETH_SPEED_NUM_10G,
	.link_duplex = ETH_LINK_FULL_DUPLEX,
	.link_status = ETH_LINK_DOWN,
	.link_autoneg = ETH_LINK_SPEED_AUTONEG
};

struct tap_info {
	char name[RTE_ETH_NAME_MAX_LEN]; /* Interface name supplied/given */
	int speed;			 /* Speed of interface */
};

struct pkt_stats {
	uint64_t opackets;		/* Number of output packets */
	uint64_t ipackets;		/* Number of input packets */
	uint64_t obytes;		/* Number of bytes on output */
	uint64_t ibytes;		/* Number of bytes on input */
	uint64_t errs;			/* Number of error packets */
};

struct rx_queue {
	struct rte_mempool *mp;		/* Mempool for RX packets */
	uint16_t in_port;		/* Port ID */
	int fd;

	struct pkt_stats stats;		/* Stats for this RX queue */
};

struct tx_queue {
	int fd;
	struct pkt_stats stats;		/* Stats for this TX queue */
};

struct pmd_internals {
	char name[RTE_ETH_NAME_MAX_LEN];	/* Internal Tap device name */
	uint16_t nb_queues;			/* Number of queues supported */
	uint16_t pad0;
	struct ether_addr eth_addr;	/* Mac address of the device port */

	int if_index;			/* IF_INDEX for the port */
	int fds[RTE_PMD_TAP_MAX_QUEUES]; /* List of all file descriptors */

	struct rx_queue rxq[RTE_PMD_TAP_MAX_QUEUES];	/* List of RX queues */
	struct tx_queue txq[RTE_PMD_TAP_MAX_QUEUES];	/* List of TX queues */
};

/*
 * Tun/Tap allocation routine
 *
 * name is the number of the interface to use, unless NULL to take the host
 * supplied name.
 */
static int
tun_alloc(char * name)
{
	struct ifreq ifr;
	unsigned int features;
	int fd;

	memset(&ifr, 0, sizeof(struct ifreq));

	ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
	if (name && name[0])
		strncpy(ifr.ifr_name, name, IFNAMSIZ);

	fd = open(TUN_TAP_DEV_PATH, O_RDWR);
	if (fd < 0) {
		RTE_LOG(ERR, PMD, "Unable to create TAP interface");
		goto error;
	}

	/* Grab the TUN features to verify we can work */
	if (ioctl(fd, TUNGETFEATURES, &features) < 0) {
		RTE_LOG(ERR, PMD, "Unable to get TUN/TAP features\n");
		goto error;
	}
	RTE_LOG(DEBUG, PMD, "TUN/TAP Features %08x\n", features);

	if (!(features & IFF_MULTI_QUEUE) && (RTE_PMD_TAP_MAX_QUEUES > 1)) {
		RTE_LOG(DEBUG, PMD, "TUN/TAP device only one queue\n");
		goto error;
	} else if ((features & IFF_ONE_QUEUE) && (RTE_PMD_TAP_MAX_QUEUES == 1)) {
		ifr.ifr_flags |= IFF_ONE_QUEUE;
		RTE_LOG(DEBUG, PMD, "Single queue only support\n");
	} else {
		ifr.ifr_flags |= IFF_MULTI_QUEUE;
		RTE_LOG(DEBUG, PMD, "Multi-queue support for %d queues\n",
			RTE_PMD_TAP_MAX_QUEUES);
	}

	/* Set the TUN/TAP configuration and get the name if needed */
	if (ioctl(fd, TUNSETIFF, (void *)&ifr) < 0) {
		RTE_LOG(ERR, PMD, "Unable to set TUNSETIFF for %s\n", ifr.ifr_name);
		perror("TUNSETIFF");
		goto error;
	}

	/* Always set the fiile descriptor to non-blocking */
	if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0) {
		RTE_LOG(ERR, PMD, "Unable to set to nonblocking\n");
		perror("F_SETFL, NONBLOCK");
		goto error;
	}

	/* If the name is different that new name as default */
	if (name && strcmp(name, ifr.ifr_name))
		strcpy(name, ifr.ifr_name);

	return fd;

error:
	if (fd > 0)
		close(fd);
	return -1;
}

/*
 * Callback to handle the rx burst of packets to the correct interface and file
 * descriptor(s) in a multi-queue setup.
 */
static uint16_t
pmd_rx_burst(void *queue, struct rte_mbuf **bufs, uint16_t nb_pkts)
{
	int len, n;
	struct rte_mbuf *mbuf;
	struct rx_queue *rxq = queue;
	struct pollfd pfd;
	uint16_t num_rx;
	unsigned long num_rx_bytes = 0;

	pfd.events = POLLIN;
	pfd.fd = rxq->fd;
	for (num_rx = 0; num_rx < nb_pkts; ) {
		n = poll(&pfd, 1, 0);

		if (n <= 0)
			break;

		if (pfd.revents == 0)
			continue;

		if (pfd.revents & POLLERR) {
			rxq->stats.errs++;
			RTE_LOG(ERR, PMD, "Packet Error\n");
			break;
		}
		if (pfd.revents & POLLHUP)
			RTE_LOG(ERR, PMD, "Peer closed connection\n");

		/* allocate the next mbuf */
		mbuf = rte_pktmbuf_alloc(rxq->mp);
		if (unlikely(mbuf == NULL)) {
			RTE_LOG(ERR, PMD, "Unable to allocate mbuf\n");
			break;
		}

		len = read(pfd.fd, rte_pktmbuf_mtod(mbuf, char *),
			   rte_pktmbuf_tailroom(mbuf));
		if (len <= 0) {
			RTE_LOG(ERR, PMD, "len %d\n", len);
			rte_pktmbuf_free(mbuf);
			break;
		}

		mbuf->data_len = len;
		mbuf->pkt_len = len;
		mbuf->port = rxq->in_port;

		/* account for the receive frame */
		bufs[num_rx++] = mbuf;
		num_rx_bytes += mbuf->pkt_len;
	}
	rxq->stats.ipackets += num_rx;
	rxq->stats.ibytes += num_rx_bytes;

	return num_rx;
}

/*
 * Callback to handle sending packets from the tap interface
 */
static uint16_t
pmd_tx_burst(void *queue, struct rte_mbuf **bufs, uint16_t nb_pkts)
{
	struct rte_mbuf *mbuf;
	struct tx_queue *txq = queue;
	struct pollfd pfd;
	uint16_t num_tx = 0;
	unsigned long num_tx_bytes = 0;
	int i, n;

	if (unlikely(nb_pkts == 0))
		return 0;

	pfd.events = POLLOUT;
	pfd.fd = txq->fd;
	for (i = 0; i < nb_pkts; i++) {
		n = poll(&pfd, 1, 0);

		if (n <= 0)
			break;

		if (pfd.revents & POLLOUT) {
			/* copy the tx frame data */
			mbuf = bufs[num_tx];
			n = write(pfd.fd, rte_pktmbuf_mtod(mbuf, void*),
				  rte_pktmbuf_pkt_len(mbuf));
			if (n <= 0)
				break;

			num_tx++;
			num_tx_bytes += mbuf->pkt_len;
			rte_pktmbuf_free(mbuf);
		}
	}

	txq->stats.opackets += num_tx;
	txq->stats.errs += nb_pkts - num_tx;
	txq->stats.obytes += num_tx_bytes;

	return num_tx;
}

static int
tap_dev_start(struct rte_eth_dev *dev)
{
	/* Force the Link up */
	dev->data->dev_link.link_status = ETH_LINK_UP;

	return 0;
}

/*
 * This function gets called when the current port gets stopped.
 */
static void
tap_dev_stop(struct rte_eth_dev *dev)
{
	int i;
	struct pmd_internals *internals = dev->data->dev_private;

	for (i = 0; i < internals->nb_queues; i++)
		if (internals->fds[i] != -1)
			close(internals->fds[i]);

	dev->data->dev_link.link_status = ETH_LINK_DOWN;
}

static int
tap_dev_configure(struct rte_eth_dev *dev __rte_unused)
{
	return 0;
}

static void
tap_dev_info(struct rte_eth_dev *dev, struct rte_eth_dev_info *dev_info)
{
	struct pmd_internals *internals = dev->data->dev_private;

	dev_info->driver_name = drivername;
	dev_info->if_index = internals->if_index;
	dev_info->max_mac_addrs = 1;
	dev_info->max_rx_pktlen = (uint32_t)ETHER_MAX_VLAN_FRAME_LEN;
	dev_info->max_rx_queues = (uint16_t)internals->nb_queues;
	dev_info->max_tx_queues = (uint16_t)internals->nb_queues;
	dev_info->min_rx_bufsize = 0;
	dev_info->pci_dev = NULL;
}

static void
tap_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *igb_stats)
{
	unsigned i, imax;
	unsigned long rx_total = 0, tx_total = 0, tx_err_total = 0;
	unsigned long rx_bytes_total = 0, tx_bytes_total = 0;
	const struct pmd_internals *internal = dev->data->dev_private;

	imax = (internal->nb_queues < RTE_ETHDEV_QUEUE_STAT_CNTRS) ?
		internal->nb_queues : RTE_ETHDEV_QUEUE_STAT_CNTRS;

	for (i = 0; i < imax; i++) {
		igb_stats->q_ipackets[i] = internal->rxq[i].stats.ipackets;
		igb_stats->q_ibytes[i] = internal->rxq[i].stats.ibytes;
		rx_total += igb_stats->q_ipackets[i];
		rx_bytes_total += igb_stats->q_ibytes[i];
	}

	imax = (internal->nb_queues < RTE_ETHDEV_QUEUE_STAT_CNTRS) ?
		internal->nb_queues : RTE_ETHDEV_QUEUE_STAT_CNTRS;

	for (i = 0; i < imax; i++) {
		igb_stats->q_opackets[i] = internal->txq[i].stats.opackets;
		igb_stats->q_errors[i] = internal->txq[i].stats.errs;
		igb_stats->q_obytes[i] = internal->txq[i].stats.obytes;
		tx_total += igb_stats->q_opackets[i];
		tx_err_total += igb_stats->q_errors[i];
		tx_bytes_total += igb_stats->q_obytes[i];
	}

	igb_stats->ipackets = rx_total;
	igb_stats->ibytes = rx_bytes_total;
	igb_stats->opackets = tx_total;
	igb_stats->oerrors = tx_err_total;
	igb_stats->obytes = tx_bytes_total;
}

static void
tap_stats_reset(struct rte_eth_dev *dev)
{
	int i;
	struct pmd_internals *internal = dev->data->dev_private;

	for (i = 0; i < internal->nb_queues; i++) {
		internal->rxq[i].stats.ipackets = 0;
		internal->rxq[i].stats.ibytes = 0;
	}

	for (i = 0; i < internal->nb_queues; i++) {
		internal->txq[i].stats.opackets = 0;
		internal->txq[i].stats.errs = 0;
		internal->txq[i].stats.obytes = 0;
	}
}

static void
tap_dev_close(struct rte_eth_dev *dev __rte_unused)
{
}

static void
tap_rx_queue_release(void *queue)
{
	struct rx_queue *rxq = queue;

	if (rxq && (rxq->fd > 0)) {
		close(rxq->fd);
		rxq->fd = -1;
	}
}

static void
tap_tx_queue_release(void *queue)
{
	struct tx_queue *txq = queue;

	if (txq && (txq->fd > 0)) {
		close(txq->fd);
		txq->fd = -1;
	}
}

static int
tap_link_update(struct rte_eth_dev *dev __rte_unused,
		int wait_to_complete __rte_unused)
{
	return 0;
}

static int
tap_setup_queue(struct rte_eth_dev *dev,
		struct pmd_internals *internals,
		uint16_t qid)
{
	struct rx_queue *rx = &internals->rxq[qid];
	struct tx_queue *tx = &internals->txq[qid];
	int fd;

	if ((fd = rx->fd) < 0)
		if ((fd = tx->fd) < 0) {
			RTE_LOG(INFO, PMD, "Add queue to TAP %s for qid %d\n",
				dev->data->name, qid);
			if ((fd = tun_alloc(dev->data->name)) < 0) {
				RTE_LOG(ERR, PMD, "tun_alloc(%s) failed\n", dev->data->name);
				return -1;
			}
		}

	dev->data->rx_queues[qid] = rx;
	dev->data->tx_queues[qid] = tx;

	rx->fd = tx->fd = fd;

	return fd;
}

static int
tap_rx_queue_setup(struct rte_eth_dev *dev,
		   uint16_t rx_queue_id,
		   uint16_t nb_rx_desc __rte_unused,
		   unsigned int socket_id __rte_unused,
		   const struct rte_eth_rxconf *rx_conf __rte_unused,
		   struct rte_mempool *mp)
{
	struct pmd_internals *internals = dev->data->dev_private;
	uint16_t buf_size;
	int fd;

	if ((rx_queue_id >= internals->nb_queues) || (mp == NULL)) {
		RTE_LOG(ERR, PMD, "nb_queues %d mp %p\n", internals->nb_queues, mp);
		return -1;
	}

	internals->rxq[rx_queue_id].mp = mp;
	internals->rxq[rx_queue_id].in_port = dev->data->port_id;

	/* Now get the space available for data in the mbuf */
	buf_size = (uint16_t) (rte_pktmbuf_data_room_size(mp) -
			       RTE_PKTMBUF_HEADROOM);

	if (buf_size < ETH_FRAME_LEN) {
		RTE_LOG(ERR, PMD,
			"%s: %d bytes will not fit in mbuf (%d bytes)\n",
			dev->data->name, ETH_FRAME_LEN, buf_size);
		return -ENOMEM;
	}

	fd = tap_setup_queue(dev, internals, rx_queue_id);
	if (fd == -1)
		return -1;

	internals->fds[rx_queue_id] = fd;
	RTE_LOG(INFO, PMD, "RX TAP device name %s, qid %d on fd %d\n",
		dev->data->name, rx_queue_id, internals->rxq[rx_queue_id].fd);

	return 0;
}

static int
tap_tx_queue_setup(struct rte_eth_dev *dev,
		   uint16_t tx_queue_id,
		   uint16_t nb_tx_desc __rte_unused,
		   unsigned int socket_id __rte_unused,
		   const struct rte_eth_txconf *tx_conf __rte_unused)
{
	struct pmd_internals *internals = dev->data->dev_private;
	int ret = -1;

	if (tx_queue_id >= internals->nb_queues)
		return -1;

	ret = tap_setup_queue(dev, internals, tx_queue_id);

	RTE_LOG(INFO, PMD, "TX TAP device name %s, qid %d on fd %d\n",
		dev->data->name, tx_queue_id, internals->txq[tx_queue_id].fd);

	return ret;
}

static const struct eth_dev_ops ops = {
	.dev_start              = tap_dev_start,
	.dev_stop               = tap_dev_stop,
	.dev_close              = tap_dev_close,
	.dev_configure          = tap_dev_configure,
	.dev_infos_get          = tap_dev_info,
	.rx_queue_setup         = tap_rx_queue_setup,
	.tx_queue_setup         = tap_tx_queue_setup,
	.rx_queue_release       = tap_rx_queue_release,
	.tx_queue_release       = tap_tx_queue_release,
	.link_update            = tap_link_update,
	.stats_get              = tap_stats_get,
	.stats_reset            = tap_stats_reset,
};

#define RTE_USE_GLOBAL_DATA	0x0000
#define RTE_USE_PRIVATE_DATA	0x0001

static int
pmd_mac_address(int fd, struct rte_eth_dev *dev, struct ether_addr *addr)
{
	struct ifreq ifr;

	if ((fd <= 0) || (dev == NULL) || (addr == NULL))
		return -1;

	memset(&ifr, 0, sizeof(ifr));

	if (ioctl(fd, SIOCGIFHWADDR, &ifr) == -1) {
		RTE_LOG(ERR, PMD, "ioctl failed (SIOCGIFHWADDR) (%s)\n",
			ifr.ifr_name);
		return -1;
	}

	/* Set the host based MAC address to this special MAC format */
	ifr.ifr_hwaddr.sa_data[0] = 'T';
	ifr.ifr_hwaddr.sa_data[1] = 'a';
	ifr.ifr_hwaddr.sa_data[2] = 'p';
	ifr.ifr_hwaddr.sa_data[3] = '-';
	ifr.ifr_hwaddr.sa_data[4] = dev->data->port_id;
	ifr.ifr_hwaddr.sa_data[5] = dev->data->numa_node;
	if (ioctl(fd, SIOCSIFHWADDR, &ifr) == -1) {
		RTE_LOG(ERR, PMD, "%s: ioctl failed (SIOCSIFHWADDR) (%s)\n",
			dev->data->name, ifr.ifr_name);
		return -1;
	}

	/*
	 * Set the local application MAC address, needs to be different then
	 * the host based MAC address.
	 */
	ifr.ifr_hwaddr.sa_data[0] = 'd';
	ifr.ifr_hwaddr.sa_data[1] = 'n';
	ifr.ifr_hwaddr.sa_data[2] = 'e';
	ifr.ifr_hwaddr.sa_data[3] = 't';
	ifr.ifr_hwaddr.sa_data[4] = dev->data->port_id;
	ifr.ifr_hwaddr.sa_data[5] = dev->data->numa_node;
	memcpy(addr, ifr.ifr_hwaddr.sa_data, ETH_ALEN);

	return 0;
}

static int
rte_eth_dev_create(const char *name, int dev_type,
		   struct rte_eth_dev **eth_dev,
		   const struct eth_dev_ops *dev_ops,
		   void **internals, size_t internal_size,
		   uint16_t flag)
{
	char buff[RTE_ETH_NAME_MAX_LEN];
	int numa_node = rte_socket_id();
	struct rte_eth_dev *dev = NULL;
	struct rte_eth_dev_data *data = NULL;
	void *priv = NULL;

	if ((name == NULL) || (eth_dev == NULL) || (dev_ops == NULL) ||
	    (internals == NULL) || (internal_size == 0)) {
		RTE_PMD_DEBUG_TRACE("Paramters are invalid\n");
		return -1;
	}

	dev = rte_eth_dev_allocate(name, dev_type);
	if (dev == NULL) {
		RTE_PMD_DEBUG_TRACE("%s: rte_eth_dev_allocate failed for %s\n",
				    name, buff);
		goto error;
	}

	if (flag & RTE_USE_PRIVATE_DATA) {
		/*
		 * now do all data allocation - for eth_dev structure, dummy
		 * pci driver and internal (private) data
		 */
		snprintf(buff, sizeof(buff), "D-%s-%d", name, numa_node);
		data = rte_zmalloc_socket(buff, sizeof(struct rte_eth_dev_data),
					  0, numa_node);
		if (data == NULL) {
			RTE_PMD_DEBUG_TRACE("%s: Unable to allocate memory\n",
					    name);
			goto error;
		}
		/* move the current state of the structure to the new one */
		rte_memcpy(data, dev->data, sizeof(struct rte_eth_dev_data));
		dev->data = data;	/* Override the current data pointer */
	} else
		data = dev->data;

	snprintf(buff, sizeof(buff), "I-%s-%d", name, numa_node);
	priv = rte_zmalloc_socket(buff, internal_size, 0, numa_node);
	if (priv == NULL) {
		RTE_PMD_DEBUG_TRACE("Unable to allocate internal memory %lu\n",
				    internal_size);
		goto error;
	}

	/* Setup some default values */
	dev->dev_ops = dev_ops;
	data->dev_private = priv;
	data->port_id = dev->data->port_id;
	memmove(data->name, dev->data->name, strlen(dev->data->name));

	dev->driver = NULL;
	data->dev_flags = RTE_ETH_DEV_DETACHABLE;
	data->kdrv = RTE_KDRV_NONE;
	data->numa_node = numa_node;

	*eth_dev = dev;
	*internals = priv;

	return 0;
error:
	rte_free(priv);

	if (flag & RTE_USE_PRIVATE_DATA)
		rte_free(data);

	rte_eth_dev_release_port(dev);

	return -1;
}

static int
pmd_init_internals(const char *name, struct tap_info *tap,
		   struct pmd_internals **internals,
		   struct rte_eth_dev **eth_dev)
{
	struct rte_eth_dev *dev = NULL;
	struct pmd_internals *internal = NULL;
	struct rte_eth_dev_data *data = NULL;
	int ret, i, fd = -1;

	RTE_LOG(INFO, PMD,
		"%s: Create TUN/TAP Ethernet device with %d queues on numa %u\n",
		name, RTE_PMD_TAP_MAX_QUEUES, rte_socket_id());

	pmd_link.link_speed = tap->speed;

	ret = rte_eth_dev_create(tap->name, RTE_ETH_DEV_VIRTUAL, &dev, &ops,
				 (void **)&internal, sizeof(struct pmd_internals),
				 RTE_USE_PRIVATE_DATA);
	if (ret < 0)
		return -1;

	strncpy(internal->name, tap->name, sizeof(internal->name));

	internal->nb_queues = RTE_PMD_TAP_MAX_QUEUES;

	/* Create the first Tap device */
	if ((fd = tun_alloc(dev->data->name)) < 0) {
		RTE_LOG(ERR, PMD, "tun_alloc(%s) failed\n", dev->data->name);
		rte_free(internal);
		rte_eth_dev_release_port(dev);
		return -1;
	}

	/* Presetup the fds to -1 as being not working */
	for(i = 0; i < RTE_PMD_TAP_MAX_QUEUES; i++) {
		internal->fds[i] = -1;
		internal->rxq[i].fd = -1;
		internal->txq[i].fd = -1;
	}

	/* Take the TUN/TAP fd and place in the first location */
	internal->rxq[0].fd = fd;
	internal->txq[0].fd = fd;
	internal->fds[0] = fd;

	if (pmd_mac_address(fd, dev, &internal->eth_addr) < 0) {
		rte_free(internal);
		rte_eth_dev_release_port(dev);
		return -1;
	}

	data = dev->data;

	data->dev_link = pmd_link;
	data->mac_addrs = &internal->eth_addr;

	data->nb_rx_queues = (uint16_t)internal->nb_queues;
	data->nb_tx_queues = (uint16_t)internal->nb_queues;
	data->drv_name = drivername;

	*eth_dev = dev;
	*internals = internal;

	return 0;
}

static int
eth_dev_tap_create(const char *name, struct tap_info *tap)
{
	struct pmd_internals *internals = NULL;
	struct rte_eth_dev *eth_dev = NULL;

	if (pmd_init_internals(name, tap, &internals, &eth_dev) < 0)
		return -1;

	eth_dev->rx_pkt_burst = pmd_rx_burst;
	eth_dev->tx_pkt_burst = pmd_tx_burst;

	return 0;
}

static int
set_interface_name(const char *key __rte_unused,
		   const char *value,
		   void *extra_args)
{
	struct tap_info *tap = (struct tap_info *)extra_args;

	if (value)
		snprintf(tap->name, sizeof(tap->name), "%s", value);
	else
		snprintf(tap->name, sizeof(tap->name), "dtap%d", (tap_unit - 1));

	return 0;
}

static int
set_interface_speed(const char *key __rte_unused,
		    const char *value,
		    void *extra_args __rte_unused)
{
	struct tap_info *tap = (struct tap_info *)extra_args;

	pmd_link.link_speed = (value) ? atoi(value) : ETH_SPEED_NUM_10G;
	tap->speed = pmd_link.link_speed;

	return 0;
}

/*
 * Open a TAP interface device.
 */
static int
pmd_tap_devinit(const char *name, const char *params)
{
	int ret = 0;
	struct rte_kvargs *kvlist;
	struct tap_info tap_info;

	/* Setup default values */
	memset(&tap_info, 0, sizeof(tap_info));

	tap_info.speed = ETH_SPEED_NUM_10G;
	snprintf(tap_info.name, sizeof(tap_info.name), "tap%d", tap_unit++);

	if ((params == NULL) || (params[0] == '\0')) {
		RTE_LOG(INFO, PMD, "Initializing pmd_tap for %s\n", name);

		ret = eth_dev_tap_create(name, &tap_info);
		goto leave;
	}

	RTE_LOG(INFO, PMD, "Initialize %s with params (%s)\n", name, params);

	kvlist = rte_kvargs_parse(params, valid_arguments);
	if (!kvlist) {
		ret = eth_dev_tap_create(name, &tap_info);
		goto leave;
	}

	if (rte_kvargs_count(kvlist, ETH_TAP_SPEED_ARG) == 1) {
		ret = rte_kvargs_process(kvlist, ETH_TAP_SPEED_ARG,
					 &set_interface_speed, &tap_info);
		if (ret < 0)
			goto leave;
	} else
		set_interface_speed(NULL, NULL, &tap_info);

	if (rte_kvargs_count(kvlist, ETH_TAP_IFACE_ARG) == 1) {
		ret = rte_kvargs_process(kvlist, ETH_TAP_IFACE_ARG,
					 &set_interface_name, &tap_info);
		if (ret < 0)
			goto leave;
	} else
		set_interface_name(NULL, NULL, (void *)&tap_info);

	rte_kvargs_free(kvlist);

leave:
	if (ret == -1)
		RTE_LOG(INFO, PMD, "Failed to create pmd_tap for %s\n", name);

	return ret;
}

/*
 * detach a TAP device.
 */
static int
pmd_tap_devuninit(const char *name)
{
	struct rte_eth_dev *eth_dev = NULL;
	struct pmd_internals *internals;
	int i;

	RTE_LOG(INFO, PMD, "Closing TUN/TAP Ethernet device on numa %u\n",
		rte_socket_id());

	if (name == NULL)
		return 0;

	/* find the ethdev entry */
	eth_dev = rte_eth_dev_allocated(name);
	if (eth_dev == NULL)
		return 0;

	internals = eth_dev->data->dev_private;
	for (i = 0; i < internals->nb_queues; i++)
		if (internals->fds[i] != -1)
			close(internals->fds[i]);

	rte_free(eth_dev->data->dev_private);
	rte_free(eth_dev->data);

	rte_eth_dev_release_port(eth_dev);

	return 0;
}

static struct rte_driver pmd_tap_drv = {
	.type = PMD_VDEV,
	.init = pmd_tap_devinit,
	.uninit = pmd_tap_devuninit,
};

PMD_REGISTER_DRIVER(pmd_tap_drv, eth_tap);
DRIVER_REGISTER_PARAM_STRING(eth_tap,
			     "iface=<string>,speed=N");
