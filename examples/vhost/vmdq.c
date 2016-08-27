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
#include <rte_atomic.h>
#include <rte_cycles.h>
#include <rte_ethdev.h>
#include <rte_log.h>
#include <rte_string_fns.h>
#include <rte_malloc.h>
#include <rte_virtio_net.h>
#include <rte_ip.h>
#include <rte_tcp.h>

#include "vswitch_common.h"
#include "vmdq.h"

#define JUMBO_FRAME_MAX_SIZE    0x2600
/* State of virtio device. */
#define DEVICE_MAC_LEARNING 0
#define DEVICE_RX			1
#define DEVICE_SAFE_REMOVE	2

#define VLAN_HLEN       4

static struct vswitch_dev *vmdq_switch_dev_g;

const uint16_t vlan_tags[] = {
	1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007,
	1008, 1009, 1010, 1011,	1012, 1013, 1014, 1015,
	1016, 1017, 1018, 1019, 1020, 1021, 1022, 1023,
	1024, 1025, 1026, 1027, 1028, 1029, 1030, 1031,
	1032, 1033, 1034, 1035, 1036, 1037, 1038, 1039,
	1040, 1041, 1042, 1043, 1044, 1045, 1046, 1047,
	1048, 1049, 1050, 1051, 1052, 1053, 1054, 1055,
	1056, 1057, 1058, 1059, 1060, 1061, 1062, 1063,
};

/* empty vmdq configuration structure. Filled in programatically */
static struct rte_eth_conf vmdq_conf_default = {
	.rxmode = {
		.mq_mode        = ETH_MQ_RX_VMDQ_ONLY,
		.split_hdr_size = 0,
		.header_split   = 0, /**< Header Split disabled */
		.hw_ip_checksum = 0, /**< IP checksum offload disabled */
		.hw_vlan_filter = 0, /**< VLAN filtering disabled */
		/*
		 * It is necessary for 1G NIC such as I350,
		 * this fixes bug of ipv4 forwarding in guest can't
		 * forward pakets from one virtio dev to another virtio dev.
		 */
		.hw_vlan_strip  = 1, /**< VLAN strip enabled. */
		.jumbo_frame    = 0, /**< Jumbo Frame Support disabled */
		.hw_strip_crc   = 0, /**< CRC stripped by hardware */
	},

	.txmode = {
		.mq_mode = ETH_MQ_TX_NONE,
	},
	.rx_adv_conf = {
		/*
		 * should be overridden separately in code with
		 * appropriate values
		 */
		.vmdq_rx_conf = {
			.nb_queue_pools = ETH_8_POOLS,
			.enable_default_pool = 0,
			.default_pool = 0,
			.nb_pool_maps = 0,
			.pool_map = {{0, 0},},
		},
	},
};


static int vmdq_switch_init(__attribute__((unused))struct vswitch_dev *vs_dev,
			    uint64_t conf_flags)
{
	uint32_t enable;

	if (conf_flags & VS_CNF_FLG_VM2VM_HARDWARE) {
		/* Enable VT loop back to let L2 switch to do it. */
		vmdq_conf_default.rx_adv_conf.vmdq_rx_conf.enable_loop_back = 1;
		RTE_LOG(DEBUG, VHOST_CONFIG,
			"Enable loop back for L2 switch in vmdq.\n");
	}

	if (conf_flags & VS_CNF_FLG_PROMISCOUS_EN) {
		vmdq_conf_default.rx_adv_conf.vmdq_rx_conf.rx_mode =
				ETH_VMDQ_ACCEPT_BROADCAST |
				ETH_VMDQ_ACCEPT_MULTICAST;
	}

	if (conf_flags & VS_CNF_FLG_JUMBO_EN) {
		vmdq_conf_default.rxmode.jumbo_frame = 1;
		vmdq_conf_default.rxmode.max_rx_pkt_len = JUMBO_FRAME_MAX_SIZE;
	}

	enable = !!(conf_flags & VS_CNF_FLG_VLAN_STRIP_EN);
	vmdq_conf_default.rxmode.hw_vlan_strip = enable;

	return 0;
}

/*
 * Builds up the correct configuration for VMDQ VLAN pool map
 * according to the pool & queue limits.
 */
static inline int
vmdq_get_eth_conf(struct vswitch_port *vs_port, struct rte_eth_conf *eth_conf)
{
	struct rte_eth_vmdq_rx_conf conf;
	struct rte_eth_vmdq_rx_conf *def_conf =
		&vmdq_conf_default.rx_adv_conf.vmdq_rx_conf;
	struct vmdq_switch_priv *priv = vs_port->vs_dev->priv;
	unsigned i;

	memset(&conf, 0, sizeof(conf));
	conf.nb_queue_pools = (enum rte_eth_nb_pools)priv->num_devices;
	conf.nb_pool_maps = priv->num_devices;
	conf.enable_loop_back = def_conf->enable_loop_back;
	conf.rx_mode = def_conf->rx_mode;

	for (i = 0; i < conf.nb_pool_maps; i++) {
		conf.pool_map[i].vlan_id = vlan_tags[ i ];
		conf.pool_map[i].pools = (1UL << i);
	}

	(void)(rte_memcpy(eth_conf, &vmdq_conf_default, sizeof(*eth_conf)));
	(void)(rte_memcpy(&eth_conf->rx_adv_conf.vmdq_rx_conf, &conf,
		   sizeof(eth_conf->rx_adv_conf.vmdq_rx_conf)));
	return 0;
}

static int vmdq_add_port_phys(struct vswitch_port *vs_port)
{
	struct rte_eth_dev_info dev_info;
	struct vmdq_switch_priv *priv = vs_port->vs_dev->priv;
	uint16_t queues_per_pool;
	int rc = 0;

	if (priv->phys_port_count >= VMDQ_MAX_PHYS_PORTS) {
		RTE_LOG(INFO, VHOST_CONFIG,
			"Physical ports greater than max devices(%d)\n",
			VMDQ_MAX_PHYS_PORTS);
		rc = -EBUSY;
		goto out;
	}

	rte_eth_dev_info_get (vs_port->port_id, &dev_info);
	if (dev_info.max_vmdq_pools > VMDQ_MAX_VIRTIO_PORTS) {
		RTE_LOG(INFO, VHOST_CONFIG,
			"Num devices (%d) greater than Max (%d)\n",
			dev_info.max_vmdq_pools, VMDQ_MAX_VIRTIO_PORTS);
		rc = -EINVAL;
		goto out;

	}

	priv->num_devices = dev_info.max_vmdq_pools;
	priv->phys_port_count++;

	priv->num_pf_queues = dev_info.max_rx_queues - dev_info.vmdq_queue_num;
	queues_per_pool =  dev_info.vmdq_queue_num / dev_info.max_vmdq_pools;
	priv->queues_per_pool = queues_per_pool;
	priv->num_vmdq_queues = priv->phys_port_count * queues_per_pool;
	priv->num_queues = priv->num_pf_queues + priv->num_vmdq_queues;
	priv->vmdq_queue_base = dev_info.vmdq_queue_base;
	priv->vmdq_pool_base  = dev_info.vmdq_pool_base;

	rc = vmdq_get_eth_conf(vs_port, &vs_port->port_conf);
	if (rc < 0) {
		goto out;
	}

	/*In VMDQ vhost_switch only one physical port is required, keep it
	 * global so that it can be accessed while doing tx/rx to physical port
	 */
	priv->phys_port = vs_port;


	printf("pf queue num: %u, configured vmdq pool num: %u, each vmdq pool has %u queues\n",
		priv->num_pf_queues, priv->num_devices, priv->queues_per_pool);

	RTE_LOG(INFO, VHOST_PORT, "Max virtio devices supported: %u\n",
		priv->num_devices);
out:
	if (rc) {
		priv->phys_port_count--;
	}
	return rc;
}

static int vmdq_add_port_virtio(struct vswitch_port *vs_port)
{
	struct vmdq_switch_priv *priv = vs_port->vs_dev->priv;
	uint16_t rxq;

	rxq = vs_port->port_id * priv->queues_per_pool + priv->vmdq_queue_base;
	vs_port->phys_port_rxq = rxq;

	priv->virtio_port_map[rxq] = vs_port;
	return 0;
}

static int vmdq_add_port(struct vswitch_port *port)
{
	int rc = 0;

	switch(port->type) {
	case VSWITCH_PTYPE_PHYS:
		rc = vmdq_add_port_phys(port);
		break;
	case VSWITCH_PTYPE_VIRTIO:
		rc = vmdq_add_port_virtio(port);
		break;
	default:
		RTE_LOG(INFO, VHOST_CONFIG, "Unkown port[id %d] type %d\n",
			port->port_id, port->type);
		rc = -EINVAL;
	}

	return rc;
}

static int vmdq_port_start(struct vswitch_port *port)
{
	int rc = 0;

	switch(port->type) {
	case VSWITCH_PTYPE_PHYS:
		rc  = rte_eth_dev_start(port->port_id);
		break;
	case VSWITCH_PTYPE_VIRTIO:
		/*No specefic function to start virtio dev (?? check) */
		rc = 0;
		break;
	default:
		RTE_LOG(INFO, VHOST_CONFIG, "Unkown port[id %d] type %d\n",
			port->port_id, port->type);
		rc = -EINVAL;
	}


	/* Start the device. */
	if (rc) {
		RTE_LOG(ERR, VHOST_PORT, "Failed to init port[id %d, typ %d]\n",
			port->port_id, port->type);
	}

	return rc;
}

/*
 * This function learns the MAC address of the device and registers this along with a
 * vlan tag to a VMDQ.
 */
static int
link_vmdq(struct vswitch_port *vs_port, struct rte_mbuf *m)
{
	struct vhost_dev *vdev = vs_port->priv;
	struct vmdq_switch_priv *priv = vs_port->vs_dev->priv;
	struct vswitch_dev *vs_dev = vs_port->vs_dev;
	struct vswitch_port *phys_port = priv->phys_port;
	struct ether_hdr *pkt_hdr;
	int i, ret;

	/* Learn MAC address of guest device from packet */
	pkt_hdr = rte_pktmbuf_mtod(m, struct ether_hdr *);

	if (find_vhost_dev(&pkt_hdr->s_addr)) {
		RTE_LOG(ERR, VHOST_DATA,
			"(%d) device is using a registered MAC!\n",
			vdev->vid);
		return -1;
	}

	for (i = 0; i < ETHER_ADDR_LEN; i++) {
		vdev->mac_address.addr_bytes[i] = pkt_hdr->s_addr.addr_bytes[i];
		vs_port->mac_addr.addr_bytes[i] = pkt_hdr->s_addr.addr_bytes[i];
	}

	/* vlan_tag currently uses the device_id. */
	vdev->vlan_tag = vlan_tags[vdev->vid];

	/* Print out VMDQ registration info. */
	RTE_LOG(INFO, VHOST_DATA,
		"(%d) mac %02x:%02x:%02x:%02x:%02x:%02x and vlan %d registered\n",
		vdev->vid,
		vdev->mac_address.addr_bytes[0], vdev->mac_address.addr_bytes[1],
		vdev->mac_address.addr_bytes[2], vdev->mac_address.addr_bytes[3],
		vdev->mac_address.addr_bytes[4], vdev->mac_address.addr_bytes[5],
		vdev->vlan_tag);

	/* Register the MAC address. */
	ret = rte_eth_dev_mac_addr_add(phys_port->port_id, &vdev->mac_address,
				(uint32_t)vdev->vid + priv->vmdq_pool_base);
	if (ret)
		RTE_LOG(ERR, VHOST_DATA,
			"(%d) failed to add device MAC address to VMDQ\n",
			vdev->vid);

	/* Enable stripping of the vlan tag as we handle routing. */
	if (vs_dev->conf_flags & VS_CNF_FLG_VLAN_STRIP_EN)
		rte_eth_dev_set_vlan_strip_on_queue(phys_port->port_id,
			(uint16_t)vs_port->phys_port_rxq, 1);

	/* Set device as ready for RX. */
	vdev->ready = DEVICE_RX;

	return 0;
}

static int vmdq_learn_port (struct vswitch_port *vs_port,
			    struct rte_mbuf **pkts,
		     __attribute__((unused))uint16_t count)
{
	int rc  = 0;

	if (vs_port->type == VSWITCH_PTYPE_VIRTIO)
		rc = link_vmdq(vs_port, pkts[0]);

	return rc;
}

/*
 * Removes MAC address and vlan tag from VMDQ. Ensures that nothing is adding buffers to the RX
 * queue before disabling RX on the device.
 */
static void unlink_vmdq(struct vswitch_port *vs_port)
{
	struct vhost_dev *vdev = vs_port->priv;
	struct vmdq_switch_priv *priv = vs_port->vs_dev->priv;
	struct vswitch_port *phys_port = priv->phys_port;
	unsigned i = 0;
	unsigned rx_count;
	struct rte_mbuf *pkts_burst[MAX_PKT_BURST];

	if (vdev->ready == DEVICE_RX) {
		/*clear MAC and VLAN settings*/
		rte_eth_dev_mac_addr_remove(phys_port->port_id,
					    &vdev->mac_address);
		for (i = 0; i < 6; i++) {
			vdev->mac_address.addr_bytes[i] = 0;
			vs_port->mac_addr.addr_bytes[i] = 0;
		}

		vdev->vlan_tag = 0;

		/*Clear out the receive buffers*/
		rx_count = rte_eth_rx_burst(phys_port->port_id,
					(uint16_t)vs_port->phys_port_rxq,
					pkts_burst, MAX_PKT_BURST);

		while (rx_count) {
			for (i = 0; i < rx_count; i++)
				rte_pktmbuf_free(pkts_burst[i]);

			rx_count = rte_eth_rx_burst(phys_port->port_id,
					(uint16_t)vdev->vmdq_rx_q, pkts_burst,
					MAX_PKT_BURST);
		}

		vdev->ready = DEVICE_MAC_LEARNING;
	}
}

static int vmdq_unlearn_port (struct vswitch_port *vs_port)
{
	int rc = 0;

	if (vs_port->type == VSWITCH_PTYPE_VIRTIO)
		unlink_vmdq(vs_port);

	return rc;
}

/*
 * Check if the destination MAC of a packet is one local VM,
 * and get its vlan tag, and offset if it is.
 */
static inline int __attribute__((always_inline))
find_local_dest(struct vhost_dev *vdev, struct rte_mbuf *m,
	uint32_t *offset, uint16_t *vlan_tag)
{
	struct vhost_dev *dst_vdev;
	struct ether_hdr *pkt_hdr = rte_pktmbuf_mtod(m, struct ether_hdr *);

	dst_vdev = find_vhost_dev(&pkt_hdr->d_addr);
	if (!dst_vdev)
		return 0;

	if (vdev->vid == dst_vdev->vid) {
		RTE_LOG(DEBUG, VHOST_DATA,
			"(%d) TX: src and dst MAC is same. Dropping packet.\n",
			vdev->vid);
		return -1;
	}

	/*
	 * HW vlan strip will reduce the packet length
	 * by minus length of vlan tag, so need restore
	 * the packet length by plus it.
	 */
	*offset  = VLAN_HLEN;
	*vlan_tag = vlan_tags[vdev->vid];

	RTE_LOG(DEBUG, VHOST_DATA,
		"(%d) TX: pkt to local VM device id: (%d), vlan tag: %u.\n",
		vdev->vid, dst_vdev->vid, *vlan_tag);

	return 0;
}

static uint16_t
get_psd_sum(void *l3_hdr, uint64_t ol_flags)
{
	if (ol_flags & PKT_TX_IPV4)
		return rte_ipv4_phdr_cksum(l3_hdr, ol_flags);
	else /* assume ethertype == ETHER_TYPE_IPv6 */
		return rte_ipv6_phdr_cksum(l3_hdr, ol_flags);
}

static void virtio_tx_offload(struct rte_mbuf *m)
{
	void *l3_hdr;
	struct ipv4_hdr *ipv4_hdr = NULL;
	struct tcp_hdr *tcp_hdr = NULL;
	struct ether_hdr *eth_hdr = rte_pktmbuf_mtod(m, struct ether_hdr *);

	l3_hdr = (char *)eth_hdr + m->l2_len;

	if (m->ol_flags & PKT_TX_IPV4) {
		ipv4_hdr = l3_hdr;
		ipv4_hdr->hdr_checksum = 0;
		m->ol_flags |= PKT_TX_IP_CKSUM;
	}

	tcp_hdr = (struct tcp_hdr *)((char *)l3_hdr + m->l3_len);
	tcp_hdr->cksum = get_psd_sum(l3_hdr, m->ol_flags);
}

/*
 * This function routes the TX packet to the correct interface. This
 * may be a local device or the physical port.
 */
static inline void __attribute__((always_inline))
virtio_tx_route(struct vswitch_port *vs_port,
		struct rte_mbuf *m, uint16_t vlan_tag)
{
	struct vhost_dev *vdev = vs_port->priv;
	struct vswitch_dev *vs_dev = vs_port->vs_dev;
	struct mbuf_table *tx_q;
	unsigned offset = 0;
	const uint16_t lcore_id = rte_lcore_id();
	struct ether_hdr *nh;


	nh = rte_pktmbuf_mtod(m, struct ether_hdr *);
	if (unlikely(is_broadcast_ether_addr(&nh->d_addr))) {

		vs_do_broadcast_fwd(vs_port->vs_dev, vs_port, m);
		goto queue2nic;
	}

	/*check if destination is local VM*/
	if ((vs_dev->conf_flags & VS_CNF_FLG_VM2VM_SOFTWARE)) {
		if (!virtio_tx_local(vdev, m))
			rte_pktmbuf_free(m);
		return;
	}

	if (unlikely(vs_dev->conf_flags & VS_CNF_FLG_VM2VM_HARDWARE)) {
		if (unlikely(find_local_dest(vdev, m, &offset,
					     &vlan_tag) != 0)) {
			rte_pktmbuf_free(m);
			return;
		}
	}

	RTE_LOG(DEBUG, VHOST_DATA,
		"(%d) TX: MAC address is external\n", vdev->vid);

queue2nic:

	/*Add packet to the port tx queue*/
	tx_q = vhost_switch_get_txq(lcore_id);

	nh = rte_pktmbuf_mtod(m, struct ether_hdr *);
	if (unlikely(nh->ether_type == rte_cpu_to_be_16(ETHER_TYPE_VLAN))) {
		/* Guest has inserted the vlan tag. */
		struct vlan_hdr *vh = (struct vlan_hdr *) (nh + 1);
		uint16_t vlan_tag_be = rte_cpu_to_be_16(vlan_tag);
		if ((vs_dev->conf_flags & VS_CNF_FLG_VM2VM_HARDWARE) &&
			(vh->vlan_tci != vlan_tag_be))
			vh->vlan_tci = vlan_tag_be;
	} else {
		m->ol_flags |= PKT_TX_VLAN_PKT;

		/*
		 * Find the right seg to adjust the data len when offset is
		 * bigger than tail room size.
		 */
		if (unlikely((vs_dev->conf_flags & VS_CNF_FLG_VM2VM_HARDWARE))){
			if (likely(offset <= rte_pktmbuf_tailroom(m)))
				m->data_len += offset;
			else {
				struct rte_mbuf *seg = m;

				while ((seg->next != NULL) &&
					(offset > rte_pktmbuf_tailroom(seg)))
					seg = seg->next;

				seg->data_len += offset;
			}
			m->pkt_len += offset;
		}

		m->vlan_tci = vlan_tag;
	}

	if (m->ol_flags & PKT_TX_TCP_SEG)
		virtio_tx_offload(m);

	tx_q->m_table[tx_q->len++] = m;
	if (vs_dev->conf_flags & VS_CNF_FLG_STATS_EN) {
		vdev->stats.tx_total++;
		vdev->stats.tx++;
	}

	if (unlikely(tx_q->len == MAX_PKT_BURST))
		do_drain_mbuf_table(tx_q);
}

static int vmdq_lookup_n_fwd_virtio(struct vswitch_port *vs_port,
			struct rte_mbuf **pkts, uint16_t count,
			__attribute__((unused)) uint16_t in_rxq)
{
	int i;
	struct vhost_dev *vdev = vs_port->priv;

	for (i = 0; i < count; ++i)
		virtio_tx_route(vs_port, pkts[i], vlan_tags[vdev->vid]);

	return 0;
}

static int vmdq_lookup_n_fwd_phys(struct vswitch_port *vs_port,
			struct rte_mbuf **pkts, uint16_t count, uint16_t in_rxq)
{
	struct vswitch_port *dest_port;
	struct vhost_dev *dest_vdev;
	struct vmdq_switch_priv *priv = vs_port->vs_dev->priv;
	uint16_t enqueue_count;

	dest_port = priv->virtio_port_map[in_rxq];
	dest_vdev = (struct vhost_dev *)dest_port->priv;
	enqueue_count = dest_port->do_tx(dest_port, rte_lcore_id(),
					 NULL, pkts, count);

	rte_atomic64_add(&dest_vdev->stats.rx_atomic, enqueue_count);

	return 0;
}

static int vmdq_lookup_n_fwd(struct vswitch_port *vs_port,
		struct rte_mbuf **pkts, uint16_t count, uint16_t in_rxq)
{
	int rc;

	switch(vs_port->type) {
	case VSWITCH_PTYPE_VIRTIO:
		rc = vmdq_lookup_n_fwd_virtio(vs_port, pkts, count, in_rxq);
		break;
	case VSWITCH_PTYPE_PHYS:
		rc = vmdq_lookup_n_fwd_phys(vs_port, pkts, count, in_rxq);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static struct vswitch_port *vmdq_sched_phys_port(struct vswitch_dev *vs_dev,
			__attribute__((unused))enum vswitch_port_type ptype,
			__attribute__((unused))uint16_t core_id)
{
	struct vmdq_switch_priv *priv = vs_dev->priv;

	/*With VMDQ do rx/tx with the only one physical port (non virtio)*/

	return priv->phys_port;
}

struct vswitch_ops vmdq_switch_ops = {
	.add_port = vmdq_add_port,
	.lookup_n_fwd = vmdq_lookup_n_fwd,
	.port_start = vmdq_port_start,
	.switch_init = vmdq_switch_init,
	.learn_port = vmdq_learn_port,
	.unlearn_port = vmdq_unlearn_port,
	.sched_rx_port = vmdq_sched_phys_port,
	.sched_tx_port = vmdq_sched_phys_port,
};

void vmdq_switch_impl_init(void)
{
	vmdq_switch_dev_g = vs_register_switch("vmdq",
			sizeof(struct vmdq_switch_priv), VMDQ_MAX_VIRTIO_PORTS,
			&vmdq_switch_ops);
	if (!vmdq_switch_dev_g) {
		RTE_LOG(DEBUG, VHOST_CONFIG, "VMDQ switch registration failure\n");
		goto out;
	}

out:
	return;
}

