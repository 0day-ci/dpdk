/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Freescale Semiconductor. All rights reserved.
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
 *     * Neither the name of Freescale Semiconductor nor the names of its
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

#include <sys/queue.h>
#include <strings.h>
#include <errno.h>

#include "vswitch_common.h"

/* Meta data for vswitch. Since this is going to be used in this file only it
 * is defined here instead of in a header file.
 */
struct vs_mdata {
	rte_spinlock_t lock;
	int vswitch_dev_count;
	LIST_HEAD(, vswitch_dev) vswitch_head;
};

static struct vs_mdata *vs_mdata_g;

static uint16_t vs_do_tx_phys_port(struct vswitch_port *port, uint16_t tx_q,
			__attribute__((unused))struct rte_mempool *mbuf_pool,
				   struct rte_mbuf **pkts, uint16_t pkt_count)
{
	return rte_eth_tx_burst(port->port_id, tx_q, pkts, pkt_count);
}


static uint16_t vs_do_tx_virtio_port(struct vswitch_port *port, uint16_t tx_q,
			__attribute__((unused)) struct rte_mempool *mbuf_pool,
				struct rte_mbuf **pkts, uint16_t pkt_count)
{
	return rte_vhost_enqueue_burst(port->port_id, tx_q, pkts, pkt_count);
}


static uint16_t vs_do_rx_phys_port (struct vswitch_port *port, uint16_t rx_q,
			__attribute__((unused))struct rte_mempool *mbuf_pool,
				 struct rte_mbuf **rx_pkts, uint16_t pkt_count)
{
	return rte_eth_rx_burst(port->port_id, rx_q, rx_pkts, pkt_count);
}

static uint16_t vs_do_rx_virtio_port (struct vswitch_port *port, uint16_t rx_q,
				struct rte_mempool *mbuf_pool,
				struct rte_mbuf **rx_pkts, uint16_t pkt_count)
{
	return rte_vhost_dequeue_burst(port->port_id, rx_q, mbuf_pool,
				       rx_pkts, pkt_count);
}

struct vswitch_dev *vs_get_vswitch_dev(const char *name)
{
	struct vswitch_dev *temp, *vs_dev = NULL;

	rte_spinlock_lock(&vs_mdata_g->lock);
	LIST_FOREACH(temp, &vs_mdata_g->vswitch_head, list) {
		if (!strncmp(temp->name, name, VSWITCH_NAME_SIZE))
			vs_dev = temp;
	}
	rte_spinlock_unlock(&vs_mdata_g->lock);

	return vs_dev;
}

static struct vswitch_port *vs_get_free_port(struct vswitch_dev *vs_dev)
{
	int i;
	struct vswitch_port *vs_port = NULL;

	for (i = 0; i < vs_dev->port_count; i++) {
		vs_port = &vs_dev->ports[i];
		if (vs_port->state == VSWITCH_PSTATE_NOT_INUSE)
			return vs_port;
	}

	return NULL;
}

static void vs_free_port(struct vswitch_port *vs_port)
{

	vs_port->state = VSWITCH_PSTATE_NOT_INUSE;
}

static __attribute__((unused))struct vswitch_port *vs_get_port(
			struct vswitch_dev *vs_dev, unsigned int port_id,
			enum vswitch_port_type type)
{
	int i;
	struct vswitch_port *vs_port = NULL;

	for (i = 0; i < vs_dev->port_count; i++) {
		vs_port = &vs_dev->ports[i];
		if ((vs_port->port_id == port_id) && (vs_port->type == type))
			return vs_port;
	}

	return NULL;
}

int vs_port_start(struct vswitch_port *vs_port)
{
	struct vswitch_ops *vs_ops = vs_port->vs_dev->ops;
	int rc = 0;

	if (vs_ops->port_start)
		rc = vs_ops->port_start(vs_port);

	if (!rc)
		vs_port->state = VSWITCH_PSTATE_LEARNING;

	return rc;
}

int vs_port_stop(struct vswitch_port *vs_port)
{
	struct vswitch_ops *vs_ops = vs_port->vs_dev->ops;
	int rc = 0;

	if (vs_ops->port_stop)
		rc = vs_ops->port_stop(vs_port);

	if (!rc)
		vs_port->state = VSWITCH_PSTATE_ADDED;

	return rc;
}

struct vswitch_port *vs_add_port(struct vswitch_dev *vs_dev, int port_id,
		enum vswitch_port_type type, void *priv)
{
	int rc = 0;
	struct vswitch_port *vs_port = NULL;
	struct vswitch_ops *vs_ops = vs_dev->ops;

	vs_port = vs_get_free_port(vs_dev);
	if (!vs_port) {
		RTE_LOG(DEBUG, VHOST_CONFIG, "Failed get free port in \
			vswitch %s\n", vs_dev->name);
		rc = -EBUSY;
		goto out;
	}

	vs_port->port_id = port_id;
	vs_port->type = type;
	vs_port->priv = priv;

	switch (type) {
	case VSWITCH_PTYPE_PHYS:
	       vs_port->do_tx = vs_do_tx_phys_port;
	       vs_port->do_rx = vs_do_rx_phys_port;
	case VSWITCH_PTYPE_VIRTIO:
	       vs_port->do_tx = vs_do_tx_virtio_port;
	       vs_port->do_rx = vs_do_rx_virtio_port;
	default:
		RTE_LOG(DEBUG, VHOST_CONFIG, "Invalid port [id %d, type %d]",
				port_id, type);
	       rc = -EINVAL;
	       goto out;
	}

	if (vs_ops->add_port)
		rc = vs_ops->add_port(vs_port);

	if (rc)
		goto out;

	vs_port->state = VSWITCH_PSTATE_ADDED;

	rte_eth_macaddr_get(vs_port->port_id, &vs_port->mac_addr);
	RTE_LOG(INFO, VHOST_PORT, "Port %u MAC: %02"PRIx8" %02"PRIx8" %02"PRIx8
			" %02"PRIx8" %02"PRIx8" %02"PRIx8"\n",
			(unsigned)port_id,
			vs_port->mac_addr.addr_bytes[0],
			vs_port->mac_addr.addr_bytes[1],
			vs_port->mac_addr.addr_bytes[2],
			vs_port->mac_addr.addr_bytes[3],
			vs_port->mac_addr.addr_bytes[4],
			vs_port->mac_addr.addr_bytes[5]);

	RTE_LOG(DEBUG, VHOST_CONFIG, "Added port [%d, type %d] to \
			vswitch %s\n", vs_port->port_id, type, vs_dev->name);
out:
	if (rc){
		RTE_LOG(INFO, VHOST_CONFIG, "Failed to Add port [%d, type %d] to \
			vswitch %s\n", port_id, type, vs_dev->name);
		if (vs_port)
			vs_free_port(vs_port);
		vs_port = NULL;
	}

	return vs_port;
}

int vs_del_port(struct vswitch_port *vs_port)
{
	int rc = 0;
	struct vswitch_ops *vs_ops = vs_port->vs_dev->ops;

	if (vs_ops->del_port)
		rc = vs_ops->del_port(vs_port);

	if (!rc)
		vs_port->state = VSWITCH_PSTATE_NOT_INUSE;


	/*TBD:XXX: may be put a dummy function which frees all packets without
	 * any tx/rx, NULL looks ugly!
	 */
	vs_port->do_tx = vs_port->do_rx = NULL;

	RTE_LOG(DEBUG, VHOST_CONFIG, "Removed port [%d, type %d] from \
			vswitch %s\n", vs_port->port_id, vs_port->type,
			vs_port->vs_dev->name);

	return rc;
}

int vs_learn_port(struct vswitch_port *vs_port, struct rte_mbuf
				**pkts, uint16_t count)
{
	struct vswitch_ops *vs_ops = vs_port->vs_dev->ops;
	int rc;

	rc = vs_ops->learn_port(vs_port, pkts, count);
	if (!rc)
		vs_port->state = VSWITCH_PSTATE_FORWARDING;

	return rc;
}

int vs_unlearn_port(struct vswitch_port *vs_port)
{
	struct vswitch_ops *vs_ops = vs_port->vs_dev->ops;

	vs_ops->unlearn_port(vs_port);
	vs_port->state = VSWITCH_PSTATE_LEARNING;

	return 0;
}


int vs_lookup_n_fwd(struct vswitch_port *vs_port, struct rte_mbuf **pkts,
		    uint16_t count, uint16_t in_rxq)
{
	int rc, i;
	struct vswitch_ops *vs_ops = vs_port->vs_dev->ops;

	rc = vs_ops->lookup_n_fwd(vs_port, pkts, count, in_rxq);

	if (rc) {
		for (i = 0; i < count; i++)
			rte_pktmbuf_free(pkts[i]);
	}

	return rc;
}

int vs_do_broadcast_fwd(struct vswitch_dev *vs_dev,
			struct vswitch_port *in_port, struct rte_mbuf *mbuf)
{
	int i = 0, tx_q;
	struct vswitch_port *dest_port;

	for (i = 0; i < vs_dev->port_count; i++) {
		dest_port = &vs_dev->ports[i];
		if ((in_port->type != dest_port->type) &&
			(in_port->port_id != dest_port->port_id)) {
			if (dest_port->type == VSWITCH_PTYPE_VIRTIO)
				tx_q = VIRTIO_TXQ;
			else
				tx_q = rte_lcore_id();
			dest_port->do_tx(dest_port, tx_q, NULL, &mbuf, 1);
		}
	}

	return 0;
}

struct vswitch_port *vs_sched_rx_port(struct vswitch_dev *vs_dev, enum
				      vswitch_port_type ptype, uint16_t core_id)
{
	struct vswitch_port *vs_port = NULL;
	struct vswitch_ops *vs_ops = vs_dev->ops;

	if (vs_ops->sched_rx_port)
		vs_port = vs_ops->sched_rx_port(vs_dev, ptype, core_id);

	return vs_port;
}

struct vswitch_port *vs_sched_tx_port(struct vswitch_dev *vs_dev, enum
				      vswitch_port_type ptype, uint16_t core_id)
{
	struct vswitch_port *vs_port = NULL;
	struct vswitch_ops *vs_ops = vs_dev->ops;

	if (vs_ops->sched_tx_port)
		vs_port = vs_ops->sched_tx_port(vs_dev, ptype, core_id);

	return vs_port;
}


struct vswitch_dev *vs_register_switch(const char *name, int priv_size,
				int max_ports, struct vswitch_ops *ops)
{
	struct vswitch_dev *vs_dev;
	struct vswitch_port *vs_port;
	int size, i;

	size = priv_size + sizeof(struct vswitch_dev);
	vs_dev = rte_malloc(NULL, size, 64);
	if (!vs_dev) {
		RTE_LOG(DEBUG, VHOST_CONFIG, "Failed to vswitch device\n");
		goto out;
	}

	strncpy(vs_dev->name, name, VSWITCH_NAME_SIZE);
	vs_dev->priv = (void *) (vs_dev + 1);

	size = max_ports * sizeof(struct vswitch_port);
	vs_dev->ports = rte_malloc(NULL, size, 64);
	if (!vs_dev->ports) {
		RTE_LOG(DEBUG, VHOST_CONFIG,
			"Failed allocate %d ports for vswitch %s\n",
			max_ports, vs_dev->name);
		goto out;
	}

	memset(vs_dev->ports, 0, size);
	for (i = 0; i < max_ports; i++)
	{
		vs_port = &vs_dev->ports[i];
		vs_port->state = VSWITCH_PSTATE_NOT_INUSE;
	}

	vs_dev->port_count = max_ports;
	vs_dev->ops = ops;

	rte_spinlock_lock(&vs_mdata_g->lock);
	LIST_INSERT_HEAD(&vs_mdata_g->vswitch_head, vs_dev, list);
	vs_mdata_g->vswitch_dev_count++;
	rte_spinlock_unlock(&vs_mdata_g->lock);

	RTE_LOG(DEBUG, VHOST_CONFIG, "Added vswitch %s, max_ports %d\n",
		vs_dev->name, vs_dev->port_count);

	return vs_dev;

out:
	if (vs_dev && vs_dev->ports)
		rte_free(vs_dev->ports);
	if (vs_dev)
		rte_free(vs_dev);

	return NULL;
}

int  vs_unregister_switch(struct vswitch_dev *vs_dev)
{
	struct vswitch_dev *temp;
	int removed = 0, rc;

	rte_spinlock_lock(&vs_mdata_g->lock);
	LIST_FOREACH(temp, &vs_mdata_g->vswitch_head, list) {
		if (!strncmp(temp->name, vs_dev->name, VSWITCH_NAME_SIZE)){
			LIST_REMOVE(temp, list);
			removed = 1;
		}
	}
	rte_spinlock_unlock(&vs_mdata_g->lock);

	if (!removed) {
		RTE_LOG(DEBUG, VHOST_CONFIG, "vswitch [%s] not found\n",
			vs_dev->name);
		rc = -ENODEV;
	}

	RTE_LOG(DEBUG, VHOST_CONFIG, "Unregistering and freeing vswitch [%s]\n",
		vs_dev->name);

	if (vs_dev->ports)
		rte_free(vs_dev->ports);

	rte_free(vs_dev);

	return rc;
}

int vs_switch_dev_init(struct vswitch_dev *vs_dev, uint16_t conf_flags)
{
	struct vswitch_ops *ops = vs_dev->ops;
	int rc = 0;

	if (ops->switch_init)
		rc = ops->switch_init(vs_dev, conf_flags);

	if (!rc)
		vs_dev->conf_flags = conf_flags;

	return rc;
}

int vs_vswitch_init(void)
{
	int rc = 0;

	vs_mdata_g = rte_malloc(NULL, sizeof(struct vs_mdata), 64);
	if (!vs_mdata_g)
	{
		RTE_LOG(DEBUG, VHOST_CONFIG,
			"Failed to allocate mem for vhost siwtch metadata\n");
		rc = -ENOMEM;
	       goto out;
	}

	memset(vs_mdata_g, 0, sizeof(struct vs_mdata));
	LIST_INIT(&vs_mdata_g->vswitch_head);
	rte_spinlock_init(&vs_mdata_g->lock);

out:
	return rc;
}

