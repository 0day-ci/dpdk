/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <rte_ethdev.h>
#include <rte_ethdev_vdev.h>
#include <rte_malloc.h>
#include <rte_vdev.h>
#include <rte_kvargs.h>
#include <rte_errno.h>

#include "rte_eth_softnic.h"
#include "rte_eth_softnic_internals.h"

#define PMD_PARAM_IFACE_NAME				"iface"
#define PMD_PARAM_IFACE_QUEUE				"txq_id"
#define PMD_PARAM_DEQ_BSZ					"deq_bsz"

static const char *pmd_valid_args[] = {
	PMD_PARAM_IFACE_NAME,
	PMD_PARAM_IFACE_QUEUE,
	PMD_PARAM_DEQ_BSZ,
	NULL
};

static struct rte_vdev_driver pmd_drv;

static int
pmd_eth_dev_configure(struct rte_eth_dev *dev)
{
	struct pmd_internals *p = dev->data->dev_private;
	int status;

	/* Stop the underlay device */
	rte_eth_dev_stop(p->uport_id);

	/* Call the current function for the underlay device */
	status = rte_eth_dev_configure(p->uport_id,
		dev->data->nb_rx_queues,
		dev->data->nb_tx_queues,
		&dev->data->dev_conf);
	if (status)
		return status;

	/* Rework on the RX queues of the overlay device */
	if (dev->data->rx_queues)
		rte_free(dev->data->rx_queues);
	dev->data->rx_queues = p->udev->data->rx_queues;

	return 0;
}

static int
pmd_eth_dev_tx_queue_setup(struct rte_eth_dev *dev,
	uint16_t tx_queue_id,
	uint16_t nb_tx_desc,
	unsigned int socket_id,
	const struct rte_eth_txconf *tx_conf)
{
	struct pmd_internals *p = dev->data->dev_private;
	int status;

	/* Call the current function for the underlay device */
	status = rte_eth_tx_queue_setup(p->uport_id,
		tx_queue_id,
		nb_tx_desc,
		socket_id,
		tx_conf);
	if (status)
		return status;

	/* Handle TX queue of the overlay device */
	dev->data->tx_queues[tx_queue_id] = (void *) p;

	return 0;
}

static int
pmd_eth_dev_start(struct rte_eth_dev *dev)
{
	struct pmd_internals *p = dev->data->dev_private;

	/* Clone dev->data from underlay to overlay */
	memcpy(dev->data->mac_pool_sel,
		p->udev->data->mac_pool_sel,
		sizeof(dev->data->mac_pool_sel));
	dev->data->promiscuous = p->udev->data->promiscuous;
	dev->data->all_multicast = p->udev->data->all_multicast;

	/* Call the current function for the underlay device */
	return rte_eth_dev_start(p->uport_id);
}

static void
pmd_eth_dev_stop(struct rte_eth_dev *dev)
{
	struct pmd_internals *p = dev->data->dev_private;

	/* Call the current function for the underlay device */
	rte_eth_dev_stop(p->uport_id);

}

static void
pmd_eth_dev_close(struct rte_eth_dev *dev)
{
	struct pmd_internals *p = dev->data->dev_private;

	/* Call the current function for the underlay device */
	rte_eth_dev_close(p->uport_id);

	/* Cleanup on the overlay device */
	dev->data->rx_queues = NULL;
	dev->data->tx_queues = NULL;

	return;
}

static void
pmd_eth_dev_promiscuous_enable(struct rte_eth_dev *dev)
{
	struct pmd_internals *p = dev->data->dev_private;

	/* Call the current function for the underlay device */
	rte_eth_promiscuous_enable(p->uport_id);
}

static void
pmd_eth_dev_promiscuous_disable(struct rte_eth_dev *dev)
{
	struct pmd_internals *p = dev->data->dev_private;

	/* Call the current function for the underlay device */
	rte_eth_promiscuous_disable(p->uport_id);
}

static void
pmd_eth_dev_all_multicast_enable(struct rte_eth_dev *dev)
{
	struct pmd_internals *p = dev->data->dev_private;

	/* Call the current function for the underlay device */
	rte_eth_allmulticast_enable(p->uport_id);
}

static void
pmd_eth_dev_all_multicast_disable(struct rte_eth_dev *dev)
{
	struct pmd_internals *p = dev->data->dev_private;

	/* Call the current function for the underlay device */
	rte_eth_allmulticast_disable(p->uport_id);
}

static int
pmd_eth_dev_link_update(struct rte_eth_dev *dev,
	int wait_to_complete __rte_unused)
{
	struct pmd_internals *p = dev->data->dev_private;
	struct rte_eth_link dev_link;

	/* Call the current function for the underlay device */
	rte_eth_link_get(p->uport_id, &dev_link);

	/* Overlay device update */
	dev->data->dev_link = dev_link;

	return 0;
}

static int
pmd_eth_dev_mtu_set(struct rte_eth_dev *dev, uint16_t mtu)
{
	struct pmd_internals *p = dev->data->dev_private;
	int status;

	/* Call the current function for the underlay device */
	status = rte_eth_dev_set_mtu(p->uport_id, mtu);
	if (status)
		return status;

	/* Overlay device update */
	dev->data->mtu = mtu;

	return 0;
}

static void
pmd_eth_dev_mac_addr_set(struct rte_eth_dev *dev,
	struct ether_addr *mac_addr)
{
	struct pmd_internals *p = dev->data->dev_private;

	/* Call the current function for the underlay device */
	rte_eth_dev_default_mac_addr_set(p->uport_id, mac_addr);
}

static int
pmd_eth_dev_mac_addr_add(struct rte_eth_dev *dev,
	struct ether_addr *mac_addr,
	uint32_t index __rte_unused,
	uint32_t vmdq)
{
	struct pmd_internals *p = dev->data->dev_private;

	/* Call the current function for the underlay device */
	return rte_eth_dev_mac_addr_add(p->uport_id, mac_addr, vmdq);
}

static void
pmd_eth_dev_mac_addr_remove(struct rte_eth_dev *dev, uint32_t index)
{
	struct pmd_internals *p = dev->data->dev_private;

	/* Call the current function for the underlay device */
	rte_eth_dev_mac_addr_remove(p->uport_id, &dev->data->mac_addrs[index]);
}

static uint16_t
pmd_eth_dev_tx_burst(void *txq,
	struct rte_mbuf **tx_pkts,
	uint16_t nb_pkts)
{
	struct pmd_internals *p = txq;

	return rte_eth_tx_burst(p->uport_id, p->txq_id, tx_pkts, nb_pkts);

}

int
rte_eth_softnic_run(uint8_t port_id __rte_unused)
{
	return 0;
}

static void
pmd_ops_build(struct eth_dev_ops *o, const struct eth_dev_ops *u)
{
	/* Inherited functionality */
	pmd_ops_inherit(o, u);

	/* Derived functionality */
	o->dev_configure = pmd_eth_dev_configure;
	o->tx_queue_setup = pmd_eth_dev_tx_queue_setup;
	o->dev_start = pmd_eth_dev_start;
	o->dev_stop = pmd_eth_dev_stop;
	o->dev_close = pmd_eth_dev_close;
	o->promiscuous_enable = pmd_eth_dev_promiscuous_enable;
	o->promiscuous_disable = pmd_eth_dev_promiscuous_disable;
	o->allmulticast_enable = pmd_eth_dev_all_multicast_enable;
	o->allmulticast_disable = pmd_eth_dev_all_multicast_disable;
	o->link_update = pmd_eth_dev_link_update;
	o->mtu_set = pmd_eth_dev_mtu_set;
	o->mac_addr_set = pmd_eth_dev_mac_addr_set;
	o->mac_addr_add = pmd_eth_dev_mac_addr_add;
	o->mac_addr_remove = pmd_eth_dev_mac_addr_remove;
}

int
rte_eth_softnic_create(struct rte_eth_softnic_params *params)
{
	struct rte_eth_dev_info uinfo;
	struct rte_eth_dev *odev, *udev;
	struct rte_eth_dev_data *odata, *udata;
	struct eth_dev_ops *odev_ops;
	const struct eth_dev_ops *udev_ops;
	void **otx_queues;
	struct pmd_internals *p;
	int numa_node;
	uint8_t oport_id, uport_id;

	/* Check input arguments */
	if ((params == NULL) ||
		(params->oname == NULL) ||
		(params->uname == NULL) ||
		(params->deq_bsz > RTE_ETH_SOFTNIC_DEQ_BSZ_MAX))
		return -EINVAL;

	if (rte_eth_dev_get_port_by_name(params->uname, &uport_id))
		return -EINVAL;
	udev = &rte_eth_devices[uport_id];
	udata = udev->data;
	udev_ops = udev->dev_ops;
	numa_node = udata->numa_node;

	rte_eth_dev_info_get(uport_id, &uinfo);
	if (params->txq_id >= uinfo.max_tx_queues)
		return -EINVAL;

	RTE_LOG(INFO, PMD, "Creating overlay device %s for underlay device %s\n",
		params->oname, params->uname);

	/* Overlay device ethdev entry: entry allocation */
	odev = rte_eth_dev_allocate(params->oname);
	if (!odev)
		return -ENOMEM;
	oport_id = odev - rte_eth_devices;

	/* Overlay device ethdev entry: memory allocation */
	odev_ops = rte_zmalloc_socket(params->oname,
		sizeof(*odev_ops), 0, numa_node);
	if (!odev_ops) {
		rte_eth_dev_release_port(odev);
		return -ENOMEM;
	}
	odev->dev_ops = odev_ops;

	odata = rte_zmalloc_socket(params->oname,
		sizeof(*odata), 0, numa_node);
	if (!odata) {
		rte_free(odev_ops);
		rte_eth_dev_release_port(odev);
		return -ENOMEM;
	}
	memmove(odata->name, odev->data->name, sizeof(odata->name));
	odata->port_id = odev->data->port_id;
	odata->mtu = odev->data->mtu;
	odev->data = odata;

	otx_queues = rte_zmalloc_socket(params->oname,
		uinfo.max_tx_queues * sizeof(void *), 0, numa_node);
	if (!otx_queues) {
		rte_free(odata);
		rte_free(odev_ops);
		rte_eth_dev_release_port(odev);
		return -ENOMEM;
	}
	odev->data->tx_queues = otx_queues;

	p = rte_zmalloc_socket(params->oname,
		sizeof(struct pmd_internals), 0, numa_node);
	if (!p) {
		rte_free(otx_queues);
		rte_free(odata);
		rte_free(odev_ops);
		rte_eth_dev_release_port(odev);
		return -ENOMEM;
	}
	odev->data->dev_private = p;

	/* Overlay device ethdev entry: fill in dev */
	odev->rx_pkt_burst = udev->rx_pkt_burst;
	odev->tx_pkt_burst = pmd_eth_dev_tx_burst;
	odev->tx_pkt_prepare = udev->tx_pkt_prepare;

	/* Overlay device ethdev entry: fill in dev->data */
	odev->data->dev_link = udev->data->dev_link;
	odev->data->mtu = udev->data->mtu;
	odev->data->min_rx_buf_size = udev->data->min_rx_buf_size;
	odev->data->mac_addrs = udev->data->mac_addrs;
	odev->data->hash_mac_addrs = udev->data->hash_mac_addrs;
	odev->data->promiscuous = udev->data->promiscuous;
	odev->data->all_multicast = udev->data->all_multicast;
	odev->data->dev_flags = udev->data->dev_flags;
	odev->data->kdrv = RTE_KDRV_NONE;
	odev->data->numa_node = numa_node;
	odev->data->drv_name = pmd_drv.driver.name;

	/* Overlay device ethdev entry: fill in dev->dev_ops */
	pmd_ops_build(odev_ops, udev_ops);

	/* Overlay device ethdev entry: fill in dev->data->dev_private */
	p->odev = odev;
	p->udev = udev;
	p->odata = odata;
	p->udata = udata;
	p->odev_ops = odev_ops;
	p->udev_ops = udev_ops;
	p->oport_id = oport_id;
	p->uport_id = uport_id;
	p->deq_bsz = params->deq_bsz;
	p->txq_id = params->txq_id;

	return 0;
}

static int
get_string_arg(const char *key __rte_unused,
               const char *value, void *extra_args)
{
	if (!value || !extra_args)
		return -EINVAL;

	*(char **)extra_args = strdup(value);

	if (!*(char **)extra_args)
		return -ENOMEM;

        return 0;
}

static int
get_int_arg(const char *key __rte_unused,
                const char *value, void *extra_args)
{
	if (!value || !extra_args)
		return -EINVAL;

	*(uint32_t *)extra_args = strtoull(value, NULL, 0);

	return 0;
}

static int
pmd_probe(struct rte_vdev_device *dev)
{
	struct rte_eth_softnic_params p;
	const char *params;
	struct rte_kvargs *kvlist;
	int ret;

	if (!dev)
		return -EINVAL;

	RTE_LOG(INFO, PMD, "Probing device %s\n", rte_vdev_device_name(dev));

	params = rte_vdev_device_args(dev);
	if(!params)
		return -EINVAL;

	kvlist = rte_kvargs_parse(params, pmd_valid_args);
	if (kvlist == NULL)
		return -EINVAL;

	p.oname = rte_vdev_device_name(dev);

	/* Interface: Mandatory */
	if (rte_kvargs_count(kvlist, PMD_PARAM_IFACE_NAME) != 1) {
		ret = -EINVAL;
		goto out_free;
	}
	ret = rte_kvargs_process(kvlist, PMD_PARAM_IFACE_NAME, &get_string_arg,
		&p.uname);
	if (ret < 0)
		goto out_free;

	/* Interface Queue ID: Optional */
	if (rte_kvargs_count(kvlist, PMD_PARAM_IFACE_QUEUE) == 1) {
		ret = rte_kvargs_process(kvlist, PMD_PARAM_IFACE_QUEUE, &get_int_arg,
			&p.txq_id);
		if (ret < 0)
			goto out_free;
	} else
		p.txq_id = RTE_ETH_SOFTNIC_TXQ_ID_DEFAULT;

	/* Dequeue Burst Size: Optional */
	if (rte_kvargs_count(kvlist, PMD_PARAM_DEQ_BSZ) == 1) {
		ret = rte_kvargs_process(kvlist, PMD_PARAM_DEQ_BSZ,
			&get_int_arg, &p.deq_bsz);
		if (ret < 0)
			goto out_free;
	} else
		p.deq_bsz = RTE_ETH_SOFTNIC_DEQ_BSZ_DEFAULT;

	return rte_eth_softnic_create(&p);

out_free:
	rte_kvargs_free(kvlist);
	return ret;
}

static int
pmd_remove(struct rte_vdev_device *dev)
{
	struct rte_eth_dev *eth_dev = NULL;
	struct pmd_internals *p;
	struct eth_dev_ops *dev_ops;

	if (!dev)
		return -EINVAL;

	RTE_LOG(INFO, PMD, "Removing device %s\n", rte_vdev_device_name(dev));

	/* Find the ethdev entry */
	eth_dev = rte_eth_dev_allocated(rte_vdev_device_name(dev));
	if (eth_dev == NULL)
		return -ENODEV;
	p = eth_dev->data->dev_private;
	dev_ops = p->odev_ops;

	pmd_eth_dev_stop(eth_dev);

	/* Free device data structures*/
	rte_free(eth_dev->data->dev_private);
	rte_free(eth_dev->data->tx_queues);
	rte_free(eth_dev->data);
	rte_free(dev_ops);
	rte_eth_dev_release_port(eth_dev);

	return 0;
}

static struct rte_vdev_driver pmd_drv = {
	.probe = pmd_probe,
	.remove = pmd_remove,
};

RTE_PMD_REGISTER_VDEV(net_softnic, pmd_drv);
RTE_PMD_REGISTER_PARAM_STRING(net_softnic,
	PMD_PARAM_IFACE_NAME "=<string> "
	PMD_PARAM_IFACE_QUEUE "=<int> "
	PMD_PARAM_DEQ_BSZ "=<int>");
