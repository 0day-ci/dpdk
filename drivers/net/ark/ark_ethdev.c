/*-
 * BSD LICENSE
 *
 * Copyright (c) 2015-2017 Atomic Rules LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * * Neither the name of copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <unistd.h>
#include <sys/stat.h>
#include <dlfcn.h>

#include <rte_kvargs.h>
#include <rte_vdev.h>

#include "ark_global.h"
#include "ark_debug.h"
#include "ark_ethdev.h"
#include "ark_ethdev_tx.h"
#include "ark_ethdev_rx.h"
#include "ark_mpu.h"
#include "ark_ddm.h"
#include "ark_udm.h"
#include "ark_rqp.h"
#include "ark_pktdir.h"
#include "ark_pktgen.h"
#include "ark_pktchkr.h"

/*  Internal prototypes */
static int eth_ark_check_args(const char *params);
static int eth_ark_dev_init(struct rte_eth_dev *dev);
static int ark_config_device(struct rte_eth_dev *dev);
static int eth_ark_dev_uninit(struct rte_eth_dev *eth_dev);
static int eth_ark_dev_configure(struct rte_eth_dev *dev);
static int eth_ark_dev_start(struct rte_eth_dev *dev);
static void eth_ark_dev_stop(struct rte_eth_dev *dev);
static void eth_ark_dev_close(struct rte_eth_dev *dev);
static void eth_ark_dev_info_get(struct rte_eth_dev *dev,
				 struct rte_eth_dev_info *dev_info);
static int eth_ark_dev_link_update(struct rte_eth_dev *dev,
				   int wait_to_complete);
static int eth_ark_dev_set_link_up(struct rte_eth_dev *dev);
static int eth_ark_dev_set_link_down(struct rte_eth_dev *dev);
static void eth_ark_dev_stats_get(struct rte_eth_dev *dev,
				  struct rte_eth_stats *stats);
static void eth_ark_dev_stats_reset(struct rte_eth_dev *dev);
static void eth_ark_set_default_mac_addr(struct rte_eth_dev *dev,
					 struct ether_addr *mac_addr);
static void eth_ark_macaddr_add(struct rte_eth_dev *dev,
				struct ether_addr *mac_addr,
				uint32_t index,
				uint32_t pool);
static void eth_ark_macaddr_remove(struct rte_eth_dev *dev,
				   uint32_t index);

#define ARK_DEV_TO_PCI(eth_dev)			\
	RTE_DEV_TO_PCI((eth_dev)->device)

#define ARK_MAX_ARG_LEN 256
static uint32_t pkt_dir_v;
static char pkt_gen_args[ARK_MAX_ARG_LEN];
static char pkt_chkr_args[ARK_MAX_ARG_LEN];

#define ARK_PKTGEN_ARG "Pkt_gen"
#define ARK_PKTCHKR_ARG "Pkt_chkr"
#define ARK_PKTDIR_ARG "Pkt_dir"

static const char * const valid_arguments[] = {
	ARK_PKTGEN_ARG,
	ARK_PKTCHKR_ARG,
	ARK_PKTDIR_ARG,
	"iface",
	NULL
};

#define MAX_ARK_PHYS 16
struct ark_adapter *gark[MAX_ARK_PHYS];

static const struct rte_pci_id pci_id_ark_map[] = {
	{RTE_PCI_DEVICE(0x1d6c, 0x100d)},
	{RTE_PCI_DEVICE(0x1d6c, 0x100e)},
	{.vendor_id = 0, /* sentinel */ },
};

static struct eth_driver rte_ark_pmd = {
	.pci_drv = {
		.probe = rte_eth_dev_pci_probe,
		.remove = rte_eth_dev_pci_remove,
		.id_table = pci_id_ark_map,
		.drv_flags = RTE_PCI_DRV_NEED_MAPPING | RTE_PCI_DRV_INTR_LSC
	},
	.eth_dev_init = eth_ark_dev_init,
	.eth_dev_uninit = eth_ark_dev_uninit,
	.dev_private_size = sizeof(struct ark_adapter),
};

static const struct eth_dev_ops ark_eth_dev_ops = {
	.dev_configure = eth_ark_dev_configure,
	.dev_start = eth_ark_dev_start,
	.dev_stop = eth_ark_dev_stop,
	.dev_close = eth_ark_dev_close,

	.dev_infos_get = eth_ark_dev_info_get,

	.rx_queue_setup = eth_ark_dev_rx_queue_setup,
	.rx_queue_count = eth_ark_dev_rx_queue_count,
	.tx_queue_setup = eth_ark_tx_queue_setup,

	.link_update = eth_ark_dev_link_update,
	.dev_set_link_up = eth_ark_dev_set_link_up,
	.dev_set_link_down = eth_ark_dev_set_link_down,

	.rx_queue_start = eth_ark_rx_start_queue,
	.rx_queue_stop = eth_ark_rx_stop_queue,

	.tx_queue_start = eth_ark_tx_queue_start,
	.tx_queue_stop = eth_ark_tx_queue_stop,

	.stats_get = eth_ark_dev_stats_get,
	.stats_reset = eth_ark_dev_stats_reset,

	.mac_addr_add = eth_ark_macaddr_add,
	.mac_addr_remove = eth_ark_macaddr_remove,
	.mac_addr_set = eth_ark_set_default_mac_addr,

};

int
ark_get_port_id(struct rte_eth_dev *dev, struct ark_adapter *ark)
{
	int n = ark->num_ports;
	int i;

	/* There has to be a smarter way to do this ... */
	for (i = 0; i < n; i++) {
		if (ark->port[i].eth_dev == dev)
			return i;
	}
	ARK_DEBUG_TRACE("ARK: Device is NOT associated with a port !!");
	return -1;
}

static
int
check_for_ext(struct rte_eth_dev *dev __rte_unused,
	      struct ark_adapter *ark __rte_unused)
{
	int found = 0;

	/* Get the env */
	const char *dllpath = getenv("ARK_EXT_PATH");

	if (dllpath == NULL) {
		ARK_DEBUG_TRACE("ARK EXT NO dll path specified\n");
		return 0;
	}
	ARK_DEBUG_TRACE("ARK EXT found dll path at %s\n", dllpath);

	/* Open and load the .so */
	ark->d_handle = dlopen(dllpath, RTLD_LOCAL | RTLD_LAZY);
	if (ark->d_handle == NULL)
		PMD_DRV_LOG(ERR, "Could not load user extension %s\n", dllpath);
	else
		ARK_DEBUG_TRACE("SUCCESS: loaded user extension %s\n", dllpath);

	/* Get the entry points */
	ark->user_ext.dev_init =
		(void *(*)(struct rte_eth_dev *, void *, int))
		dlsym(ark->d_handle, "dev_init");
	ARK_DEBUG_TRACE("device ext init pointer = %p\n",
			ark->user_ext.dev_init);
	ark->user_ext.dev_get_port_count =
		(int (*)(struct rte_eth_dev *, void *))
		dlsym(ark->d_handle, "dev_get_port_count");
	ark->user_ext.dev_uninit =
		(void (*)(struct rte_eth_dev *, void *))
		dlsym(ark->d_handle, "dev_uninit");
	ark->user_ext.dev_configure =
		(int (*)(struct rte_eth_dev *, void *))
		dlsym(ark->d_handle, "dev_configure");
	ark->user_ext.dev_start =
		(int (*)(struct rte_eth_dev *, void *))
		dlsym(ark->d_handle, "dev_start");
	ark->user_ext.dev_stop =
		(void (*)(struct rte_eth_dev *, void *))
		dlsym(ark->d_handle, "dev_stop");
	ark->user_ext.dev_close =
		(void (*)(struct rte_eth_dev *, void *))
		dlsym(ark->d_handle, "dev_close");
	ark->user_ext.link_update =
		(int (*)(struct rte_eth_dev *, int, void *))
		dlsym(ark->d_handle, "link_update");
	ark->user_ext.dev_set_link_up =
		(int (*)(struct rte_eth_dev *, void *))
		dlsym(ark->d_handle, "dev_set_link_up");
	ark->user_ext.dev_set_link_down =
		(int (*)(struct rte_eth_dev *, void *))
		dlsym(ark->d_handle, "dev_set_link_down");
	ark->user_ext.stats_get =
		(void (*)(struct rte_eth_dev *, struct rte_eth_stats *,
			  void *))
		dlsym(ark->d_handle, "stats_get");
	ark->user_ext.stats_reset =
		(void (*)(struct rte_eth_dev *, void *))
		dlsym(ark->d_handle, "stats_reset");
	ark->user_ext.mac_addr_add =
		(void (*)(struct rte_eth_dev *, struct ether_addr *, uint32_t,
			  uint32_t, void *))
		dlsym(ark->d_handle, "mac_addr_add");
	ark->user_ext.mac_addr_remove =
		(void (*)(struct rte_eth_dev *, uint32_t, void *))
		dlsym(ark->d_handle, "mac_addr_remove");
	ark->user_ext.mac_addr_set =
		(void (*)(struct rte_eth_dev *, struct ether_addr *,
			  void *))
		dlsym(ark->d_handle, "mac_addr_set");

	return found;
}

static int
eth_ark_dev_init(struct rte_eth_dev *dev)
{
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;
	struct rte_pci_device *pci_dev;
	int ret;

	ark->eth_dev = dev;

	ARK_DEBUG_TRACE("eth_ark_dev_init(struct rte_eth_dev *dev)\n");
	gark[0] = ark;

	/* Check to see if there is an extension that we need to load */
	check_for_ext(dev, ark);
	pci_dev = ARK_DEV_TO_PCI(dev);
	rte_eth_copy_pci_info(dev, pci_dev);

	if (pci_dev->device.devargs)
		eth_ark_check_args(pci_dev->device.devargs->args);
	else
		PMD_DRV_LOG(INFO, "No Device args found\n");

	/* Use dummy function until setup */
	dev->rx_pkt_burst = &eth_ark_recv_pkts_noop;
	dev->tx_pkt_burst = &eth_ark_xmit_pkts_noop;

	ark->bar0 = (uint8_t *)pci_dev->mem_resource[0].addr;
	ark->a_bar = (uint8_t *)pci_dev->mem_resource[2].addr;

	ark->sysctrl.v  = (void *)&ark->bar0[ARK_SYSCTRL_BASE];
	ark->mpurx.v  = (void *)&ark->bar0[ARK_MPU_RX_BASE];
	ark->udm.v  = (void *)&ark->bar0[ARK_UDM_BASE];
	ark->mputx.v  = (void *)&ark->bar0[ARK_MPU_TX_BASE];
	ark->ddm.v  = (void *)&ark->bar0[ARK_DDM_BASE];
	ark->cmac.v  = (void *)&ark->bar0[ARK_CMAC_BASE];
	ark->external.v  = (void *)&ark->bar0[ARK_EXTERNAL_BASE];
	ark->pktdir.v  = (void *)&ark->bar0[ARK_PKTDIR_BASE];
	ark->pktgen.v  = (void *)&ark->bar0[ARK_PKTGEN_BASE];
	ark->pktchkr.v  = (void *)&ark->bar0[ARK_PKTCHKR_BASE];

	ark->rqpacing =
		(struct ark_rqpace_t *)(ark->bar0 + ARK_RCPACING_BASE);
	ark->started = 0;

	ARK_DEBUG_TRACE
		("Sys Ctrl Const = 0x%x  HW Commit_ID: %08x\n",
		 ark->sysctrl.t32[4],
		 rte_be_to_cpu_32(ark->sysctrl.t32[0x20 / 4]));
	PMD_DRV_LOG(INFO, "ARKP PMD  HW Commit_ID: %08x\n",
		    rte_be_to_cpu_32(ark->sysctrl.t32[0x20 / 4]));

	/* If HW sanity test fails, return an error */
	if (ark->sysctrl.t32[4] != 0xcafef00d) {
		PMD_DRV_LOG(ERR,
			    "HW Sanity test has failed, expected constant"
			    " 0x%x, read 0x%x (%s)\n",
			    0xcafef00d,
			    ark->sysctrl.t32[4], __func__);
		return -1;
	}

	PMD_DRV_LOG(INFO,
		    "HW Sanity test has PASSED, expected constant"
		    " 0x%x, read 0x%x (%s)\n",
		    0xcafef00d, ark->sysctrl.t32[4], __func__);

	/* We are a single function multi-port device. */
	const unsigned int numa_node = rte_socket_id();
	struct ether_addr adr;

	ret = ark_config_device(dev);
	dev->dev_ops = &ark_eth_dev_ops;

	dev->data->mac_addrs = rte_zmalloc("ark", ETHER_ADDR_LEN, 0);
	if (!dev->data->mac_addrs) {
		PMD_DRV_LOG(ERR,
			    "Failed to allocated memory for storing mac address"
			    );
	}
	ether_addr_copy((struct ether_addr *)&adr, &dev->data->mac_addrs[0]);

	if (ark->user_ext.dev_init) {
		ark->user_data = ark->user_ext.dev_init(dev, ark->a_bar, 0);
		if (!ark->user_data) {
			PMD_DRV_LOG(INFO,
				    "Failed to initialize PMD extension!"
				    " continuing without it\n");
			memset(&ark->user_ext, 0, sizeof(struct ark_user_ext));
			dlclose(ark->d_handle);
		}
	}

	/*
	 * We will create additional devices based on the number of requested
	 * ports
	 */
	int pc = 1;
	int p;

	if (ark->user_ext.dev_get_port_count) {
		pc = ark->user_ext.dev_get_port_count(dev, ark->user_data);
		ark->num_ports = pc;
	} else {
		ark->num_ports = 1;
	}
	for (p = 0; p < pc; p++) {
		struct ark_port *port;

		port = &ark->port[p];
		struct rte_eth_dev_data *data = NULL;

		port->id = p;

		char name[RTE_ETH_NAME_MAX_LEN];

		snprintf(name, sizeof(name), "arketh%d",
			 dev->data->port_id + p);

		if (p == 0) {
			/* First port is already allocated by DPDK */
			port->eth_dev = ark->eth_dev;
			continue;
		}

		/* reserve an ethdev entry */
		port->eth_dev = rte_eth_dev_allocate(name);
		if (!port->eth_dev) {
			PMD_DRV_LOG(ERR,
				    "Could not allocate eth_dev for port %d\n",
				    p);
			goto error;
		}

		data = rte_zmalloc_socket(name, sizeof(*data), 0, numa_node);
		if (!data) {
			PMD_DRV_LOG(ERR,
				    "Could not allocate eth_dev for port %d\n",
				    p);
			goto error;
		}
		data->port_id = ark->eth_dev->data->port_id + p;
		port->eth_dev->data = data;
		port->eth_dev->device = &pci_dev->device;
		port->eth_dev->data->dev_private = ark;
		port->eth_dev->driver = ark->eth_dev->driver;
		port->eth_dev->dev_ops = ark->eth_dev->dev_ops;
		port->eth_dev->tx_pkt_burst = ark->eth_dev->tx_pkt_burst;
		port->eth_dev->rx_pkt_burst = ark->eth_dev->rx_pkt_burst;

		rte_eth_copy_pci_info(port->eth_dev, pci_dev);

		port->eth_dev->data->mac_addrs =
			rte_zmalloc(name, ETHER_ADDR_LEN, 0);
		if (!port->eth_dev->data->mac_addrs) {
			PMD_DRV_LOG(ERR,
				    "Memory allocation for MAC failed!"
				    " Exiting.\n");
			goto error;
		}
		ether_addr_copy(&adr,
				&port->eth_dev->data->mac_addrs[0]);

		if (ark->user_ext.dev_init)
			ark->user_data =
				ark->user_ext.dev_init(dev, ark->a_bar, p);
	}

	return ret;

 error:
	if (dev->data->mac_addrs)
		rte_free(dev->data->mac_addrs);

	for (p = 0; p < pc; p++) {
		if (ark->port[p].eth_dev->data)
			rte_free(ark->port[p].eth_dev->data);
		if (ark->port[p].eth_dev->data->mac_addrs)
			rte_free(ark->port[p].eth_dev->data->mac_addrs);
	}

	return -1;
}

/*
 *Initial device configuration when device is opened
 * setup the DDM, and UDM
 * Called once per PCIE device
 */
static int
ark_config_device(struct rte_eth_dev *dev)
{
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;
	uint16_t num_q, i;
	struct ark_mpu_t *mpu;

	/*
	 * Make sure that the packet director, generator and checker are in a
	 * known state
	 */
	ark->start_pg = 0;
	ark->pg = ark_pktgen_init(ark->pktgen.v, 0, 1);
	ark_pktgen_reset(ark->pg);
	ark->pc = ark_pktchkr_init(ark->pktchkr.v, 0, 1);
	ark_pktchkr_stop(ark->pc);
	ark->pd = ark_pktdir_init(ark->pktdir.v);

	/* Verify HW */
	if (ark_udm_verify(ark->udm.v))
		return -1;
	if (ark_ddm_verify(ark->ddm.v))
		return -1;

	/* UDM */
	if (ark_udm_reset(ark->udm.v)) {
		PMD_DRV_LOG(ERR, "Unable to stop and reset UDM\n");
		return -1;
	}
	/* Keep in reset until the MPU are cleared */

	/* MPU reset */
	mpu = ark->mpurx.v;
	num_q = ark_api_num_queues(mpu);
	ark->rx_queues = num_q;
	for (i = 0; i < num_q; i++) {
		ark_mpu_reset(mpu);
		mpu = RTE_PTR_ADD(mpu, ARK_MPU_QOFFSET);
	}

	ark_udm_stop(ark->udm.v, 0);
	ark_udm_configure(ark->udm.v,
			  RTE_PKTMBUF_HEADROOM,
			  RTE_MBUF_DEFAULT_DATAROOM,
			  ARK_RX_WRITE_TIME_NS);
	ark_udm_stats_reset(ark->udm.v);
	ark_udm_stop(ark->udm.v, 0);

	/* TX -- DDM */
	if (ark_ddm_stop(ark->ddm.v, 1))
		PMD_DRV_LOG(ERR, "Unable to stop DDM\n");

	mpu = ark->mputx.v;
	num_q = ark_api_num_queues(mpu);
	ark->tx_queues = num_q;
	for (i = 0; i < num_q; i++) {
		ark_mpu_reset(mpu);
		mpu = RTE_PTR_ADD(mpu, ARK_MPU_QOFFSET);
	}

	ark_ddm_reset(ark->ddm.v);
	ark_ddm_stats_reset(ark->ddm.v);

	ark_ddm_stop(ark->ddm.v, 0);
	ark_rqp_stats_reset(ark->rqpacing);

	return 0;
}

static int
eth_ark_dev_uninit(struct rte_eth_dev *dev)
{
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;

	if (rte_eal_process_type() != RTE_PROC_PRIMARY)
		return 0;

	if (ark->user_ext.dev_uninit)
		ark->user_ext.dev_uninit(dev, ark->user_data);

	ark_pktgen_uninit(ark->pg);
	ark_pktchkr_uninit(ark->pc);

	dev->dev_ops = NULL;
	dev->rx_pkt_burst = NULL;
	dev->tx_pkt_burst = NULL;
	if (dev->data->mac_addrs)
		rte_free(dev->data->mac_addrs);
	if (dev->data)
		rte_free(dev->data);

	return 0;
}

static int
eth_ark_dev_configure(struct rte_eth_dev *dev __rte_unused)
{
	ARK_DEBUG_TRACE("ARKP: In %s\n", __func__);
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;

	eth_ark_dev_set_link_up(dev);
	if (ark->user_ext.dev_configure)
		return ark->user_ext.dev_configure(dev, ark->user_data);
	return 0;
}

static void *
delay_pg_start(void *arg)
{
	struct ark_adapter *ark = (struct ark_adapter *)arg;

	/* This function is used exclusively for regression testing, We
	 * perform a blind sleep here to ensure that the external test
	 * application has time to setup the test before we generate packets
	 */
	usleep(100000);
	ark_pktgen_run(ark->pg);
	return NULL;
}

static int
eth_ark_dev_start(struct rte_eth_dev *dev)
{
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;
	int i;

	ARK_DEBUG_TRACE("ARKP: In eth_ark_dev_start\n");

	/* RX Side */
	/* start UDM */
	ark_udm_start(ark->udm.v);

	for (i = 0; i < dev->data->nb_rx_queues; i++)
		eth_ark_rx_start_queue(dev, i);

	/* TX Side */
	for (i = 0; i < dev->data->nb_tx_queues; i++)
		eth_ark_tx_queue_start(dev, i);

	/* start DDM */
	ark_ddm_start(ark->ddm.v);

	ark->started = 1;
	/* set xmit and receive function */
	dev->rx_pkt_burst = &eth_ark_recv_pkts;
	dev->tx_pkt_burst = &eth_ark_xmit_pkts;

	if (ark->start_pg)
		ark_pktchkr_run(ark->pc);

	if (ark->start_pg && (ark_get_port_id(dev, ark) == 0)) {
		pthread_t thread;

		/* Delay packet generatpr start allow the hardware to be ready
		 * This is only used for sanity checking with internal generator
		 */
		pthread_create(&thread, NULL, delay_pg_start, ark);
	}

	if (ark->user_ext.dev_start)
		ark->user_ext.dev_start(dev, ark->user_data);

	return 0;
}

static void
eth_ark_dev_stop(struct rte_eth_dev *dev)
{
	uint16_t i;
	int status;
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;
	struct ark_mpu_t *mpu;

	ARK_DEBUG_TRACE("ARKP: In eth_ark_dev_stop\n");

	if (ark->started == 0)
		return;
	ark->started = 0;

	/* Stop the extension first */
	if (ark->user_ext.dev_stop)
		ark->user_ext.dev_stop(dev, ark->user_data);

	/* Stop the packet generator */
	if (ark->start_pg)
		ark_pktgen_pause(ark->pg);

	dev->rx_pkt_burst = &eth_ark_recv_pkts_noop;
	dev->tx_pkt_burst = &eth_ark_xmit_pkts_noop;

	/* STOP TX Side */
	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		status = eth_ark_tx_queue_stop(dev, i);
		if (status != 0) {
			uint8_t port = dev->data->port_id;
			PMD_DRV_LOG(ERR,
				    "ARKP tx_queue stop anomaly"
				    " port %u, queue %u\n",
				    port, i);
		}
	}

	/* Stop DDM */
	/* Wait up to 0.1 second.  each stop is upto 1000 * 10 useconds */
	for (i = 0; i < 10; i++) {
		status = ark_ddm_stop(ark->ddm.v, 1);
		if (status == 0)
			break;
	}
	if (status || i != 0) {
		PMD_DRV_LOG(ERR, "DDM stop anomaly. status:"
			    " %d iter: %u. (%s)\n",
			    status,
			    i,
			    __func__);
		ark_ddm_dump(ark->ddm.v, "Stop anomaly");

		mpu = ark->mputx.v;
		for (i = 0; i < ark->tx_queues; i++) {
			ark_mpu_dump(mpu, "DDM failure dump", i);
			mpu = RTE_PTR_ADD(mpu, ARK_MPU_QOFFSET);
		}
	}

	/* STOP RX Side */
	/* Stop UDM  multiple tries attempted */
	for (i = 0; i < 10; i++) {
		status = ark_udm_stop(ark->udm.v, 1);
		if (status == 0)
			break;
	}
	if (status || i != 0) {
		PMD_DRV_LOG(ERR, "UDM stop anomaly. status %d iter: %u. (%s)\n",
			    status, i, __func__);
		ark_udm_dump(ark->udm.v, "Stop anomaly");

		mpu = ark->mpurx.v;
		for (i = 0; i < ark->rx_queues; i++) {
			ark_mpu_dump(mpu, "UDM Stop anomaly", i);
			mpu = RTE_PTR_ADD(mpu, ARK_MPU_QOFFSET);
		}
	}

	ark_udm_dump_stats(ark->udm.v, "Post stop");
	ark_udm_dump_perf(ark->udm.v, "Post stop");

	for (i = 0; i < dev->data->nb_tx_queues; i++)
		eth_ark_rx_dump_queue(dev, i, __func__);

	/* Stop the packet checker if it is running */
	if (ark->start_pg) {
		ark_pktchkr_dump_stats(ark->pc);
		ark_pktchkr_stop(ark->pc);
	}
}

static void
eth_ark_dev_close(struct rte_eth_dev *dev)
{
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;
	uint16_t i;

	if (ark->user_ext.dev_close)
		ark->user_ext.dev_close(dev, ark->user_data);

	eth_ark_dev_stop(dev);
	eth_ark_udm_force_close(dev);

	/*
	 * TODO This should only be called once for the device during shutdown
	 */
	ark_rqp_dump(ark->rqpacing);

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		eth_ark_tx_queue_release(dev->data->tx_queues[i]);
		dev->data->tx_queues[i] = 0;
	}

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		eth_ark_dev_rx_queue_release(dev->data->rx_queues[i]);
		dev->data->rx_queues[i] = 0;
	}
}

static void
eth_ark_dev_info_get(struct rte_eth_dev *dev,
		     struct rte_eth_dev_info *dev_info)
{
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;
	struct ark_mpu_t *tx_mpu = RTE_PTR_ADD(ark->bar0, ARK_MPU_TX_BASE);
	struct ark_mpu_t *rx_mpu = RTE_PTR_ADD(ark->bar0, ARK_MPU_RX_BASE);

	uint16_t ports = ark->num_ports;

	/* device specific configuration */
	memset(dev_info, 0, sizeof(*dev_info));

	dev_info->max_rx_queues = ark_api_num_queues_per_port(rx_mpu, ports);
	dev_info->max_tx_queues = ark_api_num_queues_per_port(tx_mpu, ports);
	dev_info->max_mac_addrs = 0;
	dev_info->if_index = 0;
	dev_info->max_rx_pktlen = (16 * 1024) - 128;
	dev_info->min_rx_bufsize = 1024;
	dev_info->rx_offload_capa = 0;
	dev_info->tx_offload_capa = 0;

	dev_info->rx_desc_lim = (struct rte_eth_desc_lim) {
		.nb_max = 4096 * 4,
		.nb_min = 512,	/* HW Q size for RX */
		.nb_align = 2,};

	dev_info->tx_desc_lim = (struct rte_eth_desc_lim) {
		.nb_max = 4096 * 4,
		.nb_min = 256,	/* HW Q size for TX */
		.nb_align = 2,};

	dev_info->rx_offload_capa = 0;
	dev_info->tx_offload_capa = 0;

	/* ARK PMD supports all line rates, how do we indicate that here ?? */
	dev_info->speed_capa = (ETH_LINK_SPEED_1G |
				ETH_LINK_SPEED_10G |
				ETH_LINK_SPEED_25G |
				ETH_LINK_SPEED_40G |
				ETH_LINK_SPEED_50G |
				ETH_LINK_SPEED_100G);
	dev_info->pci_dev = ARK_DEV_TO_PCI(dev);
	dev_info->driver_name = dev->data->drv_name;
}

static int
eth_ark_dev_link_update(struct rte_eth_dev *dev, int wait_to_complete)
{
	ARK_DEBUG_TRACE("ARKP: link status = %d\n",
			dev->data->dev_link.link_status);
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;

	if (ark->user_ext.link_update) {
		return ark->user_ext.link_update
			(dev, wait_to_complete,
			 ark->user_data);
	}
	return 0;
}

static int
eth_ark_dev_set_link_up(struct rte_eth_dev *dev)
{
	dev->data->dev_link.link_status = 1;
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;

	if (ark->user_ext.dev_set_link_up)
		return ark->user_ext.dev_set_link_up(dev, ark->user_data);
	return 0;
}

static int
eth_ark_dev_set_link_down(struct rte_eth_dev *dev)
{
	dev->data->dev_link.link_status = 0;
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;

	if (ark->user_ext.dev_set_link_down)
		return ark->user_ext.dev_set_link_down(dev, ark->user_data);
	return 0;
}

static void
eth_ark_dev_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *stats)
{
	uint16_t i;
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;

	stats->ipackets = 0;
	stats->ibytes = 0;
	stats->opackets = 0;
	stats->obytes = 0;
	stats->imissed = 0;
	stats->oerrors = 0;

	for (i = 0; i < dev->data->nb_tx_queues; i++)
		eth_tx_queue_stats_get(dev->data->tx_queues[i], stats);
	for (i = 0; i < dev->data->nb_rx_queues; i++)
		eth_rx_queue_stats_get(dev->data->rx_queues[i], stats);
	if (ark->user_ext.stats_get)
		ark->user_ext.stats_get(dev, stats, ark->user_data);
}

static void
eth_ark_dev_stats_reset(struct rte_eth_dev *dev)
{
	uint16_t i;
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;

	for (i = 0; i < dev->data->nb_tx_queues; i++)
		eth_tx_queue_stats_reset(dev->data->rx_queues[i]);
	for (i = 0; i < dev->data->nb_rx_queues; i++)
		eth_rx_queue_stats_reset(dev->data->rx_queues[i]);
	if (ark->user_ext.stats_reset)
		ark->user_ext.stats_reset(dev, ark->user_data);
}

static void
eth_ark_macaddr_add(struct rte_eth_dev *dev,
		    struct ether_addr *mac_addr,
		    uint32_t index,
		    uint32_t pool)
{
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;

	if (ark->user_ext.mac_addr_add)
		ark->user_ext.mac_addr_add(dev,
					   mac_addr,
					   index,
					   pool,
					   ark->user_data);
}

static void
eth_ark_macaddr_remove(struct rte_eth_dev *dev, uint32_t index)
{
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;

	if (ark->user_ext.mac_addr_remove)
		ark->user_ext.mac_addr_remove(dev, index, ark->user_data);
}

static void
eth_ark_set_default_mac_addr(struct rte_eth_dev *dev,
			     struct ether_addr *mac_addr)
{
	struct ark_adapter *ark =
		(struct ark_adapter *)dev->data->dev_private;

	if (ark->user_ext.mac_addr_set)
		ark->user_ext.mac_addr_set(dev, mac_addr, ark->user_data);
}

static inline int
process_pktdir_arg(const char *key, const char *value,
		   void *extra_args __rte_unused)
{
	ARK_DEBUG_TRACE("In process_pktdir_arg, key = %s, value = %s\n",
			key, value);
	pkt_dir_v = strtol(value, NULL, 16);
	ARK_DEBUG_TRACE("pkt_dir_v = 0x%x\n", pkt_dir_v);
	return 0;
}

static inline int
process_file_args(const char *key, const char *value, void *extra_args)
{
	ARK_DEBUG_TRACE("**** IN process_pktgen_arg, key = %s, value = %s\n",
			key, value);
	char *args = (char *)extra_args;

	/* Open the configuration file */
	FILE *file = fopen(value, "r");
	char line[256];
	int first = 1;

	while (fgets(line, sizeof(line), file)) {
		/* ARK_DEBUG_TRACE("%s\n", line); */
		if (first) {
			strncpy(args, line, ARK_MAX_ARG_LEN);
			first = 0;
		} else {
			strncat(args, line, ARK_MAX_ARG_LEN);
		}
	}
	ARK_DEBUG_TRACE("file = %s\n", args);
	fclose(file);
	return 0;
}

static int
eth_ark_check_args(const char *params)
{
	struct rte_kvargs *kvlist;
	unsigned int k_idx;
	struct rte_kvargs_pair *pair = NULL;

	/*
	 * TODO: the index of gark[index] should be associated with phy dev
	 * map
	 */
	struct ark_adapter *ark = gark[0];

	kvlist = rte_kvargs_parse(params, valid_arguments);
	if (kvlist == NULL)
		return 0;

	pkt_gen_args[0] = 0;
	pkt_chkr_args[0] = 0;

	for (k_idx = 0; k_idx < kvlist->count; k_idx++) {
		pair = &kvlist->pairs[k_idx];
		ARK_DEBUG_TRACE("**** Arg passed to PMD = %s:%s\n", pair->key,
				pair->value);
	}

	if (rte_kvargs_process(kvlist,
			       ARK_PKTDIR_ARG,
			       &process_pktdir_arg,
			       NULL) != 0) {
		PMD_DRV_LOG(ERR, "Unable to parse arg %s\n", ARK_PKTDIR_ARG);
	}

	if (rte_kvargs_process(kvlist,
			       ARK_PKTGEN_ARG,
			       &process_file_args,
			       pkt_gen_args) != 0) {
		PMD_DRV_LOG(ERR, "Unable to parse arg %s\n", ARK_PKTGEN_ARG);
	}

	if (rte_kvargs_process(kvlist,
			       ARK_PKTCHKR_ARG,
			       &process_file_args,
			       pkt_chkr_args) != 0) {
		PMD_DRV_LOG(ERR, "Unable to parse arg %s\n", ARK_PKTCHKR_ARG);
	}

	/* Setup the packet director */
	ark_pktdir_setup(ark->pd, pkt_dir_v);
	ARK_DEBUG_TRACE("INFO: packet director set to 0x%x\n", pkt_dir_v);

	/* Setup the packet generator */
	if (pkt_gen_args[0]) {
		PMD_DRV_LOG(INFO, "Setting up the packet generator\n");
		ark_pktgen_parse(pkt_gen_args);
		ark_pktgen_reset(ark->pg);
		ark_pktgen_setup(ark->pg);
		ark->start_pg = 1;
	}

	/* Setup the packet checker */
	if (pkt_chkr_args[0]) {
		ark_pktchkr_parse(pkt_chkr_args);
		ark_pktchkr_setup(ark->pc);
	}

	return 1;
}


/*
 * PCIE
 */
static int
pmd_ark_probe(const char *name, const char *params)
{
	RTE_LOG(INFO, PMD, "Initializing pmd_ark for %s params = %s\n", name,
		params);

	/* Parse off the v index */

	eth_ark_check_args(params);
	return 0;
}

static int
pmd_ark_remove(const char *name)
{
	RTE_LOG(INFO, PMD, "Closing ark %s ethdev on numa socket %u\n", name,
		rte_socket_id());
	return 1;
}

static struct rte_vdev_driver pmd_ark_drv = {
	.probe = pmd_ark_probe,
	.remove = pmd_ark_remove,
};

RTE_PMD_REGISTER_VDEV(net_ark, pmd_ark_drv);
RTE_PMD_REGISTER_ALIAS(net_ark, eth_ark);
RTE_PMD_REGISTER_PCI(eth_ark, rte_ark_pmd.pci_drv);
RTE_PMD_REGISTER_KMOD_DEP(net_ark, "* igb_uio | uio_pci_generic ");
RTE_PMD_REGISTER_PCI_TABLE(eth_ark, pci_id_ark_map);
