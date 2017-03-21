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

/*  Internal prototypes */
static int eth_ark_check_args(const char *params);
static int eth_ark_dev_init(struct rte_eth_dev *dev);
static int eth_ark_dev_uninit(struct rte_eth_dev *eth_dev);
static int eth_ark_dev_configure(struct rte_eth_dev *dev);
static void eth_ark_dev_info_get(struct rte_eth_dev *dev,
				 struct rte_eth_dev_info *dev_info);


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
	.dev_infos_get = eth_ark_dev_info_get,

};


static int
eth_ark_dev_init(struct rte_eth_dev *dev __rte_unused)
{
	return -1;					/* STUB */
}


static int
eth_ark_dev_uninit(struct rte_eth_dev *dev)
{
	if (rte_eal_process_type() != RTE_PROC_PRIMARY)
		return 0;

	dev->dev_ops = NULL;
	dev->rx_pkt_burst = NULL;
	dev->tx_pkt_burst = NULL;
	return 0;
}

static int
eth_ark_dev_configure(struct rte_eth_dev *dev __rte_unused)
{
	ARK_DEBUG_TRACE("ARKP: In %s\n", __func__);
	return 0;
}

static void
eth_ark_dev_info_get(struct rte_eth_dev *dev,
		     struct rte_eth_dev_info *dev_info)
{
	/* device specific configuration */
	memset(dev_info, 0, sizeof(*dev_info));

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

	ARK_DEBUG_TRACE("INFO: packet director set to 0x%x\n", pkt_dir_v);

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
