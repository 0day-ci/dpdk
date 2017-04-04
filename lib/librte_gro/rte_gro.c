#include <rte_ethdev.h>
#include <rte_mbuf.h>
#include <rte_hash.h>
#include <stdint.h>
#include <rte_malloc.h>

#include "rte_gro.h"
#include "rte_gro_common.h"

gro_reassemble_fn reassemble_functions[GRO_TYPE_MAX_NB] = {NULL};
gro_tbl_create_fn tbl_create_functions[GRO_TYPE_MAX_NB] = {NULL};

struct rte_gro_status *gro_status;

/**
 * Internal function. It creates one hashing table for all
 * DPDK-supported GRO types, and all of them are stored in an object
 * of struct rte_gro_tbl.
 *
 * @param name
 *  Name for GRO lookup table
 * @param nb_entries
 *  Element number of each hashing table
 * @param socket_id
 *  socket id
 * @param gro_tbl
 *  gro_tbl points to a rte_gro_tbl object, which will be initalized
 *  inside rte_gro_tbl_setup.
 * @return
 *  If create successfully, return a positive value; if not, return
 *  a negative value.
 */
static int
rte_gro_tbl_setup(char *name, uint32_t nb_entries,
		uint16_t socket_id, struct rte_gro_tbl *gro_tbl)
{
	gro_tbl_create_fn create_tbl_fn;
	const uint32_t len = strlen(name) + 10;
	char tbl_name[len];

	for (int i = 0; i < GRO_SUPPORT_TYPE_NB; i++) {
		sprintf(tbl_name, "%s_%u", name, i);
		create_tbl_fn = tbl_create_functions[i];
		if (create_tbl_fn && (create_tbl_fn(name,
						nb_entries,
						socket_id,
						&(gro_tbl->
							lkp_tbls[i].hash_tbl))
					< 0)) {
			return -1;
		}
		gro_tbl->lkp_tbls[i].gro_type = i;
	}
	return 1;
}

/**
 * Internal function. It frees all the hashing tables stored in
 * the given struct rte_gro_tbl object.
 */
static void
rte_gro_tbl_destroy(struct rte_gro_tbl *gro_tbl)
{
	if (gro_tbl == NULL)
		return;
	for (int i = 0; i < GRO_SUPPORT_TYPE_NB; i++) {
		rte_hash_free(gro_tbl->lkp_tbls[i].hash_tbl);
		gro_tbl->lkp_tbls[i].hash_tbl = NULL;
		gro_tbl->lkp_tbls[i].gro_type = GRO_EMPTY_TYPE;
	}
}

/**
 * Internal function. It performs all supported GRO types on inputted
 * packets. For example, if current DPDK GRO supports TCP/IPv4 and
 * TCP/IPv6 GRO, this functions just reassembles TCP/IPv4 and TCP/IPv6
 * packets. Packets of unsupported GRO types won't be processed. For
 * ethernet devices, which want to support GRO, this function is used to
 * registered as RX callback for all queues.
 *
 * @param pkts
 *  Packets to reassemble.
 * @param nb_pkts
 *  The number of packets to reassemble.
 * @param gro_tbl
 *  pointer points to an object of struct rte_gro_tbl, which has been
 *  initialized by rte_gro_tbl_setup.
 * @return
 *  Packet number after GRO. If reassemble successfully, the value is
 *  less than nb_pkts; if not, the value is equal to nb_pkts. If the
 *  parameters are invalid, return 0.
 */
static uint16_t
rte_gro_reassemble_burst(uint8_t port __rte_unused,
		uint16_t queue __rte_unused,
		struct rte_mbuf **pkts,
		uint16_t nb_pkts,
		uint16_t max_pkts __rte_unused,
		void *gro_tbl)
{
	if ((gro_tbl == NULL) || (pkts == NULL)) {
		printf("invalid parameters for GRO.\n");
		return 0;
	}
	uint16_t nb_after_gro = nb_pkts;

	return nb_after_gro;
}

void
rte_gro_init(void)
{
	uint8_t nb_port;
	uint16_t nb_queue;
	struct rte_eth_dev_info dev_info;

	/* if init already, return immediately */
	if (gro_status) {
		printf("repeatly init GRO environment\n");
		return;
	}

	gro_status = (struct rte_gro_status *)rte_zmalloc(
			NULL,
			sizeof(struct rte_gro_status),
			0);

	nb_port = rte_eth_dev_count();
	gro_status->ports = (struct gro_port_status *)rte_zmalloc(
			NULL,
			nb_port * sizeof(struct gro_port_status),
			0);
	gro_status->nb_port = nb_port;

	for (uint8_t i = 0; i < nb_port; i++) {
		rte_eth_dev_info_get(i, &dev_info);
		nb_queue = dev_info.nb_rx_queues;
		gro_status->ports[i].gro_tbls =
			(struct rte_gro_tbl **)rte_zmalloc(
					NULL,
					nb_queue * sizeof(struct rte_gro_tbl *),
					0);
		gro_status->ports[i].gro_cbs =
			(struct rte_eth_rxtx_callback **)
			rte_zmalloc(
					NULL,
					nb_queue *
					sizeof(struct rte_eth_rxtx_callback *),
					0);
	}
}

void
rte_gro_enable(uint8_t port_id, uint16_t socket_id)
{
	if (gro_status->ports[port_id].gro_enable) {
		printf("port %u has enabled GRO\n", port_id);
		return;
	}
	uint16_t nb_queue;
	struct rte_eth_dev_info dev_info;
	char tbl_name[20];

	rte_eth_dev_info_get(port_id, &dev_info);
	nb_queue = dev_info.nb_rx_queues;

	for (uint16_t i = 0; i < nb_queue; i++) {
		struct rte_gro_tbl *gro_tbl;

		/* allocate hashing tables for this port */
		sprintf(tbl_name, "GRO_TBL_%u", port_id);
		gro_tbl = (struct rte_gro_tbl *)rte_malloc
			(NULL, sizeof(struct rte_gro_tbl), 0);
		rte_gro_tbl_setup(tbl_name,
				GRO_DEFAULT_LOOKUP_TABLE_ENTRY_NB,
				socket_id,
				gro_tbl);
		gro_status->ports[port_id].gro_tbls[i] = gro_tbl;
		/**
		 * register GRO reassembly function as a rx callback for each
		 * queue of this port.
		 */
		gro_status->ports[port_id].gro_cbs[i] =
			rte_eth_add_rx_callback
			(port_id, i,
			 rte_gro_reassemble_burst,
			 gro_tbl);
	}
	gro_status->ports[port_id].gro_enable = 1;
}

void
rte_gro_disable(uint8_t port_id)
{
	if (gro_status->ports[port_id].gro_enable == 0) {
		printf("port %u has disabled GRO\n", port_id);
		return;
	}
	uint16_t nb_queue;
	struct rte_eth_dev_info dev_info;

	rte_eth_dev_info_get(port_id, &dev_info);
	nb_queue = dev_info.nb_rx_queues;

	for (uint16_t i = 0; i < nb_queue; i++) {
		/* free all hashing tables */
		rte_gro_tbl_destroy(gro_status->ports[port_id].gro_tbls[i]);
		gro_status->ports[port_id].gro_tbls[i] = NULL;

		/* remove GRO rx callback */
		rte_eth_remove_rx_callback(port_id, i,
				gro_status->ports[port_id].gro_cbs[i]);
		gro_status->ports[port_id].gro_cbs[i] = NULL;
	}
	gro_status->ports[port_id].gro_enable = 0;
}
