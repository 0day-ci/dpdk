#ifndef _GRO_COMMON_H_
#define _GRO_COMMON_H_

/**
 * the maximum number of supported GRO types
 */
#define GRO_TYPE_MAX_NB 256
/**
 * flag indicates empty GRO type
 */
#define GRO_EMPTY_TYPE 255
/**
 * current supported GRO types number
 */
#define GRO_SUPPORT_TYPE_NB 0

/**
 * default element number of the hashing table
 */
#define GRO_DEFAULT_LOOKUP_TABLE_ENTRY_NB 64

/**
 * Structure to store addresses of all hashing tables.
 */
struct rte_gro_lkp_tbl {
	struct rte_hash *hash_tbl;
	uint8_t gro_type;
};
struct rte_gro_tbl {
	struct rte_gro_lkp_tbl lkp_tbls[GRO_SUPPORT_TYPE_NB];
};

/**
 * Item-list structure.
 */
struct gro_item_list {
	void *items;	/**< item array */
	uint16_t nb_item;	/**< item number */
};

/**
 * Each packet has an object of gro_info, which records the GRO
 * information related to this packet.
 */
struct gro_info {
	struct gro_item_list item_list;	/**< pre-allocated item-list */
	/**< packets number that are merged with it */
	uint16_t nb_merged_packets;
	uint8_t gro_type;	/**< GRO type that the packet is performed */
};

/**
 * Record GRO information for each port.
 */
struct gro_port_status {
	struct rte_gro_tbl **gro_tbls;
	struct rte_eth_rxtx_callback **gro_cbs;
	uint8_t gro_enable;	/* flag indicates if the port enables GRO */
};

struct rte_gro_status {
	struct gro_port_status *ports;
	uint8_t nb_port;
};

typedef int (*gro_tbl_create_fn)(
		char *name,
		uint32_t nb_entries,
		uint16_t socket_id,
		struct rte_hash **hash_tbl);

typedef int32_t (*gro_reassemble_fn)(
		struct rte_hash *hash_tbl,
		struct gro_item_list *item_list);
#endif
