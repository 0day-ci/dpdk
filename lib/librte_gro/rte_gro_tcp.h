#ifndef _RTE_GRO_TCP_H_
#define _RTE_GRO_TCP_H_

#include <rte_ethdev.h>
#include <rte_ip.h>
#include <rte_tcp.h>
#include <rte_hash.h>
#include <rte_jhash.h>
#include <rte_malloc.h>

#if RTE_BYTE_ORDER == RTE_LITTLE_ENDIAN
#define TCP_HDR_LEN(tcph) \
	((tcph->data_off >> 4) * 4)
#define IPv4_HDR_LEN(iph) \
	((iph->version_ihl & 0x0f) * 4)
#else
#define TCP_DATAOFF_MASK 0x0f
#define TCP_HDR_LEN(tcph) \
	((tcph->data_off & TCP_DATAOFF_MASK) * 4)
#define IPv4_HDR_LEN(iph) \
	((iph->version_ihl >> 4) * 4)
#endif

#define IPV4_HDR_DF_SHIFT 14
#define IPV4_HDR_DF_MASK (1 << IPV4_HDR_DF_SHIFT)

#define RTE_GRO_TCP_HASH_ENTRIES_MIN RTE_HASH_BUCKET_ENTRIES
#define RTE_GRO_TCP_HASH_ENTRIES_MAX RTE_HASH_ENTRIES_MAX

/**
 * key structure of TCP ipv4 hash table. It describes the prerequsite
 * rules of merging packets.
 */
struct gro_tcp4_pre_rules {
	struct ether_addr eth_saddr;
	struct ether_addr eth_daddr;
	uint32_t ip_src_addr;
	uint32_t ip_dst_addr;

	uint32_t recv_ack;	/**< acknowledgment sequence number. */
	uint16_t src_port;
	uint16_t dst_port;
	uint8_t tcp_flags;	/**< TCP flags. */

	uint8_t padding[3];
};

/**
 * Item structure
 */
struct gro_tcp_item {
	struct rte_mbuf *segment;	/**< packet address. */
	uint32_t next_sent_seq;	/**< sequence number of the next packet. */
	uint32_t segment_idx;	/**< packet index. */
} __rte_cache_aligned;

/**
 * Item-list structure, which is the value in the TCP ipv4 hash table.
 */
struct gro_tcp_item_list {
	struct gro_tcp_item *items;	/**< items array */
	uint32_t nb_item;	/**< item number */
};

/**
 * Local data structure. Every packet has an object of this structure,
 * which is used for reassembling.
 */
struct gro_tcp_info {
	struct gro_tcp_item_list item_list;	/**< preallocated item-list */
	uint32_t nb_merged_pkts;	/**< the number of merged packets */
};

/**
 * Create a new TCP ipv4 GRO lookup table.
 *
 * @param name
 *	Lookup table name
 * @param nb_entries
 *  Lookup table elements number, whose value should be larger than or
 *  equal to RTE_GRO_TCP_HASH_ENTRIES_MIN, and less than or equal to
 *  RTE_GRO_TCP_HASH_ENTRIES_MAX, and should be power of two.
 * @param socket_id
 *  socket id
 * @return
 *  lookup table address
 */
struct rte_hash *
rte_gro_tcp4_tbl_create(char *name, uint32_t nb_entries,
		uint16_t socket_id);
/**
 * This function reassembles a bulk of TCP IPv4 packets. For non-TCP IPv4
 * packets, the function won't process them.
 *
 * @param hash_tbl
 *	Lookup table used to reassemble packets. It stores key-value pairs.
 *	The key describes the prerequsite rules to merge two TCP IPv4 packets;
 *	the value is a pointer pointing to a item-list, which contains
 *	packets that have the same prerequisite TCP IPv4 rules. Note that
 *	applications need to guarantee the hash_tbl is clean when first call
 *	this function.
 * @param pkts
 *	Packets to reassemble.
 * @param nb_pkts
 *	The number of packets to reassemble.
 * @return
 *	The packet number after GRO. If reassemble successfully, the value is
 *	less than nb_pkts; if not, the value is equal to nb_pkts.
 */
uint32_t
rte_gro_tcp4_reassemble_burst(struct rte_hash *hash_tbl,
		struct rte_mbuf **pkts,
		const uint32_t nb_pkts);
#endif
