/*-
 *
 *   Copyright(c) 2016-2017 Intel Corporation. All rights reserved.
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

#ifndef _RTE_GRO_TCP_H_
#define _RTE_GRO_TCP_H_

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

#define INVALID_FLOW_INDEX 0xffffUL
#define INVALID_ITEM_INDEX 0xffffffffULL

/* criteria of mergeing packets */
struct gro_tcp_flow_key {
	struct ether_addr eth_saddr;
	struct ether_addr eth_daddr;
	uint32_t ip_src_addr[4];	/**< IPv4 uses the first 8B */
	uint32_t ip_dst_addr[4];

	uint32_t recv_ack;	/**< acknowledgment sequence number. */
	uint16_t src_port;
	uint16_t dst_port;
	uint8_t tcp_flags;	/**< TCP flags. */
};

struct gro_tcp_flow {
	struct gro_tcp_flow_key key;
	uint32_t start_index;	/**< the first packet index of the flow */
	uint8_t is_valid;
};

struct gro_tcp_item {
	struct rte_mbuf *pkt;	/**< packet address. */
	/* the time when the packet in added into the table */
	uint64_t start_time;
	uint32_t next_pkt_idx;	/**< next packet index. */
	/* flag to indicate if the packet is GROed */
	uint8_t is_groed;
	uint8_t is_valid;	/**< flag indicates if the item is valid */
};

/**
 * TCP reassembly table. Both TCP/IPv4 and TCP/IPv6 use the same table
 * structure.
 */
struct gro_tcp_tbl {
	struct gro_tcp_item *items;	/**< item array */
	struct gro_tcp_flow *flows;	/**< flow array */
	uint32_t item_num;	/**< current item number */
	uint16_t flow_num;	/**< current flow num */
	uint32_t max_item_num;	/**< item array size */
	uint16_t max_flow_num;	/**< flow array size */
};

/* rules to reassemble TCP packets, which are decided by applications */
struct gro_tcp_rule {
	/* the maximum packet length after merged */
	uint32_t max_packet_size;
};

/**
 * This function is to update TCP and IPv4 header checksums
 * for merged packets in the TCP reassembly table.
 */
void gro_tcp4_tbl_cksum_update(struct gro_tcp_tbl *tbl);

/**
 * This function creates a TCP reassembly table.
 *
 * @param socket_id
 *  socket index where the Ethernet port connects to.
 * @param max_flow_num
 *  the maximum number of flows in the TCP GRO table
 * @param max_item_per_flow
 *  the maximum packet number per flow.
 * @return
 *  if create successfully, return a pointer which points to the
 *  created TCP GRO table. Otherwise, return NULL.
 */
void *gro_tcp_tbl_create(uint16_t socket_id,
		uint16_t max_flow_num,
		uint16_t max_item_per_flow);

/**
 * This function destroys a TCP reassembly table.
 * @param tbl
 *  a pointer points to the TCP reassembly table.
 */
void gro_tcp_tbl_destroy(void *tbl);

/**
 * This function searches for a packet in the TCP reassembly table to
 * merge with the inputted one. To merge two packets is to chain them
 * together and update packet headers. Note that this function won't
 * re-calculate IPv4 and TCP checksums.
 *
 * If the packet doesn't have data, or with wrong checksums, or is
 * fragmented etc., errors happen and gro_tcp4_reassemble returns
 * immediately. If no errors happen, the packet is either merged, or
 * inserted into the reassembly table.
 *
 * If applications want to get packets in the reassembly table, they
 * need to manually flush the packets.
 *
 * @param pkt
 *  packet to reassemble.
 * @param tbl
 *  a pointer that points to a TCP reassembly table.
 * @param rule
 *  TCP reassembly criteria defined by applications.
 * @return
 *  if the inputted packet is merged successfully, return an positive
 *  value. If the packet hasn't be merged with any packets in the TCP
 *  reassembly table. If errors happen, return a negative value and the
 *  packet won't be inserted into the reassemble table.
 */
int32_t
gro_tcp4_reassemble(struct rte_mbuf *pkt,
		struct gro_tcp_tbl *tbl,
		struct gro_tcp_rule *rule);

/**
 * This function flushes the packets in a TCP reassembly table to
 * applications. Before returning the packets, it will update TCP and
 * IPv4 header checksums.
 *
 * @param tbl
 *  a pointer that points to a TCP GRO table.
 * @param flush_num
 *  the number of packets that applications want to flush.
 * @param out
 *  pointer array which is used to keep flushed packets.
 * @param nb_out
 *  the maximum element number of out.
 * @return
 *  the number of packets that are flushed finally.
 */
uint16_t
gro_tcp_tbl_flush(struct gro_tcp_tbl *tbl,
		uint16_t flush_num,
		struct rte_mbuf **out,
		const uint16_t nb_out);

/**
 * This function flushes timeout packets in a TCP reassembly table to
 * applications. Before returning the packets, it updates TCP and IPv4
 * header checksums.
 *
 * @param tbl
 *  a pointer that points to a TCP GRO table.
 * @param timeout_cycles
 *  the maximum time that packets can stay in the table.
 * @param out
 *  pointer array which is used to keep flushed packets.
 * @param nb_out
 *  the maximum element number of out.
 * @return
 *  It returns the number of packets that are flushed finally.
 */
uint16_t
gro_tcp_tbl_timeout_flush(struct gro_tcp_tbl *tbl,
		uint64_t timeout_cycles,
		struct rte_mbuf **out,
		const uint16_t nb_out);
#endif
