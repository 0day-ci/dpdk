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

#ifndef _RTE_GRO_H_
#define _RTE_GRO_H_

/* maximum number of supported GRO types */
#define GRO_TYPE_MAX_NB 64
#define GRO_TYPE_SUPPORT_NB 1	/**< supported GRO types number */

/* TCP/IPv4 GRO flag */
#define GRO_TCP_IPV4_INDEX 0
#define GRO_TCP_IPV4 (1ULL << GRO_TCP_IPV4_INDEX)

/**
 * GRO table structure. DPDK GRO uses GRO table to reassemble
 * packets. In heightweight mode, applications must create GRO tables
 * before performing GRO. However, in lightweight mode, applications
 * don't need create GRO tables.
 *
 * A GRO table object stores many reassembly tables of desired
 * GRO types.
 */
struct rte_gro_tbl {
	/* table addresses of desired GRO types */
	void *tbls[GRO_TYPE_MAX_NB];
	uint64_t desired_gro_types;	/**< GRO types that want to perform */
	/**
	 * the maximum time of packets staying in GRO tables, measured in
	 * nanosecond.
	 */
	uint64_t max_timeout_cycles;
	/* the maximum length of merged packet, measured in byte */
	uint32_t max_packet_size;
};

/**
 * In lightweihgt mode, applications use this strcuture to pass the
 * needed parameters to rte_gro_reassemble_burst.
 */
struct rte_gro_param {
	uint16_t max_flow_num;	/**< max flow number */
	uint16_t max_item_per_flow;	/**< max item number per flow */
	/**
	 * It indicates the GRO types that applications want to perform,
	 * whose value is the result of OR operation on GRO type flags.
	 */
	uint64_t desired_gro_types;
	/* the maximum packet size after been merged */
	uint32_t max_packet_size;
};

typedef void *(*gro_tbl_create_fn)(uint16_t socket_id,
		uint16_t max_flow_num,
		uint16_t max_item_per_flow);
typedef void (*gro_tbl_destroy_fn)(void *tbl);

/**
 * This function create a GRO table, which is used to merge packets.
 *
 * @param socket_id
 *  socket index where the Ethernet port connects to.
 * @param max_flow_num
 *  the maximum flow number in the GRO table
 * @param max_item_per_flow
 *  the maximum packet number per flow
 * @param max_packet_size
 *  the maximum size of merged packets, which is measured in byte.
 * @param max_timeout_cycles
 *  the maximum time that a packet can stay in the GRO table.
 * @param desired_gro_types
 *  GRO types that applications want to perform. It's value is the
 *  result of OR operation on desired GRO type flags.
 * @return
 *  If create successfully, return a pointer which points to the GRO
 *  table. Otherwise, return NULL.
 */
struct rte_gro_tbl *rte_gro_tbl_create(uint16_t socket_id,
		uint16_t max_flow_num,
		uint16_t max_item_per_flow,
		uint32_t max_packet_size,
		uint64_t max_timeout_cycles,
		uint64_t desired_gro_types);
/**
 * This function destroys a GRO table.
 */
void rte_gro_tbl_destroy(struct rte_gro_tbl *gro_tbl);

/**
 * This is the main reassembly API used in lightweight mode, which
 * merges numbers of packets at a time. After it returns, applications
 * can get GROed packets immediately. Applications don't need to
 * flush packets manually. In lightweight mode, applications just need
 * to tell the reassembly API what rules should be applied when merge
 * packets. Therefore, applications can perform GRO in very a simple
 * way.
 *
 * To process one packet, we find its corresponding reassembly table
 * according to the packet type. Then search for the reassembly table
 * to find one packet to merge. If find, chain the two packets together.
 * If not find, insert the inputted packet into the reassembly table.
 * Besides, to merge two packets is to chain them together. No
 * memory copy is needed. Before rte_gro_reassemble_burst returns,
 * header checksums of merged packets are re-calculated.
 *
 * @param pkts
 *  a pointer array which points to the packets to reassemble. After
 *  GRO, it is also used to keep GROed packets.
 * @param nb_pkts
 *  the number of packets to reassemble.
 * @param param
 *  Applications use param to tell rte_gro_reassemble_burst what rules
 *  are demanded.
 * @return
 *  the number of packets after GROed.
 */
uint16_t rte_gro_reassemble_burst(struct rte_mbuf **pkts,
		const uint16_t nb_pkts,
		const struct rte_gro_param param);

/**
 * This is the main reassembly API used in heavyweight mode, which
 * merges one packet at a time. The procedure of merging one packet is
 * similar with rte_gro_reassemble_burst. But rte_gro_reassemble will
 * not update header checksums. Header checksums of merged packets are
 * re-calculated in flush APIs.
 *
 * If error happens, like packet with error checksum and with
 * unsupported GRO types, the inputted packet won't be stored in GRO
 * table. If no errors happen, the packet is either merged with an
 * existed packet, or inserted into its corresponding reassembly table.
 * Applications can get packets in the GRO table by flush APIs.
 *
 * @param pkt
 *  packet to reassemble.
 * @param gro_tbl
 *  a pointer points to a GRO table.
 * @return
 *  if merge the packet successfully, return a positive value. If fail
 *  to merge, return zero. If errors happen, return a negative value.
 */
int rte_gro_reassemble(struct rte_mbuf *pkt,
		struct rte_gro_tbl *gro_tbl);

/**
 * This function flushed packets of desired GRO types from their
 * corresponding reassembly tables.
 *
 * @param gro_tbl
 *  a pointer points to a GRO table object.
 * @param desired_gro_types
 *  GRO types whose packets will be flushed.
 * @param flush_num
 *  the number of packets that need flushing.
 * @param out
 *  a pointer array that is used to keep flushed packets.
 * @param nb_out
 *  the size of out.
 * @return
 *  the number of flushed packets. If no packets are flushed, return 0.
 */
uint16_t rte_gro_flush(struct rte_gro_tbl *gro_tbl,
		uint64_t desired_gro_types,
		uint16_t flush_num,
		struct rte_mbuf **out,
		const uint16_t max_nb_out);

/**
 * This function flushes the timeout packets from reassembly tables of
 * desired GRO types.
 *
 * @param gro_tbl
 *  a pointer points to a GRO table object.
 * @param desired_gro_types
 * rte_gro_timeout_flush only processes packets which belong to the
 * GRO types specified by desired_gro_types.
 * @param out
 *  a pointer array that is used to keep flushed packets.
 * @param nb_out
 *  the size of out.
 * @return
 *  the number of flushed packets. If no packets are flushed, return 0.
 */
uint16_t rte_gro_timeout_flush(struct rte_gro_tbl *gro_tbl,
		uint64_t desired_gro_types,
		struct rte_mbuf **out,
		const uint16_t max_nb_out);
#endif
