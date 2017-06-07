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

#include <rte_malloc.h>
#include <rte_mbuf.h>
#include <rte_ethdev.h>
#include <rte_ip.h>
#include <rte_tcp.h>

#include "rte_gro.h"
#include "rte_gro_tcp.h"

static gro_tbl_create_fn tbl_create_functions[GRO_TYPE_MAX_NB] = {
	gro_tcp_tbl_create, NULL};
static gro_tbl_destroy_fn tbl_destroy_functions[GRO_TYPE_MAX_NB] = {
	gro_tcp_tbl_destroy, NULL};

struct rte_gro_tbl *rte_gro_tbl_create(uint16_t socket_id,
		uint16_t max_flow_num,
		uint16_t max_item_per_flow,
		uint32_t max_packet_size,
		uint64_t max_timeout_cycles,
		uint64_t desired_gro_types)
{
	gro_tbl_create_fn create_tbl_fn;
	struct rte_gro_tbl *gro_tbl;
	uint64_t gro_type_flag = 0;
	uint8_t i;

	gro_tbl = rte_zmalloc_socket(__func__,
			sizeof(struct rte_gro_tbl),
			RTE_CACHE_LINE_SIZE,
			socket_id);
	gro_tbl->max_packet_size = max_packet_size;
	gro_tbl->max_timeout_cycles = max_timeout_cycles;
	gro_tbl->desired_gro_types = desired_gro_types;

	for (i = 0; i < GRO_TYPE_MAX_NB; i++) {
		gro_type_flag = 1 << i;
		if (desired_gro_types & gro_type_flag) {
			create_tbl_fn = tbl_create_functions[i];
			if (create_tbl_fn)
				create_tbl_fn(socket_id,
						max_flow_num,
						max_item_per_flow);
			else
				gro_tbl->tbls[i] = NULL;
		}
	}
	return gro_tbl;
}

void rte_gro_tbl_destroy(struct rte_gro_tbl *gro_tbl)
{
	gro_tbl_destroy_fn destroy_tbl_fn;
	uint64_t gro_type_flag;
	uint8_t i;

	if (gro_tbl == NULL)
		return;
	for (i = 0; i < GRO_TYPE_MAX_NB; i++) {
		gro_type_flag = 1 << i;
		if (gro_tbl->desired_gro_types & gro_type_flag) {
			destroy_tbl_fn = tbl_destroy_functions[i];
			if (destroy_tbl_fn)
				destroy_tbl_fn(gro_tbl->tbls[i]);
			gro_tbl->tbls[i] = NULL;
		}
	}
	rte_free(gro_tbl);
}

uint16_t
rte_gro_reassemble_burst(struct rte_mbuf **pkts,
		const uint16_t nb_pkts,
		const struct rte_gro_param param)
{
	struct ether_hdr *eth_hdr;
	struct ipv4_hdr *ipv4_hdr;
	uint16_t l3proc_type, i;
	uint16_t nb_after_gro = nb_pkts;
	const uint64_t item_num = nb_pkts >
		param.max_flow_num * param.max_item_per_flow ?
		param.max_flow_num * param.max_item_per_flow :
		nb_pkts;
	const uint32_t flow_num = nb_pkts > param.max_flow_num ?
		param.max_flow_num : nb_pkts;

	/* allocate respective GRO tables for all supported GRO types */
	struct gro_tcp_tbl tcp_tbl;
	struct gro_tcp_flow tcp_flows[flow_num];
	struct gro_tcp_item tcp_items[item_num];
	struct gro_tcp_rule tcp_rule;

	struct rte_mbuf *unprocess_pkts[nb_pkts];
	uint16_t unprocess_num = 0;
	int32_t ret;

	if (unlikely(nb_pkts <= 1))
		return nb_pkts;

	memset(tcp_flows, 0, sizeof(struct gro_tcp_flow) *
			flow_num);
	memset(tcp_items, 0, sizeof(struct gro_tcp_item) *
			item_num);
	tcp_tbl.flows = tcp_flows;
	tcp_tbl.items = tcp_items;
	tcp_tbl.flow_num = 0;
	tcp_tbl.item_num = 0;
	tcp_tbl.max_flow_num = flow_num;
	tcp_tbl.max_item_num = item_num;
	tcp_rule.max_packet_size = param.max_packet_size;

	for (i = 0; i < nb_pkts; i++) {
		eth_hdr = rte_pktmbuf_mtod(pkts[i], struct ether_hdr *);
		l3proc_type = rte_be_to_cpu_16(eth_hdr->ether_type);
		if (l3proc_type == ETHER_TYPE_IPv4) {
			ipv4_hdr = (struct ipv4_hdr *)(eth_hdr + 1);
			if (ipv4_hdr->next_proto_id == IPPROTO_TCP &&
					(param.desired_gro_types &
					 GRO_TCP_IPV4)) {
				ret = gro_tcp4_reassemble(pkts[i],
						&tcp_tbl,
						&tcp_rule);
				if (ret > 0)
					nb_after_gro--;
				else if (ret < 0)
					unprocess_pkts[unprocess_num++] =
						pkts[i];
			} else
				unprocess_pkts[unprocess_num++] =
					pkts[i];
		} else
			unprocess_pkts[unprocess_num++] =
				pkts[i];
	}

	if (nb_after_gro < nb_pkts) {
		/* update packets headers and re-arrange GROed packets */
		if (param.desired_gro_types & GRO_TCP_IPV4) {
			gro_tcp4_tbl_cksum_update(&tcp_tbl);
			for (i = 0; i < tcp_tbl.item_num; i++)
				pkts[i] = tcp_tbl.items[i].pkt;
		}
		if (unprocess_num > 0) {
			memcpy(&pkts[i], unprocess_pkts,
					sizeof(struct rte_mbuf *) *
					unprocess_num);
			i += unprocess_num;
		}
		if (nb_pkts > i)
			memset(&pkts[i], 0,
					sizeof(struct rte_mbuf *) *
					(nb_pkts - i));
	}
	return nb_after_gro;
}

int rte_gro_reassemble(struct rte_mbuf *pkt,
		struct rte_gro_tbl *gro_tbl)
{
	struct ether_hdr *eth_hdr;
	struct ipv4_hdr *ipv4_hdr;
	uint16_t l3proc_type;
	struct gro_tcp_rule tcp_rule;

	if (pkt == NULL)
		return -1;
	tcp_rule.max_packet_size = gro_tbl->max_packet_size;
	eth_hdr = rte_pktmbuf_mtod(pkt, struct ether_hdr *);
	l3proc_type = rte_be_to_cpu_16(eth_hdr->ether_type);
	if (l3proc_type == ETHER_TYPE_IPv4) {
		ipv4_hdr = (struct ipv4_hdr *)(eth_hdr + 1);
		if (ipv4_hdr->next_proto_id == IPPROTO_TCP &&
				(gro_tbl->desired_gro_types & GRO_TCP_IPV4)) {
			return gro_tcp4_reassemble(pkt,
					gro_tbl->tbls[GRO_TCP_IPV4_INDEX],
					&tcp_rule);
		}
	}
	return -1;
}

uint16_t rte_gro_flush(struct rte_gro_tbl *gro_tbl,
		uint64_t desired_gro_types,
		uint16_t flush_num,
		struct rte_mbuf **out,
		const uint16_t max_nb_out)
{
	desired_gro_types = desired_gro_types &
		gro_tbl->desired_gro_types;
	if (desired_gro_types & GRO_TCP_IPV4)
		return gro_tcp_tbl_flush(
				gro_tbl->tbls[GRO_TCP_IPV4_INDEX],
				flush_num,
				out,
				max_nb_out);
	return 0;
}

uint16_t
rte_gro_timeout_flush(struct rte_gro_tbl *gro_tbl,
		uint64_t desired_gro_types,
		struct rte_mbuf **out,
		const uint16_t max_nb_out)
{
	desired_gro_types = desired_gro_types &
		gro_tbl->desired_gro_types;
	if (desired_gro_types & GRO_TCP_IPV4)
		return gro_tcp_tbl_timeout_flush(
				gro_tbl->tbls[GRO_TCP_IPV4_INDEX],
				gro_tbl->max_timeout_cycles,
				out, max_nb_out);
	return 0;
}
