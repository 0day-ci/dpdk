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
#include <rte_cycles.h>
#include <rte_ethdev.h>
#include <rte_ip.h>
#include <rte_tcp.h>

#include "rte_gro_tcp.h"

void *gro_tcp_tbl_create(uint16_t socket_id,
		uint16_t max_flow_num,
		uint16_t max_item_per_flow)
{
	size_t size;
	uint32_t entries_num;
	struct gro_tcp_tbl *tbl;

	entries_num = max_flow_num * max_item_per_flow;
	if (entries_num == 0 || max_flow_num == 0)
		return NULL;

	tbl = (struct gro_tcp_tbl *)rte_zmalloc_socket(
			__func__,
			sizeof(struct gro_tcp_tbl),
			RTE_CACHE_LINE_SIZE,
			socket_id);

	size = sizeof(struct gro_tcp_item) * entries_num;
	tbl->items = (struct gro_tcp_item *)rte_zmalloc_socket(
			__func__,
			size,
			RTE_CACHE_LINE_SIZE,
			socket_id);
	tbl->max_item_num = entries_num;

	size = sizeof(struct gro_tcp_flow) * max_flow_num;
	tbl->flows = (struct gro_tcp_flow *)rte_zmalloc_socket(
			__func__,
			size, RTE_CACHE_LINE_SIZE,
			socket_id);
	tbl->max_flow_num = max_flow_num;
	return tbl;
}

void gro_tcp_tbl_destroy(void *tbl)
{
	struct gro_tcp_tbl *tcp_tbl = (struct gro_tcp_tbl *)tbl;

	if (tcp_tbl) {
		if (tcp_tbl->items)
			rte_free(tcp_tbl->items);
		if (tcp_tbl->flows)
			rte_free(tcp_tbl->flows);
		rte_free(tcp_tbl);
	}
}

/* update TCP header and IPv4 header checksum */
static void
gro_tcp4_cksum_update(struct rte_mbuf *pkt)
{
	uint32_t len, offset, cksum;
	struct ether_hdr *eth_hdr;
	struct ipv4_hdr *ipv4_hdr;
	struct tcp_hdr *tcp_hdr;
	uint16_t ipv4_ihl, cksum_pld;

	if (pkt == NULL)
		return;

	len = pkt->pkt_len;
	eth_hdr = rte_pktmbuf_mtod(pkt, struct ether_hdr *);
	ipv4_hdr = (struct ipv4_hdr *)(eth_hdr + 1);
	ipv4_ihl = IPv4_HDR_LEN(ipv4_hdr);
	tcp_hdr = (struct tcp_hdr *)((char *)ipv4_hdr + ipv4_ihl);

	offset = sizeof(struct ether_hdr) + ipv4_ihl;
	len -= offset;

	/* TCP cksum without IP pseudo header */
	ipv4_hdr->hdr_checksum = 0;
	tcp_hdr->cksum = 0;
	if (rte_raw_cksum_mbuf(pkt, offset, len, &cksum_pld) < 0) {
		printf("invalid param for raw_cksum_mbuf\n");
		return;
	}
	/* IP pseudo header cksum */
	cksum = cksum_pld;
	cksum += rte_ipv4_phdr_cksum(ipv4_hdr, 0);

	/* combine TCP checksum and IP pseudo header checksum */
	cksum = ((cksum & 0xffff0000) >> 16) + (cksum & 0xffff);
	cksum = (~cksum) & 0xffff;
	cksum = (cksum == 0) ? 0xffff : cksum;
	tcp_hdr->cksum = cksum;

	/* update IP header cksum */
	ipv4_hdr->hdr_checksum = rte_ipv4_cksum(ipv4_hdr);
}

void gro_tcp4_tbl_cksum_update(struct gro_tcp_tbl *tbl)
{
	uint64_t i;

	for (i = 0; i < tbl->item_num; i++) {
		if (tbl->items[i].is_groed)
			gro_tcp4_cksum_update(tbl->items[i].pkt);
	}
}

/**
 * merge two TCP/IPv4 packets without update header checksum.
 */
static int
merge_two_tcp4_packets(struct rte_mbuf *pkt_src,
		struct rte_mbuf *pkt,
		struct gro_tcp_rule *rule)
{
	struct ipv4_hdr *ipv4_hdr1, *ipv4_hdr2;
	struct tcp_hdr *tcp_hdr1;
	uint16_t ipv4_ihl1, tcp_hl1, tcp_dl1;
	struct rte_mbuf *tail;

	/* parse the given packet */
	ipv4_hdr1 = (struct ipv4_hdr *)(rte_pktmbuf_mtod(pkt,
				struct ether_hdr *) + 1);
	ipv4_ihl1 = IPv4_HDR_LEN(ipv4_hdr1);
	tcp_hdr1 = (struct tcp_hdr *)((char *)ipv4_hdr1 + ipv4_ihl1);
	tcp_hl1 = TCP_HDR_LEN(tcp_hdr1);
	tcp_dl1 = rte_be_to_cpu_16(ipv4_hdr1->total_length) - ipv4_ihl1
		- tcp_hl1;

	/* parse the original packet */
	ipv4_hdr2 = (struct ipv4_hdr *)(rte_pktmbuf_mtod(pkt_src,
				struct ether_hdr *) + 1);

	/* check reassembly rules */
	if (pkt_src->pkt_len + tcp_dl1 > rule->max_packet_size)
		return -1;

	/* remove the header of the incoming packet */
	rte_pktmbuf_adj(pkt, sizeof(struct ether_hdr) +
			ipv4_ihl1 + tcp_hl1);

	/* chain the two packet together */
	tail = rte_pktmbuf_lastseg(pkt_src);
	tail->next = pkt;

	/* update IP header */
	ipv4_hdr2->total_length = rte_cpu_to_be_16(
			rte_be_to_cpu_16(
				ipv4_hdr2->total_length)
			+ tcp_dl1);

	/* update mbuf metadata for the merged packet */
	pkt_src->nb_segs++;
	pkt_src->pkt_len += pkt->pkt_len;
	return 1;
}

static int
check_seq_option(struct rte_mbuf *pkt,
		struct tcp_hdr *tcp_hdr,
		uint16_t tcp_hl)
{
	struct ipv4_hdr *ipv4_hdr1;
	struct tcp_hdr *tcp_hdr1;
	uint16_t ipv4_ihl1, tcp_hl1, tcp_dl1;
	uint32_t sent_seq1, sent_seq;
	int ret = -1;

	ipv4_hdr1 = (struct ipv4_hdr *)(rte_pktmbuf_mtod(pkt,
				struct ether_hdr *) + 1);
	ipv4_ihl1 = IPv4_HDR_LEN(ipv4_hdr1);
	tcp_hdr1 = (struct tcp_hdr *)((char *)ipv4_hdr1 + ipv4_ihl1);
	tcp_hl1 = TCP_HDR_LEN(tcp_hdr1);
	tcp_dl1 = rte_be_to_cpu_16(ipv4_hdr1->total_length) - ipv4_ihl1
		- tcp_hl1;
	sent_seq1 = rte_be_to_cpu_32(tcp_hdr1->sent_seq) + tcp_dl1;
	sent_seq = rte_be_to_cpu_32(tcp_hdr->sent_seq);

	/* check if the two packets are neighbor */
	if ((sent_seq ^ sent_seq1) == 0) {
		/* check if the option fields equal */
		if (tcp_hl1 > sizeof(struct tcp_hdr)) {
			if ((tcp_hl1 != tcp_hl) ||
					(memcmp(tcp_hdr1 + 1,
							tcp_hdr + 1,
							tcp_hl - sizeof
							(struct tcp_hdr))
					 == 0))
				ret = 1;
		}
	}
	return ret;
}

static uint32_t
find_an_empty_item(struct gro_tcp_tbl *tbl)
{
	uint32_t i;

	for (i = 0; i < tbl->max_item_num; i++)
		if (tbl->items[i].is_valid == 0)
			return i;
	return INVALID_ITEM_INDEX;
}

static uint16_t
find_an_empty_flow(struct gro_tcp_tbl *tbl)
{
	uint16_t i;

	for (i = 0; i < tbl->max_flow_num; i++)
		if (tbl->flows[i].is_valid == 0)
			return i;
	return INVALID_FLOW_INDEX;
}

int32_t
gro_tcp4_reassemble(struct rte_mbuf *pkt,
		struct gro_tcp_tbl *tbl,
		struct gro_tcp_rule *rule)
{
	struct ether_hdr *eth_hdr;
	struct ipv4_hdr *ipv4_hdr;
	struct tcp_hdr *tcp_hdr;
	uint16_t ipv4_ihl, tcp_hl, tcp_dl, tcp_cksum, ip_cksum;

	struct gro_tcp_flow_key key;
	uint64_t ol_flags;
	uint32_t cur_idx, prev_idx, item_idx;
	uint16_t i, flow_idx;

	eth_hdr = rte_pktmbuf_mtod(pkt, struct ether_hdr *);
	ipv4_hdr = (struct ipv4_hdr *)(eth_hdr + 1);
	ipv4_ihl = IPv4_HDR_LEN(ipv4_hdr);

	/* 1. check if the packet should be processed */
	if (ipv4_ihl < sizeof(struct ipv4_hdr))
		goto fail;
	if (ipv4_hdr->next_proto_id != IPPROTO_TCP)
		goto fail;
	if ((ipv4_hdr->fragment_offset &
				rte_cpu_to_be_16(IPV4_HDR_DF_MASK))
			== 0)
		goto fail;

	tcp_hdr = (struct tcp_hdr *)((char *)ipv4_hdr + ipv4_ihl);
	tcp_hl = TCP_HDR_LEN(tcp_hdr);
	tcp_dl = rte_be_to_cpu_16(ipv4_hdr->total_length) - ipv4_ihl
		- tcp_hl;
	if (tcp_dl == 0)
		goto fail;

	/**
	 * 2. if HW rx checksum offload isn't enabled, recalculate the
	 * checksum in SW. Then, check if the checksum is correct
	 */
	ol_flags = pkt->ol_flags;
	if ((ol_flags & PKT_RX_IP_CKSUM_MASK) !=
			PKT_RX_IP_CKSUM_UNKNOWN) {
		if (ol_flags == PKT_RX_IP_CKSUM_BAD)
			goto fail;
	} else {
		ip_cksum = ipv4_hdr->hdr_checksum;
		ipv4_hdr->hdr_checksum = 0;
		ipv4_hdr->hdr_checksum = rte_ipv4_cksum(ipv4_hdr);
		if (ipv4_hdr->hdr_checksum ^ ip_cksum)
			goto fail;
	}

	if ((ol_flags & PKT_RX_L4_CKSUM_MASK) !=
			PKT_RX_L4_CKSUM_UNKNOWN) {
		if (ol_flags == PKT_RX_L4_CKSUM_BAD)
			goto fail;
	} else {
		tcp_cksum = tcp_hdr->cksum;
		tcp_hdr->cksum = 0;
		tcp_hdr->cksum = rte_ipv4_udptcp_cksum
			(ipv4_hdr, tcp_hdr);
		if (tcp_hdr->cksum ^ tcp_cksum)
			goto fail;
	}

	/**
	 * 3. search for a flow and traverse all packets in the flow
	 * to find one to merge with the given packet.
	 */
	key.eth_saddr = eth_hdr->s_addr;
	key.eth_daddr = eth_hdr->d_addr;
	key.ip_src_addr[0] = rte_be_to_cpu_32(ipv4_hdr->src_addr);
	key.ip_dst_addr[0] = rte_be_to_cpu_32(ipv4_hdr->dst_addr);
	key.src_port = rte_be_to_cpu_16(tcp_hdr->src_port);
	key.dst_port = rte_be_to_cpu_16(tcp_hdr->dst_port);
	key.recv_ack = rte_be_to_cpu_32(tcp_hdr->recv_ack);
	key.tcp_flags = tcp_hdr->tcp_flags;

	for (i = 0; i < tbl->max_flow_num; i++) {
		/* search all packets in a valid flow. */
		if (tbl->flows[i].is_valid &&
				(memcmp(&(tbl->flows[i].key), &key,
						sizeof(struct gro_tcp_flow_key))
				 == 0)) {
			cur_idx = tbl->flows[i].start_index;
			prev_idx = cur_idx;
			while (cur_idx != INVALID_ITEM_INDEX) {
				if (check_seq_option(tbl->items[cur_idx].pkt,
							tcp_hdr,
							tcp_hl) > 0) {
					if (merge_two_tcp4_packets(
								tbl->items[cur_idx].pkt,
								pkt,
								rule) > 0) {
						/* successfully merge two packets */
						tbl->items[cur_idx].is_groed = 1;
						return 1;
					}
					/**
					 * fail to merge two packets since
					 * break the rules, add the packet
					 * into the flow.
					 */
					goto insert_to_existed_flow;
				} else {
					prev_idx = cur_idx;
					cur_idx = tbl->items[cur_idx].next_pkt_idx;
				}
			}
			/**
			 * fail to merge the given packet into an existed flow,
			 * add it into the flow.
			 */
insert_to_existed_flow:
			item_idx = find_an_empty_item(tbl);
			/* the item number is beyond the maximum value */
			if (item_idx == INVALID_ITEM_INDEX)
				return -1;
			tbl->items[prev_idx].next_pkt_idx = item_idx;
			tbl->items[item_idx].pkt = pkt;
			tbl->items[item_idx].is_groed = 0;
			tbl->items[item_idx].next_pkt_idx = INVALID_ITEM_INDEX;
			tbl->items[item_idx].is_valid = 1;
			tbl->items[item_idx].start_time = rte_rdtsc();
			tbl->item_num++;
			return 0;
		}
	}

	/**
	 * merge fail as the given packet is a new flow. Therefore,
	 * insert a new flow.
	 */
	item_idx = find_an_empty_item(tbl);
	flow_idx = find_an_empty_flow(tbl);
	/**
	 * if the flow or item number are beyond the maximum values,
	 * the inputted packet won't be processed.
	 */
	if (item_idx == INVALID_ITEM_INDEX ||
			flow_idx == INVALID_FLOW_INDEX)
		return -1;
	tbl->items[item_idx].pkt = pkt;
	tbl->items[item_idx].next_pkt_idx = INVALID_ITEM_INDEX;
	tbl->items[item_idx].is_groed = 0;
	tbl->items[item_idx].is_valid = 1;
	tbl->items[item_idx].start_time = rte_rdtsc();
	tbl->item_num++;

	memcpy(&(tbl->flows[flow_idx].key),
			&key, sizeof(struct gro_tcp_flow_key));
	tbl->flows[flow_idx].start_index = item_idx;
	tbl->flows[flow_idx].is_valid = 1;
	tbl->flow_num++;

	return 0;
fail:
	return -1;
}

uint16_t gro_tcp_tbl_flush(struct gro_tcp_tbl *tbl,
		uint16_t flush_num,
		struct rte_mbuf **out,
		const uint16_t nb_out)
{
	uint16_t num, k;
	uint16_t i;
	uint32_t j;

	k = 0;
	num = tbl->item_num > flush_num ? flush_num : tbl->item_num;
	num = num > nb_out ? nb_out : num;
	if (num == 0)
		return 0;

	for (i = 0; i < tbl->max_flow_num; i++) {
		if (tbl->flows[i].is_valid) {
			j = tbl->flows[i].start_index;
			while (j != INVALID_ITEM_INDEX) {
				/* update checksum for GROed packet */
				if (tbl->items[j].is_groed)
					gro_tcp4_cksum_update(tbl->items[j].pkt);

				out[k++] = tbl->items[j].pkt;
				tbl->items[j].is_valid = 0;
				tbl->item_num--;
				j = tbl->items[j].next_pkt_idx;

				if (k == num) {
					/* delete the flow */
					if (j == INVALID_ITEM_INDEX) {
						tbl->flows[i].is_valid = 0;
						tbl->flow_num--;
					} else
						/* update flow information */
						tbl->flows[i].start_index = j;
					goto end;
				}
			}
			/* delete the flow, as all of its packets are flushed */
			tbl->flows[i].is_valid = 0;
			tbl->flow_num--;
		}
	}
end:
	return num;
}

uint16_t
gro_tcp_tbl_timeout_flush(struct gro_tcp_tbl *tbl,
		uint64_t timeout_cycles,
		struct rte_mbuf **out,
		const uint16_t nb_out)
{
	uint16_t k;
	uint16_t i;
	uint32_t j;
	uint64_t current_time;

	if (nb_out == 0)
		return 0;
	k = 0;
	current_time = rte_rdtsc();

	for (i = 0; i < tbl->max_flow_num; i++) {
		if (tbl->flows[i].is_valid) {
			j = tbl->flows[i].start_index;
			while (j != INVALID_ITEM_INDEX) {
				if (current_time - tbl->items[j].start_time >=
						timeout_cycles) {
					/* update checksum for GROed packet */
					if (tbl->items[j].is_groed)
						gro_tcp4_cksum_update(tbl->items[j].pkt);

					out[k++] = tbl->items[j].pkt;
					tbl->items[j].is_valid = 0;
					tbl->item_num--;
					j = tbl->items[j].next_pkt_idx;

					if (k == nb_out) {
						if (j == INVALID_ITEM_INDEX) {
							/* delete the flow */
							tbl->flows[i].is_valid = 0;
							tbl->flow_num--;
						} else
							tbl->flows[i].start_index = j;
						goto end;
					}
				}
			}
			/* delete the flow, as all of its packets are flushed */
			tbl->flows[i].is_valid = 0;
			tbl->flow_num--;
		}
	}
end:
	return k;
}
