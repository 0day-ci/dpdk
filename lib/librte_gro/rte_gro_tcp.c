#include "rte_gro_tcp.h"

int
rte_gro_tcp4_tbl_create(char *name,
		uint32_t nb_entries, uint16_t socket_id,
		struct rte_hash **hash_tbl)
{
	struct rte_hash_parameters ht_param = {
		.entries = nb_entries,
		.name = name,
		.key_len = sizeof(struct gro_tcp4_pre_rules),
		.hash_func = rte_jhash,
		.hash_func_init_val = 0,
		.socket_id = socket_id,
	};

	*hash_tbl = rte_hash_create(&ht_param);
	if (likely(*hash_tbl != NULL))
		return 0;
	return -1;
}

/* update TCP IPv4 checksum */
void
rte_gro_tcp4_cksum_update(struct rte_mbuf *pkt)
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

/**
 * This function traverses the item-list to find one item that can be
 * merged with the incoming packet. If merge successfully, the merged
 * packets are chained together; if not, insert the incoming packet into
 * the item-list.
 */
static int32_t
gro_tcp4_reassemble(struct rte_mbuf *pkt,
		uint16_t pkt_idx,
		uint32_t pkt_sent_seq,
		struct gro_item_list *list)
{
	struct gro_tcp_item *items;
	struct ipv4_hdr *ipv4_hdr1;
	struct tcp_hdr *tcp_hdr1;
	uint16_t ipv4_ihl1, tcp_hl1, tcp_dl1;

	items = (struct gro_tcp_item *)list->items;
	ipv4_hdr1 = (struct ipv4_hdr *)(rte_pktmbuf_mtod(pkt, struct
				ether_hdr *) + 1);
	ipv4_ihl1 = IPv4_HDR_LEN(ipv4_hdr1);
	tcp_hdr1 = (struct tcp_hdr *)((char *)ipv4_hdr1 + ipv4_ihl1);
	tcp_hl1 = TCP_HDR_LEN(tcp_hdr1);
	tcp_dl1 = rte_be_to_cpu_16(ipv4_hdr1->total_length) - ipv4_ihl1
		- tcp_hl1;

	for (uint16_t i = 0; i < list->nb_item; i++) {
		/* check if the two packets are neighbor */
		if ((pkt_sent_seq ^ items[i].next_sent_seq) == 0) {
			struct ipv4_hdr *ipv4_hdr2;
			struct tcp_hdr *tcp_hdr2;
			uint16_t ipv4_ihl2, tcp_hl2;
			struct rte_mbuf *tail;

			ipv4_hdr2 = (struct ipv4_hdr *)(rte_pktmbuf_mtod(
						items[i].segment,
						struct ether_hdr *)
					+ 1);

			/* check if the option fields equal */
			if (tcp_hl1 > sizeof(struct tcp_hdr)) {
				ipv4_ihl2 = IPv4_HDR_LEN(ipv4_hdr2);
				tcp_hdr2 = (struct tcp_hdr *)
					((char *)ipv4_hdr2 + ipv4_ihl2);
				tcp_hl2 = TCP_HDR_LEN(tcp_hdr2);
				if ((tcp_hl1 != tcp_hl2) ||
						(memcmp(tcp_hdr1 + 1,
								tcp_hdr2 + 1,
								tcp_hl2 - sizeof
								(struct tcp_hdr))
						 != 0))
					continue;
			}
			/* check if the packet length will be beyond 64K */
			if (items[i].segment->pkt_len + tcp_dl1 > UINT16_MAX)
				goto merge_fail;

			/* remove the header of the incoming packet */
			rte_pktmbuf_adj(pkt, sizeof(struct ether_hdr) +
					ipv4_ihl1 + tcp_hl1);
			/* chain the two packet together */
			tail = rte_pktmbuf_lastseg(items[i].segment);
			tail->next = pkt;

			/* update IP header for the merged packet */
			ipv4_hdr2->total_length = rte_cpu_to_be_16(
					rte_be_to_cpu_16(
						ipv4_hdr2->total_length)
					+ tcp_dl1);

			/* update the next expected sequence number */
			items[i].next_sent_seq += tcp_dl1;

			/* update mbuf metadata for the merged packet */
			items[i].segment->nb_segs++;
			items[i].segment->pkt_len += pkt->pkt_len;

			return items[i].segment_idx + 1;
		}
	}

merge_fail:
	/* fail to merge. Insert the incoming packet into the item-list */
	items[list->nb_item].next_sent_seq = pkt_sent_seq + tcp_dl1;
	items[list->nb_item].segment = pkt;
	items[list->nb_item].segment_idx = pkt_idx;
	list->nb_item++;

	return 0;
}

/**
 * Traverse the item-list to find a packet to merge with the incoming
 * one.
 * @param hash_tbl
 *  TCP IPv4 lookup table
 * @param item_list
 *  Pre-allocated item-list, in which the first item stores the packet
 *  to process.
 * @return
 *  If the incoming packet merges with one packet successfully, return
 *  the index + 1 of the merged packet; if the incoming packet hasn't
 *  been performed GRO, return -1; if the incoming packet is performed
 *  GRO but fail to merge, return 0.
 */
int32_t
rte_gro_tcp4_reassemble(struct rte_hash *hash_tbl,
		struct gro_item_list *item_list)
{
	struct ether_hdr *eth_hdr;
	struct ipv4_hdr *ipv4_hdr;
	struct tcp_hdr *tcp_hdr;
	uint16_t ipv4_ihl, tcp_hl, tcp_dl, tcp_cksum, ip_cksum;
	struct gro_tcp4_pre_rules key = {0};
	struct gro_item_list *list;
	uint64_t ol_flags;
	uint32_t sent_seq;
	int32_t ret = -1;

	/* get the packet to process */
	struct gro_tcp_item *items = item_list->items;
	struct rte_mbuf *pkt = items[0].segment;
	uint32_t pkt_idx = items[0].segment_idx;

	eth_hdr = rte_pktmbuf_mtod(pkt, struct ether_hdr *);
	ipv4_hdr = (struct ipv4_hdr *)(eth_hdr + 1);
	ipv4_ihl = IPv4_HDR_LEN(ipv4_hdr);

	/* 1. check if the packet should be processed */
	if (ipv4_ihl < sizeof(struct ipv4_hdr))
		goto end;
	if (ipv4_hdr->next_proto_id != IPPROTO_TCP)
		goto end;
	if ((ipv4_hdr->fragment_offset &
				rte_cpu_to_be_16(IPV4_HDR_DF_MASK))
			== 0)
		goto end;

	tcp_hdr = (struct tcp_hdr *)((char *)ipv4_hdr + ipv4_ihl);
	tcp_hl = TCP_HDR_LEN(tcp_hdr);
	tcp_dl = rte_be_to_cpu_16(ipv4_hdr->total_length) - ipv4_ihl
		- tcp_hl;
	if (tcp_dl == 0)
		goto end;

	ol_flags = pkt->ol_flags;
	/**
	 * 2. if HW rx checksum offload isn't enabled, recalculate the
	 * checksum in SW. Then, check if the checksum is correct
	 */
	if ((ol_flags & PKT_RX_IP_CKSUM_MASK) !=
			PKT_RX_IP_CKSUM_UNKNOWN) {
		if (ol_flags == PKT_RX_IP_CKSUM_BAD)
			goto end;
	} else {
		ip_cksum = ipv4_hdr->hdr_checksum;
		ipv4_hdr->hdr_checksum = 0;
		ipv4_hdr->hdr_checksum = rte_ipv4_cksum(ipv4_hdr);
		if (ipv4_hdr->hdr_checksum ^ ip_cksum)
			goto end;
	}

	if ((ol_flags & PKT_RX_L4_CKSUM_MASK) !=
			PKT_RX_L4_CKSUM_UNKNOWN) {
		if (ol_flags == PKT_RX_L4_CKSUM_BAD)
			goto end;
	} else {
		tcp_cksum = tcp_hdr->cksum;
		tcp_hdr->cksum = 0;
		tcp_hdr->cksum = rte_ipv4_udptcp_cksum
			(ipv4_hdr, tcp_hdr);
		if (tcp_hdr->cksum ^ tcp_cksum)
			goto end;
	}

	/* 3. search for the corresponding item-list for the packet */
	key.eth_saddr = eth_hdr->s_addr;
	key.eth_daddr = eth_hdr->d_addr;
	key.ip_src_addr = rte_be_to_cpu_32(ipv4_hdr->src_addr);
	key.ip_dst_addr = rte_be_to_cpu_32(ipv4_hdr->dst_addr);
	key.src_port = rte_be_to_cpu_16(tcp_hdr->src_port);
	key.dst_port = rte_be_to_cpu_16(tcp_hdr->dst_port);
	key.recv_ack = rte_be_to_cpu_32(tcp_hdr->recv_ack);
	key.tcp_flags = tcp_hdr->tcp_flags;

	sent_seq = rte_be_to_cpu_32(tcp_hdr->sent_seq);

	if (rte_hash_lookup_data(hash_tbl, &key, (void **)&list) >= 0) {
		ret = gro_tcp4_reassemble(pkt, pkt_idx, sent_seq, list);
	} else {
		/**
		 * fail to find a item-list. Record the sequence number of the
		 * incoming packet's neighbor into its item_list, and insert it
		 * into the hash table.
		 */
		items[0].next_sent_seq = sent_seq + tcp_dl;
		if (unlikely(rte_hash_add_key_data(hash_tbl, &key, item_list)
					!= 0))
			printf("GRO TCP hash insert fail.\n");
		else
			ret = 0;
	}
end:
	return ret;
}
