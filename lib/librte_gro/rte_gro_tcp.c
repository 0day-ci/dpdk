#include "rte_gro_tcp.h"

struct rte_hash *
rte_gro_tcp4_tbl_create(char *name,
		uint32_t nb_entries, uint16_t socket_id)
{
	struct rte_hash_parameters ht_param = {
		.entries = nb_entries,
		.name = name,
		.key_len = sizeof(struct gro_tcp4_pre_rules),
		.hash_func = rte_jhash,
		.hash_func_init_val = 0,
		.socket_id = socket_id,
	};
	struct rte_hash *tbl;

	tbl = rte_hash_create(&ht_param);
	if (tbl == NULL)
		printf("GRO TCP4: allocate hash table fail\n");
	return tbl;
}

/* update TCP IPv4 checksum */
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

/**
 * This function traverses the item-list to find one item that can be
 * merged with the incoming packet. If merge successfully, the merged
 * packets are chained together; if not, insert the incoming packet into
 * the item-list.
 */
static uint64_t
gro_tcp4_reassemble(struct gro_tcp_item_list *list,
		struct rte_mbuf *pkt,
		uint32_t pkt_sent_seq,
		uint32_t pkt_idx)
{
	struct gro_tcp_item *items;
	struct ipv4_hdr *ipv4_hdr1;
	struct tcp_hdr *tcp_hdr1;
	uint16_t ipv4_ihl1, tcp_hl1, tcp_dl1;

	items = list->items;
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

			ipv4_hdr2 = (struct ipv4_hdr *) (rte_pktmbuf_mtod(
						items[i].segment, struct ether_hdr *)
					+ 1);

			/* check if the option fields equal */
			if (tcp_hl1 > sizeof(struct tcp_hdr)) {
				ipv4_ihl2 = IPv4_HDR_LEN(ipv4_hdr2);
				tcp_hdr2 = (struct tcp_hdr *)
					((char *)ipv4_hdr2 + ipv4_ihl2);
				tcp_hl2 = TCP_HDR_LEN(tcp_hdr2);
				if ((tcp_hl1 != tcp_hl2) ||
						(memcmp(tcp_hdr1 + 1, tcp_hdr2 + 1,
								tcp_hl2 - sizeof(struct tcp_hdr))
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
					rte_be_to_cpu_16(ipv4_hdr2->total_length)
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

uint32_t
rte_gro_tcp4_reassemble_burst(struct rte_hash *hash_tbl,
		struct rte_mbuf **pkts,
		const uint32_t nb_pkts)
{
	struct ether_hdr *eth_hdr;
	struct ipv4_hdr *ipv4_hdr;
	struct tcp_hdr *tcp_hdr;
	uint16_t ipv4_ihl, tcp_hl, tcp_dl, tcp_cksum, ip_cksum;
	uint32_t sent_seq;
	struct gro_tcp4_pre_rules key;
	struct gro_tcp_item_list *list;

	/* preallocated items. Each packet has nb_pkts items */
	struct gro_tcp_item items_pool[nb_pkts * nb_pkts];

	struct gro_tcp_info gro_infos[nb_pkts];
	uint64_t ol_flags, idx;
	int ret, is_performed_gro = 0;
	uint32_t nb_after_gro = nb_pkts;

	if (hash_tbl == NULL || pkts == NULL || nb_pkts == 0) {
		printf("GRO TCP4: invalid parameters\n");
		goto end;
	}
	memset(&key, 0, sizeof(struct gro_tcp4_pre_rules));

	for (uint32_t i = 0; i < nb_pkts; i++) {
		gro_infos[i].nb_merged_pkts = 1;

		eth_hdr = rte_pktmbuf_mtod(pkts[i], struct ether_hdr *);
		ipv4_hdr = (struct ipv4_hdr *)(eth_hdr + 1);
		ipv4_ihl = IPv4_HDR_LEN(ipv4_hdr);

		/* 1. check if the packet should be processed */
		if (ipv4_ihl < sizeof(struct ipv4_hdr))
			continue;
		if (ipv4_hdr->next_proto_id != IPPROTO_TCP)
			continue;
		if ((ipv4_hdr->fragment_offset &
					rte_cpu_to_be_16(IPV4_HDR_DF_MASK))
				== 0)
			continue;

		tcp_hdr = (struct tcp_hdr *)((char *)ipv4_hdr + ipv4_ihl);
		tcp_hl = TCP_HDR_LEN(tcp_hdr);
		tcp_dl = rte_be_to_cpu_16(ipv4_hdr->total_length) - ipv4_ihl
			- tcp_hl;
		if (tcp_dl == 0)
			continue;

		ol_flags = pkts[i]->ol_flags;
		/**
		 * 2. if HW rx checksum offload isn't enabled, recalculate the
		 * checksum in SW. Then, check if the checksum is correct
		 */
		if ((ol_flags & PKT_RX_IP_CKSUM_MASK) !=
				PKT_RX_IP_CKSUM_UNKNOWN) {
			if (ol_flags == PKT_RX_IP_CKSUM_BAD)
				continue;
		} else {
			ip_cksum = ipv4_hdr->hdr_checksum;
			ipv4_hdr->hdr_checksum = 0;
			ipv4_hdr->hdr_checksum = rte_ipv4_cksum(ipv4_hdr);
			if (ipv4_hdr->hdr_checksum ^ ip_cksum)
				continue;
		}

		if ((ol_flags & PKT_RX_L4_CKSUM_MASK) !=
				PKT_RX_L4_CKSUM_UNKNOWN) {
			if (ol_flags == PKT_RX_L4_CKSUM_BAD)
				continue;
		} else {
			tcp_cksum = tcp_hdr->cksum;
			tcp_hdr->cksum = 0;
			tcp_hdr->cksum = rte_ipv4_udptcp_cksum
				(ipv4_hdr, tcp_hdr);
			if (tcp_hdr->cksum ^ tcp_cksum)
				continue;
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
		ret = rte_hash_lookup_data(hash_tbl, &key, (void **)&list);

		/* try to reassemble the packet */
		if (ret >= 0) {
			idx = gro_tcp4_reassemble(list, pkts[i], sent_seq, i);
			/* merge successfully, update gro_info */
			if (idx > 0) {
				gro_infos[i].nb_merged_pkts = 0;
				gro_infos[--idx].nb_merged_pkts++;
				nb_after_gro--;
			}
		} else {
			/**
			 * fail to find a item-list. Allocate a new item-list
			 * for the incoming packet and insert it into the hash
			 * table.
			 */
			list = &(gro_infos[i].item_list);
			list->items = &(items_pool[nb_pkts * i]);
			list->nb_item = 1;
			list->items[0].next_sent_seq = sent_seq + tcp_dl;
			list->items[0].segment = pkts[i];
			list->items[0].segment_idx = i;

			if (unlikely(rte_hash_add_key_data(hash_tbl, &key, list)
						!= 0))
				printf("GRO TCP hash insert fail.\n");

			is_performed_gro = 1;
		}
	}

	/**
	 * if there are packets been merged, update their checksum,
	 * and remove useless packet addresses from packet array
	 */
	if (nb_after_gro < nb_pkts) {
		struct rte_mbuf *tmp[nb_pkts];

		memset(tmp, 0, sizeof(struct rte_mbuf *) * nb_pkts);
		/* update checksum */
		for (uint32_t i = 0, j = 0; i < nb_pkts; i++) {
			if (gro_infos[i].nb_merged_pkts > 1)
				gro_tcp4_cksum_update(pkts[i]);
			if (gro_infos[i].nb_merged_pkts != 0)
				tmp[j++] = pkts[i];
		}
		/* update the packet array */
		rte_memcpy(pkts, tmp, nb_pkts * sizeof(struct rte_mbuf *));
	}

	/* if GRO is performed, reset the hash table */
	if (is_performed_gro)
		rte_hash_reset(hash_tbl);
end:
	return nb_after_gro;
}
