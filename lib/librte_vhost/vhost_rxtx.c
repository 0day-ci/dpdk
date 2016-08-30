/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
 *   All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <linux/virtio_net.h>

#include <rte_mbuf.h>
#include <rte_memcpy.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_virtio_net.h>
#include <rte_tcp.h>
#include <rte_udp.h>
#include <rte_sctp.h>
#include <rte_arp.h>

#include "vhost-net.h"

#define MAX_PKT_BURST 32
#define VHOST_LOG_PAGE	4096

static inline void __attribute__((always_inline))
vhost_log_page(uint8_t *log_base, uint64_t page)
{
	log_base[page / 8] |= 1 << (page % 8);
}

static inline void __attribute__((always_inline))
vhost_log_write(struct virtio_net *dev, uint64_t addr, uint64_t len)
{
	uint64_t page;

	if (likely(((dev->features & (1ULL << VHOST_F_LOG_ALL)) == 0) ||
		   !dev->log_base || !len))
		return;

	if (unlikely(dev->log_size <= ((addr + len - 1) / VHOST_LOG_PAGE / 8)))
		return;

	/* To make sure guest memory updates are committed before logging */
	rte_smp_wmb();

	page = addr / VHOST_LOG_PAGE;
	while (page * VHOST_LOG_PAGE < addr + len) {
		vhost_log_page((uint8_t *)(uintptr_t)dev->log_base, page);
		page += 1;
	}
}

static inline void __attribute__((always_inline))
vhost_log_used_vring(struct virtio_net *dev, struct vhost_virtqueue *vq,
		     uint64_t offset, uint64_t len)
{
	vhost_log_write(dev, vq->log_guest_addr + offset, len);
}

static bool
is_valid_virt_queue_idx(uint32_t idx, int is_tx, uint32_t qp_nb)
{
	return (is_tx ^ (idx & 1)) == 0 && idx < qp_nb * VIRTIO_QNUM;
}

static inline void __attribute__((always_inline))
virtio_enqueue_offload(struct rte_mbuf *m_buf, struct virtio_net_hdr *net_hdr)
{
	if (m_buf->ol_flags & PKT_TX_L4_MASK) {
		net_hdr->flags = VIRTIO_NET_HDR_F_NEEDS_CSUM;
		net_hdr->csum_start = m_buf->l2_len + m_buf->l3_len;

		switch (m_buf->ol_flags & PKT_TX_L4_MASK) {
		case PKT_TX_TCP_CKSUM:
			net_hdr->csum_offset = (offsetof(struct tcp_hdr,
						cksum));
			break;
		case PKT_TX_UDP_CKSUM:
			net_hdr->csum_offset = (offsetof(struct udp_hdr,
						dgram_cksum));
			break;
		case PKT_TX_SCTP_CKSUM:
			net_hdr->csum_offset = (offsetof(struct sctp_hdr,
						cksum));
			break;
		}
	} else {
		net_hdr->flags = 0;
		net_hdr->csum_start = 0;
		net_hdr->csum_offset = 0;
	}

	if (m_buf->ol_flags & PKT_TX_TCP_SEG) {
		if (m_buf->ol_flags & PKT_TX_IPV4)
			net_hdr->gso_type = VIRTIO_NET_HDR_GSO_TCPV4;
		else
			net_hdr->gso_type = VIRTIO_NET_HDR_GSO_TCPV6;
		net_hdr->gso_size = m_buf->tso_segsz;
		net_hdr->hdr_len = m_buf->l2_len + m_buf->l3_len
					+ m_buf->l4_len;
	} else {
		net_hdr->gso_type = 0;
		net_hdr->hdr_len = 0;
		net_hdr->gso_size = 0;
	}
}

static inline void __attribute__((always_inline))
update_used_ring(struct vhost_virtqueue *vq, uint32_t desc_chain_head,
		uint32_t desc_chain_len)
{
	vq->shadow_used_ring[vq->shadow_used_idx].id = desc_chain_head;
	vq->shadow_used_ring[vq->shadow_used_idx].len = desc_chain_len;
	vq->shadow_used_idx++;
}

static inline void __attribute__((always_inline))
flush_used_ring(struct virtio_net *dev, struct vhost_virtqueue *vq,
		uint32_t used_idx_start)
{
	if (used_idx_start + vq->shadow_used_idx < vq->size) {
		rte_memcpy(&vq->used->ring[used_idx_start],
				&vq->shadow_used_ring[0],
				vq->shadow_used_idx *
				sizeof(struct vring_used_elem));
		vhost_log_used_vring(dev, vq,
				offsetof(struct vring_used,
					ring[used_idx_start]),
				vq->shadow_used_idx *
				sizeof(struct vring_used_elem));
	} else {
		uint32_t part_1 = vq->size - used_idx_start;
		uint32_t part_2 = vq->shadow_used_idx - part_1;

		rte_memcpy(&vq->used->ring[used_idx_start],
				&vq->shadow_used_ring[0],
				part_1 *
				sizeof(struct vring_used_elem));
		vhost_log_used_vring(dev, vq,
				offsetof(struct vring_used,
					ring[used_idx_start]),
				part_1 *
				sizeof(struct vring_used_elem));
		rte_memcpy(&vq->used->ring[0],
				&vq->shadow_used_ring[part_1],
				part_2 *
				sizeof(struct vring_used_elem));
		vhost_log_used_vring(dev, vq,
				offsetof(struct vring_used,
					ring[0]),
				part_2 *
				sizeof(struct vring_used_elem));
	}
}

static inline uint32_t __attribute__((always_inline))
enqueue_packet(struct virtio_net *dev, struct vhost_virtqueue *vq,
		uint16_t avail_idx, struct rte_mbuf *mbuf,
		uint32_t is_mrg_rxbuf)
{
	struct virtio_net_hdr_mrg_rxbuf *virtio_hdr;
	struct vring_desc *desc;
	uint64_t desc_addr;
	uint32_t desc_chain_head;
	uint32_t desc_chain_len;
	uint32_t desc_current;
	uint32_t desc_offset;
	uint32_t mbuf_len;
	uint32_t mbuf_avail;
	uint32_t copy_len;
	uint32_t extra_buffers = 0;

	/* start with the first mbuf of the packet */
	mbuf_len = rte_pktmbuf_data_len(mbuf);
	mbuf_avail = mbuf_len;

	/* get the current desc */
	desc_current = vq->avail->ring[(vq->last_used_idx) & (vq->size - 1)];
	desc_chain_head = desc_current;
	desc = &vq->desc[desc_current];
	desc_addr = gpa_to_vva(dev, desc->addr);
	if (unlikely(!desc_addr))
		goto error;

	/* handle virtio header */
	virtio_hdr = (struct virtio_net_hdr_mrg_rxbuf *)(uintptr_t)desc_addr;
	virtio_enqueue_offload(mbuf, &(virtio_hdr->hdr));
	if (is_mrg_rxbuf)
		virtio_hdr->num_buffers = extra_buffers + 1;

	vhost_log_write(dev, desc->addr, dev->vhost_hlen);
	PRINT_PACKET(dev, (uintptr_t)desc_addr, dev->vhost_hlen, 0);
	desc_offset = dev->vhost_hlen;
	desc_chain_len = desc_offset;
	desc_addr += desc_offset;

	/* start copy from mbuf to desc */
	while (mbuf_avail || mbuf->next) {
		/* get the next mbuf if the current done */
		if (!mbuf_avail) {
			mbuf = mbuf->next;
			mbuf_len = rte_pktmbuf_data_len(mbuf);
			mbuf_avail = mbuf_len;
		}

		/* get the next desc if the current done */
		if (desc->len <= desc_offset) {
			if (desc->flags & VRING_DESC_F_NEXT) {
				/* go on with the current desc chain */
				desc_offset = 0;
				desc_current = desc->next;
				desc = &vq->desc[desc_current];
				desc_addr = gpa_to_vva(dev, desc->addr);
				if (unlikely(!desc_addr))
					goto error;
			} else if (is_mrg_rxbuf) {
				/* start with the next desc chain */
				update_used_ring(vq, desc_chain_head,
						desc_chain_len);
				vq->last_used_idx++;
				extra_buffers++;
				virtio_hdr->num_buffers++;
				if (avail_idx == vq->last_used_idx)
					goto error;

				desc_current =
					vq->avail->ring[(vq->last_used_idx) &
					(vq->size - 1)];
				desc_chain_head = desc_current;
				desc = &vq->desc[desc_current];
				desc_addr = gpa_to_vva(dev, desc->addr);
				if (unlikely(!desc_addr))
					goto error;

				desc_chain_len = 0;
				desc_offset = 0;
			} else
				goto error;
		}

		/* copy mbuf data */
		copy_len = RTE_MIN(desc->len - desc_offset, mbuf_avail);
		rte_memcpy((void *)(uintptr_t)desc_addr,
				rte_pktmbuf_mtod_offset(mbuf, void *,
					mbuf_len - mbuf_avail),
				copy_len);
		vhost_log_write(dev, desc->addr + desc_offset, copy_len);
		PRINT_PACKET(dev, (uintptr_t)desc_addr, copy_len, 0);
		mbuf_avail -= copy_len;
		desc_offset += copy_len;
		desc_addr += copy_len;
		desc_chain_len += copy_len;
	}

	update_used_ring(vq, desc_chain_head, desc_chain_len);
	vq->last_used_idx++;

	return 0;

error:
	/* rollback on any error if last_used_idx update on-the-fly */
	vq->last_used_idx -= extra_buffers;

	return 1;
}

static inline void __attribute__((always_inline))
notify_guest(struct virtio_net *dev, struct vhost_virtqueue *vq)
{
	rte_smp_wmb();
	vq->used->idx = vq->last_used_idx;
	vhost_log_used_vring(dev, vq, offsetof(struct vring_used, idx),
			sizeof(vq->used->idx));
	rte_mb();
	if (!(vq->avail->flags & VRING_AVAIL_F_NO_INTERRUPT)
			&& (vq->callfd >= 0))
		eventfd_write(vq->callfd, (eventfd_t)1);
}

uint16_t
rte_vhost_enqueue_burst(int vid, uint16_t queue_id,
	struct rte_mbuf **pkts, uint16_t count)
{
	struct vhost_virtqueue *vq;
	struct virtio_net *dev;
	uint32_t used_idx_start;
	uint32_t pkt_left = count;
	uint32_t pkt_idx = 0;
	uint32_t pkt_sent = 0;
	uint32_t is_mrg_rxbuf = 0;
	uint16_t avail_idx;

	if (unlikely(!pkt_left))
		return 0;

	pkt_left = RTE_MIN((uint32_t)MAX_PKT_BURST, pkt_left);

	dev = get_device(vid);
	if (unlikely(!dev))
		return 0;

	if (unlikely(!is_valid_virt_queue_idx(queue_id, 0, dev->virt_qp_nb)))
		return 0;

	vq = dev->virtqueue[queue_id];
	if (unlikely(!vq->enabled))
		return 0;

	if (dev->features & (1ULL << VIRTIO_NET_F_MRG_RXBUF))
		is_mrg_rxbuf = 1;

	/* start enqueuing packets 1 by 1 */
	vq->shadow_used_idx = 0;
	used_idx_start = vq->last_used_idx & (vq->size - 1);
	avail_idx = *((volatile uint16_t *)&vq->avail->idx);
	while (pkt_left && avail_idx != vq->last_used_idx) {
		/* prefetch the next desc */
		if (pkt_left > 1 && avail_idx != vq->last_used_idx + 1)
			rte_prefetch0(&vq->desc[vq->avail->ring[
					(vq->last_used_idx + 1) &
					(vq->size - 1)]]);

		if (enqueue_packet(dev, vq, avail_idx, pkts[pkt_idx],
					is_mrg_rxbuf))
			break;

		pkt_idx++;
		pkt_sent++;
		pkt_left--;
	}

	/* batch update used ring for better performance */
	if (likely(vq->shadow_used_idx > 0))
		flush_used_ring(dev, vq, used_idx_start);

	/* update used idx and kick the guest if necessary */
	if (pkt_sent)
		notify_guest(dev, vq);

	return pkt_sent;
}

static void
parse_ethernet(struct rte_mbuf *m, uint16_t *l4_proto, void **l4_hdr)
{
	struct ipv4_hdr *ipv4_hdr;
	struct ipv6_hdr *ipv6_hdr;
	void *l3_hdr = NULL;
	struct ether_hdr *eth_hdr;
	uint16_t ethertype;

	eth_hdr = rte_pktmbuf_mtod(m, struct ether_hdr *);

	m->l2_len = sizeof(struct ether_hdr);
	ethertype = rte_be_to_cpu_16(eth_hdr->ether_type);

	if (ethertype == ETHER_TYPE_VLAN) {
		struct vlan_hdr *vlan_hdr = (struct vlan_hdr *)(eth_hdr + 1);

		m->l2_len += sizeof(struct vlan_hdr);
		ethertype = rte_be_to_cpu_16(vlan_hdr->eth_proto);
	}

	l3_hdr = (char *)eth_hdr + m->l2_len;

	switch (ethertype) {
	case ETHER_TYPE_IPv4:
		ipv4_hdr = (struct ipv4_hdr *)l3_hdr;
		*l4_proto = ipv4_hdr->next_proto_id;
		m->l3_len = (ipv4_hdr->version_ihl & 0x0f) * 4;
		*l4_hdr = (char *)l3_hdr + m->l3_len;
		m->ol_flags |= PKT_TX_IPV4;
		break;
	case ETHER_TYPE_IPv6:
		ipv6_hdr = (struct ipv6_hdr *)l3_hdr;
		*l4_proto = ipv6_hdr->proto;
		m->l3_len = sizeof(struct ipv6_hdr);
		*l4_hdr = (char *)l3_hdr + m->l3_len;
		m->ol_flags |= PKT_TX_IPV6;
		break;
	default:
		m->l3_len = 0;
		*l4_proto = 0;
		break;
	}
}

static inline void __attribute__((always_inline))
vhost_dequeue_offload(struct virtio_net_hdr *hdr, struct rte_mbuf *m)
{
	uint16_t l4_proto = 0;
	void *l4_hdr = NULL;
	struct tcp_hdr *tcp_hdr = NULL;

	parse_ethernet(m, &l4_proto, &l4_hdr);
	if (hdr->flags == VIRTIO_NET_HDR_F_NEEDS_CSUM) {
		if (hdr->csum_start == (m->l2_len + m->l3_len)) {
			switch (hdr->csum_offset) {
			case (offsetof(struct tcp_hdr, cksum)):
				if (l4_proto == IPPROTO_TCP)
					m->ol_flags |= PKT_TX_TCP_CKSUM;
				break;
			case (offsetof(struct udp_hdr, dgram_cksum)):
				if (l4_proto == IPPROTO_UDP)
					m->ol_flags |= PKT_TX_UDP_CKSUM;
				break;
			case (offsetof(struct sctp_hdr, cksum)):
				if (l4_proto == IPPROTO_SCTP)
					m->ol_flags |= PKT_TX_SCTP_CKSUM;
				break;
			default:
				break;
			}
		}
	}

	if (hdr->gso_type != VIRTIO_NET_HDR_GSO_NONE) {
		switch (hdr->gso_type & ~VIRTIO_NET_HDR_GSO_ECN) {
		case VIRTIO_NET_HDR_GSO_TCPV4:
		case VIRTIO_NET_HDR_GSO_TCPV6:
			tcp_hdr = (struct tcp_hdr *)l4_hdr;
			m->ol_flags |= PKT_TX_TCP_SEG;
			m->tso_segsz = hdr->gso_size;
			m->l4_len = (tcp_hdr->data_off & 0xf0) >> 2;
			break;
		default:
			RTE_LOG(WARNING, VHOST_DATA,
				"unsupported gso type %u.\n", hdr->gso_type);
			break;
		}
	}
}

#define RARP_PKT_SIZE	64

static int
make_rarp_packet(struct rte_mbuf *rarp_mbuf, const struct ether_addr *mac)
{
	struct ether_hdr *eth_hdr;
	struct arp_hdr  *rarp;

	if (rarp_mbuf->buf_len < 64) {
		RTE_LOG(WARNING, VHOST_DATA,
			"failed to make RARP; mbuf size too small %u (< %d)\n",
			rarp_mbuf->buf_len, RARP_PKT_SIZE);
		return -1;
	}

	/* Ethernet header. */
	eth_hdr = rte_pktmbuf_mtod_offset(rarp_mbuf, struct ether_hdr *, 0);
	memset(eth_hdr->d_addr.addr_bytes, 0xff, ETHER_ADDR_LEN);
	ether_addr_copy(mac, &eth_hdr->s_addr);
	eth_hdr->ether_type = htons(ETHER_TYPE_RARP);

	/* RARP header. */
	rarp = (struct arp_hdr *)(eth_hdr + 1);
	rarp->arp_hrd = htons(ARP_HRD_ETHER);
	rarp->arp_pro = htons(ETHER_TYPE_IPv4);
	rarp->arp_hln = ETHER_ADDR_LEN;
	rarp->arp_pln = 4;
	rarp->arp_op  = htons(ARP_OP_REVREQUEST);

	ether_addr_copy(mac, &rarp->arp_data.arp_sha);
	ether_addr_copy(mac, &rarp->arp_data.arp_tha);
	memset(&rarp->arp_data.arp_sip, 0x00, 4);
	memset(&rarp->arp_data.arp_tip, 0x00, 4);

	rarp_mbuf->pkt_len  = rarp_mbuf->data_len = RARP_PKT_SIZE;

	return 0;
}

static inline int __attribute__((always_inline))
copy_desc_to_mbuf(struct virtio_net *dev, struct vhost_virtqueue *vq,
		  struct rte_mbuf *m, uint16_t desc_idx,
		  struct rte_mempool *mbuf_pool)
{
	struct vring_desc *desc;
	uint64_t desc_addr;
	uint32_t desc_avail, desc_offset;
	uint32_t mbuf_avail, mbuf_offset;
	uint32_t cpy_len;
	struct rte_mbuf *cur = m, *prev = m;
	struct virtio_net_hdr *hdr;
	/* A counter to avoid desc dead loop chain */
	uint32_t nr_desc = 1;

	desc = &vq->desc[desc_idx];
	if (unlikely(desc->len < dev->vhost_hlen))
		return -1;

	desc_addr = gpa_to_vva(dev, desc->addr);
	if (unlikely(!desc_addr))
		return -1;

	hdr = (struct virtio_net_hdr *)((uintptr_t)desc_addr);
	rte_prefetch0(hdr);

	/*
	 * A virtio driver normally uses at least 2 desc buffers
	 * for Tx: the first for storing the header, and others
	 * for storing the data.
	 */
	if (likely((desc->len == dev->vhost_hlen) &&
		   (desc->flags & VRING_DESC_F_NEXT) != 0)) {
		desc = &vq->desc[desc->next];

		desc_addr = gpa_to_vva(dev, desc->addr);
		if (unlikely(!desc_addr))
			return -1;

		rte_prefetch0((void *)(uintptr_t)desc_addr);

		desc_offset = 0;
		desc_avail  = desc->len;
		nr_desc    += 1;

		PRINT_PACKET(dev, (uintptr_t)desc_addr, desc->len, 0);
	} else {
		desc_avail  = desc->len - dev->vhost_hlen;
		desc_offset = dev->vhost_hlen;
	}

	mbuf_offset = 0;
	mbuf_avail  = m->buf_len - RTE_PKTMBUF_HEADROOM;
	while (1) {
		cpy_len = RTE_MIN(desc_avail, mbuf_avail);
		rte_memcpy(rte_pktmbuf_mtod_offset(cur, void *, mbuf_offset),
			(void *)((uintptr_t)(desc_addr + desc_offset)),
			cpy_len);

		mbuf_avail  -= cpy_len;
		mbuf_offset += cpy_len;
		desc_avail  -= cpy_len;
		desc_offset += cpy_len;

		/* This desc reaches to its end, get the next one */
		if (desc_avail == 0) {
			if ((desc->flags & VRING_DESC_F_NEXT) == 0)
				break;

			if (unlikely(desc->next >= vq->size ||
				     ++nr_desc > vq->size))
				return -1;
			desc = &vq->desc[desc->next];

			desc_addr = gpa_to_vva(dev, desc->addr);
			if (unlikely(!desc_addr))
				return -1;

			rte_prefetch0((void *)(uintptr_t)desc_addr);

			desc_offset = 0;
			desc_avail  = desc->len;

			PRINT_PACKET(dev, (uintptr_t)desc_addr, desc->len, 0);
		}

		/*
		 * This mbuf reaches to its end, get a new one
		 * to hold more data.
		 */
		if (mbuf_avail == 0) {
			cur = rte_pktmbuf_alloc(mbuf_pool);
			if (unlikely(cur == NULL)) {
				RTE_LOG(ERR, VHOST_DATA, "Failed to "
					"allocate memory for mbuf.\n");
				return -1;
			}

			prev->next = cur;
			prev->data_len = mbuf_offset;
			m->nb_segs += 1;
			m->pkt_len += mbuf_offset;
			prev = cur;

			mbuf_offset = 0;
			mbuf_avail  = cur->buf_len - RTE_PKTMBUF_HEADROOM;
		}
	}

	prev->data_len = mbuf_offset;
	m->pkt_len    += mbuf_offset;

	if (hdr->flags != 0 || hdr->gso_type != VIRTIO_NET_HDR_GSO_NONE)
		vhost_dequeue_offload(hdr, m);

	return 0;
}

uint16_t
rte_vhost_dequeue_burst(int vid, uint16_t queue_id,
	struct rte_mempool *mbuf_pool, struct rte_mbuf **pkts, uint16_t count)
{
	struct virtio_net *dev;
	struct rte_mbuf *rarp_mbuf = NULL;
	struct vhost_virtqueue *vq;
	uint32_t desc_indexes[MAX_PKT_BURST];
	uint32_t used_idx;
	uint32_t i = 0;
	uint16_t free_entries;
	uint16_t avail_idx;

	dev = get_device(vid);
	if (!dev)
		return 0;

	if (unlikely(!is_valid_virt_queue_idx(queue_id, 1, dev->virt_qp_nb))) {
		RTE_LOG(ERR, VHOST_DATA, "(%d) %s: invalid virtqueue idx %d.\n",
			dev->vid, __func__, queue_id);
		return 0;
	}

	vq = dev->virtqueue[queue_id];
	if (unlikely(vq->enabled == 0))
		return 0;

	/*
	 * Construct a RARP broadcast packet, and inject it to the "pkts"
	 * array, to looks like that guest actually send such packet.
	 *
	 * Check user_send_rarp() for more information.
	 */
	if (unlikely(rte_atomic16_cmpset((volatile uint16_t *)
					 &dev->broadcast_rarp.cnt, 1, 0))) {
		rarp_mbuf = rte_pktmbuf_alloc(mbuf_pool);
		if (rarp_mbuf == NULL) {
			RTE_LOG(ERR, VHOST_DATA,
				"Failed to allocate memory for mbuf.\n");
			return 0;
		}

		if (make_rarp_packet(rarp_mbuf, &dev->mac)) {
			rte_pktmbuf_free(rarp_mbuf);
			rarp_mbuf = NULL;
		} else {
			count -= 1;
		}
	}

	avail_idx =  *((volatile uint16_t *)&vq->avail->idx);
	free_entries = avail_idx - vq->last_used_idx;
	if (free_entries == 0)
		goto out;

	LOG_DEBUG(VHOST_DATA, "(%d) %s\n", dev->vid, __func__);

	/* Prefetch available ring to retrieve head indexes. */
	used_idx = vq->last_used_idx & (vq->size - 1);
	rte_prefetch0(&vq->avail->ring[used_idx]);
	rte_prefetch0(&vq->used->ring[used_idx]);

	count = RTE_MIN(count, MAX_PKT_BURST);
	count = RTE_MIN(count, free_entries);
	LOG_DEBUG(VHOST_DATA, "(%d) about to dequeue %u buffers\n",
			dev->vid, count);

	/* Retrieve all of the head indexes first to avoid caching issues. */
	for (i = 0; i < count; i++) {
		used_idx = (vq->last_used_idx + i) & (vq->size - 1);
		desc_indexes[i] = vq->avail->ring[used_idx];

		vq->used->ring[used_idx].id  = desc_indexes[i];
		vq->used->ring[used_idx].len = 0;
		vhost_log_used_vring(dev, vq,
				offsetof(struct vring_used, ring[used_idx]),
				sizeof(vq->used->ring[used_idx]));
	}

	/* Prefetch descriptor index. */
	rte_prefetch0(&vq->desc[desc_indexes[0]]);
	for (i = 0; i < count; i++) {
		int err;

		if (likely(i + 1 < count))
			rte_prefetch0(&vq->desc[desc_indexes[i + 1]]);

		pkts[i] = rte_pktmbuf_alloc(mbuf_pool);
		if (unlikely(pkts[i] == NULL)) {
			RTE_LOG(ERR, VHOST_DATA,
				"Failed to allocate memory for mbuf.\n");
			break;
		}
		err = copy_desc_to_mbuf(dev, vq, pkts[i], desc_indexes[i],
					mbuf_pool);
		if (unlikely(err)) {
			rte_pktmbuf_free(pkts[i]);
			break;
		}
	}

	rte_smp_wmb();
	rte_smp_rmb();
	vq->used->idx += i;
	vq->last_used_idx += i;
	vhost_log_used_vring(dev, vq, offsetof(struct vring_used, idx),
			sizeof(vq->used->idx));

	/* Kick guest if required. */
	if (!(vq->avail->flags & VRING_AVAIL_F_NO_INTERRUPT)
			&& (vq->callfd >= 0))
		eventfd_write(vq->callfd, (eventfd_t)1);

out:
	if (unlikely(rarp_mbuf != NULL)) {
		/*
		 * Inject it to the head of "pkts" array, so that switch's mac
		 * learning table will get updated first.
		 */
		memmove(&pkts[1], pkts, i * sizeof(struct rte_mbuf *));
		pkts[0] = rarp_mbuf;
		i += 1;
	}

	return i;
}
