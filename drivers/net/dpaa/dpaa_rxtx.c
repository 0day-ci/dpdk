/*-
 *   BSD LICENSE
 *
 *   Copyright 2016 Freescale Semiconductor, Inc. All rights reserved.
 *   Copyright 2017 NXP. All rights reserved.
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
 *     * Neither the name of  Freescale Semiconductor, Inc nor the names of its
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

/* System headers */
#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <sched.h>
#include <pthread.h>

#include <rte_config.h>
#include <rte_byteorder.h>
#include <rte_common.h>
#include <rte_interrupts.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_pci.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_tailq.h>
#include <rte_eal.h>
#include <rte_alarm.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_atomic.h>
#include <rte_malloc.h>
#include <rte_ring.h>
#include <rte_ip.h>
#include <rte_tcp.h>
#include <rte_udp.h>

#include "dpaa_ethdev.h"
#include "dpaa_rxtx.h"
#include <rte_dpaa_bus.h>
#include <dpaa_mempool.h>

#include <fsl_usd.h>
#include <fsl_qman.h>
#include <fsl_bman.h>
#include <of.h>
#include <netcfg.h>

#define DPAA_MBUF_TO_CONTIG_FD(_mbuf, _fd, _bpid) \
	do { \
		(_fd)->cmd = 0; \
		(_fd)->opaque_addr = 0; \
		(_fd)->opaque = QM_FD_CONTIG << DPAA_FD_FORMAT_SHIFT; \
		(_fd)->opaque |= ((_mbuf)->data_off) << DPAA_FD_OFFSET_SHIFT; \
		(_fd)->opaque |= (_mbuf)->pkt_len; \
		(_fd)->addr = (_mbuf)->buf_physaddr; \
		(_fd)->bpid = _bpid; \
	} while (0)

static inline void dpaa_slow_parsing(struct rte_mbuf *m __rte_unused,
				     uint64_t prs __rte_unused)
{
	DPAA_RX_LOG(DEBUG, "Slow parsing");
	/*TBD:XXX: to be implemented*/
}

static inline void dpaa_eth_packet_info(struct rte_mbuf *m,
					uint64_t fd_virt_addr)
{
	struct annotations_t *annot = GET_ANNOTATIONS(fd_virt_addr);
	uint64_t prs = *((uint64_t *)(&annot->parse)) & DPAA_PARSE_MASK;

	DPAA_RX_LOG(DEBUG, " Parsing mbuf: %p with annotations: %p", m, annot);

	switch (prs) {
	case DPAA_PKT_TYPE_NONE:
		m->packet_type = 0;
		break;
	case DPAA_PKT_TYPE_ETHER:
		m->packet_type = RTE_PTYPE_L2_ETHER;
		break;
	case DPAA_PKT_TYPE_IPV4:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4;
		break;
	case DPAA_PKT_TYPE_IPV6:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6;
		break;
	case DPAA_PKT_TYPE_IPV4_FRAG:
	case DPAA_PKT_TYPE_IPV4_FRAG_UDP:
	case DPAA_PKT_TYPE_IPV4_FRAG_TCP:
	case DPAA_PKT_TYPE_IPV4_FRAG_SCTP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_FRAG;
		break;
	case DPAA_PKT_TYPE_IPV6_FRAG:
	case DPAA_PKT_TYPE_IPV6_FRAG_UDP:
	case DPAA_PKT_TYPE_IPV6_FRAG_TCP:
	case DPAA_PKT_TYPE_IPV6_FRAG_SCTP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_FRAG;
		break;
	case DPAA_PKT_TYPE_IPV4_EXT:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4_EXT;
		break;
	case DPAA_PKT_TYPE_IPV6_EXT:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6_EXT;
		break;
	case DPAA_PKT_TYPE_IPV4_TCP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_TCP;
		break;
	case DPAA_PKT_TYPE_IPV6_TCP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_TCP;
		break;
	case DPAA_PKT_TYPE_IPV4_UDP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_UDP;
		break;
	case DPAA_PKT_TYPE_IPV6_UDP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_UDP;
		break;
	case DPAA_PKT_TYPE_IPV4_EXT_UDP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_L4_UDP;
		break;
	case DPAA_PKT_TYPE_IPV6_EXT_UDP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6_EXT | RTE_PTYPE_L4_UDP;
		break;
	case DPAA_PKT_TYPE_IPV4_EXT_TCP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_L4_TCP;
		break;
	case DPAA_PKT_TYPE_IPV6_EXT_TCP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6_EXT | RTE_PTYPE_L4_TCP;
		break;
	case DPAA_PKT_TYPE_IPV4_SCTP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_SCTP;
		break;
	case DPAA_PKT_TYPE_IPV6_SCTP:
		m->packet_type = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_SCTP;
		break;
	/* More switch cases can be added */
	default:
		dpaa_slow_parsing(m, prs);
	}

	m->tx_offload = annot->parse.ip_off[0];
	m->tx_offload |= (annot->parse.l4_off - annot->parse.ip_off[0])
					<< DPAA_PKT_L3_LEN_SHIFT;

	/* Set the hash values */
	m->hash.rss = (uint32_t)(rte_be_to_cpu_64(annot->hash));
	m->ol_flags = PKT_RX_RSS_HASH;
	/* All packets with Bad checksum are dropped by interface (and
	 * corresponding notification issued to RX error queues).
	 */
	m->ol_flags |= PKT_RX_IP_CKSUM_GOOD;

	/* Check if Vlan is present */
	if (prs & DPAA_PARSE_VLAN_MASK)
		m->ol_flags |= PKT_RX_VLAN_PKT;
	/* Packet received without stripping the vlan */
}

static inline struct rte_mbuf *dpaa_eth_fd_to_mbuf(struct qm_fd *fd,
							uint32_t ifid)
{
	struct pool_info_entry *bp_info = DPAA_BPID_TO_POOL_INFO(fd->bpid);
	struct rte_mbuf *mbuf;
	void *ptr;
	uint16_t offset =
		(fd->opaque & DPAA_FD_OFFSET_MASK) >> DPAA_FD_OFFSET_SHIFT;
	uint32_t length = fd->opaque & DPAA_FD_LENGTH_MASK;

	DPAA_RX_LOG(DEBUG, " FD--->MBUF");

	/* Ignoring case when format != qm_fd_contig */
	ptr = rte_dpaa_mem_ptov(fd->addr);
	/* Ignoring case when ptr would be NULL. That is only possible incase
	 * of a corrupted packet
	 */

	mbuf = (struct rte_mbuf *)((char *)ptr - bp_info->meta_data_size);
	/* Prefetch the Parse results and packet data to L1 */
	rte_prefetch0((void *)((uint8_t *)ptr + DEFAULT_RX_ICEOF));
	rte_prefetch0((void *)((uint8_t *)ptr + offset));

	mbuf->data_off = offset;
	mbuf->data_len = length;
	mbuf->pkt_len = length;

	mbuf->port = ifid;
	mbuf->nb_segs = 1;
	mbuf->ol_flags = 0;
	mbuf->next = NULL;
	rte_mbuf_refcnt_set(mbuf, 1);
	dpaa_eth_packet_info(mbuf, (uint64_t)mbuf->buf_addr);

	return mbuf;
}

uint16_t dpaa_eth_queue_rx(void *q,
			   struct rte_mbuf **bufs,
			   uint16_t nb_bufs)
{
	struct qman_fq *fq = q;
	struct qm_dqrr_entry *dq;
	uint32_t num_rx = 0, ifid = ((struct dpaa_if *)fq->dpaa_intf)->ifid;
	int ret;

	ret = rte_dpaa_portal_init((void *)0);
	if (ret) {
		DPAA_PMD_ERR("Failure in affining portal");
		return 0;
	}

	ret = qman_set_vdq(fq, (nb_bufs > DPAA_MAX_DEQUEUE_NUM_FRAMES) ?
				DPAA_MAX_DEQUEUE_NUM_FRAMES : nb_bufs);
	if (ret)
		return 0;

	do {
		dq = qman_dequeue(fq);
		if (!dq)
			continue;
		bufs[num_rx++] = dpaa_eth_fd_to_mbuf(&dq->fd, ifid);
		qman_dqrr_consume(fq, dq);
	} while (fq->flags & QMAN_FQ_STATE_VDQCR);

	return num_rx;
}

static void *dpaa_get_pktbuf(struct pool_info_entry *bp_info)
{
	int ret;
	uint64_t buf = 0;
	struct bm_buffer bufs;

	ret = bman_acquire(bp_info->bp, &bufs, 1, 0);
	if (ret <= 0) {
		DPAA_PMD_WARN("Failed to allocate buffers %d", ret);
		return (void *)buf;
	}

	DPAA_RX_LOG(DEBUG, "got buffer 0x%lx from pool %d",
		    (uint64_t)bufs.addr, bufs.bpid);

	buf = (uint64_t)rte_dpaa_mem_ptov(bufs.addr) - bp_info->meta_data_size;
	if (!buf)
		goto out;

out:
	return (void *)buf;
}

static struct rte_mbuf *dpaa_get_dmable_mbuf(struct rte_mbuf *mbuf,
					     struct dpaa_if *dpaa_intf)
{
	struct rte_mbuf *dpaa_mbuf;

	/* allocate pktbuffer on bpid for dpaa port */
	dpaa_mbuf = dpaa_get_pktbuf(dpaa_intf->bp_info);
	if (!dpaa_mbuf)
		return NULL;

	memcpy((uint8_t *)(dpaa_mbuf->buf_addr) + mbuf->data_off, (void *)
		((uint8_t *)(mbuf->buf_addr) + mbuf->data_off), mbuf->pkt_len);

	/* Copy only the required fields */
	dpaa_mbuf->data_off = mbuf->data_off;
	dpaa_mbuf->pkt_len = mbuf->pkt_len;
	dpaa_mbuf->ol_flags = mbuf->ol_flags;
	dpaa_mbuf->packet_type = mbuf->packet_type;
	dpaa_mbuf->tx_offload = mbuf->tx_offload;
	rte_pktmbuf_free(mbuf);
	return dpaa_mbuf;
}

uint16_t
dpaa_eth_queue_tx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	struct rte_mbuf *mbuf, *mi = NULL;
	struct rte_mempool *mp;
	struct pool_info_entry *bp_info;
	struct qm_fd fd_arr[MAX_TX_RING_SLOTS];
	uint32_t frames_to_send, loop, i = 0;
	int ret;

	ret = rte_dpaa_portal_init((void *)0);
	if (ret) {
		DPAA_PMD_ERR("Failure in affining portal");
		return 0;
	}

	DPAA_TX_LOG(DEBUG, "Transmitting %d buffers on queue: %p", nb_bufs, q);

	while (nb_bufs) {
		frames_to_send = (nb_bufs >> 3) ? MAX_TX_RING_SLOTS : nb_bufs;
		for (loop = 0; loop < frames_to_send; loop++, i++) {
			mbuf = bufs[i];
			if (RTE_MBUF_DIRECT(mbuf)) {
				mp = mbuf->pool;
			} else {
				mi = rte_mbuf_from_indirect(mbuf);
				mp = mi->pool;
			}

			bp_info = DPAA_MEMPOOL_TO_POOL_INFO(mp);
			if (mp->ops_index == bp_info->dpaa_ops_index) {
				DPAA_TX_LOG(DEBUG, "BMAN offloaded buffer, "
					    "mbuf: %p", mbuf);
				if (mbuf->nb_segs == 1) {
					if (RTE_MBUF_DIRECT(mbuf)) {
						if (rte_mbuf_refcnt_read(mbuf) > 1) {
							DPAA_MBUF_TO_CONTIG_FD(mbuf,
								&fd_arr[loop], 0xff);
							rte_mbuf_refcnt_update(mbuf, -1);
						} else {
							DPAA_MBUF_TO_CONTIG_FD(mbuf,
								&fd_arr[loop], bp_info->bpid);
						}
					} else {
						if (rte_mbuf_refcnt_read(mi) > 1) {
							DPAA_MBUF_TO_CONTIG_FD(mbuf,
								&fd_arr[loop], 0xff);
						} else {
							rte_mbuf_refcnt_update(mi, 1);
							DPAA_MBUF_TO_CONTIG_FD(mbuf,
								&fd_arr[loop], bp_info->bpid);
						}
						rte_pktmbuf_free(mbuf);
					}
				} else {
					DPAA_PMD_DEBUG("Number of Segments not supported");
					/* Set frames_to_send & nb_bufs so that
					 * packets are transmitted till
					 * previous frame.
					 */
					frames_to_send = loop;
					nb_bufs = loop;
					goto send_pkts;
				}
			} else {
				struct qman_fq *txq = q;
				struct dpaa_if *dpaa_intf = txq->dpaa_intf;

				DPAA_TX_LOG(DEBUG, "Non-BMAN offloaded buffer."
					    "Allocating an offloaded buffer");
				mbuf = dpaa_get_dmable_mbuf(mbuf, dpaa_intf);
				if (!mbuf) {
					DPAA_TX_LOG(DEBUG, "no dpaa buffers.");
					/* Set frames_to_send & nb_bufs so that
					 * packets are transmitted till
					 * previous frame.
					 */
					frames_to_send = loop;
					nb_bufs = loop;
					goto send_pkts;
				}

				DPAA_MBUF_TO_CONTIG_FD(mbuf, &fd_arr[loop],
						dpaa_intf->bp_info->bpid);
			}
		}

send_pkts:
		loop = 0;
		while (loop < frames_to_send) {
			loop += qman_enqueue_multi(q, &fd_arr[loop],
					frames_to_send - loop);
		}
		nb_bufs -= frames_to_send;
	}

	DPAA_TX_LOG(DEBUG, "Transmitted %d buffers on queue: %p", i, q);

	return i;
}

uint16_t dpaa_eth_tx_drop_all(void *q  __rte_unused,
			      struct rte_mbuf **bufs __rte_unused,
		uint16_t nb_bufs __rte_unused)
{
	DPAA_TX_LOG(DEBUG, "Drop all packets");

	/* Drop all incoming packets. No need to free packets here
	 * because the rte_eth f/w frees up the packets through tx_buffer
	 * callback in case this functions returns count less than nb_bufs
	 */
	return 0;
}
