/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Napatech A/S. All rights reserved.
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
 *     * Neither the name of Napatech nor the names of its
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

#include <time.h>
#include <rte_mbuf.h>
#include <rte_ethdev.h>
#include <rte_malloc.h>
#include <rte_memcpy.h>
#include <rte_string_fns.h>
#include <rte_cycles.h>
#include <rte_kvargs.h>
#include <rte_dev.h>
#include <net/if.h>
#include <nt.h>

#define ETH_NTNIC_PORT_ARG            "port"
#define ETH_NTNIC_PORTEND_ARG         "portend"
#define ETH_NTNIC_RXQUEUES_ARG        "rxqs"
#define ETH_NTNIC_TXQUEUES_ARG        "txqs"
#define ETH_NTNIC_STREAMID_ARG        "streamid"
#define ETH_NTNIC_HASH_ARG            "hash"

#define HW_MAX_PKT_LEN  10000
#define HW_MTU    (HW_MAX_PKT_LEN - ETHER_HDR_LEN - ETHER_CRC_LEN)
/*
 * hash = <value> (1..3)
 * 1 - RoundRobin
 * 2 - 2tupleSorted
 * 3 - 5tupleSorted
 */


#define MAX_RX_QUEUES 64
#define MAX_TX_QUEUES 64
#define MAX_NTNIC_PORTS 32

static char errorBuffer[1024];

struct array_s {
       uint32_t value[MAX_RX_QUEUES];
       int count;
};

static volatile uint16_t port_locks[MAX_NTNIC_PORTS];

struct ntnic_rx_queue {
       NtNetStreamRx_t        pNetRx[MAX_RX_QUEUES];   /* NT Rx streams */
       int                    curNetRx_idx;                    /* Current Rx stream index */
       struct rte_mempool    *mb_pool;                                 /* mbuf memory pool */
       uint16_t               buf_size;                                /* Size of data area in mbuf */
       NtNetBuf_t             pSeg;                                    /* The current NT data segment we are working with */
       struct NtNetBuf_s      pkt;                                             /* The current packet */
       volatile unsigned long rx_pkts;                                 /* Rx packet statistics */
       volatile unsigned long err_pkts;                                /* Rx error packet statistics */
       struct array_s         astreamids;                              /* Array of NT streamids to read data from */
       int                    enabled;                                 /* Enabling/disabling of this queue */
};

struct ntnic_tx_queue {
       NtNetStreamTx_t        pNetTx;                                  /* NT Tx stream */
       volatile unsigned long tx_pkts;                                 /* Tx packet statistics */
       volatile unsigned long err_pkts;                                /* Tx error packet statistics */
       volatile uint16_t     *plock;                                   /* Per port transmit atomic lock */
       uint32_t               port;                                    /* Tx port for this queue */
       int                    enabled;                                 /* Enabling/disabling of this queue */
       int                    fcs_add;                                 /* If port HW needs added room for FCS */
};

struct pmd_internals {
       struct ntnic_rx_queue rxq[MAX_RX_QUEUES];               /* Array of Rx queues configured */
       struct ntnic_tx_queue txq[MAX_TX_QUEUES];               /* Array of Tx queues configured */
       int                 if_index;                                   /* Interface index always 0 - no bonding */
       unsigned int        nb_rx_queues;                               /* Number of Rx queues configured */
       unsigned int        nb_tx_queues;                               /* Number of Tx queues configured */
       int                 MAC_50G;                                    /*  True if 50G ports */
};

static const char *valid_arguments[] = {
       ETH_NTNIC_PORT_ARG,
       ETH_NTNIC_PORTEND_ARG,
       ETH_NTNIC_RXQUEUES_ARG,
       ETH_NTNIC_TXQUEUES_ARG,
       ETH_NTNIC_STREAMID_ARG,
       ETH_NTNIC_HASH_ARG,
       NULL
};

static struct ether_addr eth_addr[MAX_NTNIC_PORTS];
static const char *drivername = "NTNIC PMD";

static int
eth_ntnic_rx_jumbo(struct rte_mempool *mb_pool,
       struct rte_mbuf *mbuf,
       const u_char *data,
       uint16_t data_len)
{
       struct rte_mbuf *m = mbuf;

       /* Copy the first segment. */
       uint16_t len = rte_pktmbuf_tailroom(mbuf);

       rte_memcpy(rte_pktmbuf_append(mbuf, len), data, len);
       data_len -= len;
       data += len;

       while (data_len > 0) {
               /* Allocate next mbuf and point to that. */
               m->next = rte_pktmbuf_alloc(mb_pool);

               if (unlikely(!m->next))
                       return -1;

               m = m->next;

               /* Headroom is not needed in chained mbufs. */
               rte_pktmbuf_prepend(m, rte_pktmbuf_headroom(m));
               m->pkt_len = 0;
               m->data_len = 0;

               /* Copy next segment. */
               len = RTE_MIN(rte_pktmbuf_tailroom(m), data_len);
               rte_memcpy(rte_pktmbuf_append(m, len), data, len);

               mbuf->nb_segs++;
               data_len -= len;
               data += len;
       }

       return mbuf->nb_segs;
}



static uint16_t
eth_ntnic_rx(void *queue,
       struct rte_mbuf **bufs,
       uint16_t nb_pkts)
{
       unsigned i;
       struct rte_mbuf *mbuf;
       struct ntnic_rx_queue *rx_q = queue;
       uint16_t num_rx = 0;
    uint16_t data_len;

       if (unlikely(rx_q->pNetRx[rx_q->curNetRx_idx] == NULL || nb_pkts == 0))
               return 0;

       /* Do we have any data segment */
       if (rx_q->pSeg == NULL) {
               /* Next stream - if multiple streams are combined */
               rx_q->curNetRx_idx = (rx_q->curNetRx_idx < (rx_q->astreamids.count - 1)) ? rx_q->curNetRx_idx + 1 : 0;

               if (NT_NetRxGet(rx_q->pNetRx[rx_q->curNetRx_idx], &rx_q->pSeg, 0) != NT_SUCCESS) {
                       if (rx_q->pSeg != NULL) {
                               NT_NetRxRelease(rx_q->pNetRx[rx_q->curNetRx_idx], rx_q->pSeg);
                               rx_q->pSeg = NULL;
                       }
                       return 0;

               }
               if (NT_NET_GET_SEGMENT_LENGTH(rx_q->pSeg)) {
                       /* Build a packet structure */
                       _nt_net_build_pkt_netbuf(rx_q->pSeg, &rx_q->pkt);
               } else {
                       NT_NetRxRelease(rx_q->pNetRx[rx_q->curNetRx_idx], rx_q->pSeg);
                       rx_q->pSeg = NULL;
                       return 0;
               }
       }

       if (rte_mempool_get_bulk(rx_q->mb_pool, (void **)bufs, nb_pkts) != 0)
               return 0;

       for (i = 0; i < nb_pkts; i++) {
               mbuf = bufs[i];
               rte_mbuf_refcnt_set(mbuf, 1);
               rte_pktmbuf_reset(mbuf);

               /* HW slicing not supported */
               data_len = (uint16_t)NT_NET_GET_PKT_WIRE_LENGTH((&rx_q->pkt)) - 4;
               if (data_len <= rx_q->buf_size) {
                       /* Packet will fit in the mbuf, go ahead and copy */
                       mbuf->pkt_len = mbuf->data_len = data_len;
                       rte_memcpy((u_char *)mbuf->buf_addr + RTE_PKTMBUF_HEADROOM, NT_NET_GET_PKT_L2_PTR((&rx_q->pkt)), mbuf->data_len);
               } else {
                       /* Try read jumbo frame into multi mbufs. */
                       if (unlikely(eth_ntnic_rx_jumbo(rx_q->mb_pool,
                                                       mbuf,
                                                       NT_NET_GET_PKT_L2_PTR((&rx_q->pkt)),
                                                       data_len) == -1))
                               break;
               }

               mbuf->port = NT_NET_GET_PKT_RXPORT((&rx_q->pkt));
               num_rx++;

               /* Get the next packet if any */
               if (_nt_net_get_next_packet(rx_q->pSeg,
                               NT_NET_GET_SEGMENT_LENGTH(rx_q->pSeg),
                               &rx_q->pkt) == 0 ) {
                       NT_NetRxRelease(rx_q->pNetRx[rx_q->curNetRx_idx], rx_q->pSeg);
                       rx_q->pSeg = NULL;
                       break;
               }

               rx_q->rx_pkts++;
       }

       if (num_rx < nb_pkts) {
         rte_mempool_put_bulk(rx_q->mb_pool, (void * const *)(bufs + num_rx), nb_pkts-num_rx);
       }
       return num_rx;
}





/*
 * Callback to handle sending packets through a NT NIC.
 */
static uint16_t
eth_ntnic_tx_ringbuffer(void *queue,
       struct rte_mbuf **bufs,
       uint16_t nb_pkts)
{
       unsigned i;
       struct ntnic_tx_queue *tx_q = queue;
       int retval;
       uint16_t old;
       uint16_t new_flag = 0;


       if (unlikely(tx_q == NULL || tx_q->pNetTx == NULL || nb_pkts == 0)) {
               return 0;
       }

       do {
               old = 0;
               new_flag = 1;
               retval = rte_atomic16_cmpset(tx_q->plock, old, new_flag);
       } while (unlikely(retval == 0));

       for (i = 0; i < nb_pkts; i++) {
               /* Not allowed to TX less than 64 byte packets */
               if (bufs[i]->pkt_len < 60) {
                       bufs[i]->pkt_len = 60;
               }
               /* transmit a single packet */
               NT_NetTxRingbufferTransmitPacket(tx_q->pNetTx, rte_pktmbuf_mtod(bufs[i], uint8_t *), bufs[i]->pkt_len + 4 - tx_q->fcs_add);
               rte_pktmbuf_free(bufs[i]);
       }
       tx_q->tx_pkts += nb_pkts;

       *tx_q->plock = 0;
       return nb_pkts;
}



static int
eth_dev_start(struct rte_eth_dev *dev)
{
       struct pmd_internals *internals = dev->data->dev_private;
       struct ntnic_rx_queue *rx_q = internals->rxq;
       struct ntnic_tx_queue *tx_q = internals->txq;
       uint queue;
       int status, idx;

       RTE_LOG(INFO, PMD, "NTNIC: %s\n", __func__);

       if ((internals->nb_tx_queues | internals->nb_rx_queues) == 0) {
               return -1;
       }

       for (queue = 0; queue < internals->nb_rx_queues; queue++) {
               if (rx_q[queue].enabled) {
                       char str[128], val[10];
                       str[0] = 0;
                       /* build list of streamids to log */
                       for (idx = 0; idx < rx_q[queue].astreamids.count; idx++) {
                               if (idx) sprintf(val,",%d",rx_q[queue].astreamids.value[idx]);
                               else sprintf(val, "%i",rx_q[queue].astreamids.value[idx]);
                               strcat(str, val);
                       }

                       for (idx = 0; idx < rx_q[queue].astreamids.count; idx++) {
                               if ((status = NT_NetRxOpen(&rx_q[queue].pNetRx[idx], "DPDK", NT_NET_INTERFACE_SEGMENT,
                                               rx_q[queue].astreamids.value[idx], -1)) != NT_SUCCESS) {
                                       /* try packet interface instead */
                                       NT_ExplainError(status, errorBuffer, sizeof(errorBuffer));
                                       RTE_LOG(ERR, PMD, "NT_NetRxOpen() failed: %s\n", errorBuffer);
                                       return -1;
                               }
                       }
               }
       }

       for (queue = 0; queue < internals->nb_tx_queues; queue++) {
               if (tx_q[queue].enabled) {
                       RTE_LOG(INFO, PMD, "NTNIC: NT_NetTxOpen(%d)\n", tx_q[queue].port);
                       if ((status = NT_NetTxOpen(&tx_q[queue].pNetTx, "DPDK", 1 << tx_q[queue].port,
                                                                               dev->pci_dev->numa_node, 0)) != NT_SUCCESS) {
                               NT_ExplainError(status, errorBuffer, sizeof(errorBuffer));
                               RTE_LOG(INFO, PMD, "NT_NetTxOpen(0x%X, %d, 0) failed: %s\n", 1 << tx_q[queue].port,
                                               dev->pci_dev->numa_node, errorBuffer);
                               return -1;
                       }

                       /* Initialize and associate RTD Tx Ring buffer to Tx handler */
                       if ((status = NT_NetTxRingbufferInit(tx_q[queue].pNetTx, tx_q[queue].port)) != NT_SUCCESS) {
                               NT_ExplainError(status, errorBuffer, sizeof(errorBuffer));
                               RTE_LOG(ERR, PMD, "NT_NetRxOpen() failed: %s\n", errorBuffer);
                               return -1;
                       }
               }
               tx_q[queue].plock = &port_locks[tx_q[queue].port];
               tx_q[queue].fcs_add = (internals->MAC_50G)?4:0;
       }

       dev->data->dev_link.link_status = 1;
       return 0;
}

/*
 * This function gets called when the current port gets stopped.
 * Is the only place for us to close all the tx streams dumpers.
 * If not called the dumpers will be flushed within each tx burst.
 */
static void
eth_dev_stop(struct rte_eth_dev *dev)
{
       struct pmd_internals *internals = dev->data->dev_private;
       struct ntnic_rx_queue *rx_q = internals->rxq;
       struct ntnic_tx_queue *tx_q = internals->txq;
       uint queue;
       int idx;
       RTE_LOG(INFO, PMD, "NTNIC: %s\n", __func__);

       for (queue = 0; queue < internals->nb_rx_queues; queue++) {
               if (rx_q[queue].pSeg) {
                       NT_NetRxRelease(rx_q[queue].pNetRx[rx_q->curNetRx_idx], rx_q[queue].pSeg);
                       rx_q[queue].pSeg = NULL;
               }
               for (idx = 0; idx < rx_q[queue].astreamids.count; idx++) {
                       if (rx_q[queue].pNetRx[idx]) {
                               (void)NT_NetRxClose(rx_q[queue].pNetRx[idx]);
                       }
               }
       }
       for (queue = 0; queue < internals->nb_tx_queues; queue++) {
               if (tx_q[queue].pNetTx) {
                       NT_NetTxRingbufferDone(tx_q[queue].pNetTx);
                       (void)NT_NetTxClose(tx_q[queue].pNetTx);
               }
       }
       dev->data->dev_link.link_status = 0;
}

static int
eth_dev_configure(struct rte_eth_dev *dev __rte_unused)
{
       RTE_LOG(INFO, PMD, "NTNIC: %s\n", __func__);
       return 0;
}

static void
eth_dev_info(struct rte_eth_dev *dev,
               struct rte_eth_dev_info *dev_info)
{
       struct pmd_internals *internals = dev->data->dev_private;
       dev_info->if_index = internals->if_index;
       dev_info->driver_name = drivername;
       dev_info->max_mac_addrs = 1;
       dev_info->max_rx_pktlen = HW_MTU;
       dev_info->max_rx_queues = (uint16_t)internals->nb_rx_queues;
       dev_info->max_tx_queues = (uint16_t)internals->nb_tx_queues;
       dev_info->min_rx_bufsize = 0;
       dev_info->pci_dev = NULL;
}

static void
eth_stats_get(struct rte_eth_dev *dev,
       struct rte_eth_stats *igb_stats)
{
       unsigned i;
       unsigned long rx_total = 0;
       unsigned long tx_total = 0;
       unsigned long tx_err_total = 0;
       const struct pmd_internals *internal = dev->data->dev_private;

       memset(igb_stats, 0, sizeof(*igb_stats));
       for (i = 0; i < RTE_ETHDEV_QUEUE_STAT_CNTRS && i < internal->nb_rx_queues; i++) {
               igb_stats->q_ipackets[i] = internal->rxq[i].rx_pkts;
               rx_total += igb_stats->q_ipackets[i];
       }
       for (i = 0; i < RTE_ETHDEV_QUEUE_STAT_CNTRS && i < internal->nb_tx_queues; i++) {
               igb_stats->q_opackets[i] = internal->txq[i].tx_pkts;
               igb_stats->q_errors[i] = internal->txq[i].err_pkts;
               tx_total += igb_stats->q_opackets[i];
               tx_err_total += igb_stats->q_errors[i];
       }

       igb_stats->ipackets = rx_total;
       igb_stats->opackets = tx_total;
       igb_stats->oerrors = tx_err_total;
}

static void
eth_stats_reset(struct rte_eth_dev *dev)
{
       unsigned i;
       struct pmd_internals *internal = dev->data->dev_private;

       for (i = 0; i < internal->nb_rx_queues; i++) {
               internal->rxq[i].rx_pkts = 0;
       }
       for (i = 0; i < internal->nb_tx_queues; i++) {
               internal->txq[i].tx_pkts = 0;
               internal->txq[i].err_pkts = 0;
       }
}

static void
eth_dev_close(struct rte_eth_dev *dev __rte_unused)
{
       RTE_LOG(INFO, PMD, "NTNIC: %s\n", __func__);
}

static void
eth_queue_release(void *q __rte_unused)
{
       RTE_LOG(INFO, PMD, "NTNIC: %s\n", __func__);
}

static int
eth_link_update(struct rte_eth_dev *dev __rte_unused,
       int wait_to_complete __rte_unused)
{
       return 0;
}

static int
eth_rx_queue_setup(struct rte_eth_dev *dev,
       uint16_t rx_queue_id,
       uint16_t nb_rx_desc __rte_unused,
       unsigned int socket_id __rte_unused,
       const struct rte_eth_rxconf *rx_conf __rte_unused,
       struct rte_mempool *mb_pool)
{
       struct rte_pktmbuf_pool_private *mbp_priv;
       struct pmd_internals *internals = dev->data->dev_private;
       struct ntnic_rx_queue *rx_q = &internals->rxq[rx_queue_id];

       RTE_LOG(INFO, PMD, "NTNIC RX queue setup\n");
       rx_q->mb_pool = mb_pool;
       dev->data->rx_queues[rx_queue_id] = rx_q;

       mbp_priv =  rte_mempool_get_priv(rx_q->mb_pool);
       rx_q->buf_size = (uint16_t) (mbp_priv->mbuf_data_room_size - RTE_PKTMBUF_HEADROOM);
       rx_q->enabled = 1;
       return 0;
}

static int
eth_tx_queue_setup(struct rte_eth_dev *dev __rte_unused,
       uint16_t tx_queue_id __rte_unused,
       uint16_t nb_tx_desc __rte_unused,
       unsigned int socket_id __rte_unused,
       const struct rte_eth_txconf *tx_conf __rte_unused)
{
       struct pmd_internals *internals = dev->data->dev_private;
       RTE_LOG(INFO, PMD, "NTNIC TX queue setup\n");
       dev->data->tx_queues[tx_queue_id] = &internals->txq[tx_queue_id];
       internals->txq[tx_queue_id].enabled = 1;
       return 0;
}

static int _dev_set_mtu(struct rte_eth_dev *dev __rte_unused, uint16_t mtu)
{
       if (mtu < 46 || mtu > HW_MTU)
               return -EINVAL;

       return 0;
}

static struct eth_dev_ops ops = {
               .dev_start = eth_dev_start,
               .dev_stop =     eth_dev_stop,
               .dev_close = eth_dev_close,
               .mtu_set = _dev_set_mtu,
               .dev_configure = eth_dev_configure,
               .dev_infos_get = eth_dev_info,
               .rx_queue_setup = eth_rx_queue_setup,
               .tx_queue_setup = eth_tx_queue_setup,
               .rx_queue_release = eth_queue_release,
               .tx_queue_release = eth_queue_release,
               .link_update = eth_link_update,
               .stats_get = eth_stats_get,
               .stats_reset = eth_stats_reset,
};

static int
rte_pmd_init_internals(const char *name,
               const unsigned nb_rx_queues,
               const unsigned nb_tx_queues,
               const unsigned numa_node,
               struct array_s *pastreamids,
               const uint32_t port,
               struct rte_eth_dev **eth_dev)
{
       struct pmd_internals *internals = NULL;
       struct rte_eth_dev_data *data = NULL;
       struct rte_pci_device *pci_dev = NULL;
       uint i, status;
       char errBuf[NT_ERRBUF_SIZE];
       NtInfoStream_t hInfo;
       NtInfo_t infoPort;
       struct rte_eth_link pmd_link;
       assert(nb_rx_queues < MAX_RX_QUEUES);
       assert(nb_tx_queues < MAX_TX_QUEUES);
       assert(port < MAX_NTNIC_PORTS);

       RTE_LOG(INFO, PMD,
                       "Creating ntnic-backend ethdev on numa socket %u\n", numa_node);

       /* now do all data allocation - for eth_dev structure, dummy pci driver
        * and internal (private) data
        */
       data = rte_zmalloc_socket(name, sizeof(*data), 0, numa_node);
       if (data == NULL)
               goto error;

       pci_dev = rte_zmalloc_socket(name, sizeof(*pci_dev), 0, numa_node);
       if (pci_dev == NULL)
               goto error;

       internals = rte_zmalloc_socket(name, sizeof(*internals), 0, numa_node);
       if (internals == NULL)
               goto error;

       /* reserve an ethdev entry */
       *eth_dev = rte_eth_dev_allocate(name, RTE_ETH_DEV_VIRTUAL);
       if (*eth_dev == NULL)
               goto error;

       /* Open the information stream */
       if ((status = NT_InfoOpen(&hInfo, "DPDK Info stream")) != NT_SUCCESS) {
               NT_ExplainError(status, errBuf, sizeof(errBuf));
               RTE_LOG(ERR, PMD, ">>> Error: NT_InfoOpen failed. Code 0x%x = %s\n", status, errBuf);
               return status;
       }

       /* Find local port offset */
       infoPort.cmd = NT_INFO_CMD_READ_PORT_V7;
       infoPort.u.port_v7.portNo = (uint8_t)(port);
       if ((status = NT_InfoRead(hInfo, &infoPort)) != 0) {
               NT_ExplainError(status, errBuf, sizeof(errBuf));
               RTE_LOG(ERR, PMD, "ERROR: NT_InfoRead failed. Code 0x%x = %s\n", status, errBuf);
               return status;
       }

       /* check for 50G MAC */
    if (infoPort.u.port_v7.data.adapterInfo.fpgaid.s.product == 9506 ||
       infoPort.u.port_v7.data.adapterInfo.fpgaid.s.product == 9509) {
       internals->MAC_50G = 1;
    } else {
       internals->MAC_50G = 0;
       /* Check for valid FPGAs (NFV NICs) */
        if (infoPort.u.port_v7.data.adapterInfo.fpgaid.s.product != 9507 &&
               infoPort.u.port_v7.data.adapterInfo.fpgaid.s.product != 9510) {
               RTE_LOG(ERR, PMD, "ERROR: NTNIC PMD does not support the NT adapter - port %i\n", port);
               return -1;
        }
    }

       internals->nb_rx_queues = nb_rx_queues;
    internals->nb_tx_queues = nb_tx_queues;
       for (i=0; i < nb_rx_queues; i++) {
               int idx;
               for (idx = 0; idx < pastreamids->count; idx++) {
                       internals->rxq[i].astreamids.value[idx] = pastreamids->value[idx] + i;
               }

               internals->rxq[i].astreamids.count = pastreamids->count;

               internals->rxq[i].pSeg = NULL;
               internals->rxq[i].enabled = 0;
       }

       for (i = 0; i < nb_tx_queues; i++) {
       if (port < infoPort.u.port_v7.data.adapterInfo.portOffset) {
               RTE_LOG(ERR, PMD, "Error wrong port specified\n");
               return -1;
       }
               internals->txq[i].port = port;
               internals->txq[i].enabled = 0;
       }

       switch (infoPort.u.port_v7.data.speed) {
       case NT_LINK_SPEED_UNKNOWN:
               pmd_link.link_speed = ETH_SPEED_NUM_1G;
               break;
       case NT_LINK_SPEED_10M:
               pmd_link.link_speed = ETH_SPEED_NUM_10M;
               break;
       case NT_LINK_SPEED_100M:
               pmd_link.link_speed = ETH_SPEED_NUM_100M;
               break;
       case NT_LINK_SPEED_1G:
               pmd_link.link_speed = ETH_SPEED_NUM_1G;
               break;
       case NT_LINK_SPEED_10G:
               pmd_link.link_speed = ETH_SPEED_NUM_10G;
               break;
       case NT_LINK_SPEED_40G:
               pmd_link.link_speed = ETH_SPEED_NUM_40G;
               break;
       case NT_LINK_SPEED_50G:
               pmd_link.link_speed = ETH_SPEED_NUM_50G;
               break;
       case NT_LINK_SPEED_100G:
               pmd_link.link_speed = ETH_SPEED_NUM_100G;
               break;
       }

       memcpy(&eth_addr[port].addr_bytes, &infoPort.u.port_v7.data.macAddress, sizeof(eth_addr[port].addr_bytes));

       if ((status = NT_InfoClose(hInfo)) != NT_SUCCESS) {
               NT_ExplainError(status, errBuf, sizeof(errBuf));
               RTE_LOG(ERR, PMD, ">>> Error: NT_InfoOpen failed. Code 0x%x = %s\n", status, errBuf);
               return status;
       }

       pmd_link.link_duplex = ETH_LINK_FULL_DUPLEX;
       pmd_link.link_status = 0;

       internals->if_index = 0;

       pci_dev->numa_node = numa_node;

       data->dev_private = internals;
       data->port_id = (*eth_dev)->data->port_id;
       data->nb_rx_queues = (uint16_t)nb_rx_queues;
       data->nb_tx_queues = (uint16_t)nb_tx_queues;
       data->dev_link = pmd_link;
       data->mac_addrs = &eth_addr[port];
       data->numa_node = numa_node;
       data->drv_name = drivername;

       (*eth_dev)->data = data;
       (*eth_dev)->dev_ops = &ops;
       (*eth_dev)->pci_dev = pci_dev;

       return 0;

error:
       if (data)
               rte_free(data);
       if (pci_dev)
               rte_free(pci_dev);
       if (internals)
               rte_free(internals);
       return -1;
}

/*
 * convert ascii to int
 */
static inline int
ascii_to_u32(const char *key __rte_unused, const char *value, void *extra_args __rte_unused)
{
       *(uint32_t*)extra_args = atoi(value);
       return 0;
}

static inline int
ascii_to_u32_array(const char *key __rte_unused, const char *value, void *extra_args)
{
       struct array_s *pastreams = (struct array_s *)extra_args;
       int sval, eval;
       if (sscanf(value, "%d..%d", &sval, &eval) == 2) {
               int cnt;
               for (cnt = sval; cnt <= eval; cnt++)
                       pastreams->value[pastreams->count++] = cnt;
       } else {
               pastreams->value[pastreams->count++] = atoi(value);
       }
       return 0;
}





static int DoNtpl(const char *ntplStr)
{
       NtConfigStream_t hCfgStream;
       NtNtplInfo_t ntplInfo;
       int status;

       if((status = NT_ConfigOpen(&hCfgStream, "capture")) != NT_SUCCESS) {
               /* Get the status code as text */
               NT_ExplainError(status, errorBuffer, sizeof(errorBuffer)-1);
               fprintf(stderr, "NT_ConfigOpen() failed: %s\n", errorBuffer);
               return -1;
       }

       RTE_LOG(INFO, PMD, "NTPL : %s\n", ntplStr);
       if((status = NT_NTPL(hCfgStream, ntplStr, &ntplInfo, NT_NTPL_PARSER_VALIDATE_NORMAL)) != NT_SUCCESS) {
               /* Get the status code as text */
               NT_ExplainError(status, errorBuffer, sizeof(errorBuffer)-1);
               fprintf(stderr, "NT_NTPL() failed: %s\n", errorBuffer);
               fprintf(stderr, ">>> NTPL errorcode: %X\n", ntplInfo.u.errorData.errCode);
               fprintf(stderr, ">>> %s\n", ntplInfo.u.errorData.errBuffer[0]);
               fprintf(stderr, ">>> %s\n", ntplInfo.u.errorData.errBuffer[1]);
               fprintf(stderr, ">>> %s\n", ntplInfo.u.errorData.errBuffer[2]);
               NT_ConfigClose(hCfgStream);
               return -1;
       }
       NT_ConfigClose(hCfgStream);
       return 0;
}




static int
rte_pmd_ntnic_devinit(const char *name, const char *params)
{
       unsigned numa_node;
       int ret = 0;
       struct rte_kvargs *kvlist;
       struct rte_eth_dev *eth_dev;
       unsigned int i;
       uint32_t rxqueues = 0;
       uint32_t txqueues = 0;
       uint32_t port=0;
       uint32_t portend = (uint32_t)-1;
       uint32_t hash = (uint32_t)-1;
       struct array_s astreamids;

       static int first = 1;
       static int stream_id = 0;
       char ntplStr[512];

       astreamids.count = 0;

       RTE_LOG(INFO, PMD, "Initializing pmd_ntnic for %s\n", name);

       numa_node = rte_socket_id();

       kvlist = rte_kvargs_parse(params, valid_arguments);
       if (kvlist == NULL) {
               return -1;
       }

       /* Get port to use for Rx/Tx */
       if ((i = rte_kvargs_count(kvlist, ETH_NTNIC_PORT_ARG))) {
               assert (i == 1);
               ret = rte_kvargs_process(kvlist, ETH_NTNIC_PORT_ARG,
                                                          &ascii_to_u32, &port);
       }
       /* If Rx port merge is need, her the portend is specified */
       if ((i = rte_kvargs_count(kvlist, ETH_NTNIC_PORTEND_ARG))) {
               assert (i == 1);
               ret = rte_kvargs_process(kvlist, ETH_NTNIC_PORTEND_ARG,
                                                                &ascii_to_u32, &portend);
       }

       /* Get # RX queues */
       if ((i = rte_kvargs_count(kvlist, ETH_NTNIC_RXQUEUES_ARG))) {
               assert (i == 1);
               ret = rte_kvargs_process(kvlist, ETH_NTNIC_RXQUEUES_ARG,
                                                                &ascii_to_u32, &rxqueues);
       }
       /* Get # TX queues */
       if ((i = rte_kvargs_count(kvlist, ETH_NTNIC_TXQUEUES_ARG))) {
               assert (i == 1);
               ret = rte_kvargs_process(kvlist, ETH_NTNIC_TXQUEUES_ARG,
                                                                &ascii_to_u32, &txqueues);
       }

       /* Get list of streamIds - if used */
       if ((i = rte_kvargs_count(kvlist, ETH_NTNIC_STREAMID_ARG))) {
               ret = rte_kvargs_process(kvlist, ETH_NTNIC_STREAMID_ARG,
                                                                &ascii_to_u32_array, &astreamids);
       }

       /* Get an alternative hash algorithm */
       if ((i = rte_kvargs_count(kvlist, ETH_NTNIC_HASH_ARG))) {
               assert (i == 1);
               ret = rte_kvargs_process(kvlist, ETH_NTNIC_HASH_ARG,
                                                                &ascii_to_u32, &hash);
       }

       /* check portend and streamids */
       if (portend != (uint32_t)-1 && astreamids.count) {
               RTE_LOG(ERR, PMD, "Cannot specify portend when one or more streamid's are specified\n");
               return -1;
       }
       rte_kvargs_free(kvlist);

       if (ret < 0)
               return -1;

       if (first)
               NT_Init(NTAPI_VERSION);

       if (astreamids.count) {
               first = 0;
               /* specific streamid specified, use that - port defaults to Tx port only */
               if (rte_pmd_init_internals(name, rxqueues, txqueues, numa_node, &astreamids, port, &eth_dev) < 0)
                       return -1;
       } else {
               struct array_s astrids;
               if (first) {
                       /* Delete all NTPL */
                       sprintf(ntplStr, "Delete=All");
                       if (DoNtpl(ntplStr) != 0) {
                               return -1;
                       }
                       first = 0;
               }
               astrids.count=1;
               astrids.value[0] = stream_id;
               if (rte_pmd_init_internals(name, rxqueues, txqueues, numa_node, &astrids, port, &eth_dev) < 0)
                       return -1;

               /* Assign the traffic */
               if (portend != (uint32_t)-1) {
                       sprintf(ntplStr, "Assign[streamid=(%d..%d);Descriptor=NT]=port==(%d..%d)", stream_id,
                                       (stream_id+rxqueues-1), port, portend);

               } else {
                       sprintf(ntplStr, "Assign[streamid=(%d..%d);Descriptor=NT]=port==%d", stream_id,
                                       (stream_id+rxqueues-1), port);
               }
               if (DoNtpl(ntplStr) != 0) {
                       return -1;
               }

               if (hash != (uint32_t)-1) {
                       switch (hash) {
                       default:
                       case 1:
                               DoNtpl("HashMode=HashRoundRobin");
                               break;
                       case 2:
                               DoNtpl("HashMode=Hash2TupleSorted");
                               break;
                       case 3:
                               DoNtpl("HashMode=Hash5TupleSorted");
                               break;
                       }
               }

               stream_id += rxqueues;
       }

       eth_dev->rx_pkt_burst = eth_ntnic_rx;
       eth_dev->tx_pkt_burst = eth_ntnic_tx_ringbuffer;

       return 0;
}



static struct rte_driver pmd_ntnic_drv = {
       .type = PMD_VDEV,
       .init = rte_pmd_ntnic_devinit,
};

PMD_REGISTER_DRIVER(pmd_ntnic_drv, eth_ntnic);
DRIVER_REGISTER_PARAM_STRING(eth_ntnic,
       "port=<int> "
       "rxqs=<int>"
       "txqs=<int>"
       "hash=<int>"
       "streamids=<int..int>");


