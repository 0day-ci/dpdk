#include <stdbool.h>
#include <rte_cycles.h>
#include <rte_common.h>
#include <rte_errno.h>
#include <rte_ethdev.h>
#include <rte_malloc.h>
#include <rte_service.h>
#include <rte_service_private.h>
#include <rte_thash.h>

#include "rte_eventdev.h"
#include "rte_event_eth_rx_adapter.h"

#define BATCH_SIZE		32
#define BLOCK_CNT_THRESHOLD	10
#define ETH_EVENT_BUFFER_SIZE	(4*BATCH_SIZE)

/*
 * There is an instance of this struct per polled Rx queue added to the
 * adapter
 */
struct eth_rx_poll_entry {
	uint8_t eth_dev_id;
	uint16_t eth_rx_qid;
};

struct rte_eth_event_enqueue_buffer {
	/* Count of events in this buffer */
	uint16_t count;
	/* Array of events in this buffer */
	struct rte_event events[ETH_EVENT_BUFFER_SIZE];
};

struct rte_event_eth_rx_adapter {
	/* per adapter EAL service */
	struct rte_service_spec *service;
	/* lock to serialize config updates with service function */
	rte_spinlock_t rx_lock;
	/* event device identifier */
	uint8_t eventdev_id;
	/* event port identifier */
	uint8_t event_port_id;
	/* max mbufs processed in any service function invocation */
	uint32_t max_nb_rx;
	/* socket identifier */
	int socket_id;
	/** Receive queues that need to be polled */
	struct eth_rx_poll_entry *eth_rx_poll;
	/** size of the eth_rx_poll array */
	uint16_t num_rx_polled;
	/** per ethernet device structure */
	struct eth_device_info *eth_devices;
	/* Weighted round robin schedule */
	uint32_t *wrr_sched;
	/* Size of wrr[] */
	uint32_t wrr_len;
	/* Next entry in wrr[] to begin polling */
	uint32_t wrr_pos;
	/* Event burst buffer */
	struct rte_eth_event_enqueue_buffer event_enqueue_buffer;
	/* per adapter stats */
	struct rte_event_eth_rx_adapter_stats stats;
	/**> Block count, counts upto BLOCK_CNT_THRESHOLD */
	uint16_t enq_block_count;
	/**> Block start ts */
	uint64_t rx_enq_block_start_ts;
} __rte_cache_aligned;


/* Per eth device */
struct eth_device_info {
	struct rte_eth_dev *dev;
	struct eth_rx_queue_info *rx_queue;
};

/* Per Rx queue */
struct eth_rx_queue_info {
	bool queue_enabled;	/* true if added */
	uint16_t wt;		/* polling weight */
	uint8_t event_queue_id;	/* Event queue to enqueue packets to */
	uint8_t sched_type;	/* sched type for events */
	uint8_t priority;	/* event priority */
	uint32_t flow_id;	/* app provided flow identifier */
	uint32_t flow_id_mask;	/* Set to ~0 if app provides flow id else 0 */
};

static struct rte_event_eth_rx_adapter **rte_event_eth_rx_adapter;
static uint8_t default_rss_key[] = {
	0x6d, 0x5a, 0x56, 0xda, 0x25, 0x5b, 0x0e, 0xc2,
	0x41, 0x67, 0x25, 0x3d, 0x43, 0xa3, 0x8f, 0xb0,
	0xd0, 0xca, 0x2b, 0xcb, 0xae, 0x7b, 0x30, 0xb4,
	0x77, 0xcb, 0x2d, 0xa3, 0x80, 0x30, 0xf2, 0x0c,
	0x6a, 0x42, 0xb7, 0x3b, 0xbe, 0xac, 0x01, 0xfa,
};

static uint8_t *rss_key_be;

static inline int
valid_id(uint8_t id)
{
	return id < RTE_MAX_EVENT_ETH_RX_ADAPTER_INSTANCE;
}

/* Greatest common divisor */
static uint16_t gcd_u16(uint16_t a, uint16_t b)
{
	uint16_t r = a % b;

	return r ? gcd_u16(b, r) : b;
}

/* Returns the next queue in the polling sequence
 *
 * http://kb.linuxvirtualserver.org/wiki/Weighted_Round-Robin_Scheduling
 */
static int
wrr_next(struct rte_event_eth_rx_adapter *rx_adapter,
	 unsigned int n, int *cw,
	 struct eth_rx_poll_entry *eth_rx_poll, uint16_t max_wt,
	 uint16_t gcd, int prev)
{
	int i = prev;
	uint16_t w;

	while (1) {
		uint16_t q;
		uint8_t d;

		i = (i + 1) % n;
		if (i == 0) {
			*cw = *cw - gcd;
			if (*cw <= 0)
				*cw = max_wt;
		}

		q = eth_rx_poll[i].eth_rx_qid;
		d = eth_rx_poll[i].eth_dev_id;
		w = rx_adapter->eth_devices[d].rx_queue[q].wt;

		if ((int)w >= *cw)
			return i;
	}
}

/* Precalculate WRR polling sequence for all queues in rx_adapter */
static int
eth_poll_wrr_calc(struct rte_event_eth_rx_adapter *rx_adapter)
{
	uint8_t d;
	uint16_t q;
	unsigned int i;

	/* Initialize variables for calculaton of wrr schedule */
	uint16_t max_wrr_pos = 0;
	unsigned int poll_q = 0;
	uint16_t max_wt = 0;
	uint16_t gcd = 0;

	struct eth_rx_poll_entry *rx_poll = NULL;
	uint32_t *rx_wrr = NULL;

	if (rx_adapter->num_rx_polled) {
		size_t len = RTE_ALIGN(rx_adapter->num_rx_polled *
				       sizeof(*rx_adapter->eth_rx_poll),
					RTE_CACHE_LINE_SIZE);
		rx_poll = rte_zmalloc_socket("eth_rx_poll",
					     len,
					     RTE_CACHE_LINE_SIZE,
					     rx_adapter->socket_id);
		if (!rx_poll)
			return -ENOMEM;

		/* Generate array of all queues to poll, the size of this
		 * array is poll_q
		 */
		for (d = 0; d < rte_eth_dev_count(); d++) {
			uint16_t nb_rx_queues;

			nb_rx_queues =
			    rx_adapter->eth_devices[d].dev->data->nb_rx_queues;
			for (q = 0; q < nb_rx_queues; q++) {
				struct eth_rx_queue_info *queue_info;
				queue_info = &rx_adapter->eth_devices[d].rx_queue[q];

				if (!queue_info->queue_enabled)
					continue;

				uint16_t wt = queue_info->wt;
				rx_poll[poll_q].eth_dev_id = d;
				rx_poll[poll_q].eth_rx_qid = q;
				max_wrr_pos += wt;
				max_wt = RTE_MAX(max_wt, wt);
				gcd = (gcd) ? gcd_u16(gcd, wt) : wt;
				poll_q++;
			}
		}

		len = RTE_ALIGN(max_wrr_pos * sizeof(*rx_wrr), RTE_CACHE_LINE_SIZE);
		rx_wrr = rte_zmalloc_socket("eth_rx_wrr",
					    len,
					    RTE_CACHE_LINE_SIZE,
					    rx_adapter->socket_id);
		if (!rx_wrr) {
			rte_free(rx_poll);
			return -ENOMEM;
		}

		/* Generate polling sequence based on weights */
		int prev = -1;
		int cw = -1;
		for (i = 0; i < max_wrr_pos; i++) {
			rx_wrr[i] = wrr_next(rx_adapter, poll_q, &cw,
					     rx_poll, max_wt, gcd, prev);
			prev = rx_wrr[i];
		}
	}

	rte_free(rx_adapter->eth_rx_poll);
	rte_free(rx_adapter->wrr_sched);

	rx_adapter->eth_rx_poll = rx_poll;
	rx_adapter->wrr_sched = rx_wrr;
	rx_adapter->wrr_len = max_wrr_pos;

	return 0;
}

#if RTE_BYTE_ORDER == RTE_LITTLE_ENDIAN
#define BE_16(x)	(uint16_t)((x) >> 8 | (x) << 8)
#else
#define BE_16(x)	(x)
#endif

#define NETWORK_ORDER(x) BE_16(x)

static inline void
mtoip(struct rte_mbuf *m, struct ipv4_hdr **ipv4_hdr,
	struct ipv6_hdr **ipv6_hdr)
{
	struct ether_hdr *eth_hdr = rte_pktmbuf_mtod(m, struct ether_hdr *);
	struct vlan_hdr *vlan_hdr;

	*ipv4_hdr = NULL;
	*ipv6_hdr = NULL;

	switch (eth_hdr->ether_type) {
	case NETWORK_ORDER(ETHER_TYPE_IPv4):
		*ipv4_hdr = (struct ipv4_hdr *)(eth_hdr + 1);
		break;

	case NETWORK_ORDER(ETHER_TYPE_IPv6):
		*ipv6_hdr = (struct ipv6_hdr *)(eth_hdr + 1);
		break;

	case NETWORK_ORDER(ETHER_TYPE_VLAN):
		vlan_hdr = (struct vlan_hdr *)(eth_hdr + 1);
		switch (vlan_hdr->eth_proto) {
		case NETWORK_ORDER(ETHER_TYPE_IPv4):
			*ipv4_hdr = (struct ipv4_hdr *)(vlan_hdr + 1);
			break;
		case NETWORK_ORDER(ETHER_TYPE_IPv6):
			*ipv6_hdr = (struct ipv6_hdr *)(vlan_hdr + 1);
			break;
		default:
			break;
		}
		break;

	default:
		break;
	}
}

/* Calculate RSS hash for IPv4/6 */
static inline uint32_t
do_softrss(struct rte_mbuf *m)
{
	uint32_t input_len;
	void *tuple;
	struct rte_ipv4_tuple ipv4_tuple;
	struct rte_ipv6_tuple ipv6_tuple;
	struct ipv4_hdr *ipv4_hdr;
	struct ipv6_hdr *ipv6_hdr;

	mtoip(m, &ipv4_hdr, &ipv6_hdr);

	if (ipv4_hdr) {
		ipv4_tuple.src_addr = rte_be_to_cpu_32(ipv4_hdr->src_addr);
		ipv4_tuple.dst_addr = rte_be_to_cpu_32(ipv4_hdr->dst_addr);
		tuple = &ipv4_tuple;
		input_len = RTE_THASH_V4_L3_LEN;
	} else if (ipv6_hdr) {
		rte_thash_load_v6_addrs(ipv6_hdr,
					(union rte_thash_tuple *)&ipv6_tuple);
		tuple = &ipv6_tuple;
		input_len = RTE_THASH_V6_L3_LEN;
	} else
		return 0;

	return rte_softrss_be(tuple, input_len, rss_key_be);
}

static inline bool
rx_enq_blocked(struct rte_event_eth_rx_adapter *rx_adapter)
{
	return !!rx_adapter->enq_block_count;
}

static inline void
rx_enq_block_start_ts(struct rte_event_eth_rx_adapter *rx_adapter)
{
	if (rx_adapter->rx_enq_block_start_ts)
		return;

	rx_adapter->enq_block_count++;
	if (rx_adapter->enq_block_count < BLOCK_CNT_THRESHOLD)
		return;

	rx_adapter->rx_enq_block_start_ts = rte_get_tsc_cycles();
}

static inline void
rx_enq_block_end_ts(struct rte_event_eth_rx_adapter *rx_adapter,
		    struct rte_event_eth_rx_adapter_stats *stats)
{
	if (unlikely(!stats->rx_enq_start_ts))
		stats->rx_enq_start_ts = rte_get_tsc_cycles();

	if (likely(!rx_enq_blocked(rx_adapter)))
		return;

	rx_adapter->enq_block_count = 0;
	if (rx_adapter->rx_enq_block_start_ts) {
		stats->rx_enq_end_ts = rte_get_tsc_cycles();
		stats->rx_enq_block_cycles += stats->rx_enq_end_ts -
		    rx_adapter->rx_enq_block_start_ts;
		rx_adapter->rx_enq_block_start_ts = 0;
	}
}

/* Add event to buffer, free space check is done prior to calling
 * this function
 */
static inline void
buf_event_enqueue(struct rte_event_eth_rx_adapter *rx_adapter,
		  struct rte_event *ev)
{
	struct rte_eth_event_enqueue_buffer *buf =
	    &rx_adapter->event_enqueue_buffer;
	rte_memcpy(&buf->events[buf->count++], ev, sizeof(struct rte_event));
}

/* Enqueue buffered events to event device */
static inline int
flush_event_buffer(uint8_t id)
{
	struct rte_event_eth_rx_adapter *rx_adapter =
	    rte_event_eth_rx_adapter[id];
	struct rte_eth_event_enqueue_buffer *buf =
	    &rx_adapter->event_enqueue_buffer;
	struct rte_event_eth_rx_adapter_stats *stats = &rx_adapter->stats;

	int n = rte_event_enqueue_burst(rx_adapter->eventdev_id,
					rx_adapter->event_port_id,
					buf->events,
					buf->count);

	if (n != buf->count) {
		memmove(buf->events,
			&buf->events[n],
			(buf->count - n) * sizeof(struct rte_event));
		stats->rx_enq_retry++;
	}
	buf->count -= n;
	stats->rx_enq_count += n;

	return n;
}

static inline void
fill_event_buffer(uint8_t id, uint8_t dev_id,
		  uint16_t rx_queue_id, struct rte_mbuf **mbufs, uint16_t num)
{
	uint32_t i;
	struct rte_event_eth_rx_adapter *rx_adapter =
	    rte_event_eth_rx_adapter[id];
	struct eth_device_info *eth_device_info =
	    &rx_adapter->eth_devices[dev_id];
	struct eth_rx_queue_info *eth_rx_queue_info =
	    &eth_device_info->rx_queue[rx_queue_id];

	int32_t qid = eth_rx_queue_info->event_queue_id;
	uint8_t sched_type = eth_rx_queue_info->sched_type;
	uint8_t priority = eth_rx_queue_info->priority;
	uint32_t flow_id;
	struct rte_event events[BATCH_SIZE];
	struct rte_mbuf *m = mbufs[0];
	uint32_t rss_mask;
	uint32_t rss;
	int do_rss;

	/* 0xffff ffff if PKT_RX_RSS_HASH is set, otherwise 0 */
	rss_mask = ~(((m->ol_flags & PKT_RX_RSS_HASH) != 0) - 1);
	do_rss = !rss_mask && !eth_rx_queue_info->flow_id_mask;

	for (i = 0; i < num; i++) {
		m = mbufs[i];
		struct rte_event *ev = &events[i];

		rss = do_rss ? do_softrss(m) : m->hash.rss;
		flow_id =
		    eth_rx_queue_info->flow_id &
				eth_rx_queue_info->flow_id_mask;
		flow_id |= rss & ~eth_rx_queue_info->flow_id_mask;

		ev->flow_id = flow_id;
		ev->op = RTE_EVENT_OP_NEW;
		ev->sched_type = sched_type;
		ev->queue_id = qid;
		ev->event_type = RTE_EVENT_TYPE_ETHDEV;
		ev->sub_event_type = 0;
		ev->priority = priority;
		ev->mbuf = m;

		buf_event_enqueue(rx_adapter, ev);
	}
}

/*
 * Polls receive queues added to the event adapter and enqueues received
 * packets to the event device.
 *
 * The receive code enqueues initially to a temporary buffer, the
 * temporary buffer is drained anytime it holds >= BATCH_SIZE packets
 *
 * If there isn't space available in the temporary buffer, packets from the
 * Rx queue arent dequeued from the eth device, this backpressures the
 * eth device, in virtual device enviroments this backpressure is relayed to the
 * hypervisor's switching layer where adjustments can be made to deal with
 * it.
 */
static inline unsigned int
eth_rx_poll(uint8_t id)
{
	unsigned int num_queue;
	uint16_t n;
	unsigned int nb_rx = 0;
	struct rte_mbuf *mbufs[BATCH_SIZE];
	struct rte_event_eth_rx_adapter *rx_adapter;
	struct rte_eth_event_enqueue_buffer *buf;
	unsigned int wrr_pos;
	unsigned int max_nb_rx;

	rx_adapter = rte_event_eth_rx_adapter[id];
	wrr_pos = rx_adapter->wrr_pos;
	max_nb_rx = rx_adapter->max_nb_rx;
	buf = &rx_adapter->event_enqueue_buffer;
	struct rte_event_eth_rx_adapter_stats *stats = &rx_adapter->stats;

	/* Iterate through a WRR sequence */
	for (num_queue = 0; num_queue < rx_adapter->wrr_len; num_queue++) {
		unsigned int poll_idx = rx_adapter->wrr_sched[wrr_pos];
		uint16_t qid = rx_adapter->eth_rx_poll[poll_idx].eth_rx_qid;
		uint8_t d = rx_adapter->eth_rx_poll[poll_idx].eth_dev_id;

		/* Don't do a batch dequeue from the rx queue if there isn't
		 * enough space in the enqueue buffer.
		 */
		if (buf->count >= BATCH_SIZE)
			flush_event_buffer(id);
		if (BATCH_SIZE > (ETH_EVENT_BUFFER_SIZE - buf->count))
			break;

		stats->rx_poll_count++;
		n = rte_eth_rx_burst(d, qid, mbufs, BATCH_SIZE);

		if (n) {
			stats->rx_packets += n;
			/* The check before rte_eth_rx_burst() ensures that
			 * all n mbufs can be buffered
			 */
			fill_event_buffer(id, d, qid, mbufs, n);
			nb_rx += n;
			if (nb_rx > max_nb_rx) {
				rx_adapter->wrr_pos =
				    (wrr_pos + 1) % rx_adapter->wrr_len;
				return nb_rx;
			}
		}

		if (++wrr_pos == rx_adapter->wrr_len)
			wrr_pos = 0;
	}

	return nb_rx;
}

static int
event_eth_rx_adapter_service_func(void *args)
{
	uint8_t id = (uint8_t) (uintptr_t) args;
	struct rte_event_eth_rx_adapter *rx_adapter;
	struct rte_eth_event_enqueue_buffer *buf;
	unsigned int nb_rx;

	rx_adapter = rte_event_eth_rx_adapter[id];
	buf = &rx_adapter->event_enqueue_buffer;
	if (!rte_spinlock_trylock(&rx_adapter->rx_lock))
		return 0;

	nb_rx = eth_rx_poll(id);

	if (!nb_rx && buf->count)
		flush_event_buffer(id);
	rte_spinlock_unlock(&rx_adapter->rx_lock);

	return 0;
}

int rte_event_eth_rx_adapter_init(void)
{
	const char *name = "rte_event_eth_rx_adapter_array";
	unsigned sz;
	const struct rte_memzone *mz;

	sz = sizeof(*rte_event_eth_rx_adapter) *
	    RTE_MAX_EVENT_ETH_RX_ADAPTER_INSTANCE;
	sz = RTE_ALIGN(sz, RTE_CACHE_LINE_SIZE);
	sz += RTE_ALIGN(sizeof(default_rss_key), RTE_CACHE_LINE_SIZE);

	if (rte_eal_process_type() == RTE_PROC_SECONDARY)
		mz = rte_memzone_lookup(name);
	else {
		mz = rte_memzone_reserve_aligned(name, sz, rte_socket_id(), 0,
						 RTE_CACHE_LINE_SIZE);
		if (mz)
			memset(mz->addr, 0, sz);
	}

	if (!mz) {
		RTE_LOG(ERR, EVENTDEV, "%s() failed to reserve memzeone err = %d\n",
			__func__, rte_errno);
		return -rte_errno;
	}

	rte_event_eth_rx_adapter = mz->addr;
	rss_key_be = (void *)&rte_event_eth_rx_adapter[RTE_MAX_EVENT_ETH_RX_ADAPTER_INSTANCE];
	rss_key_be = RTE_PTR_ALIGN(rss_key_be, RTE_CACHE_LINE_SIZE);
	if (rte_eal_process_type() == RTE_PROC_PRIMARY)
		rte_convert_rss_key((uint32_t *)default_rss_key,
				    (uint32_t *)rss_key_be,
				    RTE_DIM(default_rss_key));
	return 0;
}

int rte_event_eth_rx_adapter_create(uint8_t id,
				    const struct rte_event_eth_rx_adapter_conf *conf)
{
	struct rte_event_eth_rx_adapter *rx_adapter;
	uint8_t i;
	int ret;

	rx_adapter = rte_event_eth_rx_adapter[id];

	if (!valid_id(id) || !conf)
		return -EINVAL;
	if (rte_event_eth_rx_adapter[id]) {
		return -EEXIST;
	}

	const int socket_id = conf->socket_id;
	rx_adapter = rte_zmalloc_socket("eth_devices", sizeof(*rx_adapter),
					RTE_CACHE_LINE_SIZE, socket_id);
	if (!rx_adapter)
		return -ENOMEM;

	memset(rx_adapter, 0, sizeof(struct rte_event_eth_rx_adapter));
	rx_adapter->eventdev_id = conf->eventdev_id;
	rx_adapter->event_port_id = conf->rx_event_port_id;
	rx_adapter->max_nb_rx = conf->max_nb_rx;
	rx_adapter->socket_id = socket_id;
	rx_adapter->eth_devices = rte_zmalloc_socket("eth_devices",
						     rte_eth_dev_count() *
						     sizeof(struct
							    eth_device_info), 0,
						     socket_id);
	if (!rx_adapter->eth_devices) {
		rte_free(rx_adapter);
		return -ENOMEM;
	}
	rte_spinlock_init(&rx_adapter->rx_lock);

	for (i = 0; i < rte_eth_dev_count(); i++) {
		rx_adapter->eth_devices[i].dev = &rte_eth_devices[i];
		rx_adapter->eth_devices[i].rx_queue =
		    rte_zmalloc_socket("eth_devices",
				       rte_eth_devices[i].data->nb_rx_queues *
				       sizeof(struct eth_rx_queue_info), 0,
				       socket_id);
		if (!rx_adapter->eth_devices[i].rx_queue) {
			ret = -ENOMEM;
			goto done;
		}
	}

	struct rte_service_spec service;
	memset(&service, 0, sizeof(service));
	snprintf(service.name, RTE_SERVICE_NAME_MAX, "%s", conf->service_name);
	service.socket_id = socket_id;
	service.callback = event_eth_rx_adapter_service_func;
	service.callback_userdata = (void *)(uintptr_t) id;
	/* Service function handles locking for queue add/del updates */
	service.capabilities = RTE_SERVICE_CAP_MT_SAFE;
	ret = rte_service_register(&service);
	if (!ret) {
		rx_adapter->service = rte_service_get_by_name(service.name);
		rte_event_eth_rx_adapter[id] = rx_adapter;
	}

 done:
	if (ret && rx_adapter->eth_devices) {
		uint8_t j;
		for (j = 0; j < i; j++)
			rte_free(rx_adapter->eth_devices[j].rx_queue);
		rte_free(rx_adapter->eth_devices);
		rx_adapter->eth_devices = NULL;
	}

	return ret;
}

int rte_event_eth_rx_adapter_free(uint8_t id)
{
	struct rte_event_eth_rx_adapter *rx_adapter;
	uint32_t i;
	int ret;

	rx_adapter = rte_event_eth_rx_adapter[id];
	if (!valid_id(id) || !rte_event_eth_rx_adapter || !rx_adapter)
		return -EINVAL;

	ret = rte_service_unregister(rx_adapter->service);
	if (!ret) {
		RTE_LOG(ERR, EVENTDEV, "%s() failed to unregister service err = %d\n",
			__func__, ret);
	}

	for (i = 0; i < rte_eth_dev_count(); i++)
			rte_free(rx_adapter->eth_devices[i].rx_queue);

	rte_free(rx_adapter->eth_rx_poll);
	rte_free(rx_adapter->wrr_sched);
	rte_free(rx_adapter->eth_devices);
	rte_free(rx_adapter);
	rte_event_eth_rx_adapter[id] = NULL;

	return ret;
}

static int
_rte_event_eth_rx_adapter_queue_del(struct rte_event_eth_rx_adapter *rx_adapter,
				    struct eth_device_info *dev_info,
				    uint16_t rx_queue_id)
{
	struct eth_rx_queue_info *queue_info;

	queue_info = &dev_info->rx_queue[rx_queue_id];
	if (rx_queue_id >= dev_info->dev->data->nb_rx_queues ||
	    !queue_info->queue_enabled)
		return -EINVAL;

	rx_adapter->num_rx_polled--;
	queue_info->queue_enabled = false;

	return 0;
}

static int
_rte_event_eth_rx_adapter_queue_add(struct rte_event_eth_rx_adapter *rx_adapter,
				    struct eth_device_info *dev_info,
				    uint16_t rx_queue_id,
				    const struct rte_event_eth_rx_adapter_queue_conf *conf)
{
	int ret;
	struct eth_rx_queue_info *queue_info;
	const struct rte_event *ev;

	if (rx_queue_id >= dev_info->dev->data->nb_rx_queues)
		return -EINVAL;

	queue_info = &dev_info->rx_queue[rx_queue_id];
	if (queue_info->queue_enabled)
		return -EEXIST;

	ev = &conf->ev;
	memset(queue_info, 0, sizeof(*queue_info));
	queue_info->event_queue_id = ev->queue_id;
	queue_info->sched_type = ev->sched_type;
	queue_info->priority = ev->priority;
	queue_info->wt = conf->servicing_weight;

	if (queue_info->wt == 0) {
		struct rte_eth_dev_data *data = dev_info->dev->data;

		/* If Rx interrupts are disabled set wt = 1 */
		queue_info->wt = !data->dev_conf.intr_conf.rxq;
	}

	if (conf->
	    rx_queue_flags & RTE_EVENT_ETH_RX_ADAPTER_QUEUE_FLOW_ID_VALID) {
		queue_info->flow_id = ev->flow_id;
		queue_info->flow_id_mask = ~0;
	}

	queue_info->queue_enabled = true;
	rx_adapter->num_rx_polled++;
	ret = eth_poll_wrr_calc(rx_adapter);
	if (ret) {
		rx_adapter->num_rx_polled--;
		queue_info->queue_enabled = false;
		return ret;
	}

	return 0;
}

int
rte_event_eth_rx_adapter_queue_add(uint8_t id,
				   uint8_t eth_dev_id,
				   int32_t rx_queue_id,
				   const struct rte_event_eth_rx_adapter_queue_conf *conf)
{
	int ret = 0;
	struct rte_event_eth_rx_adapter *rx_adapter;
	struct eth_device_info *dev_info;
	uint32_t i, j;

	rx_adapter = rte_event_eth_rx_adapter[id];
	if (!valid_id(id) || !rte_event_eth_rx_adapter || !rx_adapter ||
	    eth_dev_id >= rte_eth_dev_count() || !conf)
		return -EINVAL;

	rte_spinlock_lock(&rx_adapter->rx_lock);

	dev_info = &rx_adapter->eth_devices[eth_dev_id];
	if (rx_queue_id == -1) {
		for (i = 0; i < dev_info->dev->data->nb_rx_queues; i++) {
			ret =
			    _rte_event_eth_rx_adapter_queue_add(rx_adapter, dev_info, i,
								conf);
			if (ret) {
				for (j = 0; j < i; j++)
					_rte_event_eth_rx_adapter_queue_del(rx_adapter,
									    dev_info,
									    j);
			}
		}

	} else {
		ret = _rte_event_eth_rx_adapter_queue_add(rx_adapter, dev_info,
							  (uint16_t)rx_queue_id,
							  conf);
	}

	rte_spinlock_unlock(&rx_adapter->rx_lock);

	return ret;
}

int rte_event_eth_rx_adapter_queue_del(uint8_t id, uint8_t eth_dev_id,
				       int32_t rx_queue_id)
{
	int ret = 0;
	struct rte_event_eth_rx_adapter *rx_adapter;
	struct eth_device_info *dev_info;
	unsigned int i;

	rx_adapter = rte_event_eth_rx_adapter[id];
	if (!valid_id(id) || !rte_event_eth_rx_adapter || !rx_adapter ||
	    eth_dev_id >= rte_eth_dev_count())
		return -EINVAL;

	rte_spinlock_lock(&rx_adapter->rx_lock);

	dev_info = &rx_adapter->eth_devices[eth_dev_id];
	if (rx_queue_id == -1) {
		for (i = 0; i < dev_info->dev->data->nb_rx_queues; i++) {
			ret =
			    _rte_event_eth_rx_adapter_queue_del(rx_adapter, dev_info,
								i);
			if (ret)
				break;
		}
	} else {
		ret = _rte_event_eth_rx_adapter_queue_del(rx_adapter, dev_info,
							  (uint16_t)rx_queue_id);
	}

	rte_spinlock_unlock(&rx_adapter->rx_lock);

	return ret;
}

int rte_event_eth_rx_adapter_stats_get(uint8_t id,
				       struct rte_event_eth_rx_adapter_stats *stats)
{
	struct rte_event_eth_rx_adapter *rx_adapter;

	rx_adapter = rte_event_eth_rx_adapter[id];
	if (!valid_id(id) || !rte_event_eth_rx_adapter || !rx_adapter || !stats)
		return -EINVAL;
	*stats = rx_adapter->stats;
	return 0;
}

int rte_event_eth_rx_adapter_stats_reset(uint8_t id)
{
	struct rte_event_eth_rx_adapter *rx_adapter;

	rx_adapter = rte_event_eth_rx_adapter[id];
	if (!valid_id(id) || !rte_event_eth_rx_adapter || !rx_adapter)
		return -EINVAL;
	memset(&rx_adapter->stats, 0, sizeof(rx_adapter->stats));
	return 0;
}
