/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2016 Intel Corporation. All rights reserved.
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

#include <linux/vhost.h>
#include <linux/virtio_net.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#ifdef RTE_LIBRTE_VHOST_NUMA
#include <numaif.h>
#endif

#include <rte_ethdev.h>
#include <rte_log.h>
#include <rte_string_fns.h>
#include <rte_memory.h>
#include <rte_malloc.h>
#include <rte_rwlock.h>
#include <rte_vhost.h>

#include "iotlb.h"
#include "vhost.h"

struct vhost_device {
	struct virtio_net *dev;
	rte_rwlock_t lock;
};

/* Declared as static so that .lock is initialized */
static struct vhost_device vhost_devices[MAX_VHOST_DEVICE];

static inline struct virtio_net *
__get_device(int vid)
{
	struct virtio_net *dev;

	dev = vhost_devices[vid].dev;

	if (unlikely(!dev)) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"(%d) device not found.\n", vid);
	}

	return dev;
}

struct virtio_net *
get_device(int vid)
{
	struct virtio_net *dev;

	rte_rwlock_read_lock(&vhost_devices[vid].lock);

	dev = __get_device(vid);
	if (unlikely(!dev))
		rte_rwlock_read_unlock(&vhost_devices[vid].lock);

	return dev;
}

void
put_device(int vid)
{
	rte_rwlock_read_unlock(&vhost_devices[vid].lock);
}

static struct virtio_net *
get_device_wr(int vid)
{
	struct virtio_net *dev;

	rte_rwlock_write_lock(&vhost_devices[vid].lock);

	dev = __get_device(vid);
	if (unlikely(!dev))
		rte_rwlock_write_unlock(&vhost_devices[vid].lock);

	return dev;
}

static void
put_device_wr(int vid)
{
	rte_rwlock_write_unlock(&vhost_devices[vid].lock);
}

int
realloc_device(int vid, int vq_index, int node)
{
	struct virtio_net *dev, *old_dev;
	struct vhost_virtqueue *vq;

	dev = rte_malloc_socket(NULL, sizeof(*dev), 0, node);
	if (!dev)
		return -1;

	vq = rte_malloc_socket(NULL, sizeof(*vq), 0, node);
	if (!vq)
		return -1;

	old_dev = get_device_wr(vid);
	if (!old_dev)
		return -1;

	memcpy(dev, old_dev, sizeof(*dev));
	memcpy(vq, old_dev->virtqueue[vq_index], sizeof(*vq));
	dev->virtqueue[vq_index] = vq;

	rte_free(old_dev->virtqueue[vq_index]);
	rte_free(old_dev);

	vhost_devices[vid].dev = dev;

	put_device_wr(vid);

	return 0;
}

static void
cleanup_vq(struct vhost_virtqueue *vq, int destroy)
{
	if ((vq->callfd >= 0) && (destroy != 0))
		close(vq->callfd);
	if (vq->kickfd >= 0)
		close(vq->kickfd);
}

/*
 * Unmap any memory, close any file descriptors and
 * free any memory owned by a device.
 */
void
cleanup_device(struct virtio_net *dev, int destroy)
{
	uint32_t i;

	vhost_backend_cleanup(dev);

	for (i = 0; i < dev->nr_vring; i++)
		cleanup_vq(dev->virtqueue[i], destroy);
}

/*
 * Release virtqueues and device memory.
 */
static void
free_device(struct virtio_net *dev)
{
	uint32_t i;
	struct vhost_virtqueue *vq;

	for (i = 0; i < dev->nr_vring; i++) {
		vq = dev->virtqueue[i];

		rte_free(vq->shadow_used_ring);
		rte_mempool_free(vq->iotlb_pool);

		rte_free(vq);
	}

	rte_free(dev);
}

static void
init_vring_queue(struct virtio_net *dev, uint32_t vring_idx)
{
	struct vhost_virtqueue *vq = dev->virtqueue[vring_idx];

	memset(vq, 0, sizeof(struct vhost_virtqueue));

	vq->kickfd = VIRTIO_UNINITIALIZED_EVENTFD;
	vq->callfd = VIRTIO_UNINITIALIZED_EVENTFD;

	vhost_user_iotlb_init(dev, vring_idx);
	/* Backends are set to -1 indicating an inactive device. */
	vq->backend = -1;

	/*
	 * always set the vq to enabled; this is to keep compatibility
	 * with the old QEMU, whereas there is no SET_VRING_ENABLE message.
	 */
	vq->enabled = 1;

	TAILQ_INIT(&vq->zmbuf_list);
}

static void
reset_vring_queue(struct virtio_net *dev, uint32_t vring_idx)
{
	struct vhost_virtqueue *vq = dev->virtqueue[vring_idx];
	int callfd;

	callfd = vq->callfd;
	init_vring_queue(dev, vring_idx);
	vq->callfd = callfd;
}

int
alloc_vring_queue(struct virtio_net *dev, uint32_t vring_idx)
{
	struct vhost_virtqueue *vq;

	vq = rte_malloc(NULL, sizeof(struct vhost_virtqueue), 0);
	if (vq == NULL) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"Failed to allocate memory for vring:%u.\n", vring_idx);
		return -1;
	}

	dev->virtqueue[vring_idx] = vq;
	init_vring_queue(dev, vring_idx);

	dev->nr_vring += 1;

	return 0;
}

/*
 * Reset some variables in device structure, while keeping few
 * others untouched, such as vid, ifname, nr_vring: they
 * should be same unless the device is removed.
 */
void
reset_device(struct virtio_net *dev)
{
	uint32_t i;

	dev->features = 0;
	dev->protocol_features = 0;
	dev->flags = 0;

	for (i = 0; i < dev->nr_vring; i++)
		reset_vring_queue(dev, i);
}

/*
 * Invoked when there is a new vhost-user connection established (when
 * there is a new virtio device being attached).
 */
int
vhost_new_device(void)
{
	struct virtio_net *dev;
	int i;

	dev = rte_zmalloc(NULL, sizeof(struct virtio_net), 0);
	if (dev == NULL) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"Failed to allocate memory for new dev.\n");
		return -1;
	}

	for (i = 0; i < MAX_VHOST_DEVICE; i++) {
		if (vhost_devices[i].dev == NULL)
			break;
	}
	if (i == MAX_VHOST_DEVICE) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"Failed to find a free slot for new device.\n");
		rte_free(dev);
		return -1;
	}

	rte_rwlock_write_lock(&vhost_devices[i].lock);
	vhost_devices[i].dev = dev;
	dev->vid = i;
	rte_rwlock_write_unlock(&vhost_devices[i].lock);

	return i;
}

/*
 * Invoked when there is the vhost-user connection is broken (when
 * the virtio device is being detached).
 */
void
vhost_destroy_device(int vid)
{
	struct virtio_net *dev = get_device(vid);

	if (dev == NULL)
		return;

	if (dev->flags & VIRTIO_DEV_RUNNING) {
		dev->flags &= ~VIRTIO_DEV_RUNNING;
		dev->notify_ops->destroy_device(vid);
	}

	put_device(vid);
	dev = get_device_wr(vid);

	cleanup_device(dev, 1);
	free_device(dev);

	vhost_devices[vid].dev = NULL;

	put_device_wr(vid);
}

void
vhost_set_ifname(int vid, const char *if_name, unsigned int if_len)
{
	struct virtio_net *dev;
	unsigned int len;

	dev = get_device(vid);
	if (dev == NULL)
		return;

	len = if_len > sizeof(dev->ifname) ?
		sizeof(dev->ifname) : if_len;

	strncpy(dev->ifname, if_name, len);
	dev->ifname[sizeof(dev->ifname) - 1] = '\0';

	put_device(vid);
}

void
vhost_enable_dequeue_zero_copy(int vid)
{
	struct virtio_net *dev = get_device(vid);

	if (dev == NULL)
		return;

	dev->dequeue_zero_copy = 1;

	put_device(vid);
}

int
rte_vhost_get_mtu(int vid, uint16_t *mtu)
{
	struct virtio_net *dev = get_device(vid);
	int ret = 0;

	if (!dev)
		return -ENODEV;

	if (!(dev->flags & VIRTIO_DEV_READY))
		ret = -EAGAIN;

	if (!(dev->features & VIRTIO_NET_F_MTU))
		ret = -ENOTSUP;

	*mtu = dev->mtu;

	put_device(vid);

	return ret;
}

int
rte_vhost_get_numa_node(int vid)
{
#ifdef RTE_LIBRTE_VHOST_NUMA
	struct virtio_net *dev = get_device(vid);
	int numa_node;
	int ret;

	if (dev == NULL)
		return -1;

	ret = get_mempolicy(&numa_node, NULL, 0, dev,
			    MPOL_F_NODE | MPOL_F_ADDR);
	if (ret < 0) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"(%d) failed to query numa node: %d\n", vid, ret);
		numa_node = -1;
	}

	put_device(vid);

	return numa_node;
#else
	RTE_SET_USED(vid);
	return -1;
#endif
}

uint32_t
rte_vhost_get_queue_num(int vid)
{
	struct virtio_net *dev = get_device(vid);
	uint32_t queue_num;

	if (dev == NULL)
		return 0;

	queue_num = dev->nr_vring / 2;

	put_device(vid);

	return queue_num;
}

uint16_t
rte_vhost_get_vring_num(int vid)
{
	struct virtio_net *dev = get_device(vid);
	uint16_t vring_num;

	if (dev == NULL)
		return 0;

	vring_num = dev->nr_vring;

	put_device(vid);

	return vring_num;
}

int
rte_vhost_get_ifname(int vid, char *buf, size_t len)
{
	struct virtio_net *dev = get_device(vid);

	if (dev == NULL)
		return -1;

	len = RTE_MIN(len, sizeof(dev->ifname));

	strncpy(buf, dev->ifname, len);
	buf[len - 1] = '\0';

	put_device(vid);

	return 0;
}

int
rte_vhost_get_negotiated_features(int vid, uint64_t *features)
{
	struct virtio_net *dev;

	dev = get_device(vid);
	if (!dev)
		return -1;

	*features = dev->features;

	put_device(vid);

	return 0;
}

int
rte_vhost_get_mem_table(int vid, struct rte_vhost_memory **mem)
{
	struct virtio_net *dev;
	struct rte_vhost_memory *m;
	size_t size;
	int ret = 0;

	dev = get_device(vid);
	if (!dev)
		return -1;

	size = dev->mem->nregions * sizeof(struct rte_vhost_mem_region);
	m = malloc(sizeof(struct rte_vhost_memory) + size);
	if (!m) {
		ret = -1;
		goto out;
	}

	m->nregions = dev->mem->nregions;
	memcpy(m->regions, dev->mem->regions, size);
	*mem = m;

out:
	put_device(vid);

	return ret;
}

int
rte_vhost_get_vhost_vring(int vid, uint16_t vring_idx,
			  struct rte_vhost_vring *vring)
{
	struct virtio_net *dev;
	struct vhost_virtqueue *vq;
	int ret = 0;

	dev = get_device(vid);
	if (!dev)
		return -1;

	if (vring_idx >= VHOST_MAX_VRING) {
		ret = -1;
		goto out;
	}

	vq = dev->virtqueue[vring_idx];
	if (!vq) {
		ret = -1;
		goto out;
	}

	vring->desc  = vq->desc;
	vring->avail = vq->avail;
	vring->used  = vq->used;
	vring->log_guest_addr  = vq->log_guest_addr;

	vring->callfd  = vq->callfd;
	vring->kickfd  = vq->kickfd;
	vring->size    = vq->size;

out:
	put_device(vid);

	return ret;
}

uint16_t
rte_vhost_avail_entries(int vid, uint16_t queue_id)
{
	struct virtio_net *dev;
	struct vhost_virtqueue *vq;
	uint16_t avail_entries = 0;

	dev = get_device(vid);
	if (!dev)
		return 0;

	vq = dev->virtqueue[queue_id];
	if (!vq->enabled)
		goto out;


	avail_entries = *(volatile uint16_t *)&vq->avail->idx;
	avail_entries -= vq->last_used_idx;

out:
	put_device(vid);

	return avail_entries;
}

int
rte_vhost_enable_guest_notification(int vid, uint16_t queue_id, int enable)
{
	struct virtio_net *dev = get_device(vid);
	int ret = 0;

	if (dev == NULL)
		return -1;

	if (enable) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"guest notification isn't supported.\n");
		ret = -1;
		goto out;
	}

	dev->virtqueue[queue_id]->used->flags = VRING_USED_F_NO_NOTIFY;

out:
	put_device(vid);

	return ret;
}

void
rte_vhost_log_write(int vid, uint64_t addr, uint64_t len)
{
	struct virtio_net *dev = get_device(vid);

	if (dev == NULL)
		return;

	vhost_log_write(dev, addr, len);

	put_device(vid);
}

void
rte_vhost_log_used_vring(int vid, uint16_t vring_idx,
			 uint64_t offset, uint64_t len)
{
	struct virtio_net *dev;
	struct vhost_virtqueue *vq;

	dev = get_device(vid);
	if (dev == NULL)
		return;

	if (vring_idx >= VHOST_MAX_VRING)
		goto out;
	vq = dev->virtqueue[vring_idx];
	if (!vq)
		goto out;

	vhost_log_used_vring(dev, vq, offset, len);

out:
	put_device(vid);
}

uint32_t
rte_vhost_rx_queue_count(int vid, uint16_t qid)
{
	struct virtio_net *dev;
	struct vhost_virtqueue *vq;
	uint32_t queue_count;

	dev = get_device(vid);
	if (dev == NULL)
		return 0;

	if (unlikely(qid >= dev->nr_vring || (qid & 1) == 0)) {
		RTE_LOG(ERR, VHOST_DATA, "(%d) %s: invalid virtqueue idx %d.\n",
			dev->vid, __func__, qid);
		queue_count = 0;
		goto out;
	}

	vq = dev->virtqueue[qid];
	if (vq == NULL) {
		queue_count = 0;
		goto out;
	}

	if (unlikely(vq->enabled == 0 || vq->avail == NULL)) {
		queue_count = 0;
		goto out;
	}

	queue_count = *((volatile uint16_t *)&vq->avail->idx);
	queue_count -= vq->last_avail_idx;

out:
	put_device(vid);

	return queue_count;
}
