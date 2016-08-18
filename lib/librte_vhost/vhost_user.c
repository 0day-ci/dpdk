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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <assert.h>
#ifdef RTE_LIBRTE_VHOST_NUMA
#include <numaif.h>
#endif

#include <rte_common.h>
#include <rte_malloc.h>
#include <rte_log.h>

#include "vhost.h"
#include "vhost_user.h"

static const char *vhost_message_str[VHOST_USER_MAX] = {
	[VHOST_USER_NONE] = "VHOST_USER_NONE",
	[VHOST_USER_GET_FEATURES] = "VHOST_USER_GET_FEATURES",
	[VHOST_USER_SET_FEATURES] = "VHOST_USER_SET_FEATURES",
	[VHOST_USER_SET_OWNER] = "VHOST_USER_SET_OWNER",
	[VHOST_USER_RESET_OWNER] = "VHOST_USER_RESET_OWNER",
	[VHOST_USER_SET_MEM_TABLE] = "VHOST_USER_SET_MEM_TABLE",
	[VHOST_USER_SET_LOG_BASE] = "VHOST_USER_SET_LOG_BASE",
	[VHOST_USER_SET_LOG_FD] = "VHOST_USER_SET_LOG_FD",
	[VHOST_USER_SET_VRING_NUM] = "VHOST_USER_SET_VRING_NUM",
	[VHOST_USER_SET_VRING_ADDR] = "VHOST_USER_SET_VRING_ADDR",
	[VHOST_USER_SET_VRING_BASE] = "VHOST_USER_SET_VRING_BASE",
	[VHOST_USER_GET_VRING_BASE] = "VHOST_USER_GET_VRING_BASE",
	[VHOST_USER_SET_VRING_KICK] = "VHOST_USER_SET_VRING_KICK",
	[VHOST_USER_SET_VRING_CALL] = "VHOST_USER_SET_VRING_CALL",
	[VHOST_USER_SET_VRING_ERR]  = "VHOST_USER_SET_VRING_ERR",
	[VHOST_USER_GET_PROTOCOL_FEATURES]  = "VHOST_USER_GET_PROTOCOL_FEATURES",
	[VHOST_USER_SET_PROTOCOL_FEATURES]  = "VHOST_USER_SET_PROTOCOL_FEATURES",
	[VHOST_USER_GET_QUEUE_NUM]  = "VHOST_USER_GET_QUEUE_NUM",
	[VHOST_USER_SET_VRING_ENABLE]  = "VHOST_USER_SET_VRING_ENABLE",
	[VHOST_USER_SEND_RARP]  = "VHOST_USER_SEND_RARP",
};

struct orig_region_map {
	int fd;
	uint64_t mapped_address;
	uint64_t mapped_size;
	uint64_t blksz;
};

#define orig_region(ptr, nregions) \
	((struct orig_region_map *)RTE_PTR_ADD((ptr), \
		sizeof(struct virtio_memory) + \
		sizeof(struct virtio_memory_regions) * (nregions)))

static uint64_t
get_blk_size(int fd)
{
	struct stat stat;
	int ret;

	ret = fstat(fd, &stat);
	return ret == -1 ? (uint64_t)-1 : (uint64_t)stat.st_blksize;
}

static void
free_mem_region(struct virtio_net *dev)
{
	struct orig_region_map *region;
	unsigned int idx;

	if (!dev || !dev->mem)
		return;

	region = orig_region(dev->mem, dev->mem->nregions);
	for (idx = 0; idx < dev->mem->nregions; idx++) {
		if (region[idx].mapped_address) {
			munmap((void *)(uintptr_t)region[idx].mapped_address,
					region[idx].mapped_size);
			close(region[idx].fd);
		}
	}
}

void
vhost_backend_cleanup(struct virtio_net *dev)
{
	if (dev->mem) {
		free_mem_region(dev);
		free(dev->mem);
		dev->mem = NULL;
	}
	if (dev->log_addr) {
		munmap((void *)(uintptr_t)dev->log_addr, dev->log_size);
		dev->log_addr = 0;
	}
}

/*
 * This function just returns success at the moment unless
 * the device hasn't been initialised.
 */
static int
vhost_set_owner(int vid)
{
	struct virtio_net *dev;

	dev = get_device(vid);
	if (dev == NULL)
		return -1;

	return 0;
}

static int
vhost_reset_owner(int vid)
{
	struct virtio_net *dev;

	dev = get_device(vid);
	if (dev == NULL)
		return -1;

	if (dev->flags & VIRTIO_DEV_RUNNING) {
		dev->flags &= ~VIRTIO_DEV_RUNNING;
		notify_ops->destroy_device(vid);
	}

	cleanup_device(dev, 0);
	reset_device(dev);
	return 0;
}

/*
 * The features that we support are requested.
 */
static int
vhost_get_features(int vid, uint64_t *pu)
{
	struct virtio_net *dev;

	dev = get_device(vid);
	if (dev == NULL)
		return -1;

	/* Send our supported features. */
	*pu = VHOST_FEATURES;
	return 0;
}

/*
 * We receive the negotiated features supported by us and the virtio device.
 */
static int
vhost_set_features(int vid, uint64_t *pu)
{
	struct virtio_net *dev;

	dev = get_device(vid);
	if (dev == NULL)
		return -1;
	if (*pu & ~VHOST_FEATURES)
		return -1;

	dev->features = *pu;
	if (dev->features &
		((1 << VIRTIO_NET_F_MRG_RXBUF) | (1ULL << VIRTIO_F_VERSION_1))) {
		dev->vhost_hlen = sizeof(struct virtio_net_hdr_mrg_rxbuf);
	} else {
		dev->vhost_hlen = sizeof(struct virtio_net_hdr);
	}
	LOG_DEBUG(VHOST_CONFIG,
		"(%d) mergeable RX buffers %s, virtio 1 %s\n",
		dev->vid,
		(dev->features & (1 << VIRTIO_NET_F_MRG_RXBUF)) ? "on" : "off",
		(dev->features & (1ULL << VIRTIO_F_VERSION_1)) ? "on" : "off");

	return 0;
}

/*
 * The virtio device sends us the size of the descriptor ring.
 */
static int
vhost_set_vring_num(int vid, struct vhost_vring_state *state)
{
	struct virtio_net *dev;

	dev = get_device(vid);
	if (dev == NULL)
		return -1;

	/* State->index refers to the queue index. The txq is 1, rxq is 0. */
	dev->virtqueue[state->index]->size = state->num;

	return 0;
}

/*
 * Reallocate virtio_dev and vhost_virtqueue data structure to make them on the
 * same numa node as the memory of vring descriptor.
 */
#ifdef RTE_LIBRTE_VHOST_NUMA
static struct virtio_net*
numa_realloc(struct virtio_net *dev, int index)
{
	int oldnode, newnode;
	struct virtio_net *old_dev;
	struct vhost_virtqueue *old_vq, *vq;
	int ret;

	/*
	 * vq is allocated on pairs, we should try to do realloc
	 * on first queue of one queue pair only.
	 */
	if (index % VIRTIO_QNUM != 0)
		return dev;

	old_dev = dev;
	vq = old_vq = dev->virtqueue[index];

	ret = get_mempolicy(&newnode, NULL, 0, old_vq->desc,
			    MPOL_F_NODE | MPOL_F_ADDR);

	/* check if we need to reallocate vq */
	ret |= get_mempolicy(&oldnode, NULL, 0, old_vq,
			     MPOL_F_NODE | MPOL_F_ADDR);
	if (ret) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"Unable to get vq numa information.\n");
		return dev;
	}
	if (oldnode != newnode) {
		RTE_LOG(INFO, VHOST_CONFIG,
			"reallocate vq from %d to %d node\n", oldnode, newnode);
		vq = rte_malloc_socket(NULL, sizeof(*vq) * VIRTIO_QNUM, 0,
				       newnode);
		if (!vq)
			return dev;

		memcpy(vq, old_vq, sizeof(*vq) * VIRTIO_QNUM);
		rte_free(old_vq);
	}

	/* check if we need to reallocate dev */
	ret = get_mempolicy(&oldnode, NULL, 0, old_dev,
			    MPOL_F_NODE | MPOL_F_ADDR);
	if (ret) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"Unable to get dev numa information.\n");
		goto out;
	}
	if (oldnode != newnode) {
		RTE_LOG(INFO, VHOST_CONFIG,
			"reallocate dev from %d to %d node\n",
			oldnode, newnode);
		dev = rte_malloc_socket(NULL, sizeof(*dev), 0, newnode);
		if (!dev) {
			dev = old_dev;
			goto out;
		}

		memcpy(dev, old_dev, sizeof(*dev));
		rte_free(old_dev);
	}

out:
	dev->virtqueue[index] = vq;
	dev->virtqueue[index + 1] = vq + 1;
	vhost_devices[dev->vid] = dev;

	return dev;
}
#else
static struct virtio_net*
numa_realloc(struct virtio_net *dev, int index __rte_unused)
{
	return dev;
}
#endif

/*
 * Converts QEMU virtual address to Vhost virtual address. This function is
 * used to convert the ring addresses to our address space.
 */
static uint64_t
qva_to_vva(struct virtio_net *dev, uint64_t qemu_va)
{
	struct virtio_memory_regions *region;
	uint64_t vhost_va = 0;
	uint32_t regionidx = 0;

	/* Find the region where the address lives. */
	for (regionidx = 0; regionidx < dev->mem->nregions; regionidx++) {
		region = &dev->mem->regions[regionidx];
		if ((qemu_va >= region->userspace_address) &&
			(qemu_va <= region->userspace_address +
			region->memory_size)) {
			vhost_va = qemu_va + region->guest_phys_address +
				region->address_offset -
				region->userspace_address;
			break;
		}
	}
	return vhost_va;
}

/*
 * The virtio device sends us the desc, used and avail ring addresses.
 * This function then converts these to our address space.
 */
static int
vhost_set_vring_addr(int vid, struct vhost_vring_addr *addr)
{
	struct virtio_net *dev;
	struct vhost_virtqueue *vq;

	dev = get_device(vid);
	if ((dev == NULL) || (dev->mem == NULL))
		return -1;

	/* addr->index refers to the queue index. The txq 1, rxq is 0. */
	vq = dev->virtqueue[addr->index];

	/* The addresses are converted from QEMU virtual to Vhost virtual. */
	vq->desc = (struct vring_desc *)(uintptr_t)qva_to_vva(dev,
			addr->desc_user_addr);
	if (vq->desc == 0) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"(%d) failed to find desc ring address.\n",
			dev->vid);
		return -1;
	}

	dev = numa_realloc(dev, addr->index);
	vq = dev->virtqueue[addr->index];

	vq->avail = (struct vring_avail *)(uintptr_t)qva_to_vva(dev,
			addr->avail_user_addr);
	if (vq->avail == 0) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"(%d) failed to find avail ring address.\n",
			dev->vid);
		return -1;
	}

	vq->used = (struct vring_used *)(uintptr_t)qva_to_vva(dev,
			addr->used_user_addr);
	if (vq->used == 0) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"(%d) failed to find used ring address.\n",
			dev->vid);
		return -1;
	}

	if (vq->last_used_idx != vq->used->idx) {
		RTE_LOG(WARNING, VHOST_CONFIG,
			"last_used_idx (%u) and vq->used->idx (%u) mismatches; "
			"some packets maybe resent for Tx and dropped for Rx\n",
			vq->last_used_idx, vq->used->idx);
		vq->last_used_idx     = vq->used->idx;
	}

	vq->log_guest_addr = addr->log_guest_addr;

	LOG_DEBUG(VHOST_CONFIG, "(%d) mapped address desc: %p\n",
			dev->vid, vq->desc);
	LOG_DEBUG(VHOST_CONFIG, "(%d) mapped address avail: %p\n",
			dev->vid, vq->avail);
	LOG_DEBUG(VHOST_CONFIG, "(%d) mapped address used: %p\n",
			dev->vid, vq->used);
	LOG_DEBUG(VHOST_CONFIG, "(%d) log_guest_addr: %" PRIx64 "\n",
			dev->vid, vq->log_guest_addr);

	return 0;
}

/*
 * The virtio device sends us the available ring last used index.
 */
static int
vhost_set_vring_base(int vid, struct vhost_vring_state *state)
{
	struct virtio_net *dev;

	dev = get_device(vid);
	if (dev == NULL)
		return -1;

	/* State->index refers to the queue index. The txq is 1, rxq is 0. */
	dev->virtqueue[state->index]->last_used_idx = state->num;

	return 0;
}

/*
 * We send the virtio device our available ring last used index.
 */
static int
vhost_get_vring_base(int vid, uint32_t index,
	struct vhost_vring_state *state)
{
	struct virtio_net *dev;

	dev = get_device(vid);
	if (dev == NULL)
		return -1;

	state->index = index;
	/* State->index refers to the queue index. The txq is 1, rxq is 0. */
	state->num = dev->virtqueue[state->index]->last_used_idx;

	return 0;
}

/*
 * The virtio device sends an eventfd to interrupt the guest. This fd gets
 * copied into our process space.
 */
static int
vhost_set_vring_call(int vid, struct vhost_vring_file *file)
{
	struct virtio_net *dev;
	struct vhost_virtqueue *vq;
	uint32_t cur_qp_idx = file->index / VIRTIO_QNUM;

	dev = get_device(vid);
	if (dev == NULL)
		return -1;

	/*
	 * FIXME: VHOST_SET_VRING_CALL is the first per-vring message
	 * we get, so we do vring queue pair allocation here.
	 */
	if (cur_qp_idx + 1 > dev->virt_qp_nb) {
		if (alloc_vring_queue_pair(dev, cur_qp_idx) < 0)
			return -1;
	}

	/* file->index refers to the queue index. The txq is 1, rxq is 0. */
	vq = dev->virtqueue[file->index];
	assert(vq != NULL);

	if (vq->callfd >= 0)
		close(vq->callfd);

	vq->callfd = file->fd;

	return 0;
}

/*
 * The virtio device sends an eventfd that it can use to notify us.
 * This fd gets copied into our process space.
 */
static int
vhost_set_vring_kick(int vid, struct vhost_vring_file *file)
{
	struct virtio_net *dev;
	struct vhost_virtqueue *vq;

	dev = get_device(vid);
	if (dev == NULL)
		return -1;

	/* file->index refers to the queue index. The txq is 1, rxq is 0. */
	vq = dev->virtqueue[file->index];

	if (vq->kickfd >= 0)
		close(vq->kickfd);

	vq->kickfd = file->fd;

	return 0;
}

static int
user_set_mem_table(int vid, struct VhostUserMsg *pmsg)
{
	struct VhostUserMemory memory = pmsg->payload.memory;
	struct virtio_memory_regions *pregion;
	uint64_t mapped_address, mapped_size;
	struct virtio_net *dev;
	unsigned int idx = 0;
	struct orig_region_map *pregion_orig;
	uint64_t alignment;

	/* unmap old memory regions one by one*/
	dev = get_device(vid);
	if (dev == NULL)
		return -1;

	/* Remove from the data plane. */
	if (dev->flags & VIRTIO_DEV_RUNNING) {
		dev->flags &= ~VIRTIO_DEV_RUNNING;
		notify_ops->destroy_device(vid);
	}

	if (dev->mem) {
		free_mem_region(dev);
		free(dev->mem);
		dev->mem = NULL;
	}

	dev->mem = calloc(1,
		sizeof(struct virtio_memory) +
		sizeof(struct virtio_memory_regions) * memory.nregions +
		sizeof(struct orig_region_map) * memory.nregions);
	if (dev->mem == NULL) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"(%d) failed to allocate memory for dev->mem\n",
			dev->vid);
		return -1;
	}
	dev->mem->nregions = memory.nregions;

	pregion_orig = orig_region(dev->mem, memory.nregions);
	for (idx = 0; idx < memory.nregions; idx++) {
		pregion = &dev->mem->regions[idx];
		pregion->guest_phys_address =
			memory.regions[idx].guest_phys_addr;
		pregion->guest_phys_address_end =
			memory.regions[idx].guest_phys_addr +
			memory.regions[idx].memory_size;
		pregion->memory_size =
			memory.regions[idx].memory_size;
		pregion->userspace_address =
			memory.regions[idx].userspace_addr;

		/* This is ugly */
		mapped_size = memory.regions[idx].memory_size +
			memory.regions[idx].mmap_offset;

		/* mmap() without flag of MAP_ANONYMOUS, should be called
		 * with length argument aligned with hugepagesz at older
		 * longterm version Linux, like 2.6.32 and 3.2.72, or
		 * mmap() will fail with EINVAL.
		 *
		 * to avoid failure, make sure in caller to keep length
		 * aligned.
		 */
		alignment = get_blk_size(pmsg->fds[idx]);
		if (alignment == (uint64_t)-1) {
			RTE_LOG(ERR, VHOST_CONFIG,
				"couldn't get hugepage size through fstat\n");
			goto err_mmap;
		}
		mapped_size = RTE_ALIGN_CEIL(mapped_size, alignment);

		mapped_address = (uint64_t)(uintptr_t)mmap(NULL,
			mapped_size,
			PROT_READ | PROT_WRITE, MAP_SHARED,
			pmsg->fds[idx],
			0);

		RTE_LOG(INFO, VHOST_CONFIG,
			"mapped region %d fd:%d to:%p sz:0x%"PRIx64" "
			"off:0x%"PRIx64" align:0x%"PRIx64"\n",
			idx, pmsg->fds[idx], (void *)(uintptr_t)mapped_address,
			mapped_size, memory.regions[idx].mmap_offset,
			alignment);

		if (mapped_address == (uint64_t)(uintptr_t)MAP_FAILED) {
			RTE_LOG(ERR, VHOST_CONFIG,
				"mmap qemu guest failed.\n");
			goto err_mmap;
		}

		pregion_orig[idx].mapped_address = mapped_address;
		pregion_orig[idx].mapped_size = mapped_size;
		pregion_orig[idx].blksz = alignment;
		pregion_orig[idx].fd = pmsg->fds[idx];

		mapped_address +=  memory.regions[idx].mmap_offset;

		pregion->address_offset = mapped_address -
			pregion->guest_phys_address;

		if (memory.regions[idx].guest_phys_addr == 0) {
			dev->mem->base_address =
				memory.regions[idx].userspace_addr;
			dev->mem->mapped_address =
				pregion->address_offset;
		}

		LOG_DEBUG(VHOST_CONFIG,
			"REGION: %u GPA: %p QEMU VA: %p SIZE (%"PRIu64")\n",
			idx,
			(void *)(uintptr_t)pregion->guest_phys_address,
			(void *)(uintptr_t)pregion->userspace_address,
			 pregion->memory_size);
	}

	return 0;

err_mmap:
	while (idx--) {
		munmap((void *)(uintptr_t)pregion_orig[idx].mapped_address,
				pregion_orig[idx].mapped_size);
		close(pregion_orig[idx].fd);
	}
	free(dev->mem);
	dev->mem = NULL;
	return -1;
}

static int
vq_is_ready(struct vhost_virtqueue *vq)
{
	return vq && vq->desc   &&
	       vq->kickfd != VIRTIO_UNINITIALIZED_EVENTFD &&
	       vq->callfd != VIRTIO_UNINITIALIZED_EVENTFD;
}

static int
virtio_is_ready(struct virtio_net *dev)
{
	struct vhost_virtqueue *rvq, *tvq;
	uint32_t i;

	for (i = 0; i < dev->virt_qp_nb; i++) {
		rvq = dev->virtqueue[i * VIRTIO_QNUM + VIRTIO_RXQ];
		tvq = dev->virtqueue[i * VIRTIO_QNUM + VIRTIO_TXQ];

		if (!vq_is_ready(rvq) || !vq_is_ready(tvq)) {
			RTE_LOG(INFO, VHOST_CONFIG,
				"virtio is not ready for processing.\n");
			return 0;
		}
	}

	RTE_LOG(INFO, VHOST_CONFIG,
		"virtio is now ready for processing.\n");
	return 1;
}

static void
user_set_vring_call(int vid, struct VhostUserMsg *pmsg)
{
	struct vhost_vring_file file;

	file.index = pmsg->payload.u64 & VHOST_USER_VRING_IDX_MASK;
	if (pmsg->payload.u64 & VHOST_USER_VRING_NOFD_MASK)
		file.fd = VIRTIO_INVALID_EVENTFD;
	else
		file.fd = pmsg->fds[0];
	RTE_LOG(INFO, VHOST_CONFIG,
		"vring call idx:%d file:%d\n", file.index, file.fd);
	vhost_set_vring_call(vid, &file);
}

/*
 *  In vhost-user, when we receive kick message, will test whether virtio
 *  device is ready for packet processing.
 */
static void
user_set_vring_kick(int vid, struct VhostUserMsg *pmsg)
{
	struct vhost_vring_file file;
	struct virtio_net *dev = get_device(vid);

	if (!dev)
		return;

	file.index = pmsg->payload.u64 & VHOST_USER_VRING_IDX_MASK;
	if (pmsg->payload.u64 & VHOST_USER_VRING_NOFD_MASK)
		file.fd = VIRTIO_INVALID_EVENTFD;
	else
		file.fd = pmsg->fds[0];
	RTE_LOG(INFO, VHOST_CONFIG,
		"vring kick idx:%d file:%d\n", file.index, file.fd);
	vhost_set_vring_kick(vid, &file);

	if (virtio_is_ready(dev) && !(dev->flags & VIRTIO_DEV_RUNNING)) {
		if (notify_ops->new_device(vid) == 0)
			dev->flags |= VIRTIO_DEV_RUNNING;
	}
}

/*
 * when virtio is stopped, qemu will send us the GET_VRING_BASE message.
 */
static int
user_get_vring_base(int vid, struct vhost_vring_state *state)
{
	struct virtio_net *dev = get_device(vid);

	if (dev == NULL)
		return -1;
	/* We have to stop the queue (virtio) if it is running. */
	if (dev->flags & VIRTIO_DEV_RUNNING) {
		dev->flags &= ~VIRTIO_DEV_RUNNING;
		notify_ops->destroy_device(vid);
	}

	/* Here we are safe to get the last used index */
	vhost_get_vring_base(vid, state->index, state);

	RTE_LOG(INFO, VHOST_CONFIG,
		"vring base idx:%d file:%d\n", state->index, state->num);
	/*
	 * Based on current qemu vhost-user implementation, this message is
	 * sent and only sent in vhost_vring_stop.
	 * TODO: cleanup the vring, it isn't usable since here.
	 */
	if (dev->virtqueue[state->index]->kickfd >= 0)
		close(dev->virtqueue[state->index]->kickfd);

	dev->virtqueue[state->index]->kickfd = VIRTIO_UNINITIALIZED_EVENTFD;

	return 0;
}

/*
 * when virtio queues are ready to work, qemu will send us to
 * enable the virtio queue pair.
 */
static int
user_set_vring_enable(int vid, struct vhost_vring_state *state)
{
	struct virtio_net *dev;
	int enable = (int)state->num;

	dev = get_device(vid);
	if (dev == NULL)
		return -1;

	RTE_LOG(INFO, VHOST_CONFIG,
		"set queue enable: %d to qp idx: %d\n",
		enable, state->index);

	if (notify_ops->vring_state_changed)
		notify_ops->vring_state_changed(vid, state->index, enable);

	dev->virtqueue[state->index]->enabled = enable;

	return 0;
}

static void
user_set_protocol_features(int vid, uint64_t protocol_features)
{
	struct virtio_net *dev;

	dev = get_device(vid);
	if (dev == NULL || protocol_features & ~VHOST_USER_PROTOCOL_FEATURES)
		return;

	dev->protocol_features = protocol_features;
}

static int
user_set_log_base(int vid, struct VhostUserMsg *msg)
{
	struct virtio_net *dev;
	int fd = msg->fds[0];
	uint64_t size, off;
	void *addr;

	dev = get_device(vid);
	if (!dev)
		return -1;

	if (fd < 0) {
		RTE_LOG(ERR, VHOST_CONFIG, "invalid log fd: %d\n", fd);
		return -1;
	}

	if (msg->size != sizeof(VhostUserLog)) {
		RTE_LOG(ERR, VHOST_CONFIG,
			"invalid log base msg size: %"PRId32" != %d\n",
			msg->size, (int)sizeof(VhostUserLog));
		return -1;
	}

	size = msg->payload.log.mmap_size;
	off  = msg->payload.log.mmap_offset;
	RTE_LOG(INFO, VHOST_CONFIG,
		"log mmap size: %"PRId64", offset: %"PRId64"\n",
		size, off);

	/*
	 * mmap from 0 to workaround a hugepage mmap bug: mmap will
	 * fail when offset is not page size aligned.
	 */
	addr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (addr == MAP_FAILED) {
		RTE_LOG(ERR, VHOST_CONFIG, "mmap log base failed!\n");
		return -1;
	}

	/*
	 * Free previously mapped log memory on occasionally
	 * multiple VHOST_USER_SET_LOG_BASE.
	 */
	if (dev->log_addr) {
		munmap((void *)(uintptr_t)dev->log_addr, dev->log_size);
	}
	dev->log_addr = (uint64_t)(uintptr_t)addr;
	dev->log_base = dev->log_addr + off;
	dev->log_size = size;

	return 0;
}

/*
 * An rarp packet is constructed and broadcasted to notify switches about
 * the new location of the migrated VM, so that packets from outside will
 * not be lost after migration.
 *
 * However, we don't actually "send" a rarp packet here, instead, we set
 * a flag 'broadcast_rarp' to let rte_vhost_dequeue_burst() inject it.
 */
static int
user_send_rarp(int vid, struct VhostUserMsg *msg)
{
	struct virtio_net *dev;
	uint8_t *mac = (uint8_t *)&msg->payload.u64;

	dev = get_device(vid);
	if (!dev)
		return -1;

	RTE_LOG(DEBUG, VHOST_CONFIG,
		":: mac: %02x:%02x:%02x:%02x:%02x:%02x\n",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	memcpy(dev->mac.addr_bytes, mac, 6);

	/*
	 * Set the flag to inject a RARP broadcast packet at
	 * rte_vhost_dequeue_burst().
	 *
	 * rte_smp_wmb() is for making sure the mac is copied
	 * before the flag is set.
	 */
	rte_smp_wmb();
	rte_atomic16_set(&dev->broadcast_rarp, 1);

	return 0;
}

/* return bytes# of read on success or negative val on failure. */
static int
read_vhost_message(int sockfd, struct VhostUserMsg *msg)
{
	int ret;

	ret = read_fd_message(sockfd, (char *)msg, VHOST_USER_HDR_SIZE,
		msg->fds, VHOST_MEMORY_MAX_NREGIONS);
	if (ret <= 0)
		return ret;

	if (msg && msg->size) {
		if (msg->size > sizeof(msg->payload)) {
			RTE_LOG(ERR, VHOST_CONFIG,
				"invalid msg size: %d\n", msg->size);
			return -1;
		}
		ret = read(sockfd, &msg->payload, msg->size);
		if (ret <= 0)
			return ret;
		if (ret != (int)msg->size) {
			RTE_LOG(ERR, VHOST_CONFIG,
				"read control message failed\n");
			return -1;
		}
	}

	return ret;
}

static int
send_vhost_message(int sockfd, struct VhostUserMsg *msg)
{
	int ret;

	if (!msg)
		return 0;

	msg->flags &= ~VHOST_USER_VERSION_MASK;
	msg->flags |= VHOST_USER_VERSION;
	msg->flags |= VHOST_USER_REPLY_MASK;

	ret = send_fd_message(sockfd, (char *)msg,
		VHOST_USER_HDR_SIZE + msg->size, NULL, 0);

	return ret;
}

int
vhost_user_msg_handler(int vid, int fd)
{
	struct VhostUserMsg msg;
	uint64_t features = 0;
	int ret;

	ret = read_vhost_message(fd, &msg);
	if (ret <= 0 || msg.request >= VHOST_USER_MAX) {
		if (ret < 0)
			RTE_LOG(ERR, VHOST_CONFIG,
				"vhost read message failed\n");
		else if (ret == 0)
			RTE_LOG(INFO, VHOST_CONFIG,
				"vhost peer closed\n");
		else
			RTE_LOG(ERR, VHOST_CONFIG,
				"vhost read incorrect message\n");

		return -1;
	}

	RTE_LOG(INFO, VHOST_CONFIG, "read message %s\n",
		vhost_message_str[msg.request]);
	switch (msg.request) {
	case VHOST_USER_GET_FEATURES:
		ret = vhost_get_features(vid, &features);
		msg.payload.u64 = features;
		msg.size = sizeof(msg.payload.u64);
		send_vhost_message(fd, &msg);
		break;
	case VHOST_USER_SET_FEATURES:
		features = msg.payload.u64;
		vhost_set_features(vid, &features);
		break;

	case VHOST_USER_GET_PROTOCOL_FEATURES:
		msg.payload.u64 = VHOST_USER_PROTOCOL_FEATURES;
		msg.size = sizeof(msg.payload.u64);
		send_vhost_message(fd, &msg);
		break;
	case VHOST_USER_SET_PROTOCOL_FEATURES:
		user_set_protocol_features(vid, msg.payload.u64);
		break;

	case VHOST_USER_SET_OWNER:
		vhost_set_owner(vid);
		break;
	case VHOST_USER_RESET_OWNER:
		vhost_reset_owner(vid);
		break;

	case VHOST_USER_SET_MEM_TABLE:
		user_set_mem_table(vid, &msg);
		break;

	case VHOST_USER_SET_LOG_BASE:
		user_set_log_base(vid, &msg);

		/* it needs a reply */
		msg.size = sizeof(msg.payload.u64);
		send_vhost_message(fd, &msg);
		break;
	case VHOST_USER_SET_LOG_FD:
		close(msg.fds[0]);
		RTE_LOG(INFO, VHOST_CONFIG, "not implemented.\n");
		break;

	case VHOST_USER_SET_VRING_NUM:
		vhost_set_vring_num(vid, &msg.payload.state);
		break;
	case VHOST_USER_SET_VRING_ADDR:
		vhost_set_vring_addr(vid, &msg.payload.addr);
		break;
	case VHOST_USER_SET_VRING_BASE:
		vhost_set_vring_base(vid, &msg.payload.state);
		break;

	case VHOST_USER_GET_VRING_BASE:
		ret = user_get_vring_base(vid, &msg.payload.state);
		msg.size = sizeof(msg.payload.state);
		send_vhost_message(fd, &msg);
		break;

	case VHOST_USER_SET_VRING_KICK:
		user_set_vring_kick(vid, &msg);
		break;
	case VHOST_USER_SET_VRING_CALL:
		user_set_vring_call(vid, &msg);
		break;

	case VHOST_USER_SET_VRING_ERR:
		if (!(msg.payload.u64 & VHOST_USER_VRING_NOFD_MASK))
			close(msg.fds[0]);
		RTE_LOG(INFO, VHOST_CONFIG, "not implemented\n");
		break;

	case VHOST_USER_GET_QUEUE_NUM:
		msg.payload.u64 = VHOST_MAX_QUEUE_PAIRS;
		msg.size = sizeof(msg.payload.u64);
		send_vhost_message(fd, &msg);
		break;

	case VHOST_USER_SET_VRING_ENABLE:
		user_set_vring_enable(vid, &msg.payload.state);
		break;
	case VHOST_USER_SEND_RARP:
		user_send_rarp(vid, &msg);
		break;

	default:
		break;

	}

	return 0;
}
