/*-
 *   BSD LICENSE
 *
 *   Copyright (c) 2017 Red Hat, Inc.
 *   Copyright (c) 2017 Maxime Coquelin <maxime.coquelin@redhat.com>
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

#ifdef RTE_LIBRTE_VHOST_NUMA
#include <numaif.h>
#endif

#include <rte_tailq.h>

#include "iotlb.h"
#include "vhost.h"

struct vhost_iotlb_entry {
	TAILQ_ENTRY(vhost_iotlb_entry) next;

	uint64_t iova;
	uint64_t uaddr;
	uint64_t size;
	uint8_t perm;
};

/* ToDo: refine cache size */
#define IOTLB_CACHE_SIZE 1024

/* ToDo: Coalesce contiguous entries? */
void vhost_user_iotlb_insert(struct vhost_virtqueue *vq, uint64_t iova,
				uint64_t uaddr, uint64_t size, uint8_t perm)
{
	struct vhost_iotlb_entry *node, *new_node;
	int ret;

	ret = rte_mempool_get(vq->iotlb_pool, (void **)&new_node);
	if (ret) {
		RTE_LOG(ERR, VHOST_CONFIG, "IOTLB pool empty, invalidate cache\n");
		vhost_user_iotlb_remove_all(vq);
		ret = rte_mempool_get(vq->iotlb_pool, (void **)&new_node);
		if (ret) {
			RTE_LOG(ERR, VHOST_CONFIG, "IOTLB pool still empty, failure\n");
			return;
		}
	}

	new_node->iova = iova;
	new_node->uaddr = uaddr;
	new_node->size = size;
	new_node->perm = perm;

	rte_rwlock_write_lock(&vq->iotlb_lock);

	TAILQ_FOREACH(node, &vq->iotlb_list, next) {
		/*
		 * IIUC, entries must be invalidated before being updated.
		 * So if iova already in list, assume identical.
		 */
		if (node->iova == new_node->iova) {
			rte_mempool_put(vq->iotlb_pool, new_node);
			goto unlock;
		} else if (node->iova > new_node->iova) {
			TAILQ_INSERT_BEFORE(node, new_node, next);
			goto unlock;
		}
	}

	TAILQ_INSERT_TAIL(&vq->iotlb_list, new_node, next);

unlock:
	rte_rwlock_write_unlock(&vq->iotlb_lock);
}

void vhost_user_iotlb_remove(struct vhost_virtqueue *vq,
					uint64_t iova, uint64_t size)
{
	struct vhost_iotlb_entry *node, *temp_node;

	if (unlikely(!size))
		return;

	rte_rwlock_write_lock(&vq->iotlb_lock);

	TAILQ_FOREACH_SAFE(node, &vq->iotlb_list, next, temp_node) {
		/* Sorted list */
		if (unlikely(node->iova >= iova + size)) {
			break;
		} else if ((node->iova < iova + size) &&
					(iova < node->iova + node->size)) {
			TAILQ_REMOVE(&vq->iotlb_list, node, next);
			rte_mempool_put(vq->iotlb_pool, node);
			continue;
		}
	}

	rte_rwlock_write_unlock(&vq->iotlb_lock);
}

void vhost_user_iotlb_remove_all(struct vhost_virtqueue *vq)
{
	struct vhost_iotlb_entry *node, *temp_node;

	rte_rwlock_write_lock(&vq->iotlb_lock);

	TAILQ_FOREACH_SAFE(node, &vq->iotlb_list, next, temp_node) {
		TAILQ_REMOVE(&vq->iotlb_list, node, next);
		rte_mempool_put(vq->iotlb_pool, node);
	}

	rte_rwlock_write_unlock(&vq->iotlb_lock);
}

uint64_t vhost_user_iotlb_find(struct vhost_virtqueue *vq, uint64_t iova,
						uint64_t *size, uint8_t perm)
{
	struct vhost_iotlb_entry *node;
	uint64_t offset, vva = 0, mapped = 0;

	if (unlikely(!*size))
		goto out;

	rte_rwlock_read_lock(&vq->iotlb_lock);

	TAILQ_FOREACH(node, &vq->iotlb_list, next) {
		/* List sorted by iova */
		if (unlikely(iova < node->iova))
			break;

		if (iova >= node->iova + node->size)
			continue;

		if (unlikely((perm & node->perm) != perm)) {
			vva = 0;
			break;
		}

		offset = iova - node->iova;
		if (!vva)
			vva = node->uaddr + offset;

		mapped += node->size - offset;
		iova = node->iova + node->size;

		if (mapped >= *size)
			break;
	}

	rte_rwlock_read_unlock(&vq->iotlb_lock);

out:
	if (mapped < *size)
		*size = mapped;

	/* Only part of the requested chunk is mapped */
	if (unlikely(mapped < *size))
		*size = mapped;

	return vva;
}

int vhost_user_iotlb_init(struct virtio_net *dev, int vq_index)
{
	char pool_name[RTE_MEMPOOL_NAMESIZE];
	struct vhost_virtqueue *vq = dev->virtqueue[vq_index];
	int ret = -1, socket;

	if (vq->iotlb_pool) {
		/*
		 * The cache has already been initialized,
		 * just drop all entries
		 */
		vhost_user_iotlb_remove_all(vq);
		return 0;
	}

#ifdef RTE_LIBRTE_VHOST_NUMA
	ret = get_mempolicy(&socket, NULL, 0, vq, MPOL_F_NODE | MPOL_F_ADDR);
#endif
	if (ret)
		socket = 0;

	rte_rwlock_init(&vq->iotlb_lock);

	TAILQ_INIT(&vq->iotlb_list);

	snprintf(pool_name, sizeof(pool_name), "iotlb_cache_%d_%d",
			dev->vid, vq_index);

	/* If already created, free it and recreate */
	vq->iotlb_pool = rte_mempool_lookup(pool_name);
	if (vq->iotlb_pool)
		rte_mempool_free(vq->iotlb_pool);

	vq->iotlb_pool = rte_mempool_create(pool_name,
			IOTLB_CACHE_SIZE, sizeof(struct vhost_iotlb_entry), 0,
			0, 0, NULL, NULL, NULL, socket,
			MEMPOOL_F_NO_CACHE_ALIGN |
			MEMPOOL_F_SP_PUT |
			MEMPOOL_F_SC_GET);
	if (!vq->iotlb_pool) {
		RTE_LOG(ERR, VHOST_CONFIG,
				"Failed to create IOTLB cache pool (%s)\n",
				pool_name);
		return -1;
	}

	return 0;
}

