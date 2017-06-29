/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2017 Intel Corporation. All rights reserved.
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

#ifndef _VHOST_SCSI_H_
#define _VHOST_SCSI_H_

#include <sys/uio.h>
#include <stdint.h>
#include <linux/virtio_scsi.h>
#include <linux/virtio_ring.h>

#include <rte_vhost.h>

static inline uint16_t
from_be16(const void *ptr)
{
	const uint8_t *tmp = (const uint8_t *)ptr;

	return (((uint16_t)tmp[0] << 8) | tmp[1]);
}

static inline void
to_be16(void *out, uint16_t in)
{
	uint8_t *tmp = (uint8_t *)out;

	tmp[0] = (in >> 8) & 0xFF;
	tmp[1] = in & 0xFF;
}

static inline uint32_t
from_be32(const void *ptr)
{
	const uint8_t *tmp = (const uint8_t *)ptr;

	return (((uint32_t)tmp[0] << 24) |
		((uint32_t)tmp[1] << 16) |
		((uint32_t)tmp[2] << 8) |
		((uint32_t)tmp[3]));
}

static inline void
to_be32(void *out, uint32_t in)
{
	uint8_t *tmp = (uint8_t *)out;

	tmp[0] = (in >> 24) & 0xFF;
	tmp[1] = (in >> 16) & 0xFF;
	tmp[2] = (in >> 8) & 0xFF;
	tmp[3] = in & 0xFF;
}

static inline uint64_t
from_be64(const void *ptr)
{
	const uint8_t *tmp = (const uint8_t *)ptr;

	return (((uint64_t)tmp[0] << 56) |
		((uint64_t)tmp[1] << 48) |
		((uint64_t)tmp[2] << 40) |
		((uint64_t)tmp[3] << 32) |
		((uint64_t)tmp[4] << 24) |
		((uint64_t)tmp[5] << 16) |
		((uint64_t)tmp[6] << 8) |
		((uint64_t)tmp[7]));
}

static inline void
to_be64(void *out, uint64_t in)
{
	uint8_t *tmp = (uint8_t *)out;

	tmp[0] = (in >> 56) & 0xFF;
	tmp[1] = (in >> 48) & 0xFF;
	tmp[2] = (in >> 40) & 0xFF;
	tmp[3] = (in >> 32) & 0xFF;
	tmp[4] = (in >> 24) & 0xFF;
	tmp[5] = (in >> 16) & 0xFF;
	tmp[6] = (in >> 8) & 0xFF;
	tmp[7] = in & 0xFF;
}

static inline uint16_t
from_le16(const void *ptr)
{
	const uint8_t *tmp = (const uint8_t *)ptr;

	return (((uint16_t)tmp[1] << 8) | tmp[0]);
}

static inline void
to_le16(void *out, uint16_t in)
{
	uint8_t *tmp = (uint8_t *)out;

	tmp[1] = (in >> 8) & 0xFF;
	tmp[0] = in & 0xFF;
}

static inline uint32_t
from_le32(const void *ptr)
{
	const uint8_t *tmp = (const uint8_t *)ptr;

	return (((uint32_t)tmp[3] << 24) |
		((uint32_t)tmp[2] << 16) |
		((uint32_t)tmp[1] << 8) |
		((uint32_t)tmp[0]));
}

static inline void
to_le32(void *out, uint32_t in)
{
	uint8_t *tmp = (uint8_t *)out;

	tmp[3] = (in >> 24) & 0xFF;
	tmp[2] = (in >> 16) & 0xFF;
	tmp[1] = (in >> 8) & 0xFF;
	tmp[0] = in & 0xFF;
}

static inline uint64_t
from_le64(const void *ptr)
{
	const uint8_t *tmp = (const uint8_t *)ptr;

	return (((uint64_t)tmp[7] << 56) |
		((uint64_t)tmp[6] << 48) |
		((uint64_t)tmp[5] << 40) |
		((uint64_t)tmp[4] << 32) |
		((uint64_t)tmp[3] << 24) |
		((uint64_t)tmp[2] << 16) |
		((uint64_t)tmp[1] << 8) |
		((uint64_t)tmp[0]));
}

static inline void
to_le64(void *out, uint64_t in)
{
	uint8_t *tmp = (uint8_t *)out;

	tmp[7] = (in >> 56) & 0xFF;
	tmp[6] = (in >> 48) & 0xFF;
	tmp[5] = (in >> 40) & 0xFF;
	tmp[4] = (in >> 32) & 0xFF;
	tmp[3] = (in >> 24) & 0xFF;
	tmp[2] = (in >> 16) & 0xFF;
	tmp[1] = (in >> 8) & 0xFF;
	tmp[0] = in & 0xFF;
}

struct vaddr_region {
	void *vaddr;
	uint64_t len;
};

struct vhost_scsi_queue {
	struct rte_vhost_vring vq;
	uint16_t last_avail_idx;
	uint16_t last_used_idx;
};

struct vhost_block_dev {
	/** ID for vhost library. */
	int vid;
	/** Queues for the block device */
	struct vhost_scsi_queue queues[8];
	/** Unique name for this block device. */
	char name[64];

	/** Unique product name for this kind of block device. */
	char product_name[256];

	/** Size in bytes of a logical block for the backend */
	uint32_t blocklen;

	/** Number of blocks */
	uint64_t blockcnt;

	/** write cache enabled, not used at the moment */
	int write_cache;

	/** use memory as disk storage space */
	uint8_t *data;
};

struct vhost_scsi_ctrlr {
	char *name;
	/** Only support 1 LUN for the example */
	struct vhost_block_dev *bdev;
	/** VM memory region */
	struct rte_vhost_memory *mem;
} __rte_cache_aligned;

#define VHOST_SCSI_MAX_IOVS 128

enum scsi_data_dir {
	SCSI_DIR_NONE = 0,
	SCSI_DIR_TO_DEV = 1,
	SCSI_DIR_FROM_DEV = 2,
};

struct vhost_scsi_task {
	int req_idx;
	uint32_t dxfer_dir;
	uint32_t data_len;
	struct virtio_scsi_cmd_req *req;
	struct virtio_scsi_cmd_resp *resp;
	struct iovec iovs[VHOST_SCSI_MAX_IOVS];
	uint32_t iovs_cnt;
	struct vring_desc *desc;
	struct rte_vhost_vring *vq;
	struct vhost_block_dev *bdev;
	struct vhost_scsi_ctrlr *ctrlr;
};

int vhost_bdev_process_scsi_commands(struct vhost_block_dev *bdev,
				     struct vhost_scsi_task *task);

#endif /* _VHOST_SCSI_H_ */
