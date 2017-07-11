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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <rte_memory.h>
#include <rte_branch_prediction.h>

#include <rte_rib.h>
#include <rte_dir24_8.h>

#define ROUNDUP(x, y)   ((((x - 1) >> (32 - y)) + 1) << (32 - y))
#define RTE_DIR24_8_GET_TBL24_P(fib, ip)			\
	((void *)&((uint8_t *)fib->tbl24)[(ip &			\
		RTE_DIR24_8_TBL24_MASK) >> (8 - fib->nh_sz)])	\

int
rte_dir24_8_lookup_1b(void *fib_p, uint32_t ip, uint64_t *next_hop)
{
	struct rte_dir24_8_tbl *fib = (struct rte_dir24_8_tbl *)fib_p;
	uint32_t tbl24_index = (ip >> 8);

	RTE_RIB_RETURN_IF_TRUE(((fib_p == NULL) ||
		(next_hop == NULL)), -EINVAL);

	RTE_DIR24_8_LOOKUP(uint8_t);

	return (tbl_entry & RTE_DIR24_8_LOOKUP_SUCCESS) ? 0 : -ENOENT;
}

int
rte_dir24_8_lookup_2b(void *fib_p, uint32_t ip, uint64_t *next_hop)
{
	struct rte_dir24_8_tbl *fib = (struct rte_dir24_8_tbl *)fib_p;
	uint32_t tbl24_index = (ip >> 8);

	RTE_RIB_RETURN_IF_TRUE(((fib_p == NULL) ||
		(next_hop == NULL)), -EINVAL);

	RTE_DIR24_8_LOOKUP(uint16_t);

	return (tbl_entry & RTE_DIR24_8_LOOKUP_SUCCESS) ? 0 : -ENOENT;
}

int
rte_dir24_8_lookup_4b(void *fib_p, uint32_t ip, uint64_t *next_hop)
{
	struct rte_dir24_8_tbl *fib = (struct rte_dir24_8_tbl *)fib_p;
	uint32_t tbl24_index = (ip >> 8);

	RTE_RIB_RETURN_IF_TRUE(((fib_p == NULL) ||
		(next_hop == NULL)), -EINVAL);

	RTE_DIR24_8_LOOKUP(uint32_t);

	return (tbl_entry & RTE_DIR24_8_LOOKUP_SUCCESS) ? 0 : -ENOENT;
}

int
rte_dir24_8_lookup_8b(void *fib_p, uint32_t ip, uint64_t *next_hop)
{
	struct rte_dir24_8_tbl *fib = (struct rte_dir24_8_tbl *)fib_p;
	uint32_t tbl24_index = (ip >> 8);

	RTE_RIB_RETURN_IF_TRUE(((fib_p == NULL) ||
		(next_hop == NULL)), -EINVAL);

	RTE_DIR24_8_LOOKUP(uint64_t);

	return (tbl_entry & RTE_DIR24_8_LOOKUP_SUCCESS) ? 0 : -ENOENT;
}

static int
tbl8_alloc(struct rte_dir24_8_tbl *fib)
{
	uint32_t i;
	uint8_t *ptr8 = (uint8_t *)fib->tbl8;
	uint16_t *ptr16 = (uint16_t *)fib->tbl8;
	uint32_t *ptr32 = (uint32_t *)fib->tbl8;
	uint64_t *ptr64 = (uint64_t *)fib->tbl8;

	if (fib->cur_tbl8s >= fib->number_tbl8s)
		return -ENOSPC;

	switch (fib->nh_sz) {
	case RTE_DIR24_8_1B:
		for (i = 0; i < fib->number_tbl8s; i++) {
			if ((ptr8[i * RTE_DIR24_8_TBL8_GRP_NUM_ENT] &
					RTE_DIR24_8_VALID_EXT_ENT) !=
					RTE_DIR24_8_VALID_EXT_ENT) {
				fib->cur_tbl8s++;
				return i;
			}
		}
		break;
	case RTE_DIR24_8_2B:
		for (i = 0; i < fib->number_tbl8s; i++) {
			if ((ptr16[i * RTE_DIR24_8_TBL8_GRP_NUM_ENT] &
					RTE_DIR24_8_VALID_EXT_ENT) !=
					RTE_DIR24_8_VALID_EXT_ENT) {
				fib->cur_tbl8s++;
				return i;
			}
		}
	case RTE_DIR24_8_4B:
		for (i = 0; i < fib->number_tbl8s; i++) {
			if ((ptr32[i * RTE_DIR24_8_TBL8_GRP_NUM_ENT] &
					RTE_DIR24_8_VALID_EXT_ENT) !=
					RTE_DIR24_8_VALID_EXT_ENT) {
				fib->cur_tbl8s++;
				return i;
			}
		}
	case RTE_DIR24_8_8B:
		for (i = 0; i < fib->number_tbl8s; i++) {
			if ((ptr64[i * RTE_DIR24_8_TBL8_GRP_NUM_ENT] &
					RTE_DIR24_8_VALID_EXT_ENT) !=
					RTE_DIR24_8_VALID_EXT_ENT) {
				fib->cur_tbl8s++;
				return i;
			}
		}
	}
	return -ENOSPC;
}

static void
write_to_fib(void *ptr, uint64_t val, enum rte_dir24_8_nh_sz size, int n)
{
	int i;
	uint8_t *ptr8 = (uint8_t *)ptr;
	uint16_t *ptr16 = (uint16_t *)ptr;
	uint32_t *ptr32 = (uint32_t *)ptr;
	uint64_t *ptr64 = (uint64_t *)ptr;

	switch (size) {
	case RTE_DIR24_8_1B:
		for (i = 0; i < n; i++)
			ptr8[i] = (uint8_t)val;
		break;
	case RTE_DIR24_8_2B:
		for (i = 0; i < n; i++)
			ptr16[i] = (uint16_t)val;
		break;
	case RTE_DIR24_8_4B:
		for (i = 0; i < n; i++)
			ptr32[i] = (uint32_t)val;
		break;
	case RTE_DIR24_8_8B:
		for (i = 0; i < n; i++)
			ptr64[i] = (uint64_t)val;
		break;
	}
}

static int
install_to_fib(struct rte_dir24_8_tbl *fib, uint32_t ledge, uint32_t redge,
	uint64_t next_hop, int valid)
{
	int tbl8_idx;
	uint64_t tbl24_tmp;
	uint8_t *tbl8_ptr;

	if (ROUNDUP(ledge, 24) <= redge) {
		if (ledge < ROUNDUP(ledge, 24)) {
			tbl24_tmp = RTE_DIR24_8_GET_TBL24(fib, ledge);
			if ((tbl24_tmp & RTE_DIR24_8_VALID_EXT_ENT) !=
					RTE_DIR24_8_VALID_EXT_ENT) {
				tbl8_idx = tbl8_alloc(fib);
				if (tbl8_idx < 0)
					return tbl8_idx;
				tbl8_ptr = (uint8_t *)fib->tbl8 +
					((tbl8_idx * RTE_DIR24_8_TBL8_GRP_NUM_ENT) <<
					fib->nh_sz);
				/*Init tbl8 entries with nexthop from tbl24*/
				write_to_fib((void *)tbl8_ptr, tbl24_tmp|
					RTE_DIR24_8_VALID_EXT_ENT, fib->nh_sz,
					RTE_DIR24_8_TBL8_GRP_NUM_ENT);
				/*update dir24 entry with tbl8 index*/
				write_to_fib(RTE_DIR24_8_GET_TBL24_P(fib, ledge),
					(tbl8_idx << 2)|
					RTE_DIR24_8_VALID_EXT_ENT|
					valid, fib->nh_sz, 1);
			} else
				tbl8_idx = tbl24_tmp >> 2;
			tbl8_ptr = (uint8_t *)fib->tbl8 +
				(((tbl8_idx * RTE_DIR24_8_TBL8_GRP_NUM_ENT) +
				(ledge & ~RTE_DIR24_8_TBL24_MASK)) <<
				fib->nh_sz);
			/*update tbl8 with new next hop*/
			write_to_fib((void *)tbl8_ptr, (next_hop << 2)|
				RTE_DIR24_8_VALID_EXT_ENT|valid,
				fib->nh_sz, ROUNDUP(ledge, 24) - ledge);
		}
		if (ROUNDUP(ledge, 24) < (redge & RTE_DIR24_8_TBL24_MASK)) {
			write_to_fib(RTE_DIR24_8_GET_TBL24_P(fib,
				ROUNDUP(ledge, 24)), (next_hop << 2)|valid,
				fib->nh_sz, ((redge & RTE_DIR24_8_TBL24_MASK) -
				ROUNDUP(ledge, 24)) >> 8);
		}
		if (redge & ~RTE_DIR24_8_TBL24_MASK) {
			tbl24_tmp = RTE_DIR24_8_GET_TBL24(fib, redge);
			if ((tbl24_tmp & RTE_DIR24_8_VALID_EXT_ENT) !=
					RTE_DIR24_8_VALID_EXT_ENT) {
				tbl8_idx = tbl8_alloc(fib);
				if (tbl8_idx < 0)
					return tbl8_idx;
				tbl8_ptr = (uint8_t *)fib->tbl8 +
					((tbl8_idx * RTE_DIR24_8_TBL8_GRP_NUM_ENT) <<
					fib->nh_sz);
				/*Init tbl8 entries with nexthop from tbl24*/
				write_to_fib((void *)tbl8_ptr, tbl24_tmp|
					RTE_DIR24_8_VALID_EXT_ENT, fib->nh_sz,
					RTE_DIR24_8_TBL8_GRP_NUM_ENT);
				/*update dir24 entry with tbl8 index*/
				write_to_fib(RTE_DIR24_8_GET_TBL24_P(fib, redge),
					(tbl8_idx << 2)|
					RTE_DIR24_8_VALID_EXT_ENT|
					valid, fib->nh_sz, 1);
			} else
				tbl8_idx = tbl24_tmp >> 2;
			tbl8_ptr = (uint8_t *)fib->tbl8 +
				((tbl8_idx * RTE_DIR24_8_TBL8_GRP_NUM_ENT) <<
				fib->nh_sz);
			/*update tbl8 with new next hop*/
			write_to_fib((void *)tbl8_ptr, (next_hop << 2)|
				RTE_DIR24_8_VALID_EXT_ENT|valid,
				fib->nh_sz, redge & ~RTE_DIR24_8_TBL24_MASK);
		}
	} else {
		tbl24_tmp = RTE_DIR24_8_GET_TBL24(fib, ledge);
			if ((tbl24_tmp & RTE_DIR24_8_VALID_EXT_ENT) !=
				RTE_DIR24_8_VALID_EXT_ENT) {
				tbl8_idx = tbl8_alloc(fib);
				if (tbl8_idx < 0)
					return tbl8_idx;
				tbl8_ptr = (uint8_t *)fib->tbl8 +
					((tbl8_idx * RTE_DIR24_8_TBL8_GRP_NUM_ENT) <<
					fib->nh_sz);

				/*Init tbl8 entries with nexthop from tbl24*/
				write_to_fib((void *)tbl8_ptr, tbl24_tmp|
					RTE_DIR24_8_VALID_EXT_ENT, fib->nh_sz,
					RTE_DIR24_8_TBL8_GRP_NUM_ENT);
				/*update dir24 entry with tbl8 index*/
				write_to_fib(RTE_DIR24_8_GET_TBL24_P(fib, ledge),
					(tbl8_idx << 2)|
					RTE_DIR24_8_VALID_EXT_ENT|
					valid, fib->nh_sz, 1);
			} else
				tbl8_idx = tbl24_tmp >> 2;
			tbl8_ptr = (uint8_t *)fib->tbl8 +
				(((tbl8_idx * RTE_DIR24_8_TBL8_GRP_NUM_ENT) +
				(ledge & ~RTE_DIR24_8_TBL24_MASK)) <<
				fib->nh_sz);
			/*update tbl8 with new next hop*/
			write_to_fib((void *)tbl8_ptr, (next_hop << 2)|
				RTE_DIR24_8_VALID_EXT_ENT|valid,
				fib->nh_sz, redge - ledge);
	}
	return 0;
}

int
rte_dir24_8_modify(struct rte_rib *rib, uint32_t ip, uint8_t mask_len,
	uint64_t next_hop, enum rte_rib_op rib_op)
{
	int ret, valid = RTE_DIR24_8_LOOKUP_SUCCESS;
	struct rte_rib_v4_node *ent, *tmp;
	struct rte_rib_simple_ext *ext, *tmp_ext;
	struct rte_dir24_8_tbl *fib;
	uint32_t ledge, redge;
	int tbl8_idx;
	uint8_t *tbl8_ptr;

	if ((rib == NULL) || (mask_len > RTE_RIB_V4_MAXDEPTH))
		return -EINVAL;

	fib = rib->fib;
	ip &= (uint32_t)(UINT64_MAX << (32 - mask_len));

	ent = rte_rib_v4_lookup_exact(rib, ip, mask_len);
	if (rib_op == RTE_RIB_OP_ADD) {
		if (ent)
			return -EEXIST;
		ent = rte_rib_v4_insert(rib, ip, mask_len);
	}
	if (ent == NULL)
		return -ENOENT;
	ext = (struct rte_rib_simple_ext *)ent->ext;

	switch (rib_op) {
	case RTE_RIB_OP_DEL:
		tmp = rte_rib_v4_lookup_parent(ent);
		if (tmp == NULL) {
			valid = 0;
			break;
		}
		tmp_ext = (struct rte_rib_simple_ext *)tmp->ext;
		if (ext->next_hop == tmp_ext->next_hop)
			goto delete_finish;
		next_hop = tmp_ext->next_hop;
		break;
	case RTE_RIB_OP_MODIFY:
		if (ext->next_hop == next_hop)
			return 0;
		break;
	case RTE_RIB_OP_ADD:
		ext->next_hop = next_hop;
		break;
	default:
		return -EINVAL;
	}

	tmp = NULL;
	ledge = ip;
	do {
		tmp = rte_rib_v4_get_next_child(rib, ip, mask_len, tmp,
			GET_NXT_COVER);
		if (tmp != NULL) {
			if (tmp->mask_len == mask_len)
				continue;
			redge = tmp->key;
			if (ledge == redge) {
				ledge = redge +
					(uint32_t)(1ULL << (32 - tmp->mask_len));
				continue;
			}
			ret = install_to_fib(rib->fib, ledge, redge,
				next_hop, valid);
			if (ret != 0) {
				if ((ret == -ENOSPC) && (mask_len > 24))
					rte_rib_v4_remove(rib, ip, mask_len);
				return ret;
			}
			ledge = redge +
				(uint32_t)(1ULL << (32 - tmp->mask_len));
		} else {
			redge = ip + (uint32_t)(1ULL << (32 - mask_len));
			ret = install_to_fib(rib->fib, ledge, redge,
				next_hop, valid);
			if (ret != 0) {
				if ((ret == -ENOSPC) && (mask_len > 24))
					rte_rib_v4_remove(rib, ip, mask_len);
				return ret;
			}
		}
	} while (tmp);

	if (rib_op == RTE_RIB_OP_DEL) {
		/*tbl8 recycle check*/
delete_finish:
		rte_rib_v4_remove(rib, ip, mask_len);
		if (mask_len > 24) {
			tmp = rte_rib_v4_get_next_child(rib,
				(ip & RTE_DIR24_8_TBL24_MASK), 24, NULL,
				GET_NXT_ALL);
			if (tmp == NULL) {
				ent = rte_rib_v4_lookup(rib, ip);
				if (ent) {
					ext = (struct rte_rib_simple_ext *)ent->ext;
					next_hop = ext->next_hop << 2|
						RTE_DIR24_8_LOOKUP_SUCCESS;
				} else {
					next_hop = 0;
				}
				tbl8_idx = RTE_DIR24_8_GET_TBL24(fib, ip) >> 2;
				tbl8_ptr = (uint8_t *)fib->tbl8 +
					((tbl8_idx * RTE_DIR24_8_TBL8_GRP_NUM_ENT) <<
					fib->nh_sz);
				/*update tbl24*/
				write_to_fib(RTE_DIR24_8_GET_TBL24_P(fib, ip),
					next_hop, fib->nh_sz, 1);
				/*Cleanup tbl8*/
				write_to_fib((void *)tbl8_ptr, 0, fib->nh_sz,
					RTE_DIR24_8_TBL8_GRP_NUM_ENT);
			}
		}
	}
	return 0;
}
