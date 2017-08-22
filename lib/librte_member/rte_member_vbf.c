/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
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

#include <math.h>
#include <string.h>

#include <rte_malloc.h>
#include <rte_memory.h>
#include <rte_errno.h>
#include <rte_log.h>

#include "rte_member.h"
#include "rte_member_vbf.h"

/*
 * vBF currently implemented as a big array.
 * The BFs have a vertical layout. Bits in same location of all bfs will stay
 * in the same cache line.
 * For example, if we have 32 bloom filters, we use a uint32_t array to
 * represent all of them. array[0] represent the first location of all the
 * bloom filters, array[1] represents the second location of all the
 * bloom filters, etc. The advantage of this layout is to minimize the average
 * number of memory accesses to test all bloom filters.
 *
 * Currently the implementation supports vBF containing 1,2,4,8,16,32 BFs.
 */

int
rte_member_create_vbf(struct rte_member_setsum *ss,
		const struct rte_member_parameters *params)
{

	if (params->num_set > 32 || !rte_is_power_of_2(params->num_set)) {
		rte_errno = EINVAL;
		RTE_LOG(ERR, MEMBER, "vBF create with invalid parameters\n");
	}

	/* We assume expected keys evenly distribute to all BFs */
	uint32_t num_keys_per_bf = (params->num_keys + ss->num_set - 1) /
					ss->num_set;


	/*
	 * Note that the false positive rate is for all BFs in the vBF
	 * such that the single BF's false positive rate need to be
	 * calculated.
	 * Assume each BF's False positive rate is x. The total false positive
	 * rate is fp = 1-(1-x)^n.
	 * => x = 1 - (1-fp)^(1/n)
	 */

	float x = 1 - pow((1 - params->false_positive_rate), 1.0 / ss->num_set);

	uint32_t bits = ceil((num_keys_per_bf *
				log(x)) / log(1.0 / (pow(2.0, log(2.0)))));

	/* We round to power of 2 for performance during lookup */
	ss->bits = rte_align32pow2(bits);

	ss->num_hashes = (uint32_t)(log(2.0) * bits / num_keys_per_bf);
	ss->bit_mask = ss->bits - 1;

	ss->table = rte_zmalloc_socket(NULL, sizeof(uint32_t) * ss->num_set *
			(ss->bits >> 5), RTE_CACHE_LINE_SIZE, ss->socket_id);


	/*
	 * Since we round the bits to power of 2, the final false positive
	 * rate will probably not be same as the user specified. We log the
	 * new value as debug message.
	 */
	float new_fp = pow((1 - pow((1 - 1.0 / ss->bits), num_keys_per_bf *
					ss->num_hashes)), ss->num_hashes);
	new_fp = 1 - pow((1 - new_fp), ss->num_set);

	RTE_LOG(DEBUG, MEMBER, "vector bloom filter created, "
		"each bloom filter expects %u keys, needs %u bits, %u hashes, "
		"with false positive rate set as %.5f, "
		"The new calculated vBF false positive rate is %.5f\n",
		num_keys_per_bf, ss->bits, ss->num_hashes, x, new_fp);

	if (ss->table == NULL)
		return -ENOMEM;

	return 0;
}


/*
 * a is how many bits could be represented by one uint32_t variable.
 * b is used for division shift
 * shift is used for multiplication shift
 */
static inline uint32_t
test_bit(uint32_t h1, uint32_t h2, uint32_t iter, uint32_t shift, uint32_t a,
		uint32_t b, const struct rte_member_setsum *ss)
{
	uint32_t *vbf = ss->table;
	uint32_t n = ss->num_set;
	uint32_t bit_loc = (h1 + iter * h2) & ss->bit_mask;
	/*
	 * x>>b is the divide, x & (a-1) is the mod, & (1<<n-1) to mask out bits
	 * we do not need
	 */
	return (vbf[bit_loc>>b] >> ((bit_loc & (a - 1)) << shift)) &
							((1ULL << n) - 1);
}


static inline void
set_bit(uint32_t h1, uint32_t h2, uint32_t iter, uint32_t shift, uint32_t a,
		uint32_t b, const struct rte_member_setsum *ss, int32_t set)
{
	uint32_t *vbf = ss->table;
	uint32_t bit_loc = (h1 + iter * h2) & ss->bit_mask;
	vbf[bit_loc>>b] |= 1U << (((bit_loc & (a - 1)) << shift) + set - 1);
}



int
rte_member_lookup_vbf(const struct rte_member_setsum *ss, const void *key,
		MEMBER_SET_TYPE *set_id)
{
	uint32_t j;
	uint32_t h1 = MEMBER_PRIM_HASH(key, ss->key_len, ss->prim_hash_seed);
	uint32_t h2 = MEMBER_SEC_HASH(&h1, 4, ss->sec_hash_seed);
	uint32_t mask = 0xFFFFFFFF;
	uint32_t shift = __builtin_ctz(ss->num_set);
	uint32_t a = 32 >> shift;
	uint32_t b = __builtin_ctz(a);

	for (j = 0; j < ss->num_hashes; j++)
		mask &= test_bit(h1, h2, j, shift, a, b, ss);

	if (mask) {
		*set_id = __builtin_ctz(mask) + 1;
		return 1;
	}

	*set_id = RTE_MEMBER_NO_MATCH;
	return 0;
}



uint32_t
rte_member_lookup_bulk_vbf(const struct rte_member_setsum *ss,
		const void **keys, uint32_t num_keys, MEMBER_SET_TYPE *set_ids)
{
	uint32_t i, k, h1, h2;
	uint32_t ret = 0;
	uint32_t mask;
	uint32_t shift = __builtin_ctz(ss->num_set);
	uint32_t a = 32 >> shift;
	uint32_t b = __builtin_ctz(a);

	for (i = 0; i < num_keys; i++) {
		mask = 0xFFFFFFFF;
		h1 = MEMBER_PRIM_HASH(keys[i], ss->key_len,
					ss->prim_hash_seed);
		h2 = MEMBER_SEC_HASH(&h1, 4, ss->sec_hash_seed);
		for (k = 0; k < ss->num_hashes; k++)
			mask &= test_bit(h1, h2, k, shift, a, b, ss);

		if (mask) {
			set_ids[i] = __builtin_ctz(mask) + 1;
			ret++;
		} else
			set_ids[i] = RTE_MEMBER_NO_MATCH;
	}
	return ret;
}



uint32_t
rte_member_lookup_multi_vbf(const struct rte_member_setsum *ss,
		const void *key, uint32_t match_per_key,
		MEMBER_SET_TYPE *set_id)
{
	uint32_t ret = 0;
	uint32_t j;
	uint32_t h1 = MEMBER_PRIM_HASH(key, ss->key_len, ss->prim_hash_seed);
	uint32_t h2 = MEMBER_SEC_HASH(&h1, 4, ss->sec_hash_seed);
	uint32_t mask = 0xFFFFFFFF;
	uint32_t shift = __builtin_ctz(ss->num_set);
	uint32_t a = 32 >> shift;
	uint32_t b = __builtin_ctz(a);

	for (j = 0; j < ss->num_hashes; j++)
		mask &= test_bit(h1, h2, j, shift, a, b, ss);

	while (mask) {
		uint32_t loc = __builtin_ctz(mask);
		set_id[ret] = loc + 1;
		ret++;
		if (ret >= match_per_key)
			return ret;
		mask &= ~(1U << loc);
	}
	return ret;
}



uint32_t
rte_member_lookup_multi_bulk_vbf(const struct rte_member_setsum *ss,
		const void **keys, uint32_t num_keys, uint32_t match_per_key,
		uint32_t *match_count,
		MEMBER_SET_TYPE *set_ids)
{
	uint32_t i, k, h1, h2;
	uint32_t ret = 0;
	uint32_t match_cnt_t;
	uint32_t mask;
	uint32_t shift = __builtin_ctz(ss->num_set);
	uint32_t a = 32 >> shift;
	uint32_t b = __builtin_ctz(a);

	for (i = 0; i < num_keys; i++) {
		match_cnt_t = 0;
		mask = 0xFFFFFFFF;
		h1 = MEMBER_PRIM_HASH(keys[i], ss->key_len,
					ss->prim_hash_seed);
		h2 = MEMBER_SEC_HASH(&h1, 4, ss->sec_hash_seed);

		for (k = 0; k < ss->num_hashes; k++)
			mask &= test_bit(h1, h2, k, shift, a, b, ss);

		while (mask) {
			uint32_t loc = __builtin_ctz(mask);
			if (match_cnt_t >= match_per_key)
				break;
			set_ids[i * match_per_key + match_cnt_t] = loc + 1;
			match_cnt_t++;
			mask &= ~(1U << loc);
		}
		match_count[i] = match_cnt_t;
		if (match_cnt_t != 0)
			ret++;
	}
	return ret;
}


int
rte_member_add_vbf(const struct rte_member_setsum *ss,
		const void *key, MEMBER_SET_TYPE set_id)
{
	uint32_t i, h1, h2;

	if (set_id > ss->num_set || set_id == RTE_MEMBER_NO_MATCH)
		return -EINVAL;

	h1 = MEMBER_PRIM_HASH(key, ss->key_len, ss->prim_hash_seed);
	h2 = MEMBER_SEC_HASH(&h1, 4, ss->sec_hash_seed);
	uint32_t shift = __builtin_ctz(ss->num_set);
	uint32_t a = 32 >> shift;
	uint32_t b = __builtin_ctz(a);

	for (i = 0; i < ss->num_hashes; i++)
		set_bit(h1, h2, i, shift, a, b, ss, set_id);
	return 0;
}

void
rte_member_free_vbf(struct rte_member_setsum *ss)
{
	rte_free(ss->table);
}


void
rte_member_reset_vbf(const struct rte_member_setsum *ss)
{
	uint32_t *vbf = ss->table;
	memset(vbf, 0, (ss->num_set * ss->bits) >> 3);
}
