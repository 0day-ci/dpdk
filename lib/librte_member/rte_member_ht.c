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

#include <rte_errno.h>
#include <rte_malloc.h>
#include <rte_prefetch.h>
#include <rte_random.h>
#include <rte_log.h>

#include "rte_member.h"
#include "rte_member_ht.h"


static inline int
insert_overwrite_search(uint32_t bucket, SIG_TYPE tmp_sig,
		struct member_ht_bucket *buckets,
		MEMBER_SET_TYPE set_id)
{
	int i;
	for (i = 0; i < RTE_MEMBER_BUCKET_ENTRIES; i++) {
		if (buckets[bucket].sigs[i] == tmp_sig) {
			buckets[bucket].sets[i] = set_id;
			return 1;
		}
	}
	return 0;
}




static inline int
search_bucket_single(uint32_t bucket, SIG_TYPE tmp_sig,
		struct member_ht_bucket *buckets,
		MEMBER_SET_TYPE *set_id)
{
	int iter;
	for (iter = 0; iter < RTE_MEMBER_BUCKET_ENTRIES; iter++) {
		if (tmp_sig == buckets[bucket].sigs[iter] &&
				buckets[bucket].sets[iter] !=
				RTE_MEMBER_NO_MATCH) {
			*set_id = buckets[bucket].sets[iter];
			return 1;
		}
	}
	return 0;
}


static inline void
search_bucket_multi(uint32_t bucket, SIG_TYPE tmp_sig,
		struct member_ht_bucket *buckets,
		uint32_t *counter,
		uint32_t match_per_key,
		MEMBER_SET_TYPE *set_id)
{
	int iter;
	for (iter = 0; iter < RTE_MEMBER_BUCKET_ENTRIES; iter++) {
		if (tmp_sig == buckets[bucket].sigs[iter] &&
				buckets[bucket].sets[iter] !=
				RTE_MEMBER_NO_MATCH) {
			set_id[*counter] = buckets[bucket].sets[iter];
			(*counter)++;
			if (*counter >= match_per_key)
				return;
		}
	}
}


int
rte_member_create_ht(struct rte_member_setsum *ss,
		const struct rte_member_parameters *params)
{
	uint32_t i, j;
	uint32_t size_bucket_t;

	const uint32_t num_buckets = rte_align32pow2(params->num_keys) /
			RTE_MEMBER_BUCKET_ENTRIES;

	if ((num_buckets / RTE_MEMBER_BUCKET_ENTRIES >
			RTE_MEMBER_ENTRIES_MAX) ||
			!rte_is_power_of_2(RTE_MEMBER_BUCKET_ENTRIES)) {
		rte_errno = EINVAL;
		RTE_LOG(ERR, MEMBER,
			"Membership HT create with invalid parameters\n");
	}
	size_bucket_t = sizeof(struct member_ht_bucket);

	struct member_ht_bucket *buckets = rte_zmalloc_socket(NULL,
			num_buckets * size_bucket_t,
			RTE_CACHE_LINE_SIZE, ss->socket_id);

	if (buckets == NULL)
		return -ENOMEM;

	ss->table = buckets;
	ss->bucket_cnt = num_buckets;
	ss->bucket_mask = num_buckets - 1;
	ss->cache = params->iscache;

	for (i = 0; i < num_buckets; i++) {
		for (j = 0; j < RTE_MEMBER_BUCKET_ENTRIES; j++)
			buckets[i].sets[j] = RTE_MEMBER_NO_MATCH;
	}


	RTE_LOG(DEBUG, MEMBER, "Hash table based filter created, "
		"the table has %u entries, %u buckets\n",
		num_buckets,
		num_buckets / RTE_MEMBER_BUCKET_ENTRIES);
	return 0;
}

static inline
void get_buckets_index(const struct rte_member_setsum *ss, const void *key,
		uint32_t *prim_bkt, uint32_t *sec_bkt, SIG_TYPE *sig)
{
	uint32_t first_hash = MEMBER_PRIM_HASH(key, ss->key_len,
						ss->prim_hash_seed);
	uint32_t sec_hash = MEMBER_SEC_HASH(&first_hash, 4, ss->sec_hash_seed);
	*sig = first_hash & SIG_BITMASK;
	if (ss->cache) {
		*prim_bkt = sec_hash & ss->bucket_mask;
		*sec_bkt =  (sec_hash >> 16) & ss->bucket_mask;
	} else {
		*prim_bkt = sec_hash & ss->bucket_mask;
		*sec_bkt =  (*prim_bkt ^ *sig) & ss->bucket_mask;
	}
}


int
rte_member_lookup_ht(const struct rte_member_setsum *ss,
		const void *key, MEMBER_SET_TYPE *set_id)
{
	uint32_t prim_bucket, sec_bucket;
	SIG_TYPE tmp_sig;
	struct member_ht_bucket *buckets = ss->table;


	*set_id = RTE_MEMBER_NO_MATCH;
	get_buckets_index(ss, key, &prim_bucket, &sec_bucket, &tmp_sig);

	if (search_bucket_single(prim_bucket, tmp_sig, buckets,
			set_id) ||
			search_bucket_single(sec_bucket, tmp_sig,
				buckets, set_id))
		return 1;

	return 0;
}


uint32_t
rte_member_lookup_bulk_ht(const struct rte_member_setsum *ss,
		const void **keys, uint32_t num_keys, MEMBER_SET_TYPE *set_id)
{
	uint32_t i;
	uint32_t ret = 0;
	struct member_ht_bucket *buckets = ss->table;
	SIG_TYPE tmp_sig[RTE_MEMBER_LOOKUP_BULK_MAX] = {0};
	uint32_t prim_buckets[RTE_MEMBER_LOOKUP_BULK_MAX] = {0};
	uint32_t sec_buckets[RTE_MEMBER_LOOKUP_BULK_MAX] = {0};

	for (i = 0; i < num_keys; i++) {
		get_buckets_index(ss, keys[i], &prim_buckets[i],
				&sec_buckets[i], &tmp_sig[i]);
		rte_prefetch0(&buckets[prim_buckets[i]]);
		rte_prefetch0(&buckets[sec_buckets[i]]);
	}

	for (i = 0; i < num_keys; i++) {
		if (search_bucket_single(prim_buckets[i], tmp_sig[i],
				buckets, &set_id[i]) ||
				search_bucket_single(sec_buckets[i],
				tmp_sig[i], buckets, &set_id[i]))
			ret++;
		else
			set_id[i] = RTE_MEMBER_NO_MATCH;
	}
	return ret;
}


uint32_t
rte_member_lookup_multi_ht(const struct rte_member_setsum *ss,
		const void *key, uint32_t match_per_key,
		MEMBER_SET_TYPE *set_id)
{
	uint32_t ret = 0;
	uint32_t prim_bucket, sec_bucket;
	SIG_TYPE tmp_sig;
	struct member_ht_bucket *buckets = ss->table;

	get_buckets_index(ss, key, &prim_bucket, &sec_bucket, &tmp_sig);

	search_bucket_multi(prim_bucket, tmp_sig, buckets, &ret,
			 match_per_key, set_id);
	if (ret < match_per_key)
		search_bucket_multi(sec_bucket, tmp_sig,
			buckets, &ret, match_per_key, set_id);
	return ret;
}


uint32_t
rte_member_lookup_multi_bulk_ht(const struct rte_member_setsum *ss,
		const void **keys, uint32_t num_keys, uint32_t match_per_key,
		uint32_t *match_count,
		MEMBER_SET_TYPE *set_ids)
{
	uint32_t i;
	uint32_t ret = 0;
	struct member_ht_bucket *buckets = ss->table;
	uint32_t match_cnt_t;
	SIG_TYPE tmp_sig[RTE_MEMBER_LOOKUP_BULK_MAX] = {0};
	uint32_t prim_buckets[RTE_MEMBER_LOOKUP_BULK_MAX] = {0};
	uint32_t sec_buckets[RTE_MEMBER_LOOKUP_BULK_MAX] = {0};

	for (i = 0; i < num_keys; i++) {
		get_buckets_index(ss, keys[i], &prim_buckets[i],
				&sec_buckets[i], &tmp_sig[i]);
		rte_prefetch0(&buckets[prim_buckets[i]]);
		rte_prefetch0(&buckets[sec_buckets[i]]);
	}
	for (i = 0; i < num_keys; i++) {
		match_cnt_t = 0;

		search_bucket_multi(prim_buckets[i], tmp_sig[i],
			buckets, &match_cnt_t, match_per_key,
			&set_ids[i*match_per_key]);
		if (match_cnt_t < match_per_key)
			search_bucket_multi(sec_buckets[i], tmp_sig[i],
				buckets, &match_cnt_t, match_per_key,
				&set_ids[i*match_per_key]);
		match_count[i] = match_cnt_t;
		if (match_cnt_t != 0)
			ret++;
	}
	return ret;
}

static inline int
try_insert(struct member_ht_bucket *buckets, uint32_t prim, uint32_t sec,
		SIG_TYPE sig, MEMBER_SET_TYPE set_id)
{
	int i;
	/* If not full then insert into one slot*/
	for (i = 0; i < RTE_MEMBER_BUCKET_ENTRIES; i++) {
		if (buckets[prim].sets[i] == RTE_MEMBER_NO_MATCH) {
			buckets[prim].sigs[i] = sig;
			buckets[prim].sets[i] = set_id;
			return 0;
		}
	}
	/* if prim failed, we need to access second cache line */
	for (i = 0; i < RTE_MEMBER_BUCKET_ENTRIES; i++) {
		if (buckets[sec].sets[i] == RTE_MEMBER_NO_MATCH) {
			buckets[sec].sigs[i] = sig;
			buckets[sec].sets[i] = set_id;
			return 0;
		}
	}
	return -1;
}


static inline int
try_overwrite(struct member_ht_bucket *buckets, uint32_t prim, uint32_t sec,
		SIG_TYPE sig, MEMBER_SET_TYPE set_id)
{
	if (insert_overwrite_search(prim, sig, buckets, set_id) ||
			insert_overwrite_search(sec, sig, buckets,
				set_id))
		return 0;
	return -1;
}

static inline int
evict_from_bucket(void)
{
	/* for now, we randomly pick one entry to evict */
	return rte_rand() & (RTE_MEMBER_BUCKET_ENTRIES - 1);
}



/*
 * This function is similar to the cuckoo hash make_space function in hash
 * library
 */
static inline int
make_space_bucket(const struct rte_member_setsum *ss, uint32_t bkt_num)
{

	static unsigned int nr_pushes;

	unsigned int i, j;
	int ret;
	struct member_ht_bucket *buckets = ss->table;
	uint32_t next_bucket_idx;
	struct member_ht_bucket *next_bkt[RTE_MEMBER_BUCKET_ENTRIES];
	struct member_ht_bucket *bkt = &buckets[bkt_num];
	MEMBER_SET_TYPE flag_mask = 1U << (sizeof(MEMBER_SET_TYPE) * 8 - 1);
	/*
	 * Push existing item (search for bucket with space in
	 * alternative locations) to its alternative location
	 */
	for (i = 0; i < RTE_MEMBER_BUCKET_ENTRIES; i++) {
		/* Search for space in alternative locations */
		next_bucket_idx = (bkt->sigs[i] ^ bkt_num) & ss->bucket_mask;
		next_bkt[i] = &buckets[next_bucket_idx];
		for (j = 0; j < RTE_MEMBER_BUCKET_ENTRIES; j++) {
			if (next_bkt[i]->sets[j] == RTE_MEMBER_NO_MATCH)
				break;
		}

		if (j != RTE_MEMBER_BUCKET_ENTRIES)
			break;
	}

	/* Alternative location has spare room (end of recursive function) */
	if (i != RTE_MEMBER_BUCKET_ENTRIES) {
		next_bkt[i]->sigs[j] = bkt->sigs[i];
		next_bkt[i]->sets[j] = bkt->sets[i];
		return i;
	}

	/* Pick entry that has not been pushed yet */
	for (i = 0; i < RTE_MEMBER_BUCKET_ENTRIES; i++)
		if ((bkt->sets[i] & flag_mask) == 0)
			break;

	/* All entries have been pushed, so entry cannot be added */
	if (i == RTE_MEMBER_BUCKET_ENTRIES ||
			nr_pushes > RTE_MEMBER_MAX_PUSHES)
		return -ENOSPC;

	next_bucket_idx = (bkt->sigs[i] ^ bkt_num) & ss->bucket_mask;
	/* Set flag to indicate that this entry is going to be pushed */
	bkt->sets[i] |= flag_mask;

	nr_pushes++;
	/* Need room in alternative bucket to insert the pushed entry */
	ret = make_space_bucket(ss, next_bucket_idx);
	/*
	 * After recursive function.
	 * Clear flags and insert the pushed entry
	 * in its alternative location if successful,
	 * or return error
	 */
	bkt->sets[i] &= ~flag_mask;
	nr_pushes = 0;
	if (ret >= 0) {
		next_bkt[i]->sigs[ret] = bkt->sigs[i];
		next_bkt[i]->sets[ret] = bkt->sets[i];
		return i;
	} else
		return ret;
}


int
rte_member_add_ht(const struct rte_member_setsum *ss,
		const void *key, MEMBER_SET_TYPE set_id)
{
	int ret;
	uint32_t prim_bucket, sec_bucket;
	SIG_TYPE tmp_sig;
	struct member_ht_bucket *buckets = ss->table;
	MEMBER_SET_TYPE flag_mask = 1U << (sizeof(MEMBER_SET_TYPE) * 8 - 1);

	if (set_id == RTE_MEMBER_NO_MATCH || (set_id & flag_mask) != 0)
		return -EINVAL;

	get_buckets_index(ss, key, &prim_bucket, &sec_bucket, &tmp_sig);

	/* if it is cache based filter, we try overwriting existing entry */
	if (ss->cache) {
		ret = try_overwrite(buckets, prim_bucket, sec_bucket, tmp_sig,
					set_id);
		if (ret != -1)
			return ret;
	}
	/* If not full then insert into one slot*/
	ret = try_insert(buckets, prim_bucket, sec_bucket, tmp_sig, set_id);
	if (ret != -1)
		return ret;

	/* random pick prim or sec for recursive displacement */

	uint32_t select_bucket = (tmp_sig && 1U) ? prim_bucket : sec_bucket;
	if (ss->cache) {
		ret = evict_from_bucket();
		buckets[select_bucket].sigs[ret] = tmp_sig;
		buckets[select_bucket].sets[ret] = set_id;
		return 1;
	}

	ret = make_space_bucket(ss, select_bucket);
	if (ret >= 0) {
		buckets[select_bucket].sigs[ret] = tmp_sig;
		buckets[select_bucket].sets[ret] = set_id;
		ret = 1;
	}

	return ret;
}


void
rte_member_free_ht(struct rte_member_setsum *ss)
{
	rte_free(ss->table);
}


int
rte_member_delete_ht(struct rte_member_setsum *ss, const void *key,
		MEMBER_SET_TYPE set_id)
{
	int i;
	uint32_t prim_bucket, sec_bucket;
	SIG_TYPE tmp_sig;
	struct member_ht_bucket *buckets = ss->table;

	get_buckets_index(ss, key, &prim_bucket, &sec_bucket, &tmp_sig);

	for (i = 0; i < RTE_MEMBER_BUCKET_ENTRIES; i++) {
		if (tmp_sig == buckets[prim_bucket].sigs[i] &&
				set_id == buckets[prim_bucket].sets[i]) {
			buckets[prim_bucket].sets[i] = RTE_MEMBER_NO_MATCH;
			return 0;
		}
	}

	for (i = 0; i < RTE_MEMBER_BUCKET_ENTRIES; i++) {
		if (tmp_sig == buckets[sec_bucket].sigs[i] &&
				set_id == buckets[sec_bucket].sets[i]) {
			buckets[sec_bucket].sets[i] = RTE_MEMBER_NO_MATCH;
			return 0;
		}
	}
	return -ENOENT;
}


void
rte_member_reset_ht(const struct rte_member_setsum *ss)
{
	uint32_t i, j;
	struct member_ht_bucket *buckets = ss->table;
	for (i = 0; i < ss->bucket_cnt; i++) {
		for (j = 0; j < RTE_MEMBER_BUCKET_ENTRIES; j++)
			buckets[i].sets[j] = RTE_MEMBER_NO_MATCH;
	}
}
