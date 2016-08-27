/*-
 *	 BSD LICENSE
 *
 *	 Copyright(c) 2016 Intel Corporation. All rights reserved.
 *	 All rights reserved.
 *
 *	 Redistribution and use in source and binary forms, with or without
 *	 modification, are permitted provided that the following conditions
 *	 are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *		 notice, this list of conditions and the following disclaimer.
 *	* Redistributions in binary form must reproduce the above copyright
 *		 notice, this list of conditions and the following disclaimer in
 *		 the documentation and/or other materials provided with the
 *		 distribution.
 *	* Neither the name of Intel Corporation nor the names of its
 *		 contributors may be used to endorse or promote products derived
 *		 from this software without specific prior written permission.
 *
 *	 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *	 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *	 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *	 A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *	 OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *	 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *	 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *	 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *	 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *	 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <string.h>
#include <stdio.h>

#include <rte_common.h>
#include <rte_mbuf.h>
#include <rte_memory.h>
#include <rte_malloc.h>
#include <rte_log.h>

#include <rte_hash.h>
#include "rte_table_hash.h"

#ifdef RTE_TABLE_STATS_COLLECT

#define RTE_TABLE_HASH_CUCKOO_STATS_PKTS_IN_ADD(table, val) \
	table->stats.n_pkts_in += val
#define RTE_TABLE_HASH_CUCKOO_STATS_PKTS_LOOKUP_MISS(table, val) \
	table->stats.n_pkts_lookup_miss += val

#else

#define RTE_TABLE_HASH_CUCKOO_STATS_PKTS_IN_ADD(table, val)
#define RTE_TABLE_HASH_CUCKOO_STATS_PKTS_LOOKUP_MISS(table, val)

#endif


struct rte_table_hash {
	/* ToDo: */
	struct rte_table_stats stats;

	/* Input parameters */
	uint32_t key_size;
	uint32_t entry_size;
	uint32_t n_keys;
	rte_table_hash_op_hash f_hash;
	uint32_t seed;
	uint32_t signature_offset;
	uint32_t key_offset;
	const char *name;

	/* cuckoo hash table object */
	struct rte_hash *h_table;

	/* Lookup table */
	uint8_t memory[0] __rte_cache_aligned;
};

static int
check_params_create_hash_cuckoo(const struct rte_table_hash_cuckoo_params *params)
{
	/* Check for valid parameters */
	if (params == NULL) {
		RTE_LOG(ERR, TABLE, "NULL Input Parameters.\n");
		return -EINVAL;
	}

	if(params->key_size == 0){
		RTE_LOG(ERR, TABLE, "Invalid key_size.\n");
		return -EINVAL;
	}

	if (params->n_keys == 0) {
		RTE_LOG(ERR, TABLE, "Invalid n_keys.\n");
		return -EINVAL;
	}

	if (params->f_hash== NULL) {
		RTE_LOG(ERR, TABLE, "f_hash is NULL.\n");
		return -EINVAL;
	}

	if (params->name == NULL) {
		RTE_LOG(ERR, TABLE, "Table name is NULL.\n");
		return -EINVAL;
	}

	return 0;
}

static void *
rte_table_hash_cuckoo_create(void * params,
			int socket_id,
			uint32_t entry_size)
{
	struct rte_hash *rte_hash_handle;
	struct rte_table_hash *t;
	uint32_t total_size;

	/* Check input parameters */
	struct rte_table_hash_cuckoo_params *p =
		(struct rte_table_hash_cuckoo_params *) params;

	if(check_params_create_hash_cuckoo(params))
		return NULL;

	/* Memory allocation */
	total_size = sizeof(struct rte_table_hash) + p->n_keys
			 * entry_size + entry_size;
	t = rte_zmalloc_socket("TABLE",
			total_size,
			RTE_CACHE_LINE_SIZE,
			socket_id);
	if (t == NULL) {
		RTE_LOG(ERR, TABLE,
			"%s: Cannot allocate %u bytes for Cuckoo hash table\n",
			__func__,
			(uint32_t)sizeof(struct rte_table_hash));
		return NULL;
	}

	/* Create cuckoo hash table */
	struct rte_hash_parameters hash_cuckoo_params = {
		.entries = p->n_keys,
		.key_len = p->key_size,
		.hash_func = (rte_hash_function)(p->f_hash),
		.hash_func_init_val = p->seed,
		.socket_id = socket_id,
		.name = p->name
	};

	rte_hash_handle = rte_hash_find_existing(p->name);
	if (rte_hash_handle == NULL) {
		rte_hash_handle = rte_hash_create(&hash_cuckoo_params);
		if( NULL == rte_hash_handle) {
			RTE_LOG(ERR, TABLE,
				"%s: failed to create cuckoo hash table. keysize: %u",
				__func__, hash_cuckoo_params.key_len);
			rte_free(t);
			return NULL;
		}
	}

	/* initialize the cuckoo hash parameters */
	t->key_size = p->key_size;
	t->entry_size = entry_size;
	t->n_keys = p->n_keys;
	t->f_hash = p->f_hash;
	t->seed = p->seed;
	t->signature_offset = p->signature_offset;
	t->key_offset = p->key_offset;
	t->name = p->name;
	t->h_table = rte_hash_handle;

	RTE_LOG(INFO, TABLE,
		"%s: Cuckoo Hash table memory footprint is %u bytes\n",
		__func__, total_size);
	return t;
}

static int
rte_table_hash_cuckoo_free(void *table)
{
	if(table == NULL) {
		RTE_LOG(ERR, TABLE, "%s: table parameter is NULL\n", __func__);
		return -EINVAL;
	}

	struct rte_table_hash *t = (struct rte_table_hash*)table;

	rte_hash_free(t->h_table);
	rte_free(t);

	return 0;
}

static int
rte_table_hash_cuckoo_entry_add(void *table, void *key, void *entry,
        int *key_found, void **entry_ptr)
{
	int pos = 0;

	if(table == NULL) {
		RTE_LOG(ERR, TABLE, "%s: table parameter is NULL\n", __func__);
		return -EINVAL;
	}

	if(key == NULL) {
		RTE_LOG(ERR, TABLE, "%s: key parameter is NULL\n", __func__);
		return -EINVAL;
	}

	if(entry == NULL) {
		RTE_LOG(ERR, TABLE, "%s: entry parameter is NULL\n", __func__);
		return -EINVAL;
	}

	struct rte_table_hash *t = (struct rte_table_hash*)table;

	// Find Existing entries
	pos = rte_hash_lookup(t->h_table, key);
	if (pos >= 0) {
		uint8_t *existing_entry;

		*key_found = 1;
		existing_entry = &t->memory[pos * t->entry_size];
		memcpy(existing_entry, entry, t->entry_size);
		*entry_ptr = existing_entry;

		return 0;
	} else if (pos == -ENOENT) {
	// Entry not found. Adding new entry
		uint8_t *new_entry;

		pos = rte_hash_add_key(t->h_table, key);
		if (pos < 0) {
			RTE_LOG(ERR, TABLE,
				"%s: Entry not added, status : %u\n",
				__func__, pos);
			return pos;
		}

		new_entry = &t->memory[pos * t->entry_size];
		memcpy(new_entry, entry, t->entry_size);

		*key_found = 0;
		*entry_ptr = new_entry;
		return 0;
	}
	return pos;
}

static int
rte_table_hash_cuckoo_entry_delete(void *table, void *key, 
        int *key_found, __rte_unused void *entry)
{
	int pos = 0;

	if(table == NULL) {
		RTE_LOG(ERR, TABLE, "%s: table parameter is NULL\n", __func__);
		return -EINVAL;
	}

	if(key == NULL) {
		RTE_LOG(ERR, TABLE, "%s: key parameter is NULL\n", __func__);
		return -EINVAL;
	}

	struct rte_table_hash *t = (struct rte_table_hash*)table;
	pos = rte_hash_del_key(t->h_table, key);
	if (pos >= 0) {
		*key_found = 1;
		memset(&t->memory[pos * t->entry_size], 0, t->entry_size);
	}

	return pos;
}

static int
rte_table_hash_cuckoo_lookup_bulk_dosig(void *table, 
        struct rte_mbuf **pkts,
        uint64_t pkts_mask,
        uint64_t *lookup_hit_mask,
        void **entries)
{
	uint8_t *keys[64];
	int32_t positions[64];
	uint32_t i;
	__rte_unused int32_t num_hits;

	struct rte_table_hash *t = (struct rte_table_hash*)table;
	uint32_t n_pkts_in = __builtin_popcountll(pkts_mask);
	RTE_TABLE_HASH_CUCKOO_STATS_PKTS_IN_ADD(t, n_pkts_in);
	
	/* Key prep for Bulk Lookup */
	for (i = 0; i < n_pkts_in; i++) {
		keys[i] = RTE_MBUF_METADATA_UINT8_PTR(pkts[i], t->key_offset);
	}

	/* Bulk Lookup */
	num_hits = rte_hash_lookup_bulk(t->h_table,
					(const void **) keys,
					n_pkts_in,
					positions);

	/* Retrieving results from Bulk Lookup */
	for (i = 0; i < n_pkts_in; i++) {

		if (unlikely(positions[i] < 0)) {
			entries[i] = NULL;
			continue;
		}
		entries[i] = &t->memory[positions[i] * t->entry_size];
		*lookup_hit_mask = *lookup_hit_mask | (uint64_t)1 << i;
	}
	RTE_TABLE_HASH_CUCKOO_STATS_PKTS_IN_ADD(t, n_pkts_in -
				 __builtin_popcountll(*lookup_hit_mask));

	return 0;
}

static int
rte_table_hash_cuckoo_lookup_dosig(void *table, 
        struct rte_mbuf **pkts,
        uint64_t pkts_mask,
        uint64_t *lookup_hit_mask,
        void **entries)
{
	int pos = 0;
	uint32_t i;
	uint8_t *key;

	if(table == NULL) {
		RTE_LOG(ERR, TABLE, "%s: table parameter is NULL\n", __func__);
		return -EINVAL;
	}

	if(pkts == NULL) {
		RTE_LOG(ERR, TABLE, "%s: pkts parameter is NULL\n", __func__);
		return -EINVAL;
	}

	struct rte_table_hash *t = (struct rte_table_hash*)table;
	*lookup_hit_mask = 0;

	/* Bulk Lookup */
	if (likely(!(pkts_mask & (pkts_mask + 1)))) {
		return(rte_table_hash_cuckoo_lookup_bulk_dosig(table, 
							pkts,
							pkts_mask,
							lookup_hit_mask,
							entries));
	}

	/* One by One Lookup */
	__rte_unused uint32_t n_pkts_in = __builtin_popcountll(pkts_mask);
	RTE_TABLE_HASH_CUCKOO_STATS_PKTS_IN_ADD(t, n_pkts_in);

	for ( i = 0; pkts_mask; i++) {
		if (!(pkts_mask & 1)) {
			pkts_mask = pkts_mask >> 1;
			continue;
		}

		struct rte_mbuf *pkt = pkts[i];
		key = RTE_MBUF_METADATA_UINT8_PTR(pkt, t->key_offset);
		pos = rte_hash_lookup(t->h_table, key);
		if (unlikely(pos < 0)) {
			RTE_LOG(INFO, TABLE, "%s: Lookup miss for Packet (%u)n",
				__func__, i); 
			pkts_mask = pkts_mask >> 1;
			continue;
		}
		entries[i] = &t->memory[pos * t->entry_size];
		*lookup_hit_mask = *lookup_hit_mask | ((uint64_t)1 << i);
		pkts_mask = pkts_mask >> 1;
	}
	RTE_TABLE_HASH_CUCKOO_STATS_PKTS_IN_ADD(t, n_pkts_in -
				 __builtin_popcountll(*lookup_hit_mask));

	return 0;
}

static int                                                                  
rte_table_hash_cuckoo_stats_read(void *table, struct rte_table_stats *stats, 
        int clear)
{
	if(table == NULL) {
		RTE_LOG(ERR, TABLE, "%s: table parameter is NULL\n", __func__);
		return -EINVAL;
	}

	struct rte_table_hash *t = (struct rte_table_hash *) table;

	if (stats != NULL)
		memcpy(stats, &t->stats, sizeof(t->stats));

	if (clear)
		memset(&t->stats, 0, sizeof(t->stats));

	return 0;
}

struct rte_table_ops rte_table_hash_cuckoo_dosig_ops = {
	.f_create = rte_table_hash_cuckoo_create,
	.f_free = rte_table_hash_cuckoo_free,
	.f_add = rte_table_hash_cuckoo_entry_add,
	.f_delete = rte_table_hash_cuckoo_entry_delete,
	.f_add_bulk = NULL,
	.f_delete_bulk = NULL,
	.f_lookup = rte_table_hash_cuckoo_lookup_dosig,
	.f_stats = rte_table_hash_cuckoo_stats_read,
};         

