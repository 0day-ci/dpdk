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



/**
 * @file
 *
 * RTE Membership Library
 *
 * The Membership Library is an extension and generalization of a traditional
 * filter (for example Bloom Filter) structure that has multiple usages in a
 * variety of workloads and applications. The library is used to test if a key
 * belongs to certain sets. Two types of such "set-summary" structures are
 * implemented: hash-table based (HT) and vector bloom filter (vBF). For HT
 * setsummary, two subtype or modes are available, cache and non-cache modes.
 * The table below summarize some properties of the different implementations.
 *
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 */


/**
 * <!--
 * +==========+=====================+================+=========================+
 * |   type   |      vbf            |     HT-cache   |     HT-non-cache        |
 * +==========+=====================+==========================================+
 * |structure |  bloom-filter array |  hash-table like without storing key     |
 * +----------+---------------------+------------------------------------------+
 * |set id    | limited by bf count |           [1, 0x7fff]                    |
 * |          | up to 32.           |                                          |
 * +----------+---------------------+------------------------------------------+
 * |usages &  | small set range,    | can delete,    | cache most recent keys, |
 * |properties| user-specified      | big set range, | have both false-positive|
 * |          | false-positive rate,| small false    | and false-negative      |
 * |          | no deletion support.| positive depend| depend on table size,   |
 * |          |                     | on table size, | automatic overwritten.  |
 * |          |                     | new key does   |                         |
 * |          |                     | not overwrite  |                         |
 * |          |                     | existing key.  |                         |
 * +----------+---------------------+----------------+-------------------------+
 * -->
 */


#ifndef _RTE_MEMBER_H_
#define _RTE_MEMBER_H_



#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <stdint.h>
#include <rte_hash_crc.h>

/** The set ID type that stored internally in hash table based set summary. */
typedef uint16_t MEMBER_SET_TYPE;
/** Invalid set ID used to mean no match found. */
#define RTE_MEMBER_NO_MATCH 0
/** Maximum size of hash table that can be created. */
#define RTE_MEMBER_ENTRIES_MAX (1 << 30)
/** Maximum number of keys that can be searched as a bulk */
#define RTE_MEMBER_LOOKUP_BULK_MAX 64
/** Entry count per bucket in hash table based mode. */
#define RTE_MEMBER_BUCKET_ENTRIES 16
/** Maximum number of characters in setsum name. */
#define RTE_MEMBER_NAMESIZE 32

/** @internal Primary hash function to calculate 1st independent hash. */
#define MEMBER_PRIM_HASH(key, key_len, seed) \
	(uint32_t)(rte_hash_crc(key, key_len, seed))
/** @internal Secondary hash function to calculate 2nd independent hash. */
#define MEMBER_SEC_HASH(key, key_len, seed) \
	(uint32_t)(rte_hash_crc(key, key_len, seed))


struct rte_member_setsum;

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Parameter struct used to create set summary
 */
struct rte_member_parameters;


/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Define different set summary types
 */
enum rte_member_setsum_type {
	RTE_MEMBER_TYPE_HT = 0,  /**< Hash table based set summary. */
	RTE_MEMBER_TYPE_VBF,     /**< vector of bloom filters. */
	RTE_MEMBER_NUM_TYPE
};

/** @internal compare function for different arch. */
enum rte_member_sig_compare_function {
	RTE_MEMBER_COMPARE_SCALAR = 0,
	RTE_MEMBER_COMPARE_AVX2,
	RTE_MEMBER_COMPARE_NUM
};

/** @internal setsummary structure. */
struct rte_member_setsum {
	enum rte_member_setsum_type type;
	const char *name;
	uint32_t key_len;
	uint32_t socket_id;          /* NUMA Socket ID for memory. */
	uint32_t prim_hash_seed;
	uint32_t sec_hash_seed;


	/* hash table based */
	uint32_t bucket_cnt;
	uint32_t bucket_mask;
	uint8_t cache;
	enum rte_member_sig_compare_function sig_cmp_fn;

	/* vector bloom filter*/
	uint32_t num_set;
	uint32_t bits;
	uint32_t bit_mask;
	uint32_t num_hashes;
	void *table;
};


/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Parameters used when create the set summary table. Currently user can
 * specify two types of setsummary: HT based and vBF. For HT based, user can
 * specify cache or non-cache mode. Here is a table to describe some differences
 *
 */
struct rte_member_parameters {
	const char *name;			/**< Name of the hash. */

	/**
	 * User to specify the type of the setsummary from one of
	 * rte_member_setsum_type.
	 *
	 * HT based setsummary is implemented like a hash table. User should use
	 * this type when there are many sets.
	 *
	 * vBF setsummary is a vector of bloom filters. It is used when number
	 * of sets is not big (less than 32 for current implementation).
	 */
	enum rte_member_setsum_type type;
	/**
	 * If it is HT based setsummary, user to specify the subtype or mode
	 * of the setsummary. It could be cache, or non-cache mode.
	 * Set iscache to be 1 if to use as cache mode.
	 *
	 * For cache mode, keys can be evicted out of the HT setsummary. Keys
	 * with the same signature and map to the same bucket
	 * will overwrite each other in the setsummary table.
	 * This mode is useful for the case that the set-summary only
	 * needs to keep record of the recently inserted keys. Both
	 * false-negative and false-positive could happen.
	 *
	 * For non-cache mode, keys cannot be evicted out of the cache. So for
	 * this mode the setsummary will become full eventually. Keys with the
	 * same signature but map to the same bucket will still occupy multiple
	 * entries. This mode does not give false-negative result.
	 */
	uint8_t iscache;

	/**
	 * For HT setsummary, num_keys equals to the number of entries of the
	 * table. When the number of keys that inserted to the HT setsummary
	 * approaches this number, eviction could happen. For cache mode,
	 * keys could be evicted out of the table. For non-cache mode, keys will
	 * be evicted to other buckets like cuckoo hash. The table will also
	 * likely to become full before the number of inserted keys equal to the
	 * total number of entries.
	 *
	 * For vBF, num_keys equal to the expected number of keys that will
	 * be inserted into the vBF. The implementation assumes the keys are
	 * evenly distributed to each BF in vBF. This is used to calculate the
	 * number of bits we need for each BF. User does not specify the size of
	 * each BF directly because the optimal size depends on the num_keys
	 * and false positive rate.
	 */
	uint32_t num_keys;


	/**
	 * The length of key is used for hash calculation. Since key is not
	 * stored in set-summary, large key does not require more memory space.
	 */
	uint32_t key_len;


	/**
	 * num_set is only relevant to vBF based setsummary.
	 * num_set is equal to the number of BFs in vBF. For current
	 * implementation, it only supports 1,2,4,8,16,32 BFs in one vBF set
	 * summary. If other number of sets are needed, for example 5, the user
	 * should allocate the minimum available value that larger than 5,
	 * which is 8.
	 */
	uint32_t num_set;

	/**
	 * false_postive_rate is only relevant to vBF based setsummary.
	 * false_postivie_rate is the user-defined false positive rate
	 * given expected number of inserted keys (num_keys). It is used to
	 * calculate the total number of bits for each BF, and the number of
	 * hash values used during lookup and insertion. For details please
	 * refer to vBF implementation and membership library documentation.
	 *
	 * HT setsummary's false positive rate is in the order of:
	 * false_pos = (1/bucket_count)*(1/2^16), since we use 16-bit signature.
	 * This is because two keys needs to map to same bucket and same
	 * signature to have a collision (false positive). bucket_count is equal
	 * to number of entries (num_keys) divided by entry count per bucket
	 * (RTE_MEMBER_BUCKET_ENTRIES). Thus, the false_postivie_rate is not
	 * directly set by users.
	 */
	float false_positive_rate;

	/**
	 * We use two seeds to calculate two independent hashes for each key.
	 *
	 * For HT type, one hash is used as signature, and the other is used
	 * for bucket location.
	 * For vBF type, these two hashes and their combinations are used as
	 * hash locations to index the bit array.
	 */
	uint32_t prim_hash_seed;

	/**
	 * The secondary seed should be a different value from the primary seed.
	 */
	uint32_t sec_hash_seed;

	int socket_id;			/**< NUMA Socket ID for memory. */
};

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Find an existing set-summary and return a pointer to it.
 *
 * @param name
 *   Name of the set-summary
 * @return
 *   Pointer to the set-summary or NULL if object not found
 *   with rte_errno set appropriately. Possible rte_errno values include:
 *    - ENOENT - value not available for return
 */
void *
rte_member_find_existing(const char *name);

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Create set-summary (SS).
 *
 * @param params
 *   parameters to initialize the setsummary
 * @return
 *   return the pointer to the setsummary
 *   return value is NULL if the creation failed
 */

void *
rte_member_create(const struct rte_member_parameters *params);



/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Lookup key in set-summary (SS).
 * Single key lookup and return as soon as the first match found
 * @param setsum
 *   pointer of a setsummary
 * @param key
 *   pointer of the key that needs to lookup
 * @param set_id
 *   output the set id matches the key
 * @return
 *   return 1 for found a match and 0 for not found a match
 */

int
rte_member_lookup(const void *setsum,
		const void *key, MEMBER_SET_TYPE *set_id);


/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * lookup bulk of keys in set-summary (SS).
 * Each key lookup returns as soon as the first match found
 * @param setsum
 *   Pointer of a setsummary
 * @param keys
 *   Pointer of bulk of keys that to be lookup
 * @param num_keys
 *   Number of keys that will be lookup
 * @param set_ids
 *   Output set ids for all the keys to this array.
 *   User should preallocate array that can contain all results, which size is
 *   the num_keys.
 * @return
 *   The number of keys that found a match
 */


int
rte_member_lookup_bulk(const void *setsum,
		const void **keys, uint32_t num_keys,
		MEMBER_SET_TYPE *set_ids);




/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Lookup a key in set-summary (SS) for multiple matches.
 * The key lookup will find all matched entries (multiple match).
 * Note that for cache mode of HT, each key can have at most one match. This is
 * because keys with same signature that maps to same bucket will overwrite
 * each other. So multi-match lookup should be used for vBF and non-cache HT.
 * @param setsum
 *   pointer of a set-summary
 * @param key
 *   The key that to be lookup
 * @param max_match_per_key
 *   User specified maximum number of matches for each key. The function returns
 *   as soon as this number of matches found for the key.
 * @param set_id
 *   Output set ids for all the matches of the key. User needs to preallocate
 *   the array that can contain max_match_per_key number of results.
 * @return
 *   The number of matches that found for the key.
 *   For cache mode HT set-summary, the number should be at most 1
 */

int
rte_member_lookup_multi(const void *setsum,
		const void *key, uint32_t max_match_per_key,
		MEMBER_SET_TYPE *set_id);




/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Lookup a bulk of keys in set-summary (SS) for multiple matches each key.
 * Each key lookup will find all matched entries (multiple match).
 * Note that for cache mode HT, each key can have at most one match. So
 * multi-match function is mainly used for vBF and non-cache mode HT.
 * @param setsum
 *   pointer of a setsummary
 * @param keys
 *   The keys that to be lookup
 * @param num_keys
 *   The number of keys that will be lookup
 * @param max_match_per_key
 *   The possible maximum number of matches for each key
 * @param match_count
 *   Output the number of matches for each key in an array
 * @param set_ids
 *   Return set ids for all the matches of all keys. User pass in a preallocated
 *   2D array with first dimension as key index and second dimension as match
 *   index. For example set_ids[bulk_size][max_match_per_key]
 * @return
 *   The number of keys that found one or more matches in the set-summary
 */

int
rte_member_lookup_multi_bulk(const void *setsum,
		const void **keys, uint32_t num_keys,
		uint32_t max_match_per_key,
		uint32_t *match_count,
		MEMBER_SET_TYPE *set_ids);

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Insert key into set-summary (SS).
 *
 * @param setsum
 *   pointer of a set-summary
 * @param key
 *   the key that needs to be added
 * @param set_id
 *   The set id associated with the key that needs to be added. Different mode
 *   supports different set_id ranges. 0 cannot be used as set_id since
 *   RTE_MEMBER_NO_MATCH by default is set as 0.
 *   For HT mode, the set_id has range as [1, 0x7FFF], MSB is reserved.
 *   For vBF mode the set id is limited by the num_set parameter when create
 *   the set-summary.
 * @return
 *   HT (cache mode) and vBF should never fail unless the set_id is not in the
 *   valid range. In such case -EINVAL is returned.
 *   For HT (non-cache mode) it could fail with -ENOSPC error code when table is
 *   full.
 *   For success it returns different values for different modes to provide
 *   extra information for users.
 *   Return 0 for HT (cache mode) if the add does not cause
 *   eviction, return 1 otherwise. Return 0 for HT mode if success, -ENOSPC for
 *   full, and 1 if cuckoo eviction happens. Always return 0 for vBF mode.
 */

int
rte_member_add(const void *setsum, const void *key,
		MEMBER_SET_TYPE set_id);


/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * De-allocate memory used by set-summary.
 * @param setsum
 *   Pointer to the set summary
 */
void
rte_member_free(void *setsum);



/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Reset the set-summary tables. E.g. reset bits to be 0 in BF,
 * reset set_id in each entry to be RTE_MEMBER_NO_MATCH in HT based SS.
 * @param setsum
 *   Pointer to the set-summary
 */
void
rte_member_reset(const void *setsum);




/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Delete items from the set-summary. Note that vBF does not support deletion
 * in current implementation. For vBF, error code of -EINVAL will be returned.
 * @param setsum
 *   Pointer to the set-summary
 * @param key
 *   The key to be deleted
 * @param set_id
 *   For HT mode, we need both key and its corresponding set_id to
 *   properly delete the key. Without set_id, we may delete other keys with the
 *   same signature
 * @return
 *   If no entry found to delete, an error code of -ENOENT could be returned
 */

int
rte_member_delete(void *setsum, const void *key,
		MEMBER_SET_TYPE set_id);



#ifdef __cplusplus
}
#endif

#endif /* _RTE_MEMBER_H_ */
