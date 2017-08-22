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

/* This test is for member library's simple feature test */

#include <rte_memcpy.h>
#include <rte_malloc.h>
#include <rte_member.h>
#include <rte_byteorder.h>
#include <rte_random.h>
#include <rte_debug.h>
#include <rte_ip.h>

#include "test.h"

void *setsum_ht;
void *setsum_cache;
void *setsum_vbf;

/* 5-tuple key type */
struct flow_key {
	uint32_t ip_src;
	uint32_t ip_dst;
	uint16_t port_src;
	uint16_t port_dst;
	uint8_t proto;
} __attribute__((packed));


/* Keys used by unit test functions */
static struct flow_key keys[5] = {
	{
		.ip_src = IPv4(0x03, 0x02, 0x01, 0x00),
		.ip_dst = IPv4(0x07, 0x06, 0x05, 0x04),
		.port_src = 0x0908,
		.port_dst = 0x0b0a,
		.proto = 0x0c,
	},
	{
		.ip_src = IPv4(0x13, 0x12, 0x11, 0x10),
		.ip_dst = IPv4(0x17, 0x16, 0x15, 0x14),
		.port_src = 0x1918,
		.port_dst = 0x1b1a,
		.proto = 0x1c,
	},
	{
		.ip_src = IPv4(0x23, 0x22, 0x21, 0x20),
		.ip_dst = IPv4(0x27, 0x26, 0x25, 0x24),
		.port_src = 0x2928,
		.port_dst = 0x2b2a,
		.proto = 0x2c,
	},
	{
		.ip_src = IPv4(0x33, 0x32, 0x31, 0x30),
		.ip_dst = IPv4(0x37, 0x36, 0x35, 0x34),
		.port_src = 0x3938,
		.port_dst = 0x3b3a,
		.proto = 0x3c,
	},
	{
		.ip_src = IPv4(0x43, 0x42, 0x41, 0x40),
		.ip_dst = IPv4(0x47, 0x46, 0x45, 0x44),
		.port_src = 0x4948,
		.port_dst = 0x4b4a,
		.proto = 0x4c,
	}
};

uint32_t test_set[5] = {1, 2, 3, 4, 5};

#define ITERATIONS  3
#define KEY_SIZE  4

#define MAX_ENTRIES (1 << 16)
uint8_t gened_keys[MAX_ENTRIES][KEY_SIZE];

static struct rte_member_parameters params = {
		.num_keys = MAX_ENTRIES,       /* Total hash table entries. */
		.key_len = KEY_SIZE,       /* Length of hash key. */

		/*num_set and false_positive_rate only relevant to vBF setsum*/
		.num_set = 32,
		.false_positive_rate = 0.03,
		.prim_hash_seed = 1,
		.sec_hash_seed = 11,
		.socket_id = 1          /* NUMA Socket ID for memory. */
};


/* Create test setsummaries. */
static int test_member_create(void)
{
	params.key_len = sizeof(struct flow_key);

	params.name = "test_member_ht";
	params.iscache = 0;
	params.type = RTE_MEMBER_TYPE_HT;
	setsum_ht = rte_member_create(&params);

	params.name = "test_member_cache";
	params.iscache = 1;
	setsum_cache = rte_member_create(&params);

	params.name = "test_member_vbf";
	params.type = RTE_MEMBER_TYPE_VBF;
	setsum_vbf = rte_member_create(&params);

	if (setsum_ht == NULL || setsum_cache == NULL || setsum_vbf == NULL) {
		printf("Creation of setsums fail\n");
		return -1;
	}
	printf("Creation of setsums success\n");
	return 0;
}

static int test_member_insert(void)
{
	int ret_ht, ret_cache, ret_vbf, i;

	for (i = 0; i < 5; i++) {
		ret_ht = rte_member_add(setsum_ht, &keys[i], test_set[i]);
		ret_cache = rte_member_add(setsum_cache, &keys[i],
						test_set[i]);
		ret_vbf = rte_member_add(setsum_vbf, &keys[i], test_set[i]);
		TEST_ASSERT(ret_ht >= 0 && ret_cache >= 0 && ret_vbf >= 0,
				"insert error");
	}
	printf("insert key success\n");
	return 0;
}

static int test_member_lookup(void)
{
	int ret_ht, ret_cache, ret_vbf, i;
	uint16_t set_ht, set_cache, set_vbf;
	MEMBER_SET_TYPE set_ids_ht[5] = {0};
	MEMBER_SET_TYPE set_ids_cache[5] = {0};
	MEMBER_SET_TYPE set_ids_vbf[5] = {0};

	uint32_t num_key_ht = 5;
	uint32_t num_key_cache = 5;
	uint32_t num_key_vbf = 5;

	const void *key_array[5];

	/* single lookup test */
	for (i = 0; i < 5; i++) {
		ret_ht = rte_member_lookup(setsum_ht, &keys[i], &set_ht);
		ret_cache = rte_member_lookup(setsum_cache, &keys[i],
							&set_cache);
		ret_vbf = rte_member_lookup(setsum_vbf, &keys[i], &set_vbf);
		TEST_ASSERT(ret_ht >= 0 && ret_cache >= 0 && ret_vbf >= 0,
				"single lookup function error");

		TEST_ASSERT(set_ht == test_set[i] &&
				set_cache == test_set[i] &&
				set_vbf == test_set[i],
				"single lookup set value error");
	}
	printf("lookup single key success\n");

	/* bulk lookup test */
	for (i = 0; i < 5; i++)
		key_array[i] = &keys[i];

	ret_ht = rte_member_lookup_bulk(setsum_ht, &key_array[0],
			num_key_ht, set_ids_ht);

	ret_cache = rte_member_lookup_bulk(setsum_cache, &key_array[0],
			num_key_cache, set_ids_cache);

	ret_vbf = rte_member_lookup_bulk(setsum_vbf, &key_array[0],
			num_key_vbf, set_ids_vbf);

	TEST_ASSERT(ret_ht >= 0 && ret_cache >= 0 && ret_vbf >= 0,
			"bulk lookup function error");

	for (i = 0; i < 5; i++) {
		TEST_ASSERT((set_ids_ht[i] == test_set[i]) &&
				(set_ids_cache[i] == test_set[i]) &&
				(set_ids_vbf[i] == test_set[i]),
				"bulk lookup result error");
	}

	return 0;
}


static int test_member_delete(void)
{
	int ret_ht, ret_cache, ret_vbf, i;
	uint16_t set_ht, set_cache, set_vbf;
	for (i = 0; i < 5; i++) {
		ret_ht = rte_member_delete(setsum_ht, &keys[i], test_set[i]);
		ret_cache = rte_member_delete(setsum_cache, &keys[i],
						test_set[i]);
		ret_vbf = rte_member_delete(setsum_vbf, &keys[i], test_set[i]);
		/* VBF does not support delete yet, so return error code */
		TEST_ASSERT(ret_ht >= 0 && ret_cache >= 0,
				"key deletion function error");
		TEST_ASSERT(ret_vbf < 0,
				"vbf does not support deletion, error");
	}

	for (i = 0; i < 5; i++) {
		ret_ht = rte_member_lookup(setsum_ht, &keys[i], &set_ht);
		ret_cache = rte_member_lookup(setsum_cache, &keys[i],
						&set_cache);
		ret_vbf = rte_member_lookup(setsum_vbf, &keys[i], &set_vbf);
		TEST_ASSERT(ret_ht >= 0 && ret_cache >= 0,
				"key lookup function error");
		TEST_ASSERT(set_ht == RTE_MEMBER_NO_MATCH &&
				ret_cache == RTE_MEMBER_NO_MATCH,
				"key deletion failed");
	}
	printf("delete success\n");
	return 0;
}


static int test_member_multimatch(void)
{
	int ret_ht, ret_vbf, ret_cache;
	MEMBER_SET_TYPE set_ids_ht[32] = {0};
	MEMBER_SET_TYPE set_ids_vbf[32] = {0};
	MEMBER_SET_TYPE set_ids_cache[32] = {0};

	MEMBER_SET_TYPE set_ids_ht_m[5][32] = {{0} };
	MEMBER_SET_TYPE set_ids_vbf_m[5][32] = {{0} };
	MEMBER_SET_TYPE set_ids_cache_m[5][32] = {{0} };

	uint32_t match_count_ht[5];
	uint32_t match_count_vbf[5];
	uint32_t match_count_cache[5];

	uint32_t num_key_ht = 5;
	uint32_t num_key_vbf = 5;
	uint32_t num_key_cache = 5;

	const void *key_array[5];

	uint32_t i, j;
	/* same key at most inserted 2*entry_per_bucket times for HT mode */
	for (i = 1; i < 33; i++) {
		for (j = 0; j < 5; j++) {
			ret_ht = rte_member_add(setsum_ht, &keys[j], i);
			ret_vbf = rte_member_add(setsum_vbf, &keys[j], i);
			ret_cache = rte_member_add(setsum_cache, &keys[j], i);

			TEST_ASSERT(ret_ht >= 0 && ret_vbf >= 0 &&
					ret_cache >= 0,
					"insert function error");
		}
	}

	/* single multimatch test */
	for (i = 0; i < 5; i++) {
		ret_vbf = rte_member_lookup_multi(setsum_vbf, &keys[i], 32,
							set_ids_vbf);
		ret_ht = rte_member_lookup_multi(setsum_ht, &keys[i], 32,
							set_ids_ht);
		ret_cache = rte_member_lookup_multi(setsum_cache, &keys[i], 32,
							set_ids_cache);
		/*
		 * for cache mode, it does not support multimatch
		 * the mutimatch should work like single match
		 */
		TEST_ASSERT(ret_ht == 32 && ret_vbf == 32 && ret_cache == 1,
				"single lookup_multi error");
		TEST_ASSERT(set_ids_cache[0] == 32,
				"single lookup_multi cache error");

		for (j = 1; j < 33; j++) {
			TEST_ASSERT(set_ids_ht[j-1] == j &&
					set_ids_vbf[j-1] == j,
					"single multimatch lookup error");
		}
	}
	printf("lookup single key for multimatch success\n");

	/* bulk multimatch test */

	for (i = 0; i < 5; i++)
		key_array[i] = &keys[i];
	ret_vbf = rte_member_lookup_multi_bulk(setsum_vbf,
			&key_array[0], num_key_ht, 32, match_count_vbf,
			(MEMBER_SET_TYPE *)set_ids_vbf_m);

	ret_ht = rte_member_lookup_multi_bulk(setsum_ht,
			&key_array[0], num_key_vbf, 32, match_count_ht,
			(MEMBER_SET_TYPE *)set_ids_ht_m);

	ret_cache = rte_member_lookup_multi_bulk(setsum_cache,
			&key_array[0], num_key_cache, 32, match_count_cache,
			(MEMBER_SET_TYPE *)set_ids_cache_m);


	for (j = 0; j < 5; j++) {
		TEST_ASSERT(match_count_ht[j] == 32,
			"bulk multimatch lookup HT match count error");
		TEST_ASSERT(match_count_vbf[j] == 32,
			"bulk multimatch lookup vBF match count error");
		TEST_ASSERT(match_count_cache[j] == 1,
			"bulk multimatch lookup CACHE match count error");
		TEST_ASSERT(set_ids_cache_m[j][0] == 32,
			"bulk multimatch lookup CACHE set value error");

		for (i = 1; i < 33; i++) {
			TEST_ASSERT(set_ids_ht_m[j][i-1] == i,
				"bulk multimatch lookup HT set value error");
			TEST_ASSERT(set_ids_vbf_m[j][i-1] == i,
				"bulk multimatch lookup vBF set value error");
		}
	}

	printf("lookup for bulk multimatch success\n");

	return 0;
}


static int key_compare(const void *key1, const void *key2)
{
	return memcmp(key1, key2, KEY_SIZE);
}

static void
setup_keys_and_data(void)
{
	unsigned int i, j;
	int num_duplicates;

	/* Reset all arrays */
	for (i = 0; i < KEY_SIZE; i++)
		gened_keys[0][i] = 0;

	/* Generate a list of keys, some of which may be duplicates */
	for (i = 0; i < MAX_ENTRIES; i++) {
		for (j = 0; j < KEY_SIZE; j++)
			gened_keys[i][j] = rte_rand() & 0xFF;
	}

	/* Remove duplicates from the keys array */
	do {
		num_duplicates = 0;
		/* Sort the list of keys to make it easier to find duplicates */
		qsort(gened_keys, MAX_ENTRIES, KEY_SIZE, key_compare);

		/* Sift through the list of keys and look for duplicates */
		int num_duplicates = 0;
		for (i = 0; i < MAX_ENTRIES - 1; i++) {
			if (memcmp(gened_keys[i], gened_keys[i + 1],
					KEY_SIZE) == 0) {
				/* This key already exists, try again */
				num_duplicates++;
				for (j = 0; j < KEY_SIZE; j++)
					gened_keys[i][j] = rte_rand() & 0xFF;
			}
		}
	} while (num_duplicates != 0);
}


static inline int
add_gened_keys(void *setsum, unsigned int *added_keys)
{
	int ret = 0;

	for (*added_keys = 0; ret >= 0 && *added_keys < MAX_ENTRIES;
			(*added_keys)++) {
		uint16_t set = (rte_rand() & 0xf) + 1;
		ret = rte_member_add(setsum, &gened_keys[*added_keys], set);
	}
	return ret;
}


static inline int
add_gened_keys_cache(void *setsum, unsigned int *added_keys)
{
	int ret = 0;

	for (*added_keys = 0; ret == 0 && *added_keys < MAX_ENTRIES;
			(*added_keys)++) {
		uint16_t set = (rte_rand() & 0xf) + 1;
		ret = rte_member_add(setsum, &gened_keys[*added_keys], set);
	}
	return ret;
}

static int
test_member_loadfactor(void)
{
	unsigned  int j;
	unsigned int added_keys, average_keys_added = 0;
	int ret;

	setup_keys_and_data();

	rte_member_free(setsum_ht);
	rte_member_free(setsum_cache);
	rte_member_free(setsum_vbf);

	params.key_len = KEY_SIZE;
	params.name = "test_member_ht";
	params.iscache = 0;
	params.type = RTE_MEMBER_TYPE_HT;
	setsum_ht = rte_member_create(&params);

	params.name = "test_member_cache";
	params.iscache = 1;
	setsum_cache = rte_member_create(&params);


	if (setsum_ht == NULL || setsum_cache == NULL) {
		printf("Creation of setsums fail\n");
		return -1;
	}
	/* test HT mode */
	for (j = 0; j < ITERATIONS; j++) {
		/* Add random entries until key cannot be added */
		ret = add_gened_keys(setsum_ht, &added_keys);
		if (ret != -ENOSPC) {
			printf("Unexpected error when adding keys\n");
			return -1;
		}
		average_keys_added += added_keys;

		/* Reset the table */
		rte_member_reset(setsum_ht);

		/* Print a dot to show progress on operations */
		printf(".");
		fflush(stdout);
	}

	average_keys_added /= ITERATIONS;

	printf("\nKeys inserted when no space(non-cache) = %.2f%% (%u/%u)\n",
		((double) average_keys_added / params.num_keys * 100),
		average_keys_added, params.num_keys);

	/* test cache mode */
	added_keys = average_keys_added = 0;
	for (j = 0; j < ITERATIONS; j++) {
		/* Add random entries until key cannot be added */
		ret = add_gened_keys_cache(setsum_cache, &added_keys);
		if (ret != 1) {
			printf("Unexpected error when adding keys\n");
			return -1;
		}
		average_keys_added += added_keys;

		/* Reset the table */
		rte_member_reset(setsum_cache);

		/* Print a dot to show progress on operations */
		printf(".");
		fflush(stdout);
	}

	average_keys_added /= ITERATIONS;

	printf("\nKeys inserted when eviction happens(cache)= %.2f%% (%u/%u)\n",
		((double) average_keys_added / params.num_keys * 100),
		average_keys_added, params.num_keys);
	return 0;
}



static int
test_member(void)
{

	/* Simple tests for initial debug usage */
	if (test_member_create() < 0) {
		rte_member_free(setsum_ht);
		rte_member_free(setsum_cache);
		rte_member_free(setsum_vbf);
		return -1;
	}
	if (test_member_insert() < 0) {
		rte_member_free(setsum_ht);
		rte_member_free(setsum_cache);
		rte_member_free(setsum_vbf);
		return -1;
	}
	if (test_member_lookup() < 0) {
		rte_member_free(setsum_ht);
		rte_member_free(setsum_cache);
		rte_member_free(setsum_vbf);
		return -1;
	}
	if (test_member_delete() < 0) {
		rte_member_free(setsum_ht);
		rte_member_free(setsum_cache);
		rte_member_free(setsum_vbf);
		return -1;
	}
	if (test_member_multimatch() < 0) {
		rte_member_free(setsum_ht);
		rte_member_free(setsum_cache);
		rte_member_free(setsum_vbf);
		return -1;
	}


	if (test_member_loadfactor() < 0) {
		rte_member_free(setsum_ht);
		rte_member_free(setsum_cache);
	}

	rte_member_free(setsum_ht);
	rte_member_free(setsum_cache);
	rte_member_free(setsum_vbf);
	return 0;
}

REGISTER_TEST_COMMAND(member_autotest, test_member);
