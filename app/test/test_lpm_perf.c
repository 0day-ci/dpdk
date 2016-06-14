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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <rte_cycles.h>
#include <rte_random.h>
#include <rte_branch_prediction.h>
#include <rte_lpm.h>
#include <rte_ip.h>
#include <rte_malloc.h>

#include "test.h"
#include "resource.h"
#include "test_xmmt_ops.h"

REGISTER_LINKED_RESOURCE(test_lpm_data)

struct route_rule {
	uint32_t ip;
	uint8_t depth;
};
static struct route_rule *large_route_table;
static unsigned int num_route_entries;

#define TEST_LPM_ASSERT(cond) do {                                            \
	if (!(cond)) {                                                        \
		printf("Error at line %d: \n", __LINE__);                     \
		return -1;                                                    \
	}                                                                     \
} while(0)

#define ITERATIONS (1 << 10)
#define BATCH_SIZE (1 << 12)
#define BULK_SIZE 32

static void
print_route_distribution(const struct route_rule *table, uint32_t n)
{
	unsigned i, j;

	printf("Route distribution per prefix width: \n");
	printf("DEPTH    QUANTITY (PERCENT)\n");
	printf("--------------------------- \n");

	/* Count depths. */
	for (i = 1; i <= 32; i++) {
		unsigned depth_counter = 0;
		double percent_hits;

		for (j = 0; j < n; j++)
			if (table[j].depth == (uint8_t) i)
				depth_counter++;

		percent_hits = ((double)depth_counter)/((double)n) * 100;
		printf("%.2u%15u (%.2f)\n", i, depth_counter, percent_hits);
	}
	printf("\n");
}

static int
load_large_route_table(void)
{
	const struct resource *r;
	const char *lpm_data;

	r = resource_find("test_lpm_data");
	TEST_ASSERT_NOT_NULL(r, "No large lpm table data found");

	/* the routing table size is going to be less than the size of the
	 * resource, since text extries are more verbose. Allocate this as
	 * the max size, and shrink the allocation later
	 */
	large_route_table = rte_malloc(NULL, resource_size(r), 0);
	if (large_route_table == NULL)
		return -1;

	/* parse the lpm table. All entries are of format:
	 *  {IP-as-decimal-unsigned, depth}
	 * For example:
	 *  {1234567U, 24},
	 * We use the "U" and "}" characters as format check characters,
	 * after parsing each number.
	 */
	for (lpm_data = r->begin; lpm_data < r->end; lpm_data++) {
		if (*lpm_data == '{') {
			char *endptr;

			lpm_data++;
			large_route_table[num_route_entries].ip = \
				strtoul(lpm_data, &endptr, 0);
			if (*endptr != 'U') {
				if (num_route_entries > 0)
					printf("Failed parse of %.*s\n", 12, lpm_data);
				continue;
			}

			lpm_data = endptr + 2; /* skip U and , */
			large_route_table[num_route_entries].depth = \
				strtoul(lpm_data, &endptr, 0);
			if (*endptr != '}') {
				if (num_route_entries > 0)
					printf("Failed parse of %.*s\n",5, lpm_data);
				continue;
			}

			num_route_entries++;
		}
	}

	large_route_table = rte_realloc(large_route_table,
		sizeof(large_route_table[0]) * num_route_entries, 0);
	printf("Read %u route entries\n", num_route_entries);
	return 0;
}

static int
test_lpm_perf(void)
{
	struct rte_lpm *lpm = NULL;
	struct rte_lpm_config config;

	config.max_rules = 1000000;
	config.number_tbl8s = 256;
	config.flags = 0;
	uint64_t begin, total_time, lpm_used_entries = 0;
	unsigned i, j;
	uint32_t next_hop_add = 0xAA, next_hop_return = 0;
	int status = 0;
	uint64_t cache_line_counter = 0;
	int64_t count = 0;

	rte_srand(rte_rdtsc());

	TEST_ASSERT_SUCCESS(load_large_route_table(), "Error loading lpm table");
	printf("No. routes = %u\n", (unsigned) num_route_entries);

	print_route_distribution(large_route_table, (uint32_t) num_route_entries);

	lpm = rte_lpm_create(__func__, SOCKET_ID_ANY, &config);
	TEST_LPM_ASSERT(lpm != NULL);

	/* Measue add. */
	begin = rte_rdtsc();

	for (i = 0; i < num_route_entries; i++) {
		if (rte_lpm_add(lpm, large_route_table[i].ip,
				large_route_table[i].depth, next_hop_add) == 0)
			status++;
	}
	/* End Timer. */
	total_time = rte_rdtsc() - begin;

	printf("Unique added entries = %d\n", status);
	/* Obtain add statistics. */
	for (i = 0; i < RTE_LPM_TBL24_NUM_ENTRIES; i++) {
		if (lpm->tbl24[i].valid)
			lpm_used_entries++;

		if (i % 32 == 0) {
			if ((uint64_t)count < lpm_used_entries) {
				cache_line_counter++;
				count = lpm_used_entries;
			}
		}
	}

	printf("Used table 24 entries = %u (%g%%)\n",
			(unsigned) lpm_used_entries,
			(lpm_used_entries * 100.0) / RTE_LPM_TBL24_NUM_ENTRIES);
	printf("64 byte Cache entries used = %u (%u bytes)\n",
			(unsigned) cache_line_counter, (unsigned) cache_line_counter * 64);

	printf("Average LPM Add: %g cycles\n",
			(double)total_time / num_route_entries);

	/* Measure single Lookup */
	total_time = 0;
	count = 0;

	for (i = 0; i < ITERATIONS; i++) {
		static uint32_t ip_batch[BATCH_SIZE];

		for (j = 0; j < BATCH_SIZE; j++)
			ip_batch[j] = rte_rand();

		/* Lookup per batch */
		begin = rte_rdtsc();

		for (j = 0; j < BATCH_SIZE; j++) {
			if (rte_lpm_lookup(lpm, ip_batch[j], &next_hop_return) != 0)
				count++;
		}

		total_time += rte_rdtsc() - begin;

	}
	printf("Average LPM Lookup: %.1f cycles (fails = %.1f%%)\n",
			(double)total_time / ((double)ITERATIONS * BATCH_SIZE),
			(count * 100.0) / (double)(ITERATIONS * BATCH_SIZE));

	/* Measure bulk Lookup */
	total_time = 0;
	count = 0;
	for (i = 0; i < ITERATIONS; i++) {
		static uint32_t ip_batch[BATCH_SIZE];
		uint32_t next_hops[BULK_SIZE];

		/* Create array of random IP addresses */
		for (j = 0; j < BATCH_SIZE; j++)
			ip_batch[j] = rte_rand();

		/* Lookup per batch */
		begin = rte_rdtsc();
		for (j = 0; j < BATCH_SIZE; j += BULK_SIZE) {
			unsigned k;
			rte_lpm_lookup_bulk(lpm, &ip_batch[j], next_hops, BULK_SIZE);
			for (k = 0; k < BULK_SIZE; k++)
				if (unlikely(!(next_hops[k] & RTE_LPM_LOOKUP_SUCCESS)))
					count++;
		}

		total_time += rte_rdtsc() - begin;
	}
	printf("BULK LPM Lookup: %.1f cycles (fails = %.1f%%)\n",
			(double)total_time / ((double)ITERATIONS * BATCH_SIZE),
			(count * 100.0) / (double)(ITERATIONS * BATCH_SIZE));

	/* Measure LookupX4 */
	total_time = 0;
	count = 0;
	for (i = 0; i < ITERATIONS; i++) {
		static uint32_t ip_batch[BATCH_SIZE];
		uint32_t next_hops[4];

		/* Create array of random IP addresses */
		for (j = 0; j < BATCH_SIZE; j++)
			ip_batch[j] = rte_rand();

		/* Lookup per batch */
		begin = rte_rdtsc();
		for (j = 0; j < BATCH_SIZE; j += RTE_DIM(next_hops)) {
			unsigned k;
			xmm_t ipx4;

			ipx4 = vect_loadu_sil128((xmm_t *)(ip_batch + j));
			ipx4 = *(xmm_t *)(ip_batch + j);
			rte_lpm_lookupx4(lpm, ipx4, next_hops, UINT32_MAX);
			for (k = 0; k < RTE_DIM(next_hops); k++)
				if (unlikely(next_hops[k] == UINT32_MAX))
					count++;
		}

		total_time += rte_rdtsc() - begin;
	}
	printf("LPM LookupX4: %.1f cycles (fails = %.1f%%)\n",
			(double)total_time / ((double)ITERATIONS * BATCH_SIZE),
			(count * 100.0) / (double)(ITERATIONS * BATCH_SIZE));

	/* Delete */
	status = 0;
	begin = rte_rdtsc();

	for (i = 0; i < num_route_entries; i++) {
		/* rte_lpm_delete(lpm, ip, depth) */
		status += rte_lpm_delete(lpm, large_route_table[i].ip,
				large_route_table[i].depth);
	}

	total_time += rte_rdtsc() - begin;

	printf("Average LPM Delete: %g cycles\n",
			(double)total_time / num_route_entries);

	rte_lpm_delete_all(lpm);
	rte_lpm_free(lpm);

	return 0;
}

static struct test_command lpm_perf_cmd = {
	.command = "lpm_perf_autotest",
	.callback = test_lpm_perf,
};
REGISTER_TEST_COMMAND(lpm_perf_cmd);
