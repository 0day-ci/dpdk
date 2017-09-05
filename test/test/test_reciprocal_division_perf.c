/*
 *   BSD LICENSE
 *
 *   Copyright (C) Cavium, Inc. 2017.
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
 *     * Neither the name of Cavium, Inc nor the names of its
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

#include "test.h"

#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>

#include <rte_common.h>
#include <rte_cycles.h>
#include <rte_random.h>
#include <rte_reciprocal.h>

#define MAX_ITERATIONS	1000000
#define DIVIDE_ITER		100

static int
test_reciprocal_division_perf(void)
{
	int result = 0;
	uint32_t divisor_u32 = 0;
	volatile uint32_t dividend_u32;
	uint32_t nresult_u32;
	uint32_t rresult_u32;
	uint64_t divisor_u64 = 0;
	volatile uint64_t dividend_u64;
	uint64_t nresult_u64;
	uint64_t rresult_u64;
	uint64_t start_cyc;
	uint64_t split_cyc;
	uint64_t end_cyc;
	uint64_t tot_cyc_n = 0;
	uint64_t tot_cyc_r = 0;
	uint64_t i;
	struct rte_reciprocal_u32 reci_u32 = {0};
	struct rte_reciprocal_u64 reci_u64 = {0};

	rte_srand(rte_rdtsc());

	printf("Validating unsigned 32bit division.\n");

	for (i = 0; i < MAX_ITERATIONS; i++) {
		/* Change divisor every DIVIDE_ITER iterations. */
		if (i % DIVIDE_ITER == 0) {
			divisor_u32 = rte_rand();
			reci_u32 = rte_reciprocal_value_u32(divisor_u32);
		}

		dividend_u32 = rte_rand();

		start_cyc = rte_rdtsc();
		nresult_u32 = dividend_u32 / divisor_u32;
		split_cyc = rte_rdtsc();
		rresult_u32 = rte_reciprocal_divide_u32(dividend_u32,
				&reci_u32);
		end_cyc = rte_rdtsc();

		tot_cyc_n += split_cyc - start_cyc;
		tot_cyc_r += end_cyc - split_cyc;
		if (nresult_u32 != rresult_u32) {
			printf("Division failed, expected %"PRIu32" "
					"result %"PRIu32"",
					nresult_u32, rresult_u32);
			result = 1;
			break;
		}
	}
	printf("32bit Division results:\n");
	printf("Total number of cycles normal division     : %"PRIu64"\n",
			tot_cyc_n);
	printf("Total number of cycles reciprocal division : %"PRIu64"\n",
			tot_cyc_r);
	printf("Cycles per division(normal) : %3.2f\n",
			((double)tot_cyc_n)/i);
	printf("Cycles per division(reciprocal) : %3.2f\n\n",
			((double)tot_cyc_r)/i);

	tot_cyc_n = 0;
	tot_cyc_r = 0;

	printf("Validating unsigned 64bit division.\n");

	for (i = 0; i < MAX_ITERATIONS; i++) {
		/* Change divisor every DIVIDE_ITER iterations. */
		if (i % DIVIDE_ITER == 0) {
			divisor_u64 = rte_rand();
			reci_u64 = rte_reciprocal_value_u64(divisor_u64);
		}

		dividend_u64 = rte_rand();

		start_cyc = rte_rdtsc();
		nresult_u64 = dividend_u64 / divisor_u64;
		split_cyc = rte_rdtsc();
		rresult_u64 = rte_reciprocal_divide_u64(dividend_u64,
				&reci_u64);
		end_cyc = rte_rdtsc();

		tot_cyc_n += split_cyc - start_cyc;
		tot_cyc_r += end_cyc - split_cyc;
		if (nresult_u64 != rresult_u64) {
			printf("Division failed, expected %"PRIu64" "
					"result %"PRIu64"",
					nresult_u64, rresult_u64);
			result = 1;
			break;
		}
	}
	printf("64bit Division results:\n");
	printf("Total number of cycles normal division     : %"PRIu64"\n",
			tot_cyc_n);
	printf("Total number of cycles reciprocal division : %"PRIu64"\n",
			tot_cyc_r);
	printf("Cycles per division(normal) : %3.2f\n",
			((double)tot_cyc_n)/i);
	printf("Cycles per division(reciprocal) : %3.2f\n\n",
			((double)tot_cyc_r)/i);

	tot_cyc_n = 0;
	tot_cyc_r = 0;

	printf("Validating unsigned 64bit division with 32bit divisor.\n");

	for (i = 0; i < MAX_ITERATIONS; i++) {
		/* Change divisor every DIVIDE_ITER iterations. */
		if (i % DIVIDE_ITER == 0) {
			divisor_u64 = rte_rand() >> 32;
			reci_u64 = rte_reciprocal_value_u64(divisor_u64);
		}

		dividend_u64 = rte_rand();

		start_cyc = rte_rdtsc();
		nresult_u64 = dividend_u64 / divisor_u64;
		split_cyc = rte_rdtsc();
		rresult_u64 = rte_reciprocal_divide_u64(dividend_u64,
				&reci_u64);
		end_cyc = rte_rdtsc();

		tot_cyc_n += split_cyc - start_cyc;
		tot_cyc_r += end_cyc - split_cyc;
		if (nresult_u64 != rresult_u64) {
			printf("Division failed, expected %"PRIu64" "
					"result %"PRIu64"",
					nresult_u64, rresult_u64);
			result = 1;
			break;
		}
	}
	printf("64bit Division results:\n");
	printf("Total number of cycles normal division     : %"PRIu64"\n",
			tot_cyc_n);
	printf("Total number of cycles reciprocal division : %"PRIu64"\n",
			tot_cyc_r);
	printf("Cycles per division(normal) : %3.2f\n",
			((double)tot_cyc_n)/i);
	printf("Cycles per division(reciprocal) : %3.2f\n",
			((double)tot_cyc_r)/i);

	return result;
}

REGISTER_TEST_COMMAND(reciprocal_division_perf, test_reciprocal_division_perf);
