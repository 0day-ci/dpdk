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

 /*
  * This is a simple functional test for rte_smp_mb() implementation.
  * I.E. make sure that LOAD and STORE operations that precede the
  * rte_smp_mb() call are globally visible across the lcores
  * before the the LOAD and STORE operations that follows it.
  * The test uses simple implementation of Peterson's lock algorithm
  * for two execution units to make sure that rte_smp_mb() prevents
  * store-load reordering to happen.
  * Also when executed on a single lcore could be used as a approxiamate
  * estimation of number of cycles particular implementation of rte_smp_mb()
  * will take.
  */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include <rte_memory.h>
#include <rte_per_lcore.h>
#include <rte_launch.h>
#include <rte_atomic.h>
#include <rte_eal.h>
#include <rte_lcore.h>
#include <rte_pause.h>
#include <rte_random.h>
#include <rte_cycles.h>
#include <rte_vect.h>
#include <rte_debug.h>

#include "test.h"

#define ADD_MAX		8
#define ITER_MAX	0x1000000

enum plock_use_type {
	USE_MB,
	USE_SMP_MB,
	USE_NUM
};

struct plock {
	volatile uint32_t flag[2];
	volatile uint32_t victim;
	enum plock_use_type utype;
};

struct plock_test {
	struct plock lock;
	uint32_t val;
	uint32_t iter;
};

struct lcore_plock_test {
	struct plock_test *pt[2];
	uint32_t sum[2];
	uint32_t iter;
	uint32_t lc;
};

static inline void
store_load_barrier(uint32_t utype)
{
	if (utype == USE_MB)
		rte_mb();
	else if (utype == USE_SMP_MB)
		rte_smp_mb();
	else
		RTE_VERIFY(0);
}

static void
plock_lock(struct plock *l, uint32_t self)
{
	uint32_t other;

	other = self ^ 1;

	l->flag[self] = 1;
	l->victim = self;

	store_load_barrier(l->utype);

	while (l->flag[other] == 1 && l->victim == self)
		rte_pause();
}

static void
plock_unlock(struct plock *l, uint32_t self)
{
	rte_smp_wmb();
	l->flag[self] = 0;
}

static void
plock_reset(struct plock *l, enum plock_use_type utype)
{
	memset(l, 0, sizeof(*l));
	l->utype = utype;
}

static void
plock_add(struct plock_test *pt, uint32_t self, uint32_t n)
{
	plock_lock(&pt->lock, self);
	pt->iter++;
	pt->val += n;
	plock_unlock(&pt->lock, self);
}

static int
plock_test1_lcore(void *data)
{
	uint64_t tm;
	uint32_t i, lc, ln, n;
	struct lcore_plock_test *lpt;

	lpt = data;
	lc = rte_lcore_id();

	for (ln = rte_lcore_count(); ln != 0 && lpt->lc != lc; lpt++, ln--)
		;

	if (ln == 0) {
		printf("%s(%u) error at init\n", __func__, lc);
		return -1;
	}

	n = rte_rand() % ADD_MAX;
	tm = rte_get_timer_cycles();

	for (i = 0; i != lpt->iter; i++) {

		plock_add(lpt->pt[0], 0, n);
		plock_add(lpt->pt[1], 1, n);

		lpt->sum[0] += n;
		lpt->sum[1] += n;

		n = (n + 1) % ADD_MAX;
	}

	tm = rte_get_timer_cycles() - tm;

	printf("%s(%u): %u iterations finished, in %" PRIu64
		" cycles, %#Lf cycles/iteration, "
		"local sum={%u, %u}\n",
		__func__, lc, i, tm, (long double)tm / i,
		lpt->sum[0], lpt->sum[1]);
	return 0;
}

static int
plock_test(uint32_t iter, enum plock_use_type utype)
{
	int32_t rc;
	uint32_t i, lc, n;
	uint32_t *sum;
	struct plock_test *pt;
	struct lcore_plock_test *lpt;

	n = rte_lcore_count();
	pt = calloc(n + 1, sizeof(*pt));
	lpt = calloc(n, sizeof(*lpt));
	sum = calloc(n + 1, sizeof(*sum));

	printf("%s(iter=%u, utype=%u) started on %u lcores\n",
		__func__, iter, utype, n);

	if (pt == NULL || lpt == NULL) {
		printf("%s: failed to allocate memory for %u lcores\n",
			__func__, n);
		free(pt);
		free(lpt);
		free(sum);
		return -ENOMEM;
	}

	for (i = 0; i != n + 1; i++)
		plock_reset(&pt[i].lock, utype);

	i = 0;
	RTE_LCORE_FOREACH(lc) {

		lpt[i].lc = lc;
		lpt[i].iter = iter;
		lpt[i].pt[0] = pt + i;
		lpt[i].pt[1] = pt + i + 1;
		i++;
	}

	lpt[i - 1].pt[1] = pt;

	for (i = 0; i != n; i++)
		printf("lpt[%u]={lc=%u, pt={%p, %p},};\n",
			i, lpt[i].lc, lpt[i].pt[0], lpt[i].pt[1]);


	rte_eal_mp_remote_launch(plock_test1_lcore, lpt, CALL_MASTER);
	rte_eal_mp_wait_lcore();

	for (i = 0; i != n; i++) {
		sum[i] += lpt[i].sum[0];
		sum[i + 1] += lpt[i].sum[1];
	}

	sum[0] += sum[i];

	rc = 0;
	for (i = 0; i != n; i++) {
		printf("%s: sum[%u]=%u, pt[%u].val=%u, pt[%u].iter=%u;\n",
			__func__, i, sum[i], i, pt[i].val, i, pt[i].iter);
		if (sum[i] != pt[i].val || 2 * iter != pt[i].iter) {
			printf("error: local and shared sums don't much\n");
			rc = -1;
		}
	}

	free(pt);
	free(lpt);
	free(sum);

	printf("%s(utype=%u) returns %d\n", __func__, utype, rc);
	return rc;
}

static int
test_mb(void)
{
	int32_t i, ret, rc[USE_NUM];

	for (i = 0; i != RTE_DIM(rc); i++)
		rc[i] = plock_test(ITER_MAX, i);

	ret = 0;
	for (i = 0; i != RTE_DIM(rc); i++) {
		printf("%s for utype=%d %s\n",
			__func__, i, rc[i] == 0 ? "passed" : "failed");
		ret |= rc[i];
	}

	return ret;
}

REGISTER_TEST_COMMAND(mb_autotest, test_mb);
