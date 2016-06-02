/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Intel Corporation. All rights reserved.
 *   Copyright(c) 2016 6WIND S.A.
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
#include <string.h>

#include <rte_mempool.h>

/* indirect jump table to support external memory pools */
struct rte_mempool_ops_table rte_mempool_ops_table = {
	.sl =  RTE_SPINLOCK_INITIALIZER ,
	.num_ops = 0
};

/* add a new ops struct in rte_mempool_ops_table, return its index */
int
rte_mempool_ops_register(const struct rte_mempool_ops *h)
{
	struct rte_mempool_ops *ops;
	int16_t ops_index;

	rte_spinlock_lock(&rte_mempool_ops_table.sl);

	if (rte_mempool_ops_table.num_ops >=
			RTE_MEMPOOL_MAX_OPS_IDX) {
		rte_spinlock_unlock(&rte_mempool_ops_table.sl);
		RTE_LOG(ERR, MEMPOOL,
			"Maximum number of mempool ops structs exceeded\n");
		return -ENOSPC;
	}

	if (h->put == NULL || h->get == NULL || h->get_count == NULL) {
		rte_spinlock_unlock(&rte_mempool_ops_table.sl);
		RTE_LOG(ERR, MEMPOOL,
			"Missing callback while registering mempool ops\n");
		return -EINVAL;
	}

	ops_index = rte_mempool_ops_table.num_ops++;
	ops = &rte_mempool_ops_table.ops[ops_index];
	snprintf(ops->name, sizeof(ops->name), "%s", h->name);
	ops->alloc = h->alloc;
	ops->put = h->put;
	ops->get = h->get;
	ops->get_count = h->get_count;

	rte_spinlock_unlock(&rte_mempool_ops_table.sl);

	return ops_index;
}

/* wrapper to allocate an external mempool's private (pool) data */
void *
rte_mempool_ops_alloc(const struct rte_mempool *mp)
{
	struct rte_mempool_ops *ops;

	ops = rte_mempool_ops_get(mp->ops_index);
	if (ops->alloc == NULL)
		return NULL;
	return ops->alloc(mp);
}

/* wrapper to free an external pool ops */
void
rte_mempool_ops_free(struct rte_mempool *mp)
{
	struct rte_mempool_ops *ops;

	ops = rte_mempool_ops_get(mp->ops_index);
	if (ops->free == NULL)
		return;
	return ops->free(mp);
}

/* wrapper to get available objects in an external mempool */
unsigned int
rte_mempool_ops_get_count(const struct rte_mempool *mp)
{
	struct rte_mempool_ops *ops;

	ops = rte_mempool_ops_get(mp->ops_index);
	return ops->get_count(mp->pool_data);
}

/* sets mempool ops previously registered by rte_mempool_ops_register */
int
rte_mempool_set_ops_byname(struct rte_mempool *mp, const char *name)
{
	struct rte_mempool_ops *ops = NULL;
	unsigned i;

	/* too late, the mempool is already populated */
	if (mp->flags & MEMPOOL_F_RING_CREATED)
		return -EEXIST;

	for (i = 0; i < rte_mempool_ops_table.num_ops; i++) {
		if (!strcmp(name,
				rte_mempool_ops_table.ops[i].name)) {
			ops = &rte_mempool_ops_table.ops[i];
			break;
		}
	}

	if (ops == NULL)
		return -EINVAL;

	mp->ops_index = i;
	return 0;
}
