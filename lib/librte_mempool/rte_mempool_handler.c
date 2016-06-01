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
struct rte_mempool_handler_table rte_mempool_handler_table = {
	.sl =  RTE_SPINLOCK_INITIALIZER ,
	.num_handlers = 0
};

/* add a new handler in rte_mempool_handler_table, return its index */
int
rte_mempool_handler_register(const struct rte_mempool_ops *h)
{
	struct rte_mempool_ops *handler;
	int16_t handler_idx;

	rte_spinlock_lock(&rte_mempool_handler_table.sl);

	if (rte_mempool_handler_table.num_handlers >=
			RTE_MEMPOOL_MAX_HANDLER_IDX) {
		rte_spinlock_unlock(&rte_mempool_handler_table.sl);
		RTE_LOG(ERR, MEMPOOL,
			"Maximum number of mempool handlers exceeded\n");
		return -ENOSPC;
	}

	if (h->put == NULL || h->get == NULL || h->get_count == NULL) {
		rte_spinlock_unlock(&rte_mempool_handler_table.sl);
		RTE_LOG(ERR, MEMPOOL,
			"Missing callback while registering mempool handler\n");
		return -EINVAL;
	}

	handler_idx = rte_mempool_handler_table.num_handlers++;
	handler = &rte_mempool_handler_table.handler_ops[handler_idx];
	snprintf(handler->name, sizeof(handler->name), "%s", h->name);
	handler->alloc = h->alloc;
	handler->put = h->put;
	handler->get = h->get;
	handler->get_count = h->get_count;

	rte_spinlock_unlock(&rte_mempool_handler_table.sl);

	return handler_idx;
}

/* wrapper to allocate an external pool handler */
void *
rte_mempool_ops_alloc(struct rte_mempool *mp)
{
	struct rte_mempool_ops *handler;

	handler = rte_mempool_handler_get(mp->handler_idx);
	if (handler->alloc == NULL)
		return NULL;
	return handler->alloc(mp);
}

/* wrapper to free an external pool handler */
void
rte_mempool_ops_free(struct rte_mempool *mp)
{
	struct rte_mempool_ops *handler;

	handler = rte_mempool_handler_get(mp->handler_idx);
	if (handler->free == NULL)
		return;
	return handler->free(mp);
}

/* wrapper to get available objects in an external pool handler */
unsigned int
rte_mempool_ops_get_count(const struct rte_mempool *mp)
{
	struct rte_mempool_ops *handler;

	handler = rte_mempool_handler_get(mp->handler_idx);
	return handler->get_count(mp->pool);
}

/* sets a handler previously registered by rte_mempool_handler_register */
int
rte_mempool_set_handler(struct rte_mempool *mp, const char *name)
{
	struct rte_mempool_ops *handler = NULL;
	unsigned i;

	/* too late, the mempool is already populated */
	if (mp->flags & MEMPOOL_F_RING_CREATED)
		return -EEXIST;

	for (i = 0; i < rte_mempool_handler_table.num_handlers; i++) {
		if (!strcmp(name,
				rte_mempool_handler_table.handler_ops[i].name)) {
			handler = &rte_mempool_handler_table.handler_ops[i];
			break;
		}
	}

	if (handler == NULL)
		return -EINVAL;

	mp->handler_idx = i;
	return 0;
}
