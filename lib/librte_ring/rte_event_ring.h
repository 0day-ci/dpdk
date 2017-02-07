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

#ifndef _RTE_EVENT_RING_H_
#define _RTE_EVENT_RING_H_

/**
 * @file
 * RTE Event Ring
 *
 * The Ring Manager is a fixed-size event queue, implemented as a table of
 * events. Head and tail pointers are modified atomically, allowing
 * concurrent access to it. It has the following features:
 *
 * - FIFO (First In First Out)
 * - Maximum size is fixed; the pointers are stored in a table.
 * - Lockless implementation.
 * - Multi- or single-consumer dequeue.
 * - Multi- or single-producer enqueue.
 * - Bulk dequeue.
 * - Bulk enqueue.
 *
 * Note: the ring implementation is not preemptable. A lcore must not
 * be interrupted by another task that uses the same ring.
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "rte_ring.h"

struct rte_event {
	uint64_t event_metadata;
	struct rte_mbuf *mbuf;
};

struct rte_event_ring {
	struct rte_ring hdr;
};

/**
 * Initialize a ring structure.
 *
 * Initialize a ring structure in memory pointed by "r". The size of the
 * memory area must be large enough to store the ring structure and the
 * object table. It is advised to use rte_event_ring_get_memsize() to get the
 * appropriate size.
 *
 * The ring size is set to *count*, which must be a power of two. Water
 * marking is disabled by default. The real usable ring size is
 * *count-1* instead of *count* to differentiate a free ring from an
 * empty ring.
 *
 * The ring is not added in RTE_TAILQ_RING global list. Indeed, the
 * memory given by the caller may not be shareable among dpdk
 * processes.
 *
 * @param r
 *   The pointer to the ring structure followed by the objects table.
 * @param name
 *   The name of the ring.
 * @param count
 *   The number of elements in the ring (must be a power of 2).
 * @param flags
 *   An OR of the following:
 *    - RING_F_SP_ENQ: If this flag is set, the default behavior when
 *      using ``rte_event_ring_enqueue()`` or ``rte_event_ring_enqueue_bulk()``
 *      is "single-producer". Otherwise, it is "multi-producers".
 *    - RING_F_SC_DEQ: If this flag is set, the default behavior when
 *      using ``rte_event_ring_dequeue()`` or ``rte_event_ring_dequeue_bulk()``
 *      is "single-consumer". Otherwise, it is "multi-consumers".
 * @return
 *   0 on success, or a negative value on error.
 */
int rte_event_ring_init(struct rte_event_ring *r, const char *name,
		unsigned int count, unsigned int flags);

/**
 * Create a new ring named *name* in memory.
 *
 * This function uses ``memzone_reserve()`` to allocate memory. Then it
 * calls rte_event_ring_init() to initialize an empty ring.
 *
 * The new ring size is set to *count*, which must be a power of
 * two. Water marking is disabled by default. The real usable ring size
 * is *count-1* instead of *count* to differentiate a free ring from an
 * empty ring.
 *
 * The ring is added in RTE_TAILQ_RING list.
 *
 * @param name
 *   The name of the ring.
 * @param count
 *   The size of the ring (must be a power of 2).
 * @param socket_id
 *   The *socket_id* argument is the socket identifier in case of
 *   NUMA. The value can be *SOCKET_ID_ANY* if there is no NUMA
 *   constraint for the reserved zone.
 * @param flags
 *   An OR of the following:
 *    - RING_F_SP_ENQ: If this flag is set, the default behavior when
 *      using ``rte_event_ring_enqueue()`` or ``rte_event_ring_enqueue_bulk()``
 *      is "single-producer". Otherwise, it is "multi-producers".
 *    - RING_F_SC_DEQ: If this flag is set, the default behavior when
 *      using ``rte_event_ring_dequeue()`` or ``rte_event_ring_dequeue_bulk()``
 *      is "single-consumer". Otherwise, it is "multi-consumers".
 * @return
 *   On success, the pointer to the new allocated ring. NULL on error with
 *    rte_errno set appropriately. Possible errno values include:
 *    - E_RTE_NO_CONFIG - function could not get pointer to rte_config structure
 *    - E_RTE_SECONDARY - function was called from a secondary process instance
 *    - EINVAL - count provided is not a power of 2
 *    - ENOSPC - the maximum number of memzones has already been allocated
 *    - EEXIST - a memzone with the same name already exists
 *    - ENOMEM - no appropriate memory area found in which to create memzone
 */
struct rte_event_ring *rte_event_ring_create(const char *name, unsigned count,
				 int socket_id, unsigned flags);
/**
 * De-allocate all memory used by the ring.
 *
 * @param r
 *   Ring to free
 */
void rte_event_ring_free(struct rte_event_ring *r);

/**
 * Dump the status of the ring to a file.
 *
 * @param f
 *   A pointer to a file for output
 * @param r
 *   A pointer to the ring structure.
 */
void rte_event_ring_dump(FILE *f, const struct rte_event_ring *r);

/**
 * Test if a ring is full.
 *
 * @param r
 *   A pointer to the ring structure.
 * @return
 *   - 1: The ring is full.
 *   - 0: The ring is not full.
 */
static inline int
rte_event_ring_full(const struct rte_event_ring *r)
{
	return rte_ring_full(&r->hdr);
}

/**
 * Test if a ring is empty.
 *
 * @param r
 *   A pointer to the ring structure.
 * @return
 *   - 1: The ring is empty.
 *   - 0: The ring is not empty.
 */
static inline int
rte_event_ring_empty(const struct rte_event_ring *r)
{
	return rte_ring_empty(&r->hdr);
}

/**
 * Return the number of entries in a ring.
 *
 * @param r
 *   A pointer to the ring structure.
 * @return
 *   The number of entries in the ring.
 */
static inline unsigned
rte_event_ring_count(const struct rte_event_ring *r)
{
	return rte_ring_count(&r->hdr);
}

/**
 * Return the number of free entries in a ring.
 *
 * @param r
 *   A pointer to the ring structure.
 * @return
 *   The number of free entries in the ring.
 */
static inline unsigned
rte_event_ring_free_count(const struct rte_event_ring *r)
{
	return rte_ring_free_count(&r->hdr);
}

/**
 * Dump the status of all rings on the console
 *
 * @param f
 *   A pointer to a file for output
 */
void rte_event_ring_list_dump(FILE *f);

/**
 * Search a ring from its name
 *
 * @param name
 *   The name of the ring.
 * @return
 *   The pointer to the ring matching the name, or NULL if not found,
 *   with rte_errno set appropriately. Possible rte_errno values include:
 *    - ENOENT - required entry not available to return.
 */
struct rte_event_ring *rte_event_ring_lookup(const char *name);


static inline __attribute__((always_inline)) unsigned int
__rte_event_ring_do_enqueue(struct rte_event_ring *r,
		const struct rte_event *obj_table, unsigned int n,
		enum rte_ring_queue_behavior behavior, int is_sp,
		unsigned int *free_space)
{
	uint32_t prod_head, prod_next;
	uint32_t free_entries;

	n = __rte_ring_move_prod_head(&r->hdr, is_sp, n, behavior,
			&prod_head, &prod_next, &free_entries);
	if (n == 0)
		goto end;

	ENQUEUE_PTRS(&r->hdr, obj_table, n, prod_head, struct rte_event);
	rte_smp_wmb();

	update_tail(&r->hdr.prod, prod_head, prod_next);
end:
	if (free_space != NULL)
		*free_space = free_entries - n;
	return n;
}

static inline __attribute__((always_inline)) unsigned int
__rte_event_ring_do_dequeue(struct rte_event_ring *r,
		struct rte_event *obj_table, unsigned int n,
		enum rte_ring_queue_behavior behavior,
		int is_sc, unsigned int *available)
{
	uint32_t cons_head, cons_next;
	uint32_t entries;

	n = __rte_ring_move_cons_head(&r->hdr, is_sc, n, behavior,
			&cons_head, &cons_next, &entries);
	if (n == 0)
		goto end;

	DEQUEUE_PTRS(&r->hdr, obj_table, n, cons_head, struct rte_event);
	rte_smp_rmb();

	update_tail(&r->hdr.cons, cons_head, cons_next);

end:
	if (available != NULL)
		*available = entries - n;
	return n;
}

/**
 * Enqueue several objects on the ring (multi-producers safe).
 *
 * This function uses a "compare and set" instruction to move the
 * producer index atomically.
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @param free_space
 *   Returns the amount of free space in the ring after the enqueue call
 * @return
 *   - n: Actual number of objects enqueued.
 */
static inline unsigned __attribute__((always_inline))
rte_event_ring_mp_enqueue_burst(struct rte_event_ring *r,
		const struct rte_event * const obj_table, unsigned int n,
		unsigned int *free_space)
{
	return __rte_event_ring_do_enqueue(r, obj_table, n,
			RTE_RING_QUEUE_VARIABLE, __IS_MP, free_space);
}

/**
 * Enqueue several objects on a ring (NOT multi-producers safe).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @param free_space
 *   Returns the amount of free space in the ring after the enqueue call
 * @return
 *   - n: Actual number of objects enqueued.
 */
static inline unsigned __attribute__((always_inline))
rte_event_ring_sp_enqueue_burst(struct rte_event_ring *r,
		const struct rte_event * const obj_table, unsigned int n,
		unsigned int *free_space)
{
	return __rte_event_ring_do_enqueue(r, obj_table, n,
			RTE_RING_QUEUE_VARIABLE, __IS_SP, free_space);
}

/**
 * Enqueue several objects on a ring.
 *
 * This function calls the multi-producer or the single-producer
 * version depending on the default behavior that was specified at
 * ring creation time (see flags).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @return
 *   - n: Actual number of objects enqueued.
 */
static inline unsigned __attribute__((always_inline))
rte_event_ring_enqueue_burst(struct rte_event_ring *r,
		const struct rte_event * const obj_table, unsigned int n,
		unsigned int *free_space)
{
	return __rte_event_ring_do_enqueue(r, obj_table, n,
			RTE_RING_QUEUE_VARIABLE, r->hdr.prod.sp_enqueue,
			free_space);
}

/**
 * Enqueue all objects on a ring, or enqueue none.
 *
 * This function calls the multi-producer or the single-producer
 * version depending on the default behavior that was specified at
 * ring creation time (see flags).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects).
 * @param n
 *   The number of objects to add in the ring from the obj_table.
 * @return
 *   - n: All objects enqueued.
 *   - 0: No objects enqueued
 */
static inline unsigned __attribute__((always_inline))
rte_event_ring_enqueue_bulk(struct rte_event_ring *r,
		const struct rte_event * const obj_table, unsigned int n,
		unsigned int *free_space)
{
	return __rte_event_ring_do_enqueue(r, obj_table, n,
			RTE_RING_QUEUE_FIXED, r->hdr.prod.sp_enqueue,
			free_space);
}

/**
 * Dequeue several objects from a ring (multi-consumers safe). When the request
 * objects are more than the available objects, only dequeue the actual number
 * of objects
 *
 * This function uses a "compare and set" instruction to move the
 * consumer index atomically.
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table.
 * @return
 *   - n: Actual number of objects dequeued, 0 if ring is empty
 */
static inline unsigned __attribute__((always_inline))
rte_event_ring_mc_dequeue_burst(struct rte_event_ring *r,
		struct rte_event * const obj_table, unsigned int n,
		unsigned int *available)
{
	return __rte_event_ring_do_dequeue(r, obj_table, n,
			RTE_RING_QUEUE_VARIABLE, __IS_MC, available);
}

/**
 * Dequeue several objects from a ring (NOT multi-consumers safe).When the
 * request objects are more than the available objects, only dequeue the
 * actual number of objects
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table.
 * @return
 *   - n: Actual number of objects dequeued, 0 if ring is empty
 */
static inline unsigned __attribute__((always_inline))
rte_event_ring_sc_dequeue_burst(struct rte_event_ring *r,
		struct rte_event * const obj_table, unsigned int n,
		unsigned int *available)
{
	return __rte_event_ring_do_dequeue(r, obj_table, n,
			RTE_RING_QUEUE_VARIABLE, __IS_SC, available);
}

/**
 * Dequeue multiple objects from a ring up to a maximum number.
 *
 * This function calls the multi-consumers or the single-consumer
 * version, depending on the default behaviour that was specified at
 * ring creation time (see flags).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table.
 * @return
 *   - Number of objects dequeued
 */
static inline unsigned __attribute__((always_inline))
rte_event_ring_dequeue_burst(struct rte_event_ring *r,
		struct rte_event * const obj_table, unsigned int n,
		unsigned int *available)
{
	return __rte_event_ring_do_dequeue(r, obj_table, n,
			RTE_RING_QUEUE_VARIABLE, r->hdr.cons.sc_dequeue,
			available);
}

/**
 * Dequeue all requested objects from a ring, or dequeue none .
 *
 * This function calls the multi-consumers or the single-consumer
 * version, depending on the default behaviour that was specified at
 * ring creation time (see flags).
 *
 * @param r
 *   A pointer to the ring structure.
 * @param obj_table
 *   A pointer to a table of void * pointers (objects) that will be filled.
 * @param n
 *   The number of objects to dequeue from the ring to the obj_table.
 * @return
 *   - n: all objects dequeued
 *   - 0: nothing dequeued
 */
static inline unsigned __attribute__((always_inline))
rte_event_ring_dequeue_bulk(struct rte_event_ring *r,
		struct rte_event * const obj_table, unsigned int n,
		unsigned int *available)
{
	return __rte_event_ring_do_dequeue(r, obj_table, n,
			RTE_RING_QUEUE_FIXED, r->hdr.cons.sc_dequeue,
			available);
}



#ifdef __cplusplus
}
#endif

#endif /* _RTE_EVENT_RING_H_ */
