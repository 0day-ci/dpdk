/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Intel Corporation. All rights reserved.
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
#include <sys/queue.h>
#include <string.h>
#include <rte_mbuf.h>
#include <rte_memory.h>
#include <rte_cycles.h>
#include <rte_memzone.h>
#include <rte_errno.h>
#include <rte_string_fns.h>
#include <rte_eal_memconfig.h>
#include "rte_distributor_common.h"
#include "rte_distributor_burst.h"
#include "smmintrin.h"

/**
 * Number of packets to deal with in bursts. Needs to be 8 so as to
 * fit in one cache line.
 */
#define RTE_DIST_BURST_SIZE (sizeof(__m128i) / sizeof(uint16_t))

/**
 * Buffer structure used to pass the pointer data between cores. This is cache
 * line aligned, but to improve performance and prevent adjacent cache-line
 * prefetches of buffers for other workers, e.g. when worker 1's buffer is on
 * the next cache line to worker 0, we pad this out to two cache lines.
 * We can pass up to 8 mbufs at a time in one cacheline.
 * There is a separate cacheline for returns.
 */
struct rte_distributor_buffer {
	volatile int64_t bufptr64[RTE_DIST_BURST_SIZE]
			__rte_cache_aligned; /* <= outgoing to worker */

	int64_t pad1 __rte_cache_aligned;    /* <= one cache line  */

	volatile int64_t retptr64[RTE_DIST_BURST_SIZE]
			__rte_cache_aligned; /* <= incoming from worker */

	int64_t pad2 __rte_cache_aligned;    /* <= one cache line  */

	int count __rte_cache_aligned;       /* <= number of current mbufs */
};

struct rte_distributor_backlog {
	unsigned int start;
	unsigned int count;
	int64_t pkts[RTE_DIST_BURST_SIZE] __rte_cache_aligned;
	uint16_t *tags; /* will point to second cacheline of inflights */
} __rte_cache_aligned;

struct rte_distributor_returned_pkts {
	unsigned int start;
	unsigned int count;
	struct rte_mbuf *mbufs[RTE_DISTRIB_MAX_RETURNS];
};

struct rte_distributor {
	TAILQ_ENTRY(rte_distributor) next;    /**< Next in list. */

	char name[RTE_DISTRIBUTOR_NAMESIZE];  /**< Name of the ring. */
	unsigned int num_workers;             /**< Number of workers polling */

	/**>
	 * First cache line in the this array are the tags inflight
	 * on the worker core. Second cache line are the backlog
	 * that are going to go to the worker core.
	 */
	uint16_t in_flight_tags[RTE_DISTRIB_MAX_WORKERS][RTE_DIST_BURST_SIZE*2]
			__rte_cache_aligned;

	struct rte_distributor_backlog backlog[RTE_DISTRIB_MAX_WORKERS]
			__rte_cache_aligned;

	struct rte_distributor_buffer bufs[RTE_DISTRIB_MAX_WORKERS];

	struct rte_distributor_returned_pkts returns;
};

TAILQ_HEAD(rte_distributor_list, rte_distributor);

static struct rte_tailq_elem rte_distributor_tailq = {
	.name = "RTE_DISTRIBUTOR",
};
EAL_REGISTER_TAILQ(rte_distributor_tailq)

/**** APIs called by workers ****/

/**** Burst Packet APIs called by workers ****/

/* This function should really be called return_pkt_burst() */
void
rte_distributor_request_pkt_burst(struct rte_distributor *d,
		unsigned int worker_id, struct rte_mbuf **oldpkt,
		unsigned int count)
{
	struct rte_distributor_buffer *buf = &(d->bufs[worker_id]);
	unsigned int i;

	volatile int64_t *retptr64;

	retptr64 = &(buf->retptr64[0]);
	/* Spin while handshake bits are set (scheduler clears it) */
	while (unlikely(*retptr64 & RTE_DISTRIB_GET_BUF)) {
		rte_pause();
		uint64_t t = __rdtsc()+100;

		while (__rdtsc() < t)
			rte_pause();
	}

	/*
	 * OK, if we've got here, then the scheduler has just cleared the
	 * handshake bits. Populate the retptrs with returning packets.
	 */

	for (i = count; i < RTE_DIST_BURST_SIZE; i++)
		buf->retptr64[i] = 0;

	/* Set Return bit for each packet returned */
	for (i = count; i-- > 0; )
		buf->retptr64[i] =
			(((int64_t)(uintptr_t)(oldpkt[i])) <<
			RTE_DISTRIB_FLAG_BITS) | RTE_DISTRIB_RETURN_BUF;

	/*
	 * Finally, set the GET_BUF  to signal to distributor that cache
	 * line is ready for processing
	 */
	*retptr64 |= RTE_DISTRIB_GET_BUF;
}

int
rte_distributor_poll_pkt_burst(struct rte_distributor *d,
		unsigned int worker_id, struct rte_mbuf **pkts)
{
	struct rte_distributor_buffer *buf = &d->bufs[worker_id];
	uint64_t ret;
	int count = 0;

	/* If bit is set, return */
	if (buf->bufptr64[0] & RTE_DISTRIB_GET_BUF)
		return 0;

	/* since bufptr64 is signed, this should be an arithmetic shift */
	for (unsigned int i = 0; i < RTE_DIST_BURST_SIZE; i++) {
		if (likely(buf->bufptr64[i] & RTE_DISTRIB_VALID_BUF)) {
			ret = buf->bufptr64[i] >> RTE_DISTRIB_FLAG_BITS;
			pkts[count++] = (struct rte_mbuf *)((uintptr_t)(ret));
		}
	}

	/*
	 * so now we've got the contents of the cacheline into an  array of
	 * mbuf pointers, so toggle the bit so scheduler can start working
	 * on the next cacheline while we're working.
	 */
	buf->bufptr64[0] |= RTE_DISTRIB_GET_BUF;


	return count;
}

int
rte_distributor_get_pkt_burst(struct rte_distributor *d,
		unsigned int worker_id, struct rte_mbuf **pkts,
		struct rte_mbuf **oldpkt, unsigned int return_count)
{
	unsigned int count;
	uint64_t retries = 0;

	rte_distributor_request_pkt_burst(d, worker_id, oldpkt, return_count);

	count = rte_distributor_poll_pkt_burst(d, worker_id, pkts);
	while (count == 0) {
		rte_pause();
		retries++;
		if (retries > 1000) {
			retries = 0;
			return 0;
		}
		uint64_t t = __rdtsc()+100;

		while (__rdtsc() < t)
			rte_pause();

		count = rte_distributor_poll_pkt_burst(d, worker_id, pkts);
	}
	return count;
}

int
rte_distributor_return_pkt_burst(struct rte_distributor *d,
		unsigned int worker_id, struct rte_mbuf **oldpkt, int num)
{
	struct rte_distributor_buffer *buf = &d->bufs[worker_id];
	unsigned int i;

	for (i = 0; i < RTE_DIST_BURST_SIZE; i++)
		/* Switch off the return bit first */
		buf->retptr64[i] &= ~RTE_DISTRIB_RETURN_BUF;

	for (i = num; i-- > 0; )
		buf->retptr64[i] = (((int64_t)(uintptr_t)oldpkt[i]) <<
			RTE_DISTRIB_FLAG_BITS) | RTE_DISTRIB_RETURN_BUF;

	/* set the GET_BUF but even if we got no returns */
	buf->retptr64[0] |= RTE_DISTRIB_GET_BUF;

	return 0;
}

/**** APIs called on distributor core ***/

/* stores a packet returned from a worker inside the returns array */
static inline void
store_return(uintptr_t oldbuf, struct rte_distributor *d,
		unsigned int *ret_start, unsigned int *ret_count)
{
	if (!oldbuf)
		return;
	/* store returns in a circular buffer */
	d->returns.mbufs[(*ret_start + *ret_count) & RTE_DISTRIB_RETURNS_MASK]
			= (void *)oldbuf;
	*ret_start += (*ret_count == RTE_DISTRIB_RETURNS_MASK);
	*ret_count += (*ret_count != RTE_DISTRIB_RETURNS_MASK);
}


static void
find_match(struct rte_distributor *d,
			uint16_t *data_ptr,
			uint16_t *output_ptr)
{
	/* Setup */
	__m128i incoming_fids;
	__m128i inflight_fids;
	__m128i preflight_fids;
	__m128i wkr;
	__m128i mask1;
	__m128i mask2;
	__m128i output;
	struct rte_distributor_backlog *bl;

	/*
	 * Function overview:
	 * 2. Loop through all worker ID's
	 *  2a. Load the current inflights for that worker into an xmm reg
	 *  2b. Load the current backlog for that worker into an xmm reg
	 *  2c. use cmpestrm to intersect flow_ids with backlog and inflights
	 *  2d. Add any matches to the output
	 * 3. Write the output xmm (matching worker ids).
	 */


	output = _mm_set1_epi16(0);
	incoming_fids = _mm_load_si128((__m128i *)data_ptr);

	for (uint16_t i = 0; i < d->num_workers; i++) {
		bl = &d->backlog[i];

		inflight_fids =
			_mm_load_si128((__m128i *)&(d->in_flight_tags[i]));
		preflight_fids =
			_mm_load_si128((__m128i *)(bl->tags));

		/*
		 * Any incoming_fid that exists anywhere in inflight_fids will
		 * have 0xffff in same position of the mask as the incoming fid
		 * Example (shortened to bytes for brevity):
		 * incoming_fids   0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08
		 * inflight_fids   0x03 0x05 0x07 0x00 0x00 0x00 0x00 0x00
		 * mask            0x00 0x00 0xff 0x00 0xff 0x00 0xff 0x00
		 */

		mask1 = _mm_cmpestrm(inflight_fids, 8, incoming_fids, 8,
			_SIDD_UWORD_OPS |
			_SIDD_CMP_EQUAL_ANY |
			_SIDD_UNIT_MASK);
		mask2 = _mm_cmpestrm(preflight_fids, 8, incoming_fids, 8,
			_SIDD_UWORD_OPS |
			_SIDD_CMP_EQUAL_ANY |
			_SIDD_UNIT_MASK);

		mask1 = _mm_or_si128(mask1, mask2);
		 /*
		 * Now mask contains 0xffff where there's a match.
		 * Next we need to store the worker_id in the relevant position
		 * in the output.
		 */

		wkr = _mm_set1_epi16(i+1);
		mask1 = _mm_and_si128(mask1, wkr);
		output = _mm_or_si128(mask1, output);
	}

	/*
	 * At this stage, the output 128-bit contains 8 16-bit values, with
	 * each non-zero value containing the worker ID on which the
	 * corresponding flow is pinned to.
	 */
	_mm_store_si128((__m128i *)output_ptr, output);
}


static unsigned int
release(struct rte_distributor *d, unsigned int wkr)
{
	uintptr_t oldbuf;
	unsigned int ret_start = d->returns.start,
			ret_count = d->returns.count;
	struct rte_distributor_buffer *buf = &(d->bufs[wkr]);
	unsigned int i;

	if (d->backlog[wkr].count == 0)
		return 0;

	/*
	 * wait for the GET_BUF bit to go high, otherwise we can't send
	 * the packets to the worker
	 */
	while (!(d->bufs[wkr].bufptr64[0] & RTE_DISTRIB_GET_BUF))
		rte_pause();

	if (buf->retptr64[0] & RTE_DISTRIB_GET_BUF) {
		for (unsigned int i = 0; i < RTE_DIST_BURST_SIZE; i++) {
			if (buf->retptr64[i] & RTE_DISTRIB_RETURN_BUF) {
				oldbuf = ((uintptr_t)(buf->retptr64[i] >>
					RTE_DISTRIB_FLAG_BITS));
				/* store returns in a circular buffer */
				store_return(oldbuf, d, &ret_start, &ret_count);
				buf->retptr64[i] &= ~RTE_DISTRIB_RETURN_BUF;
			}
		}
		d->returns.start = ret_start;
		d->returns.count = ret_count;
		/* Clear for the worker to populate with more returns */
		buf->retptr64[0] = 0;
	}

	buf->count = 0;

	for (i = 0; i < d->backlog[wkr].count; i++) {
		d->bufs[wkr].bufptr64[i] = d->backlog[wkr].pkts[i] |
				RTE_DISTRIB_GET_BUF | RTE_DISTRIB_VALID_BUF;
		d->in_flight_tags[wkr][i] = d->backlog[wkr].tags[i];
	}
	buf->count = i;
	for ( ; i < RTE_DIST_BURST_SIZE ; i++) {
		buf->bufptr64[i] = RTE_DISTRIB_GET_BUF;
		d->in_flight_tags[wkr][i] = 0;
	}
	d->backlog[wkr].count = 0;

	/* Clear the GET bit */
	buf->bufptr64[0] &= ~RTE_DISTRIB_GET_BUF;
	return  buf->count;

}


/* process a set of packets to distribute them to workers */
int
rte_distributor_process_burst(struct rte_distributor *d,
		struct rte_mbuf **mbufs, unsigned int num_mbufs)
{
	unsigned int next_idx = 0;
	static unsigned int wkr;
	struct rte_mbuf *next_mb = NULL;
	int64_t next_value = 0;
	uint16_t new_tag = 0;
	uint16_t flows[8] __rte_cache_aligned;

	if (unlikely(num_mbufs == 0)) {
		/* Flush out all non-full cache-lines to workers. */
		for (unsigned int wid = 0 ; wid < d->num_workers; wid++) {
			if ((d->bufs[wid].bufptr64[0] & RTE_DISTRIB_GET_BUF))
				release(d, wid);
		}
		return 0;
	}

	while (next_idx < num_mbufs) {
		uint16_t matches[8];
		int pkts;

		if (d->bufs[wkr].bufptr64[0] & RTE_DISTRIB_GET_BUF)
			d->bufs[wkr].count = 0;

		for (unsigned int i = 0; i < RTE_DIST_BURST_SIZE; i++) {
			if (mbufs[next_idx + i]) {
				/* flows have to be non-zero */
				flows[i] = mbufs[next_idx + i]->hash.usr | 1;
			} else
				flows[i] = 0;
		}

		find_match(d, &flows[0], &matches[0]);

		/*
		 * Matches array now contain the intended worker ID (+1) of
		 * the incoming packets. Any zeroes need to be assigned
		 * workers.
		 */

		if ((num_mbufs - next_idx) < RTE_DIST_BURST_SIZE)
			pkts = num_mbufs - next_idx;
		else
			pkts = RTE_DIST_BURST_SIZE;

		for (int j = 0; j < pkts; j++) {

			next_mb = mbufs[next_idx++];
			next_value = (((int64_t)(uintptr_t)next_mb) <<
					RTE_DISTRIB_FLAG_BITS);
			/*
			 * User is advocated to set tag vaue for each
			 * mbuf before calling rte_distributor_process.
			 * User defined tags are used to identify flows,
			 * or sessions.
			 */
			/* flows MUST be non-zero */
			new_tag = (uint16_t)(next_mb->hash.usr) | 1;

			/*
			 * Using the next line will cause the find_match
			 * function to be optimised out, making this function
			 * do parallel (non-atomic) distribution
			 */
			//matches[j] = 0;

			if (matches[j]) {
				struct rte_distributor_backlog *bl =
						&d->backlog[matches[j]-1];
				if (unlikely(bl->count == RTE_DIST_BURST_SIZE))
					release(d, matches[j]-1);

				/* Add to worker that already has flow */
				unsigned int idx = bl->count++;

				bl->tags[idx] = new_tag;
				bl->pkts[idx] = next_value;

			} else {
				struct rte_distributor_backlog *bl =
						&d->backlog[wkr];
				if (unlikely(bl->count == RTE_DIST_BURST_SIZE))
					release(d, wkr);

				/* Add to current worker worker */
				unsigned int idx = bl->count++;

				bl->tags[idx] = new_tag;
				bl->pkts[idx] = next_value;
				/*
				 * Now that we've just added an unpinned flow
				 * to a worker, we need to ensure that all
				 * other packets with that same flow will go
				 * to the same worker in this burst.
				 */
				for (int w = j; w < pkts; w++)
					if (flows[w] == new_tag)
						matches[w] = wkr+1;
			}
		}
		wkr++;
		if (wkr >= d->num_workers)
			wkr = 0;
	}

	/* Flush out all non-full cache-lines to workers. */
	for (unsigned int wid = 0 ; wid < d->num_workers; wid++) {
		if ((d->bufs[wid].bufptr64[0] & RTE_DISTRIB_GET_BUF))
			release(d, wid);
	}

	return num_mbufs;
}

/* return to the caller, packets returned from workers */
int
rte_distributor_returned_pkts_burst(struct rte_distributor *d,
		struct rte_mbuf **mbufs, unsigned int max_mbufs)
{
	struct rte_distributor_returned_pkts *returns = &d->returns;
	unsigned int retval = (max_mbufs < returns->count) ?
			max_mbufs : returns->count;
	unsigned int i;

	for (i = 0; i < retval; i++) {
		unsigned int idx = (returns->start + i) &
				RTE_DISTRIB_RETURNS_MASK;

		mbufs[i] = returns->mbufs[idx];
	}
	returns->start += i;
	returns->count -= i;

	return retval;
}

/*
 * Return the number of packets in-flight in a distributor, i.e. packets
 * being workered on or queued up in a backlog.
 */
static inline unsigned int
total_outstanding(const struct rte_distributor *d)
{
	unsigned int wkr, total_outstanding = 0;

	for (wkr = 0; wkr < d->num_workers; wkr++)
		total_outstanding += d->backlog[wkr].count;

	return total_outstanding;
}

/*
 * Flush the distributor, so that there are no outstanding packets in flight or
 * queued up.
 */
int
rte_distributor_flush_burst(struct rte_distributor *d)
{
	const unsigned int flushed = total_outstanding(d);

	while (total_outstanding(d) > 0)
		rte_distributor_process_burst(d, NULL, 0);

	return flushed;
}

/* clears the internal returns array in the distributor */
void
rte_distributor_clear_returns_burst(struct rte_distributor *d)
{
	/* throw away returns, so workers can exit */
	for (unsigned int wkr = 0; wkr < d->num_workers; wkr++)
		d->bufs[wkr].retptr64[0] = 0;
}

/* creates a distributor instance */
struct rte_distributor *
rte_distributor_create_burst(const char *name,
		unsigned int socket_id,
		unsigned int num_workers)
{
	struct rte_distributor *d;
	struct rte_distributor_list *distributor_list;
	char mz_name[RTE_MEMZONE_NAMESIZE];
	const struct rte_memzone *mz;

	/* compilation-time checks */
	RTE_BUILD_BUG_ON((sizeof(*d) & RTE_CACHE_LINE_MASK) != 0);
	RTE_BUILD_BUG_ON((RTE_DISTRIB_MAX_WORKERS & 7) != 0);

	if (name == NULL || num_workers >= RTE_DISTRIB_MAX_WORKERS) {
		rte_errno = EINVAL;
		return NULL;
	}

	snprintf(mz_name, sizeof(mz_name), RTE_DISTRIB_PREFIX"%s", name);
	mz = rte_memzone_reserve(mz_name, sizeof(*d), socket_id, NO_FLAGS);
	if (mz == NULL) {
		rte_errno = ENOMEM;
		return NULL;
	}

	d = mz->addr;
	snprintf(d->name, sizeof(d->name), "%s", name);
	d->num_workers = num_workers;

	/*
	 * Set up the backog tags so they're pointing at the second cache
	 * line for performance during flow matching
	 */
	for (unsigned int i = 0 ; i < num_workers ; i++)
		d->backlog[i].tags = &d->in_flight_tags[i][RTE_DIST_BURST_SIZE];

	distributor_list = RTE_TAILQ_CAST(rte_distributor_tailq.head,
					  rte_distributor_list);

	rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);
	TAILQ_INSERT_TAIL(distributor_list, d, next);
	rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);

	return d;
}
