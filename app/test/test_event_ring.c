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

#include <rte_random.h>
#include <rte_event_ring.h>
#include "test.h"

#define BURST_SIZE 24 /* not a power of two so we can test wrap around better */
#define RING_SIZE 128
#define ENQ_ITERATIONS 48

#define ERR_OUT() do { \
	printf("Error %s:%d\n", __FILE__, __LINE__); \
	return -1; \
} while (0)

static int
test_event_ring(void)
{
	struct rte_event in_events[BURST_SIZE];
	struct rte_event out_events[BURST_SIZE];
	unsigned int i;

	struct rte_event_ring *ering = rte_event_ring_create("TEST_RING",
			RING_SIZE, rte_socket_id(),
			RING_F_SP_ENQ|RING_F_SC_DEQ);
	if (ering == NULL)
		ERR_OUT();

	for (i = 0; i < BURST_SIZE; i++)
		in_events[i].event_metadata = rte_rand();

	for (i = 0; i < ENQ_ITERATIONS; i++)
		if (rte_event_ring_enqueue_bulk(ering, in_events,
				RTE_DIM(in_events), NULL) != 0) {
			unsigned j;

			if (rte_event_ring_dequeue_burst(ering, out_events,
					RTE_DIM(out_events), NULL)
						!= RTE_DIM(out_events))
				ERR_OUT();
			for (j = 0; j < RTE_DIM(out_events); j++)
				if (out_events[j].event_metadata !=
						in_events[j].event_metadata)
					ERR_OUT();
			/* retry, now that we've made space */
			if (rte_event_ring_enqueue_bulk(ering, in_events,
					RTE_DIM(in_events), NULL) != 0)
				ERR_OUT();
		}

	return 0;
}

REGISTER_TEST_COMMAND(event_ring_autotest, test_event_ring);
