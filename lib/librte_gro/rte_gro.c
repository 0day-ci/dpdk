/*-
 *
 *   Copyright(c) 2016-2017 Intel Corporation. All rights reserved.
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

#include <rte_malloc.h>
#include <rte_mbuf.h>

#include "rte_gro.h"

static gro_tbl_create_fn tbl_create_functions[GRO_TYPE_MAX_NB];
static gro_tbl_destroy_fn tbl_destroy_functions[GRO_TYPE_MAX_NB];

struct rte_gro_tbl *rte_gro_tbl_create(uint16_t socket_id,
		uint16_t max_flow_num,
		uint16_t max_item_per_flow,
		uint32_t max_packet_size,
		uint64_t max_timeout_cycles,
		uint64_t desired_gro_types)
{
	gro_tbl_create_fn create_tbl_fn;
	struct rte_gro_tbl *gro_tbl;
	uint64_t gro_type_flag = 0;
	uint8_t i;

	gro_tbl = rte_zmalloc_socket(__func__,
			sizeof(struct rte_gro_tbl),
			RTE_CACHE_LINE_SIZE,
			socket_id);
	gro_tbl->max_packet_size = max_packet_size;
	gro_tbl->max_timeout_cycles = max_timeout_cycles;
	gro_tbl->desired_gro_types = desired_gro_types;

	for (i = 0; i < GRO_TYPE_MAX_NB; i++) {
		gro_type_flag = 1 << i;
		if (desired_gro_types & gro_type_flag) {
			create_tbl_fn = tbl_create_functions[i];
			if (create_tbl_fn)
				create_tbl_fn(socket_id,
						max_flow_num,
						max_item_per_flow);
			else
				gro_tbl->tbls[i] = NULL;
		}
	}
	return gro_tbl;
}

void rte_gro_tbl_destroy(struct rte_gro_tbl *gro_tbl)
{
	gro_tbl_destroy_fn destroy_tbl_fn;
	uint64_t gro_type_flag;
	uint8_t i;

	if (gro_tbl == NULL)
		return;
	for (i = 0; i < GRO_TYPE_MAX_NB; i++) {
		gro_type_flag = 1 << i;
		if (gro_tbl->desired_gro_types & gro_type_flag) {
			destroy_tbl_fn = tbl_destroy_functions[i];
			if (destroy_tbl_fn)
				destroy_tbl_fn(gro_tbl->tbls[i]);
			gro_tbl->tbls[i] = NULL;
		}
	}
	rte_free(gro_tbl);
}

uint16_t
rte_gro_reassemble_burst(struct rte_mbuf **pkts __rte_unused,
		const uint16_t nb_pkts,
		const struct rte_gro_param param __rte_unused)
{
	return nb_pkts;
}

int rte_gro_reassemble(struct rte_mbuf *pkt __rte_unused,
		struct rte_gro_tbl *gro_tbl __rte_unused)
{
	return -1;
}

uint16_t rte_gro_flush(struct rte_gro_tbl *gro_tbl __rte_unused,
		uint64_t desired_gro_types __rte_unused,
		uint16_t flush_num __rte_unused,
		struct rte_mbuf **out __rte_unused,
		const uint16_t max_nb_out __rte_unused)
{
	return 0;
}

uint16_t
rte_gro_timeout_flush(struct rte_gro_tbl *gro_tbl __rte_unused,
		uint64_t desired_gro_types __rte_unused,
		struct rte_mbuf **out __rte_unused,
		const uint16_t max_nb_out __rte_unused)
{
	return 0;
}
