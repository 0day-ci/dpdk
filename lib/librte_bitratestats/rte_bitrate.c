/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Intel Corporation. All rights reserved.
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

#include <rte_common.h>
#include <rte_ethdev.h>
#include <rte_malloc.h>
#include <rte_metrics.h>
#include <rte_bitrate.h>


struct rte_stats_bitrate_s {
	uint64_t last_ibytes;
	uint64_t last_obytes;
	uint64_t peak_ibits;
	uint64_t peak_obits;
	uint64_t ewma_ibits;
	uint64_t ewma_obits;
};

struct rte_stats_bitrates_s {
	struct rte_stats_bitrate_s port_stats[RTE_MAX_ETHPORTS];
	uint16_t id_stats_set;
};


struct rte_stats_bitrates_s *rte_stats_bitrate_create(void)
{
	return rte_zmalloc(NULL, sizeof(struct rte_stats_bitrates_s), 0);
}


int
rte_stats_bitrate_reg(struct rte_stats_bitrates_s *bitrate_data)
{
	const char *names[] = {
		"mean_bits_in", "mean_bits_out",
		"peak_bits_in", "peak_bits_out",
	};
	int return_value;

	bitrate_data = rte_stats_bitrate_create();
	if (bitrate_data == NULL)
		rte_exit(EXIT_FAILURE, "Could not allocate bitrate data.\n");
	return_value = rte_metrics_reg_metrics(&names[0], 4);
	if (return_value >= 0)
		bitrate_data->id_stats_set = return_value;
	return return_value;
}


int
rte_stats_bitrate_calc(struct rte_stats_bitrates_s *bitrate_data,
	uint8_t port_id)
{
	struct rte_stats_bitrate_s *port_data;
	struct rte_eth_stats eth_stats;
	int ret_code;
	uint64_t cnt_bits;
	int64_t delta;
	const int64_t alpha_percent = 20;
	uint64_t values[4];

	ret_code = rte_eth_stats_get(port_id, &eth_stats);
	if (ret_code != 0)
		return ret_code;

	port_data = &bitrate_data->port_stats[port_id];

	/* Incoming */
	cnt_bits = (eth_stats.ibytes - port_data->last_ibytes) << 3;
	port_data->last_ibytes = eth_stats.ibytes;
	if (cnt_bits > port_data->peak_ibits)
		port_data->peak_ibits = cnt_bits;
	delta = cnt_bits;
	delta -= port_data->ewma_ibits;
	delta = (delta * alpha_percent) / 100;
	port_data->ewma_ibits += delta;

	/* Outgoing */
	cnt_bits = (eth_stats.obytes - port_data->last_obytes) << 3;
	port_data->last_obytes = eth_stats.obytes;
	if (cnt_bits > port_data->peak_obits)
		port_data->peak_obits = cnt_bits;
	delta = cnt_bits;
	delta -= port_data->ewma_obits;
	delta = (delta * alpha_percent) / 100;
	port_data->ewma_obits += delta;

	values[0] = port_data->ewma_ibits;
	values[1] = port_data->ewma_obits;
	values[2] = port_data->peak_ibits;
	values[3] = port_data->peak_obits;
	rte_metrics_update_metrics(port_id, bitrate_data->id_stats_set,
		values, 4);
	return 0;
}
