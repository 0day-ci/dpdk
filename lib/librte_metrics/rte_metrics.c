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

#include <string.h>
#include <sys/queue.h>

#include <rte_common.h>
#include <rte_malloc.h>
#include <rte_metrics.h>
#include <rte_lcore.h>
#include <rte_memzone.h>


#define RTE_METRICS_MAX_METRICS 256


#define RTE_METRICS_MEMZONE_NAME "RTE_METRICS"

/**
 * Internal stats metadata and value entry.
 *
 * @internal
 * @param name
 *   Name of metric
 * @param value
 *   Current value for metric
 * @param idx_next_set
 *   Index of next root element (zero for none)
 * @param idx_next_metric
 *   Index of next metric in set (zero for none)
 *
 * Only the root of each set needs idx_next_set but since it has to be
 * assumed that number of sets could equal total number of metrics,
 * having a separate set metadata table doesn't save any memory.
 */
struct rte_metrics_meta_s {
	char name[RTE_METRICS_MAX_NAME_LEN];
	uint64_t value[RTE_MAX_ETHPORTS];
	uint16_t idx_next_set;
	uint16_t idx_next_stat;
};

/**
 * Internal stats info structure.
 *
 * @internal
 * @param idx_last_set
 *   Index of last metadata entry with valid data. This value is
 *   not valid if cnt_stats is zero.
 * @param cnt_stats
 *   Number of metrics.
 * @param metadata
 *   Stat data memory block.
 *
 * Offsets into metadata are used instead of pointers because ASLR
 * means that having the same physical addresses in different
 * processes is not guaranteed.
 */
struct rte_metrics_data_s {
	uint16_t idx_last_set;
	uint16_t cnt_stats;
	struct rte_metrics_meta_s metadata[RTE_METRICS_MAX_METRICS];
};


void
rte_metrics_init(void)
{
	struct rte_metrics_data_s *stats;
	const struct rte_memzone *memzone;

	if (rte_eal_process_type() != RTE_PROC_PRIMARY)
		return;

	memzone = rte_memzone_lookup(RTE_METRICS_MEMZONE_NAME);
	if (memzone != NULL)
		return;
	memzone = rte_memzone_reserve(RTE_METRICS_MEMZONE_NAME,
		sizeof(struct rte_metrics_data_s), rte_socket_id(), 0);
	if (memzone == NULL)
		rte_exit(EXIT_FAILURE, "Unable to allocate stats memzone\n");
	stats = memzone->addr;
	memset(stats, 0, sizeof(struct rte_metrics_data_s));
}


int
rte_metrics_reg_metric(const char *name)
{
	const char *list_names[] = {name};

	return rte_metrics_reg_metrics(list_names, 1);
}


int
rte_metrics_reg_metrics(const char **names, uint16_t cnt_names)
{
	struct rte_metrics_meta_s *entry;
	struct rte_metrics_data_s *stats;
	const struct rte_memzone *memzone;
	uint16_t idx_name;
	uint16_t idx_base;

	/* Some sanity checks */
	if (cnt_names < 1 || names == NULL)
		return -EINVAL;

	rte_metrics_init();
	memzone = rte_memzone_lookup(RTE_METRICS_MEMZONE_NAME);
	if (memzone == NULL)
		return -EIO;
	stats = memzone->addr;

	if (stats->cnt_stats + cnt_names >= RTE_METRICS_MAX_METRICS)
		return -ENOMEM;

	/* Overwritten later if this is actually first set.. */
	stats->metadata[stats->idx_last_set].idx_next_set = stats->cnt_stats;

	stats->idx_last_set = idx_base = stats->cnt_stats;

	for (idx_name = 0; idx_name < cnt_names; idx_name++) {
		entry = &stats->metadata[idx_name + stats->cnt_stats];
		strncpy(entry->name, names[idx_name],
			RTE_METRICS_MAX_NAME_LEN);
		memset(entry->value, 0, sizeof(entry->value));
		entry->idx_next_stat = idx_name + stats->cnt_stats + 1;
	}
	entry->idx_next_stat = 0;
	entry->idx_next_set = 0;
	stats->cnt_stats += cnt_names;

	return idx_base;
}


int
rte_metrics_update_metric(int port_id, uint16_t key, const uint64_t value)
{
	return rte_metrics_update_metrics(port_id, key, &value, 1);
}


int
rte_metrics_update_metrics(int port_id,
	uint16_t key,
	const uint64_t *values,
	uint32_t count)
{
	struct rte_metrics_meta_s *entry;
	struct rte_metrics_data_s *stats;
	const struct rte_memzone *memzone;
	uint16_t idx_metric;
	uint16_t idx_value;
	uint16_t cnt_setsize;

	rte_metrics_init();
	memzone = rte_memzone_lookup(RTE_METRICS_MEMZONE_NAME);
	if (memzone == NULL)
		return -EIO;
	stats = memzone->addr;

	idx_metric = key;
	cnt_setsize = 1;
	while (idx_metric < stats->cnt_stats) {
		entry = &stats->metadata[idx_metric];
		if (entry->idx_next_stat == 0)
			break;
		cnt_setsize++;
		idx_metric++;
	}
	/* Check update does not cross set border */
	if (count > cnt_setsize)
		return -ERANGE;

	for (idx_value = 0; idx_value < count; idx_value++) {
		idx_metric = key + idx_value;
		stats->metadata[idx_metric].value[port_id] = values[idx_value];
	}
	return 0;
}


int
rte_metrics_get_names(struct rte_metric_name *names,
	uint16_t capacity)
{
	struct rte_metrics_data_s *stats;
	const struct rte_memzone *memzone;
	uint16_t idx_name;

	memzone = rte_memzone_lookup(RTE_METRICS_MEMZONE_NAME);
	/* If not allocated, fail silently */
	if (memzone == NULL)
		return 0;
	stats = memzone->addr;

	if (names != NULL) {
		if (capacity < stats->cnt_stats)
			return -ERANGE;
		for (idx_name = 0; idx_name < stats->cnt_stats; idx_name++)
			strncpy(names[idx_name].name,
				stats->metadata[idx_name].name,
				RTE_METRICS_MAX_NAME_LEN);
	}
	return stats->cnt_stats;
}


int
rte_metrics_get_values(int port_id,
	struct rte_stat_value *values,
	uint16_t capacity)
{
	struct rte_metrics_meta_s *entry;
	struct rte_metrics_data_s *stats;
	const struct rte_memzone *memzone;
	uint16_t idx_name;

	memzone = rte_memzone_lookup(RTE_METRICS_MEMZONE_NAME);
	/* If not allocated, fail silently */
	if (memzone == NULL)
		return 0;
	stats = memzone->addr;

	if (values != NULL) {
		if (capacity < stats->cnt_stats)
			return -ERANGE;
		for (idx_name = 0; idx_name < stats->cnt_stats; idx_name++) {
			entry = &stats->metadata[idx_name];
			values[idx_name].key = idx_name;
			values[idx_name].value = entry->value[port_id];
		}
	}
	return stats->cnt_stats;
}
