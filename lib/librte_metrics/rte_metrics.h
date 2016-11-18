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

/**
 * @file
 *
 * RTE Metrics module
 *
 * Metric information is populated using a push model, where the
 * information provider calls an update function on the relevant
 * metrics. Currently only bulk querying of metrics is supported.
 */

#ifndef _RTE_METRICS_H_
#define _RTE_METRICS_H_

/** Maximum length of metric name (including null-terminator) */
#define RTE_METRICS_MAX_NAME_LEN 64

/** Used to indicate port-independent information */
#define RTE_METRICS_NONPORT -1


/**
 * Metric name
 */
struct rte_metric_name {
	/** String describing metric */
	char name[RTE_METRICS_MAX_NAME_LEN];
};


/**
 * Metric name.
 */
struct rte_metric_value {
	/** Numeric identifier of metric */
	uint16_t key;
	/** Value for metric */
	uint64_t value;
};


/**
 * Initializes metric module. This only has to be explicitly called if you
 * intend to use rte_metrics_reg_metric() or rte_metrics_reg_metrics() from a
 * secondary process. This function must be called from a primary process.
 */
void rte_metrics_init(void);


/**
 * Register a metric
 *
 * @param name
 *   Metric name
 *
 * @return
 *  - Zero or positive: Success
 *  - Negative: Failure
 */
int rte_metrics_reg_metric(const char *name);

/**
 * Register a set of metrics
 *
 * @param names
 *   List of metric names
 *
 * @param cnt_names
 *   Number of metrics in set
 *
 * @return
 *  - Zero or positive: Success
 *  - Negative: Failure
 */
int rte_metrics_reg_metrics(const char **names, uint16_t cnt_names);

/**
 * Get metric name-key lookup table.
 *
 * @param names
 *   Array of names to receive key names
 *
 * @param capacity
 *   Space available in names
 *
 * @return
 *   - Non-negative: Success (number of names)
 *   - Negative: Failure
 */
int rte_metrics_get_names(
	struct rte_metric_name *names,
	uint16_t capacity);

/**
 * Fetch metrics.
 *
 * @param port_id
 *   Port id to query
 *
 * @param values
 *   Array to receive values and their keys
 *
 * @param capacity
 *   Space available in values
 *
 * @return
 *   - Non-negative: Success (number of names)
 *   - Negative: Failure
 */
int rte_metrics_get_values(
	int port_id,
	struct rte_metric_value *values,
	uint16_t capacity);

/**
 * Updates a metric
 *
 * @param port_id
 *   Port to update metrics for
 * @param key
 *   Id of metric to update
 * @param value
 *   New value
 *
 * @return
 *   - -EIO if unable to access shared metrics memory
 *   - Zero on success
 */
int rte_metrics_update_metric(
	int port_id,
	uint16_t key,
	const uint64_t value);

/**
 * Updates a metric set. Note that it is an error to try to
 * update across a set boundary.
 *
 * @param port_id
 *   Port to update metrics for
 * @param key
 *   Base id of metrics set to update
 * @param values
 *   Set of new values
 * @param count
 *   Number of new values
 *
 * @return
 *   - -ERANGE if count exceeds metric set size
 *   - -EIO if upable to access shared metrics memory
 *   - Zero on success
 */
int rte_metrics_update_metrics(
	int port_id,
	uint16_t key,
	const uint64_t *values,
	uint32_t count);

#endif
