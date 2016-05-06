/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2015 Intel Corporation. All rights reserved.
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
#include <sys/queue.h>
#include <netinet/in.h>
#include <unistd.h>

#include <rte_common.h>
#include <rte_hexdump.h>
#include <rte_malloc.h>
#include <cmdline_rdline.h>
#include <cmdline_parse.h>
#include <cmdline_parse_num.h>
#include <cmdline_parse_string.h>
#include <cmdline_parse_ipaddr.h>
#include <cmdline_parse_etheraddr.h>

#include "app.h"
#include "pipeline_common_fe.h"
#include "pipeline_flow_actions.h"
#include "hash_func.h"
#include "parser.h"

/*
 * Flow actions pipeline
 */
#ifndef N_FLOWS_BULK
#define N_FLOWS_BULK					4096
#endif

#define BUF_SIZE						1024
#define FA_MAX_NUM_OF_TOKENS			48
#define FA_MIN_NUM_OF_TOKENS			15
#define FA_MAX_NUM_OF_RULES				4
#define FA_MIN_NUM_OF_RULES				1

struct app_pipeline_fa_flow {
	struct pipeline_fa_flow_params params;
	void *entry_ptr;
};

struct app_pipeline_fa_dscp {
	uint32_t traffic_class;
	enum rte_meter_color color;
};

struct app_pipeline_fa {
	/* Parameters */
	uint32_t n_ports_in;
	uint32_t n_ports_out;
	struct pipeline_fa_params params;

	/* Flows */
	struct app_pipeline_fa_dscp dscp[PIPELINE_FA_N_DSCP];
	struct app_pipeline_fa_flow *flows;
} __rte_cache_aligned;

struct app_pipeline_add_bulk_params {
	struct pipeline_fa_flow_params *keys;
	uint32_t n_keys;
	uint32_t *flow_ids;
};

static void*
app_pipeline_fa_init(struct pipeline_params *params,
	__rte_unused void *arg)
{
	struct app_pipeline_fa *p;
	uint32_t size, i;

	/* Check input arguments */
	if ((params == NULL) ||
		(params->n_ports_in == 0) ||
		(params->n_ports_out == 0))
		return NULL;

	/* Memory allocation */
	size = RTE_CACHE_LINE_ROUNDUP(sizeof(struct app_pipeline_fa));
	p = rte_zmalloc(NULL, size, RTE_CACHE_LINE_SIZE);
	if (p == NULL)
		return NULL;

	/* Initialization */
	p->n_ports_in = params->n_ports_in;
	p->n_ports_out = params->n_ports_out;
	if (pipeline_fa_parse_args(&p->params, params)) {
		rte_free(p);
		return NULL;
	}

	/* Memory allocation */
	size = RTE_CACHE_LINE_ROUNDUP(
		p->params.n_flows * sizeof(struct app_pipeline_fa_flow));
	p->flows = rte_zmalloc(NULL, size, RTE_CACHE_LINE_SIZE);
	if (p->flows == NULL) {
		rte_free(p);
		return NULL;
	}

	/* Initialization of flow table */
	for (i = 0; i < p->params.n_flows; i++)
		pipeline_fa_flow_params_set_default(&p->flows[i].params);

	/* Initialization of DSCP table */
	for (i = 0; i < RTE_DIM(p->dscp); i++) {
		p->dscp[i].traffic_class = 0;
		p->dscp[i].color = e_RTE_METER_GREEN;
	}

	return (void *) p;
}

static int
app_pipeline_fa_free(void *pipeline)
{
	struct app_pipeline_fa *p = pipeline;

	/* Check input arguments */
	if (p == NULL)
		return -1;

	/* Free resources */
	rte_free(p->flows);
	rte_free(p);

	return 0;
}

static int
flow_params_check(struct app_pipeline_fa *p,
	__rte_unused uint32_t meter_update_mask,
	uint32_t policer_update_mask,
	uint32_t port_update,
	struct pipeline_fa_flow_params *params)
{
	uint32_t mask, i;

	/* Meter */

	/* Policer */
	for (i = 0, mask = 1; i < PIPELINE_FA_N_TC_MAX; i++, mask <<= 1) {
		struct pipeline_fa_policer_params *p = &params->p[i];
		uint32_t j;

		if ((mask & policer_update_mask) == 0)
			continue;

		for (j = 0; j < e_RTE_METER_COLORS; j++) {
			struct pipeline_fa_policer_action *action =
				&p->action[j];

			if ((action->drop == 0) &&
				(action->color >= e_RTE_METER_COLORS))
				return -1;
		}
	}

	/* Port */
	if (port_update && (params->port_id >= p->n_ports_out))
		return -1;

	return 0;
}

int
app_pipeline_fa_flow_config(struct app_params *app,
	uint32_t pipeline_id,
	uint32_t flow_id,
	uint32_t meter_update_mask,
	uint32_t policer_update_mask,
	uint32_t port_update,
	struct pipeline_fa_flow_params *params)
{
	struct app_pipeline_fa *p;
	struct app_pipeline_fa_flow *flow;

	struct pipeline_fa_flow_config_msg_req *req;
	struct pipeline_fa_flow_config_msg_rsp *rsp;

	uint32_t i, mask;

	/* Check input arguments */
	if ((app == NULL) ||
		((meter_update_mask == 0) &&
		(policer_update_mask == 0) &&
		(port_update == 0)) ||
		(meter_update_mask >= (1 << PIPELINE_FA_N_TC_MAX)) ||
		(policer_update_mask >= (1 << PIPELINE_FA_N_TC_MAX)) ||
		(params == NULL))
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id,
		&pipeline_flow_actions);
	if (p == NULL)
		return -1;

	if (flow_params_check(p,
		meter_update_mask,
		policer_update_mask,
		port_update,
		params) != 0)
		return -1;

	flow_id %= p->params.n_flows;
	flow = &p->flows[flow_id];

	/* Allocate and write request */
	req = app_msg_alloc(app);
	if (req == NULL)
		return -1;

	req->type = PIPELINE_MSG_REQ_CUSTOM;
	req->subtype = PIPELINE_FA_MSG_REQ_FLOW_CONFIG;
	req->entry_ptr = flow->entry_ptr;
	req->flow_id = flow_id;
	req->meter_update_mask = meter_update_mask;
	req->policer_update_mask = policer_update_mask;
	req->port_update = port_update;
	memcpy(&req->params, params, sizeof(*params));

	/* Send request and wait for response */
	rsp = app_msg_send_recv(app, pipeline_id, req, MSG_TIMEOUT_DEFAULT);
	if (rsp == NULL)
		return -1;

	/* Read response */
	if (rsp->status ||
		(rsp->entry_ptr == NULL)) {
		app_msg_free(app, rsp);
		return -1;
	}

	/* Commit flow */
	for (i = 0, mask = 1; i < PIPELINE_FA_N_TC_MAX; i++, mask <<= 1) {
		if ((mask & meter_update_mask) == 0)
			continue;

		memcpy(&flow->params.m[i], &params->m[i], sizeof(params->m[i]));
	}

	for (i = 0, mask = 1; i < PIPELINE_FA_N_TC_MAX; i++, mask <<= 1) {
		if ((mask & policer_update_mask) == 0)
			continue;

		memcpy(&flow->params.p[i], &params->p[i], sizeof(params->p[i]));
	}

	if (port_update)
		flow->params.port_id = params->port_id;

	flow->entry_ptr = rsp->entry_ptr;

	/* Free response */
	app_msg_free(app, rsp);

	return 0;
}

int
app_pipeline_fa_flow_config_bulk(struct app_params *app,
	uint32_t pipeline_id,
	uint32_t *flow_id,
	uint32_t n_flows,
	uint32_t meter_update_mask,
	uint32_t policer_update_mask,
	uint32_t port_update,
	struct pipeline_fa_flow_params *params)
{
	struct app_pipeline_fa *p;
	struct pipeline_fa_flow_config_bulk_msg_req *req;
	struct pipeline_fa_flow_config_bulk_msg_rsp *rsp;
	void **req_entry_ptr;
	uint32_t *req_flow_id;
	uint32_t i;

	/* Check input arguments */
	if ((app == NULL) ||
		(flow_id == NULL) ||
		(n_flows == 0) ||
		((meter_update_mask == 0) &&
		(policer_update_mask == 0) &&
		(port_update == 0)) ||
		(meter_update_mask >= (1 << PIPELINE_FA_N_TC_MAX)) ||
		(policer_update_mask >= (1 << PIPELINE_FA_N_TC_MAX)) ||
		(params == NULL))
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id,
		&pipeline_flow_actions);
	if (p == NULL)
		return -1;

	for (i = 0; i < n_flows; i++) {
		struct pipeline_fa_flow_params *flow_params = &params[i];

		if (flow_params_check(p,
			meter_update_mask,
			policer_update_mask,
			port_update,
			flow_params) != 0)
			return -1;
	}

	/* Allocate and write request */
	req_entry_ptr = (void **) rte_malloc(NULL,
		n_flows * sizeof(void *),
		RTE_CACHE_LINE_SIZE);
	if (req_entry_ptr == NULL)
		return -1;

	req_flow_id = (uint32_t *) rte_malloc(NULL,
		n_flows * sizeof(uint32_t),
		RTE_CACHE_LINE_SIZE);
	if (req_flow_id == NULL) {
		rte_free(req_entry_ptr);
		return -1;
	}

	for (i = 0; i < n_flows; i++) {
		uint32_t fid = flow_id[i] % p->params.n_flows;
		struct app_pipeline_fa_flow *flow = &p->flows[fid];

		req_flow_id[i] = fid;
		req_entry_ptr[i] = flow->entry_ptr;
	}

	req = app_msg_alloc(app);
	if (req == NULL) {
		rte_free(req_flow_id);
		rte_free(req_entry_ptr);
		return -1;
	}

	req->type = PIPELINE_MSG_REQ_CUSTOM;
	req->subtype = PIPELINE_FA_MSG_REQ_FLOW_CONFIG_BULK;
	req->entry_ptr = req_entry_ptr;
	req->flow_id = req_flow_id;
	req->n_flows = n_flows;
	req->meter_update_mask = meter_update_mask;
	req->policer_update_mask = policer_update_mask;
	req->port_update = port_update;
	req->params = params;

	/* Send request and wait for response */
	rsp = app_msg_send_recv(app, pipeline_id, req, MSG_TIMEOUT_DEFAULT);
	if (rsp == NULL) {
		rte_free(req_flow_id);
		rte_free(req_entry_ptr);
		return -1;
	}

	/* Read response */

	/* Commit flows */
	for (i = 0; i < rsp->n_flows; i++) {
		uint32_t fid = flow_id[i] % p->params.n_flows;
		struct app_pipeline_fa_flow *flow = &p->flows[fid];
		struct pipeline_fa_flow_params *flow_params = &params[i];
		void *entry_ptr = req_entry_ptr[i];
		uint32_t j, mask;

		for (j = 0, mask = 1; j < PIPELINE_FA_N_TC_MAX;
			j++, mask <<= 1) {
			if ((mask & meter_update_mask) == 0)
				continue;

			memcpy(&flow->params.m[j],
				&flow_params->m[j],
				sizeof(flow_params->m[j]));
		}

		for (j = 0, mask = 1; j < PIPELINE_FA_N_TC_MAX;
			j++, mask <<= 1) {
			if ((mask & policer_update_mask) == 0)
				continue;

			memcpy(&flow->params.p[j],
				&flow_params->p[j],
				sizeof(flow_params->p[j]));
		}

		if (port_update)
			flow->params.port_id = flow_params->port_id;

		flow->entry_ptr = entry_ptr;
	}

	/* Free response */
	app_msg_free(app, rsp);
	rte_free(req_flow_id);
	rte_free(req_entry_ptr);

	return (rsp->n_flows == n_flows) ? 0 : -1;
}

int
app_pipeline_fa_dscp_config(struct app_params *app,
	uint32_t pipeline_id,
	uint32_t dscp,
	uint32_t traffic_class,
	enum rte_meter_color color)
{
	struct app_pipeline_fa *p;

	struct pipeline_fa_dscp_config_msg_req *req;
	struct pipeline_fa_dscp_config_msg_rsp *rsp;

	/* Check input arguments */
	if ((app == NULL) ||
		(dscp >= PIPELINE_FA_N_DSCP) ||
		(traffic_class >= PIPELINE_FA_N_TC_MAX) ||
		(color >= e_RTE_METER_COLORS))
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id,
		&pipeline_flow_actions);
	if (p == NULL)
		return -1;

	if (p->params.dscp_enabled == 0)
		return -1;

	/* Allocate and write request */
	req = app_msg_alloc(app);
	if (req == NULL)
		return -1;

	req->type = PIPELINE_MSG_REQ_CUSTOM;
	req->subtype = PIPELINE_FA_MSG_REQ_DSCP_CONFIG;
	req->dscp = dscp;
	req->traffic_class = traffic_class;
	req->color = color;

	/* Send request and wait for response */
	rsp = app_msg_send_recv(app, pipeline_id, req, MSG_TIMEOUT_DEFAULT);
	if (rsp == NULL)
		return -1;

	/* Read response */
	if (rsp->status) {
		app_msg_free(app, rsp);
		return -1;
	}

	/* Commit DSCP */
	p->dscp[dscp].traffic_class = traffic_class;
	p->dscp[dscp].color = color;

	/* Free response */
	app_msg_free(app, rsp);

	return 0;
}

int
app_pipeline_fa_flow_policer_stats_read(struct app_params *app,
	uint32_t pipeline_id,
	uint32_t flow_id,
	uint32_t policer_id,
	int clear,
	struct pipeline_fa_policer_stats *stats)
{
	struct app_pipeline_fa *p;
	struct app_pipeline_fa_flow *flow;

	struct pipeline_fa_policer_stats_msg_req *req;
	struct pipeline_fa_policer_stats_msg_rsp *rsp;

	/* Check input arguments */
	if ((app == NULL) || (stats == NULL))
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id,
		&pipeline_flow_actions);
	if (p == NULL)
		return -1;

	flow_id %= p->params.n_flows;
	flow = &p->flows[flow_id];

	if ((policer_id >= p->params.n_meters_per_flow) ||
		(flow->entry_ptr == NULL))
		return -1;

	/* Allocate and write request */
	req = app_msg_alloc(app);
	if (req == NULL)
		return -1;

	req->type = PIPELINE_MSG_REQ_CUSTOM;
	req->subtype = PIPELINE_FA_MSG_REQ_POLICER_STATS_READ;
	req->entry_ptr = flow->entry_ptr;
	req->policer_id = policer_id;
	req->clear = clear;

	/* Send request and wait for response */
	rsp = app_msg_send_recv(app, pipeline_id, req, MSG_TIMEOUT_DEFAULT);
	if (rsp == NULL)
		return -1;

	/* Read response */
	if (rsp->status) {
		app_msg_free(app, rsp);
		return -1;
	}

	memcpy(stats, &rsp->stats, sizeof(*stats));

	/* Free response */
	app_msg_free(app, rsp);

	return 0;
}

static const char *
color_to_string(enum rte_meter_color color)
{
	switch (color) {
	case e_RTE_METER_GREEN: return "G";
	case e_RTE_METER_YELLOW: return "Y";
	case e_RTE_METER_RED: return "R";
	default: return "?";
	}
}

static int
string_to_color(char *s, enum rte_meter_color *c)
{
	if (strcmp(s, "G") == 0) {
		*c = e_RTE_METER_GREEN;
		return 0;
	}

	if (strcmp(s, "Y") == 0) {
		*c = e_RTE_METER_YELLOW;
		return 0;
	}

	if (strcmp(s, "R") == 0) {
		*c = e_RTE_METER_RED;
		return 0;
	}

	return -1;
}

static const char *
policer_action_to_string(struct pipeline_fa_policer_action *a)
{
	if (a->drop)
		return "D";

	return color_to_string(a->color);
}

static int
string_to_policer_action(char *s, struct pipeline_fa_policer_action *a)
{
	if (strcmp(s, "G") == 0) {
		a->drop = 0;
		a->color = e_RTE_METER_GREEN;
		return 0;
	}

	if (strcmp(s, "Y") == 0) {
		a->drop = 0;
		a->color = e_RTE_METER_YELLOW;
		return 0;
	}

	if (strcmp(s, "R") == 0) {
		a->drop = 0;
		a->color = e_RTE_METER_RED;
		return 0;
	}

	if (strcmp(s, "D") == 0) {
		a->drop = 1;
		a->color = e_RTE_METER_GREEN;
		return 0;
	}

	return -1;
}

static int
app_pipeline_fa_add_bulk_parse_file(char *filename,
		struct app_pipeline_add_bulk_params *params)
{
	FILE *file;
	char file_buf[BUF_SIZE];
	char *tokens[FA_MAX_NUM_OF_TOKENS];
	uint32_t i, num_value, ret_tokens, line = 0;
	uint64_t num_value64;
	int status = 0;

	file = fopen(filename, "r");
	if (file == NULL) {
		if (getcwd(file_buf, sizeof(file_buf)) != NULL)
			printf("not found file %s in the current working dir %s\n",
					filename, file_buf);
		return -1;
	}

	params->n_keys = 0;
	while (fgets(file_buf, BUF_SIZE, file) != NULL)
		if (file_buf[0] != '\0' && file_buf[0] != '\n' && file_buf[0] != '#')
			params->n_keys++;
	rewind(file);

	if (params->n_keys == 0) {
		printf("not found any keys in the file %s\n", filename);
		status = -1;
		goto end;
	}

	params->keys = rte_malloc(NULL,
			params->n_keys * sizeof(struct pipeline_fa_flow_params),
			RTE_CACHE_LINE_SIZE);
	if (params->keys == NULL) {
		printf("out of memory\n");
		status = -1;
		goto end;
	}

	params->flow_ids = rte_malloc(NULL,
			params->n_keys * sizeof(uint32_t),
			RTE_CACHE_LINE_SIZE);
	if (params->flow_ids == NULL) {
		printf("out of memory\n");
		status = -1;
		goto end;
	}

	/* set default values for each key */
	for (i = 0; i < params->n_keys; i++) {
		status = pipeline_fa_flow_params_set_default(&params->keys[i]);

		if (status != 0) {
			printf("there was a problem with the setting default value\n");
			status = -1;
			goto end;
		}
	}

	i = 0;
	while (fgets(file_buf, BUF_SIZE, file) != NULL) {
		uint8_t j, pos = 0, policer_count = 0, meter_count = 0, id_mask = 0;

		ret_tokens = FA_MAX_NUM_OF_TOKENS;

		status = parse_tokenize_string(file_buf, tokens, &ret_tokens);

		if (status != 0) {
			if (status == -E2BIG)
				printf("too many parameters at %d line\n", line + 1);
			else
				printf("there was a problem with tokenize at %d line\n",
						line + 1);

			status = -1;
			goto end;

		} else if (ret_tokens == 0 || tokens[0][0] == '#') {
			ret_tokens = FA_MAX_NUM_OF_TOKENS;
			line++;
			continue;

		} else if (ret_tokens < FA_MIN_NUM_OF_TOKENS) {
			printf("not enough parameters at %d line\n", line + 1);
			status = -1;
			goto end;
		}

		status = strcmp(tokens[pos++], "flow");
		if (status != 0) {
			printf("not found keyword \'flow\' at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		status = parser_read_uint32(&num_value, tokens[pos]);
		if (status != 0) {
			printf("conversion error flow id: \'%s\' at line %d\n",
					tokens[pos], line + 1);
			status = -1;
			goto end;
		}
		params->flow_ids[i] = num_value;
		pos++;

		/* check the number of occurrences of keywords 'policer' and 'meter' */
		for (j = 0; j < ret_tokens; j++) {
			if (strcmp(tokens[j], "policer") == 0)
				policer_count++;
			else if (strcmp(tokens[j], "meter") == 0)
				meter_count++;
		}

		/* flow + id + port + id = 4 tokens */
		if (ret_tokens != (uint32_t)(4 + (policer_count * 5) +
				(meter_count * 6))) {
			printf("incorrect amount of parameters at %d line for %d policers "
					"and %d meters\n", line + 1, policer_count, meter_count);
			status = -1;
			goto end;
		}

		if (policer_count == 0) {
			printf("the lack of any policers at line %d, required at least 1\n",
					line + 1);
			status = -1;
			goto end;
		} else if (policer_count > PIPELINE_FA_N_TC_MAX) {
			printf("too much instances of policers at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (meter_count == 0) {
			printf("the lack of any meters at line %d, required at least 1\n",
					line + 1);
			status = -1;
			goto end;
		} else if (meter_count > PIPELINE_FA_N_TC_MAX) {
			printf("too much instances of meters at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		/* set up all meters from the parse file */
		for (j = 0; j < meter_count; j++) {

			status = strcmp(tokens[pos++], "meter");
			if (status != 0) {
				printf("not found keyword \'meter\' at line %d.\n", line + 1);
				status = -1;
				goto end;
			}

			status = parser_read_uint32(&num_value, tokens[pos]);
			if (status != 0) {
				printf("conversion error meter id: \'%s\' at line %d\n",
						tokens[pos], line + 1);
				status = -1;
				goto end;
			}

			if (num_value > PIPELINE_FA_N_TC_MAX - 1) {
				printf("meter id %d at line %d is not in the range <0,3>\n",
						num_value, line + 1);
				status = -1;
				goto end;
			}

			if (id_mask & (1 << num_value)) {
				printf("there were 2 the same ID for meters at line %d",
						line + 1);
				status = -1;
				goto end;
			}
			id_mask |= 1 << num_value;
			pos++;

			struct rte_meter_trtcm_params *m = &params->keys[i].m[num_value];

			status = parser_read_uint64(&num_value64, tokens[pos]);
			if (status != 0) {
				printf("conversion error cir: \'%s\' at line %d\n",
						tokens[pos], line + 1);
				status = -1;
				goto end;
			}
			m->cir = num_value64;
			pos++;

			status = parser_read_uint64(&num_value64, tokens[pos]);
			if (status != 0) {
				printf("conversion error pir: \'%s\' at line %d\n",
						tokens[pos], line + 1);
				status = -1;
				goto end;
			}
			m->pir = num_value64;
			pos++;

			status = parser_read_uint64(&num_value64, tokens[pos]);
			if (status != 0) {
				printf("conversion error cbs: \'%s\' at line %d\n",
						tokens[pos], line + 1);
				status = -1;
				goto end;
			}
			m->cbs = num_value64;
			pos++;

			status = parser_read_uint64(&num_value64, tokens[pos]);
			if (status != 0) {
				printf("conversion error pbs: \'%s\' at line %d\n",
						tokens[pos], line + 1);
				status = -1;
				goto end;
			}
			m->pbs = num_value64;
			pos++;
		}

		id_mask = 0;
		/* set up all policers from the parse file */
		for (j = 0; j < policer_count; j++) {

			status = strcmp(tokens[pos++], "policer");
			if (status != 0) {
				printf("not found keyword \'policer\' at line %d.\n", line + 1);
				status = -1;
				goto end;
			}

			status = parser_read_uint32(&num_value, tokens[pos]);
			if (status != 0) {
				printf("conversion error policer id: \'%s\' at line %d\n",
						tokens[pos], line + 1);
				status = -1;
				goto end;
			}

			if (num_value > PIPELINE_FA_N_TC_MAX - 1) {
				printf("policer id %d at line %d is not in the range <0,3>\n",
						num_value, line + 1);
				status = -1;
				goto end;
			}

			if (id_mask & (1 << num_value)) {
				printf("there were 2 the same ID for policers at line %d",
						line + 1);
				status = -1;
				goto end;
			}
			id_mask |= 1 << num_value;

			struct pipeline_fa_policer_params *p =
												&params->keys[i].p[num_value];
			pos++;

			uint8_t k;

			for (k = 0; k < e_RTE_METER_COLORS; k++) {

				struct pipeline_fa_policer_action *a = &p->action[k];

				status = string_to_policer_action(tokens[pos], a);

				if (status != 0) {
					printf("there was a problem with the set up"
							" policer with id %d at line %d with action: %s\n",
							num_value, line + 1, tokens[pos]);
					status = 1;
					goto end;
				}
				pos++;
			}
		}

		status = strcmp(tokens[pos++], "port");
		if (status != 0) {
			printf("not found keyword \'port\' at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		status = parser_read_uint32(&num_value, tokens[pos]);
		if (status != 0) {
			printf("conversion error port id: \'%s\' at line %d\n",
					tokens[pos], line + 1);
			status = -1;
			goto end;
		}
		params->keys[i].port_id = num_value;

		line++;
		i++;
	}

end:
	fclose(file);
	return status;
}

static void
print_flow(struct app_pipeline_fa *p,
	uint32_t flow_id,
	struct app_pipeline_fa_flow *flow)
{
	uint32_t i;

	printf("Flow ID = %" PRIu32 "\n", flow_id);

	for (i = 0; i < p->params.n_meters_per_flow; i++) {
		struct rte_meter_trtcm_params *meter = &flow->params.m[i];
		struct pipeline_fa_policer_params *policer = &flow->params.p[i];

	printf("\ttrTCM [CIR = %" PRIu64
		", CBS = %" PRIu64 ", PIR = %" PRIu64
		", PBS = %" PRIu64	"] Policer [G : %s, Y : %s, R : %s]\n",
		meter->cir,
		meter->cbs,
		meter->pir,
		meter->pbs,
		policer_action_to_string(&policer->action[e_RTE_METER_GREEN]),
		policer_action_to_string(&policer->action[e_RTE_METER_YELLOW]),
		policer_action_to_string(&policer->action[e_RTE_METER_RED]));
	}

	printf("\tPort %u (entry_ptr = %p)\n",
		flow->params.port_id,
		flow->entry_ptr);
}


static int
app_pipeline_fa_flow_ls(struct app_params *app,
		uint32_t pipeline_id)
{
	struct app_pipeline_fa *p;
	uint32_t i;

	/* Check input arguments */
	if (app == NULL)
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id,
		&pipeline_flow_actions);
	if (p == NULL)
		return -1;

	for (i = 0; i < p->params.n_flows; i++) {
		struct app_pipeline_fa_flow *flow = &p->flows[i];

		print_flow(p, i, flow);
	}

	return 0;
}

static int
app_pipeline_fa_dscp_ls(struct app_params *app,
		uint32_t pipeline_id)
{
	struct app_pipeline_fa *p;
	uint32_t i;

	/* Check input arguments */
	if (app == NULL)
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id,
		&pipeline_flow_actions);
	if (p == NULL)
		return -1;

	if (p->params.dscp_enabled == 0)
		return -1;

	for (i = 0; i < RTE_DIM(p->dscp); i++) {
		struct app_pipeline_fa_dscp *dscp =	&p->dscp[i];

		printf("DSCP = %2" PRIu32 ": Traffic class = %" PRIu32
			", Color = %s\n",
			i,
			dscp->traffic_class,
			color_to_string(dscp->color));
	}

	return 0;
}

/*
 * action cmd
 */

struct cmd_action_result {
	cmdline_fixed_string_t p_string;
	uint32_t pipeline_id;
	cmdline_fixed_string_t action_string;
	cmdline_multi_string_t multi_string;
};

static void
cmd_action_parsed(
	void *parsed_result,
	__rte_unused struct cmdline *cl,
	void *data)
{
	struct cmd_action_result *params = parsed_result;
	struct app_params *app = data;

	char *tokens[16];
	uint32_t n_tokens = RTE_DIM(tokens);
	int status;

	status = parse_tokenize_string(params->multi_string, tokens, &n_tokens);
	if (status != 0) {
		printf("Command \"action\": Too many parameters.\n");
		return;
	}

	/* "action flow" or "action flow bulk" or "action flow ls" */
	if ((n_tokens > 0) && (0 == strcmp(tokens[0], "flow"))) {
		struct pipeline_fa_flow_params flow_params;
		uint32_t flow_id;

		if (n_tokens < 2) {
			printf("Not enough parameters for \"action flow\" or"
					" \"action flow bulk\" or \"action flow ls\".\n");
			return;
		}

		/* action flow */
		if (0 == parser_read_uint32(&flow_id, tokens[1])) {
			if (n_tokens < 3) {
				printf("Not enough parameters for \"action flow\".\n");
				return;
			}

			/*
			 * Flow meter configuration (single flow)
			 *
			 * p <pipeline ID> action flow <flow ID> meter <meter ID>
			 * trtcm <cir> <pir> <cbs> <pbs>
			 */
			if (0 == strcmp(tokens[2], "meter")) {
				uint32_t meter_id;

				if (n_tokens != 9) {
					printf("Incorrect number of parameters for action flow"
						" meter configuration.\n");
					return;
				}

				if (parser_read_uint32(&meter_id, tokens[3])) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is meter id.\n",
						tokens[3]);
					return;
				}
				if (meter_id >= PIPELINE_FA_N_TC_MAX) {
					printf("Incorrect parameter: \"%s\"."
						" Expected meter id is less than %d.\n",
						tokens[3], PIPELINE_FA_N_TC_MAX);
					return;
				}
				if (strcmp(tokens[4], "trtcm")) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is word \"trtcm\".\n",
						tokens[4]);
					return;
				}
				if (parser_read_uint64(&flow_params.m[meter_id].cir,
					tokens[5])) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is cir.\n",
						tokens[5]);
					return;
				}
				if (parser_read_uint64(&flow_params.m[meter_id].pir,
					tokens[6])) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is pir.\n",
						tokens[6]);
					return;
				}
				if (parser_read_uint64(&flow_params.m[meter_id].cbs,
					tokens[7])) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is cbs.\n",
						tokens[7]);
					return;
				}
				if (parser_read_uint64(&flow_params.m[meter_id].pbs,
					tokens[8])) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is pbs.\n",
						tokens[8]);
					return;
				}

				status = app_pipeline_fa_flow_config(app,
					params->pipeline_id,
					flow_id,
					1 << meter_id,
					0,
					0,
					&flow_params);
				if (status != 0)
					printf("Command \"action flow meter\" failed.\n");

				return;
			}
			/*
			 * Flow policer configuration (single flow)
			 *
			 * p <pipeline ID> action flow <flow ID> policer <policer ID>
			 *    g <action> y <action> r <action>
			 *
			 * <action> = G (green) | Y (yellow) | R (red) | D (drop)
			 */
			else if (0 == strcmp(tokens[2], "policer")) {
				uint32_t policer_id;

				if (n_tokens != 10) {
					printf("Incorrect number of parameters for action flow"
						" policer configuration.\n");
					return;
				}

				if (parser_read_uint32(&policer_id, tokens[3])) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is policer id.\n",
						tokens[3]);
					return;
				}
				if (policer_id >= PIPELINE_FA_N_TC_MAX) {
					printf("Incorrect parameter: \"%s\"."
						" Expected policer id is less than %d.\n",
						tokens[3], PIPELINE_FA_N_TC_MAX);
					return;
				}
				if (strcmp(tokens[4], "g")) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is word \"g\".\n",
						tokens[4]);
					return;
				}
				if (string_to_policer_action(tokens[5],
					&flow_params.p[policer_id].action[e_RTE_METER_GREEN])) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is policer green action.\n",
						tokens[5]);
					return;
				}
				if (strcmp(tokens[6], "y")) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is word \"y\".\n",
						tokens[6]);
					return;
				}
				if (string_to_policer_action(tokens[7],
					&flow_params.p[policer_id].action[e_RTE_METER_YELLOW])) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is policer yellow action.\n",
						tokens[7]);
					return;
				}
				if (strcmp(tokens[8], "r")) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is word \"r\".\n",
						tokens[8]);
					return;
				}
				if (string_to_policer_action(tokens[9],
					&flow_params.p[policer_id].action[e_RTE_METER_RED])) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is policer red action.\n",
						tokens[9]);
					return;
				}

				status = app_pipeline_fa_flow_config(app,
					params->pipeline_id,
					flow_id,
					0,
					1 << policer_id,
					0,
					&flow_params);
				if (status != 0)
					printf("Command \"action flow policer\" failed.\n");

				return;
			}
			/*
			 * Flow output port configuration (single flow)
			 *
			 * p <pipeline ID> action flow <flow ID> port <port ID>
			 */
			else if (0 == strcmp(tokens[2], "port")) {
				if (n_tokens != 4) {
					printf("Incorrect number of parameters for action flow"
						" output port configuration.\n");
					return;
				}

				if (parser_read_uint32(&flow_params.port_id, tokens[3])) {
					printf("Incorrect parameter: \"%s\"."
						" Expected parameter is port id.\n",
						tokens[3]);
					return;
				}

				status = app_pipeline_fa_flow_config(app,
					params->pipeline_id,
					flow_id,
					0,
					0,
					1,
					&flow_params);
				if (status != 0)
					printf("Command \"action flow port\" failed.\n");

				return;
			}
			/*
			 * Flow policer stats read
			 *
			 * p <pipeline ID> action flow <flow ID> stats
			 */
			else if (0 == strcmp(tokens[2], "stats")) {
				struct pipeline_fa_policer_stats stats;
				uint32_t policer_id;

				if (n_tokens != 3) {
					printf("Incorrect number of parameters for action flow"
						" policer stats read.\n");
					return;
				}

				for (policer_id = 0;
					policer_id < PIPELINE_FA_N_TC_MAX;
					policer_id++) {
					status = app_pipeline_fa_flow_policer_stats_read(app,
						params->pipeline_id,
						flow_id,
						policer_id,
						1,
						&stats);
					if (status != 0) {
						printf("Command \"action flow stats\" for policer %"
							PRIu32 " failed.\n", policer_id);
						return;
					}

					/* Display stats */
					printf("\tPolicer: %" PRIu32
						"\tPkts G: %" PRIu64
						"\tPkts Y: %" PRIu64
						"\tPkts R: %" PRIu64
						"\tPkts D: %" PRIu64 "\n",
						policer_id,
						stats.n_pkts[e_RTE_METER_GREEN],
						stats.n_pkts[e_RTE_METER_YELLOW],
						stats.n_pkts[e_RTE_METER_RED],
						stats.n_pkts_drop);
				}

				return;
			}
			else {
				printf("Incorrect parameter: \"%s\"."
					" Expected parameter is word \"meter\" or \"policer\" or"
					" \"port\" or \"stats\".\n",
					tokens[2]);
				return;
			}
		}
		/*
		 * Flow meter, policer and output port configuration (multiple flows)
		 *
		 * p <pipeline ID> action flow bulk <file>
		 */
		else if (0 == strcmp(tokens[1], "bulk")) {
			struct app_pipeline_add_bulk_params bulk_params;

			if (n_tokens != 3) {
				printf(
					"Incorrect number of parameters for action flow bulk.\n");
				return;
			}

			bulk_params.keys = NULL;
			bulk_params.flow_ids = NULL;

			status = app_pipeline_fa_add_bulk_parse_file(tokens[2],
				&bulk_params);
			if (status != 0) {
				printf("Command \"action flow bulk\" failed.\n");
				rte_free(bulk_params.keys);
				rte_free(bulk_params.flow_ids);
				return;
			}

			status = app_pipeline_fa_flow_config_bulk(app,
				params->pipeline_id,
				bulk_params.flow_ids,
				bulk_params.n_keys,
				0x0F,
				0x0F,
				1,
				bulk_params.keys);
			if (status != 0)
				printf("Command \"action flow bulk\" failed.\n");

			rte_free(bulk_params.keys);
			rte_free(bulk_params.flow_ids);
			return;
		}
		/*
		 * Flow list
		 *
		 * p <pipeline ID> action flow ls
		 */
		else if (0 == strcmp(tokens[1], "ls")) {
			if (n_tokens != 2) {
				printf("Incorrect number of parameters for action flow ls.\n");
				return;
			}

			status = app_pipeline_fa_flow_ls(app, params->pipeline_id);
			if (status != 0)
				printf("Command \"action flow ls\" failed.\n");

			return;
		}
		else {
			printf("Incorrect parameter: \"%s\"."
				" Expected parameter is flow id or word \"bulk\" or"
				" word \"ls\".\n",
				tokens[1]);
			return;
		}
	}

	/* "action dscp" or "action dscp ls" */
	if ((n_tokens > 0) && (0 == strcmp(tokens[0], "dscp"))) {
		uint32_t dscp_id;

		if (n_tokens < 2) {
			printf("Not enough parameters for \"action dscp\" or"
					" \"action dscp ls\".\n");
			return;
		}

		/*
		 * Flow DiffServ Code Point (DSCP) translation table configuration
		 *
		 * p <pipeline ID> action dscp <DSCP ID> class <traffic class ID>
		 * color <color>
		 *
		 * <color> = G (green) | Y (yellow) | R (red)
		 */
		if (0 == parser_read_uint32(&dscp_id, tokens[1])) {
			enum rte_meter_color color;
			uint32_t traffic_class_id;

			if (n_tokens != 6) {
				printf("Incorrect number of parameters for action dscp.\n");
				return;
			}

			if (strcmp(tokens[2], "class")) {
				printf("Incorrect parameter: \"%s\"."
					" Expected parameter is word \"class\".\n",
					tokens[2]);
				return;
			}
			if (parser_read_uint32(&traffic_class_id, tokens[3])) {
				printf("Incorrect parameter: \"%s\"."
					" Expected parameter is traffic class id.\n",
					tokens[3]);
				return;
			}
			if (strcmp(tokens[4], "color")) {
				printf("Incorrect parameter: \"%s\"."
					" Expected parameter is word \"color\".\n",
					tokens[4]);
				return;
			}
			if (string_to_color(tokens[5], &color)) {
				printf("Incorrect parameter: \"%s\"."
					" Expected parameter is color.\n",
					tokens[5]);
				return;
			}

			status = app_pipeline_fa_dscp_config(app,
				params->pipeline_id,
				dscp_id,
				traffic_class_id,
				color);
			if (status != 0)
				printf("Command \"action dscp\" failed.\n");

			return;
		}
		/*
		 * Flow DiffServ Code Point (DSCP) translation table list
		 *
		 * p <pipeline ID> action dscp ls
		 */
		else if (0 == strcmp(tokens[1], "ls")) {
			if (n_tokens != 2) {
				printf("Incorrect number of parameters for action dscp ls.\n");
				return;
			}

			status = app_pipeline_fa_dscp_ls(app, params->pipeline_id);
			if (status != 0)
				printf("Command \"action dscp ls\" failed.\n");

			return;
		}
		else {
			printf("Incorrect parameter: \"%s\"."
				" Expected parameter is dscp id or word \"ls\".\n",
				tokens[1]);
			return;
		}
	}

	printf("!Command \"action\" failed."
		" Expected parameter is \"flow\" or \"dscp\".\n");
	return;
}

static cmdline_parse_token_string_t cmd_action_p_string =
	TOKEN_STRING_INITIALIZER(struct cmd_action_result, p_string, "p");

static cmdline_parse_token_num_t cmd_action_pipeline_id =
	TOKEN_NUM_INITIALIZER(struct cmd_action_result, pipeline_id, UINT32);

static cmdline_parse_token_string_t cmd_action_action_string =
	TOKEN_STRING_INITIALIZER(struct cmd_action_result, action_string, "action");

static cmdline_parse_token_string_t cmd_action_multi_string =
	TOKEN_STRING_INITIALIZER(struct cmd_action_result, multi_string,
	TOKEN_STRING_MULTI);

cmdline_parse_inst_t cmd_action = {
	.f = cmd_action_parsed,
	.data = NULL,
	.help_str =
		"\n Flow meter configuration (single flow):"
		"\n p <pipeline ID> action flow <flow ID> meter <meter ID> trtcm <cir>"
		" <pir> <cbs> <pbs>"
		"\n Flow policer configuration (single flow):"
		"\n p <pipeline ID> action flow <flow ID> policer <policer ID> g"
		" <action> y <action> r <action>"
		"\n Flow output port configuration (single flow):"
		"\n p <pipeline ID> action flow <flow ID> port <port ID>"
		"\n Flow meter, policer and output port configuration (multiple flows):"
		"\n p <pipeline ID> action flow bulk <file>"
		"\n Flow policer stats read:"
		"\n p <pipeline ID> action flow <flow ID> stats"
		"\n Flow actions list:"
		"\n p <pipeline ID> action flow ls"
		"\n Flow DSCP translation table configuration:"
		"\n p <pipeline ID> action dscp <dscp ID> class <class ID> color"
		" <color>"
		"\n Flow DSCP translaton table list:"
		"\n p <pipeline ID> action dscp ls",
	.tokens = {
		(void *) &cmd_action_p_string,
		(void *) &cmd_action_pipeline_id,
		(void *) &cmd_action_action_string,
		(void *) &cmd_action_multi_string,
		NULL,
	},
};

static cmdline_parse_ctx_t pipeline_cmds[] = {
	(cmdline_parse_inst_t *) &cmd_action,
	NULL,
};

static struct pipeline_fe_ops pipeline_flow_actions_fe_ops = {
	.f_init = app_pipeline_fa_init,
	.f_free = app_pipeline_fa_free,
	.cmds = pipeline_cmds,
};

struct pipeline_type pipeline_flow_actions = {
	.name = "FLOW_ACTIONS",
	.be_ops = &pipeline_flow_actions_be_ops,
	.fe_ops = &pipeline_flow_actions_fe_ops,
};
