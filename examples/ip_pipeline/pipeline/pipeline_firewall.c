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
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/queue.h>
#include <netinet/in.h>

#include <rte_common.h>
#include <rte_hexdump.h>
#include <rte_malloc.h>
#include <cmdline_rdline.h>
#include <cmdline_parse.h>
#include <cmdline_parse_num.h>
#include <cmdline_parse_string.h>
#include <cmdline_parse_ipaddr.h>
#include <cmdline_parse_etheraddr.h>
#include <cmdline_socket.h>

#include "app.h"
#include "pipeline_common_fe.h"
#include "pipeline_firewall.h"
#include "parser.h"


#define BUF_SIZE		1024

struct app_pipeline_firewall_rule {
	struct pipeline_firewall_key key;
	int32_t priority;
	uint32_t port_id;
	void *entry_ptr;

	TAILQ_ENTRY(app_pipeline_firewall_rule) node;
};

struct app_pipeline_firewall {
	/* parameters */
	uint32_t n_ports_in;
	uint32_t n_ports_out;

	/* rules */
	TAILQ_HEAD(, app_pipeline_firewall_rule) rules;
	uint32_t n_rules;
	uint32_t default_rule_present;
	uint32_t default_rule_port_id;
	void *default_rule_entry_ptr;
};

struct app_pipeline_add_bulk_params {
	struct pipeline_firewall_key *keys;
	uint32_t n_keys;
	uint32_t *priorities;
	uint32_t *port_ids;
};

struct app_pipeline_del_bulk_params {
	struct pipeline_firewall_key *keys;
	uint32_t n_keys;
};

static void
print_firewall_ipv4_rule(struct app_pipeline_firewall_rule *rule)
{
	printf("Prio = %" PRId32 " (SA = %" PRIu32 ".%" PRIu32
		".%" PRIu32 ".%" PRIu32 "/%" PRIu32 ", "
		"DA = %" PRIu32 ".%" PRIu32
		".%"PRIu32 ".%" PRIu32 "/%" PRIu32 ", "
		"SP = %" PRIu32 "-%" PRIu32 ", "
		"DP = %" PRIu32 "-%" PRIu32 ", "
		"Proto = %" PRIu32 " / 0x%" PRIx32 ") => "
		"Port = %" PRIu32 " (entry ptr = %p)\n",

		rule->priority,

		(rule->key.key.ipv4_5tuple.src_ip >> 24) & 0xFF,
		(rule->key.key.ipv4_5tuple.src_ip >> 16) & 0xFF,
		(rule->key.key.ipv4_5tuple.src_ip >> 8) & 0xFF,
		rule->key.key.ipv4_5tuple.src_ip & 0xFF,
		rule->key.key.ipv4_5tuple.src_ip_mask,

		(rule->key.key.ipv4_5tuple.dst_ip >> 24) & 0xFF,
		(rule->key.key.ipv4_5tuple.dst_ip >> 16) & 0xFF,
		(rule->key.key.ipv4_5tuple.dst_ip >> 8) & 0xFF,
		rule->key.key.ipv4_5tuple.dst_ip & 0xFF,
		rule->key.key.ipv4_5tuple.dst_ip_mask,

		rule->key.key.ipv4_5tuple.src_port_from,
		rule->key.key.ipv4_5tuple.src_port_to,

		rule->key.key.ipv4_5tuple.dst_port_from,
		rule->key.key.ipv4_5tuple.dst_port_to,

		rule->key.key.ipv4_5tuple.proto,
		rule->key.key.ipv4_5tuple.proto_mask,

		rule->port_id,
		rule->entry_ptr);
}

static struct app_pipeline_firewall_rule *
app_pipeline_firewall_rule_find(struct app_pipeline_firewall *p,
	struct pipeline_firewall_key *key)
{
	struct app_pipeline_firewall_rule *r;

	TAILQ_FOREACH(r, &p->rules, node)
		if (memcmp(key,
			&r->key,
			sizeof(struct pipeline_firewall_key)) == 0)
			return r;

	return NULL;
}

static int
app_pipeline_firewall_ls(
	struct app_params *app,
	uint32_t pipeline_id)
{
	struct app_pipeline_firewall *p;
	struct app_pipeline_firewall_rule *rule;
	uint32_t n_rules;
	int priority;

	/* Check input arguments */
	if (app == NULL)
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id, &pipeline_firewall);
	if (p == NULL)
		return -1;

	n_rules = p->n_rules;
	for (priority = 0; n_rules; priority++)
		TAILQ_FOREACH(rule, &p->rules, node)
			if (rule->priority == priority) {
				print_firewall_ipv4_rule(rule);
				n_rules--;
			}

	if (p->default_rule_present)
		printf("Default rule: port %" PRIu32 " (entry ptr = %p)\n",
			p->default_rule_port_id,
			p->default_rule_entry_ptr);
	else
		printf("Default rule: DROP\n");

	printf("\n");

	return 0;
}

static void*
app_pipeline_firewall_init(struct pipeline_params *params,
	__rte_unused void *arg)
{
	struct app_pipeline_firewall *p;
	uint32_t size;

	/* Check input arguments */
	if ((params == NULL) ||
		(params->n_ports_in == 0) ||
		(params->n_ports_out == 0))
		return NULL;

	/* Memory allocation */
	size = RTE_CACHE_LINE_ROUNDUP(sizeof(struct app_pipeline_firewall));
	p = rte_zmalloc(NULL, size, RTE_CACHE_LINE_SIZE);
	if (p == NULL)
		return NULL;

	/* Initialization */
	p->n_ports_in = params->n_ports_in;
	p->n_ports_out = params->n_ports_out;

	TAILQ_INIT(&p->rules);
	p->n_rules = 0;
	p->default_rule_present = 0;
	p->default_rule_port_id = 0;
	p->default_rule_entry_ptr = NULL;

	return (void *) p;
}

static int
app_pipeline_firewall_free(void *pipeline)
{
	struct app_pipeline_firewall *p = pipeline;

	/* Check input arguments */
	if (p == NULL)
		return -1;

	/* Free resources */
	while (!TAILQ_EMPTY(&p->rules)) {
		struct app_pipeline_firewall_rule *rule;

		rule = TAILQ_FIRST(&p->rules);
		TAILQ_REMOVE(&p->rules, rule, node);
		rte_free(rule);
	}

	rte_free(p);
	return 0;
}

static int
app_pipeline_firewall_key_check_and_normalize(struct pipeline_firewall_key *key)
{
	switch (key->type) {
	case PIPELINE_FIREWALL_IPV4_5TUPLE:
	{
		uint32_t src_ip_depth = key->key.ipv4_5tuple.src_ip_mask;
		uint32_t dst_ip_depth = key->key.ipv4_5tuple.dst_ip_mask;
		uint16_t src_port_from = key->key.ipv4_5tuple.src_port_from;
		uint16_t src_port_to = key->key.ipv4_5tuple.src_port_to;
		uint16_t dst_port_from = key->key.ipv4_5tuple.dst_port_from;
		uint16_t dst_port_to = key->key.ipv4_5tuple.dst_port_to;

		uint32_t src_ip_netmask = 0;
		uint32_t dst_ip_netmask = 0;

		if ((src_ip_depth > 32) ||
			(dst_ip_depth > 32) ||
			(src_port_from > src_port_to) ||
			(dst_port_from > dst_port_to))
			return -1;

		if (src_ip_depth)
			src_ip_netmask = (~0U) << (32 - src_ip_depth);

		if (dst_ip_depth)
			dst_ip_netmask = ((~0U) << (32 - dst_ip_depth));

		key->key.ipv4_5tuple.src_ip &= src_ip_netmask;
		key->key.ipv4_5tuple.dst_ip &= dst_ip_netmask;

		return 0;
	}

	default:
		return -1;
	}
}

static int
app_pipeline_add_bulk_parse_file(char *filename,
		struct app_pipeline_add_bulk_params *params)
{
	FILE *f = NULL;
	char file_buf[BUF_SIZE];
	uint32_t i, line = 0;
	int status = 0;

	f = fopen(filename, "r");
	if (f == NULL) {
		if (getcwd(file_buf, sizeof(file_buf)) != NULL)
			printf("not found file %s in the current working dir %s\n",
					filename, file_buf);
		return -1;
	}

	params->n_keys = 0;
	while (fgets(file_buf, BUF_SIZE, f) != NULL)
		if (file_buf[0] != '\0' && file_buf[0] != '\n' && file_buf[0] != '#')
			params->n_keys++;
	rewind(f);

	if (params->n_keys == 0) {
		printf("not found any keys in the file %s\n", filename);
		status = -1;
		goto end;
	}

	params->keys = rte_zmalloc(NULL,
			params->n_keys * sizeof(struct pipeline_firewall_key),
			RTE_CACHE_LINE_SIZE);
	if (params->keys == NULL) {
		status = -1;
		goto end;
	}

	params->priorities = rte_malloc(NULL,
			params->n_keys * sizeof(uint32_t),
			RTE_CACHE_LINE_SIZE);
	if (params->priorities == NULL) {
		status = -1;
		goto end;
	}

	params->port_ids = rte_malloc(NULL,
			params->n_keys * sizeof(uint32_t),
			RTE_CACHE_LINE_SIZE);
	if (params->port_ids == NULL) {
		status = -1;
		goto end;
	}

	i = 0;
	while (fgets(file_buf, BUF_SIZE, f) != NULL) {
		char *tokens[17];
		uint32_t n_tokens = RTE_DIM(tokens);
		status = parse_tokenize_string(file_buf, tokens, &n_tokens);
		uint32_t priority = 0;
		struct in_addr sipaddr;
		uint32_t sipdepth = 0;
		struct in_addr dipaddr;
		uint32_t dipdepth = 0;
		uint16_t sport0 = 0;
		uint16_t sport1 = 0;
		uint16_t dport0 = 0;
		uint16_t dport1 = 0;
		uint8_t proto = 0;
		uint8_t protomask = 0;
		uint32_t port_id = 0;

		if (n_tokens == 0 || tokens[0][0] == '#') {
			line++;
			continue;
		}

		if (status != 0) {
			printf("Too many tokens\n");
			status = -1;
			goto end;
		}

		if (n_tokens != 15) {
			printf("Bad number of tokens at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (strcmp(tokens[0], "priority") != 0) {
			printf("Incorrect string for priority at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint32(&priority, tokens[1])) {
			printf("Incorrect format for priority at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (strcmp(tokens[2], "ipv4")) {
			status = -1;
			goto end;
		}

		if (parse_ipv4_addr(tokens[3], &sipaddr)) {
			printf("Incorrect format for sipaddr at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint32(&sipdepth, tokens[4])) {
			printf("Incorrect format for sipdepth at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parse_ipv4_addr(tokens[5], &dipaddr)) {
			printf("Incorrect format for dipaddr at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint32(&dipdepth, tokens[6])) {
			printf("Incorrect format for dipdepth at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint16(&sport0, tokens[7])) {
			printf("Incorrect format for sport0 at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint16(&sport1, tokens[8])) {
			printf("Incorrect format for sport1 at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint16(&dport0, tokens[9])) {
			printf("Incorrect format for dport0 at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint16(&dport1, tokens[10])) {
			printf("Incorrect format for dport1 at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint8(&proto, tokens[11])) {
			printf("Incorrect format for proto at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		char *e;
		protomask = strtoul(tokens[12], &e, 16);
		if (*e || errno == EINVAL || errno == ERANGE) {
			printf("Incorrect format for protomask - should be in hex  at line"
					" %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (strcmp(tokens[13], "port")) {
			printf("string supported port at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint32(&port_id, tokens[14])) {
			printf("Incorrect format for port ID at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		params->keys[i].type = PIPELINE_FIREWALL_IPV4_5TUPLE;
		params->keys[i].key.ipv4_5tuple.src_ip = rte_bswap32(
			(uint32_t) sipaddr.s_addr);
		params->keys[i].key.ipv4_5tuple.src_ip_mask = sipdepth;
		params->keys[i].key.ipv4_5tuple.dst_ip = rte_bswap32(
			(uint32_t) dipaddr.s_addr);
		params->keys[i].key.ipv4_5tuple.dst_ip_mask = dipdepth;
		params->keys[i].key.ipv4_5tuple.src_port_from = sport0;
		params->keys[i].key.ipv4_5tuple.src_port_to = sport1;
		params->keys[i].key.ipv4_5tuple.dst_port_from = dport0;
		params->keys[i].key.ipv4_5tuple.dst_port_to = dport1;
		params->keys[i].key.ipv4_5tuple.proto = proto;
		params->keys[i].key.ipv4_5tuple.proto_mask = protomask;

		params->port_ids[i] = port_id;
		params->priorities[i] = priority;

		line++;
		i++;


	}

end:
	fclose(f);
	return status;
}

static int
app_pipeline_del_bulk_parse_file(char *filename,
		struct app_pipeline_del_bulk_params *params)
{
	FILE *f = NULL;
	char file_buf[BUF_SIZE];
	uint32_t i, line = 0;
	int status = 0;

	f = fopen(filename, "r");
	if (f == NULL) {
		if (getcwd(file_buf, sizeof(file_buf)) != NULL)
			printf("not found file %s in the current working dir %s\n",
					filename, file_buf);
		return -1;
	}

	params->n_keys = 0;
	while (fgets(file_buf, BUF_SIZE, f) != NULL)
		if (file_buf[0] != '\0' && file_buf[0] != '\n' && file_buf[0] != '#')
			params->n_keys++;
	rewind(f);

	if (params->n_keys == 0) {
		printf("not found any keys in the file %s\n", filename);
		status = -1;
		goto end;
	}

	params->keys = rte_zmalloc(NULL,
			params->n_keys * sizeof(struct pipeline_firewall_key),
			RTE_CACHE_LINE_SIZE);
	if (params->keys == NULL) {
		status = -1;
		goto end;
	}

	i = 0;
	while (fgets(file_buf, BUF_SIZE, f) != NULL) {
		char *tokens[17];
		uint32_t n_tokens = RTE_DIM(tokens);
		status = parse_tokenize_string(file_buf, tokens, &n_tokens);
		struct in_addr sipaddr;
		uint32_t sipdepth = 0;
		struct in_addr dipaddr;
		uint32_t dipdepth;
		uint16_t sport0 = 0;
		uint16_t sport1 = 0;
		uint16_t dport0 = 0;
		uint16_t dport1 = 0;
		uint8_t proto = 0;
		uint8_t protomask = 0;

		if (n_tokens == 0 || tokens[0][0] == '#') {
					line++;
					continue;
				}

		if (status != 0) {
			printf("Too many tokens\n");
			status = -1;
			goto end;
		}

		if (n_tokens != 11) {
			status = -1;
			printf("\nToo many tokens in bulk file at line %d\n", line + 1);
			goto end;
		}

		if (strcmp(tokens[0], "ipv4")) {
			printf("\nThe line should begin with string 'ipv4' %d\n", line + 1);
			status = -1;
			goto end;		}

		if (parse_ipv4_addr(tokens[1], &sipaddr)) {
			printf("\nIncorrect format for sipaddr at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint32(&sipdepth, tokens[2])) {
			printf("\nIncorrect format for sipdepth at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parse_ipv4_addr(tokens[3], &dipaddr)) {
			printf("\nIncorrect format for dipaddr at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint32(&dipdepth, tokens[4])) {
			printf("\nIncorrect format for dipdepth at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint16(&sport0, tokens[5])) {
			printf("\nIncorrect format for sport0 at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint16(&sport1, tokens[6])) {
			printf("\nIncorrect format for sport1 at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint16(&dport0, tokens[7])) {
			printf("\nIncorrect format for dport0 at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint16(&dport1, tokens[8])) {
			printf("\nIncorrect format for dport1 at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		if (parser_read_uint8(&proto, tokens[9])) {
			printf("\nIncorrect format for proto at line %d\n", line + 1);
			status = -1;
			goto end;
		}

		char *e;
		protomask = strtoul(tokens[10], &e, 16);
		if (*e || errno == EINVAL || errno == ERANGE) {
			printf("\nIncorrect format for protomask - should be in hex  at "
					"line %d\n", line + 1);
			status = -1;
			goto end;
		}


		params->keys[i].type = PIPELINE_FIREWALL_IPV4_5TUPLE;
		params->keys[i].key.ipv4_5tuple.src_ip = rte_bswap32(
			(uint32_t) sipaddr.s_addr);
		params->keys[i].key.ipv4_5tuple.src_ip_mask = sipdepth;
		params->keys[i].key.ipv4_5tuple.dst_ip = rte_bswap32(
			(uint32_t) dipaddr.s_addr);
		params->keys[i].key.ipv4_5tuple.dst_ip_mask = dipdepth;
		params->keys[i].key.ipv4_5tuple.src_port_from = sport0;
		params->keys[i].key.ipv4_5tuple.src_port_to = sport1;
		params->keys[i].key.ipv4_5tuple.dst_port_from = dport0;
		params->keys[i].key.ipv4_5tuple.dst_port_to = dport1;
		params->keys[i].key.ipv4_5tuple.proto = proto;
		params->keys[i].key.ipv4_5tuple.proto_mask = protomask;

		line++;
		i++;

	}

	for (i = 0; i < params->n_keys; i++) {
		if (app_pipeline_firewall_key_check_and_normalize(&params->keys[i]) != 0) {
			status = -1;
			goto end;
		}
	}

end:
	fclose(f);
	return status;
}

int
app_pipeline_firewall_add_rule(struct app_params *app,
	uint32_t pipeline_id,
	struct pipeline_firewall_key *key,
	uint32_t priority,
	uint32_t port_id)
{
	struct app_pipeline_firewall *p;
	struct app_pipeline_firewall_rule *rule;
	struct pipeline_firewall_add_msg_req *req;
	struct pipeline_firewall_add_msg_rsp *rsp;
	int new_rule;

	/* Check input arguments */
	if ((app == NULL) ||
		(key == NULL) ||
		(key->type != PIPELINE_FIREWALL_IPV4_5TUPLE))
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id, &pipeline_firewall);
	if (p == NULL)
		return -1;

	if (port_id >= p->n_ports_out)
		return -1;

	if (app_pipeline_firewall_key_check_and_normalize(key) != 0)
		return -1;

	/* Find existing rule or allocate new rule */
	rule = app_pipeline_firewall_rule_find(p, key);
	new_rule = (rule == NULL);
	if (rule == NULL) {
		rule = rte_malloc(NULL, sizeof(*rule), RTE_CACHE_LINE_SIZE);

		if (rule == NULL)
			return -1;
	}

	/* Allocate and write request */
	req = app_msg_alloc(app);
	if (req == NULL) {
		if (new_rule)
			rte_free(rule);
		return -1;
	}

	req->type = PIPELINE_MSG_REQ_CUSTOM;
	req->subtype = PIPELINE_FIREWALL_MSG_REQ_ADD;
	memcpy(&req->key, key, sizeof(*key));
	req->priority = priority;
	req->port_id = port_id;

	/* Send request and wait for response */
	rsp = app_msg_send_recv(app, pipeline_id, req, MSG_TIMEOUT_DEFAULT);
	if (rsp == NULL) {
		if (new_rule)
			rte_free(rule);
		return -1;
	}

	/* Read response and write rule */
	if (rsp->status ||
		(rsp->entry_ptr == NULL) ||
		((new_rule == 0) && (rsp->key_found == 0)) ||
		((new_rule == 1) && (rsp->key_found == 1))) {
		app_msg_free(app, rsp);
		if (new_rule)
			rte_free(rule);
		return -1;
	}

	memcpy(&rule->key, key, sizeof(*key));
	rule->priority = priority;
	rule->port_id = port_id;
	rule->entry_ptr = rsp->entry_ptr;

	/* Commit rule */
	if (new_rule) {
		TAILQ_INSERT_TAIL(&p->rules, rule, node);
		p->n_rules++;
	}

	print_firewall_ipv4_rule(rule);

	/* Free response */
	app_msg_free(app, rsp);

	return 0;
}

int
app_pipeline_firewall_delete_rule(struct app_params *app,
	uint32_t pipeline_id,
	struct pipeline_firewall_key *key)
{
	struct app_pipeline_firewall *p;
	struct app_pipeline_firewall_rule *rule;
	struct pipeline_firewall_del_msg_req *req;
	struct pipeline_firewall_del_msg_rsp *rsp;

	/* Check input arguments */
	if ((app == NULL) ||
		(key == NULL) ||
		(key->type != PIPELINE_FIREWALL_IPV4_5TUPLE))
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id, &pipeline_firewall);
	if (p == NULL)
		return -1;

	if (app_pipeline_firewall_key_check_and_normalize(key) != 0)
		return -1;

	/* Find rule */
	rule = app_pipeline_firewall_rule_find(p, key);
	if (rule == NULL)
		return 0;

	/* Allocate and write request */
	req = app_msg_alloc(app);
	if (req == NULL)
		return -1;

	req->type = PIPELINE_MSG_REQ_CUSTOM;
	req->subtype = PIPELINE_FIREWALL_MSG_REQ_DEL;
	memcpy(&req->key, key, sizeof(*key));

	/* Send request and wait for response */
	rsp = app_msg_send_recv(app, pipeline_id, req, MSG_TIMEOUT_DEFAULT);
	if (rsp == NULL)
		return -1;

	/* Read response */
	if (rsp->status || !rsp->key_found) {
		app_msg_free(app, rsp);
		return -1;
	}

	/* Remove rule */
	TAILQ_REMOVE(&p->rules, rule, node);
	p->n_rules--;
	rte_free(rule);

	/* Free response */
	app_msg_free(app, rsp);

	return 0;
}

int
app_pipeline_firewall_add_bulk(struct app_params *app,
		uint32_t pipeline_id,
		struct pipeline_firewall_key *keys,
		uint32_t n_keys,
		uint32_t *priorities,
		uint32_t *port_ids)
{
	struct app_pipeline_firewall *p;
	struct pipeline_firewall_add_bulk_msg_req *req;
	struct pipeline_firewall_add_bulk_msg_rsp *rsp;

	struct app_pipeline_firewall_rule **rules;
	int *new_rules;

	int *keys_found;
	void **entries_ptr;

	uint32_t i;
	int status = 0;

	/* Check input arguments */
	if (app == NULL)
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id, &pipeline_firewall);
	if (p == NULL)
		return -1;

	rules = rte_malloc(NULL,
			n_keys * sizeof(struct app_pipeline_firewall_rule *),
			RTE_CACHE_LINE_SIZE);
	if (rules == NULL)
		return -1;

	new_rules = rte_malloc(NULL,
			n_keys * sizeof(int),
			RTE_CACHE_LINE_SIZE);
	if (new_rules == NULL) {
		rte_free(rules);
		return -1;
	}

	/* check data integrity and add to rule list */
	for (i = 0; i < n_keys; i++) {
		if (port_ids[i]  >= p->n_ports_out) {
			rte_free(rules);
			rte_free(new_rules);
			return -1;
		}

		if (app_pipeline_firewall_key_check_and_normalize(&keys[i]) != 0) {
			rte_free(rules);
			rte_free(new_rules);
			return -1;
		}

		rules[i] = app_pipeline_firewall_rule_find(p, &keys[i]);
		new_rules[i] = (rules[i] == NULL);
		if (rules[i] == NULL) {
			rules[i] = rte_malloc(NULL, sizeof(*rules[i]),
					RTE_CACHE_LINE_SIZE);

			if (rules[i] == NULL) {
				uint32_t j;

				for (j = 0; j <= i; j++)
					if (new_rules[j])
						rte_free(rules[j]);

				rte_free(rules);
				rte_free(new_rules);
				return -1;
			}
		}
	}

	keys_found = rte_malloc(NULL,
			n_keys * sizeof(int),
			RTE_CACHE_LINE_SIZE);
	if (keys_found == NULL) {
		uint32_t j;

		for (j = 0; j < n_keys; j++)
			if (new_rules[j])
				rte_free(rules[j]);

		rte_free(rules);
		rte_free(new_rules);
		return -1;
	}

	entries_ptr = rte_malloc(NULL,
			n_keys * sizeof(struct rte_pipeline_table_entry *),
			RTE_CACHE_LINE_SIZE);
	if (entries_ptr == NULL) {
		uint32_t j;

		for (j = 0; j < n_keys; j++)
			if (new_rules[j])
				rte_free(rules[j]);

		rte_free(rules);
		rte_free(new_rules);
		rte_free(keys_found);
		return -1;
	}
	for (i = 0; i < n_keys; i++) {
		entries_ptr[i] = rte_malloc(NULL,
				sizeof(struct rte_pipeline_table_entry),
				RTE_CACHE_LINE_SIZE);

		if (entries_ptr[i] == NULL) {
			uint32_t j;

			for (j = 0; j < n_keys; j++)
				if (new_rules[j])
					rte_free(rules[j]);

			for (j = 0; j <= i; j++)
				rte_free(entries_ptr[j]);

			rte_free(rules);
			rte_free(new_rules);
			rte_free(keys_found);
			rte_free(entries_ptr);
			return -1;
		}
	}

	/* Allocate and write request */
	req = app_msg_alloc(app);
	if (req == NULL) {
		uint32_t j;

		for (j = 0; j < n_keys; j++)
			if (new_rules[j])
				rte_free(rules[j]);

		for (j = 0; j < n_keys; j++)
			rte_free(entries_ptr[j]);

		rte_free(rules);
		rte_free(new_rules);
		rte_free(keys_found);
		rte_free(entries_ptr);
		return -1;
	}

	req->type = PIPELINE_MSG_REQ_CUSTOM;
	req->subtype = PIPELINE_FIREWALL_MSG_REQ_ADD_BULK;

	req->keys = keys;
	req->n_keys = n_keys;
	req->port_ids = port_ids;
	req->priorities = priorities;
	req->keys_found = keys_found;
	req->entries_ptr = entries_ptr;

	/* Send request and wait for response */
	rsp = app_msg_send_recv(app, pipeline_id, req, MSG_TIMEOUT_DEFAULT);
	if (rsp == NULL) {
		uint32_t j;

		for (j = 0; j < n_keys; j++)
			if (new_rules[j])
				rte_free(rules[j]);

		for (j = 0; j < n_keys; j++)
			rte_free(entries_ptr[j]);

		rte_free(rules);
		rte_free(new_rules);
		rte_free(keys_found);
		rte_free(entries_ptr);
		return -1;
	}

	if (rsp->status) {
		for (i = 0; i < n_keys; i++)
			if (new_rules[i])
				rte_free(rules[i]);

		for (i = 0; i < n_keys; i++)
			rte_free(entries_ptr[i]);

		status = -1;
		goto cleanup;
	}

	for (i = 0; i < n_keys; i++) {
		if (entries_ptr[i] == NULL ||
			((new_rules[i] == 0) && (keys_found[i] == 0)) ||
			((new_rules[i] == 1) && (keys_found[i] == 1))) {
			for (i = 0; i < n_keys; i++)
				if (new_rules[i])
					rte_free(rules[i]);

			for (i = 0; i < n_keys; i++)
				rte_free(entries_ptr[i]);

			status = -1;
			goto cleanup;
		}
	}

	for (i = 0; i < n_keys; i++) {
		memcpy(&rules[i]->key, &keys[i], sizeof(keys[i]));
		rules[i]->priority = priorities[i];
		rules[i]->port_id = port_ids[i];
		rules[i]->entry_ptr = entries_ptr[i];

		/* Commit rule */
		if (new_rules[i]) {
			TAILQ_INSERT_TAIL(&p->rules, rules[i], node);
			p->n_rules++;
		}

		print_firewall_ipv4_rule(rules[i]);
	}

cleanup:
	app_msg_free(app, rsp);
	rte_free(rules);
	rte_free(new_rules);
	rte_free(keys_found);
	rte_free(entries_ptr);

	return status;
}

int
app_pipeline_firewall_delete_bulk(struct app_params *app,
	uint32_t pipeline_id,
	struct pipeline_firewall_key *keys,
	uint32_t n_keys)
{
	struct app_pipeline_firewall *p;
	struct pipeline_firewall_del_bulk_msg_req *req;
	struct pipeline_firewall_del_bulk_msg_rsp *rsp;

	struct app_pipeline_firewall_rule **rules;
	int *keys_found;

	uint32_t i;
	int status = 0;

	/* Check input arguments */
	if (app == NULL)
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id, &pipeline_firewall);
	if (p == NULL)
		return -1;

	rules = rte_malloc(NULL,
			n_keys * sizeof(struct app_pipeline_firewall_rule *),
			RTE_CACHE_LINE_SIZE);
	if (rules == NULL)
		return -1;

	for (i = 0; i < n_keys; i++) {
		if (app_pipeline_firewall_key_check_and_normalize(&keys[i]) != 0) {
			return -1;
		}

		rules[i] = app_pipeline_firewall_rule_find(p, &keys[i]);
	}

	keys_found = rte_malloc(NULL,
			n_keys * sizeof(int),
			RTE_CACHE_LINE_SIZE);
	if (keys_found == NULL) {
		rte_free(rules);
		return -1;
	}

	/* Allocate and write request */
	req = app_msg_alloc(app);
	if (req == NULL) {
		rte_free(rules);
		rte_free(keys_found);
		return -1;
	}

	req->type = PIPELINE_MSG_REQ_CUSTOM;
	req->subtype = PIPELINE_FIREWALL_MSG_REQ_DEL_BULK;

	req->keys = keys;
	req->n_keys = n_keys;
	req->keys_found = keys_found;

	/* Send request and wait for response */
	rsp = app_msg_send_recv(app, pipeline_id, req, MSG_TIMEOUT_DEFAULT);
	if (rsp == NULL) {
		rte_free(rules);
		rte_free(keys_found);
		return -1;
	}

	if (rsp->status) {
		status = -1;
		goto cleanup;
	}

	for (i = 0; i < n_keys; i++) {
		if (keys_found[i] == 0) {
			status = -1;
			goto cleanup;
		}
	}

	for (i = 0; i < n_keys; i++) {
		TAILQ_REMOVE(&p->rules, rules[i], node);
		p->n_rules--;
		rte_free(rules[i]);
	}

cleanup:
	app_msg_free(app, rsp);
	rte_free(rules);
	rte_free(keys_found);

	return status;
}

int
app_pipeline_firewall_add_default_rule(struct app_params *app,
	uint32_t pipeline_id,
	uint32_t port_id)
{
	struct app_pipeline_firewall *p;
	struct pipeline_firewall_add_default_msg_req *req;
	struct pipeline_firewall_add_default_msg_rsp *rsp;

	/* Check input arguments */
	if (app == NULL)
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id, &pipeline_firewall);
	if (p == NULL)
		return -1;

	if (port_id >= p->n_ports_out)
		return -1;

	/* Allocate and write request */
	req = app_msg_alloc(app);
	if (req == NULL)
		return -1;

	req->type = PIPELINE_MSG_REQ_CUSTOM;
	req->subtype = PIPELINE_FIREWALL_MSG_REQ_ADD_DEFAULT;
	req->port_id = port_id;

	/* Send request and wait for response */
	rsp = app_msg_send_recv(app, pipeline_id, req, MSG_TIMEOUT_DEFAULT);
	if (rsp == NULL)
		return -1;

	/* Read response and write rule */
	if (rsp->status || (rsp->entry_ptr == NULL)) {
		app_msg_free(app, rsp);
		return -1;
	}

	p->default_rule_port_id = port_id;
	p->default_rule_entry_ptr = rsp->entry_ptr;

	/* Commit rule */
	p->default_rule_present = 1;

	/* Free response */
	app_msg_free(app, rsp);

	return 0;
}

int
app_pipeline_firewall_delete_default_rule(struct app_params *app,
	uint32_t pipeline_id)
{
	struct app_pipeline_firewall *p;
	struct pipeline_firewall_del_default_msg_req *req;
	struct pipeline_firewall_del_default_msg_rsp *rsp;

	/* Check input arguments */
	if (app == NULL)
		return -1;

	p = app_pipeline_data_fe(app, pipeline_id, &pipeline_firewall);
	if (p == NULL)
		return -1;

	/* Allocate and write request */
	req = app_msg_alloc(app);
	if (req == NULL)
		return -1;

	req->type = PIPELINE_MSG_REQ_CUSTOM;
	req->subtype = PIPELINE_FIREWALL_MSG_REQ_DEL_DEFAULT;

	/* Send request and wait for response */
	rsp = app_msg_send_recv(app, pipeline_id, req, MSG_TIMEOUT_DEFAULT);
	if (rsp == NULL)
		return -1;

	/* Read response and write rule */
	if (rsp->status) {
		app_msg_free(app, rsp);
		return -1;
	}

	/* Commit rule */
	p->default_rule_present = 0;

	/* Free response */
	app_msg_free(app, rsp);

	return 0;
}

struct cmd_firewall_parsed_result {
	cmdline_fixed_string_t p_string;
	uint32_t pipeline_id;
	cmdline_fixed_string_t firewall_string;
	cmdline_multi_string_t multi_string;
};

static void cmd_firewall_parsed(void *parsed_result,
		__attribute__((unused))  struct cmdline *cl, void *data) {
	struct cmd_firewall_parsed_result *params = parsed_result;
	struct app_params *app = data;
	int status;
	struct pipeline_firewall_key key;

	uint32_t priority;
	struct in_addr sipaddr;
	uint32_t sipdepth;
	struct in_addr dipaddr;
	uint32_t dipdepth;
	uint16_t sport0;
	uint16_t sport1;
	uint16_t dport0;
	uint16_t dport1;
	uint8_t proto;
	uint8_t protomask;
	uint32_t port_id;
	char *filepath;

	char *tokens[17];
	uint32_t n_tokens = RTE_DIM(tokens);
	status = parse_tokenize_string(params->multi_string, tokens, &n_tokens);

	if (status != 0) {
		printf("Too many tokens\n");
		return;
	}

	if ((n_tokens > 0) && (0 != strcmp(tokens[0], "add")) && (0 != strcmp(tokens[0], "del"))
			&& (0 != strcmp(tokens[0], "ls"))) {
		printf("Usage: \n p <id> firewall add ... \n p <id> firewall del... \n p <id> firewall ls\n");
		return;
	}

	/* Parsing add command*/
	if ((n_tokens > 0) && (0 == strcmp(tokens[0], "add"))) {
		if (n_tokens > 18) {
			printf("Command failed firewall add too many arguments\n");
			return;
		}
		if (n_tokens < 2) {
			printf(
					"Usage: \n p <id> firewall add priority ... \n p <id> "
					"firewall add bulk ... \n p <id> firewall default ..\n");
			return;
		}

		/* Parsing add priority */
		if (0 == strcmp(tokens[1], "priority")) {

			if (n_tokens > 16) {
				printf(
						"Usage\n too many arguments\n p <pipeline ID> firewall "
						"add priority <priority> ipv4 <sipaddr> <sipdepth> "
						"<dipaddr> <dipdepth> <sport0> <sport1> <dport0> "
						"<dport1> <proto> <protomask> port <port ID>\n");
				return;
			}

			if (n_tokens < 16) {
				printf(
						"Usage\nnot enough arguments\n p <pipeline ID> firewall"
						" add priority <priority> ipv4 <sipaddr> <sipdepth>"
						" <dipaddr> <dipdepth> <sport0> <sport1> <dport0> "
						"<dport1> <proto> <protomask> port <port ID>\n");
				return;
			}

			if (parser_read_uint32(&priority, tokens[2])) {
				printf("Incorrect format for priority\n");
				return;
			}

			if (strcmp(tokens[3], "ipv4")) {
				printf("string  supported ipv4\n");
				return;
			}

			if (parse_ipv4_addr(tokens[4], &sipaddr)) {
				printf("Incorrect format for sipaddr\n");
				return;
			}

			if (parser_read_uint32(&sipdepth, tokens[5])) {
				printf("Incorrect format for sipdepth\n");
				return;
			}

			if (parse_ipv4_addr(tokens[6], &dipaddr)) {
				printf("Incorrect format for dipaddr\n");
				return;
			}

			if (parser_read_uint32(&dipdepth, tokens[7])) {
				printf("Incorrect format for dipdepth\n");
				return;
			}

			if (parser_read_uint16(&sport0, tokens[8])) {
				printf("Incorrect format for sport0\n");
				return;
			}

			if (parser_read_uint16(&sport1, tokens[9])) {
				printf("Incorrect format for sport1\n");
				return;
			}

			if (parser_read_uint16(&dport0, tokens[10])) {
				printf("Incorrect format for dport0\n");
				return;
			}

			if (parser_read_uint16(&dport1, tokens[11])) {
				printf("Incorrect format for dport1\n");
				return;
			}

			if (parser_read_uint8(&proto, tokens[12])) {
				printf("Incorrect format for proto\n");
				return;
			}

			char *e;
			protomask = strtoul(tokens[13], &e, 16);
			if (*e || errno == EINVAL || errno == ERANGE) {
				printf("Incorrect format for protomask - should be in hex \n");
				return;
			}

			if (strcmp(tokens[14], "port")) {
				printf("string  supported port\n");
				return;
			}

			if (parser_read_uint32(&port_id, tokens[15])) {
				printf("Incorrect format for port ID\n");
				return;
			}

			key.type = PIPELINE_FIREWALL_IPV4_5TUPLE;
			key.key.ipv4_5tuple.src_ip = rte_bswap32((uint32_t) sipaddr.s_addr);
			key.key.ipv4_5tuple.src_ip_mask = sipdepth;
			key.key.ipv4_5tuple.dst_ip = rte_bswap32((uint32_t) dipaddr.s_addr);
			key.key.ipv4_5tuple.dst_ip_mask = dipdepth;
			key.key.ipv4_5tuple.src_port_from = sport0;
			key.key.ipv4_5tuple.src_port_to = sport1;
			key.key.ipv4_5tuple.dst_port_from = dport0;
			key.key.ipv4_5tuple.dst_port_to = dport1;
			key.key.ipv4_5tuple.proto = proto;
			key.key.ipv4_5tuple.proto_mask = protomask;

			status = app_pipeline_firewall_add_rule(app, params->pipeline_id,
					&key, priority, port_id);

			if (status != 0)
				printf("Command failed\n");

			return;

		}

		/* Parsing add default */
		if ((0 == strcmp(tokens[1], "default"))
				&& (0 != strcmp(tokens[1], "bulk"))
				&& (0 != strcmp(tokens[1], "priority"))) {
			if (n_tokens > 3) {
				printf(
						"too many arguments: \np <pipeline ID> firewall add "
						"default <port ID>");
				return;
			}

			if (n_tokens < 3) {
				printf(
						"not enough arguments \np <pipeline ID> firewall add "
						"default <port ID>");
				return;
			}

			if (parser_read_uint32(&port_id, tokens[2])) {
				printf("Incorrect format for port ID\n");
				return;
			}

			status = app_pipeline_firewall_add_default_rule(app,
					params->pipeline_id, port_id);

			if (status != 0)
				printf("Command failed\n");

			return;
		}

		/* Parsing add bulk */
		if (0 == strcmp(tokens[1], "bulk")) {
			if (n_tokens > 3) {
				printf(
						"too many arguments: \np <pipeline ID> firewall add "
						"bulk <file>");
				return;
			}

			if (n_tokens < 3) {
				printf(
						"not enough arguments \np <pipeline ID> firewall add "
						"bulk <file>");
				return;
			}

			filepath = tokens[2];
			printf("%s", filepath);

			struct app_pipeline_add_bulk_params add_bulk_params;
			memset(&add_bulk_params, 0, sizeof(struct app_pipeline_add_bulk_params));

			status = app_pipeline_add_bulk_parse_file(filepath,
					&add_bulk_params);
			if (status != 0) {
				printf("Command failed\n");
				goto end;
			}

			status = app_pipeline_firewall_add_bulk(app, params->pipeline_id,
					add_bulk_params.keys, add_bulk_params.n_keys,
					add_bulk_params.priorities, add_bulk_params.port_ids);
			if (status != 0) {
				printf("Command failed\n");
				goto end;
			}

			return;

			end:
			rte_free(add_bulk_params.keys);
			rte_free(add_bulk_params.priorities);
			rte_free(add_bulk_params.port_ids);
			return;

		}
	}

	/* Parsing del command */
	if ((n_tokens > 0) && (0 == strcmp(tokens[0], "del"))) {
		if (n_tokens > 15) {
			printf("Command failed firewall add too many arguments\n");
			return;
		}
		if (n_tokens < 2) {
			printf(
					"Usage: \n p <id> firewall del ipv4 ... \n p <id> firewall "
					"del bulk ... \n p <id> firewall del default ...\n");
			return;
		}

		if (strcmp(tokens[1], "ipv4") == 0) {
			if (n_tokens > 12) {
				printf(
						"too many arguments: \np <pipeline ID> firewall del"
						" ipv4 .. ");
				return;
			}

			if (n_tokens < 12) {
				printf(
						"not enough arguments \np <pipeline ID> firewall del"
						" ipv4 .. ");
				return;
			}

			if (parse_ipv4_addr(tokens[2], &sipaddr)) {
				printf("Incorrect format for sipaddr\n");
				return;
			}

			if (parser_read_uint32(&sipdepth, tokens[3])) {
				printf("Incorrect format for sipdepth\n");
				return;
			}

			if (parse_ipv4_addr(tokens[4], &dipaddr)) {
				printf("Incorrect format for dipaddr\n");
				return;
			}

			if (parser_read_uint32(&dipdepth, tokens[5])) {
				printf("Incorrect format for dipdepth\n");
				return;
			}

			if (parser_read_uint16(&sport0, tokens[6])) {
				printf("Incorrect format for sport0\n");
				return;
			}

			if (parser_read_uint16(&sport1, tokens[7])) {
				printf("Incorrect format for sport1\n");
				return;
			}
			if (parser_read_uint16(&dport0, tokens[8])) {
				printf("Incorrect format for dport0\n");
				return;
			}

			if (parser_read_uint16(&dport1, tokens[9])) {
				printf("Incorrect format for dport1\n");
				return;
			}

			if (parser_read_uint8(&proto, tokens[10])) {
				printf("Incorrect format for proto\n");
				return;
			}

			char *e;
			protomask = strtoul(tokens[11], &e, 16);
			if (*e || errno == EINVAL || errno == ERANGE) {
				printf("Incorrect format for protomask - should be in hex \n");
				return;
			}

			key.type = PIPELINE_FIREWALL_IPV4_5TUPLE;
			key.key.ipv4_5tuple.src_ip = rte_bswap32((uint32_t) sipaddr.s_addr);
			key.key.ipv4_5tuple.src_ip_mask = sipdepth;
			key.key.ipv4_5tuple.dst_ip = rte_bswap32((uint32_t) dipaddr.s_addr);
			key.key.ipv4_5tuple.dst_ip_mask = dipdepth;
			key.key.ipv4_5tuple.src_port_from = sport0;
			key.key.ipv4_5tuple.src_port_to = sport1;
			key.key.ipv4_5tuple.dst_port_from = dport0;
			key.key.ipv4_5tuple.dst_port_to = dport1;
			key.key.ipv4_5tuple.proto = proto;
			key.key.ipv4_5tuple.proto_mask = protomask;

			status = app_pipeline_firewall_delete_rule(app, params->pipeline_id,
					&key);

			if (status != 0)
				printf("Command failed\n");

			return;
		}

		/* del bulk */
		if (strcmp(tokens[1], "bulk") == 0) {
			if (n_tokens > 3) {
				printf(
						"too many arguments: \np <pipeline ID> firewall del "
						"bulk <file>");
				return;
			}

			if (n_tokens < 3) {
				printf(
						"not enough arguments \np <pipeline ID> firewall del"
						" bulk <file>");
				return;
			}

			filepath = tokens[2];
			struct app_pipeline_del_bulk_params del_bulk_params;
			memset(&del_bulk_params, 0, sizeof(struct app_pipeline_del_bulk_params));

			status = app_pipeline_del_bulk_parse_file(filepath,
					&del_bulk_params);
			if (status != 0) {
				printf("Command failed\n");
				goto end1;
			}

			status = app_pipeline_firewall_delete_bulk(app, params->pipeline_id,
					del_bulk_params.keys, del_bulk_params.n_keys);
			if (status != 0) {
				printf("Command failed\n");
				goto end1;
			}
			return;

			end1: rte_free(del_bulk_params.keys);
			return;

		}

		if (strcmp(tokens[1], "default") == 0) {
			if (n_tokens > 2) {
				printf(
						"too many arguments: \np <pipeline ID> firewall del "
						"default");
				return;
			}

			if (n_tokens < 2) {
				printf(
						"not enough arguments \np <pipeline ID> firewall del "
						"default");
				return;
			}

			status = app_pipeline_firewall_delete_default_rule(app,
					params->pipeline_id);

			if (status != 0)
				printf("Command failed\n");

			return;

		}
	}

	/* Parsing ls command */
	if ((n_tokens > 0) && (0 == strcmp(tokens[0], "ls"))) {

		if (n_tokens > 1) {
			printf("Command firewall ls too many arguments\n");
			return;
		}

		status = app_pipeline_firewall_ls(app, params->pipeline_id);

		if (status != 0)
			printf("Command firewall ls failed\n");

		return;

	}

}

cmdline_parse_token_string_t cmd_firewall_p_string =
	TOKEN_STRING_INITIALIZER(struct cmd_firewall_parsed_result, p_string,
		"p");

cmdline_parse_token_num_t cmd_firewall_pipeline_id =
	TOKEN_NUM_INITIALIZER(struct cmd_firewall_parsed_result, pipeline_id,
		UINT32);

cmdline_parse_token_string_t cmd_firewall_firewall_string =
	TOKEN_STRING_INITIALIZER(struct cmd_firewall_parsed_result,
	firewall_string, "firewall");

static cmdline_parse_token_string_t cmd_firewall_multi_string =
	TOKEN_STRING_INITIALIZER(struct cmd_firewall_parsed_result, multi_string,
	TOKEN_STRING_MULTI);

cmdline_parse_inst_t cmd_firewall = {
	.f = cmd_firewall_parsed,
	.data = NULL,
	.help_str = "\n p <pipeline ID> firewall add priority <priority> ipv4 "
				"<sipaddr> <sipdepth> <dipaddr> <dipdepth> <sport0> <sport1> "
				"<dport0> <dport1> <proto> <protomask> port <port ID> \n"
				"\n p <pipeline ID> firewall add bulk <file>  \n"
				"\n p <pipeline ID> firewall add default <port ID>\n"
				"\n p <pipeline ID> firewall del ipv4 <sipaddr> <sipdepth>"
				"<dipaddr> <dipdepth> <sport0> <sport1> <dport0> <dport1> "
				"<proto> <protomask> \n"
				"\n p <pipeline ID> firewall del bulk <file> \n"
				"\n p <pipeline ID> firewall del default \n"
				"\n p <pipeline ID> firewall ls \n",
	.tokens = {
		(void *) &cmd_firewall_p_string,
		(void *) &cmd_firewall_pipeline_id,
		(void *) &cmd_firewall_firewall_string,
		(void *)&cmd_firewall_multi_string,
		NULL,
	},
};

static cmdline_parse_ctx_t pipeline_cmds[] = {
	(cmdline_parse_inst_t *) &cmd_firewall,
	NULL,
};

static struct pipeline_fe_ops pipeline_firewall_fe_ops = {
	.f_init = app_pipeline_firewall_init,
	.f_free = app_pipeline_firewall_free,
	.cmds = pipeline_cmds,
};

struct pipeline_type pipeline_firewall = {
	.name = "FIREWALL",
	.be_ops = &pipeline_firewall_be_ops,
	.fe_ops = &pipeline_firewall_fe_ops,
};
