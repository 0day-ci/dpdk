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

/*
 * Ethernet Address
 */
#include <stdint.h>
#include <sys/types.h>
#include <rte_ether.h>
#include <rte_errno.h>
#include <rte_ethdev.h>

#include "ipsec.h"
#include "parser.h"

#define ETH_DST_MAX_RULES	1024

struct addr {
	uint8_t port;
	struct ether_addr src;
	struct ether_addr dst;
};

struct addr eth_addr[ETH_DST_MAX_RULES];
uint32_t nb_eth_addr;

void
parse_eth_tokens(char **tokens, uint32_t n_tokens,
	struct parse_status *status)
{
	uint32_t ti;
	uint32_t *n_addr = NULL;
	struct addr *addr = NULL;

	if (strcmp(tokens[0], "addr") == 0) {
		n_addr = &nb_eth_addr;
		addr = &eth_addr[*n_addr];

		APP_CHECK(*n_addr <= ETH_DST_MAX_RULES - 1, status,
			"too many eth dst rules, abort insertion\n");
		if (status->status < 0)
			return;
	} else {
		APP_CHECK(0, status, "unrecognized input \"%s\"",
			tokens[0]);
		return;
	}

	for (ti = 1; ti < n_tokens; ti++) {
		if (strcmp(tokens[ti], "src") == 0) {
			INCREMENT_TOKEN_INDEX(ti, n_tokens, status);
			if (status->status < 0)
				return;

			if (addr != NULL) {
				APP_CHECK(parse_eth_addr(tokens[ti],
					  &addr->src) == 0, status,
					  "unrecognized input \"%s\", "
					  "expect valid src addr",
					  tokens[ti]);
				if (status->status < 0)
					return;
			} else {
				APP_CHECK(0, status, "addr is NULL");
			}
		}

		if (strcmp(tokens[ti], "dst") == 0) {
			INCREMENT_TOKEN_INDEX(ti, n_tokens, status);
			if (status->status < 0)
				return;

			if (addr != NULL) {
				APP_CHECK(parse_eth_addr(tokens[ti],
					  &addr->dst) == 0, status,
					  "unrecognized input \"%s\", "
					  "expect valid dst addr",
					  tokens[ti]);
				if (status->status < 0)
					return;
			} else {
				APP_CHECK(0, status, "addr is NULL");
			}
		}

		if (strcmp(tokens[ti], "port") == 0) {
			INCREMENT_TOKEN_INDEX(ti, n_tokens, status);
			if (status->status < 0)
				return;

			APP_CHECK_TOKEN_IS_NUM(tokens, ti, status);
			if (status->status < 0)
				return;

			if (addr != NULL)
				addr->port = atoi(tokens[ti]);
			else
				APP_CHECK(0, status, "addr is NULL");
		}
	}

	*n_addr = *n_addr + 1;
}

static struct eth_ctx *
eth_create(const char *name, int32_t socket_id)
{
	char s[PATH_MAX];
	struct eth_ctx *eth_ctx;
	uint32_t mz_size;
	const struct rte_memzone *mz;

	snprintf(s, sizeof(s), "%s_%u", name, socket_id);

	/* Create SA array table */
	printf("Creating ETH context with %u maximum entries\n",
			RTE_MAX_ETHPORTS);

	mz_size = sizeof(struct eth_ctx) * RTE_MAX_ETHPORTS;
	mz = rte_memzone_reserve(s, mz_size, socket_id,
			RTE_MEMZONE_1GB | RTE_MEMZONE_SIZE_HINT_ONLY);
	if (mz == NULL) {
		printf("Failed to allocate SA DB memory\n");
		rte_errno = -ENOMEM;
		return NULL;
	}
	memset(mz->addr, 0, mz_size);

	eth_ctx = (struct eth_ctx *)mz->addr;
	return eth_ctx;
}

static void
eth_ctx_dump(struct eth_ctx *eth_addr, uint32_t mask)
{
	char name[256];
	uint32_t nb_ports;
	uint8_t port;

	nb_ports = rte_eth_dev_count();
	for (port = 0; port < nb_ports; ++port) {
		if ((mask & (1 << port)) == 0)
			continue;

		if (rte_eth_dev_get_name_by_port(port, name) < 0)
			rte_exit(EXIT_FAILURE, "Unable to find name "
					"to port=%d\n", port);

		printf("%s-dst-0x%lx\n", name, eth_addr[port].dst);
		printf("%s-src-0x%lx\n", name, eth_addr[port].src);
	}
}

void
eth_init(struct socket_ctx *ctx, int32_t socket_id, uint32_t mask)
{
	const char *name;
	uint32_t i, nb_ports;
	uint8_t port;
	struct ether_addr ethaddr;

	if (ctx == NULL)
		rte_exit(EXIT_FAILURE, "NULL context.\n");

	if (ctx->eth_addr != NULL)
		rte_exit(EXIT_FAILURE, "ETH Address Table for socket %u "
			"already initialized\n", socket_id);

	if (nb_eth_addr == 0)
		RTE_LOG(WARNING, IPSEC, "No ETH address rule specified\n");

	/* create the ETH table */
	name = "eth_addr";
	ctx->eth_addr = eth_create(name, socket_id);
	if (ctx->eth_addr == NULL)
		rte_exit(EXIT_FAILURE, "Error [%d] creating ETH "
				"context %s in socket %d\n", rte_errno,
				name, socket_id);

	/* populate the ETH table */
	for (i = 0; i < nb_eth_addr; ++i) {
		port = eth_addr[i].port;

		if (ctx->eth_addr[port].dst != 0)
			rte_exit(EXIT_FAILURE, "ETH destination address "
					"for port %u already in use\n",
					port);
		if (ctx->eth_addr[port].src != 0)
			rte_exit(EXIT_FAILURE, "ETH source address "
					"for port %u already in use\n",
					port);

		ctx->eth_addr[port].dst = ETHADDR_TO_UINT64(eth_addr[i].dst);
		ctx->eth_addr[port].src = ETHADDR_TO_UINT64(eth_addr[i].src);
	}

	nb_ports = rte_eth_dev_count();
	for (port = 0; port < nb_ports; ++port) {
		if ((mask & (1 << port)) == 0)
			continue;

		if (ctx->eth_addr[port].src == 0) {
			rte_eth_macaddr_get(port, &ethaddr);
			ctx->eth_addr[port].src = ETHADDR_TO_UINT64(ethaddr);
		}
	}
	eth_ctx_dump(ctx->eth_addr, mask);
}
