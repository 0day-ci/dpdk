/*   BSD LICENSE
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

#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/ip.h>

#ifndef __PARSER_H
#define __PARSER_H

struct parse_status {
	int status;
	char parse_msg[256];
};

#define	APP_CHECK(exp, status, fmt, ...)				\
do {									\
	if (!(exp)) {							\
		sprintf(status->parse_msg, fmt "\n",			\
			## __VA_ARGS__);				\
		status->status = -1;					\
	} else								\
		status->status = 0;					\
} while (0)

#define APP_CHECK_PRESENCE(val, str, status)				\
	APP_CHECK(val == 0, status,					\
		"item \"%s\" already present", str)

#define APP_CHECK_TOKEN_EQUAL(tokens, index, ref, status)		\
	APP_CHECK(strcmp(tokens[index], ref) == 0, status,		\
		"unrecognized input \"%s\": expect \"%s\"\n",		\
		tokens[index], ref)

static inline int
is_str_num(const char *str)
{
	uint32_t i;

	for (i = 0; i < strlen(str); i++)
		if (!isdigit(str[i]))
			return -1;

	return 0;
}

#define APP_CHECK_TOKEN_IS_NUM(tokens, index, status)			\
	APP_CHECK(is_str_num(tokens[index]) == 0, status,		\
	"input \"%s\" is not valid number string", tokens[index])


#define INCREMENT_TOKEN_INDEX(index, max_num, status)			\
do {									\
	APP_CHECK(index + 1 < max_num, status, "reaching the end of "	\
		"the token array");					\
	index++;							\
} while (0)

#if RTE_BYTE_ORDER != RTE_LITTLE_ENDIAN
#define __BYTES_TO_UINT64(a, b, c, d, e, f, g, h) \
	(((uint64_t)((a) & 0xff) << 56) | \
	((uint64_t)((b) & 0xff) << 48) | \
	((uint64_t)((c) & 0xff) << 40) | \
	((uint64_t)((d) & 0xff) << 32) | \
	((uint64_t)((e) & 0xff) << 24) | \
	((uint64_t)((f) & 0xff) << 16) | \
	((uint64_t)((g) & 0xff) << 8)  | \
	((uint64_t)(h) & 0xff))
#else
#define __BYTES_TO_UINT64(a, b, c, d, e, f, g, h) \
	(((uint64_t)((h) & 0xff) << 56) | \
	((uint64_t)((g) & 0xff) << 48) | \
	((uint64_t)((f) & 0xff) << 40) | \
	((uint64_t)((e) & 0xff) << 32) | \
	((uint64_t)((d) & 0xff) << 24) | \
	((uint64_t)((c) & 0xff) << 16) | \
	((uint64_t)((b) & 0xff) << 8) | \
	((uint64_t)(a) & 0xff))
#endif

#define ETHADDR_TO_UINT64(addr) __BYTES_TO_UINT64( \
		addr.addr_bytes[0], addr.addr_bytes[1], \
		addr.addr_bytes[2], addr.addr_bytes[3], \
		addr.addr_bytes[4], addr.addr_bytes[5], \
		0, 0)

int
parse_ipv4_addr(const char *token, struct in_addr *ipv4, uint32_t *mask);

int
parse_ipv6_addr(const char *token, struct in6_addr *ipv6, uint32_t *mask);

int
parse_eth_addr(const char *token, struct ether_addr *addr);

int
parse_range(const char *token, uint16_t *low, uint16_t *high);

void
parse_sp4_tokens(char **tokens, uint32_t n_tokens,
	struct parse_status *status);

void
parse_sp6_tokens(char **tokens, uint32_t n_tokens,
	struct parse_status *status);

void
parse_sa_tokens(char **tokens, uint32_t n_tokens,
	struct parse_status *status);

void
parse_rt_tokens(char **tokens, uint32_t n_tokens,
	struct parse_status *status);

void
parse_eth_tokens(char **tokens, uint32_t n_tokens,
	struct parse_status *status);

int
parse_cfg_file(const char *cfg_filename);

#endif
