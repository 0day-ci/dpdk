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
 * Security Associations
 */
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>

#include <rte_memzone.h>
#include <rte_crypto.h>
#include <rte_cryptodev.h>
#include <rte_byteorder.h>
#include <rte_errno.h>
#include <rte_ip.h>

#include "ipsec.h"
#include "esp.h"
#include "parser.h"

struct supported_cipher_algo {
	const char *keyword;
	enum rte_crypto_cipher_algorithm algo;
	uint16_t iv_len;
	uint16_t block_size;
};

struct supported_auth_algo {
	const char *keyword;
	enum rte_crypto_auth_algorithm algo;
	uint16_t digest_len;
};

const struct supported_cipher_algo cipher_algos[] = {
	{
		.keyword = "null",
		.algo = RTE_CRYPTO_CIPHER_NULL,
		.iv_len = 0,
		.block_size = 4
	},
	{
		.keyword = "aes-128-cbc",
		.algo = RTE_CRYPTO_CIPHER_AES_CBC,
		.iv_len = 16,
		.block_size = 16
	}
};

const struct supported_auth_algo auth_algos[] = {
	{
		.keyword = "null",
		.algo = RTE_CRYPTO_AUTH_NULL,
		.digest_len = 0,
	},
	{
		.keyword = "sha1-hmac",
		.algo = RTE_CRYPTO_AUTH_SHA1_HMAC,
		.digest_len = 12
	}
};

static int
find_match_algo(struct ipsec_sa *rule,
	const char *cipher_keyword, const char *auth_keyword)
{
	size_t i;
	int ret = -2;

	if (cipher_keyword != NULL) {
		for (i = 0; i < RTE_DIM(cipher_algos); i++) {
			const struct supported_cipher_algo *algo =
				&cipher_algos[i];

			if (strcmp(cipher_keyword, algo->keyword) == 0) {
				rule->cipher_algo = algo->algo;
				rule->block_size = algo->block_size;
				rule->iv_len = algo->iv_len;
				ret += 1;
				break;
			}
		}
	}

	if (auth_keyword != NULL) {
		for (i = 0; i < RTE_DIM(auth_algos); i++) {
			const struct supported_auth_algo *algo =
				&auth_algos[i];

			if (strcmp(auth_keyword, algo->keyword) == 0) {
				rule->auth_algo = algo->algo;
				rule->digest_len = algo->digest_len;
				ret += 1;
				break;
			}
		}
	}

	return ret;
}

#define MAX_SA_RULE_NUM		1000

struct ipsec_sa sa_out[MAX_SA_RULE_NUM];
uint32_t nb_sa_out;

struct ipsec_sa sa_in[MAX_SA_RULE_NUM];
uint32_t nb_sa_in;

void
parse_sa_tokens(char **tokens, uint32_t n_tokens,
	struct parse_status *status)
{
	struct ipsec_sa *rule = NULL;
	uint32_t ti; /*token index*/
	uint32_t *ri /*rule index*/;
	const char *cipher_str, *auth_str;
	uint32_t src_p = 0;
	uint32_t dst_p = 0;

	if (strcmp(tokens[0], "in") == 0) {
		ri = &nb_sa_in;

		APP_CHECK(*ri <= MAX_SA_RULE_NUM - 1, status,
			"too many sa rules, abort insertion\n");
		if (status->status < 0)
			return;

		rule = &sa_in[*ri];
	} else {
		ri = &nb_sa_out;

		APP_CHECK(*ri <= MAX_SA_RULE_NUM - 1, status,
			"too many sa rules, abort insertion\n");
		if (status->status < 0)
			return;

		rule = &sa_out[*ri];
	}

	/* spi number */
	APP_CHECK_TOKEN_IS_NUM(tokens, 1, status);
	if (status->status < 0)
		return;
	rule->spi = atoi(tokens[1]);

	cipher_str = tokens[2];
	auth_str = tokens[3];

	APP_CHECK(find_match_algo(rule, cipher_str, auth_str) == 0,
		status, "Unrecognized cipher or auth algorithm (%s:%s)",
		cipher_str, auth_str);
	if (status->status < 0)
		return;

	if (strcmp(tokens[4], "ipv4-tunnel") == 0)
		rule->flags = IP4_TUNNEL;
	else if (strcmp(tokens[4], "ipv6-tunnel") == 0)
		rule->flags = IP6_TUNNEL;
	else if (strcmp(tokens[4], "transport") == 0)
		rule->flags = TRANSPORT;
	else {
		APP_CHECK(0, status, "unrecognized input \"%s\"",
			tokens[4]);
		return;
	}

	for (ti = 5; ti < n_tokens; ti++) {
		if (strcmp(tokens[ti], "src") == 0) {
			APP_CHECK_PRESENCE(src_p, tokens[ti], status);
			if (status->status < 0)
				return;

			INCREMENT_TOKEN_INDEX(ti, n_tokens, status);
			if (status->status < 0)
				return;

			if (rule->flags == IP4_TUNNEL) {
				struct in_addr ip;

				APP_CHECK(parse_ipv4_addr(tokens[ti],
					&ip, NULL) == 0, status,
					"unrecognized input \"%s\", "
					"expect valid ipv4 addr",
					tokens[ti]);
				if (status->status < 0)
					return;
				rule->src.ip4 = rte_bswap32(
					(uint32_t)ip.s_addr);
			} else if (rule->flags == IP6_TUNNEL) {
				struct in6_addr ip;

				APP_CHECK(parse_ipv6_addr(tokens[ti], &ip,
					NULL) == 0, status,
					"unrecognized input \"%s\", "
					"expect valid ipv6 addr",
					tokens[ti]);
				if (status->status < 0)
					return;
				memcpy(rule->src.ip6_b, ip.s6_addr, 16);
			} else if (rule->flags == TRANSPORT) {
				APP_CHECK(0, status, "unrecognized input "
					"\"%s\"", tokens[ti]);
				return;
			}

			src_p = 1;
			continue;
		}

		if (strcmp(tokens[ti], "dst") == 0) {
			APP_CHECK_PRESENCE(dst_p, tokens[ti], status);
			if (status->status < 0)
				return;

			INCREMENT_TOKEN_INDEX(ti, n_tokens, status);
			if (status->status < 0)
				return;

			if (rule->flags == IP4_TUNNEL) {
				struct in_addr ip;

				APP_CHECK(parse_ipv4_addr(tokens[ti],
					&ip, NULL) == 0, status,
					"unrecognized input \"%s\", "
					"expect valid ipv4 addr",
					tokens[ti]);
				if (status->status < 0)
					return;
				rule->dst.ip4 = rte_bswap32(
					(uint32_t)ip.s_addr);
			} else if (rule->flags == IP6_TUNNEL) {
				struct in6_addr ip;

				APP_CHECK(parse_ipv6_addr(tokens[ti], &ip,
					NULL) == 0, status,
					"unrecognized input \"%s\", "
					"expect valid ipv6 addr",
					tokens[ti]);
				if (status->status < 0)
					return;
				memcpy(rule->dst.ip6_b, ip.s6_addr, 16);
			} else if (rule->flags == TRANSPORT) {
				APP_CHECK(0, status, "unrecognized "
					"input \"%s\"",	tokens[ti]);
				return;
			}

			dst_p = 1;
			continue;
		}

		/* unrecognizeable input */
		APP_CHECK(0, status, "unrecognized input \"%s\"",
			tokens[ti]);
		return;
	}

	*ri = *ri + 1;
}

static uint8_t cipher_key[256] = "sixteenbytes key";

/* AES CBC xform */
const struct rte_crypto_sym_xform aescbc_enc_xf = {
	NULL,
	RTE_CRYPTO_SYM_XFORM_CIPHER,
	{.cipher = { RTE_CRYPTO_CIPHER_OP_ENCRYPT, RTE_CRYPTO_CIPHER_AES_CBC,
		.key = { cipher_key, 16 } }
	}
};

const struct rte_crypto_sym_xform aescbc_dec_xf = {
	NULL,
	RTE_CRYPTO_SYM_XFORM_CIPHER,
	{.cipher = { RTE_CRYPTO_CIPHER_OP_DECRYPT, RTE_CRYPTO_CIPHER_AES_CBC,
		.key = { cipher_key, 16 } }
	}
};

static uint8_t auth_key[256] = "twentybytes hash key";

/* SHA1 HMAC xform */
const struct rte_crypto_sym_xform sha1hmac_gen_xf = {
	NULL,
	RTE_CRYPTO_SYM_XFORM_AUTH,
	{.auth = { RTE_CRYPTO_AUTH_OP_GENERATE, RTE_CRYPTO_AUTH_SHA1_HMAC,
		.key = { auth_key, 20 }, 12, 0 }
	}
};

const struct rte_crypto_sym_xform sha1hmac_verify_xf = {
	NULL,
	RTE_CRYPTO_SYM_XFORM_AUTH,
	{.auth = { RTE_CRYPTO_AUTH_OP_VERIFY, RTE_CRYPTO_AUTH_SHA1_HMAC,
		.key = { auth_key, 20 }, 12, 0 }
	}
};

/* AES CBC xform */
const struct rte_crypto_sym_xform null_cipher_xf = {
	NULL,
	RTE_CRYPTO_SYM_XFORM_CIPHER,
	{.cipher = { .algo = RTE_CRYPTO_CIPHER_NULL }
	}
};

const struct rte_crypto_sym_xform null_auth_xf = {
	NULL,
	RTE_CRYPTO_SYM_XFORM_AUTH,
	{.auth = { .algo = RTE_CRYPTO_AUTH_NULL }
	}
};

struct sa_ctx {
	struct ipsec_sa sa[IPSEC_SA_MAX_ENTRIES];
	struct {
		struct rte_crypto_sym_xform a;
		struct rte_crypto_sym_xform b;
	} xf[IPSEC_SA_MAX_ENTRIES];
};

static struct sa_ctx *
sa_create(const char *name, int32_t socket_id)
{
	char s[PATH_MAX];
	struct sa_ctx *sa_ctx;
	uint32_t mz_size;
	const struct rte_memzone *mz;

	snprintf(s, sizeof(s), "%s_%u", name, socket_id);

	/* Create SA array table */
	printf("Creating SA context with %u maximum entries\n",
			IPSEC_SA_MAX_ENTRIES);

	mz_size = sizeof(struct sa_ctx);
	mz = rte_memzone_reserve(s, mz_size, socket_id,
			RTE_MEMZONE_1GB | RTE_MEMZONE_SIZE_HINT_ONLY);
	if (mz == NULL) {
		printf("Failed to allocate SA DB memory\n");
		rte_errno = -ENOMEM;
		return NULL;
	}

	sa_ctx = (struct sa_ctx *)mz->addr;

	return sa_ctx;
}

static int
sa_add_rules(struct sa_ctx *sa_ctx, const struct ipsec_sa entries[],
		uint32_t nb_entries, uint32_t inbound)
{
	struct ipsec_sa *sa;
	uint32_t i, idx;

	for (i = 0; i < nb_entries; i++) {
		idx = SPI2IDX(entries[i].spi);
		sa = &sa_ctx->sa[idx];
		if (sa->spi != 0) {
			printf("Index %u already in use by SPI %u\n",
					idx, sa->spi);
			return -EINVAL;
		}
		*sa = entries[i];
		sa->seq = 0;

		switch (sa->flags) {
		case IP4_TUNNEL:
			sa->src.ip4 = rte_cpu_to_be_32(sa->src.ip4);
			sa->dst.ip4 = rte_cpu_to_be_32(sa->dst.ip4);
		}

		if (inbound) {
			if (sa->cipher_algo == RTE_CRYPTO_CIPHER_NULL) {
				sa_ctx->xf[idx].a = null_auth_xf;
				sa_ctx->xf[idx].b = null_cipher_xf;
			} else {
				sa_ctx->xf[idx].a = sha1hmac_verify_xf;
				sa_ctx->xf[idx].b = aescbc_dec_xf;
			}
		} else { /* outbound */
			if (sa->cipher_algo == RTE_CRYPTO_CIPHER_NULL) {
				sa_ctx->xf[idx].a = null_cipher_xf;
				sa_ctx->xf[idx].b = null_auth_xf;
			} else {
				sa_ctx->xf[idx].a = aescbc_enc_xf;
				sa_ctx->xf[idx].b = sha1hmac_gen_xf;
			}
		}
		sa_ctx->xf[idx].a.next = &sa_ctx->xf[idx].b;
		sa_ctx->xf[idx].b.next = NULL;
		sa->xforms = &sa_ctx->xf[idx].a;
	}

	return 0;
}

static inline int
sa_out_add_rules(struct sa_ctx *sa_ctx, const struct ipsec_sa entries[],
		uint32_t nb_entries)
{
	return sa_add_rules(sa_ctx, entries, nb_entries, 0);
}

static inline int
sa_in_add_rules(struct sa_ctx *sa_ctx, const struct ipsec_sa entries[],
		uint32_t nb_entries)
{
	return sa_add_rules(sa_ctx, entries, nb_entries, 1);
}

void
sa_init(struct socket_ctx *ctx, int32_t socket_id)
{
	const char *name;

	if (ctx == NULL)
		rte_exit(EXIT_FAILURE, "NULL context.\n");

	if (ctx->sa_in != NULL)
		rte_exit(EXIT_FAILURE, "Inbound SA DB for socket %u already "
				"initialized\n", socket_id);

	if (ctx->sa_out != NULL)
		rte_exit(EXIT_FAILURE, "Outbound SA DB for socket %u already "
				"initialized\n", socket_id);

	if (nb_sa_out == 0 && nb_sa_in == 0)
		RTE_LOG(WARNING, IPSEC, "No SA rule specified\n");

	name = "sa_in";
	ctx->sa_in = sa_create(name, socket_id);
	if (ctx->sa_in == NULL)
		rte_exit(EXIT_FAILURE, "Error [%d] creating SA context %s "
				"in socket %d\n", rte_errno, name, socket_id);

	name = "sa_out";
	ctx->sa_out = sa_create(name, socket_id);
	if (ctx->sa_out == NULL)
		rte_exit(EXIT_FAILURE, "Error [%d] creating SA context %s "
				"in socket %d\n", rte_errno, name, socket_id);

	sa_in_add_rules(ctx->sa_in, sa_in, nb_sa_in);

	sa_out_add_rules(ctx->sa_out, sa_out, nb_sa_out);
}

int
inbound_sa_check(struct sa_ctx *sa_ctx, struct rte_mbuf *m, uint32_t sa_idx)
{
	struct ipsec_mbuf_metadata *priv;

	priv = RTE_PTR_ADD(m, sizeof(struct rte_mbuf));

	return (sa_ctx->sa[sa_idx].spi == priv->sa->spi);
}

static inline void
single_inbound_lookup(struct ipsec_sa *sadb, struct rte_mbuf *pkt,
		struct ipsec_sa **sa_ret)
{
	struct esp_hdr *esp;
	struct ip *ip;
	uint32_t *src4_addr;
	uint8_t *src6_addr;
	struct ipsec_sa *sa;

	*sa_ret = NULL;

	ip = rte_pktmbuf_mtod(pkt, struct ip *);
	if (ip->ip_v == IPVERSION)
		esp = (struct esp_hdr *)(ip + 1);
	else
		esp = (struct esp_hdr *)(((struct ip6_hdr *)ip) + 1);

	if (esp->spi == INVALID_SPI)
		return;

	sa = &sadb[SPI2IDX(rte_be_to_cpu_32(esp->spi))];
	if (rte_be_to_cpu_32(esp->spi) != sa->spi)
		return;

	switch (sa->flags) {
	case IP4_TUNNEL:
		src4_addr = RTE_PTR_ADD(ip, offsetof(struct ip, ip_src));
		if ((ip->ip_v == IPVERSION) &&
				(sa->src.ip4 == *src4_addr) &&
				(sa->dst.ip4 == *(src4_addr + 1)))
			*sa_ret = sa;
		break;
	case IP6_TUNNEL:
		src6_addr = RTE_PTR_ADD(ip, offsetof(struct ip6_hdr, ip6_src));
		if ((ip->ip_v == IP6_VERSION) &&
				!memcmp(&sa->src.ip6, src6_addr, 16) &&
				!memcmp(&sa->dst.ip6, src6_addr + 16, 16))
			*sa_ret = sa;
		break;
	case TRANSPORT:
		*sa_ret = sa;
	}
}

void
inbound_sa_lookup(struct sa_ctx *sa_ctx, struct rte_mbuf *pkts[],
		struct ipsec_sa *sa[], uint16_t nb_pkts)
{
	uint32_t i;

	for (i = 0; i < nb_pkts; i++)
		single_inbound_lookup(sa_ctx->sa, pkts[i], &sa[i]);
}

void
outbound_sa_lookup(struct sa_ctx *sa_ctx, uint32_t sa_idx[],
		struct ipsec_sa *sa[], uint16_t nb_pkts)
{
	uint32_t i;

	for (i = 0; i < nb_pkts; i++)
		sa[i] = &sa_ctx->sa[sa_idx[i]];
}
