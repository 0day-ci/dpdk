/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2016 Intel Corporation. All rights reserved.
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
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <string.h>
#include <sys/queue.h>
#include <stdarg.h>
#include <errno.h>
#include <getopt.h>
#include <stdbool.h>
#include <netinet/in.h>

#include <rte_debug.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_mempool.h>
#include <rte_cycles.h>
#include <rte_mbuf.h>
#include <rte_ip.h>
#include <rte_tcp.h>
#include <rte_udp.h>
#include <rte_hash.h>

#include "l3fwd.h"

#if defined(RTE_MACHINE_CPUFLAG_SSE4_2) || defined(RTE_MACHINE_CPUFLAG_CRC32)
#define EM_HASH_CRC 1
#endif

#ifdef EM_HASH_CRC
#include <rte_hash_crc.h>
#define DEFAULT_HASH_FUNC       rte_hash_crc
#else
#include <rte_jhash.h>
#define DEFAULT_HASH_FUNC       rte_jhash
#endif

#define IPV6_ADDR_LEN 16

struct ipv4_5tuple {
	uint32_t ip_dst;
	uint32_t ip_src;
	uint16_t port_dst;
	uint16_t port_src;
	uint8_t  proto;
} __attribute__((__packed__));

union ipv4_5tuple_host {
	struct {
		uint8_t  pad0;
		uint8_t  proto;
		uint16_t pad1;
		uint32_t ip_src;
		uint32_t ip_dst;
		uint16_t port_src;
		uint16_t port_dst;
	};
	xmm_t xmm;
};

#define XMM_NUM_IN_IPV6_5TUPLE 3

struct ipv6_5tuple {
	union {
		uint8_t  ip_dst[IPV6_ADDR_LEN];
		uint32_t ip32_dst[4];
	};
	union {
		uint8_t  ip_src[IPV6_ADDR_LEN];
		uint32_t ip32_src[4];
	};
	uint16_t port_dst;
	uint16_t port_src;
	uint8_t  proto;
} __attribute__((__packed__));

union ipv6_5tuple_host {
	struct {
		uint16_t pad0;
		uint8_t  proto;
		uint8_t  pad1;
		uint8_t  ip_src[IPV6_ADDR_LEN];
		uint8_t  ip_dst[IPV6_ADDR_LEN];
		uint16_t port_src;
		uint16_t port_dst;
		uint64_t reserve;
	};
	xmm_t xmm[XMM_NUM_IN_IPV6_5TUPLE];
};

enum {
	CB_FLD_DST_ADDR,
	CB_FLD_SRC_ADDR,
	CB_FLD_DST_PORT,
	CB_FLD_SRC_PORT,
	CB_FLD_PROTO,
	CB_FLD_IF_OUT,
	CB_FLD_MAX
};

struct em_rule {
	union {
		struct ipv4_5tuple v4_key;
		struct ipv6_5tuple v6_key;
	};
	uint8_t if_out;
};

struct rte_hash *ipv4_l3fwd_em_lookup_struct[NB_SOCKETS];
struct rte_hash *ipv6_l3fwd_em_lookup_struct[NB_SOCKETS];

static inline uint32_t
ipv4_hash_crc(const void *data, __rte_unused uint32_t data_len,
		uint32_t init_val)
{
	const union ipv4_5tuple_host *k;
	uint32_t t;
	const uint32_t *p;

	k = data;
	t = k->proto;
	p = (const uint32_t *)&k->port_src;

#ifdef EM_HASH_CRC
	init_val = rte_hash_crc_4byte(t, init_val);
	init_val = rte_hash_crc_4byte(k->ip_src, init_val);
	init_val = rte_hash_crc_4byte(k->ip_dst, init_val);
	init_val = rte_hash_crc_4byte(*p, init_val);
#else
	init_val = rte_jhash_1word(t, init_val);
	init_val = rte_jhash_1word(k->ip_src, init_val);
	init_val = rte_jhash_1word(k->ip_dst, init_val);
	init_val = rte_jhash_1word(*p, init_val);
#endif

	return init_val;
}

static inline uint32_t
ipv6_hash_crc(const void *data, __rte_unused uint32_t data_len,
		uint32_t init_val)
{
	const union ipv6_5tuple_host *k;
	uint32_t t;
	const uint32_t *p;
#ifdef EM_HASH_CRC
	const uint32_t  *ip_src0, *ip_src1, *ip_src2, *ip_src3;
	const uint32_t  *ip_dst0, *ip_dst1, *ip_dst2, *ip_dst3;
#endif

	k = data;
	t = k->proto;
	p = (const uint32_t *)&k->port_src;

#ifdef EM_HASH_CRC
	ip_src0 = (const uint32_t *) k->ip_src;
	ip_src1 = (const uint32_t *)(k->ip_src+4);
	ip_src2 = (const uint32_t *)(k->ip_src+8);
	ip_src3 = (const uint32_t *)(k->ip_src+12);
	ip_dst0 = (const uint32_t *) k->ip_dst;
	ip_dst1 = (const uint32_t *)(k->ip_dst+4);
	ip_dst2 = (const uint32_t *)(k->ip_dst+8);
	ip_dst3 = (const uint32_t *)(k->ip_dst+12);
	init_val = rte_hash_crc_4byte(t, init_val);
	init_val = rte_hash_crc_4byte(*ip_src0, init_val);
	init_val = rte_hash_crc_4byte(*ip_src1, init_val);
	init_val = rte_hash_crc_4byte(*ip_src2, init_val);
	init_val = rte_hash_crc_4byte(*ip_src3, init_val);
	init_val = rte_hash_crc_4byte(*ip_dst0, init_val);
	init_val = rte_hash_crc_4byte(*ip_dst1, init_val);
	init_val = rte_hash_crc_4byte(*ip_dst2, init_val);
	init_val = rte_hash_crc_4byte(*ip_dst3, init_val);
	init_val = rte_hash_crc_4byte(*p, init_val);
#else
	init_val = rte_jhash_1word(t, init_val);
	init_val = rte_jhash(k->ip_src,
			sizeof(uint8_t) * IPV6_ADDR_LEN, init_val);
	init_val = rte_jhash(k->ip_dst,
			sizeof(uint8_t) * IPV6_ADDR_LEN, init_val);
	init_val = rte_jhash_1word(*p, init_val);
#endif
	return init_val;
}

static uint8_t ipv4_l3fwd_out_if[L3FWD_HASH_ENTRIES] __rte_cache_aligned;
static uint8_t ipv6_l3fwd_out_if[L3FWD_HASH_ENTRIES] __rte_cache_aligned;

static rte_xmm_t mask0;
static rte_xmm_t mask1;
static rte_xmm_t mask2;

#if defined(__SSE2__)
static inline xmm_t
em_mask_key(void *key, xmm_t mask)
{
	__m128i data = _mm_loadu_si128((__m128i *)(key));

	return _mm_and_si128(data, mask);
}
#elif defined(RTE_MACHINE_CPUFLAG_NEON)
static inline xmm_t
em_mask_key(void *key, xmm_t mask)
{
	int32x4_t data = vld1q_s32((int32_t *)key);

	return vandq_s32(data, mask);
}
#elif defined(RTE_MACHINE_CPUFLAG_ALTIVEC)
static inline xmm_t
em_mask_key(void *key, xmm_t mask)
{
	xmm_t data = vec_ld(0, (xmm_t *)(key));

	return vec_and(data, mask);
}
#else
#error No vector engine (SSE, NEON, ALTIVEC) available, check your toolchain
#endif

static inline uint8_t
em_get_ipv4_dst_port(void *ipv4_hdr, uint8_t portid, void *lookup_struct)
{
	int ret = 0;
	union ipv4_5tuple_host key;
	struct rte_hash *ipv4_l3fwd_lookup_struct =
		(struct rte_hash *)lookup_struct;

	ipv4_hdr = (uint8_t *)ipv4_hdr + offsetof(struct ipv4_hdr, time_to_live);

	/*
	 * Get 5 tuple: dst port, src port, dst IP address,
	 * src IP address and protocol.
	 */
	key.xmm = em_mask_key(ipv4_hdr, mask0.x);

	/* Find destination port */
	ret = rte_hash_lookup(ipv4_l3fwd_lookup_struct, (const void *)&key);
	return (uint8_t)((ret < 0) ? portid : ipv4_l3fwd_out_if[ret]);
}

static inline uint8_t
em_get_ipv6_dst_port(void *ipv6_hdr,  uint8_t portid, void *lookup_struct)
{
	int ret = 0;
	union ipv6_5tuple_host key;
	struct rte_hash *ipv6_l3fwd_lookup_struct =
		(struct rte_hash *)lookup_struct;

	ipv6_hdr = (uint8_t *)ipv6_hdr + offsetof(struct ipv6_hdr, payload_len);
	void *data0 = ipv6_hdr;
	void *data1 = ((uint8_t *)ipv6_hdr) + sizeof(xmm_t);
	void *data2 = ((uint8_t *)ipv6_hdr) + sizeof(xmm_t) + sizeof(xmm_t);

	/* Get part of 5 tuple: src IP address lower 96 bits and protocol */
	key.xmm[0] = em_mask_key(data0, mask1.x);

	/*
	 * Get part of 5 tuple: dst IP address lower 96 bits
	 * and src IP address higher 32 bits.
	 */
	key.xmm[1] = *(xmm_t *)data1;

	/*
	 * Get part of 5 tuple: dst port and src port
	 * and dst IP address higher 32 bits.
	 */
	key.xmm[2] = em_mask_key(data2, mask2.x);

	/* Find destination port */
	ret = rte_hash_lookup(ipv6_l3fwd_lookup_struct, (const void *)&key);
	return (uint8_t)((ret < 0) ? portid : ipv6_l3fwd_out_if[ret]);
}

#if defined(__SSE4_1__)
#if defined(NO_HASH_MULTI_LOOKUP)
#include "l3fwd_em_sse.h"
#else
#include "l3fwd_em_hlm_sse.h"
#endif
#else
#include "l3fwd_em.h"
#endif

static int
em_parse_v6_addr(const char *in, const char **end, uint32_t v[IPV6_ADDR_U32],
	char dlm)
{
	uint32_t addr[IPV6_ADDR_U16];

	GET_CB_FIELD(in, addr[0], 16, UINT16_MAX, ':');
	GET_CB_FIELD(in, addr[1], 16, UINT16_MAX, ':');
	GET_CB_FIELD(in, addr[2], 16, UINT16_MAX, ':');
	GET_CB_FIELD(in, addr[3], 16, UINT16_MAX, ':');
	GET_CB_FIELD(in, addr[4], 16, UINT16_MAX, ':');
	GET_CB_FIELD(in, addr[5], 16, UINT16_MAX, ':');
	GET_CB_FIELD(in, addr[6], 16, UINT16_MAX, ':');
	GET_CB_FIELD(in, addr[7], 16, UINT16_MAX, dlm);

	*end = in;

	v[0] = (addr[0] << 16) + addr[1];
	v[1] = (addr[2] << 16) + addr[3];
	v[2] = (addr[4] << 16) + addr[5];
	v[3] = (addr[6] << 16) + addr[7];

	return 0;
}

static int
em_parse_v6_net(const char *in, uint32_t *v)
{
	int32_t rc;
	const char *mp;

	/* get address. */
	rc = em_parse_v6_addr(in, &mp, v, 0);
	if (rc != 0) {
		RTE_LOG(ERR, L3FWD, "parse_v6_addr failed %d\n", rc);
		return rc;
	}
	return 0;
}

static int
em_parse_v6_rule(char *str, struct em_rule *v)
{
	int i, rc;
	char *s, *sp, *in[CB_FLD_MAX];
	static const char *dlm = " \t\n";
	int dim = CB_FLD_MAX;
	s = str;

	for (i = 0; i != dim; i++, s = NULL) {
		in[i] = strtok_r(s, dlm, &sp);
		if (in[i] == NULL) {
			RTE_LOG(ERR, L3FWD, "strtok failed\n");
			return -EINVAL;
		}
	}

	rc = em_parse_v6_net(in[CB_FLD_DST_ADDR], v->v6_key.ip32_dst);
	if (rc != 0) {
		RTE_LOG(ERR, L3FWD, "parse_v6_net failed for dst %d\n", rc);
		return rc;
	}
	rc = em_parse_v6_net(in[CB_FLD_SRC_ADDR], v->v6_key.ip32_src);
	if (rc != 0) {
		RTE_LOG(ERR, L3FWD, "parse_v6_net failed for src %d\n", rc);
		return rc;
	}
	/* source port. */
	GET_CB_FIELD(in[CB_FLD_SRC_PORT], v->v6_key.port_src, 0, UINT16_MAX, 0);
	/* destination port. */
	GET_CB_FIELD(in[CB_FLD_DST_PORT], v->v6_key.port_dst, 0, UINT16_MAX, 0);
	/* protocol. */
	GET_CB_FIELD(in[CB_FLD_PROTO], v->v6_key.proto, 0, UINT8_MAX, 0);
	/* out interface. */
	GET_CB_FIELD(in[CB_FLD_IF_OUT], v->if_out, 0, UINT8_MAX, 0);

	return 0;
}

static int
em_parse_v4_net(const char *in, uint32_t *addr)
{
	uint8_t a, b, c, d;

	GET_CB_FIELD(in, a, 0, UINT8_MAX, '.');
	GET_CB_FIELD(in, b, 0, UINT8_MAX, '.');
	GET_CB_FIELD(in, c, 0, UINT8_MAX, '.');
	GET_CB_FIELD(in, d, 0, UINT8_MAX, 0);

	addr[0] = IPv4(a, b, c, d);
	return 0;
}

static int
em_parse_v4_rule(char *str, struct em_rule *v)
{
	int i, rc;
	char *s, *sp, *in[CB_FLD_MAX];
	static const char *dlm = " \t\n";
	int dim = CB_FLD_MAX;
	s = str;

	for (i = 0; i != dim; i++, s = NULL) {
		in[i] = strtok_r(s, dlm, &sp);
		if (in[i] == NULL) {
			RTE_LOG(ERR, L3FWD, "parse_v4_rule strtok fail\n");
			return -EINVAL;
		}
	}

	rc = em_parse_v4_net(in[CB_FLD_DST_ADDR], &(v->v4_key.ip_dst));
	if (rc != 0) {
		RTE_LOG(ERR, L3FWD, "parse_v4_net dst failed %d\n", rc);
		return rc;
	}
	rc = em_parse_v4_net(in[CB_FLD_SRC_ADDR], &(v->v4_key.ip_src));
	if (rc != 0) {
		RTE_LOG(ERR, L3FWD, "parse_v4_net src failed %d\n", rc);
		return rc;
	}
	/* source port. */
	GET_CB_FIELD(in[CB_FLD_SRC_PORT], v->v4_key.port_src, 0, UINT16_MAX, 0);
	/* destination port. */
	GET_CB_FIELD(in[CB_FLD_DST_PORT], v->v4_key.port_dst, 0, UINT16_MAX, 0);
	/* protocol. */
	GET_CB_FIELD(in[CB_FLD_PROTO], v->v4_key.proto, 0, UINT8_MAX, 0);
	/* out interface. */
	GET_CB_FIELD(in[CB_FLD_IF_OUT], v->if_out, 0, UINT8_MAX, 0);

	return 0;
}

static int
em_add_rules(const char *rule_path,
		struct em_rule **proute_base,
		unsigned int *proute_num,
		int (*parser)(char *, struct em_rule*))
{
	uint8_t *route_rules;
	struct em_rule *next;
	unsigned int route_num = 0;
	unsigned int route_cnt = 0;
	char buff[LINE_MAX];
	FILE *fh = fopen(rule_path, "rb");
	unsigned int i = 0, rule_size = sizeof(*next);

	if (fh == NULL)
		rte_exit(EXIT_FAILURE, "%s: Open %s failed\n", __func__,
			rule_path);

	while ((fgets(buff, LINE_MAX, fh) != NULL)) {
		if (buff[0] == EM_LEAD_CHAR)
			route_num++;
	}

	if (route_num == 0)
		rte_exit(EXIT_FAILURE, "Not find any route entries in %s!\n",
				rule_path);

	fseek(fh, 0, SEEK_SET);

	route_rules = calloc(route_num, rule_size);

	if (route_rules == NULL)
		rte_exit(EXIT_FAILURE, "%s: failed to malloc memory\n",
			__func__);

	i = 0;
	while (fgets(buff, LINE_MAX, fh) != NULL) {
		i++;

		if (is_bypass_line(buff))
			continue;

		char s = buff[0];

		/* Route entry */
		if (s == EM_LEAD_CHAR)
			next = (struct em_rule *)(route_rules +
				route_cnt * rule_size);

		/* Illegal line */
		else
			rte_exit(EXIT_FAILURE,
				"%s Line %u: should start with leading "
				"char %c\n",
				rule_path, i, EM_LEAD_CHAR);

		if (parser(buff + 1, next) != 0)
			rte_exit(EXIT_FAILURE,
				"%s Line %u: parse rules error\n",
				rule_path, i);

		route_cnt++;
	}

	fclose(fh);

	*proute_base = (struct em_rule *)route_rules;
	*proute_num = route_cnt;

	return 0;
}

static int
check_em_config(void)
{
	if (parm_config.rule_ipv4_name == NULL) {
		RTE_LOG(ERR, L3FWD, "EM IPv4 rule file not specified\n");
		return -1;
	} else if (parm_config.rule_ipv6_name == NULL) {
		RTE_LOG(ERR, L3FWD, "EM IPv6 rule file not specified\n");
		return -1;
	}

	return 0;
}

static void
convert_ipv4_5tuple(struct ipv4_5tuple *key1,
		union ipv4_5tuple_host *key2)
{
	key2->ip_dst = rte_cpu_to_be_32(key1->ip_dst);
	key2->ip_src = rte_cpu_to_be_32(key1->ip_src);
	key2->port_dst = rte_cpu_to_be_16(key1->port_dst);
	key2->port_src = rte_cpu_to_be_16(key1->port_src);
	key2->proto = key1->proto;
	key2->pad0 = 0;
	key2->pad1 = 0;
}

static void
convert_ipv6_5tuple(struct ipv6_5tuple *key1,
		union ipv6_5tuple_host *key2)
{
	uint32_t i;

	for (i = 0; i < 16; i++) {
		key2->ip_dst[i] = key1->ip_dst[i];
		key2->ip_src[i] = key1->ip_src[i];
	}
	key2->port_dst = rte_cpu_to_be_16(key1->port_dst);
	key2->port_src = rte_cpu_to_be_16(key1->port_src);
	key2->proto = key1->proto;
	key2->pad0 = 0;
	key2->pad1 = 0;
	key2->reserve = 0;
}

#define BYTE_VALUE_MAX 256
#define ALL_32_BITS 0xffffffff
#define BIT_8_TO_15 0x0000ff00

static inline void
populate_ipv4_few_flow_into_table(const struct rte_hash *h)
{
	uint32_t i;
	int32_t ret;
	struct em_rule *route_base_v4;
	unsigned int route_num_v4 = 0;

	mask0 = (rte_xmm_t){.u32 = {BIT_8_TO_15, ALL_32_BITS,
				ALL_32_BITS, ALL_32_BITS} };

	/* Load rules from the input file */
	if (em_add_rules(parm_config.rule_ipv4_name,
		&route_base_v4, &route_num_v4,
		&em_parse_v4_rule) < 0)
		rte_exit(EXIT_FAILURE, "Failed to add em v4 rules\n");

	for (i = 0; i < route_num_v4; i++) {
		struct em_rule  entry;
		union ipv4_5tuple_host newkey;

		entry = route_base_v4[i];
		convert_ipv4_5tuple(&entry.v4_key, &newkey);
		ret = rte_hash_add_key(h, (void *) &newkey);
		if (ret < 0) {
			rte_exit(EXIT_FAILURE, "Unable to add entry %" PRIu32
				" to the l3fwd hash.\n", i);
		}
		ipv4_l3fwd_out_if[ret] = entry.if_out;
	}
	printf("Hash: Adding 0x%" PRIx64 " keys\n",
		(uint64_t)route_num_v4);
}

#define BIT_16_TO_23 0x00ff0000
static inline void
populate_ipv6_few_flow_into_table(const struct rte_hash *h)
{
	uint32_t i;
	int32_t ret;
	struct em_rule *route_base_v6;
	unsigned int route_num_v6 = 0;

	mask1 = (rte_xmm_t){.u32 = {BIT_16_TO_23, ALL_32_BITS,
				ALL_32_BITS, ALL_32_BITS} };

	mask2 = (rte_xmm_t){.u32 = {ALL_32_BITS, ALL_32_BITS, 0, 0} };

	/* Load rules from the input file */
	if (em_add_rules(parm_config.rule_ipv6_name,
		&route_base_v6, &route_num_v6,
		&em_parse_v6_rule) < 0)
		rte_exit(EXIT_FAILURE, "Failed to add em v6 rules\n");

	for (i = 0; i < route_num_v6; i++) {
		struct em_rule entry;
		union ipv6_5tuple_host newkey;

		entry = route_base_v6[i];
		convert_ipv6_5tuple(&entry.v6_key, &newkey);
		ret = rte_hash_add_key(h, (void *) &newkey);
		if (ret < 0) {
			rte_exit(EXIT_FAILURE, "Unable to add entry %" PRIu32
				" to the l3fwd hash.\n", i);
		}
		ipv6_l3fwd_out_if[ret] = entry.if_out;
	}
	printf("Hash: Adding 0x%" PRIx64 "keys\n",
		(uint64_t)route_num_v6);
}

#define NUMBER_PORT_USED 4
static inline void
populate_ipv4_many_flow_into_table(const struct rte_hash *h,
		unsigned int nr_flow)
{
	unsigned i;
	struct em_rule *route_base_v4;
	unsigned int route_num_v4 = 0;

	mask0 = (rte_xmm_t){.u32 = {BIT_8_TO_15, ALL_32_BITS,
				ALL_32_BITS, ALL_32_BITS} };

	/* Load rules from the input file */
	if (em_add_rules(parm_config.rule_ipv4_name,
		&route_base_v4, &route_num_v4,
		&em_parse_v4_rule) < 0)
		rte_exit(EXIT_FAILURE, "Failed to add em v4 rules\n");

	for (i = 0; i < nr_flow; i++) {
		struct em_rule entry;
		union ipv4_5tuple_host newkey;

		uint8_t a = (uint8_t)
			((i/NUMBER_PORT_USED)%BYTE_VALUE_MAX);
		uint8_t b = (uint8_t)
			(((i/NUMBER_PORT_USED)/BYTE_VALUE_MAX)%BYTE_VALUE_MAX);
		uint8_t c = (uint8_t)
			((i/NUMBER_PORT_USED)/(BYTE_VALUE_MAX*BYTE_VALUE_MAX));

		/* Create the ipv4 exact match flow */
		memset(&entry, 0, sizeof(entry));
		switch (i & (NUMBER_PORT_USED - 1)) {
		case 0:
			entry = route_base_v4[0];
			entry.v4_key.ip_dst = IPv4(101, c, b, a);
			break;
		case 1:
			entry = route_base_v4[1];
			entry.v4_key.ip_dst = IPv4(201, c, b, a);
			break;
		case 2:
			entry = route_base_v4[2];
			entry.v4_key.ip_dst = IPv4(111, c, b, a);
			break;
		case 3:
			entry = route_base_v4[3];
			entry.v4_key.ip_dst = IPv4(211, c, b, a);
			break;
		};
		convert_ipv4_5tuple(&entry.v4_key, &newkey);
		int32_t ret = rte_hash_add_key(h, (void *) &newkey);

		if (ret < 0)
			rte_exit(EXIT_FAILURE, "Unable to add entry %u\n", i);

		ipv4_l3fwd_out_if[ret] = (uint8_t) entry.if_out;

	}
	printf("Hash: Adding 0x%x keys\n", nr_flow);
}

static inline void
populate_ipv6_many_flow_into_table(const struct rte_hash *h,
		unsigned int nr_flow)
{
	unsigned i;
	struct em_rule *route_base_v6;
	unsigned int route_num_v6 = 0;

	mask1 = (rte_xmm_t){.u32 = {BIT_16_TO_23, ALL_32_BITS,
				ALL_32_BITS, ALL_32_BITS} };
	mask2 = (rte_xmm_t){.u32 = {ALL_32_BITS, ALL_32_BITS, 0, 0} };

	/* Load rules from the input file */
	if (em_add_rules(parm_config.rule_ipv6_name,
		&route_base_v6, &route_num_v6,
		&em_parse_v6_rule) < 0)
		rte_exit(EXIT_FAILURE, "Failed to add em v6 rules\n");

	for (i = 0; i < nr_flow; i++) {
		struct em_rule entry;
		union ipv6_5tuple_host newkey;

		uint8_t a = (uint8_t)
			((i/NUMBER_PORT_USED)%BYTE_VALUE_MAX);
		uint8_t b = (uint8_t)
			(((i/NUMBER_PORT_USED)/BYTE_VALUE_MAX)%BYTE_VALUE_MAX);
		uint8_t c = (uint8_t)
			((i/NUMBER_PORT_USED)/(BYTE_VALUE_MAX*BYTE_VALUE_MAX));

		/* Create the ipv6 exact match flow */
		memset(&entry, 0, sizeof(entry));
		switch (i & (NUMBER_PORT_USED - 1)) {
		case 0:
			entry = route_base_v6[0];
			break;
		case 1:
			entry = route_base_v6[1];
			break;
		case 2:
			entry = route_base_v6[2];
			break;
		case 3:
			entry = route_base_v6[3];
			break;
		};
		entry.v6_key.ip_dst[13] = c;
		entry.v6_key.ip_dst[14] = b;
		entry.v6_key.ip_dst[15] = a;
		convert_ipv6_5tuple(&entry.v6_key, &newkey);
		int32_t ret = rte_hash_add_key(h, (void *) &newkey);

		if (ret < 0)
			rte_exit(EXIT_FAILURE, "Unable to add entry %u\n", i);

		ipv6_l3fwd_out_if[ret] = (uint8_t) entry.if_out;

	}
	printf("Hash: Adding 0x%x keys\n", nr_flow);
}

/* Requirements:
 * 1. IP packets without extension;
 * 2. L4 payload should be either TCP or UDP.
 */
int
em_check_ptype(int portid)
{
	int i, ret;
	int ptype_l3_ipv4_ext = 0;
	int ptype_l3_ipv6_ext = 0;
	int ptype_l4_tcp = 0;
	int ptype_l4_udp = 0;
	uint32_t ptype_mask = RTE_PTYPE_L3_MASK | RTE_PTYPE_L4_MASK;

	ret = rte_eth_dev_get_supported_ptypes(portid, ptype_mask, NULL, 0);
	if (ret <= 0)
		return 0;

	uint32_t ptypes[ret];

	ret = rte_eth_dev_get_supported_ptypes(portid, ptype_mask, ptypes, ret);
	for (i = 0; i < ret; ++i) {
		switch (ptypes[i]) {
		case RTE_PTYPE_L3_IPV4_EXT:
			ptype_l3_ipv4_ext = 1;
			break;
		case RTE_PTYPE_L3_IPV6_EXT:
			ptype_l3_ipv6_ext = 1;
			break;
		case RTE_PTYPE_L4_TCP:
			ptype_l4_tcp = 1;
			break;
		case RTE_PTYPE_L4_UDP:
			ptype_l4_udp = 1;
			break;
		}
	}

	if (ptype_l3_ipv4_ext == 0)
		printf("port %d cannot parse RTE_PTYPE_L3_IPV4_EXT\n", portid);
	if (ptype_l3_ipv6_ext == 0)
		printf("port %d cannot parse RTE_PTYPE_L3_IPV6_EXT\n", portid);
	if (!ptype_l3_ipv4_ext || !ptype_l3_ipv6_ext)
		return 0;

	if (ptype_l4_tcp == 0)
		printf("port %d cannot parse RTE_PTYPE_L4_TCP\n", portid);
	if (ptype_l4_udp == 0)
		printf("port %d cannot parse RTE_PTYPE_L4_UDP\n", portid);
	if (ptype_l4_tcp && ptype_l4_udp)
		return 1;

	return 0;
}

static inline void
em_parse_ptype(struct rte_mbuf *m)
{
	struct ether_hdr *eth_hdr;
	uint32_t packet_type = RTE_PTYPE_UNKNOWN;
	uint16_t ether_type;
	void *l3;
	int hdr_len;
	struct ipv4_hdr *ipv4_hdr;
	struct ipv6_hdr *ipv6_hdr;

	eth_hdr = rte_pktmbuf_mtod(m, struct ether_hdr *);
	ether_type = eth_hdr->ether_type;
	l3 = (uint8_t *)eth_hdr + sizeof(struct ether_hdr);
	if (ether_type == rte_cpu_to_be_16(ETHER_TYPE_IPv4)) {
		ipv4_hdr = (struct ipv4_hdr *)l3;
		hdr_len = (ipv4_hdr->version_ihl & IPV4_HDR_IHL_MASK) *
			  IPV4_IHL_MULTIPLIER;
		if (hdr_len == sizeof(struct ipv4_hdr)) {
			packet_type |= RTE_PTYPE_L3_IPV4;
			if (ipv4_hdr->next_proto_id == IPPROTO_TCP)
				packet_type |= RTE_PTYPE_L4_TCP;
			else if (ipv4_hdr->next_proto_id == IPPROTO_UDP)
				packet_type |= RTE_PTYPE_L4_UDP;
		} else
			packet_type |= RTE_PTYPE_L3_IPV4_EXT;
	} else if (ether_type == rte_cpu_to_be_16(ETHER_TYPE_IPv4)) {
		ipv6_hdr = (struct ipv6_hdr *)l3;
		if (ipv6_hdr->proto == IPPROTO_TCP)
			packet_type |= RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_TCP;
		else if (ipv6_hdr->proto == IPPROTO_UDP)
			packet_type |= RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_UDP;
		else
			packet_type |= RTE_PTYPE_L3_IPV6_EXT_UNKNOWN;
	}

	m->packet_type = packet_type;
}

uint16_t
em_cb_parse_ptype(uint8_t port __rte_unused, uint16_t queue __rte_unused,
		  struct rte_mbuf *pkts[], uint16_t nb_pkts,
		  uint16_t max_pkts __rte_unused,
		  void *user_param __rte_unused)
{
	unsigned i;

	for (i = 0; i < nb_pkts; ++i)
		em_parse_ptype(pkts[i]);

	return nb_pkts;
}

/* main processing loop */
int
em_main_loop(__attribute__((unused)) void *dummy)
{
	struct rte_mbuf *pkts_burst[MAX_PKT_BURST];
	unsigned lcore_id;
	uint64_t prev_tsc, diff_tsc, cur_tsc;
	int i, nb_rx;
	uint8_t portid, queueid;
	struct lcore_conf *qconf;
	const uint64_t drain_tsc = (rte_get_tsc_hz() + US_PER_S - 1) /
		US_PER_S * BURST_TX_DRAIN_US;

	prev_tsc = 0;

	lcore_id = rte_lcore_id();
	qconf = &lcore_conf[lcore_id];

	if (qconf->n_rx_queue == 0) {
		RTE_LOG(INFO, L3FWD, "lcore %u has nothing to do\n", lcore_id);
		return 0;
	}

	RTE_LOG(INFO, L3FWD, "entering main loop on lcore %u\n", lcore_id);

	for (i = 0; i < qconf->n_rx_queue; i++) {

		portid = qconf->rx_queue_list[i].port_id;
		queueid = qconf->rx_queue_list[i].queue_id;
		RTE_LOG(INFO, L3FWD,
			" -- lcoreid=%u portid=%hhu rxqueueid=%hhu\n",
			lcore_id, portid, queueid);
	}

	while (!force_quit) {

		cur_tsc = rte_rdtsc();

		/*
		 * TX burst queue drain
		 */
		diff_tsc = cur_tsc - prev_tsc;
		if (unlikely(diff_tsc > drain_tsc)) {

			for (i = 0; i < qconf->n_tx_port; ++i) {
				portid = qconf->tx_port_id[i];
				if (qconf->tx_mbufs[portid].len == 0)
					continue;
				send_burst(qconf,
					qconf->tx_mbufs[portid].len,
					portid);
				qconf->tx_mbufs[portid].len = 0;
			}

			prev_tsc = cur_tsc;
		}

		/*
		 * Read packet from RX queues
		 */
		for (i = 0; i < qconf->n_rx_queue; ++i) {
			portid = qconf->rx_queue_list[i].port_id;
			queueid = qconf->rx_queue_list[i].queue_id;
			nb_rx = rte_eth_rx_burst(portid, queueid, pkts_burst,
				MAX_PKT_BURST);
			if (nb_rx == 0)
				continue;

#if defined(__SSE4_1__)
			l3fwd_em_send_packets(nb_rx, pkts_burst,
							portid, qconf);
#else
			l3fwd_em_no_opt_send_packets(nb_rx, pkts_burst,
							portid, qconf);
#endif /* __SSE_4_1__ */
		}
	}

	return 0;
}

/*
 * Initialize exact match (hash) parameters.
 */
void
setup_hash(const int socketid)
{
	struct rte_hash_parameters ipv4_l3fwd_hash_params = {
		.name = NULL,
		.entries = L3FWD_HASH_ENTRIES,
		.key_len = sizeof(union ipv4_5tuple_host),
		.hash_func = ipv4_hash_crc,
		.hash_func_init_val = 0,
	};

	struct rte_hash_parameters ipv6_l3fwd_hash_params = {
		.name = NULL,
		.entries = L3FWD_HASH_ENTRIES,
		.key_len = sizeof(union ipv6_5tuple_host),
		.hash_func = ipv6_hash_crc,
		.hash_func_init_val = 0,
	};

	char s[64];

	if (check_em_config() != 0)
		rte_exit(EXIT_FAILURE, "Failed to get valid EM options\n");

	/* create ipv4 hash */
	snprintf(s, sizeof(s), "ipv4_l3fwd_hash_%d", socketid);
	ipv4_l3fwd_hash_params.name = s;
	ipv4_l3fwd_hash_params.socket_id = socketid;
	ipv4_l3fwd_em_lookup_struct[socketid] =
		rte_hash_create(&ipv4_l3fwd_hash_params);
	if (ipv4_l3fwd_em_lookup_struct[socketid] == NULL)
		rte_exit(EXIT_FAILURE,
			"Unable to create the v4 hash on socket %d\n",
			socketid);

	/* create ipv6 hash */
	snprintf(s, sizeof(s), "ipv6_l3fwd_hash_%d", socketid);
	ipv6_l3fwd_hash_params.name = s;
	ipv6_l3fwd_hash_params.socket_id = socketid;
	ipv6_l3fwd_em_lookup_struct[socketid] =
		rte_hash_create(&ipv6_l3fwd_hash_params);
	if (ipv6_l3fwd_em_lookup_struct[socketid] == NULL)
		rte_exit(EXIT_FAILURE,
			"Unable to create the v6 hash on socket %d\n",
			socketid);

	if (hash_entry_number != HASH_ENTRY_NUMBER_DEFAULT) {
		/* For testing hash matching with a large number
		 * of flows we generate millions of IP 5-tuples
		 * with an incremented dst address to initialize
		 * the hash table.
		 */
		if (ipv6 == 0) {
			/* populate the ipv4 hash */
			populate_ipv4_many_flow_into_table(
				ipv4_l3fwd_em_lookup_struct[socketid],
				hash_entry_number);
		} else {
			/* populate the ipv6 hash */
			populate_ipv6_many_flow_into_table(
				ipv6_l3fwd_em_lookup_struct[socketid],
			hash_entry_number);
		}
	} else {
		/*
		 * Use data in ipv4/ipv6 l3fwd lookup table
		 * directly to initialize the hash table.
		 */
		if (ipv6 == 0) {
			/* populate the ipv4 hash */
			populate_ipv4_few_flow_into_table(
				ipv4_l3fwd_em_lookup_struct[socketid]);
		} else {
			/* populate the ipv6 hash */
			populate_ipv6_few_flow_into_table(
				ipv6_l3fwd_em_lookup_struct[socketid]);
		}
	}
}

/* Return ipv4/ipv6 em fwd lookup struct. */
void *
em_get_ipv4_l3fwd_lookup_struct(const int socketid)
{
	return ipv4_l3fwd_em_lookup_struct[socketid];
}

void *
em_get_ipv6_l3fwd_lookup_struct(const int socketid)
{
	return ipv6_l3fwd_em_lookup_struct[socketid];
}
