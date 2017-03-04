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

#ifndef __L3FWD_ACL_H__
#define __L3FWD_ACL_H__

#define MAX_ACL_RULE_NUM	100000
#define DEFAULT_MAX_CATEGORIES	1
#define L3FWD_ACL_IPV4_NAME	"l3fwd-acl-ipv4"
#define L3FWD_ACL_IPV6_NAME	"l3fwd-acl-ipv6"
#define ACL_DENY_SIGNATURE	0xf0000000
#define RTE_LOGTYPE_L3FWDACL	RTE_LOGTYPE_USER3
#define acl_log(format, ...)	RTE_LOG(ERR, L3FWDACL, format, ##__VA_ARGS__)
#define uint32_t_to_char(ip, a, b, c, d) do {\
		*a = (unsigned char)(ip >> 24 & 0xff);\
		*b = (unsigned char)(ip >> 16 & 0xff);\
		*c = (unsigned char)(ip >> 8 & 0xff);\
		*d = (unsigned char)(ip & 0xff);\
	} while (0)
#define OFF_ETHHEAD	(sizeof(struct ether_hdr))
#define OFF_IPV42PROTO (offsetof(struct ipv4_hdr, next_proto_id))
#define OFF_IPV62PROTO (offsetof(struct ipv6_hdr, proto))
#define MBUF_IPV4_2PROTO(m)	\
	rte_pktmbuf_mtod_offset((m), uint8_t *, OFF_ETHHEAD + OFF_IPV42PROTO)
#define MBUF_IPV6_2PROTO(m)	\
	rte_pktmbuf_mtod_offset((m), uint8_t *, OFF_ETHHEAD + OFF_IPV62PROTO)

#define GET_CB_FIELD(in, fd, base, lim, dlm)	do {            \
	unsigned long val;                                      \
	char *end;                                              \
	errno = 0;                                              \
	val = strtoul((in), &end, (base));                      \
	if (errno != 0 || end[0] != (dlm) || val > (lim))       \
		return -EINVAL;                               \
	(fd) = (typeof(fd))val;                                 \
	(in) = end + 1;                                         \
} while (0)

/*
  * ACL rules should have higher priorities than route ones to ensure ACL rule
  * always be found when input packets have multi-matches in the database.
  * A exception case is performance measure, which can define route rules with
  * higher priority and route rules will always be returned in each lookup.
  * Reserve range from ACL_RULE_PRIORITY_MAX + 1 to
  * RTE_ACL_MAX_PRIORITY for route entries in performance measure
  */
#define ACL_RULE_PRIORITY_MAX 0x10000000

/*
  * Forward port info save in ACL lib starts from 1
  * since ACL assume 0 is invalid.
  * So, need add 1 when saving and minus 1 when forwarding packets.
  */
#define FWD_PORT_SHIFT 1

static inline void
print_one_ipv4_rule(struct acl4_rule *rule, int extra)
{
	unsigned char a, b, c, d;

	uint32_t_to_char(rule->field[SRC_FIELD_IPV4].value.u32,
			&a, &b, &c, &d);
	printf("%hhu.%hhu.%hhu.%hhu/%u ", a, b, c, d,
			rule->field[SRC_FIELD_IPV4].mask_range.u32);
	uint32_t_to_char(rule->field[DST_FIELD_IPV4].value.u32,
			&a, &b, &c, &d);
	printf("%hhu.%hhu.%hhu.%hhu/%u ", a, b, c, d,
			rule->field[DST_FIELD_IPV4].mask_range.u32);
	printf("%hu : %hu %hu : %hu 0x%hhx/0x%hhx ",
		rule->field[SRCP_FIELD_IPV4].value.u16,
		rule->field[SRCP_FIELD_IPV4].mask_range.u16,
		rule->field[DSTP_FIELD_IPV4].value.u16,
		rule->field[DSTP_FIELD_IPV4].mask_range.u16,
		rule->field[PROTO_FIELD_IPV4].value.u8,
		rule->field[PROTO_FIELD_IPV4].mask_range.u8);
	if (extra)
		printf("0x%x-0x%x-0x%x ",
			rule->data.category_mask,
			rule->data.priority,
			rule->data.userdata);
}

static inline void
print_one_ipv6_rule(struct acl6_rule *rule, int extra)
{
	unsigned char a, b, c, d;

	uint32_t_to_char(rule->field[SRC1_FIELD_IPV6].value.u32,
		&a, &b, &c, &d);
	printf("%.2x%.2x:%.2x%.2x", a, b, c, d);
	uint32_t_to_char(rule->field[SRC2_FIELD_IPV6].value.u32,
		&a, &b, &c, &d);
	printf(":%.2x%.2x:%.2x%.2x", a, b, c, d);
	uint32_t_to_char(rule->field[SRC3_FIELD_IPV6].value.u32,
		&a, &b, &c, &d);
	printf(":%.2x%.2x:%.2x%.2x", a, b, c, d);
	uint32_t_to_char(rule->field[SRC4_FIELD_IPV6].value.u32,
		&a, &b, &c, &d);
	printf(":%.2x%.2x:%.2x%.2x/%u ", a, b, c, d,
			rule->field[SRC1_FIELD_IPV6].mask_range.u32
			+ rule->field[SRC2_FIELD_IPV6].mask_range.u32
			+ rule->field[SRC3_FIELD_IPV6].mask_range.u32
			+ rule->field[SRC4_FIELD_IPV6].mask_range.u32);

	uint32_t_to_char(rule->field[DST1_FIELD_IPV6].value.u32,
		&a, &b, &c, &d);
	printf("%.2x%.2x:%.2x%.2x", a, b, c, d);
	uint32_t_to_char(rule->field[DST2_FIELD_IPV6].value.u32,
		&a, &b, &c, &d);
	printf(":%.2x%.2x:%.2x%.2x", a, b, c, d);
	uint32_t_to_char(rule->field[DST3_FIELD_IPV6].value.u32,
		&a, &b, &c, &d);
	printf(":%.2x%.2x:%.2x%.2x", a, b, c, d);
	uint32_t_to_char(rule->field[DST4_FIELD_IPV6].value.u32,
		&a, &b, &c, &d);
	printf(":%.2x%.2x:%.2x%.2x/%u ", a, b, c, d,
			rule->field[DST1_FIELD_IPV6].mask_range.u32
			+ rule->field[DST2_FIELD_IPV6].mask_range.u32
			+ rule->field[DST3_FIELD_IPV6].mask_range.u32
			+ rule->field[DST4_FIELD_IPV6].mask_range.u32);

	printf("%hu : %hu %hu : %hu 0x%hhx/0x%hhx ",
		rule->field[SRCP_FIELD_IPV6].value.u16,
		rule->field[SRCP_FIELD_IPV6].mask_range.u16,
		rule->field[DSTP_FIELD_IPV6].value.u16,
		rule->field[DSTP_FIELD_IPV6].mask_range.u16,
		rule->field[PROTO_FIELD_IPV6].value.u8,
		rule->field[PROTO_FIELD_IPV6].mask_range.u8);
	if (extra)
		printf("0x%x-0x%x-0x%x ",
			rule->data.category_mask,
			rule->data.priority,
			rule->data.userdata);
}

/* Bypass comment and empty lines */
static inline int
is_bypass_line(char *buff)
{
	int i = 0;

	/* comment line */
	if (buff[0] == COMMENT_LEAD_CHAR)
		return 1;
	/* empty line */
	while (buff[i] != '\0') {
		if (!isspace(buff[i]))
			return 0;
		i++;
	}
	return 1;
}

#ifdef L3FWDACL_DEBUG
static inline void
dump_acl4_rule(struct rte_mbuf *m, uint32_t sig)
{
	uint32_t offset = sig & ~ACL_DENY_SIGNATURE;
	unsigned char a, b, c, d;
	struct ipv4_hdr *ipv4_hdr = rte_pktmbuf_mtod_offset(m,
					    struct ipv4_hdr *,
					    sizeof(struct ether_hdr));

	uint32_t_to_char(rte_bswap32(ipv4_hdr->src_addr), &a, &b, &c, &d);
	printf("Packet Src:%hhu.%hhu.%hhu.%hhu ", a, b, c, d);
	uint32_t_to_char(rte_bswap32(ipv4_hdr->dst_addr), &a, &b, &c, &d);
	printf("Dst:%hhu.%hhu.%hhu.%hhu ", a, b, c, d);

	printf("Src port:%hu,Dst port:%hu ",
			rte_bswap16(*(uint16_t *)(ipv4_hdr + 1)),
			rte_bswap16(*((uint16_t *)(ipv4_hdr + 1) + 1)));
	printf("hit ACL %d - ", offset);

	print_one_ipv4_rule(acl_config.rule_ipv4 + offset, 1);

	printf("\n\n");
}

static inline void
dump_acl6_rule(struct rte_mbuf *m, uint32_t sig)
{
	unsigned i;
	uint32_t offset = sig & ~ACL_DENY_SIGNATURE;
	struct ipv6_hdr *ipv6_hdr = rte_pktmbuf_mtod_offset(m,
					    struct ipv6_hdr *,
					    sizeof(struct ether_hdr));

	printf("Packet Src");
	for (i = 0; i < RTE_DIM(ipv6_hdr->src_addr); i += sizeof(uint16_t))
		printf(":%.2x%.2x",
			ipv6_hdr->src_addr[i], ipv6_hdr->src_addr[i + 1]);

	printf("\nDst");
	for (i = 0; i < RTE_DIM(ipv6_hdr->dst_addr); i += sizeof(uint16_t))
		printf(":%.2x%.2x",
			ipv6_hdr->dst_addr[i], ipv6_hdr->dst_addr[i + 1]);

	printf("\nSrc port:%hu,Dst port:%hu ",
			rte_bswap16(*(uint16_t *)(ipv6_hdr + 1)),
			rte_bswap16(*((uint16_t *)(ipv6_hdr + 1) + 1)));
	printf("hit ACL %d - ", offset);

	print_one_ipv6_rule(acl_config.rule_ipv6 + offset, 1);

	printf("\n\n");
}
#endif /* L3FWDACL_DEBUG */

static inline void
dump_ipv4_rules(struct acl4_rule *rule, int num, int extra)
{
	int i;

	for (i = 0; i < num; i++, rule++) {
		printf("\t%d:", i + 1);
		print_one_ipv4_rule(rule, extra);
		printf("\n");
	}
}

static inline void
dump_ipv6_rules(struct acl6_rule *rule, int num, int extra)
{
	int i;

	for (i = 0; i < num; i++, rule++) {
		printf("\t%d:", i + 1);
		print_one_ipv6_rule(rule, extra);
		printf("\n");
	}
}

#endif /* __L3FWD_ACL_H__ */
