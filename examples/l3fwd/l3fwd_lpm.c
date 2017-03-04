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

#include <rte_debug.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_mempool.h>
#include <rte_cycles.h>
#include <rte_mbuf.h>
#include <rte_ip.h>
#include <rte_tcp.h>
#include <rte_udp.h>
#include <rte_lpm.h>
#include <rte_lpm6.h>

#include "l3fwd.h"

enum {
	CB_FLD_DST_ADDR,
	CB_FLD_IF_OUT,
	CB_FLD_MAX
};

struct lpm_rule {
	union {
		uint32_t ip;
		union {
			uint32_t ip_32[4];
			uint8_t ip_8[16];
		};
	};
	uint8_t  depth;
	uint8_t  if_out;
};

#define IPV4_L3FWD_LPM_MAX_RULES         1024
#define IPV4_L3FWD_LPM_NUMBER_TBL8S (1 << 8)
#define IPV6_L3FWD_LPM_MAX_RULES         1024
#define IPV6_L3FWD_LPM_NUMBER_TBL8S (1 << 16)

struct rte_lpm *ipv4_l3fwd_lpm_lookup_struct[NB_SOCKETS];
struct rte_lpm6 *ipv6_l3fwd_lpm_lookup_struct[NB_SOCKETS];

#if defined(__SSE4_1__)
#include "l3fwd_lpm_sse.h"
#else
#include "l3fwd_lpm.h"
#endif

static int
lpm_parse_v6_addr(const char *in, const char **end, uint32_t v[IPV6_ADDR_U32],
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
lpm_parse_v6_net(const char *in, uint32_t *v, uint8_t *mask_len)
{
	int32_t rc;
	const char *mp;
	uint8_t m;
	uint32_t tmp[4];

	/* get address. */
	rc = lpm_parse_v6_addr(in, &mp, v, '/');
	if (rc != 0) {
		RTE_LOG(ERR, L3FWD, "parse_v6_addr failed %d\n", rc);
		return rc;
	}

	/* get mask. */
	GET_CB_FIELD(mp, m, 0, sizeof(tmp) * CHAR_BIT, 0);
	*mask_len = m;

	return 0;
}

static int
lpm_parse_v6_rule(char *str, struct lpm_rule *v)
{
	int i, rc;
	char *s, *sp, *in[CB_FLD_MAX];
	static const char *dlm = " \t\n";
	int dim = CB_FLD_MAX;
	s = str;

	for (i = 0; i != dim; i++, s = NULL) {
		in[i] = strtok_r(s, dlm, &sp);
		if (in[i] == NULL) {
			RTE_LOG(ERR, L3FWD,
				"\nparse_v6_rule strtok_r failed\n");
			return -EINVAL;
		}
	}

	rc = lpm_parse_v6_net(in[CB_FLD_DST_ADDR], v->ip_32, &v->depth);
	if (rc != 0) {
		RTE_LOG(ERR, L3FWD, "parse_v6_net failed\n");
		return rc;
	}

	GET_CB_FIELD(in[CB_FLD_IF_OUT], v->if_out, 0, UINT8_MAX, 0);

	return 0;
}

static int
lpm_parse_v4_net(const char *in, uint32_t *addr, uint8_t *mask_len)
{
	uint8_t a, b, c, d, m;

	GET_CB_FIELD(in, a, 0, UINT8_MAX, '.');
	GET_CB_FIELD(in, b, 0, UINT8_MAX, '.');
	GET_CB_FIELD(in, c, 0, UINT8_MAX, '.');
	GET_CB_FIELD(in, d, 0, UINT8_MAX, '/');
	GET_CB_FIELD(in, m, 0, sizeof(uint32_t) * CHAR_BIT, 0);

	addr[0] = IPv4(a, b, c, d);
	*mask_len = m;

	return 0;
}

static int
lpm_parse_v4_rule(char *str, struct lpm_rule *v)
{
	int i, rc;
	char *s, *sp, *in[CB_FLD_MAX];
	static const char *dlm = " \t\n";
	int dim = CB_FLD_MAX;
	s = str;

	for (i = 0; i != dim; i++, s = NULL) {
		in[i] = strtok_r(s, dlm, &sp);
		if (in[i] == NULL)
			return -EINVAL;
	}

	rc = lpm_parse_v4_net(in[CB_FLD_DST_ADDR], &v->ip, &v->depth);
	if (rc != 0) {
		RTE_LOG(ERR, L3FWD, "parse_v4_net failed %d\n", rc);
		return rc;
	}

	GET_CB_FIELD(in[CB_FLD_IF_OUT], v->if_out, 0, UINT8_MAX, 0);

	return 0;
}

static int
lpm_add_rules(const char *rule_path,
		struct lpm_rule **proute_base,
		unsigned int *proute_num,
		int (*parser)(char *, struct lpm_rule *))
{
	uint8_t *route_rules;
	struct lpm_rule *next;
	unsigned int route_num = 0;
	unsigned int route_cnt = 0;
	char buff[LINE_MAX];
	FILE *fh = fopen(rule_path, "rb");
	unsigned int i = 0, rule_size = sizeof(*next);

	if (fh == NULL)
		rte_exit(EXIT_FAILURE, "%s: Open %s failed\n", __func__,
			rule_path);

	while ((fgets(buff, LINE_MAX, fh) != NULL)) {
		if (buff[0] == LPM_LEAD_CHAR)
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
		if (s == LPM_LEAD_CHAR)
			next = (struct lpm_rule *)
				(route_rules + route_cnt * rule_size);

		/* Illegal line */
		else
			rte_exit(EXIT_FAILURE,
				"%s Line %u: should start with leading "
				"char %c\n",
				rule_path, i, LPM_LEAD_CHAR);

		if (parser(buff + 1, next) != 0)
			rte_exit(EXIT_FAILURE,
				"%s Line %u: parse rules error\n",
				rule_path, i);

		route_cnt++;
	}

	fclose(fh);

	*proute_base = (struct lpm_rule *)route_rules;
	*proute_num = route_cnt;

	return 0;
}

static int
check_lpm_config(void)
{
	if (parm_config.rule_ipv4_name == NULL) {
		RTE_LOG(ERR, L3FWD, "LPM IPv4 rule file not specified\n");
		return -1;
	} else if (parm_config.rule_ipv6_name == NULL) {
		RTE_LOG(ERR, L3FWD, "LPM IPv6 rule file not specified\n");
		return -1;
	}

	return 0;
}

/* main processing loop */
int
lpm_main_loop(__attribute__((unused)) void *dummy)
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
			l3fwd_lpm_send_packets(nb_rx, pkts_burst,
						portid, qconf);
#else
			l3fwd_lpm_no_opt_send_packets(nb_rx, pkts_burst,
							portid, qconf);
#endif /* __SSE_4_1__ */
		}
	}

	return 0;
}

void
setup_lpm(const int socketid)
{
	struct rte_lpm6_config config;
	struct rte_lpm_config config_ipv4;
	struct lpm_rule *route_base_v4;
	struct lpm_rule *route_base_v6;
	unsigned int route_num_v4 = 0, route_num_v6 = 0;
	unsigned i;
	int ret;
	char s[64];

	if (check_lpm_config() != 0)
		rte_exit(EXIT_FAILURE, "Failed to get valid LPM options\n");

	/* create the LPM table */
	config_ipv4.max_rules = IPV4_L3FWD_LPM_MAX_RULES;
	config_ipv4.number_tbl8s = IPV4_L3FWD_LPM_NUMBER_TBL8S;
	config_ipv4.flags = 0;
	snprintf(s, sizeof(s), "IPV4_L3FWD_LPM_%d", socketid);
	ipv4_l3fwd_lpm_lookup_struct[socketid] =
			rte_lpm_create(s, socketid, &config_ipv4);
	if (ipv4_l3fwd_lpm_lookup_struct[socketid] == NULL)
		rte_exit(EXIT_FAILURE,
			"Unable to create v4 LPM table on socket %d\n",
			socketid);

	/* Load rules from the input file */
	if (lpm_add_rules(parm_config.rule_ipv4_name,
		&route_base_v4, &route_num_v4,
		&lpm_parse_v4_rule) < 0)
		rte_exit(EXIT_FAILURE, "Failed to add lpm v4 rules\n");

	/* populate the LPM table */
	for (i = 0; i < route_num_v4; i++) {

		/* skip unused ports */
		if ((1 << route_base_v4[i].if_out &
			enabled_port_mask) == 0)
			continue;

		ret = rte_lpm_add(
			ipv4_l3fwd_lpm_lookup_struct[socketid],
			route_base_v4[i].ip,
			route_base_v4[i].depth,
			route_base_v4[i].if_out);

		if (ret < 0) {
			rte_exit(EXIT_FAILURE,
				"Unable to add entry %u to v4 LPM table on socket %d\n", i, socketid);
		}

		printf("LPM: Adding route 0x%08x / %d (%d)\n",
			(unsigned int)route_base_v4[i].ip,
			route_base_v4[i].depth,
			route_base_v4[i].if_out);
	}

	/* create the LPM6 table */
	snprintf(s, sizeof(s), "IPV6_L3FWD_LPM_%d", socketid);

	config.max_rules = IPV6_L3FWD_LPM_MAX_RULES;
	config.number_tbl8s = IPV6_L3FWD_LPM_NUMBER_TBL8S;
	config.flags = 0;
	ipv6_l3fwd_lpm_lookup_struct[socketid] =
			rte_lpm6_create(s, socketid, &config);
	if (ipv6_l3fwd_lpm_lookup_struct[socketid] == NULL)
		rte_exit(EXIT_FAILURE,
			"Unable to create v6 LPM table on socket %d\n",
			socketid);

	/* Load rules from the input file */
	if (lpm_add_rules(parm_config.rule_ipv6_name,
		&route_base_v6, &route_num_v6,
		&lpm_parse_v6_rule) < 0)
		rte_exit(EXIT_FAILURE, "Failed to add lpm v6 rules\n");

	/* populate the LPM table */
	for (i = 0; i < route_num_v6; i++) {

		/* skip unused ports */
		if ((1 << route_base_v6[i].if_out &
			enabled_port_mask) == 0)
			continue;

		ret = rte_lpm6_add(
			ipv6_l3fwd_lpm_lookup_struct[socketid],
			route_base_v6[i].ip_8,
			route_base_v6[i].depth,
			route_base_v6[i].if_out);

		if (ret < 0) {
			rte_exit(EXIT_FAILURE,
				"Unable to add entry %u to the l3fwd LPM table on socket %d\n", i, socketid);
		}

		printf("LPM: Adding route %s / %d (%d)\n",
			"IPV6", route_base_v6[i].depth,
			route_base_v6[i].if_out);
	}
}

int
lpm_check_ptype(int portid)
{
	int i, ret;
	int ptype_l3_ipv4 = 0, ptype_l3_ipv6 = 0;
	uint32_t ptype_mask = RTE_PTYPE_L3_MASK;

	ret = rte_eth_dev_get_supported_ptypes(portid, ptype_mask, NULL, 0);
	if (ret <= 0)
		return 0;

	uint32_t ptypes[ret];

	ret = rte_eth_dev_get_supported_ptypes(portid, ptype_mask, ptypes, ret);
	for (i = 0; i < ret; ++i) {
		if (ptypes[i] & RTE_PTYPE_L3_IPV4)
			ptype_l3_ipv4 = 1;
		if (ptypes[i] & RTE_PTYPE_L3_IPV6)
			ptype_l3_ipv6 = 1;
	}

	if (ptype_l3_ipv4 == 0)
		printf("port %d cannot parse RTE_PTYPE_L3_IPV4\n", portid);

	if (ptype_l3_ipv6 == 0)
		printf("port %d cannot parse RTE_PTYPE_L3_IPV6\n", portid);

	if (ptype_l3_ipv4 && ptype_l3_ipv6)
		return 1;

	return 0;

}

static inline void
lpm_parse_ptype(struct rte_mbuf *m)
{
	struct ether_hdr *eth_hdr;
	uint32_t packet_type = RTE_PTYPE_UNKNOWN;
	uint16_t ether_type;

	eth_hdr = rte_pktmbuf_mtod(m, struct ether_hdr *);
	ether_type = eth_hdr->ether_type;
	if (ether_type == rte_cpu_to_be_16(ETHER_TYPE_IPv4))
		packet_type |= RTE_PTYPE_L3_IPV4_EXT_UNKNOWN;
	else if (ether_type == rte_cpu_to_be_16(ETHER_TYPE_IPv6))
		packet_type |= RTE_PTYPE_L3_IPV6_EXT_UNKNOWN;

	m->packet_type = packet_type;
}

uint16_t
lpm_cb_parse_ptype(uint8_t port __rte_unused, uint16_t queue __rte_unused,
		   struct rte_mbuf *pkts[], uint16_t nb_pkts,
		   uint16_t max_pkts __rte_unused,
		   void *user_param __rte_unused)
{
	unsigned i;

	for (i = 0; i < nb_pkts; ++i)
		lpm_parse_ptype(pkts[i]);

	return nb_pkts;
}

/* Return ipv4/ipv6 lpm fwd lookup struct. */
void *
lpm_get_ipv4_l3fwd_lookup_struct(const int socketid)
{
	return ipv4_l3fwd_lpm_lookup_struct[socketid];
}

void *
lpm_get_ipv6_l3fwd_lookup_struct(const int socketid)
{
	return ipv6_l3fwd_lpm_lookup_struct[socketid];
}
