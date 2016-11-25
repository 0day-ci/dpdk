/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2014 Jörgen Grahn. All rights reserved.
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
#include "rte_lpm_iterate.h"
#include "rte_lpm.h"

#include <arpa/inet.h>


/**
 * Iterate through the lpm, pulling out at most 'buflen' valid routes
 * (less means we've hit the end).  The cursor should be initialized
 * to { 0, 0 } before the first call.
 *
 * The routes are partially sorted, by prefix length.  Undefined
 * results if the lpm is modified in parallel with or inbetween calls,
 * although the iteration will still terminate properly.
 */
unsigned
rte_lpm_iterate(struct rte_lpm_route* const buf, unsigned buflen,
		const struct rte_lpm* lpm,
		struct rte_lpm_cursor* const cursor)
{
	struct rte_lpm_route* p = buf;
	struct rte_lpm_route* const end = p + buflen;

	const struct rte_lpm_rule_info* const rinfo = lpm->rule_info;
	const struct rte_lpm_rule* const rtbl = lpm->rules_tbl;

	unsigned d = cursor->d;
	unsigned n = cursor->n;

	while(p!=end) {
		if(d==32) break;
		if(n>=rinfo[d].used_rules) {
			d++;
			n = 0;
			continue;
		}
		const struct rte_lpm_rule rule = rtbl[rinfo[d].first_rule + n];
		p->addr.s_addr = htonl(rule.ip);
		p->plen = d+1;
		p->nh = rule.next_hop;
		p++;
		n++;
	}

	cursor->d = d;
	cursor->n = n;

	return p - buf;
}
