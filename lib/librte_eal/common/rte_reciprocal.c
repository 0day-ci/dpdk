/*-
 *   BSD LICENSE
 *
 *   Copyright(c) Hannes Frederic Sowa
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

#include <rte_reciprocal.h>

/* find largest set bit.
 * portable and slow but does not matter for this usage.
 */
static inline int
fls_u32(uint32_t x)
{
	int b;

	for (b = 31; b >= 0; --b) {
		if (x & (1u << b))
			return b + 1;
	}

	return 0;
}

struct rte_reciprocal_u32
rte_reciprocal_value_u32(uint32_t d)
{
	struct rte_reciprocal_u32 R;
	uint64_t m;
	int l;

	l = fls_u32(d - 1);
	m = ((1ULL << 32) * ((1ULL << l) - d));
	m /= d;

	++m;
	R.m = m;
	R.sh1 = RTE_MIN(l, 1);
	R.sh2 = RTE_MAX(l - 1, 0);

	return R;
}

/* Code taken from Hacker's Delight:
 * http://www.hackersdelight.org/HDcode/divlu.c.
 * License permits inclusion here per:
 * http://www.hackersdelight.org/permissions.htm
 */
static inline uint64_t
divide_128_div_64_to_64(uint64_t u1, uint64_t u0, uint64_t v, uint64_t *r)
{
	const uint64_t b = (1ULL << 32); /* Number base (16 bits). */
	uint64_t un1, un0,           /* Norm. dividend LSD's. */
			 vn1, vn0,           /* Norm. divisor digits. */
			 q1, q0,             /* Quotient digits. */
			 un64, un21, un10,   /* Dividend digit pairs. */
			 rhat;               /* A remainder. */
	int s;                       /* Shift amount for norm. */

    /* If overflow, set rem. to an impossible value. */
	if (u1 >= v) {
		if (r != NULL)
			*r = (uint64_t) -1;
		return (uint64_t) -1;
	}

	/* Count leading zeros. */
	s = __builtin_clzll(v);
	if (s > 0) {
		v = v << s;
		un64 = (u1 << s) | ((u0 >> (64 - s)) & (-s >> 31));
		un10 = u0 << s;
	} else {

		un64 = u1 | u0;
		un10 = u0;
	}

	vn1 = v >> 32;
	vn0 = v & 0xFFFFFFFF;

	un1 = un10 >> 32;
	un0 = un10 & 0xFFFFFFFF;

	q1 = un64/vn1;
	rhat = un64 - q1*vn1;
again1:
	if (q1 >= b || q1*vn0 > b*rhat + un1) {
		q1 = q1 - 1;
		rhat = rhat + vn1;
		if (rhat < b)
			goto again1;
	}

	un21 = un64*b + un1 - q1*v;

	q0 = un21/vn1;
	rhat = un21 - q0*vn1;
again2:
	if (q0 >= b || q0*vn0 > b*rhat + un0) {
		q0 = q0 - 1;
		rhat = rhat + vn1;
		if (rhat < b)
			goto again2;
	}

	if (r != NULL)
		*r = (un21*b + un0 - q0*v) >> s;
	return q1*b + q0;
}

struct rte_reciprocal_u64
rte_reciprocal_value_u64(uint64_t d)
{
	struct rte_reciprocal_u64 R;

	const uint32_t fld = 63 - __builtin_clzll(d);

	if ((d & (d - 1)) == 0) {
		R.m = 0;
		R.sh1 = (fld - 1) | 0x40;
	} else {
		uint64_t rem;
		uint64_t multiplier;
		uint8_t more;

		multiplier = divide_128_div_64_to_64(1ULL << fld, 0, d, &rem);
		multiplier += multiplier;

		const uint64_t twice_rem = rem + rem;
		if (twice_rem >= d || twice_rem < rem)
			multiplier += 1;
		more = fld;
		R.m = 1 + multiplier;
		R.sh1 = more | 0x40;
	}

	R.sh1 &= 0x3F;

	return R;
}
