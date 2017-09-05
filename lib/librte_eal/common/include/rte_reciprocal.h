/*
 * Reciprocal divide
 *
 * Used with permission from original authors
 *  Hannes Frederic Sowa and Daniel Borkmann
 *
 * This algorithm is based on the paper "Division by Invariant
 * Integers Using Multiplication" by Torbjörn Granlund and Peter
 * L. Montgomery.
 *
 * The assembler implementation from Agner Fog, which this code is
 * based on, can be found here:
 * http://www.agner.org/optimize/asmlib.zip
 *
 * This optimization for A/B is helpful if the divisor B is mostly
 * runtime invariant. The reciprocal of B is calculated in the
 * slow-path with reciprocal_value(). The fast-path can then just use
 * a much faster multiplication operation with a variable dividend A
 * to calculate the division A/B.
 */

#ifndef _RTE_RECIPROCAL_H_
#define _RTE_RECIPROCAL_H_

#include <rte_memory.h>

/**
 * Unsigned 32-bit divisor structure.
 */
struct rte_reciprocal_u32 {
	uint32_t m;
	uint8_t sh1, sh2;
} __rte_cache_aligned;

/**
 * Unsigned 64-bit divisor structure.
 */
struct rte_reciprocal_u64 {
	uint64_t m;
	uint8_t sh1;
} __rte_cache_aligned;

/**
 * Divide given unsigned 32-bit integer with pre calculated divisor.
 *
 * @param a
 *   The 32-bit dividend.
 * @param R
 *   The pointer to pre calculated divisor reciprocal structure.
 *
 * @return
 *   The result of the division
 */
static inline uint32_t
rte_reciprocal_divide_u32(uint32_t a, struct rte_reciprocal_u32 *R)
{
	uint32_t t = (((uint64_t)a * R->m) >> 32);

	return (t + ((a - t) >> R->sh1)) >> R->sh2;
}

static inline uint64_t
mullhi_u64(uint64_t x, uint64_t y)
{
#ifdef __SIZEOF_INT128__
	__uint128_t xl = x;
	__uint128_t rl = xl * y;

	return (rl >> 64);
#else
	uint64_t u0, u1, v0, v1, k, t;
	uint64_t w1, w2;
	uint64_t whi;

	u1 = x >> 32; u0 = x & 0xFFFFFFFF;
	v1 = y >> 32; v0 = y & 0xFFFFFFFF;

	t = u0*v0;
	k = t >> 32;

	t = u1*v0 + k;
	w1 = t & 0xFFFFFFFF;
	w2 = t >> 32;

	t = u0*v1 + w1;
	k = t >> 32;

	whi = u1*v1 + w2 + k;

	return whi;
#endif
}

/**
 * Divide given unsigned 64-bit integer with pre calculated divisor.
 *
 * @param a
 *   The 64-bit dividend.
 * @param R
 *   The pointer to pre calculated divisor reciprocal structure.
 *
 * @return
 *   The result of the division
 */
static inline uint64_t
rte_reciprocal_divide_u64(uint64_t a, struct rte_reciprocal_u64 *R)
{
	uint64_t q = mullhi_u64(R->m, a);
	uint64_t t = ((a - q) >> 1) + q;

	return t >> R->sh1;
}

/**
 * Generate pre calculated divisor structure.
 *
 * @param d
 *   The unsigned 32-bit divisor.
 *
 * @return
 *   Divisor structure.
 */
struct rte_reciprocal_u32
rte_reciprocal_value_u32(uint32_t d);

/**
 * Generate pre calculated divisor structure.
 *
 * @param d
 *   The unsigned 64-bit divisor.
 *
 * @return
 *   Divisor structure.
 */
struct rte_reciprocal_u64
rte_reciprocal_value_u64(uint64_t d);

#endif /* _RTE_RECIPROCAL_H_ */
