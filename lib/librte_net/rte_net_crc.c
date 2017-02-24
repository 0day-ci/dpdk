/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation.
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

#include <rte_net_crc.h>
#include <stddef.h>

/* Macros for printing using RTE_LOG */
#define RTE_LOGTYPE_CRC RTE_LOGTYPE_USER1

/** CRC polynomials */
#define CRC32_ETH_POLYNOMIAL 0x04c11db7UL
#define CRC16_CCITT_POLYNOMIAL 0x1021U

typedef int (*rte_net_crc_handler)(struct rte_net_crc_params *);

static int rte_crc16_ccitt_handler(struct rte_net_crc_params *p);
static int rte_crc32_eth_handler(struct rte_net_crc_params *p);
static int rte_crc_invalid_handler(struct rte_net_crc_params *p);

static rte_net_crc_handler *handlers;

static rte_net_crc_handler handlers_scalar[] = {
	[RTE_NET_CRC16_CCITT] = rte_crc16_ccitt_handler,
	[RTE_NET_CRC32_ETH] = rte_crc32_eth_handler,
	[RTE_NET_CRC_REQS] = rte_crc_invalid_handler,
};

int
rte_crc_invalid_handler(__rte_unused struct rte_net_crc_params *p)
{
	RTE_LOG(ERR, CRC, "CRC type not supported!\n");
	return -1;	/* Error */
}

#if defined(RTE_ARCH_X86_64) && defined(RTE_MACHINE_CPUFLAG_SSE4_2)

#include <cpuid.h>

/** PCLMULQDQ CRC computation context structure */
struct crc_pclmulqdq_ctx {
	__m128i rk1_rk2;
	__m128i rk5_rk6;
	__m128i rk7_rk8;
};

struct crc_pclmulqdq_ctx crc32_eth_pclmulqdq __rte_aligned(16);
struct crc_pclmulqdq_ctx crc16_ccitt_pclmulqdq __rte_aligned(16);
/**
 * @brief Performs one folding round
 *
 * Logically function operates as follows:
 *     DATA = READ_NEXT_16BYTES();
 *     F1 = LSB8(FOLD)
 *     F2 = MSB8(FOLD)
 *     T1 = CLMUL(F1, RK1)
 *     T2 = CLMUL(F2, RK2)
 *     FOLD = XOR(T1, T2, DATA)
 *
 * @param data_block 16 byte data block
 * @param precomp precomputed rk1 constanst
 * @param fold running 16 byte folded data
 *
 * @return New 16 byte folded data
 */
static inline __attribute__((always_inline)) __m128i
crcr32_folding_round(const __m128i data_block,
		const __m128i precomp,
		const __m128i fold)
{
	__m128i tmp0 = _mm_clmulepi64_si128(fold, precomp, 0x01);
	__m128i tmp1 = _mm_clmulepi64_si128(fold, precomp, 0x10);

	return _mm_xor_si128(tmp1, _mm_xor_si128(data_block, tmp0));
}

/**
 * @brief Performs reduction from 128 bits to 64 bits
 *
 * @param data128 128 bits data to be reduced
 * @param precomp rk5 and rk6 precomputed constants
 *
 * @return data reduced to 64 bits
 */

static inline __attribute__((always_inline)) __m128i
crcr32_reduce_128_to_64(__m128i data128,
	const __m128i precomp)
{
	__m128i tmp0, tmp1, tmp2;

	/* 64b fold */
	tmp0 = _mm_clmulepi64_si128(data128, precomp, 0x00);
	tmp1 = _mm_srli_si128(data128, 8);
	tmp0 = _mm_xor_si128(tmp0, tmp1);

	/* 32b fold */
	tmp2 = _mm_slli_si128(tmp0, 4);
	tmp1 = _mm_clmulepi64_si128(tmp2, precomp, 0x10);

	return _mm_xor_si128(tmp1, tmp0);
}

/**
 * @brief Performs Barret's reduction from 64 bits to 32 bits
 *
 * @param data64 64 bits data to be reduced
 * @param precomp rk7 precomputed constant
 *
 * @return data reduced to 32 bits
 */

static inline __attribute__((always_inline)) uint32_t
crcr32_reduce_64_to_32(__m128i data64,
	const __m128i precomp)
{
	static const uint32_t mask1[4] __rte_aligned(16) = {
		0xffffffff, 0xffffffff, 0x00000000, 0x00000000
	};

	static const uint32_t mask2[4] __rte_aligned(16) = {
		0x00000000, 0xffffffff, 0xffffffff, 0xffffffff
	};
	__m128i tmp0, tmp1, tmp2;

	tmp0 = _mm_and_si128(data64, _mm_load_si128((const __m128i *)mask2));

	tmp1 = _mm_clmulepi64_si128(tmp0, precomp, 0x00);
	tmp1 = _mm_xor_si128(tmp1, tmp0);
	tmp1 = _mm_and_si128(tmp1, _mm_load_si128((const __m128i *)mask1));

	tmp2 = _mm_clmulepi64_si128(tmp1, precomp, 0x10);
	tmp2 = _mm_xor_si128(tmp2, tmp1);
	tmp2 = _mm_xor_si128(tmp2, tmp0);

	return _mm_extract_epi32(tmp2, 2);
}

/**
 * @brief Computes constant for CLMUL algorithm
 *
 * Result is: X^exp mod poly
 *
 * @param poly polynomial
 * @param exp exponent
 *
 * @return constant value
 */

static inline uint32_t
get_poly_constant(const uint32_t poly, const uint32_t exp)
{
	uint32_t i, res = poly;

	for (i = 32; i < exp; i++)
		if (res & 0x80000000)
			res = (res << 1) ^ poly;
		else
			res = (res << 1);

	return res;
}

/**
 * @brief Calculates quotient and reminder of X^64 / P(X)
 *
 * @param poly P(X)
 * @param q_ptr place to store quotient
 * @param r_ptr place to store reminder
 */
static inline void
div_poly(const uint64_t poly,
	uint64_t *q_ptr,
	uint64_t *r_ptr)
{
	uint64_t p = 0, q = 0, r = 0;
	int i;

	p = poly | 0x100000000ULL;

	r = p;
	r = r << 32;

	i = 32;
	do {
		uint64_t one_shl_n = 0;

		q = q << 1;
		if ((i + 32) < 64)
			one_shl_n = 1ULL << (32 + i);

		if (r & one_shl_n) {
			r ^= (p << i);
			q |= 1;
		}
		i--;
	} while (i >= 0);

	if (q_ptr != NULL)
		*q_ptr = q;

	if (r_ptr != NULL)
		*r_ptr = r;
}

/**
 * @brief Reflects selected group of bits in \a v
 *
 * @param v value to be reflected
 * @param n size of the bit field to be reflected
 *
 * @return bit reflected value
 */
static uint64_t
reflect(uint64_t v, const uint32_t n)
{
	uint32_t i;
	uint64_t r = 0;

	for (i = 0; i < n; i++) {
		if (i != 0) {
			r <<= 1;
			v >>= 1;
		}
		r |= (v & 1);
	}

	return r;
}

const uint8_t crc_xmm_shift_tab[48] __rte_aligned(16) = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/**
 * @brief Shifts left 128 bit register by specified number of bytes
 *
 * @param reg 128 bit value
 * @param num number of bytes to shift left reg by (0-16)
 *
 * @return reg << (num * 8)
 */

static inline __attribute__((always_inline)) __m128i
xmm_shift_left(__m128i reg, const unsigned int num)
{
	const __m128i *p = (const __m128i *)(crc_xmm_shift_tab + 16 - num);

	return _mm_shuffle_epi8(reg, _mm_loadu_si128(p));
}

/**
 * @brief Initializes CRC computation context structure for given polynomial
 *
 * @param pctx plcmulqdq CRC computation context structure to be initialized
 * @param poly CRC polynomial
 */
static inline __attribute__((always_inline)) int
crc32_eth_init_pclmulqdq(
	struct crc_pclmulqdq_ctx *pctx,
	const uint64_t poly)
{
	uint64_t k1, k2, k5, k6;
	uint64_t p = 0, q = 0;

	if (pctx == NULL)
		return -1;

	k1 = get_poly_constant(poly, 128 - 32);
	k2 = get_poly_constant(poly, 128 + 32);
	k5 = get_poly_constant(poly, 96);
	k6 = get_poly_constant(poly, 64);

	div_poly(poly, &q, NULL);
	q = q & 0xffffffff;			/** quotient X^64 / P(X) */
	p = poly | 0x100000000ULL;	/** P(X) */

	k1 = reflect(k1 << 32, 64) << 1;
	k2 = reflect(k2 << 32, 64) << 1;
	k5 = reflect(k5 << 32, 64) << 1;
	k6 = reflect(k6 << 32, 64) << 1;
	q = reflect(q, 33);
	p = reflect(p, 33);

	/** Save the params in context structure */
	pctx->rk1_rk2 = _mm_setr_epi64(_m_from_int64(k1), _m_from_int64(k2));
	pctx->rk5_rk6 = _mm_setr_epi64(_m_from_int64(k5), _m_from_int64(k6));
	pctx->rk7_rk8 = _mm_setr_epi64(_m_from_int64(q), _m_from_int64(p));

	return 0;
}

static inline __attribute__((always_inline)) uint32_t
crc32_eth_calc_pclmulqdq(
	const uint8_t *data,
	uint32_t data_len,
	uint32_t crc,
	const struct crc_pclmulqdq_ctx *params)
{
	__m128i temp, fold, k;
	uint32_t n;

	if (unlikely(data == NULL))
		return crc;

	if (unlikely(data_len == 0))
		return crc;

	if (unlikely(params == NULL))
		return crc;

	/* Get CRC init value */
	temp = _mm_insert_epi32(_mm_setzero_si128(), crc, 0);

	/**
	  * Folding all data into single 16 byte data block
	  * Assumes: fold holds first 16 bytes of data
	  */

	if (unlikely(data_len < 32)) {
		if (unlikely(data_len == 16)) {
			/* 16 bytes */
			fold = _mm_loadu_si128((const __m128i *)data);
			fold = _mm_xor_si128(fold, temp);
			goto reduction_128_64;
		}

		if (unlikely(data_len < 16)) {
			/* 0 to 15 bytes */
			uint8_t buffer[16] __rte_aligned(16);

			memset(buffer, 0, sizeof(buffer));
			memcpy(buffer, data, data_len);

			fold = _mm_load_si128((const __m128i *)buffer);
			fold = _mm_xor_si128(fold, temp);
			if (unlikely(data_len < 4)) {
				fold = xmm_shift_left(fold, 8 - data_len);
				goto barret_reduction;
			}
			fold = xmm_shift_left(fold, 16 - data_len);
			goto reduction_128_64;
		}
		/* 17 to 31 bytes */
		fold = _mm_loadu_si128((const __m128i *)data);
		fold = _mm_xor_si128(fold, temp);
		n = 16;
		k = params->rk1_rk2;
		goto partial_bytes;
	}

	/** At least 32 bytes in the buffer */
	/** Apply CRC initial value */
	fold = _mm_loadu_si128((const __m128i *)data);
	fold = _mm_xor_si128(fold, temp);

	/** Main folding loop - the last 16 bytes is processed separately */
	k = params->rk1_rk2;
	for (n = 16; (n + 16) <= data_len; n += 16) {
		temp = _mm_loadu_si128((const __m128i *)&data[n]);
		fold = crcr32_folding_round(temp, k, fold);
	}

partial_bytes:
	if (likely(n < data_len)) {

		const uint32_t mask3[4] __rte_aligned(16) = {
			0x80808080, 0x80808080, 0x80808080, 0x80808080
		};

		const uint8_t shf_table[32] __rte_aligned(16) = {
			0x00, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
			0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f,
			0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
			0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
		};

		__m128i last16, a, b;

		last16 = _mm_loadu_si128((const __m128i *)&data[data_len - 16]);

		temp = _mm_loadu_si128((const __m128i *)
			&shf_table[data_len & 15]);
		a = _mm_shuffle_epi8(fold, temp);

		temp = _mm_xor_si128(temp,
			_mm_load_si128((const __m128i *)mask3));
		b = _mm_shuffle_epi8(fold, temp);
		b = _mm_blendv_epi8(b, last16, temp);

		/* k = rk1 & rk2 */
		temp = _mm_clmulepi64_si128(a, k, 0x01);
		fold = _mm_clmulepi64_si128(a, k, 0x10);

		fold = _mm_xor_si128(fold, temp);
		fold = _mm_xor_si128(fold, b);
	}

	/** Reduction 128 -> 32 Assumes: fold holds 128bit folded data */
reduction_128_64:
	k = params->rk5_rk6;
	fold = crcr32_reduce_128_to_64(fold, k);

barret_reduction:
	k = params->rk7_rk8;
	n = crcr32_reduce_64_to_32(fold, k);

	return n;
}


static int
rte_net_crc_sse42_init(void)
{
	int status = 0;

	/** Initialize CRC functions */
	status = crc32_eth_init_pclmulqdq(&crc16_ccitt_pclmulqdq,
		CRC16_CCITT_POLYNOMIAL << 16);
	if (status == -1)
		return -1;

	status = crc32_eth_init_pclmulqdq(&crc32_eth_pclmulqdq,
		CRC32_ETH_POLYNOMIAL);
	if (status == -1)
		return -1;

	_mm_empty();

	return 0;
}

static inline int
rte_crc16_ccitt_sse42_handler(struct rte_net_crc_params *p)
{
	uint16_t ret;
	const uint8_t *data =
		rte_pktmbuf_mtod_offset(p->mbuf, uint8_t *, p->data_offset);

	ret = (uint16_t)~crc32_eth_calc_pclmulqdq(data,
		p->data_len,
		0xffff,
		&crc16_ccitt_pclmulqdq);

	return ret;
}

static inline int
rte_crc32_eth_sse42_handler(struct rte_net_crc_params *p)
{
	uint32_t ret;
	const uint8_t *data =
		rte_pktmbuf_mtod_offset(p->mbuf, uint8_t *, p->data_offset);

	ret = ~crc32_eth_calc_pclmulqdq(data,
		p->data_len,
		0xffffffffUL,
		&crc32_eth_pclmulqdq);

	return ret;
}

static rte_net_crc_handler handlers_sse42[] = {
	[RTE_NET_CRC16_CCITT] = rte_crc16_ccitt_sse42_handler,
	[RTE_NET_CRC32_ETH] = rte_crc32_eth_sse42_handler,
	[RTE_NET_CRC_REQS] = rte_crc_invalid_handler,
};

#endif

/** Local data */
static uint32_t crc32_eth_lut[256];
static uint32_t crc16_ccitt_lut[256];

/**
 * @brief Reflect the bits about the middle
 *
 * @param x value to be reflected
 *
 * @return reflected value
 */
static uint32_t
reflect_32bits(const uint32_t val)
{
	uint32_t i, res = 0;

	for (i = 0; i < 32; i++)
		if ((val & (1 << i)) != 0)
			res |= (uint32_t)(1 << (31 - i));

	return res;
}

static int
crc32_eth_init_lut(const uint32_t poly,
	uint32_t *lut)
{
	uint_fast32_t i, j;

	if (lut == NULL)
		return -1;

	for (i = 0; i < 256; i++) {
		uint_fast32_t crc = reflect_32bits(i);

		for (j = 0; j < 8; j++) {
			if (crc & 0x80000000L)
				crc = (crc << 1) ^ poly;
			else
				crc <<= 1;
		}
	lut[i] = reflect_32bits(crc);
	}

	return 0;
}

static inline __attribute__((always_inline)) uint32_t
crc32_eth_calc_lut(const uint8_t *data,
	uint32_t data_len,
	uint32_t crc,
	const uint32_t *lut)
{
	if (unlikely(data == NULL || lut == NULL))
		return crc;

	while (data_len--)
		crc = lut[(crc ^ *data++) & 0xffL] ^ (crc >> 8);

	return crc;
}

static int
rte_net_crc_scalar_init(void)
{
	int status = 0;

	/** 32-bit crc init */
	status = crc32_eth_init_lut(CRC32_ETH_POLYNOMIAL,
		crc32_eth_lut);
	if (status == -1)
		return -1;

	/** 16-bit CRC init */
	status = crc32_eth_init_lut(CRC16_CCITT_POLYNOMIAL << 16,
		crc16_ccitt_lut);
	if (status == -1)
		return -1;

	return 0;
}

static inline int
rte_crc16_ccitt_handler(struct rte_net_crc_params *p)
{
	uint16_t ret;
	const uint8_t *data =
		rte_pktmbuf_mtod_offset(p->mbuf, uint8_t *, p->data_offset);

	ret = (uint16_t)~crc32_eth_calc_lut(data,
		p->data_len,
		0xffff,
		crc16_ccitt_lut);

	return ret;
}

static inline int
rte_crc32_eth_handler(struct rte_net_crc_params *p)
{
	uint32_t ret;
	const uint8_t *data =
		rte_pktmbuf_mtod_offset(p->mbuf, uint8_t *, p->data_offset);

	ret = ~crc32_eth_calc_lut(data,
		p->data_len,
		0xffffffffUL,
		crc32_eth_lut);

	return ret;
}

int
rte_net_crc_init(enum rte_net_crc_mode m)
{
	int status;

	switch (m) {

	case RTE_NET_CRC_SSE42:
		if (rte_cpu_get_flag_enabled(RTE_CPUFLAG_SSE4_2)) {

			status = rte_net_crc_sse42_init();
			handlers = handlers_sse42;

		} else {
			RTE_LOG(ERR, CRC,
				"bad configuration(SSE4.2 not supported!)\n %s",
				__func__);
			status = -1;
		}
		return status;

	case RTE_NET_CRC_SCALAR:
	default:
		status = rte_net_crc_scalar_init();
		handlers = handlers_scalar;

		return status;
	}
}

int
rte_net_crc_calc(struct rte_net_crc_params *p)
{
	int ret;
	rte_net_crc_handler f_handle;

	f_handle = handlers[p->type];
	ret = f_handle(p);

	return ret;
}
