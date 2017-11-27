/*
 *   BSD LICENSE
 *
 *   Copyright (C) Cavium, Inc. 2015.
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
 *     * Neither the name of Cavium, Inc nor the names of its
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

#ifndef _RTE_MEMCPY_ARM64_H_
#define _RTE_MEMCPY_ARM64_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

#include "generic/rte_memcpy.h"

#ifdef RTE_ARCH_ARM64_MEMCPY
#include <rte_common.h>
#include <rte_branch_prediction.h>

/*******************************************************************************
 * The memory copy performance differs on different AArch64 micro-architectures.
 * And the most recent glibc (e.g. 2.23 or later) can provide a better memcpy()
 * performance compared to old glibc versions. It's always suggested to use a
 * more recent glibc if possible, from which the entire system can get benefit.
 *
 * This implementation improves memory copy on some aarch64 micro-architectures,
 * when an old glibc (e.g. 2.19, 2.17...) is being used. It is disabled by
 * default and needs "RTE_ARCH_ARM64_MEMCPY" defined to activate. It's not
 * always providing better performance than memcpy() so users need to run unit
 * test "memcpy_perf_autotest" and customize parameters in customization section
 * below for best performance.
 *
 * Compiler version will also impact the rte_memcpy() performance. It's observed
 * on some platforms and with the same code, GCC 7.2.0 compiled binaries can
 * provide better performance than GCC 4.8.5 compiled binaries.
 ******************************************************************************/

/**************************************
 * Beginning of customization section
 **************************************/
#define ALIGNMENT_MASK 0x0F
#ifndef RTE_ARCH_ARM64_MEMCPY_STRICT_ALIGN
// Only src unalignment will be treaed as unaligned copy
#define IS_UNALIGNED_COPY(dst, src) ((uintptr_t)(dst) & ALIGNMENT_MASK)
#else
// Both dst and src unalignment will be treated as unaligned copy
#define IS_UNALIGNED_COPY(dst, src) \
		(((uintptr_t)(dst) | (uintptr_t)(src)) & ALIGNMENT_MASK)
#endif


// If copy size is larger than threshold, memcpy() will be used.
// Run "memcpy_perf_autotest" to determine the proper threshold.
#define ALIGNED_THRESHOLD       ((size_t)(0xffffffff))
#define UNALIGNED_THRESHOLD     ((size_t)(0xffffffff))


/**************************************
 * End of customization section
 **************************************/
#ifdef RTE_TOOLCHAIN_GCC
#if (GCC_VERSION < 50400)
#warning "The GCC version is quite old, which may result in sub-optimal \
performance of the compiled code. It is suggested that at least GCC 5.4.0 \
be used."
#endif
#endif

static inline void __attribute__ ((__always_inline__))
rte_mov16(uint8_t *restrict dst, const uint8_t *restrict src)
{
	__int128 * restrict dst128 = (__int128 * restrict)dst;
	const __int128 * restrict src128 = (const __int128 * restrict)src;
	*dst128 = *src128;
}

static inline void __attribute__ ((__always_inline__))
rte_mov32(uint8_t *restrict dst, const uint8_t *restrict src)
{
	__int128 * restrict dst128 = (__int128 * restrict)dst;
	const __int128 * restrict src128 = (const __int128 * restrict)src;
	dst128[0] = src128[0];
	dst128[1] = src128[1];
}

static inline void __attribute__ ((__always_inline__))
rte_mov48(uint8_t *restrict dst, const uint8_t *restrict src)
{
	__int128 * restrict dst128 = (__int128 * restrict)dst;
	const __int128 * restrict src128 = (const __int128 * restrict)src;
	dst128[0] = src128[0];
	dst128[1] = src128[1];
	dst128[2] = src128[2];
}

static inline void __attribute__ ((__always_inline__))
rte_mov64(uint8_t *restrict dst, const uint8_t *restrict src)
{
	__int128 * restrict dst128 = (__int128 * restrict)dst;
	const __int128 * restrict src128 = (const __int128 * restrict)src;
	dst128[0] = src128[0];
	dst128[1] = src128[1];
	dst128[2] = src128[2];
	dst128[3] = src128[3];
}

static inline void __attribute__ ((__always_inline__))
rte_mov128(uint8_t *restrict dst, const uint8_t *restrict src)
{
	rte_mov64(dst, src);
	rte_mov64(dst + 64, src + 64);
}

static inline void __attribute__ ((__always_inline__))
rte_mov256(uint8_t *restrict dst, const uint8_t *restrict src)
{
	rte_mov128(dst, src);
	rte_mov128(dst + 128, src + 128);
}

static inline void __attribute__ ((__always_inline__))
rte_memcpy_lt16(uint8_t *restrict dst, const uint8_t *restrict src, size_t n)
{
	if (n & 0x08) {
		/* copy 8 ~ 15 bytes */
		*(uint64_t *)dst = *(const uint64_t *)src;
		*(uint64_t *)(dst - 8 + n) = *(const uint64_t *)(src - 8 + n);
	} else if (n & 0x04) {
		/* copy 4 ~ 7 bytes */
		*(uint32_t *)dst = *(const uint32_t *)src;
		*(uint32_t *)(dst - 4 + n) = *(const uint32_t *)(src - 4 + n);
	} else if (n & 0x02) {
		/* copy 2 ~ 3 bytes */
		*(uint16_t *)dst = *(const uint16_t *)src;
		*(uint16_t *)(dst - 2 + n) = *(const uint16_t *)(src - 2 + n);
	} else if (n & 0x01) {
		/* copy 1 byte */
		*dst = *src;
	}
}

static inline void __attribute__ ((__always_inline__))
rte_memcpy_ge16_lt64
(uint8_t *restrict dst, const uint8_t *restrict src, size_t n)
{
	if (n == 16) {
		rte_mov16(dst, src);
	} else if (n <= 32) {
		rte_mov16(dst, src);
		rte_mov16(dst - 16 + n, src - 16 + n);
	} else if (n <= 48) {
		rte_mov32(dst, src);
		rte_mov16(dst - 16 + n, src - 16 + n);
	} else {
		rte_mov48(dst, src);
		rte_mov16(dst - 16 + n, src - 16 + n);
	}
}

static inline void __attribute__ ((__always_inline__))
rte_memcpy_ge64(uint8_t *restrict dst, const uint8_t *restrict src, size_t n)
{
	do {
		rte_mov64(dst, src);
		src += 64;
		dst += 64;
		n -= 64;
	} while (likely(n >= 64));

	if (likely(n)) {
		if (n > 48)
			rte_mov64(dst - 64 + n, src - 64 + n);
		else if (n > 32)
			rte_mov48(dst - 48 + n, src - 48 + n);
		else if (n > 16)
			rte_mov32(dst - 32 + n, src - 32 + n);
		else
			rte_mov16(dst - 16 + n, src - 16 + n);
	}
}

static inline void *__attribute__ ((__always_inline__))
rte_memcpy(void *restrict dst, const void *restrict src, size_t n)
{
	if (n < 16) {
		rte_memcpy_lt16((uint8_t *)dst, (const uint8_t *)src, n);
		return dst;
	}
	if (n < 64) {
		rte_memcpy_ge16_lt64((uint8_t *)dst, (const uint8_t *)src, n);
		return dst;
	}
	__builtin_prefetch(src, 0, 0);
	__builtin_prefetch(dst, 1, 0);
	if (likely(
		  (!IS_UNALIGNED_COPY(dst, src) && n <= ALIGNED_THRESHOLD)
		   || (IS_UNALIGNED_COPY(dst, src) && n <= UNALIGNED_THRESHOLD)
		  )) {
		rte_memcpy_ge64((uint8_t *)dst, (const uint8_t *)src, n);
		return dst;
	} else
		return memcpy(dst, src, n);
}


#else
static inline void
rte_mov16(uint8_t *dst, const uint8_t *src)
{
	memcpy(dst, src, 16);
}

static inline void
rte_mov32(uint8_t *dst, const uint8_t *src)
{
	memcpy(dst, src, 32);
}

static inline void
rte_mov48(uint8_t *dst, const uint8_t *src)
{
	memcpy(dst, src, 48);
}

static inline void
rte_mov64(uint8_t *dst, const uint8_t *src)
{
	memcpy(dst, src, 64);
}

static inline void
rte_mov128(uint8_t *dst, const uint8_t *src)
{
	memcpy(dst, src, 128);
}

static inline void
rte_mov256(uint8_t *dst, const uint8_t *src)
{
	memcpy(dst, src, 256);
}

#define rte_memcpy(d, s, n)	memcpy((d), (s), (n))

#endif

#ifdef __cplusplus
}
#endif

#endif /* _RTE_MEMCPY_ARM_64_H_ */
