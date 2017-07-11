/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
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

#ifndef _RTE_DIR24_8_H_
#define _RTE_DIR24_8_H_

/**
 * @file
 * RTE Longest Prefix Match (LPM)
 */

#ifdef __cplusplus
extern "C" {
#endif

/** @internal Total number of tbl24 entries. */
#define RTE_DIR24_8_TBL24_NUM_ENT	(1 << 24)

/** Maximum depth value possible for IPv4 LPM. */
#define RTE_DIR24_8_MAX_DEPTH		32

/** @internal Number of entries in a tbl8 group. */
#define RTE_DIR24_8_TBL8_GRP_NUM_ENT	256

/** @internal bitmask with valid and valid_group fields set */
#define RTE_DIR24_8_VALID_EXT_ENT	0x03

/** Bitmask used to indicate successful lookup */
#define RTE_DIR24_8_LOOKUP_SUCCESS	0x01

#define RTE_DIR24_8_TBL24_MASK		0xffffff00

/** Size of nexthop (1 << nh_sz) bits */
enum rte_dir24_8_nh_sz {
	RTE_DIR24_8_1B,
	RTE_DIR24_8_2B,
	RTE_DIR24_8_4B,
	RTE_DIR24_8_8B
};

#define DIR24_8_TBL_IDX(a)	((a) >> (3 - fib->nh_sz))
#define DIR24_8_PSD_IDX(a)	((a) & ((1 << (3 - fib->nh_sz)) - 1))
#define DIR24_8_BITS_IN_NH	(8 * (1 << fib->nh_sz))

#define DIR24_8_TBL24_VAL(ip)	(ip >> 8)
#define DIR24_8_TBL8_VAL(res, ip)					\
	((res >> 2) * RTE_DIR24_8_TBL8_GRP_NUM_ENT + (uint8_t)ip)	\

#define DIR24_8_LOOKUP_MSK	((((uint64_t)1 << ((1 << (fib->nh_sz + 3)) - 1)) << 1) - 1)
#define RTE_DIR24_8_GET_TBL24(fib, ip)					\
	((fib->tbl24[DIR24_8_TBL_IDX(DIR24_8_TBL24_VAL(ip))] >>		\
	(DIR24_8_PSD_IDX(DIR24_8_TBL24_VAL(ip)) *			\
	DIR24_8_BITS_IN_NH)) & DIR24_8_LOOKUP_MSK)			\

#define RTE_DIR24_8_GET_TBL8(fib, res, ip)				\
	((fib->tbl8[DIR24_8_TBL_IDX(DIR24_8_TBL8_VAL(res, ip))] >>	\
	(DIR24_8_PSD_IDX(DIR24_8_TBL8_VAL(res, ip)) *			\
	DIR24_8_BITS_IN_NH)) & DIR24_8_LOOKUP_MSK)			\

#define RTE_DIR24_8_LOOKUP(type)					\
	const type *ptbl = (const type *)				\
		&((type *)fib->tbl24)[tbl24_index];			\
	type tbl_entry = *ptbl;						\
	if (unlikely((tbl_entry & RTE_DIR24_8_VALID_EXT_ENT) ==		\
			RTE_DIR24_8_VALID_EXT_ENT)) {			\
		uint32_t tbl8_index = (uint8_t)ip + ((tbl_entry >> 2) *	\
				RTE_DIR24_8_TBL8_GRP_NUM_ENT);		\
									\
		ptbl = (const type *)&((type *)fib->tbl8)[tbl8_index];	\
		tbl_entry = *ptbl;					\
	}								\
									\
	*next_hop = (uint64_t)tbl_entry >> 2				\
									\

struct rte_dir24_8_tbl {
	uint32_t	number_tbl8s;	/**< Total number of tbl8s. */
	uint32_t	cur_tbl8s;	/**< Current cumber of tbl8s. */
	enum rte_dir24_8_nh_sz	nh_sz;	/**< Size of nexthop entry */
	uint64_t	*tbl8;		/**< LPM tbl8 table. */
	uint64_t	tbl24[0] __rte_cache_aligned; /**< LPM tbl24 table. */
};

int rte_dir24_8_modify(struct rte_rib *rib, uint32_t key, uint8_t mask_len,
	uint64_t next_hop, enum rte_rib_op rib_op);
int rte_dir24_8_lookup_1b(void *fib_p, uint32_t ip, uint64_t *next_hop);
int rte_dir24_8_lookup_2b(void *fib_p, uint32_t ip, uint64_t *next_hop);
int rte_dir24_8_lookup_4b(void *fib_p, uint32_t ip, uint64_t *next_hop);
int rte_dir24_8_lookup_8b(void *fib_p, uint32_t ip, uint64_t *next_hop);
int rte_dir24_8_modify(struct rte_rib *rib, uint32_t key, uint8_t mask_len,
	uint64_t next_hop, enum rte_rib_op rib_op);

static inline int
rte_dir24_8_lookup(void *fib_p, uint32_t ip, uint64_t *next_hop)
{
	uint64_t res;
	struct rte_dir24_8_tbl *fib = (struct rte_dir24_8_tbl *)fib_p;

	/* DEBUG: Check user input arguments. */
	RTE_RIB_RETURN_IF_TRUE(((fib == NULL) || (ip == NULL) ||
		(next_hop == NULL)), -EINVAL);

	res = RTE_DIR24_8_GET_TBL24(fib, ip);
	if (unlikely((res & RTE_DIR24_8_VALID_EXT_ENT) ==
		RTE_DIR24_8_VALID_EXT_ENT)) {
		res = RTE_DIR24_8_GET_TBL8(fib, res, ip);
	}
	*next_hop = res >> 2;
	return (res & RTE_DIR24_8_LOOKUP_SUCCESS) ? 0 : -ENOENT;
}

#ifdef __cplusplus
}
#endif

#endif /* _RTE_DIR24_8_H_ */

