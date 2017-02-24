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
#include <rte_malloc.h>

#include "test.h"

#define MBUF_DATA_SIZE          2048
#define NB_MBUF                 64
#define CRC_VEC_LEN				32
#define CRC32_VEC_LEN1			1512
#define CRC32_VEC_LEN2			348
#define CRC16_VEC_LEN1			12
#define CRC16_VEC_LEN2			2

/* CRC test vector */
static const uint8_t crc_vec[CRC_VEC_LEN] = {
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
	'g', 'h', 'i', 'j', 'A', 'B', 'C', 'D',
	'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L',
};

/* 32-bit CRC test vector */
static const uint8_t crc32_vec1[12] = {
	0xBE, 0xD7, 0x23, 0x47, 0x6B, 0x8F,
	0xB3, 0x14, 0x5E, 0xFB, 0x35, 0x59,
};

/* 16-bit CRC test vector 1*/
static const uint8_t crc16_vec1[CRC16_VEC_LEN1] = {
	0x0D, 0x01, 0x01, 0x23, 0x45, 0x67,
	0x89, 0x01, 0x23, 0x45, 0x00, 0x01,
};

/* 16-bit CRC test vector 2*/
static const uint8_t crc16_vec2[CRC16_VEC_LEN2] = {
	0x03, 0x3f,
};
/** CRC results */
static const uint32_t crc32_vec_res = 0xb491aab4;
static const uint32_t crc32_vec1_res = 0xac54d294;
static const uint32_t crc32_vec2_res = 0xefaae02f;
static const uint32_t crc16_vec_res = 0x6bec;
static const uint16_t crc16_vec1_res = 0x8cdd;
static const uint16_t crc16_vec2_res = 0xec5b;

static int
crc_calc(struct rte_mempool *mp,
	const uint8_t *vec,
	uint32_t vec_len,
	uint32_t crc_r,
	enum rte_net_crc_type type)
{
	struct rte_net_crc_params p;
	uint8_t *data;

	/* alloc first mbuf from the mempool */
	p.mbuf = rte_pktmbuf_alloc(mp);
	if (p.mbuf == NULL) {
		printf("rte_pktmbuf_alloc() failed!\n");
		return -1;
	}

	/* copy parameters */
	p.type = type;
	p.data_offset = 0;
	p.data_len = vec_len;

	/* append data length to an mbuf */
	data = (uint8_t *)rte_pktmbuf_append(p.mbuf, p.data_len);

	/* copy ref_vector */
	rte_memcpy(data, vec, p.data_len);

	/* dump mbuf data on console*/
	rte_pktmbuf_dump(stdout, p.mbuf, p.data_len);

	/* compute CRC */
	int ret = rte_net_crc_calc(&p);

	if (crc_r != (uint32_t) ret) {
		rte_pktmbuf_free(p.mbuf);
		return ret;
	}

	/* free mbuf */
	rte_pktmbuf_free(p.mbuf);

	return 0;
}

static int
test_crc(void) {
	struct rte_mempool *pktmbuf_pool = NULL;
	uint32_t i;
	enum rte_net_crc_type type;
	uint8_t *test_data;
	int ret = 0;

	/* init crc */
	if (rte_net_crc_init(RTE_NET_CRC_SCALAR) == -1) {
		printf("test_crc: rte_net_crc_init() failed!\n");
		return -1;
	}

	/* create pktmbuf pool */
	pktmbuf_pool = rte_pktmbuf_pool_create("pktmbuf_pool",
			NB_MBUF, 32, 0, MBUF_DATA_SIZE, SOCKET_ID_ANY);

	if (pktmbuf_pool == NULL) {
		printf("test_crc: cannot allocate mbuf pool!\n");
		return -1;
	}

	/* 32-bit ethernet CRC: Test 1 */
	type = RTE_NET_CRC32_ETH;

	ret = crc_calc(pktmbuf_pool,
		crc_vec,
		CRC_VEC_LEN,
		crc32_vec_res,
		type);
	if (ret) {
		printf("test_crc(32-bit): test1 failed!\n");
		return -1;
	}

	/* 32-bit ethernet CRC: Test 2 */
	test_data = rte_zmalloc(NULL, CRC32_VEC_LEN1, 0);

	for (i = 0; i < CRC32_VEC_LEN1; i += 12)
	rte_memcpy(&test_data[i], crc32_vec1, 12);

	ret = crc_calc(pktmbuf_pool,
		test_data,
		CRC32_VEC_LEN1,
		crc32_vec1_res,
		type);
	if (ret) {
		printf("test_crc(32-bit): test2 failed!\n");
		return -1;
	}

	/* 32-bit ethernet CRC: Test 3 */
	test_data = rte_zmalloc(NULL, CRC32_VEC_LEN2, 0);

	for (i = 0; i < CRC32_VEC_LEN2; i += 12)
	rte_memcpy(&test_data[i], crc32_vec1, 12);

	ret = crc_calc(pktmbuf_pool,
		test_data,
		CRC32_VEC_LEN2,
		crc32_vec2_res,
		type);
	if (ret) {
		printf("test_crc(32-bit): test3 failed!\n");
		return -1;
	}

	/* 16-bit CCITT CRC:  Test 4 */
	type = RTE_NET_CRC16_CCITT;
	ret = crc_calc(pktmbuf_pool,
		crc_vec,
		CRC_VEC_LEN,
		crc16_vec_res,
		type);
	if (ret) {
		printf("test_crc (16-bit): test4 failed!\n");
		return -1;
	}

	/* 16-bit CCITT CRC:  Test 5 */
	ret = crc_calc(pktmbuf_pool,
		crc16_vec1,
		CRC16_VEC_LEN1,
		crc16_vec1_res,
		type);
	if (ret) {
		printf("test_crc (16-bit): test5 failed!\n");
		return -1;
	}

	/* 16-bit CCITT CRC:  Test 6 */
	ret = crc_calc(pktmbuf_pool,
		crc16_vec2,
		CRC16_VEC_LEN2,
		crc16_vec2_res,
		type);
	if (ret) {
		printf("test_crc (16-bit): test6 failed!\n");
		return -1;
	}

	return 0;
}

REGISTER_TEST_COMMAND(crc_autotest, test_crc);
