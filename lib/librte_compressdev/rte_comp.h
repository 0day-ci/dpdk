/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
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

#ifndef _RTE_COMP_H_
#define _RTE_COMP_H_

/**
 * @file rte_comp.h
 *
 * RTE definitions for Data Compression Service
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <rte_mempool.h>


/** Status of comp operation */
enum rte_comp_op_status {
	RTE_COMP_OP_STATUS_SUCCESS = 0,
	/**< Operation completed successfully */
	RTE_COMP_OP_STATUS_NOT_PROCESSED,
	/**< Operation has not yet been processed by the device */
	RTE_COMP_OP_STATUS_INVALID_SESSION,
	/**< Operation failed due to invalid session arguments */
	RTE_COMP_OP_STATUS_INVALID_ARGS,
	/**< Operation failed due to invalid arguments in request */
	RTE_COMP_OP_STATUS_ERROR,
	/**< Error handling operation */
	RTE_COMP_OP_STATUS_INVALID_STATE,
	/**< Operation is invoked in invalid state */
	RTE_COMP_OP_STATUS_OUT_OF_SPACE,
	/**< Output buffer ran out of space before operation completed */

	/* Note:
	 * QAT API has 19 error types.
	 * xISA-l has 5 inflate and 6 deflate errors.
	 * zlib has 6 errors
	 * Propose only include common subset in status - only values where appl
	 *  would have different behaviour.
	 * Add separate error field on op return which a PMD could populate
	 */
};


/** Compression Algorithms */
enum rte_comp_algorithm {
	RTE_COMP_NULL = 0,
	/**< No compression.
	 * Pass-through, data is copied unchanged from source buffer to
	 * destination buffer.
	 */
	RTE_COMP_DEFLATE,
	/**< DEFLATE compression algorithm
	 * https://tools.ietf.org/html/rfc1951
	 */
	RTE_COMP_LZS,
	/**< LZS compression algorithm
	 * https://tools.ietf.org/html/rfc2395
	 */
	RTE_COMP_ALGO_LIST_END
};

/**< Compression Level.
 * The number is interpreted by each PMD differently. However, lower numbers
 * give fastest compression, at the expense of compression ratio while
 * higher numbers may give better compression ratios but are likely slower.
 */
#define	RTE_COMP_LEVEL_PMD_DEFAULT	(-1)
/** Use PMD Default */
#define	RTE_COMP_LEVEL_NONE		(0)
/** Output uncompressed blocks if supported by the specified algorithm */
#define RTE_COMP_LEVEL_MIN		(1)
/** Use minimum compression level supported by the PMD */
#define RTE_COMP_LEVEL_MAX		(9)
/** Use maximum compression level supported by the PMD */

/** Compression checksum types */
enum rte_comp_checksum_type {
	RTE_COMP_NONE,
	/**< No checksum generated */
	RTE_COMP_CRC32,
	/**< Generates a CRC32 checksum, as used by gzip */
	RTE_COMP_ADLER32,
	/**< Generates an Adler-32 checksum, as used by zlib */
	RTE_COMP_CRC32_ADLER32,
	/**< Generates both Adler-32 and CRC32 checksums, concatenated.
	 * CRC32 is in the lower 32bits, Adler-32 in the upper 32 bits.
	 */
};

/*
 * enum rte_comp_hash_algo {
 *   RTE_COMP_HASH_NONE,
 *   RTE_COMP_HASH_SHA1,
 *   RTE_COMP_HASH_SHA256,
 * };
 * Need further input from cavium on this
 * xform will need a flag with above enum value
 * op will need to provide a virt/phys ptr to a data buffer of appropriate size.
 * And via capability PMD can say whether supported or not.
 */

/** Compression Huffman Type - used by DEFLATE algorithm */
enum rte_comp_huffman {
	RTE_COMP_DEFAULT,
	/**< PMD may choose which Huffman codes to use */
	RTE_COMP_FIXED,
	/**< Use Fixed Huffman codes */
	RTE_COMP_DYNAMIC,
	/**< Use Dynamic Huffman codes */
};


enum rte_comp_flush_flag {
	RTE_COMP_FLUSH_NONE,
	/**< Data is not flushed. Output may remain in the compressor and be
	 * processed during a following op. It may not be possible to decompress
	 * output until a later op with some other flush flag has been sent.
	 */
	RTE_COMP_FLUSH_SYNC,
	/**< All data should be flushed to output buffer. Output data can be
	 * decompressed. However state and history is not cleared, so future
	 * ops may use history from this op */
	RTE_COMP_FLUSH_FULL,
	/**< All data should be flushed to output buffer. Output data can be
	 * decompressed. State and history data is cleared, so future
	 * ops will be independent of ops processed before this. 
	 */
	RTE_COMP_FLUSH_FINAL
	/**< Same as RTE_COMP_FLUSH_FULL but also bfinal bit is set in last block
	 */
	 /* TODO:
	  * describe flag meanings for decompression.
	  * describe behavous in OUT_OF_SPACE case.
	  * At least the last flag is specific to deflate algo. Should this be
	  * called rte_comp_deflate_flush_flag? And should there be 
	  * comp_op_deflate_params in the op? */
};

/** Compression transform types */
enum rte_comp_xform_type {
	RTE_COMP_COMPRESS,
	/**< Compression service - compress */
	RTE_COMP_DECOMPRESS,
	/**< Compression service - decompress */
};

enum rte_comp_op_type {
    RTE_COMP_OP_STATELESS,
    /**< All data to be processed is submitted in the op, no state or history
     * from previous ops is used and none will be stored for future ops.
     * flush must be set to either FLUSH_FULL or FLUSH_FINAL 
     */
    RTE_COMP_OP_STATEFUL
    /**< There may be more data to be processed after this op, it's part of a
     * stream of data. State and history from previous ops can be used
     * and resulting state and history can be stored for future ops,
     * depending on flush_flag. 
     */
};


/** Parameters specific to the deflate algorithm */
struct rte_comp_deflate_params {
	enum rte_comp_huffman huffman;
	/**< Compression huffman encoding type */
};

/**
 * Session Setup Data common to all compress transforms.
 * Includes params common to stateless and stateful
 */
struct rte_comp_compress_common_params {
	enum rte_comp_algorithm algo;
	/**< Algorithm to use for compress operation */
	union {
		struct rte_comp_deflate_params deflate;
		/**< Parameters specific to the deflate algorithm */
	}; /**< Algorithm specific parameters */
	int level;
	/**< Compression level */
	uint16_t window_size;
	/**< depth of sliding window to be used */
	enum rte_comp_checksum_type chksum;
	/**< Type of checksum to generate on the uncompressed data */
};

/**
 * Session Setup Data for stateful compress transform.
 * Extra params for stateful transform
 */
struct rte_comp_compress_stateful_params {
	/*TODO : add extra params just needed for stateful, e.g. */
	/* history buffer size, window size, state, state buffers, etc...?*/
};
/* Session Setup Data for compress transform. */
struct rte_comp_compress_xform {
	struct rte_comp_compress_common_params cmn;
	struct rte_comp_compress_stateful_params stateful;
};

/**
 * Session Setup Data common to all decompress transforms.
 * Includes params common to stateless and stateful
 */
struct rte_comp_decompress_common_params {
	enum rte_comp_algorithm algo;
	/**< Algorithm to use for decompression */
	enum rte_comp_checksum_type chksum;
	/**< Type of checksum to generate on the decompressed data. */
	uint16_t window_size;
	/**< depth of sliding window which was used on compression */
};
/**
 *  Session Setup Data for decompress transform.
 * Extra params for stateful transform
 */
struct rte_comp_decompress_stateful_params {
	/*TODO : add extra params just needed for stateful, e.g.*/
	/* history buffer size, window size, state, state buffers, etc...?*/
};
/* Session Setup Data for decompress transform. */
struct rte_comp_decompress_xform {
	struct rte_comp_decompress_common_params cmn;
	struct rte_comp_decompress_stateful_params stateful;
};


/**
 * Compression transform structure.
 *
 * This is used to specify the compression transforms required.
 * Each transform structure can hold a single transform, the type field is
 * used to specify which transform is contained within the union.
 * There are no chain cases currently supported, just single xforms of
 *  - compress-only
 *  - decompress-only
 *
 */
struct rte_comp_xform {
	struct rte_comp_xform *next;
	/**< next xform in chain */
	enum rte_comp_xform_type type;
	/**< xform type */
	union {
		struct rte_comp_compress_xform compress;
		/**< xform for compress operation */
		struct rte_comp_decompress_xform decompress;
		/**< decompress xform */
	};
};


struct rte_comp_session;
/**
 * Compression Operation.
 *
 * This structure contains data relating to performing a compression
 * operation on the referenced mbuf data buffers.
 *
 * All compression operations are Out-of-place (OOP) operations,
 * as the size of the output data is different to the size of the input data.
 *
 * Comp operations are enqueued and dequeued in comp PMDs using the
 * rte_compressdev_enqueue_burst() / rte_compressdev_dequeue_burst() APIs
 */
struct rte_comp_op {

	enum rte_comp_op_type op_type;
	void * stream_private; 
	/* location where PMD maintains stream state
	 * only required if op_type is STATEFUL, else should be NULL
	 */
	struct rte_comp_session *session;
	/**< Handle for the initialised session context */
	struct rte_mempool *mempool;
	/**< mempool from which operation is allocated */
	phys_addr_t phys_addr;
	/**< physical address of this operation */
	struct rte_mbuf *m_src;
	/**< source mbuf
	 * The total size of the input buffer(s) can be retrieved using
	 * rte_pktmbuf_data_len(m_src)
	 */
	struct rte_mbuf *m_dst;
	/**< destination mbuf
	 * The total size of the output buffer(s) can be retrieved using
	 * rte_pktmbuf_data_len(m_dst)
	 */

	struct {
		uint32_t offset;
		/**< Starting point for compression or decompression,
		 * specified as number of bytes from start of packet in
		 * source buffer.
		 * Starting point for checksum generation in compress direction.
		 */
		uint32_t length;
		/**< The length, in bytes, of the data in source buffer
		 * to be compressed or decompressed.
		 * Also the length of the data over which the checksum
		 * should be generated in compress direction
		 */
	} src;
	struct {
		uint32_t offset;
		/**< Starting point for writing output data, specified as
		 * number of bytes from start of packet in dest
		 * buffer. Starting point for checksum generation in
		 * decompress direction.
		 */
	} dst;
	enum rte_comp_flush_flag flush_flag;
	/**< defines flush characteristics for the output data.
	 * Only applicable in compress direction
	 */
	uint64_t input_chksum;
	/**< An input checksum can be provided to generate a
	 * cumulative checksum across sequential blocks.
	 * Checksum type is as specified in xform chksum_type
	 */
	uint64_t output_chksum;
	/**< If a checksum is generated it will be written in here.
	 * Checksum type is as specified in xform chksum_type.
	 */
	uint32_t consumed;
	/**< The number of bytes from the source buffer
	 * which were compressed/decompressed.
	 */
	uint32_t produced;
	/**< The number of bytes written to the destination buffer
	 * which were compressed/decompressed.
	 */
	uint64_t debug_status;
	/**<
	 * Status of the operation is returned in the status param.
	 * This field allows the PMD to pass back extra
	 * pmd-specific debug information. Value is not defined on the API.
	 */
	uint8_t status;
	/**<
	 * operation status - use values from enum rte_comp_status.
	 * This is reset to
	 * RTE_COMP_OP_STATUS_NOT_PROCESSED on allocation from mempool and
	 * will be set to RTE_COMP_OP_STATUS_SUCCESS after operation
	 * is successfully processed by a PMD
	 */

	/*
	 * TODO - Are extra params needed on stateful op or are all in xform?
	 * rte_comp_op_common_params/_stateful_params?
	 */
};


/**
 * Reset the fields of an operation to their default values.
 *
 * @param	op	The operation to be reset.
 */
static inline void
__rte_comp_op_reset(struct rte_comp_op *op)
{
	struct rte_mempool *tmp_mp = op->mempool;
	phys_addr_t tmp_phys_addr = op->phys_addr;

	memset(op, 0, tmp_mp->elt_size);
	op->status = RTE_COMP_OP_STATUS_NOT_PROCESSED;
	op->phys_addr = tmp_phys_addr;
	op->mempool = tmp_mp;
}


/**
 * Attach a session to a compression operation
 *
 * @param	op	operation
 * @param	sess	session
 */
static inline int
__rte_comp_op_attach_comp_session(struct rte_comp_op *op,
		struct rte_comp_session *sess)
{
	op->session = sess;

	return 0;
}


/**
 * Private data structure belonging to an operation pool.
 */
struct rte_comp_op_pool_private {
	uint16_t user_size;
	/**< Size of private user data with each operation. */
};


/**
 * Returns the size of private user data allocated with each object in
 * the mempool
 *
 * @param	mempool	mempool for operations
 *
 * @return	user data size
 */
static inline uint16_t
__rte_comp_op_get_user_data_size(struct rte_mempool *mempool)
{
	struct rte_comp_op_pool_private *priv =
	    (struct rte_comp_op_pool_private *)rte_mempool_get_priv(mempool);

	return priv->user_size;
}


/**
 * Creates an operation pool
 *
 * @param	name		pool name
 * @param	nb_elts		number of elements in pool
 * @param	cache_size	Number of elements to cache on lcore, see
 *				*rte_mempool_create* for further details about
 *				cache size
 * @param	user_size	Size of private data to allocate for user with
 *				each operation
 * @param	socket_id	Socket to allocate memory on
 *
 * @return
 *  - On success pointer to mempool
 *  - On failure NULL
 */
extern struct rte_mempool *
rte_comp_op_pool_create(const char *name,
		unsigned int nb_elts, unsigned int cache_size,
		 uint16_t user_size, int socket_id);

/**
 * Bulk allocate raw element from mempool and return as comp operations
 *
 * @param	mempool		operation mempool.
 * @param	ops		Array to place allocated operations
 * @param	nb_ops		Number of operations to allocate
 *
 * @returns
 * - On success returns  number of ops allocated
 */
static inline int
__rte_comp_op_raw_bulk_alloc(struct rte_mempool *mempool,
		struct rte_comp_op **ops, uint16_t nb_ops)
{

	if (rte_mempool_get_bulk(mempool, (void **)ops, nb_ops) == 0)
		return nb_ops;

	return 0;
}

/**
 * Allocate an operation from a mempool with default parameters set
 *
 * @param	mempool	operation mempool
 *
 * @returns
 * - On success returns a valid rte_comp_op structure
 * - On failure returns NULL
 */
static inline struct rte_comp_op *
rte_comp_op_alloc(struct rte_mempool *mempool)
{
	struct rte_comp_op *op = NULL;
	int retval;

	retval = __rte_comp_op_raw_bulk_alloc(mempool, &op, 1);
	if (unlikely(retval != 1))
		return NULL;

	__rte_comp_op_reset(op);

	return op;
}


/**
 * Bulk allocate operations from a mempool with default parameters set
 *
 * @param	mempool	comp operation mempool
 * @param	ops	Array to place allocated operations
 * @param	nb_ops	Number of operations to allocate
 *
 * @returns
 * - nb_ops if the number of operations requested were allocated.
 * - 0 if the requested number of ops are not available.
 *   None are allocated in this case.
 */

static inline unsigned
rte_comp_op_bulk_alloc(struct rte_mempool *mempool,
		struct rte_comp_op **ops, uint16_t nb_ops)
{
	int i;

	if (unlikely(__rte_comp_op_raw_bulk_alloc(mempool, ops, nb_ops)
			!= nb_ops))
		return 0;

	for (i = 0; i < nb_ops; i++)
		__rte_comp_op_reset(ops[i]);

	return nb_ops;
}



/**
 * Returns a pointer to the private user data of an operation if
 * that operation has enough capacity for requested size.
 *
 * @param	op	operation.
 * @param	size	size of space requested in private data.
 *
 * @returns
 * - if sufficient space available returns pointer to start of user data
 * - if insufficient space returns NULL
 */
static inline void *
__rte_comp_op_get_user_data(struct rte_comp_op *op, uint32_t size)
{
	uint32_t user_size;

	if (likely(op->mempool != NULL)) {
		user_size = __rte_comp_op_get_user_data_size(op->mempool);

		if (likely(user_size >= size))
			return (void *)(op + 1);

	}

	return NULL;
}

/**
 * free operation structure
 * If operation has been allocate from a rte_mempool, then the operation will
 * be returned to the mempool.
 *
 * @param	op operation
 */
static inline void
rte_comp_op_free(struct rte_comp_op *op)
{
	if (op != NULL && op->mempool != NULL)
		rte_mempool_put(op->mempool, op);
}

/**
 * Attach a session to an operation
 *
 * @param	op	operation
 * @param	sess	session
 */
static inline int
rte_comp_op_attach_session(struct rte_comp_op *op,
		struct rte_comp_session *sess)
{
	op->session = sess;
	return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* _RTE_COMP_H_ */
