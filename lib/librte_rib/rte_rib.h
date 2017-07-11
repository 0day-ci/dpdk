/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Vladimir Medvedkin <medvedkinv@gmail.com>
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

#ifndef _RTE_RIB_H_
#define _RTE_RIB_H_

/**
 * @file
 * Compressed trie implementation for Longest Prefix Match
 */

/** @internal Macro to enable/disable run-time checks. */
#if defined(RTE_LIBRTE_DIR24_8_DEBUG)
#define RTE_RIB_RETURN_IF_TRUE(cond, retval) do {	\
	if (cond)					\
		return retval;				\
} while (0)
#else
#define RTE_RIB_RETURN_IF_TRUE(cond, retval)
#endif

#define VALID_NODE	1
#define GET_NXT_ALL	0
#define GET_NXT_COVER	1

/** Max number of characters in RIB name. */
#define RTE_RIB_NAMESIZE	64

/** Maximum depth value possible for IPv4 RIB. */
#define RTE_RIB_V4_MAXDEPTH	32

/**
 * Macro to check if prefix1 {key1/mask_len1}
 * is covered by prefix2 {key2/mask_len2}
 */
#define IS_COVERED(key1, mask_len1, key2, mask_len2)			     \
	((((key1 ^ key2) & (uint32_t)(UINT64_MAX << (32 - mask_len2))) == 0) \
		&& (mask_len1 > mask_len2))

/** @internal Macro to get next node in tree*/
#define GET_NXT_NODE(node, key)						 \
	((key & (1 << (31 - node->mask_len))) ? node->right : node->left)
/** @internal Macro to check if node is right child*/
#define IS_RIGHT_NODE(node)	(node->parent->right == node)


struct rte_rib_v4_node {
	struct rte_rib_v4_node *left;
	struct rte_rib_v4_node *right;
	struct rte_rib_v4_node *parent;
	uint32_t	key;
	uint8_t		mask_len;
	uint8_t		flag;
	uint64_t	ext[0];
};

/** simple extension to keep only single next_hop */
struct rte_rib_simple_ext {
	uint64_t next_hop;
};

struct rte_rib;

enum rte_rib_op {
	RTE_RIB_OP_ADD = 0,
	RTE_RIB_OP_DEL,
	RTE_RIB_OP_MODIFY
};

/** Type of FIB struct*/
enum rte_rib_type {
	RTE_RIB_DIR24_8_1B,
	RTE_RIB_DIR24_8_2B,
	RTE_RIB_DIR24_8_4B,
	RTE_RIB_DIR24_8_8B,
	RTE_RIB_TYPE_MAX
};

/** RIB nodes allocation type */
enum rte_rib_alloc_type {
	RTE_RIB_MALLOC,
	RTE_RIB_MEMPOOL,
	RTE_RIB_ALLOC_MAX
};

typedef int (*rte_rib_modify_fn_t)(struct rte_rib *rib, uint32_t key,
	uint8_t mask_len, uint64_t next_hop, enum rte_rib_op rib_op);
typedef int (*rte_rib_lookup_fn_t)(void *fib, uint32_t key, uint64_t *next_hop);
typedef struct rte_rib_v4_node *(*rte_rib_alloc_node_fn_t)(struct rte_rib *rib);
typedef void (*rte_rib_free_node_fn_t)(struct rte_rib *rib,
	struct rte_rib_v4_node *node);

struct rte_rib {
	char name[RTE_RIB_NAMESIZE];
	/*pointer to rib trie*/
	struct rte_rib_v4_node	*trie;
	/*pointer to dataplane struct*/
	void	*fib;
	/*prefix modification callback*/
	rte_rib_modify_fn_t	modify_cb;
	/* lookup fn callback*/
	rte_rib_lookup_fn_t	lookup_cb;
	/*alloc trie element*/
	rte_rib_alloc_node_fn_t	alloc_node;
	/*free trie element*/
	rte_rib_free_node_fn_t	free_node;
	void	*node_pool;
	int	node_sz;
};

/** RIB configuration structure */
struct rte_rib_conf {
	enum rte_rib_type type;
	int	max_nodes;
	size_t	node_sz;
	int	number_tbl8s;
	enum rte_rib_alloc_type alloc_type;
};

/**
 * Lookup an IP into the RIB structure
 *
 * @param rib
 *  RIB object handle
 * @param key
 *  IP to be looked up in the RIB
 * @return
 *  pointer to struct rte_rib_v4_node on success,
 *  NULL otherwise
 */
struct rte_rib_v4_node *rte_rib_v4_lookup(struct rte_rib *rib, uint32_t key);

/**
 * Lookup less specific route into the RIB structure
 *
 * @param ent
 *  Pointer to struct rte_rib_v4_node that represents target route
 * @return
 *  pointer to struct rte_rib_v4_node that represents
 *  less specific route on success,
 *  NULL otherwise
 */
struct rte_rib_v4_node *rte_rib_v4_lookup_parent(struct rte_rib_v4_node *ent);

/**
 * Lookup prefix into the RIB structure
 *
 * @param rib
 *  RIB object handle
 * @param key
 *  net to be looked up in the RIB
 * @param mask_len
 *  prefix length
 * @return
 *  pointer to struct rte_rib_v4_node on success,
 *  NULL otherwise
 */
struct rte_rib_v4_node *rte_rib_v4_lookup_exact(struct rte_rib *rib,
	uint32_t key, uint8_t mask_len);

/**
 * Retrieve next more specific prefix from the RIB
 * that is covered by key/mask_len supernet
 *
 * @param rib
 *  RIB object handle
 * @param key
 *  net address of supernet prefix that covers returned more specific prefixes
 * @param mask_len
 *  supernet prefix length
 * @param cur
 *   pointer to the last returned prefix to get next prefix
 *   or
 *   NULL to get first more specific prefix
 * @param flag
 *  -GET_NXT_ALL
 *   get all prefixes from subtrie
 *  -GET_NXT_COVER
 *   get only first more specific prefix even if it have more specifics
 * @return
 *  pointer to the next more specific prefix
 *  or
 *  NULL if there is no prefixes left
 */
struct rte_rib_v4_node *rte_rib_v4_get_next_child(struct rte_rib *rib,
	uint32_t key, uint8_t mask_len, struct rte_rib_v4_node *cur, int flag);

/**
 * Remove prefix from the RIB
 *
 * @param rib
 *  RIB object handle
 * @param key
 *  net to be removed from the RIB
 * @param mask_len
 *  prefix length
 */
void rte_rib_v4_remove(struct rte_rib *rib, uint32_t key, uint8_t mask_len);

/**
 * Insert prefix into the RIB
 *
 * @param rib
 *  RIB object handle
 * @param key
 *  net to be inserted to the RIB
 * @param mask_len
 *  prefix length
 * @return
 *  pointer to new rte_rib_v4_node on success
 *  NULL otherwise
 */
struct rte_rib_v4_node *rte_rib_v4_insert(struct rte_rib *rib, uint32_t key,
	uint8_t mask_len);

/**
 * Create RIB
 *
 * @param name
 *  RIB name
 * @param socket_id
 *  NUMA socket ID for RIB table memory allocation
 * @param conf
 *  Structure containing the configuration
 * @return
 *  Handle to LPM object on success
 *  NULL otherwise with rte_errno set to an appropriate values.
 */
struct rte_rib *rte_rib_v4_create(const char *name, int socket_id,
	struct rte_rib_conf *conf);

#endif /* _RTE_RIB_H_ */

