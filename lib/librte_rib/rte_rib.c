/*
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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <x86intrin.h>
#include <sys/queue.h>

#include <rte_eal.h>
#include <rte_eal_memconfig.h>
#include <rte_common.h>
#include <rte_tailq.h>
#include <rte_errno.h>
#include <rte_rwlock.h>
#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_mempool.h>
#include <rte_malloc.h>
#include <rte_log.h>

#include <rte_rib.h>
#include <rte_dir24_8.h>

TAILQ_HEAD(rte_rib_list, rte_tailq_entry);
static struct rte_tailq_elem rte_rib_tailq = {
	.name = "RTE_RIB",
};
EAL_REGISTER_TAILQ(rte_rib_tailq)

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

static struct rte_rib_v4_node *
new_node_malloc(struct rte_rib *rib)
{
	return malloc(rib->node_sz);
}

static void
free_node_malloc(__rte_unused struct rte_rib *rib, struct rte_rib_v4_node *ent)
{
	free(ent);
}

static struct rte_rib_v4_node *
new_node_mempool(struct rte_rib *rib)
{
	struct rte_rib_v4_node *ent;
	int ret;

	ret = rte_mempool_get(rib->node_pool, (void *)&ent);
	if (ret != 0)
		return NULL;
	return ent;
}

static void
free_node_mempool(struct rte_rib *rib, struct rte_rib_v4_node *ent)
{
	rte_mempool_put(rib->node_pool, ent);
}

struct rte_rib_v4_node *
rte_rib_v4_lookup(struct rte_rib *rib, uint32_t key)
{
	struct rte_rib_v4_node *cur = rib->trie;
	struct rte_rib_v4_node *prev = NULL;

	while ((cur != NULL) && (((cur->key ^ key) &
		(uint32_t)(UINT64_MAX << (32 - cur->mask_len))) != 0)) {
		if (cur->flag & VALID_NODE)
			prev = cur;
		cur = GET_NXT_NODE(cur, key);
	}
	return prev;
}

struct rte_rib_v4_node *
rte_rib_v4_lookup_parent(struct rte_rib_v4_node *ent)
{
	struct rte_rib_v4_node *tmp;

	if (ent == NULL)
		return NULL;
	tmp = ent->parent;
	while ((tmp != NULL) && (tmp->flag & VALID_NODE) != VALID_NODE)
		tmp = tmp->parent;
	return tmp;
}

struct rte_rib_v4_node *
rte_rib_v4_lookup_exact(struct rte_rib *rib, uint32_t key, uint8_t mask_len)
{
	struct rte_rib_v4_node *cur = rib->trie;

	key &= (uint32_t)(UINT64_MAX << (32 - mask_len));
	while (cur != NULL) {
		if ((cur->key == key) && (cur->mask_len == mask_len) &&
				(cur->flag & VALID_NODE))
			return cur;
		if ((cur->mask_len > mask_len) ||
				(((uint64_t)key >> (32 - cur->mask_len)) !=
				((uint64_t)cur->key >> (32 - cur->mask_len))))
			break;
		cur = GET_NXT_NODE(cur, key);
	}
	return NULL;
}

struct rte_rib_v4_node *
rte_rib_v4_get_next_child(struct rte_rib *rib, uint32_t key,
	uint8_t mask_len, struct rte_rib_v4_node *cur, int flag)
{
	struct rte_rib_v4_node *tmp, *prev = NULL;

	if (cur == NULL) {
		tmp = rib->trie;
		while ((tmp) && (tmp->mask_len < mask_len))
			tmp = GET_NXT_NODE(tmp, key);
	} else {
		tmp = cur;
		while ((tmp->parent != NULL) && (IS_RIGHT_NODE(tmp) ||
				(tmp->parent->right == NULL))) {
			tmp = tmp->parent;
			if ((tmp->flag & VALID_NODE) &&
				(IS_COVERED(tmp->key, tmp->mask_len, key, mask_len)))
				return tmp;
		}
		tmp = (tmp->parent) ? tmp->parent->right : NULL;
	}
	while (tmp) {
		if ((tmp->flag & VALID_NODE) &&
			(IS_COVERED(tmp->key, tmp->mask_len, key, mask_len))) {
			prev = tmp;
			if (flag == GET_NXT_COVER)
				return prev;
		}
		tmp = (tmp->left) ? tmp->left : tmp->right;
	}
	return prev;
}

void
rte_rib_v4_remove(struct rte_rib *rib, uint32_t key, uint8_t mask_len)
{
	struct rte_rib_v4_node *cur, *prev, *child;

	cur = rte_rib_v4_lookup_exact(rib, key, mask_len);
	if (cur == NULL)
		return;

	cur->flag &= ~VALID_NODE;
	while ((cur->flag & VALID_NODE) != VALID_NODE) {
		if ((cur->left != NULL) && (cur->right != NULL)) {
			rib->free_node(rib, cur);
			return;
		}
		child = (cur->left == NULL) ? cur->right : cur->left;
		if (child != NULL)
			child->parent = cur->parent;
		if (cur->parent == NULL) {
			rib->trie = child;
			rib->free_node(rib, cur);
			return;
		}
		if (cur->parent->left == cur)
			cur->parent->left = child;
		else
			cur->parent->right = child;
		prev = cur;
		cur = cur->parent;
		rib->free_node(rib, prev);
	}
}

struct rte_rib_v4_node *
rte_rib_v4_insert(struct rte_rib *rib, uint32_t key, uint8_t mask_len)
{
	struct rte_rib_v4_node **tmp = &rib->trie;
	struct rte_rib_v4_node *prev = NULL;
	struct rte_rib_v4_node *new_node = NULL;
	struct rte_rib_v4_node *common_node = NULL;
	int i = 0;
	uint32_t common_prefix;
	uint8_t common_mask_len;

	if (mask_len > 32)
		return NULL;

	key &= (uint32_t)(UINT64_MAX << (32 - mask_len));
	new_node = rte_rib_v4_lookup_exact(rib, key, mask_len);
	if (new_node)
		return NULL;

	new_node = rib->alloc_node(rib);
	if (new_node == NULL) {
		RTE_LOG(ERR, LPM, "RIB node allocation failed\n");
		return NULL;
	}
	new_node->left = NULL;
	new_node->right = NULL;
	new_node->parent = NULL;
	new_node->key = key;
	new_node->mask_len = mask_len;
	new_node->flag = VALID_NODE;

	while (1) {
		if (*tmp == NULL) {
			*tmp = new_node;
			new_node->parent = prev;
		}
		if ((key == (*tmp)->key) && (mask_len == (*tmp)->mask_len)) {
			if (new_node != *tmp) {
				rib->free_node(rib, new_node);
				(*tmp)->flag |= VALID_NODE;
			}
			return *tmp;
		}
		i = (*tmp)->mask_len;
		if ((i >= mask_len) || (((uint64_t)key >> (32 - i)) !=
				((uint64_t)(*tmp)->key >> (32 - i))))
			break;
		prev = *tmp;
		tmp = (key & (1 << (31 - i))) ? &(*tmp)->right : &(*tmp)->left;
	}
	common_mask_len = MIN(mask_len, (*tmp)->mask_len);
	common_prefix = key ^ (*tmp)->key;
#ifdef __LZCNT__
	i = _lzcnt_u32(common_prefix);
#else
	for (i = 0; i <= common_mask_len; i++) {
		if ((common_prefix & (1 << (31 - i))) != 0)
			break;
	}
#endif
	common_mask_len = MIN(i, common_mask_len);
	common_prefix = key & (uint32_t)(UINT64_MAX << (32 - common_mask_len));
	if ((common_prefix == key) && (common_mask_len == mask_len)) {
		if ((*tmp)->key & (1 << (31 - mask_len)))
			new_node->right = *tmp;
		else
			new_node->left = *tmp;
		new_node->parent = (*tmp)->parent;
		(*tmp)->parent = new_node;
		*tmp = new_node;
	} else {
		common_node = rib->alloc_node(rib);
		if (common_node == NULL) {
			RTE_LOG(ERR, LPM, "RIB node allocation failed\n");
			rib->free_node(rib, new_node);
			return NULL;
		}
		common_node->key = common_prefix;
		common_node->mask_len = common_mask_len;
		common_node->flag = 0;
		common_node->parent = (*tmp)->parent;
		new_node->parent = common_node;
		(*tmp)->parent = common_node;
		if ((new_node->key & (1 << (31 - common_mask_len))) == 0) {
			common_node->left = new_node;
			common_node->right = *tmp;
		} else {
			common_node->left = *tmp;
			common_node->right = new_node;
		}
		*tmp = common_node;
	}
	return new_node;
}

struct rte_rib *
rte_rib_v4_create(const char *name, int socket_id, struct rte_rib_conf *conf)
{
	char mem_name[RTE_RIB_NAMESIZE];
	struct rte_rib *rib = NULL;
	struct rte_tailq_entry *te;
	struct rte_rib_list *rib_list;
	int dir24_8_nh_size;
	rte_rib_lookup_fn_t lookup_fn;

	/* Check user arguments. */
	if ((name == NULL) || (conf == NULL) || (socket_id < -1) ||
			(conf->type >= RTE_RIB_TYPE_MAX) ||
			(conf->alloc_type >= RTE_RIB_ALLOC_MAX) ||
			(conf->max_nodes == 0) ||
			(conf->node_sz < sizeof(struct rte_rib_v4_node))) {
		rte_errno = EINVAL;
		return NULL;
	}

	snprintf(mem_name, sizeof(mem_name), "RIB_%s", name);

	rib_list = RTE_TAILQ_CAST(rte_rib_tailq.head, rte_rib_list);

	rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);
	/* guarantee there's no existing */
	TAILQ_FOREACH(te, rib_list, next) {
		rib = (struct rte_rib *)te->data;
		if (strncmp(name, rib->name, RTE_RIB_NAMESIZE) == 0)
			break;
	}
	rib = NULL;
	if (te != NULL) {
		rte_errno = EEXIST;
		goto exit;
	}

	/* allocate tailq entry */
	te = rte_zmalloc("RIB_TAILQ_ENTRY", sizeof(*te), 0);
	if (te == NULL) {
		RTE_LOG(ERR, LPM, "Failed to allocate tailq entry\n");
		goto exit;
	}

	/* Allocate memory to store the LPM data structures. */
	rib = (struct rte_rib *)rte_zmalloc_socket(mem_name,
		sizeof(struct rte_rib),	RTE_CACHE_LINE_SIZE, socket_id);
	if (rib == NULL) {
		RTE_LOG(ERR, LPM, "RIB memory allocation failed\n");
		goto free_te;
	}
	snprintf(rib->name, sizeof(rib->name), "%s", name);
	rib->trie = NULL;
	rib->node_sz = conf->node_sz;

	snprintf(mem_name, sizeof(mem_name), "FIB_%s", name);

	if (conf->type <= RTE_RIB_DIR24_8_8B) {
		switch (conf->type) {
		case RTE_RIB_DIR24_8_1B:
			dir24_8_nh_size = RTE_DIR24_8_1B;
			lookup_fn = rte_dir24_8_lookup_1b;
			break;
		case RTE_RIB_DIR24_8_2B:
			dir24_8_nh_size = RTE_DIR24_8_2B;
			lookup_fn = rte_dir24_8_lookup_2b;
			break;
		case RTE_RIB_DIR24_8_4B:
			dir24_8_nh_size = RTE_DIR24_8_4B;
			lookup_fn = rte_dir24_8_lookup_4b;
			break;
		case RTE_RIB_DIR24_8_8B:
			dir24_8_nh_size = RTE_DIR24_8_8B;
			lookup_fn = rte_dir24_8_lookup_8b;
			break;
		case RTE_RIB_TYPE_MAX:
		default:
			RTE_LOG(ERR, LPM, "Bad RIB type\n");
			goto free_rib;
		}
		rib->fib = (void *)rte_zmalloc_socket(mem_name,
			sizeof(struct rte_dir24_8_tbl) +
			RTE_DIR24_8_TBL24_NUM_ENT * (1 << dir24_8_nh_size),
			RTE_CACHE_LINE_SIZE, socket_id);
		if (rib->fib == NULL) {
			RTE_LOG(ERR, LPM, "Failed to allocate FIB\n");
			goto free_rib;
		}
		snprintf(mem_name, sizeof(mem_name), "TBL8_%s", name);
		((struct rte_dir24_8_tbl *)rib->fib)->tbl8 =
			(void *)rte_zmalloc_socket(mem_name,
			RTE_DIR24_8_TBL8_GRP_NUM_ENT * (1 << dir24_8_nh_size) *
			conf->number_tbl8s, RTE_CACHE_LINE_SIZE, socket_id);
		if (((struct rte_dir24_8_tbl *)rib->fib)->tbl8 == NULL) {
			RTE_LOG(ERR, LPM, "Failed to allocate TBL8\n");
			rte_free(rib->fib);
			goto free_rib;
		}
		rib->modify_cb = rte_dir24_8_modify;
		rib->lookup_cb = lookup_fn;
		((struct rte_dir24_8_tbl *)rib->fib)->nh_sz =
			(enum rte_dir24_8_nh_sz)dir24_8_nh_size;
		((struct rte_dir24_8_tbl *)rib->fib)->number_tbl8s =
			conf->number_tbl8s;
	}

	rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);

	switch (conf->alloc_type) {
	case RTE_RIB_MALLOC:
		rib->alloc_node = new_node_malloc;
		rib->free_node = free_node_malloc;
		break;
	case RTE_RIB_MEMPOOL:
		snprintf(mem_name, sizeof(mem_name), "MP_%s", name);
		rib->node_pool = rte_mempool_create(mem_name, conf->max_nodes,
			conf->node_sz, 0, 0, NULL, NULL, NULL, NULL,
			socket_id, 0);

		if (rib->node_pool == NULL) {
			RTE_LOG(ERR, LPM, "Failed to allocate mempool\n");
			rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);
			goto free_fib;
		}
		rib->alloc_node = new_node_mempool;
		rib->free_node = free_node_mempool;
		break;
	case RTE_RIB_ALLOC_MAX:
	default:
		RTE_LOG(ERR, LPM, "Bad RIB alloc type\n");
		rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);
		goto free_fib;
	}

	rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);

	te->data = (void *)rib;
	TAILQ_INSERT_TAIL(rib_list, te, next);

	rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);

	return rib;

free_fib:
	rte_free(((struct rte_dir24_8_tbl *)rib->fib)->tbl8);
	rte_free(rib->fib);
free_rib:
	rte_free(rib);
	rib = NULL;
free_te:
	rte_free(te);
exit:
	rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);

	return NULL;
}
