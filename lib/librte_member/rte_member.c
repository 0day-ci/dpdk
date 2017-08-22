/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
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

#include <string.h>

#include <rte_eal.h>
#include <rte_eal_memconfig.h>
#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_malloc.h>
#include <rte_errno.h>

#include "rte_member.h"
#include "rte_member_ht.h"
#include "rte_member_vbf.h"

TAILQ_HEAD(rte_member_list, rte_tailq_entry);
static struct rte_tailq_elem rte_member_tailq = {
	.name = "RTE_MEMBER",
};
EAL_REGISTER_TAILQ(rte_member_tailq)


void *
rte_member_find_existing(const char *name)
{
	struct rte_member_setsum *setsum;
	struct rte_tailq_entry *te;
	struct rte_member_list *member_list;

	member_list = RTE_TAILQ_CAST(rte_member_tailq.head, rte_member_list);

	rte_rwlock_read_lock(RTE_EAL_TAILQ_RWLOCK);
	TAILQ_FOREACH(te, member_list, next) {
		setsum = (struct rte_member_setsum *) te->data;
		if (strncmp(name, setsum->name, RTE_MEMBER_NAMESIZE) == 0)
			break;
	}
	rte_rwlock_read_unlock(RTE_EAL_TAILQ_RWLOCK);

	if (te == NULL) {
		rte_errno = ENOENT;
		return NULL;
	}
	return setsum;
}

void
rte_member_free(void *ss)
{
	struct rte_member_setsum *setsum;
	struct rte_member_list *member_list = NULL;
	struct rte_tailq_entry *te;

	if (ss == NULL)
		return;
	setsum = ss;
	member_list = RTE_TAILQ_CAST(rte_member_tailq.head, rte_member_list);
	rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);
	TAILQ_FOREACH(te, member_list, next) {
		if (te->data == ss)
			break;
	}
	if (te == NULL) {
		rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);
		return;
	}
	TAILQ_REMOVE(member_list, te, next);
	rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);

	switch (setsum->type) {
	case RTE_MEMBER_TYPE_HT:
		rte_member_free_ht(setsum);
		break;
	case RTE_MEMBER_TYPE_VBF:
		rte_member_free_vbf(setsum);
		break;
	default:
		break;
	}
	rte_free(setsum);
	rte_free(te);
}


void *
rte_member_create(const struct rte_member_parameters *params)
{
	struct rte_tailq_entry *te;
	struct rte_member_list *member_list = NULL;
	struct rte_member_setsum *setsum = NULL;
	int ret;

	if (params == NULL) {
		rte_errno = EINVAL;
		return NULL;
	}

	if ((params->key_len == 0)) {
		rte_errno = EINVAL;
		RTE_LOG(ERR, MEMBER,
			"Memship create with invalid parameters\n");
		return NULL;
	}

	member_list = RTE_TAILQ_CAST(rte_member_tailq.head, rte_member_list);

	rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);

	TAILQ_FOREACH(te, member_list, next) {
		setsum = (struct rte_member_setsum *) te->data;
		if (strncmp(params->name, setsum->name,
				RTE_MEMBER_NAMESIZE) == 0)
			break;
	}
	setsum = NULL;
	if (te != NULL) {
		rte_errno = EEXIST;
		te = NULL;
		goto error_unlock_exit;
	}
	te = rte_zmalloc("MEMBER_TAILQ_ENTRY", sizeof(*te), 0);
	if (te == NULL) {
		RTE_LOG(ERR, MEMBER, "tailq entry allocation failed\n");
		goto error_unlock_exit;
	}

	/* Create a new setsum structure */
	setsum = (struct rte_member_setsum *) rte_zmalloc_socket(params->name,
			sizeof(struct rte_member_setsum), RTE_CACHE_LINE_SIZE,
			params->socket_id);
	if (setsum == NULL) {
		RTE_LOG(ERR, MEMBER, "Create setsummary failed\n");
		goto error_unlock_exit;
	}
	setsum->type = params->type;
	setsum->socket_id = params->socket_id;
	setsum->key_len = params->key_len;
	setsum->num_set = params->num_set;
	setsum->name = params->name;
	setsum->prim_hash_seed = params->prim_hash_seed;
	setsum->sec_hash_seed = params->sec_hash_seed;

	switch (setsum->type) {
	case RTE_MEMBER_TYPE_HT:
		ret = rte_member_create_ht(setsum, params);
		break;
	case RTE_MEMBER_TYPE_VBF:
		ret = rte_member_create_vbf(setsum, params);
		break;
	default:
		goto error_unlock_exit;
	}
	if (ret < 0)
		goto error_unlock_exit;
	RTE_LOG(DEBUG, MEMBER, "Creating a setsummary table with mode %u\n",
			setsum->type);

	te->data = (void *)setsum;
	TAILQ_INSERT_TAIL(member_list, te, next);
	rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);
	return setsum;

error_unlock_exit:
	rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);
	rte_member_free(setsum);
	return NULL;
}


int
rte_member_add(const void *ss, const void *key, MEMBER_SET_TYPE set_id)
{
	const struct rte_member_setsum *setsum = ss;

	if (setsum == NULL || key == NULL)
		return -EINVAL;
	int ret = 0;
	switch (setsum->type) {
	case RTE_MEMBER_TYPE_HT:
		ret = rte_member_add_ht(setsum, key, set_id);
		break;
	case RTE_MEMBER_TYPE_VBF:
		ret = rte_member_add_vbf(setsum, key, set_id);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}


int
rte_member_lookup(const void *ss, const void *key,
		MEMBER_SET_TYPE *set_id)
{
	const struct rte_member_setsum *setsum = ss;
	if (setsum == NULL || key == NULL || set_id == NULL)
		return -EINVAL;

	int ret = 0;
	switch (setsum->type) {
	case RTE_MEMBER_TYPE_HT:
		ret = rte_member_lookup_ht(setsum, key, set_id);
		break;
	case RTE_MEMBER_TYPE_VBF:
		ret = rte_member_lookup_vbf(setsum, key, set_id);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}


int
rte_member_lookup_bulk(const void *ss, const void **keys, uint32_t num_keys,
		MEMBER_SET_TYPE *set_ids)
{
	const struct rte_member_setsum *setsum = ss;

	if (setsum == NULL || keys == NULL || set_ids == NULL)
		return -EINVAL;

	int ret = 0;
	switch (setsum->type) {
	case RTE_MEMBER_TYPE_HT:
		ret = rte_member_lookup_bulk_ht(setsum, keys, num_keys,
				set_ids);
		break;
	case RTE_MEMBER_TYPE_VBF:
		ret = rte_member_lookup_bulk_vbf(setsum, keys, num_keys,
				set_ids);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

int
rte_member_lookup_multi(const void *ss, const void *key,
		uint32_t match_per_key, MEMBER_SET_TYPE *set_id)
{
	const struct rte_member_setsum *setsum = ss;

	if (setsum == NULL || key == NULL || set_id == NULL)
		return -EINVAL;
	int ret = 0;
	switch (setsum->type) {
	case RTE_MEMBER_TYPE_HT:
		ret = rte_member_lookup_multi_ht(setsum, key, match_per_key,
				set_id);
		break;
	case RTE_MEMBER_TYPE_VBF:
		ret = rte_member_lookup_multi_vbf(setsum, key, match_per_key,
				set_id);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}


int
rte_member_lookup_multi_bulk(const void *ss, const void **keys,
		uint32_t num_keys, uint32_t max_match_per_key,
		uint32_t *match_count, MEMBER_SET_TYPE *set_ids)
{
	const struct rte_member_setsum *setsum = ss;
	if (setsum == NULL || keys == NULL || set_ids == NULL ||
			match_count == NULL)
		return -EINVAL;
	int ret = 0;
	switch (setsum->type) {
	case RTE_MEMBER_TYPE_HT:
		ret = rte_member_lookup_multi_bulk_ht(setsum, keys, num_keys,
				max_match_per_key, match_count, set_ids);
		break;
	case RTE_MEMBER_TYPE_VBF:
		ret = rte_member_lookup_multi_bulk_vbf(setsum, keys, num_keys,
				max_match_per_key, match_count, set_ids);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}


int
rte_member_delete(void *ss, const void *key, MEMBER_SET_TYPE set_id)
{
	struct rte_member_setsum *setsum = ss;
	if (setsum == NULL || key == NULL)
		return -EINVAL;
	int ret = 0;
	switch (setsum->type) {
	case RTE_MEMBER_TYPE_HT:
		ret = rte_member_delete_ht(setsum, key, set_id);
		break;
	case RTE_MEMBER_TYPE_VBF:
	default:
		return -EINVAL;
	}
	return ret;
}


void
rte_member_reset(const void *ss)
{
	const struct rte_member_setsum *setsum = ss;
	if (setsum == NULL)
		return;
	switch (setsum->type) {
	case RTE_MEMBER_TYPE_HT:
		rte_member_reset_ht(setsum);
		break;
	case RTE_MEMBER_TYPE_VBF:
		rte_member_reset_vbf(setsum);
		break;
	default:
		break;
	}
}
