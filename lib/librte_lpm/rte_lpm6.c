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
#include <stdint.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <errno.h>
#include <sys/queue.h>

#include <rte_log.h>
#include <rte_branch_prediction.h>
#include <rte_common.h>
#include <rte_memory.h>
#include <rte_malloc.h>
#include <rte_memzone.h>
#include <rte_memcpy.h>
#include <rte_eal.h>
#include <rte_eal_memconfig.h>
#include <rte_per_lcore.h>
#include <rte_string_fns.h>
#include <rte_errno.h>
#include <rte_rwlock.h>
#include <rte_spinlock.h>
#include <rte_compat.h>
#include <tree.h>

#include "rte_lpm6.h"

#define RTE_LPM6_TBL24_NUM_ENTRIES        (1 << 24)
#define RTE_LPM6_TBL8_GROUP_NUM_ENTRIES         256
#define RTE_LPM6_TBL8_MAX_NUM_GROUPS      (1 << 21)

#define RTE_LPM6_VALID_EXT_ENTRY_BITMASK 0xA0000000
#define RTE_LPM6_LOOKUP_SUCCESS          0x20000000
#define RTE_LPM6_TBL8_BITMASK            0x001FFFFF

#define ADD_FIRST_BYTE                            3
#define LOOKUP_FIRST_BYTE                         4
#define BYTE_SIZE                                 8
#define BYTES2_SIZE                              16

#define lpm6_tbl8_gindex next_hop

/** Flags for setting an entry as valid/invalid. */
enum valid_flag {
	INVALID = 0,
	VALID
};

TAILQ_HEAD(rte_lpm6_list, rte_tailq_entry);

static struct rte_tailq_elem rte_lpm6_tailq = {
	.name = "RTE_LPM6",
};
EAL_REGISTER_TAILQ(rte_lpm6_tailq)

/** Tbl entry structure. It is the same for both tbl24 and tbl8 */
struct rte_lpm6_tbl_entry {
	uint32_t next_hop:	21;  /**< Next hop / next table to be checked. */
	uint32_t depth	:8;      /**< Rule depth. */

	/* Flags. */
	uint32_t valid     :1;   /**< Validation flag. */
	uint32_t valid_group :1; /**< Group validation flag. */
	uint32_t ext_entry :1;   /**< External entry. */
};

/** Rules tbl entry structure. */
struct rte_lpm6_rule {
	uint8_t ip[RTE_LPM6_IPV6_ADDR_SIZE]; /**< Rule IP address. */
	uint16_t next_hop; /**< Rule next hop. */
	uint8_t depth; /**< Rule depth. */
	RB_ENTRY(rte_lpm6_rule) link;
};

/** LPM6 structure. */
struct rte_lpm6 {
	/* LPM metadata. */
	char name[RTE_LPM6_NAMESIZE];    /**< Name of the lpm. */
	uint32_t number_tbl8s;           /**< Number of tbl8s to allocate. */

	/* LPM rules. */
	RB_HEAD(rte_lpm6_rules_tree, rte_lpm6_rule) rules[RTE_LPM6_MAX_DEPTH + 1];

	/* LPM Tables. */
	struct rte_lpm6_tbl_entry tbl24[RTE_LPM6_TBL24_NUM_ENTRIES]
			__rte_cache_aligned; /**< LPM tbl24 table. */
	struct rte_lpm6_tbl_entry tbl8[0]
			__rte_cache_aligned; /**< LPM tbl8 table. */
};

/* Comparison function for red-black tree nodes.
   "If the first argument is smaller than the second, the function
	returns a value smaller than zero.	If they are equal, the function
	returns zero.  Otherwise, it should return a value greater than zero."
*/
static inline int rules_cmp(const struct rte_lpm6_rule *r1,
				const struct rte_lpm6_rule *r2)
{
	return memcmp(r1->ip, r2->ip, RTE_LPM6_IPV6_ADDR_SIZE);
}

/* Satisfy old style attribute in tree.h header */
#ifndef __unused
#define __unused __attribute__ ((unused))
#endif

/* Generate internal functions and make them static. */
RB_GENERATE_STATIC(rte_lpm6_rules_tree, rte_lpm6_rule, link, rules_cmp)

/*
 * Takes an array of uint8_t (IPv6 address) and masks it using the depth.
 * It leaves untouched one bit per unit in the depth variable
 * and set the rest to 0.
 */
static inline void
mask_ip(uint8_t *ip, uint8_t depth)
{
		int16_t part_depth, mask;
		int i;

		part_depth = depth;

		for (i = 0; i < RTE_LPM6_IPV6_ADDR_SIZE; i++) {
			if (part_depth < BYTE_SIZE && part_depth >= 0) {
				mask = (uint16_t)(~(UINT8_MAX >> part_depth));
				ip[i] = (uint8_t)(ip[i] & mask);
			} else if (part_depth < 0) {
				ip[i] = 0;
			}
			part_depth -= BYTE_SIZE;
		}
}

/*
 * Allocates memory for LPM object
 */
struct rte_lpm6 *
rte_lpm6_create(const char *name, int socket_id,
		const struct rte_lpm6_config *config)
{
	char mem_name[RTE_LPM6_NAMESIZE];
	struct rte_lpm6 *lpm = NULL;
	struct rte_tailq_entry *te;
	uint64_t mem_size;
	unsigned int depth;
	struct rte_lpm6_list *lpm_list;

	lpm_list = RTE_TAILQ_CAST(rte_lpm6_tailq.head, rte_lpm6_list);

	RTE_BUILD_BUG_ON(sizeof(struct rte_lpm6_tbl_entry) != sizeof(uint32_t));

	/* Check user arguments. */
	if ((name == NULL) || (socket_id < -1) || (config == NULL) ||
			config->number_tbl8s > RTE_LPM6_TBL8_MAX_NUM_GROUPS) {
		rte_errno = EINVAL;
		return NULL;
	}

	snprintf(mem_name, sizeof(mem_name), "LPM_%s", name);

	/* Determine the amount of memory to allocate. */
	mem_size = sizeof(*lpm) + (sizeof(lpm->tbl8[0]) *
			RTE_LPM6_TBL8_GROUP_NUM_ENTRIES * config->number_tbl8s);

	rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);

	/* Guarantee there's no existing */
	TAILQ_FOREACH(te, lpm_list, next) {
		lpm = (struct rte_lpm6 *) te->data;
		if (strncmp(name, lpm->name, RTE_LPM6_NAMESIZE) == 0)
			break;
	}
	lpm = NULL;
	if (te != NULL) {
		rte_errno = EEXIST;
		goto exit;
	}

	/* allocate tailq entry */
	te = rte_zmalloc("LPM6_TAILQ_ENTRY", sizeof(*te), 0);
	if (te == NULL) {
		RTE_LOG(ERR, LPM, "Failed to allocate tailq entry!\n");
		goto exit;
	}

	/* Allocate memory to store the LPM data structures. */
	lpm = (struct rte_lpm6 *)rte_zmalloc_socket(mem_name, (size_t)mem_size,
			RTE_CACHE_LINE_SIZE, socket_id);

	if (lpm == NULL) {
		RTE_LOG(ERR, LPM, "LPM memory allocation failed\n");
		rte_free(te);
		goto exit;
	}

	/* Save user arguments. */
	lpm->number_tbl8s = config->number_tbl8s;
	snprintf(lpm->name, sizeof(lpm->name), "%s", name);

	/* Vyatta change to use red-black tree */
	for (depth = 0; depth <= RTE_LPM6_MAX_DEPTH; ++depth)
		RB_INIT(&lpm->rules[depth]);

	te->data = (void *) lpm;

	TAILQ_INSERT_TAIL(lpm_list, te, next);

exit:
	rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);

	return lpm;
}

/*
 * Find an existing lpm table and return a pointer to it.
 */
struct rte_lpm6 *
rte_lpm6_find_existing(const char *name)
{
	struct rte_lpm6 *l = NULL;
	struct rte_tailq_entry *te;
	struct rte_lpm6_list *lpm_list;

	lpm_list = RTE_TAILQ_CAST(rte_lpm6_tailq.head, rte_lpm6_list);

	rte_rwlock_read_lock(RTE_EAL_TAILQ_RWLOCK);
	TAILQ_FOREACH(te, lpm_list, next) {
		l = (struct rte_lpm6 *) te->data;
		if (strncmp(name, l->name, RTE_LPM6_NAMESIZE) == 0)
			break;
	}
	rte_rwlock_read_unlock(RTE_EAL_TAILQ_RWLOCK);

	if (te == NULL) {
		rte_errno = ENOENT;
		return NULL;
	}

	return l;
}

/*
 * Deallocates memory for given LPM table.
 */
void
rte_lpm6_free(struct rte_lpm6 *lpm)
{
	struct rte_lpm6_list *lpm_list;
	struct rte_tailq_entry *te;

	/* Check user arguments. */
	if (lpm == NULL)
		return;

	lpm_list = RTE_TAILQ_CAST(rte_lpm6_tailq.head, rte_lpm6_list);

	rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);

	/* find our tailq entry */
	TAILQ_FOREACH(te, lpm_list, next) {
		if (te->data == (void *) lpm)
			break;
	}

	if (te != NULL)
		TAILQ_REMOVE(lpm_list, te, next);

	rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);
	rte_lpm6_delete_all(lpm);

	rte_free(lpm);
	rte_free(te);
}

/*
 * Checks if a rule already exists in the rules table and updates
 * the nexthop if so. Otherwise it adds a new rule if enough space is available.
 */
static struct rte_lpm6_rule *
rule_add(struct rte_lpm6 *lpm, uint8_t *ip, uint16_t next_hop, uint8_t depth)
{
	struct rte_lpm6_rules_tree *head = &lpm->rules[depth];
	struct rte_lpm6_rule *r, *old;

	/*
	 * NB: uses regular malloc to avoid chewing up precious
	 *  memory pool space for rules.
	 */
	r = malloc(sizeof(*r));
	if (!r)
		return NULL;

	rte_memcpy(r->ip, ip, RTE_LPM6_IPV6_ADDR_SIZE);
	r->next_hop = next_hop;
	r->depth = depth;

	old = RB_INSERT(rte_lpm6_rules_tree, head, r);
	if (!old)
		return r;

	/* collision with existing rule */
	free(r);

	return old;
}

/*
 * Find, clean and allocate a tbl8.
 */
static inline int32_t
tbl8_alloc(struct rte_lpm6 *lpm)
{
	uint32_t tbl8_gindex; /* tbl8 group index. */
	struct rte_lpm6_tbl_entry *tbl8_entry;
	struct rte_lpm6_tbl_entry *tbl8 = lpm->tbl8;

	/* Scan through tbl8 to find a free (i.e. INVALID) tbl8 group. */
	for (tbl8_gindex = 0; tbl8_gindex < lpm->number_tbl8s;
			tbl8_gindex++) {
		tbl8_entry = &tbl8[tbl8_gindex * RTE_LPM6_TBL8_GROUP_NUM_ENTRIES];
		/* If a free tbl8 group is found clean it and set as VALID. */
		if (!tbl8_entry->valid_group) {
			memset(&tbl8_entry[0], 0,
					RTE_LPM6_TBL8_GROUP_NUM_ENTRIES *
					sizeof(tbl8_entry[0]));

			tbl8_entry->valid_group = VALID;

			/* Return group index for allocated tbl8 group. */
			return tbl8_gindex;
		}
	}

	/* If there are no tbl8 groups free then return error. */
	return -ENOSPC;
}

static inline void
tbl8_free(struct rte_lpm6_tbl_entry *tbl8, uint32_t tbl8_group_start)
{
	/* Set tbl8 group invalid*/
	memset(&tbl8[tbl8_group_start], 0, RTE_LPM6_TBL8_GROUP_NUM_ENTRIES *
		sizeof(tbl8[0]));
}

static int
tbl8_is_free(struct rte_lpm6_tbl_entry *tbl8, uint32_t tbl8_group_start)
{
	uint32_t i, tbl8_group_end;
		tbl8_group_end = tbl8_group_start +
			RTE_LPM6_TBL8_GROUP_NUM_ENTRIES;

	for (i = tbl8_group_start; i < tbl8_group_end; i++) {
		if (tbl8[i].valid)
			return 0;
	}
	return 1;
}

/*
 * Function that expands a rule across the data structure when a less-generic
 * one has been added before. It assures that every possible combination of bits
 * in the IP address returns a match.
 */
static void
expand_rule(struct rte_lpm6 *lpm, uint32_t tbl8_gindex, uint8_t depth,
		uint16_t next_hop)
{
	uint32_t tbl8_group_end, tbl8_gindex_next, j;

	tbl8_group_end = tbl8_gindex + RTE_LPM6_TBL8_GROUP_NUM_ENTRIES;

	struct rte_lpm6_tbl_entry new_tbl8_entry = {
		.valid = VALID,
		.valid_group = VALID,
		.depth = depth,
		.next_hop = next_hop,
		.ext_entry = 0,
	};

	rte_compiler_barrier();

	for (j = tbl8_gindex; j < tbl8_group_end; j++) {
		if (!lpm->tbl8[j].valid || (lpm->tbl8[j].ext_entry == 0
				&& lpm->tbl8[j].depth <= depth)) {

			lpm->tbl8[j] = new_tbl8_entry;

		} else if (lpm->tbl8[j].ext_entry == 1) {

			tbl8_gindex_next = lpm->tbl8[j].lpm6_tbl8_gindex
					* RTE_LPM6_TBL8_GROUP_NUM_ENTRIES;
			expand_rule(lpm, tbl8_gindex_next, depth, next_hop);
		}
	}
}

/*
 * Partially adds a new route to the data structure (tbl24+tbl8s).
 * It returns 0 on success, a negative number on failure, or 1 if
 * the process needs to be continued by calling the function again.
 */
static inline int
add_step(struct rte_lpm6 *lpm, struct rte_lpm6_tbl_entry *tbl,
		struct rte_lpm6_tbl_entry **tbl_next, uint8_t *ip, uint8_t bytes,
		uint8_t first_byte, uint8_t depth, uint16_t next_hop)
{
	uint32_t tbl_index, tbl_range, tbl8_group_start, tbl8_group_end, i;
	int32_t tbl8_gindex;
	int8_t bitshift;
	uint8_t bits_covered;

	/*
	 * Calculate index to the table based on the number and position
	 * of the bytes being inspected in this step.
	 */
	tbl_index = 0;
	for (i = first_byte; i < (uint32_t)(first_byte + bytes); i++) {
		bitshift = (int8_t)((bytes - i)*BYTE_SIZE);

		if (bitshift < 0) bitshift = 0;
		tbl_index = tbl_index | ip[i-1] << bitshift;
	}

	/* Number of bits covered in this step */
	bits_covered = (uint8_t)((bytes+first_byte-1)*BYTE_SIZE);

	/*
	 * If depth if smaller than this number (ie this is the last step)
	 * expand the rule across the relevant positions in the table.
	 */
	if (depth <= bits_covered) {
		struct rte_lpm6_tbl_entry new_tbl_entry = {
			.next_hop = next_hop,
			.depth = depth,
			.valid = VALID,
			.valid_group = VALID,
			.ext_entry = 0,
		};

		rte_compiler_barrier();

		tbl_range = 1 << (bits_covered - depth);

		for (i = tbl_index; i < (tbl_index + tbl_range); i++) {
			if (!tbl[i].valid || (tbl[i].ext_entry == 0 &&
					tbl[i].depth <= depth)) {

				tbl[i] = new_tbl_entry;

			} else if (tbl[i].ext_entry == 1) {

				/*
				 * If tbl entry is valid and extended calculate the index
				 * into next tbl8 and expand the rule across the data structure.
				 */
				tbl8_gindex = tbl[i].lpm6_tbl8_gindex *
						RTE_LPM6_TBL8_GROUP_NUM_ENTRIES;
				expand_rule(lpm, tbl8_gindex, depth, next_hop);
			}
		}

		return 0;
	}
	/*
	 * If this is not the last step just fill one position
	 * and calculate the index to the next table.
	 */
	else {
		/* If it's invalid a new tbl8 is needed */
		if (!tbl[tbl_index].valid) {
			tbl8_gindex = tbl8_alloc(lpm);
			if (tbl8_gindex < 0) {
				return -ENOSPC;
			}

			struct rte_lpm6_tbl_entry new_tbl_entry = {
				.lpm6_tbl8_gindex = tbl8_gindex,
				.depth = 0,
				.valid = VALID,
				.valid_group = VALID,
				.ext_entry = 1,
			};

			rte_compiler_barrier();

			tbl[tbl_index] = new_tbl_entry;
		}
		/*
		 * If it's valid but not extended the rule that was stored *
		 * here needs to be moved to the next table.
		 */
		else if (tbl[tbl_index].ext_entry == 0) {
			tbl8_gindex = tbl8_alloc(lpm);
			if (tbl8_gindex < 0) {
				return -ENOSPC;
			}
			tbl8_group_start = tbl8_gindex *
					RTE_LPM6_TBL8_GROUP_NUM_ENTRIES;
			tbl8_group_end = tbl8_group_start +
					RTE_LPM6_TBL8_GROUP_NUM_ENTRIES;

			/* Populate new tbl8 with tbl value. */
			for (i = tbl8_group_start; i < tbl8_group_end; i++) {
				lpm->tbl8[i].valid = VALID;
				lpm->tbl8[i].depth = tbl[tbl_index].depth;
				lpm->tbl8[i].next_hop = tbl[tbl_index].next_hop;
				lpm->tbl8[i].ext_entry = 0;
			}

			/*
			 * Update tbl entry to point to new tbl8 entry. Note: The
			 * ext_flag and tbl8_index need to be updated simultaneously,
			 * so assign whole structure in one go.
			 */
			struct rte_lpm6_tbl_entry new_tbl_entry = {
				.lpm6_tbl8_gindex = tbl8_gindex,
				.depth = 0,
				.valid = VALID,
				.valid_group = VALID,
				.ext_entry = 1,
			};

			rte_compiler_barrier();

			tbl[tbl_index] = new_tbl_entry;
		}

		*tbl_next = &(lpm->tbl8[tbl[tbl_index].lpm6_tbl8_gindex *
				RTE_LPM6_TBL8_GROUP_NUM_ENTRIES]);
	}

	return 1;
}

int
rte_lpm6_add_v20(struct rte_lpm6 *lpm, uint8_t *ip, uint8_t depth,
		uint8_t next_hop)
{
	return rte_lpm6_add_v1607(lpm, ip, depth, next_hop);
}
VERSION_SYMBOL(rte_lpm6_add, _v20, 2.0);

/*
 * Add a route
 */
int
rte_lpm6_add_v1607(struct rte_lpm6 *lpm, uint8_t *ip, uint8_t depth,
		uint16_t next_hop)
{
	struct rte_lpm6_tbl_entry *tbl;
	struct rte_lpm6_tbl_entry *tbl_next;
	struct rte_lpm6_rule *rule;
	int status, i;
	uint8_t masked_ip[RTE_LPM6_IPV6_ADDR_SIZE];

	/* Check user arguments. */
	if ((lpm == NULL) || (depth < 1) || (depth > RTE_LPM6_MAX_DEPTH))
		return -EINVAL;

	/* Copy the IP and mask it to avoid modifying user's input data. */
	memcpy(masked_ip, ip, RTE_LPM6_IPV6_ADDR_SIZE);
	mask_ip(masked_ip, depth);

	/* Add the rule to the rule table. */
	rule = rule_add(lpm, masked_ip, next_hop, depth);

	/* If there is no space available for new rule return error. */
	if (rule == NULL) {
		return -EINVAL;
	}

	/* inspect the first three bytes through tbl24 on the first step. */
	tbl = lpm->tbl24;
	status = add_step (lpm, tbl, &tbl_next, masked_ip, ADD_FIRST_BYTE, 1,
			depth, next_hop);
	if (status < 0) {
		rte_lpm6_delete(lpm, masked_ip, depth);

		return status;
	}

	/*
	 * inspect one by one the rest of the bytes until
	 * the process is completed.
	 */
	for (i = ADD_FIRST_BYTE; i < RTE_LPM6_IPV6_ADDR_SIZE && status == 1; i++) {
		tbl = tbl_next;
		status = add_step (lpm, tbl, &tbl_next, masked_ip, 1, (uint8_t)(i+1),
				depth, next_hop);
		if (status < 0) {
			rte_lpm6_delete(lpm, masked_ip, depth);

			return status;
		}
	}

	return status;
}
BIND_DEFAULT_SYMBOL(rte_lpm6_add, _v1607, 16.07);
MAP_STATIC_SYMBOL(int rte_lpm6_add(struct rte_lpm6 *lpm, uint8_t *ip,
		uint8_t depth, uint16_t next_hop), rte_lpm6_add_v1607);

/*
 * Takes a pointer to a table entry and inspect one level.
 * The function returns 0 on lookup success, ENOENT if no match was found
 * or 1 if the process needs to be continued by calling the function again.
 */
static inline int
lookup_step(const struct rte_lpm6 *lpm, const struct rte_lpm6_tbl_entry *tbl,
		const struct rte_lpm6_tbl_entry **tbl_next, uint8_t *ip,
		uint8_t first_byte, uint16_t *next_hop)
{
	uint32_t tbl8_index, tbl_entry;

	/* Take the integer value from the pointer. */
	tbl_entry = *(const uint32_t *)tbl;

	/* If it is valid and extended we calculate the new pointer to return. */
	if ((tbl_entry & RTE_LPM6_VALID_EXT_ENTRY_BITMASK) ==
			RTE_LPM6_VALID_EXT_ENTRY_BITMASK) {

		tbl8_index = ip[first_byte-1] +
				((tbl_entry & RTE_LPM6_TBL8_BITMASK) *
				RTE_LPM6_TBL8_GROUP_NUM_ENTRIES);

		*tbl_next = &lpm->tbl8[tbl8_index];

		return 1;
	} else {
		/* If not extended then we can have a match. */
		*next_hop = (uint16_t)tbl_entry;
		return (tbl_entry & RTE_LPM6_LOOKUP_SUCCESS) ? 0 : -ENOENT;
	}
}

int
rte_lpm6_lookup_v20(const struct rte_lpm6 *lpm, uint8_t *ip, uint8_t *next_hop)
{
	return rte_lpm6_lookup_v1607(lpm, ip, (uint16_t*)next_hop);
}
VERSION_SYMBOL(rte_lpm6_lookup, _v20, 2.0);
/*
 * Looks up an IP
 */
int
rte_lpm6_lookup_v1607(const struct rte_lpm6 *lpm, uint8_t *ip, uint16_t *next_hop)
{
	const struct rte_lpm6_tbl_entry *tbl;
	const struct rte_lpm6_tbl_entry *tbl_next = NULL;
	int status;
	uint8_t first_byte;
	uint32_t tbl24_index;

	/* DEBUG: Check user input arguments. */
	if ((lpm == NULL) || (ip == NULL) || (next_hop == NULL)) {
		return -EINVAL;
	}

	first_byte = LOOKUP_FIRST_BYTE;
	tbl24_index = (ip[0] << BYTES2_SIZE) | (ip[1] << BYTE_SIZE) | ip[2];

	/* Calculate pointer to the first entry to be inspected */
	tbl = &lpm->tbl24[tbl24_index];

	do {
		/* Continue inspecting following levels until success or failure */
		status = lookup_step(lpm, tbl, &tbl_next, ip, first_byte++, next_hop);
		tbl = tbl_next;
	} while (status == 1);

	return status;
}
BIND_DEFAULT_SYMBOL(rte_lpm6_lookup, _v1607, 16.07);
MAP_STATIC_SYMBOL(int rte_lpm6_lookup(const struct rte_lpm6 *lpm, uint8_t *ip,
		uint16_t *next_hop), rte_lpm6_lookup_v1607);

int
rte_lpm6_lookup_bulk_func_v20(const struct rte_lpm6 *lpm,
		uint8_t ips[][RTE_LPM6_IPV6_ADDR_SIZE],
		int16_t *next_hops, unsigned n)
{
	return rte_lpm6_lookup_bulk_func_v1607(lpm, ips, (int32_t*)next_hops, n);
}
VERSION_SYMBOL(rte_lpm6_lookup_bulk_func, _v20, 2.0);

/*
 * Looks up a group of IP addresses
 */
int
rte_lpm6_lookup_bulk_func_v1607(const struct rte_lpm6 *lpm,
		uint8_t ips[][RTE_LPM6_IPV6_ADDR_SIZE],
		int32_t *next_hops, unsigned n)
{
	unsigned i;
	const struct rte_lpm6_tbl_entry *tbl;
	const struct rte_lpm6_tbl_entry *tbl_next = NULL;
	uint32_t tbl24_index;
	uint8_t first_byte;
	uint16_t next_hop;
	int status;

	/* DEBUG: Check user input arguments. */
	if ((lpm == NULL) || (ips == NULL) || (next_hops == NULL)) {
		return -EINVAL;
	}

	for (i = 0; i < n; i++) {
		first_byte = LOOKUP_FIRST_BYTE;
		tbl24_index = (ips[i][0] << BYTES2_SIZE) |
				(ips[i][1] << BYTE_SIZE) | ips[i][2];

		/* Calculate pointer to the first entry to be inspected */
		tbl = &lpm->tbl24[tbl24_index];

		do {
			/* Continue inspecting following levels until success or failure */
			status = lookup_step(lpm, tbl, &tbl_next, ips[i], first_byte++,
					&next_hop);
			tbl = tbl_next;
		} while (status == 1);

		if (status < 0)
			next_hops[i] = -1;
		else
			next_hops[i] = next_hop;
	}

	return 0;
}
BIND_DEFAULT_SYMBOL(rte_lpm6_lookup_bulk_func, _v1607, 16.07);
MAP_STATIC_SYMBOL(int rte_lpm6_lookup_bulk_func(const struct rte_lpm6 *lpm, uint8_t ips[][RTE_LPM6_IPV6_ADDR_SIZE],
		int32_t *next_hops, unsigned n), rte_lpm6_lookup_bulk_func_v1607);

/*
 * Finds a rule in rule table.
 * NOTE: Valid range for depth parameter is 0 .. 128 inclusive.
 */
static struct rte_lpm6_rule *
rule_find(struct rte_lpm6 *lpm, uint8_t *ip, uint8_t depth)
{

	struct rte_lpm6_rules_tree *head = &lpm->rules[depth];
	struct rte_lpm6_rule k;

	rte_memcpy(k.ip, ip, RTE_LPM6_IPV6_ADDR_SIZE);

	return RB_FIND(rte_lpm6_rules_tree, head, &k);
}

static struct rte_lpm6_rule *
find_previous_rule(struct rte_lpm6 *lpm, uint8_t *ip, uint8_t depth,
			uint8_t *sub_rule_depth)
{
	struct rte_lpm6_rule *rule;
	uint8_t ip_masked[RTE_LPM6_IPV6_ADDR_SIZE];
	int prev_depth;

	/* Copy the IP and mask it to avoid modifying user's input data. */
	rte_memcpy(ip_masked, ip, RTE_LPM6_IPV6_ADDR_SIZE);
	for (prev_depth = depth; prev_depth >= 0; prev_depth--) {
		mask_ip(ip_masked, prev_depth);
		rule = rule_find(lpm, ip_masked, prev_depth);
		if (rule) {
			*sub_rule_depth = prev_depth;
			return rule;
		}
	}

	return NULL;
}

int
rte_lpm6_is_rule_present_v20(struct rte_lpm6 *lpm, uint8_t *ip, uint8_t depth,
uint8_t *next_hop)
{
	return rte_lpm6_is_rule_present_v1607(lpm, ip, depth, (uint16_t*)next_hop);
}
VERSION_SYMBOL(rte_lpm6_is_rule_present, _v20, 2.0);
/*
 * Look for a rule in the high-level rules table
 */
int
rte_lpm6_is_rule_present_v1607(struct rte_lpm6 *lpm, uint8_t *ip, uint8_t depth,
uint16_t *next_hop)
{
	uint8_t ip_masked[RTE_LPM6_IPV6_ADDR_SIZE];
	struct rte_lpm6_rule *rule;

	/* Check user arguments. */
	if ((lpm == NULL) || next_hop == NULL || ip == NULL ||
			(depth < 1) || (depth > RTE_LPM6_MAX_DEPTH))
		return -EINVAL;

	/* Copy the IP and mask it to avoid modifying user's input data. */
	rte_memcpy(ip_masked, ip, RTE_LPM6_IPV6_ADDR_SIZE);
	mask_ip(ip_masked, depth);

	/* Look for the rule using rule_find. */
	rule = rule_find(lpm, ip, depth);

	if (rule != NULL) {
		*next_hop = rule->next_hop;
		return 1;
	}

	/* If rule is not found return 0. */
	return 0;
}
BIND_DEFAULT_SYMBOL(rte_lpm6_is_rule_present, _v1607, 16.07);
MAP_STATIC_SYMBOL(int rte_lpm6_is_rule_present(struct rte_lpm6 *lpm, uint8_t *ip,
		uint8_t depth, uint16_t *next_hop), rte_lpm6_is_rule_present_v1607);

/*
 * Delete a rule from the rule table.
 * NOTE: Valid range for depth parameter is 0 .. 128 inclusive.
 */
static inline void
rule_delete(struct rte_lpm6 *lpm, struct rte_lpm6_rule *r, uint8_t depth)
{
	struct rte_lpm6_rules_tree *head = &lpm->rules[depth];

	RB_REMOVE(rte_lpm6_rules_tree, head, r);

	free(r);
}

/*
 * Function that run through the data structure when and clean entries
 * that where expanded by expand_rule().
 */
static void
delete_expand_step(struct rte_lpm6 *lpm, uint32_t tbl8_gindex, uint8_t depth,
		uint16_t next_hop)
{
	uint32_t tbl8_group_end, tbl8_gindex_next, j;

	tbl8_group_end = tbl8_gindex + RTE_LPM6_TBL8_GROUP_NUM_ENTRIES;

	struct rte_lpm6_tbl_entry empty_tbl8_entry = {
		.valid = INVALID,
		.valid_group = INVALID,
		.depth = 0,
		.next_hop = 0,
		.ext_entry = 0,
	};

	rte_compiler_barrier();

	for (j = tbl8_gindex; j < tbl8_group_end; j++) {
		if (lpm->tbl8[j].valid && lpm->tbl8[j].ext_entry == 0
				&& lpm->tbl8[j].depth == depth && lpm->tbl8[j].next_hop == next_hop) {

			lpm->tbl8[j] = empty_tbl8_entry;

		} else if (lpm->tbl8[j].ext_entry == 1) {

			tbl8_gindex_next = lpm->tbl8[j].lpm6_tbl8_gindex
					* RTE_LPM6_TBL8_GROUP_NUM_ENTRIES;
			delete_expand_step(lpm, tbl8_gindex_next, depth, next_hop);
		}
	}

}
/*
 * Partially delete a route from the data structure (tbl24+tbl8s).
 * It may be called and partially added routes (after an unsuccesful add_step).
 */
static void
delete_step(struct rte_lpm6 *lpm, struct rte_lpm6_tbl_entry *tbl,
		uint8_t *ip, uint32_t tbl_group_start, struct rte_lpm6_rule *sub_rule, uint8_t depth, uint8_t bits_covered, uint16_t next_hop)
{
	uint32_t i, tbl_index = tbl_group_start + ip[bits_covered / 8 - 1];
	struct rte_lpm6_tbl_entry empty_tbl8_entry = {
		.valid = INVALID,
		.valid_group = INVALID,
		.depth = 0,
		.next_hop = 0,
		.ext_entry = 0,
	};

	rte_compiler_barrier();

	/* recurse until we are at the last tbl8 that should have been deleted for this rule without an expand */
	if (depth > bits_covered) {
		uint32_t tbl_group_next;

		/* check if we have a partially added rule */
		if (tbl[tbl_index].ext_entry == 1) {
			/* calculate the next tbl_index */
			tbl_group_next = tbl[tbl_index].lpm6_tbl8_gindex * RTE_LPM6_TBL8_GROUP_NUM_ENTRIES;
			delete_step(lpm, lpm->tbl8, ip, tbl_group_next, sub_rule, depth, bits_covered + 8, next_hop);
		}
	} else {
		uint32_t tbl_range;

		/* last iteration, we may need to remove what we have done through the expand */
		tbl_range = 1 << (bits_covered - depth);
		for (i = tbl_index; i < (tbl_index + tbl_range); i++) {
			if (tbl[i].ext_entry == 1) {
				uint32_t tbl8_gindex = tbl[i].lpm6_tbl8_gindex *
						RTE_LPM6_TBL8_GROUP_NUM_ENTRIES;
				delete_expand_step(lpm, tbl8_gindex, depth, next_hop);
			} else {
				/* do a normal cleaning */
				if (tbl[i].depth == depth) {
					if (sub_rule) {
						tbl[i].depth = sub_rule->depth;
						tbl[i].next_hop = sub_rule->next_hop;
					} else {
						tbl[i] = empty_tbl8_entry;
					}
				}
			}
		}
		return;
	}

	/* check if we can recycle the current tbl_entry because we have cleaned all its childs  */
	if (tbl[tbl_index].ext_entry) {
		if (tbl8_is_free(lpm->tbl8, tbl[tbl_index].lpm6_tbl8_gindex * RTE_LPM6_TBL8_GROUP_NUM_ENTRIES)) {
			tbl[tbl_index] = empty_tbl8_entry;
		}
	}

	/* we are recursing on the tbl24 so we don't need to check those cases */
	if (bits_covered == 24) {
		return;
	}

	/* try to clean our current tbl group from tbl_group_start to tbl_group_end */
	for (i = tbl_group_start; i < tbl_group_start + RTE_LPM6_TBL8_GROUP_NUM_ENTRIES; i++) {
		if (tbl[i].valid == 1) {
			break;
		}
	}
	if (i == tbl_group_start + RTE_LPM6_TBL8_GROUP_NUM_ENTRIES) {
		/* we can free a whole group */
		tbl8_free(lpm->tbl8, tbl_group_start);
	}
}

/*
 * Find rule to replace the just deleted. If there is no rule to
 * replace the rule_to_delete we return NULL and invalidate the table
 * entries associated with this rule.
 */
static void
rule_replace(struct rte_lpm6 *lpm, uint8_t *ip, uint8_t depth, uint16_t next_hop)
{
	uint8_t ip_masked[RTE_LPM6_IPV6_ADDR_SIZE];
	struct rte_lpm6_rule *sub_rule;
	uint8_t sub_depth = 0;
	uint32_t tbl_index;

	/* Copy the IP and mask it to avoid modifying user's input data. */
	rte_memcpy(ip_masked, ip, RTE_LPM6_IPV6_ADDR_SIZE);
	mask_ip(ip_masked, depth);

	sub_rule = find_previous_rule(lpm, ip_masked, depth, &sub_depth);

	tbl_index = (ip_masked[0] << BYTES2_SIZE) | (ip_masked[1] << BYTE_SIZE);

	delete_step(lpm, lpm->tbl24, ip_masked, tbl_index, sub_rule, depth, 24, next_hop);
}

/*
 * Deletes a rule
 */
int
rte_lpm6_delete(struct rte_lpm6 *lpm, uint8_t *ip, uint8_t depth)
{
	struct rte_lpm6_rule *rule;
	uint8_t ip_masked[RTE_LPM6_IPV6_ADDR_SIZE];
	uint16_t next_hop;

	/*
	 * Check input arguments.
	 */
	if ((lpm == NULL) || (depth < 1) || (depth > RTE_LPM6_MAX_DEPTH)) {
		return -EINVAL;
	}

	/* Copy the IP and mask it to avoid modifying user's input data. */
	rte_memcpy(ip_masked, ip, RTE_LPM6_IPV6_ADDR_SIZE);
	mask_ip(ip_masked, depth);

	/*
	 * Find the index of the input rule, that needs to be deleted, in the
	 * rule table.
	 */
	rule = rule_find(lpm, ip_masked, depth);

	/*
	 * Check if rule_to_delete_index was found. If no rule was found the
	 * function rule_find returns -ENOENT.
	 */
	if (rule == NULL)
		return -EINVAL;

	next_hop = rule->next_hop;

	/* Delete the rule from the rule table. */
	rule_delete(lpm, rule, depth);

	/* Replace with next level up rule */
	rule_replace(lpm, ip_masked, depth, next_hop);

	return 0;
}

/*
 * Deletes a group of rules
 */
int
rte_lpm6_delete_bulk_func(struct rte_lpm6 *lpm,
		uint8_t ips[][RTE_LPM6_IPV6_ADDR_SIZE], uint8_t *depths, unsigned n)
{
	struct rte_lpm6_rule *rule;
	uint8_t ip_masked[RTE_LPM6_IPV6_ADDR_SIZE];
	unsigned i;
	uint16_t next_hop;

	/*
	 * Check input arguments.
	 */
	if ((lpm == NULL) || (ips == NULL) || (depths == NULL)) {
		return -EINVAL;
	}

	for (i = 0; i < n; i++) {
		/* Copy the IP and mask it to avoid modifying user's input data. */
		memcpy(ip_masked, ips[i], RTE_LPM6_IPV6_ADDR_SIZE);
		mask_ip(ip_masked, depths[i]);

		/*
		 * Find the index of the input rule, that needs to be deleted, in the
		 * rule table.
		 */
		rule = rule_find(lpm, ip_masked, depths[i]);

		/*
		 * Check if rule_to_delete_index was found. If no rule was found the
		 * function rule_find returns -ENOENT.
		 */
		if (rule == NULL)
			continue;

		next_hop = rule->next_hop;

		/* Delete the rule from the rule table. */
		rule_delete(lpm, rule, depths[i]);

		/* Replace with next level up rule */
		rule_replace(lpm, ip_masked, depths[i], next_hop);
	}

	return 0;
}


/*
 * Delete all rules from the LPM table.
 */
void
rte_lpm6_delete_all(struct rte_lpm6 *lpm)
{
	uint8_t depth;

	/* Delete all rules form the rules table. */
	for (depth = 0; depth < RTE_LPM6_MAX_DEPTH; ++depth) {
		struct rte_lpm6_rules_tree *head = &lpm->rules[depth];
		struct rte_lpm6_rule *r, *n;

		RB_FOREACH_SAFE(r, rte_lpm6_rules_tree, head, n) {
			rule_delete(lpm, r, depth);
		}
	}

	/* Zero tbl24. */
	memset(lpm->tbl24, 0, sizeof(lpm->tbl24));

	/* Zero tbl8. */
	memset(lpm->tbl8, 0, sizeof(lpm->tbl8[0]) *
			RTE_LPM6_TBL8_GROUP_NUM_ENTRIES * lpm->number_tbl8s);
}

/* Count usage of tbl8 */
unsigned
rte_lpm6_tbl8_count(const struct rte_lpm6 *lpm)
{
	unsigned i, count = 0;

	for (i = 0; i < lpm->number_tbl8s ; i++) {
		const struct rte_lpm6_tbl_entry *tbl8_entry
			= lpm->tbl8 + i * RTE_LPM6_TBL8_GROUP_NUM_ENTRIES;
		if (tbl8_entry->valid_group)
			++count;
	}
	return count;
}

unsigned
rte_lpm6_tbl8_free_count(const struct rte_lpm6 *lpm)
{
	return lpm->number_tbl8s - rte_lpm6_tbl8_count(lpm);
}
