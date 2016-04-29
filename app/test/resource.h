/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 RehiveTech. All rights reserved.
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
 *     * Neither the name of RehiveTech nor the names of its
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

#ifndef _RESOURCE_H_
#define _RESOURCE_H_

#include <sys/queue.h>
#include <stdio.h>
#include <stddef.h>

#include <rte_eal.h>
#include <rte_common.h>

TAILQ_HEAD(resource_list, resource);
extern struct resource_list resource_list;

struct resource {
	const char *name;
	const char *beg;
	const char *end;
	TAILQ_ENTRY(resource) next;
};

static inline size_t resource_size(const struct resource *r)
{
	return r->end - r->beg;
}

const struct resource *resource_find(const char *name);

int resource_fwrite(const struct resource *r, FILE *f);
int resource_fwrite_file(const struct resource *r, const char *fname);

/**
 * Treat the given resource as a tar archive. Extract
 * the archive to the current directory.
 */
int resource_untar(const struct resource *res);

/**
 * Treat the given resource as a tar archive. Remove
 * all files (related to the current directory) listed
 * in the tar archive.
 */
int resource_rm_by_tar(const struct resource *res);

void __resource_register(struct resource *r);

#define REGISTER_LINKED_RESOURCE(_n) \
extern const char beg_ ##_n;         \
extern const char end_ ##_n;         \
REGISTER_RESOURCE(_n, &beg_ ##_n, &end_ ##_n); \

#define REGISTER_RESOURCE(_n, _b, _e) \
static struct resource linkres_ ##_n = {       \
	.name = RTE_STR(_n),     \
	.beg = _b,               \
	.end = _e,               \
};                               \
__REGISTER_RESOURCE(linkres_ ##_n)

#define __REGISTER_RESOURCE(name)     \
RTE_INIT(resinitfn_ ##name);        \
static void resinitfn_ ##name(void) \
{                                   \
	__resource_register(&name); \
}

#endif
