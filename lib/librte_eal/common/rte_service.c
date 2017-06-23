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

#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <limits.h>
#include <string.h>
#include <dirent.h>

#include <rte_service.h>
#include "include/rte_service_private.h"

#include <rte_eal.h>
#include <rte_lcore.h>
#include <rte_common.h>
#include <rte_debug.h>
#include <rte_cycles.h>
#include <rte_atomic.h>

#define RTE_SERVICE_NUM_MAX 64

#define RTE_SERVICE_FLAG_REGISTERED_SHIFT 0

#define RTE_SERVICE_RUNSTATE_STOPPED 0
#define RTE_SERVICE_RUNSTATE_RUNNING 1

/* internal representation of a service */
struct rte_service_spec_impl {
	/* public part of the struct */
	struct rte_service_spec spec;

	/* atomic lock that when set indicates a service core is currently
	 * running this service callback. When not set, a core may take the
	 * lock and then run the service callback.
	 */
	rte_atomic32_t execute_lock;

	/* API set/get-able variables */
	int32_t runstate;
	uint8_t internal_flags;

	/* per service statistics */
	uint32_t num_mapped_cores;
	uint64_t calls;
	uint64_t cycles_spent;
};

/* the internal values of a service core */
struct core_state {
	uint64_t service_mask; /* map of services IDs are run on this core */
	uint8_t runstate; /* running or stopped */
	uint8_t is_service_core; /* set if core is currently a service core */

	/* extreme statistics */
	uint64_t calls_per_service[RTE_SERVICE_NUM_MAX];
};

static uint32_t rte_service_count;
static struct rte_service_spec_impl rte_services[RTE_SERVICE_NUM_MAX];
static struct core_state cores_state[RTE_MAX_LCORE];

/* returns 1 if service is registered and has not been unregistered
 * Returns 0 if service never registered, or has been unregistered
 */
static int
service_valid(uint32_t id) {
	return !!(rte_services[id].internal_flags &
		 (1 << RTE_SERVICE_FLAG_REGISTERED_SHIFT));
}

uint32_t
rte_service_get_count(void)
{
	return rte_service_count;
}

struct rte_service_spec *
rte_service_get_by_id(uint32_t id)
{
	struct rte_service_spec *service = NULL;
	if (id < rte_service_count)
		service = (struct rte_service_spec *)&rte_services[id];

	return service;
}

const char *
rte_service_get_name(const struct rte_service_spec *service)
{
	return service->name;
}

int32_t
rte_service_probe_capability(const struct rte_service_spec *service,
			     uint32_t capability)
{
	return service->capabilities & capability;
}

int32_t
rte_service_is_running(const struct rte_service_spec *spec)
{
	if (!spec)
		return -EINVAL;

	const struct rte_service_spec_impl *impl =
		(const struct rte_service_spec_impl *)spec;
	return impl->runstate == RTE_SERVICE_RUNSTATE_RUNNING;
}

int32_t
rte_service_register(const struct rte_service_spec *spec)
{
	uint32_t i;
	int32_t free_slot = -1;

	if (spec->callback == NULL || strlen(spec->name) == 0)
		return -EINVAL;

	for (i = 0; i < RTE_SERVICE_NUM_MAX; i++) {
		if (!service_valid(i)) {
			free_slot = i;
			break;
		}
	}

	if (free_slot < 0)
		return -ENOSPC;

	struct rte_service_spec_impl *s = &rte_services[free_slot];
	s->spec = *spec;
	s->internal_flags |= (1 << RTE_SERVICE_FLAG_REGISTERED_SHIFT);

	rte_smp_wmb();
	rte_service_count++;

	return 0;
}

int32_t
rte_service_unregister(struct rte_service_spec *spec)
{
	struct rte_service_spec_impl *s = NULL;
	struct rte_service_spec_impl *spec_impl =
		(struct rte_service_spec_impl *)spec;

	uint32_t i;
	uint32_t service_id;
	for (i = 0; i < RTE_SERVICE_NUM_MAX; i++) {
		if (&rte_services[i] == spec_impl) {
			s = spec_impl;
			service_id = i;
			break;
		}
	}

	if (!s)
		return -EINVAL;

	s->internal_flags &= ~(1 << RTE_SERVICE_FLAG_REGISTERED_SHIFT);

	for (i = 0; i < RTE_MAX_LCORE; i++)
		cores_state[i].service_mask &= ~(1 << service_id);

	memset(&rte_services[service_id], 0,
			sizeof(struct rte_service_spec_impl));

	rte_smp_wmb();
	rte_service_count--;

	return 0;
}

int32_t
rte_service_start(struct rte_service_spec *service)
{
	struct rte_service_spec_impl *s =
		(struct rte_service_spec_impl *)service;
	s->runstate = RTE_SERVICE_RUNSTATE_RUNNING;
	return 0;
}

int32_t
rte_service_stop(struct rte_service_spec *service)
{
	struct rte_service_spec_impl *s =
		(struct rte_service_spec_impl *)service;
	s->runstate = RTE_SERVICE_RUNSTATE_STOPPED;
	return 0;
}

static int32_t
rte_service_runner_func(void *arg)
{
	RTE_SET_USED(arg);
	uint32_t i;
	const int lcore = rte_lcore_id();
	struct core_state *cs = &cores_state[lcore];

	while (cores_state[lcore].runstate == RTE_SERVICE_RUNSTATE_RUNNING) {
		for (i = 0; i < rte_service_count; i++) {
			struct rte_service_spec_impl *s = &rte_services[i];
			uint64_t service_mask = cs->service_mask;

			if (s->runstate != RTE_SERVICE_RUNSTATE_RUNNING ||
					!(service_mask & (1 << i)))
				continue;

			uint32_t *lock = (uint32_t *)&s->execute_lock;
			if (rte_atomic32_cmpset(lock, 0, 1)) {
				void *userdata = s->spec.callback_userdata;
				uint64_t start = rte_rdtsc();
				s->spec.callback(userdata);
				uint64_t end = rte_rdtsc();

				uint64_t spent = end - start;
				s->cycles_spent += spent;
				s->calls++;
				cs->calls_per_service[i]++;

				rte_atomic32_clear(&s->execute_lock);
			}
		}
		rte_mb();
	}

	/* mark core as ready to accept work again */
	lcore_config[lcore].state = WAIT;

	return 0;
}

int32_t
rte_service_core_count(void)
{
	int32_t count = 0;
	uint32_t i;
	for (i = 0; i < RTE_MAX_LCORE; i++)
		count += cores_state[i].is_service_core;
	return count;
}

int32_t
rte_service_core_list(uint32_t array[], uint32_t n)
{
	uint32_t count = rte_service_core_count();
	if (count > n)
		return -ENOMEM;

	uint32_t i;
	uint32_t idx = 0;
	for (i = 0; i < RTE_MAX_LCORE; i++) {
		struct core_state *cs = &cores_state[i];
		if (cs->is_service_core) {
			array[idx] = i;
			idx++;
		}
	}

	return count;
}

int32_t
rte_service_init_default_mapping(void)
{
	/* create a default mapping from cores to services, then start the
	 * services to make them transparent to unaware applications.
	 */
	uint32_t i;
	int ret;
	uint32_t count = rte_service_get_count();
	struct rte_config *cfg = rte_eal_get_configuration();

	for (i = 0; i < count; i++) {
		struct rte_service_spec *s = rte_service_get_by_id(i);
		if (!s)
			return -EINVAL;

		ret = 0;
		int j;
		for (j = 0; j < RTE_MAX_LCORE; j++) {
			/* TODO: add lcore -> service mapping logic here */
			if (cfg->lcore_role[j] == ROLE_SERVICE) {
				ret = rte_service_enable_on_core(s, j);
				if (ret)
					rte_panic("Enabling service core %d on service %s failed\n",
							j, s->name);
			}
		}

		ret = rte_service_start(s);
		if (ret)
			rte_panic("failed to start service %s\n", s->name);
	}

	return 0;
}

static int32_t
service_update(struct rte_service_spec *service, uint32_t lcore,
		uint32_t *set, uint32_t *enabled)
{
	uint32_t i;
	int32_t sid = -1;

	for (i = 0; i < RTE_SERVICE_NUM_MAX; i++) {
		if ((struct rte_service_spec *)&rte_services[i] == service &&
				service_valid(i)) {
			sid = i;
			break;
		}
	}

	if (sid == -1 || lcore >= RTE_MAX_LCORE)
		return -EINVAL;

	if (!cores_state[lcore].is_service_core)
		return -EINVAL;

	if (set) {
		if (*set) {
			cores_state[lcore].service_mask |=  (1 << sid);
			rte_services[sid].num_mapped_cores++;
		} else {
			cores_state[lcore].service_mask &= ~(1 << sid);
			rte_services[sid].num_mapped_cores--;
		}
	}

	if (enabled)
		*enabled = (cores_state[lcore].service_mask & (1 << sid));

	return 0;
}

int32_t rte_service_get_enabled_on_core(struct rte_service_spec *service,
					uint32_t lcore)
{
	uint32_t enabled;
	int ret = service_update(service, lcore, 0, &enabled);
	if (ret == 0)
		return enabled;
	return -EINVAL;
}

int32_t
rte_service_enable_on_core(struct rte_service_spec *service, uint32_t lcore)
{
	uint32_t on = 1;
	return service_update(service, lcore, &on, 0);
}

int32_t
rte_service_disable_on_core(struct rte_service_spec *service, uint32_t lcore)
{
	uint32_t off = 0;
	return service_update(service, lcore, &off, 0);
}

int32_t rte_service_core_reset_all(void)
{
	/* loop over cores, reset all to mask 0 */
	uint32_t i;
	for (i = 0; i < RTE_MAX_LCORE; i++) {
		cores_state[i].service_mask = 0;
		cores_state[i].is_service_core = 0;
	}

	return 0;
}

int32_t
rte_service_core_add(uint32_t lcore)
{
	if (lcore >= RTE_MAX_LCORE)
		return -EINVAL;
	if (cores_state[lcore].is_service_core)
		return -EALREADY;

	lcore_config[lcore].core_role = ROLE_SERVICE;

	/* TODO: take from EAL by setting ROLE_SERVICE? */
	cores_state[lcore].is_service_core = 1;
	cores_state[lcore].service_mask = 0;

	return 0;
}

int32_t
rte_service_core_del(uint32_t lcore)
{
	if (lcore >= RTE_MAX_LCORE)
		return -EINVAL;

	struct core_state *cs = &cores_state[lcore];
	if (!cs->is_service_core)
		return -EINVAL;

	if (cs->runstate != RTE_SERVICE_RUNSTATE_STOPPED)
		return -EBUSY;

	lcore_config[lcore].core_role = ROLE_RTE;
	cores_state[lcore].is_service_core = 0;
	/* TODO: return to EAL by setting ROLE_RTE? */

	return 0;
}

int32_t
rte_service_core_start(uint32_t lcore)
{
	if (lcore >= RTE_MAX_LCORE)
		return -EINVAL;

	struct core_state *cs = &cores_state[lcore];
	if (!cs->is_service_core)
		return -EINVAL;

	if (cs->runstate == RTE_SERVICE_RUNSTATE_RUNNING)
		return -EALREADY;

	/* set core to run state first, and then launch otherwise it will
	 * return immidiatly as runstate keeps it in the service poll loop
	 */
	cores_state[lcore].runstate = RTE_SERVICE_RUNSTATE_RUNNING;

	int ret = rte_eal_remote_launch(rte_service_runner_func, 0, lcore);
	/* returns -EBUSY if the core is already launched, 0 on success */
	return ret;
}

int32_t
rte_service_core_stop(uint32_t lcore)
{
	if (lcore >= RTE_MAX_LCORE)
		return -EINVAL;

	if (cores_state[lcore].runstate == RTE_SERVICE_RUNSTATE_STOPPED)
		return -EALREADY;

	uint32_t i;
	for (i = 0; i < RTE_SERVICE_NUM_MAX; i++) {
		int32_t enabled = cores_state[i].service_mask & (1 << i);
		int32_t service_running = rte_services[i].runstate !=
						RTE_SERVICE_RUNSTATE_STOPPED;
		int32_t only_core = rte_services[i].num_mapped_cores == 1;

		/* if the core is mapped, and the service is running, and this
		 * is the only core that is mapped, the service would cease to
		 * run if this core stopped, so fail instead.
		 */
		if (enabled && service_running && only_core)
			return -EBUSY;
	}

	cores_state[lcore].runstate = RTE_SERVICE_RUNSTATE_STOPPED;

	return 0;
}

static void
rte_service_dump_one(FILE *f, struct rte_service_spec_impl *s,
		     uint64_t all_cycles, uint32_t reset)
{
	/* avoid divide by zeros */
	if (all_cycles == 0)
		all_cycles = 1;

	int calls = 1;
	if (s->calls != 0)
		calls = s->calls;

	float cycles_pct = (((float)s->cycles_spent) / all_cycles) * 100.f;
	fprintf(f,
			"  %s : %0.1f %%\tcalls %"PRIu64"\tcycles %"PRIu64"\tavg: %"PRIu64"\n",
			s->spec.name, cycles_pct, s->calls, s->cycles_spent,
			s->cycles_spent / calls);

	if (reset) {
		s->cycles_spent = 0;
		s->calls = 0;
	}
}

static void
service_dump_calls_per_core(FILE *f, uint32_t lcore, uint32_t reset)
{
	uint32_t i;
	struct core_state *cs = &cores_state[lcore];

	fprintf(f, "%02d\t", lcore);
	for (i = 0; i < RTE_SERVICE_NUM_MAX; i++) {
		if (!service_valid(i))
			continue;
		fprintf(f, "%04"PRIu64"\t", cs->calls_per_service[i]);
		if (reset)
			cs->calls_per_service[i] = 0;
	}
	fprintf(f, "\n");
}

int32_t rte_service_dump(FILE *f, struct rte_service_spec *service)
{
	uint32_t i;

	uint64_t total_cycles = 0;
	for (i = 0; i < rte_service_count; i++)
		total_cycles += rte_services[i].cycles_spent;

	if (service) {
		struct rte_service_spec_impl *s =
			(struct rte_service_spec_impl *)service;
		fprintf(f, "Service %s Summary\n", s->spec.name);
		uint32_t reset = 0;
		rte_service_dump_one(f, s, total_cycles, reset);
		return 0;
	}

	struct rte_config *cfg = rte_eal_get_configuration();

	fprintf(f, "Services Summary\n");
	for (i = 0; i < rte_service_count; i++) {
		uint32_t reset = 1;
		rte_service_dump_one(f, &rte_services[i], total_cycles, reset);
	}

	fprintf(f, "Service Cores Summary\n");
	for (i = 0; i < RTE_MAX_LCORE; i++) {
		if (cfg->lcore_role[i] != ROLE_SERVICE)
			continue;

		uint32_t reset = 0;
		service_dump_calls_per_core(f, i, reset);
	}

	return 0;
}
