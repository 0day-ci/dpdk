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

#include <rte_common.h>
#include <rte_hexdump.h>
#include <rte_mbuf.h>
#include <rte_malloc.h>
#include <rte_memcpy.h>
#include <rte_cycles.h>

#include <rte_service.h>
#include <rte_service_private.h>

#include "test.h"

/* used as the service core ID */
static uint32_t score_id;
/* used as timestamp to detect if a service core is running */
static uint64_t service_tick;

#define SERVICE_DELAY 1

static int
testsuite_setup(void)
{
	/* assuming lcore 1 is available for service-core testing */
	score_id = 1;
	return TEST_SUCCESS;
}

static void
testsuite_teardown(void)
{
	/* release service cores? */
}

static int32_t dummy_cb(void *args)
{
	RTE_SET_USED(args);
	service_tick++;
	rte_delay_ms(SERVICE_DELAY);
	return 0;
}

/* unregister all services */
static int
dummy_unregister(void)
{
	uint32_t i;
	struct rte_service_spec *dead = (struct rte_service_spec *)0xdead;

	TEST_ASSERT_EQUAL(-EINVAL, rte_service_unregister(0),
			"Unregistered NULL pointer");
	TEST_ASSERT_EQUAL(-EINVAL, rte_service_unregister(dead),
			"Unregistered invalid pointer");

	uint32_t c = rte_service_get_count();
	for (i = 0; i < c; i++) {
		struct rte_service_spec *s = rte_service_get_by_id(i);
		TEST_ASSERT_EQUAL(0, rte_service_unregister(s),
				"Error unregistering a valid service");
	}

	rte_service_core_reset_all();

	return TEST_SUCCESS;
}

/* register a single dummy service */
static int
dummy_register(void)
{
	/* make sure there are no remains from previous tests */
	dummy_unregister();

	struct rte_service_spec service;
	memset(&service, 0, sizeof(struct rte_service_spec));

	TEST_ASSERT_EQUAL(-EINVAL, rte_service_register(&service),
			"Invalid callback");
	service.callback = dummy_cb;

	TEST_ASSERT_EQUAL(-EINVAL, rte_service_register(&service),
			"Invalid name");
	snprintf(service.name, sizeof(service.name), "dummy_service");

	TEST_ASSERT_EQUAL(0, rte_service_register(&service),
			"Failed to register valid service");

	return TEST_SUCCESS;
}

/* start and stop a service */
static int
service_start_stop(void)
{
	struct rte_service_spec *service = rte_service_get_by_id(0);

	TEST_ASSERT_EQUAL(0, rte_service_is_running(service),
			"Error: Service is running - should be stopped");

	TEST_ASSERT_EQUAL(0, rte_service_stop(service),
			"Error: Service stopped returned non-zero");

	TEST_ASSERT_EQUAL(0, rte_service_is_running(service),
			"Error: Service is running - should be stopped");

	TEST_ASSERT_EQUAL(0, rte_service_start(service),
			"Error: Service start returned non-zero");

	TEST_ASSERT_EQUAL(1, rte_service_is_running(service),
			"Error: Service is not running");

	return dummy_unregister();
}

/* enable and disable a core for a service */
static int
service_core_enable_disable(void)
{
	struct rte_service_spec *s = rte_service_get_by_id(0);

	/* expected failure cases */
	TEST_ASSERT_EQUAL(-EINVAL, rte_service_enable_on_core(s, 100000),
			"Enable on invalid core did not fail");
	TEST_ASSERT_EQUAL(-EINVAL, rte_service_disable_on_core(s, 100000),
			"Dispable on invalid core did not fail");

	/* add service core to allow enabling */
	TEST_ASSERT_EQUAL(0, rte_service_core_add(score_id),
			"Add service core failed when not in use before");

	/* valid enable */
	TEST_ASSERT_EQUAL(0, rte_service_enable_on_core(s, score_id),
			"Enabling valid service and core failed");
	TEST_ASSERT_EQUAL(1, rte_service_get_enabled_on_core(s, score_id),
			"Enabled core returned not-enabled");

	/* valid disable */
	TEST_ASSERT_EQUAL(0, rte_service_disable_on_core(s, score_id),
			"Disabling valid service and core failed");
	TEST_ASSERT_EQUAL(0, rte_service_get_enabled_on_core(s, score_id),
			"Disabled core returned enabled");

	return dummy_unregister();
}

static int
service_core_running_check(void)
{
	uint64_t tick = service_tick;
	rte_delay_ms(SERVICE_DELAY * 10);
	/* if (tick != service_tick) we know the core as polled the service */
	return tick != service_tick;
}

static int
service_core_add_del(void)
{
	/* check initial count */
	TEST_ASSERT_EQUAL(0, rte_service_core_count(),
			"Service core count has value before adding a core");

	/* check service core add */
	TEST_ASSERT_EQUAL(0, rte_service_core_add(score_id),
			"Add service core failed when not in use before");
	TEST_ASSERT_EQUAL(-EALREADY, rte_service_core_add(score_id),
			"Add service core failed to refuse in-use lcore");

	/* check count */
	TEST_ASSERT_EQUAL(1, rte_service_core_count(),
			"Service core count not equal to one");

	/* retrieve core list, checking lcore ids */
	const uint32_t size = 4;
	uint32_t service_core_ids[size];
	int32_t n = rte_service_core_list(service_core_ids, size);
	TEST_ASSERT_EQUAL(1, n, "Service core list return should equal 1");
	TEST_ASSERT_EQUAL(score_id, service_core_ids[0],
				"Service core list lcore must equal score_id");

	/* recheck count, add more cores, and check count */
	TEST_ASSERT_EQUAL(1, rte_service_core_count(),
			"Service core count not equal to one");
	TEST_ASSERT_EQUAL(0, rte_service_core_add(2),
			"Service core add did not return zero");
	TEST_ASSERT_EQUAL(0, rte_service_core_add(3),
			"Service core add did not return zero");
	TEST_ASSERT_EQUAL(3, rte_service_core_count(),
			"Service core count not equal to three");

	/* check longer service core list */
	n = rte_service_core_list(service_core_ids, size);
	TEST_ASSERT_EQUAL(3, n, "Service core list return should equal 3");
	TEST_ASSERT_EQUAL(score_id, service_core_ids[0],
				"Service core list[0] lcore must equal 1");
	TEST_ASSERT_EQUAL(2, service_core_ids[1],
				"Service core list[1] lcore must equal 2");
	TEST_ASSERT_EQUAL(3, service_core_ids[2],
				"Service core list[2] lcore must equal 3");

	/* recheck count, remove lcores, check remaining lcore_id is correct */
	TEST_ASSERT_EQUAL(3, rte_service_core_count(),
			"Service core count not equal to three");
	TEST_ASSERT_EQUAL(0, rte_service_core_del(1),
			"Service core add did not return zero");
	TEST_ASSERT_EQUAL(0, rte_service_core_del(2),
			"Service core add did not return zero");
	TEST_ASSERT_EQUAL(1, rte_service_core_count(),
			"Service core count not equal to one");
	n = rte_service_core_list(service_core_ids, size);
	TEST_ASSERT_EQUAL(1, n, "Service core list return should equal one");
	TEST_ASSERT_EQUAL(3, service_core_ids[0],
				"Service core list[0] lcore must equal three");

	return dummy_unregister();
}

/* start and stop a service core - ensuring it goes back to sleep */
static int
service_core_start_stop(void)
{
	/* start service core and service, create mapping so tick() runs */
	struct rte_service_spec *s = rte_service_get_by_id(0);
	TEST_ASSERT_EQUAL(0, rte_service_start(s),
			"Starting valid service failed");
	/* TODO: should this work? Core isn't enabled as service core */
	TEST_ASSERT_EQUAL(-EINVAL, rte_service_enable_on_core(s, score_id),
			"Enabling valid service on invalid core must fail");

	/* core start */
	TEST_ASSERT_EQUAL(-EINVAL, rte_service_core_start(score_id),
			"Service core start without add should return EINVAL");
	TEST_ASSERT_EQUAL(0, rte_service_core_add(score_id),
			"Service core add did not return zero");
	TEST_ASSERT_EQUAL(0, rte_service_enable_on_core(s, score_id),
			"Enabling valid service on valid core failed");
	TEST_ASSERT_EQUAL(0, rte_service_core_start(score_id),
			"Service core start after add failed");
	TEST_ASSERT_EQUAL(-EALREADY, rte_service_core_start(score_id),
			"Service core expected as running but was stopped");

	/* ensures core really is running the service function */
	TEST_ASSERT_EQUAL(1, service_core_running_check(),
			"Service core expected to poll service but it didn't");

	/* core stop */
	TEST_ASSERT_EQUAL(-EINVAL, rte_service_core_stop(100000),
			"Invalid Service core stop should return -EINVAL");
	TEST_ASSERT_EQUAL(0, rte_service_core_stop(score_id),
			"Service core stop expected to return 0");
	TEST_ASSERT_EQUAL(-EALREADY, rte_service_core_stop(score_id),
			"Already stopped service core should return -EALREADY");

	/* ensure service is not longer running */
	TEST_ASSERT_EQUAL(0, service_core_running_check(),
			"Service core expected to poll service but it didn't");

	return dummy_unregister();
}

static struct unit_test_suite service_tests  = {
	.suite_name = "service core test suite",
	.setup = testsuite_setup,
	.teardown = testsuite_teardown,
	.unit_test_cases = {
		TEST_CASE_ST(dummy_register, NULL, dummy_unregister),
		TEST_CASE_ST(dummy_register, NULL, service_start_stop),
		TEST_CASE_ST(dummy_register, NULL, service_core_add_del),
		TEST_CASE_ST(dummy_register, NULL, service_core_start_stop),
		TEST_CASE_ST(dummy_register, NULL, service_core_enable_disable),
		TEST_CASES_END() /**< NULL terminate unit test array */
	}
};

static int
test_service_common(void)
{
	return unit_test_suite_runner(&service_tests);
}

REGISTER_TEST_COMMAND(service_autotest, test_service_common);
