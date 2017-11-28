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

#include <string.h>

#include <rte_eventdev.h>
#include <rte_dev.h>
#include <rte_bus_vdev.h>
#include <rte_event_timer_adapter.h>
#include <rte_mempool.h>
#include <rte_errno.h>
#include <rte_service_component.h>

#include "test.h"

/* Example from RFC */
#define NB_TEST_EVENT_TIMERS 40000

static int evdev;
//struct rte_event_timer_adapter *g_adapter;
struct rte_event_timer *g_evtimer;
struct rte_mempool *g_event_timer_pool;

static inline void
devconf_set_default_sane_values(struct rte_event_dev_config *dev_conf,
			struct rte_event_dev_info *info)
{
	memset(dev_conf, 0, sizeof(struct rte_event_dev_config));
	dev_conf->dequeue_timeout_ns = info->min_dequeue_timeout_ns;
	/* Leave a port for the adapter to allocate */
	dev_conf->nb_event_ports = info->max_event_ports - 1;
	dev_conf->nb_event_queues = info->max_event_queues;
	dev_conf->nb_event_queue_flows = info->max_event_queue_flows;
	dev_conf->nb_event_port_dequeue_depth =
			info->max_event_port_dequeue_depth;
	dev_conf->nb_event_port_enqueue_depth =
			info->max_event_port_enqueue_depth;
	dev_conf->nb_event_port_enqueue_depth =
			info->max_event_port_enqueue_depth;
	dev_conf->nb_events_limit =
			info->max_num_events;
}

static int
configure_event_dev(void)
{
	struct rte_event_dev_config devconf;
	int ret;
	const char *eventdev_name = "event_sw0";
	struct rte_event_dev_info info;
	int i;

	evdev = rte_event_dev_get_dev_id(eventdev_name);
	if (evdev < 0) {
		if (rte_vdev_init(eventdev_name, NULL) < 0) {
			printf("Error creating eventdev\n");
			return TEST_FAILED;
		}
		evdev = rte_event_dev_get_dev_id(eventdev_name);
		if (evdev < 0) {
			printf("Error finding newly created eventdev\n");
			return TEST_FAILED;
		}
	}

	ret = rte_event_dev_info_get(evdev, &info);
	TEST_ASSERT_SUCCESS(ret, "Failed to get event dev info");

	devconf_set_default_sane_values(&devconf, &info);

	ret = rte_event_dev_configure(evdev, &devconf);
	TEST_ASSERT_SUCCESS(ret, "Failed to configure eventdev");

	/* Map the event_sw0 service to a service core */
	ret = rte_service_start_with_defaults();
	TEST_ASSERT_SUCCESS(ret, "Failed to start sw_evdev service");

	/* Set up event queues */
	uint32_t queue_count;
	TEST_ASSERT_SUCCESS(rte_event_dev_attr_get(evdev,
			    RTE_EVENT_DEV_ATTR_QUEUE_COUNT, &queue_count),
			    "Queue count get failed");

	for (i = 0; i < (int)queue_count; i++) {
		ret = rte_event_queue_setup(evdev, i, NULL);
		TEST_ASSERT_SUCCESS(ret, "Failed to setup queue=%d", i);
	}

	/* Set up event ports */
	uint32_t port_count;
	TEST_ASSERT_SUCCESS(rte_event_dev_attr_get(evdev,
			    RTE_EVENT_DEV_ATTR_PORT_COUNT,
			    &port_count), "Port count get failed");

	for (i = 0; i < (int)port_count; i++) {
		ret = rte_event_port_setup(evdev, i, NULL);
		TEST_ASSERT_SUCCESS(ret, "Failed to setup port=%d", i);
		/* Link each queues to all ports */
		ret = rte_event_port_link(evdev, i, NULL, NULL, 0);
		TEST_ASSERT(ret >= 0, "Failed to link all queues port=%d", i);
	}

	/* Start the event device */
	ret = rte_event_dev_start(evdev);
	TEST_ASSERT_SUCCESS(ret, "Failed to start device");

	return TEST_SUCCESS;
}

static int
testsuite_setup(void)
{
	int ret;

	/* Setup and start event device. */
	ret = configure_event_dev();
	if (ret) {
		printf("Failed to configure event dev\n");
		return TEST_FAILED;
	}

	/* Create a mempool of event timers. */
	g_event_timer_pool = rte_mempool_create("event_timer_mempool",
						NB_TEST_EVENT_TIMERS,
						sizeof(struct rte_event_timer),
						0,
						0,
						NULL,
						NULL,
						NULL,
						NULL,
						rte_socket_id(),
						0);
	if (g_event_timer_pool == NULL) {
		/* Failed to create event timer mempool. */
		printf("Failed to configure event timer mempool: %s\n",
		       rte_strerror(rte_errno));
		return TEST_FAILED;
	}

	return TEST_SUCCESS;
}

static void
testsuite_teardown(void)
{
	/* TODO: tear down adapter and evdev */

	rte_mempool_free(g_event_timer_pool);
}

#define NSECPERSEC 1E9  // No of ns for 1 sec

static int
adapter_create_free(void)
{
	int ret;
	int adapter_id = 0;
	struct rte_event_timer_adapter *adapter;

	struct rte_event_timer_adapter_conf conf = {
		.event_dev_id = evdev,
		.timer_adapter_id = adapter_id,
		.clk_src = RTE_EVENT_TIMER_ADAPTER_CPU_CLK,
		.timer_tick_ns = NSECPERSEC / 10,  // 100 milliseconds
		.max_tmo_ns = 180 * NSECPERSEC,  // 2 minutes
		.nb_timers = NB_TEST_EVENT_TIMERS,
		.flags = 0,
	};

	adapter = rte_event_timer_adapter_create(&conf);
	if (adapter == NULL) {
		printf("Failed to create adapter\n");
		return TEST_FAILED;
	}

	/* Move to separate tests later; just verify plugin connections for
	 * now
	 */

	struct rte_event_timer_adapter_info adapter_info;
	ret = rte_event_timer_adapter_get_info(adapter, &adapter_info);
	if (ret < 0)
		return TEST_FAILED;

	ret = rte_event_timer_adapter_start(adapter);
	if (ret < 0)
		return TEST_FAILED;

	ret = rte_event_timer_adapter_stop(adapter);
	if (ret < 0)
		return TEST_FAILED;

	ret = rte_event_timer_adapter_free(adapter);
	if (ret) {
		printf("Failed to free adapter\n");
		return TEST_FAILED;
	}

	return TEST_SUCCESS;
}

static struct unit_test_suite adapter_tests  = {
	.suite_name = "event timer adapter test suite",
	.setup = testsuite_setup,
	.teardown = testsuite_teardown,
	.unit_test_cases = {
		TEST_CASE(adapter_create_free),
		TEST_CASES_END() /**< NULL terminate unit test array */
	}
};

static int
test_event_timer_adapter_common(void)
{
	return unit_test_suite_runner(&adapter_tests);
}

REGISTER_TEST_COMMAND(event_timer_adapter_autotest,
		      test_event_timer_adapter_common);
