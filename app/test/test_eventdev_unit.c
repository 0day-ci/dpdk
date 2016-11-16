/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Intel Corporation. All rights reserved.
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
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include "test.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>

#include <rte_eventdev.h>
#include <rte_lcore.h>
#include <rte_mbuf.h>

typedef enum eventdev_api_ut_ids_s {
	EVENTDEV_API_UT_001 = 1,
	EVENTDEV_API_UT_002,
	EVENTDEV_API_UT_003,
	EVENTDEV_API_UT_004,
	EVENTDEV_API_UT_005,
	EVENTDEV_API_UT_006,
	EVENTDEV_API_UT_007,
	EVENTDEV_API_UT_008,
	EVENTDEV_API_UT_009,
	EVENTDEV_API_UT_010,
	EVENTDEV_API_UT_011,
	EVENTDEV_API_UT_012,
	EVENTDEV_API_UT_013,
	EVENTDEV_API_UT_014,
	EVENTDEV_API_UT_015,
	EVENTDEV_API_UT_016,
	EVENTDEV_API_UT_017,
	EVENTDEV_API_UT_MAX
} eventdev_api_ut_ids_t;

typedef enum eventdev_tc_status_s {
	TC_FAILED,
	TC_PASSED
} eventdev_tc_status_t;

typedef struct eventdev_api_ut_status_s {
	bool executed;
	eventdev_tc_status_t status;
} eventdev_api_ut_status_t;

eventdev_api_ut_status_t api_ut_status[EVENTDEV_API_UT_MAX];

#define CONFIG_NB_EVENT_QUEUES 2
#define CONFIG_NB_EVENT_PORTS 2
#define CONFIG_NB_EVENT_LIMIT 128

uint8_t queues[CONFIG_NB_EVENT_QUEUES];
uint8_t ports[CONFIG_NB_EVENT_PORTS];

/* FIXME: Check that dependent tests have executed */

static int test_EVENTDEV_API_UT_001_rte_event_dev_count(void)
{
	uint8_t count = rte_event_dev_count();

	if (count == 1) {
		api_ut_status[EVENTDEV_API_UT_001].status = TC_PASSED;
		return 0;
	} else {
		api_ut_status[EVENTDEV_API_UT_001].status = TC_FAILED;
		return 1;
	}
}

static int test_EVENTDEV_API_UT_002_rte_event_dev_get_dev_id(void)
{
	int8_t id;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	if (id < 0) {
		api_ut_status[EVENTDEV_API_UT_002].status = TC_FAILED;
		return 1;
	}

	id = rte_event_dev_get_dev_id("evdev_abcd123");

	if (id >= 0) {
		api_ut_status[EVENTDEV_API_UT_002].status = TC_FAILED;
		return 1;
	}

	api_ut_status[EVENTDEV_API_UT_002].status = TC_PASSED;
	return 0;
}

static int test_EVENTDEV_API_UT_003_rte_event_dev_info_get(void)
{
	struct rte_event_dev_info info;
	int8_t id;
	int ret;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	ret = rte_event_dev_info_get(id, &info);
	if (ret)
		goto fail;

	if (strncmp(info.driver_name, "evdev_sw", sizeof("evdev_sw")) != 0)
		goto fail;

	/* FIXME: Add checks for remaining fields */

	api_ut_status[EVENTDEV_API_UT_003].status = TC_PASSED;
	return 0;

fail:
	api_ut_status[EVENTDEV_API_UT_003].status = TC_FAILED;
	return 1;
}

static int test_EVENTDEV_API_UT_004_rte_event_dev_configure(void)
{
	struct rte_event_dev_config config;
	int8_t id;
	int ret;

	api_ut_status[EVENTDEV_API_UT_004].executed = true;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	config.nb_event_queues = CONFIG_NB_EVENT_QUEUES; /* FIXME: Test max */
	config.nb_event_ports = CONFIG_NB_EVENT_PORTS; /* FIXME: Test max */
	config.nb_events_limit = CONFIG_NB_EVENT_LIMIT; /* FIXME: Test max */
	config.dequeue_wait_ns = 0; /* FIXME: Test max */

	ret = rte_event_dev_configure(id, &config);
	if (ret)
		goto fail;

	api_ut_status[EVENTDEV_API_UT_004].status = TC_PASSED;
	return 0;

fail:
	api_ut_status[EVENTDEV_API_UT_004].status = TC_FAILED;
	return 1;
}

static int test_EVENTDEV_API_UT_005_rte_event_queue_count_pre(void)
{
	int8_t id;
	uint8_t count;

	if (!api_ut_status[EVENTDEV_API_UT_004].executed)
		test_EVENTDEV_API_UT_004_rte_event_dev_configure();
	if (api_ut_status[EVENTDEV_API_UT_004].status == TC_FAILED)
		goto fail;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	count = rte_event_queue_count(id);
	if (count != CONFIG_NB_EVENT_QUEUES)
		goto fail;

	api_ut_status[EVENTDEV_API_UT_005].status = TC_PASSED;
	return 0;

fail:
	api_ut_status[EVENTDEV_API_UT_005].status = TC_FAILED;
	return 1;
}

static int test_EVENTDEV_API_UT_006_rte_event_queue_setup(void)
{
	struct rte_event_queue_conf config;
	int8_t id;
	int ret;

	api_ut_status[EVENTDEV_API_UT_006].executed = true;

	if (!api_ut_status[EVENTDEV_API_UT_004].executed)
		test_EVENTDEV_API_UT_004_rte_event_dev_configure();
	if (api_ut_status[EVENTDEV_API_UT_004].status == TC_FAILED)
		goto fail;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	config.event_queue_cfg = 0;
	config.priority = 0;

	queues[0] = 0;

	ret = rte_event_queue_setup(id, queues[0], &config);
	if (ret < 0)
		goto fail;

	config.event_queue_cfg = RTE_EVENT_QUEUE_CFG_SINGLE_CONSUMER;
	config.priority = 0;

	queues[1] = 1;

	ret = rte_event_queue_setup(id, queues[1], &config);
	if (ret < 0)
		goto fail;

	api_ut_status[EVENTDEV_API_UT_006].status = TC_PASSED;
	return 0;

fail:
	api_ut_status[EVENTDEV_API_UT_006].status = TC_FAILED;
	return 1;
}

static int test_EVENTDEV_API_UT_007_rte_event_queue_count_post(void)
{
	int8_t id;
	uint8_t count;

	if (!api_ut_status[EVENTDEV_API_UT_004].executed)
		test_EVENTDEV_API_UT_004_rte_event_dev_configure();
	if (api_ut_status[EVENTDEV_API_UT_004].status == TC_FAILED)
		goto fail;

	if (!api_ut_status[EVENTDEV_API_UT_006].executed)
		test_EVENTDEV_API_UT_006_rte_event_queue_setup();
	if (api_ut_status[EVENTDEV_API_UT_006].status == TC_FAILED)
		goto fail;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	count = rte_event_queue_count(id);
	if (count != CONFIG_NB_EVENT_QUEUES)
		goto fail;

	api_ut_status[EVENTDEV_API_UT_007].status = TC_PASSED;
	return 0;

fail:
	api_ut_status[EVENTDEV_API_UT_007].status = TC_FAILED;
	return 1;
}

static int test_EVENTDEV_API_UT_008_rte_event_port_count_pre(void)
{
	int8_t id;
	uint8_t count;

	if (!api_ut_status[EVENTDEV_API_UT_004].executed)
		test_EVENTDEV_API_UT_004_rte_event_dev_configure();
	if (api_ut_status[EVENTDEV_API_UT_004].status == TC_FAILED)
		goto fail;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	count = rte_event_port_count(id);
	if (count != CONFIG_NB_EVENT_PORTS)
		goto fail;

	api_ut_status[EVENTDEV_API_UT_008].status = TC_PASSED;
	return 0;

fail:
	api_ut_status[EVENTDEV_API_UT_008].status = TC_FAILED;
	return 1;
}

static int test_EVENTDEV_API_UT_009_rte_event_port_setup(void)
{
	struct rte_event_port_conf config;
	int8_t id;
	int ret;

	if (!api_ut_status[EVENTDEV_API_UT_004].executed)
		test_EVENTDEV_API_UT_004_rte_event_dev_configure();
	if (api_ut_status[EVENTDEV_API_UT_004].status == TC_FAILED)
		goto fail;

	api_ut_status[EVENTDEV_API_UT_009].executed = true;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	config.dequeue_queue_depth = 4;
	config.enqueue_queue_depth = 4;
	config.new_event_threshold = 64;

	ports[0] = 0;

	ret = rte_event_port_setup(id, ports[0], &config);
	if (ret < 0)
		goto fail;

	config.dequeue_queue_depth = 4;
	config.enqueue_queue_depth = 4;
	config.new_event_threshold = 64;

	ports[1] = 1;

	ret = rte_event_port_setup(id, ports[1], &config);
	if (ret < 0)
		goto fail;

	api_ut_status[EVENTDEV_API_UT_009].status = TC_PASSED;
	return 0;

fail:
	api_ut_status[EVENTDEV_API_UT_009].status = TC_FAILED;
	return 1;
}

static int test_EVENTDEV_API_UT_010_rte_event_port_count_post(void)
{
	int8_t id;
	uint8_t count;

	if (!api_ut_status[EVENTDEV_API_UT_004].executed)
		test_EVENTDEV_API_UT_004_rte_event_dev_configure();
	if (api_ut_status[EVENTDEV_API_UT_004].status == TC_FAILED)
		goto fail;

	if (!api_ut_status[EVENTDEV_API_UT_009].executed)
		test_EVENTDEV_API_UT_009_rte_event_port_setup();
	if (api_ut_status[EVENTDEV_API_UT_009].status == TC_FAILED)
		goto fail;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	count = rte_event_port_count(id);
	if (count != CONFIG_NB_EVENT_PORTS)
		goto fail;

	api_ut_status[EVENTDEV_API_UT_010].status = TC_PASSED;
	return 0;

fail:
	api_ut_status[EVENTDEV_API_UT_010].status = TC_FAILED;
	return 1;
}

static int test_EVENTDEV_API_UT_011_rte_event_dev_start(void)
{
	int8_t id;
	int ret;

	if (!api_ut_status[EVENTDEV_API_UT_004].executed)
		test_EVENTDEV_API_UT_004_rte_event_dev_configure();
	if (api_ut_status[EVENTDEV_API_UT_004].status == TC_FAILED)
		goto fail;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	ret = rte_event_dev_start(id);
	if (ret != 0)
		goto fail;

	api_ut_status[EVENTDEV_API_UT_011].status = TC_PASSED;
	return 0;

fail:
	api_ut_status[EVENTDEV_API_UT_011].status = TC_FAILED;
	return 1;
}

static int test_EVENTDEV_API_UT_012_rte_event_port_link(void)
{
	struct rte_event_queue_link link;
	int8_t id;
	int ret;

	if (!api_ut_status[EVENTDEV_API_UT_004].executed)
		test_EVENTDEV_API_UT_004_rte_event_dev_configure();
	if (api_ut_status[EVENTDEV_API_UT_004].status == TC_FAILED)
		goto fail;

	if (!api_ut_status[EVENTDEV_API_UT_006].executed)
		test_EVENTDEV_API_UT_006_rte_event_queue_setup();
	if (api_ut_status[EVENTDEV_API_UT_006].status == TC_FAILED)
		goto fail;

	if (!api_ut_status[EVENTDEV_API_UT_009].executed)
		test_EVENTDEV_API_UT_009_rte_event_port_setup();
	if (api_ut_status[EVENTDEV_API_UT_009].status == TC_FAILED)
		goto fail;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	link.queue_id = queues[0];
	link.priority = 0;

	/* Connect port to previously configured scheduled queue */
	ret = rte_event_port_link(id, ports[0], &link, 1);
	if (ret != 1) {
		printf("%d: failed here\n", __LINE__);
		goto fail;
	}

	/* Check idempotency of re-linking port to queues[0] */
	ret = rte_event_port_link(id, ports[0], &link, 1);
	if (ret != 1) {
		printf("%d: failed here\n", __LINE__);
		goto fail;
	}

	link.queue_id = queues[1];
	link.priority = 0;

	/* Attempt to connect to FIFO queue as well */
	ret = rte_event_port_link(id, ports[0], &link, 1);
	if (ret == 1) {
		printf("%d: failed here\n", __LINE__);
		goto fail;
	}

	link.queue_id = queues[1];
	link.priority = 0;

	/* Connect port to previously configured FIFO queue */
	ret = rte_event_port_link(id, ports[1], &link, 1);
	if (ret != 1) {
		printf("%d: failed here\n", __LINE__);
		goto fail;
	}

	link.queue_id = queues[0];
	link.priority = 0;

	/* Attempt to connect to scheduled queue as well */
	ret = rte_event_port_link(id, ports[1], &link, 1);
	if (ret == 1) {
		printf("%d: failed here\n", __LINE__);
		goto fail;
	}

	/* link to 2nd queue, enabling start() to pass later */
	link.queue_id = queues[1];
	link.priority = 0;
	ret = rte_event_port_link(id, ports[1], &link, 1);
	if (ret == 1) {
		printf("%d: failed here\n", __LINE__);
		goto fail;
	}

	api_ut_status[EVENTDEV_API_UT_012].status = TC_PASSED;
	return 0;

fail:
	api_ut_status[EVENTDEV_API_UT_012].status = TC_FAILED;
	return 1;
}

static int test_EVENTDEV_API_UT_014_rte_event_dev_stop(void)
{
	int8_t id;

	if (!api_ut_status[EVENTDEV_API_UT_004].executed)
		test_EVENTDEV_API_UT_004_rte_event_dev_configure();
	if (api_ut_status[EVENTDEV_API_UT_004].status == TC_FAILED)
		return 1;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	rte_event_dev_stop(id);

	api_ut_status[EVENTDEV_API_UT_014].status = TC_PASSED;
	return 0;
}

static int test_EVENTDEV_API_UT_015_rte_event_dev_close(void)
{
	int8_t id;
	int ret;

	if (!api_ut_status[EVENTDEV_API_UT_004].executed)
		test_EVENTDEV_API_UT_004_rte_event_dev_configure();
	if (api_ut_status[EVENTDEV_API_UT_004].status == TC_FAILED)
		goto fail;

	id = rte_event_dev_get_dev_id("evdev_sw0");

	ret = rte_event_dev_close(id);
	if (ret != 0)
		goto fail;

	api_ut_status[EVENTDEV_API_UT_015].status = TC_PASSED;
	return 0;

fail:
	api_ut_status[EVENTDEV_API_UT_015].status = TC_FAILED;
	return 1;
}

static int
test_setup(void)
{
	return 0;
}

static struct unit_test_suite eventdev_test_suite  = {
	.setup = test_setup,
	.suite_name = "Eventdev Test Suite",
	.unit_test_cases = {
		/* device aquisition and config */
		TEST_CASE(test_EVENTDEV_API_UT_001_rte_event_dev_count),
		TEST_CASE(test_EVENTDEV_API_UT_002_rte_event_dev_get_dev_id),
		TEST_CASE(test_EVENTDEV_API_UT_003_rte_event_dev_info_get),
		TEST_CASE(test_EVENTDEV_API_UT_004_rte_event_dev_configure),
		/* queue config */
		TEST_CASE(test_EVENTDEV_API_UT_005_rte_event_queue_count_pre),
		TEST_CASE(test_EVENTDEV_API_UT_006_rte_event_queue_setup),
		TEST_CASE(test_EVENTDEV_API_UT_007_rte_event_queue_count_post),
		/* port config */
		TEST_CASE(test_EVENTDEV_API_UT_008_rte_event_port_count_pre),
		TEST_CASE(test_EVENTDEV_API_UT_009_rte_event_port_setup),
		TEST_CASE(test_EVENTDEV_API_UT_010_rte_event_port_count_post),
		TEST_CASE(test_EVENTDEV_API_UT_012_rte_event_port_link),
		TEST_CASE(test_EVENTDEV_API_UT_011_rte_event_dev_start),
		/* device cleanup */
		TEST_CASE(test_EVENTDEV_API_UT_014_rte_event_dev_stop),
		TEST_CASE(test_EVENTDEV_API_UT_015_rte_event_dev_close),
		TEST_CASES_END()
	}
};

static int
test_eventdev_unit(void)
{
	return unit_test_suite_runner(&eventdev_test_suite);
}

REGISTER_TEST_COMMAND(eventdev_unit_autotest, test_eventdev_unit);
