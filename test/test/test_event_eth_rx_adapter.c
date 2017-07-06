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
#include <rte_common.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_ethdev.h>
#include <rte_eventdev.h>

#include <rte_event_eth_rx_adapter.h>

#include "test.h"

struct rte_mempool *mp;

static inline int
port_init(uint8_t port, struct rte_mempool *mp)
{
	static const struct rte_eth_conf port_conf_default = {
		.rxmode = {
			.mq_mode = ETH_MQ_RX_RSS,
			.max_rx_pkt_len = ETHER_MAX_LEN
		},
		.rx_adv_conf = {
			.rss_conf = {
				.rss_hf = ETH_RSS_IP |
					  ETH_RSS_TCP |
					  ETH_RSS_UDP,
			}
		}
	};
	const uint16_t rx_rings = 1, tx_rings = 1;
	const uint16_t rx_ring_size = 512, tx_ring_size = 512;
	struct rte_eth_conf port_conf = port_conf_default;
	int retval;
	uint16_t q;

	if (port >= rte_eth_dev_count())
		return -1;

	/* Configure the Ethernet device. */
	retval = rte_eth_dev_configure(port, rx_rings, tx_rings, &port_conf);
	if (retval != 0)
		return retval;

	/* Allocate and set up 1 RX queue per Ethernet port. */
	for (q = 0; q < rx_rings; q++) {
		retval = rte_eth_rx_queue_setup(port, q, rx_ring_size,
				rte_eth_dev_socket_id(port), NULL, mp);
		if (retval < 0)
			return retval;
	}

	/* Allocate and set up 1 TX queue per Ethernet port. */
	for (q = 0; q < tx_rings; q++) {
		retval = rte_eth_tx_queue_setup(port, q, tx_ring_size,
				rte_eth_dev_socket_id(port), NULL);
		if (retval < 0)
			return retval;
	}

	/* Start the Ethernet port. */
	retval = rte_eth_dev_start(port);
	if (retval < 0)
		return retval;

	/* Display the port MAC address. */
	struct ether_addr addr;
	rte_eth_macaddr_get(port, &addr);
	printf("Port %u MAC: %02" PRIx8 " %02" PRIx8 " %02" PRIx8
			   " %02" PRIx8 " %02" PRIx8 " %02" PRIx8 "\n",
			(unsigned int)port,
			addr.addr_bytes[0], addr.addr_bytes[1],
			addr.addr_bytes[2], addr.addr_bytes[3],
			addr.addr_bytes[4], addr.addr_bytes[5]);

	/* Enable RX in promiscuous mode for the Ethernet device. */
	rte_eth_promiscuous_enable(port);

	return 0;
}

static int
init_ports(int num_ports)
{
	uint8_t portid;

	mp = rte_pktmbuf_pool_create("packet_pool",
			/* mbufs */ 16384 * num_ports,
			/* cache_size */ 512,
			/* priv_size*/ 0,
			/* data_room_size */ RTE_MBUF_DEFAULT_BUF_SIZE,
			rte_socket_id());

	for (portid = 0; portid < num_ports; portid++)
		if (port_init(portid, mp) != 0)
			rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu8 "\n",
				portid);

	return 0;
}

static int
testsuite_setup(void)
{
	int err;
	err = init_ports(rte_eth_dev_count());
	TEST_ASSERT(err == 0, "Port initialization failed err %d\n", err);
	err = rte_event_eth_rx_adapter_init();
	TEST_ASSERT(err == 0, "Event adapter initalization failed err %d\n", err);
	return TEST_SUCCESS;
}

static void
testsuite_teardown(void)
{
	uint32_t i;
	rte_mempool_free(mp);
	for (i = 0; i < rte_eth_dev_count(); i++)
		rte_eth_dev_stop(i);
}

static int adapter_create(void)
{
	struct rte_event_eth_rx_adapter_conf config = {
		.eventdev_id = 0,
		.rx_event_port_id = 0,
		.max_nb_rx = 128,
		.socket_id = 0
	};

	snprintf(config.service_name, RTE_SERVICE_NAME_MAX, "%s", "rx_adapter");
	int err = rte_event_eth_rx_adapter_create(0, &config);
	TEST_ASSERT(err == 0, "Expected 0 got %d", err);

	return err;
}

static void adapter_free(void)
{
	rte_event_eth_rx_adapter_free(0);
}

static int
adapter_create_free(void)
{
	int err;
	struct rte_event_eth_rx_adapter_conf config = {
		.eventdev_id = 0,
		.rx_event_port_id = 0,
		.max_nb_rx = 128,
		.socket_id = 0
	};

	snprintf(config.service_name, RTE_SERVICE_NAME_MAX, "%s", "rx_adapter");
	err = rte_event_eth_rx_adapter_create(0, NULL);
	TEST_ASSERT(err == -EINVAL, "Expected -EINVAL got %d", err);

	err = rte_event_eth_rx_adapter_create(0, &config);
	TEST_ASSERT(err == 0, "Expected 0 got %d", err);

	err = rte_event_eth_rx_adapter_create(0, &config);
	TEST_ASSERT(err == -EEXIST, "Expected -EEXIST %d got %d", -EEXIST, err);

	err = rte_event_eth_rx_adapter_free(0);
	TEST_ASSERT(err == 0, "Expected 0 got %d", err);

	err = rte_event_eth_rx_adapter_free(0);
	TEST_ASSERT(err == -EINVAL, "Expected -EINVAL %d got %d", -EINVAL, err);

	err = rte_event_eth_rx_adapter_free(1);
	TEST_ASSERT(err == -EINVAL, "Expected -EINVAL %d got %d", -EINVAL, err);

	return TEST_SUCCESS;
}

static int
adapter_queue_add_del(void)
{
	int err;
	struct rte_event ev;

	ev.queue_id = 0;
	ev.sched_type = RTE_SCHED_TYPE_ATOMIC;
	ev.priority = 0;

	struct rte_event_eth_rx_adapter_queue_conf queue_config;

	queue_config.ev = ev;
	queue_config.rx_queue_flags = 0;
	queue_config.servicing_weight = 1;

	err = rte_event_eth_rx_adapter_queue_add(0, 0, -1, &queue_config);
	TEST_ASSERT(err == 0, "Expected 0 got %d", err);

	err = rte_event_eth_rx_adapter_queue_add(0, 0, -1, &queue_config);
	TEST_ASSERT(err == -EEXIST, "Expected -EEXIST got %d", err);

	err = rte_event_eth_rx_adapter_queue_add(0, 0, 0, &queue_config);
	TEST_ASSERT(err == -EEXIST, "Expected -EEXIST got %d", err);

	err = rte_event_eth_rx_adapter_queue_del(0, 0, -1);
	TEST_ASSERT(err == 0, "Expected 0 got %d", err);

	err = rte_event_eth_rx_adapter_queue_del(0, 0, -1);
	TEST_ASSERT(err == -EINVAL, "Expected -EINVAL got %d", err);

	return TEST_SUCCESS;
}

static int
adapter_stats(void)
{
	int err;
	struct rte_event_eth_rx_adapter_stats stats;

	err = rte_event_eth_rx_adapter_stats_get(0, NULL);
	TEST_ASSERT(err == -EINVAL, "Expected -EINVAL got %d", err);

	err = rte_event_eth_rx_adapter_stats_get(0, &stats);
	TEST_ASSERT(err == 0, "Expected 0 got %d", err);

	err = rte_event_eth_rx_adapter_stats_get(1, &stats);
	TEST_ASSERT(err == -EINVAL, "Expected -EINVAL got %d", err);

	return TEST_SUCCESS;
}

static struct unit_test_suite service_tests  = {
	.suite_name = "rx event eth adapter test suite",
	.setup = testsuite_setup,
	.teardown = testsuite_teardown,
	.unit_test_cases = {
		TEST_CASE_ST(NULL, NULL, adapter_create_free),
		TEST_CASE_ST(adapter_create, adapter_free, adapter_queue_add_del),
		TEST_CASE_ST(adapter_create, adapter_free, adapter_stats),
		TEST_CASES_END() /**< NULL terminate unit test array */
	}
};

static int
test_event_eth_rx_adapter_common(void)
{
	return unit_test_suite_runner(&service_tests);
}

REGISTER_TEST_COMMAND(event_eth_rx_adapter_autotest, test_event_eth_rx_adapter_common);
