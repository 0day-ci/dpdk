#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <stdio.h>

#include <rte_common.h>
#include <rte_eal.h>

int main(int argc, char *argv[])
{
	/* Initialise DPDK EAL */
	int ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Invalid EAL arguments\n");
	argc -= ret;
	argv += ret;

	testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}	
