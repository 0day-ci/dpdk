/*
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

#ifndef _RTE_SERVICE_H_
#define _RTE_SERVICE_H_

/**
 * @file
 *
 * Service functions
 *
 * The service functionality provided by this header allows a DPDK component
 * to indicate that it requires a function call in order for it to perform
 * its processing.
 *
 * An example usage of this functionality would be a component that registers
 * a service to perform a particular packet processing duty: for example the
 * eventdev software PMD. At startup the application requests all services
 * that have been registered, and the app decides how many cores will run the
 * required services. The EAL removes these number of cores from the available
 * runtime cores, and dedicates them to performing service-core workloads. The
 * application has access to the remaining lcores as normal.
 *
 * An example of the service core infrastructure with an application
 * configuring a single service (the eventdev sw PMD), and dedicating one core
 * exclusively to run the service would interact with the API as follows:
 *
 * 1. Eventdev SW PMD calls *rte_eal_service_register*, indicating that it
 *    requires an lcore to call a function in order to operate. EAL registers
 *    this service, but performs no other actions yet.
 *
 * 2. Application calls *rte_eal_service_get*, allowing EAL to provide it
 *    with an array of *rte_service_config* structures. These structures
 *    provide the application with the name of the service, along with
 *    metadata provided by the service when it was registered.
 *
 * 3. The application logic iterates over the services that require running,
 *    and decides to run the eventdev sw PMD service using one lcore.
 *
 * 4. The application calls *rte_eal_service_use_lcore* multiple times, once
 *    for each lcore that should be used as a service core. These cores are
 *    removed from the application usage, and EAL will refuse to launch
 *    user-specified functions on these cores.
 *
 * 5. The application calls *rte_eal_service_set_coremask* to set the coremask
 *    for the service. Note that EAL is already aware of ALL lcores that will
 *    be used for service-core purposes (see step 4 above) which allows EAL to
 *    error-check the coremask here, and ensure at least one core is going to
 *    be running the service.
 *
 * 6. The application now calls remote_launch() as usual, and the application
 *    can perform its processing as required without manually handling the
 *    partitioning of lcore resources for DPDK functionality.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define RTE_SERVICE_NAMESIZE 32

/**
 * Signature of callback back function to run a service.
 */
typedef void (*rte_eal_service_func)(void *args);

struct rte_service_config {
	/* name of the service */
	char name[RTE_SERVICE_NAMESIZE];
	/* cores that run this service */
	uint64_t coremask;
	/* when set, multiple lcores can run this service simultaneously without
	 * the need for software atomics to ensure that two cores do not
	 * attempt to run the service at the same time.
	 */
	uint8_t multithread_capable;
};

/** @internal - this will not be visible to application, defined in a seperate
 * rte_eal_service_impl.h header. The application does not need to be exposed
 * to the actual function pointers - so hide them. */
struct rte_service {
	/* callback to be called to run the service */
	rte_eal_service_func cb;
	/* args to the callback function */
	void *cb_args;
	/* configuration of the service */
	struct rte_service_config config;
};

/**
 * @internal - Only DPDK internal components request "service" cores.
 *
 * Registers a service in EAL.
 *
 * Registered services' configurations are exposed to an application using
 * *rte_eal_service_get_all*. These services have names which can be understood
 * by the application, and the application logic can decide how to allocate
 * cores to run the various services.
 *
 * This function is expected to be called by a DPDK component to indicate that
 * it require a CPU to run a specific function in order for it to perform its
 * processing. An example of such a component is the eventdev software PMD.
 *
 * The config struct should be filled in as appropriate by the PMD. For example
 * the name field should include some level of detail (e.g. "ethdev_p1_rx_q3"
 * might mean RX from an ethdev from port 1, on queue 3).
 *
 * @param service
 *   The service structure to be registered with EAL.
 *
 * @return
 *   On success, zero
 *   On failure, a negative error
 */
int rte_eal_service_register(const struct rte_service *service);

/**
 * Get the count of services registered in the EAL.
 *
 * @return the number of services registered with EAL.
 */
uint32_t rte_eal_service_get_count();

/**
 * Writes all registered services to the application supplied array.
 *
 * This function can be used by the application to understand if and what
 * services require running. Each service provides a config struct exposing
 * attributes of the service, which can be used by the application to decide on
 * its strategy of running services on cores.
 *
 * @param service_config
 *   An array of service config structures to be filled in
 *
 * @param n
 *   The size of the service config array
 *
 * @return 0 on successful providing all service configs
 *         -ENOSPC if array passed in is too small
 */
int rte_eal_service_get_all(const struct rte_service_config *service_config,
			    uint32_t n);

/**
 * Use *lcore_id* as a service core.
 *
 * EAL will internally remove *lcore_id* from the application accessible
 * core list. As a result, the application will never have its code run on
 * the service core, making the service cores totally transparent to an app.
 *
 * This function should be called before *rte_eal_service_set_coremask* so that
 * when setting the core mask the value can be error checked to ensure that EAL
 * has an lcore backing the coremask specified.
 */
int rte_eal_service_use_lcore(uint16_t lcore_id);

/**
 * Set a coremask for a service.
 *
 * A configuration for a service indicates which cores run the service, and
 * what other parameters are required to correclty handle the underlying device.
 *
 * Examples of advanced usage might be a HW device that supports multiple lcores
 * transmitting packets on the same queue - the hardware provides a mechanism
 * for multithreaded enqueue. In this case, the atomic_
 *
 * @return 0 Success
 *         -ENODEV   Device with *name* does not exist
 *         -ENOTSUP  Configuration is un-supported
 */
int rte_eal_service_set_coremask(const char *name, uint64_t coremask);


#ifdef __cplusplus
}
#endif


#endif /* _RTE_SERVICE_H_ */
