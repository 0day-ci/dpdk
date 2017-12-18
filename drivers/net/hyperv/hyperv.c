/*-
 *   BSD LICENSE
 *
 *   Copyright 2017 6WIND S.A.
 *   Copyright 2017 Mellanox
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
 *     * Neither the name of 6WIND S.A. nor the names of its
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

#include <stddef.h>
#include <string.h>

#include <rte_bus_vdev.h>
#include <rte_config.h>
#include <rte_kvargs.h>
#include <rte_log.h>

#define HYPERV_DRIVER net_hyperv
#define HYPERV_ARG_IFACE "iface"
#define HYPERV_ARG_MAC "mac"

#ifdef RTE_LIBRTE_HYPERV_DEBUG

#define PMD_DRV_LOG(level, ...) \
	RTE_LOG(level, PMD, \
		RTE_FMT("%s:%u: %s(): " RTE_FMT_HEAD(__VA_ARGS__,) "\n", \
			strrchr("/" __FILE__, '/') + 1, \
			__LINE__, \
			__func__, \
			RTE_FMT_TAIL(__VA_ARGS__,)))

#else /* RTE_LIBRTE_HYPERV_DEBUG */

#define PMD_DRV_LOG(level, ...) \
	RTE_LOG(level, PMD, \
		RTE_FMT(RTE_STR(HYPERV_DRIVER) ": " \
			RTE_FMT_HEAD(__VA_ARGS__,) "\n", \
		RTE_FMT_TAIL(__VA_ARGS__,)))

#endif /* RTE_LIBRTE_HYPERV_DEBUG */

#define DEBUG(...) PMD_DRV_LOG(DEBUG, __VA_ARGS__)
#define INFO(...) PMD_DRV_LOG(INFO, __VA_ARGS__)
#define WARN(...) PMD_DRV_LOG(WARNING, __VA_ARGS__)
#define ERROR(...) PMD_DRV_LOG(ERR, __VA_ARGS__)

/** Number of PMD instances relying on context list. */
static unsigned int hyperv_ctx_inst;

/**
 * Probe NetVSC interfaces.
 *
 * @param dev
 *   Virtual device context for PMD instance.
 *
 * @return
 *    Always 0, even in case of errors.
 */
static int
hyperv_vdev_probe(struct rte_vdev_device *dev)
{
	static const char *const hyperv_arg[] = {
		HYPERV_ARG_IFACE,
		HYPERV_ARG_MAC,
		NULL,
	};
	const char *name = rte_vdev_device_name(dev);
	const char *args = rte_vdev_device_args(dev);
	struct rte_kvargs *kvargs = rte_kvargs_parse(args ? args : "",
						     hyperv_arg);

	DEBUG("invoked as \"%s\", using arguments \"%s\"", name, args);
	if (!kvargs) {
		ERROR("cannot parse arguments list");
		goto error;
	}
error:
	if (kvargs)
		rte_kvargs_free(kvargs);
	++hyperv_ctx_inst;
	return 0;
}

/**
 * Remove PMD instance.
 *
 * @param dev
 *   Virtual device context for PMD instance.
 *
 * @return
 *   Always 0.
 */
static int
hyperv_vdev_remove(struct rte_vdev_device *dev)
{
	(void)dev;
	--hyperv_ctx_inst;
	return 0;
}

/** Virtual device descriptor. */
static struct rte_vdev_driver hyperv_vdev = {
	.probe = hyperv_vdev_probe,
	.remove = hyperv_vdev_remove,
};

RTE_PMD_REGISTER_VDEV(HYPERV_DRIVER, hyperv_vdev);
RTE_PMD_REGISTER_ALIAS(HYPERV_DRIVER, eth_hyperv);
RTE_PMD_REGISTER_PARAM_STRING(net_hyperv,
			      HYPERV_ARG_IFACE "=<string> "
			      HYPERV_ARG_MAC "=<string>");
