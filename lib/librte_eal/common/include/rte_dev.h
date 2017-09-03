/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2014 6WIND S.A.
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

#ifndef _RTE_DEV_H_
#define _RTE_DEV_H_

/**
 * @file
 *
 * RTE PMD Driver Registration Interface
 *
 * This file manages the list of device drivers.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <sys/queue.h>

#include <rte_config.h>
#include <rte_log.h>

struct rte_device;

struct rte_eal_uev_callback;
/** @internal Structure to keep track of registered callbacks */
TAILQ_HEAD(rte_eal_uev_cb_list, rte_eal_uev_callback);


__attribute__((format(printf, 2, 0)))
static inline void
rte_pmd_debug_trace(const char *func_name, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);

	char buffer[vsnprintf(NULL, 0, fmt, ap) + 1];

	va_end(ap);

	va_start(ap, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, ap);
	va_end(ap);

	rte_log(RTE_LOG_ERR, RTE_LOGTYPE_PMD, "%s: %s", func_name, buffer);
}

/*
 * Enable RTE_PMD_DEBUG_TRACE() when at least one component relying on the
 * RTE_*_RET() macros defined below is compiled in debug mode.
 */
#if defined(RTE_LIBRTE_ETHDEV_DEBUG) || \
	defined(RTE_LIBRTE_CRYPTODEV_DEBUG) || \
	defined(RTE_LIBRTE_EVENTDEV_DEBUG)
#define RTE_PMD_DEBUG_TRACE(...) \
	rte_pmd_debug_trace(__func__, __VA_ARGS__)
#else
#define RTE_PMD_DEBUG_TRACE(...) (void)0
#endif

/* Macros for checking for restricting functions to primary instance only */
#define RTE_PROC_PRIMARY_OR_ERR_RET(retval) do { \
	if (rte_eal_process_type() != RTE_PROC_PRIMARY) { \
		RTE_PMD_DEBUG_TRACE("Cannot run in secondary processes\n"); \
		return retval; \
	} \
} while (0)

#define RTE_PROC_PRIMARY_OR_RET() do { \
	if (rte_eal_process_type() != RTE_PROC_PRIMARY) { \
		RTE_PMD_DEBUG_TRACE("Cannot run in secondary processes\n"); \
		return; \
	} \
} while (0)

/* Macros to check for invalid function pointers */
#define RTE_FUNC_PTR_OR_ERR_RET(func, retval) do { \
	if ((func) == NULL) { \
		RTE_PMD_DEBUG_TRACE("Function not supported\n"); \
		return retval; \
	} \
} while (0)

#define RTE_FUNC_PTR_OR_RET(func) do { \
	if ((func) == NULL) { \
		RTE_PMD_DEBUG_TRACE("Function not supported\n"); \
		return; \
	} \
} while (0)

/**
 * Device driver.
 */
enum rte_kernel_driver {
	RTE_KDRV_UNKNOWN = 0,
	RTE_KDRV_IGB_UIO,
	RTE_KDRV_VFIO,
	RTE_KDRV_UIO_GENERIC,
	RTE_KDRV_NIC_UIO,
	RTE_KDRV_NONE,
};

/**
 * Device policies.
 */
enum rte_dev_policy {
	RTE_DEV_WHITELISTED,
	RTE_DEV_BLACKLISTED,
};

/**
 * A generic memory resource representation.
 */
struct rte_mem_resource {
	uint64_t phys_addr; /**< Physical address, 0 if not resource. */
	uint64_t len;       /**< Length of the resource. */
	void *addr;         /**< Virtual address, NULL when not mapped. */
};

/**
 * A structure describing a device driver.
 */
struct rte_driver {
	TAILQ_ENTRY(rte_driver) next;  /**< Next in list. */
	const char *name;                   /**< Driver name. */
	const char *alias;              /**< Driver alias. */
};

#define RTE_DEV_NAME_MAX_LEN (32)

/**
 * A structure describing a generic device.
 */
struct rte_device {
	TAILQ_ENTRY(rte_device) next; /**< Next device */
	const char *name;             /**< Device name */
	const struct rte_driver *driver;/**< Associated driver */
	int numa_node;                /**< NUMA node connection */
	struct rte_devargs *devargs;  /**< Device user arguments */
	/** User application callbacks for device uevent monitoring  */
	struct rte_eal_uev_cb_list uev_cbs;
};

/**
 * Initialize a driver specified by name.
 *
 * @param name
 *   The pointer to a driver name to be initialized.
 * @param args
 *   The pointer to arguments used by driver initialization.
 * @return
 *  0 on success, negative on error
 */
int rte_vdev_init(const char *name, const char *args);

/**
 * Uninitalize a driver specified by name.
 *
 * @param name
 *   The pointer to a driver name to be initialized.
 * @return
 *  0 on success, negative on error
 */
int rte_vdev_uninit(const char *name);

/**
 * Attach a device to a registered driver.
 *
 * @param name
 *   The device name, that refers to a pci device (or some private
 *   way of designating a vdev device). Based on this device name, eal
 *   will identify a driver capable of handling it and pass it to the
 *   driver probing function.
 * @param devargs
 *   Device arguments to be passed to the driver.
 * @return
 *   0 on success, negative on error.
 */
int rte_eal_dev_attach(const char *name, const char *devargs);

/**
 * Detach a device from its driver.
 *
 * @param dev
 *   A pointer to a rte_device structure.
 * @return
 *   0 on success, negative on error.
 */
int rte_eal_dev_detach(struct rte_device *dev);

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Hotplug add a given device to a specific bus.
 *
 * @param busname
 *   The bus name the device is added to.
 * @param devname
 *   The device name. Based on this device name, eal will identify a driver
 *   capable of handling it and pass it to the driver probing function.
 * @param devargs
 *   Device arguments to be passed to the driver.
 * @return
 *   0 on success, negative on error.
 */
int rte_eal_hotplug_add(const char *busname, const char *devname,
			const char *devargs);

/**
 * @warning
 * @b EXPERIMENTAL: this API may change without prior notice
 *
 * Hotplug remove a given device from a specific bus.
 *
 * @param busname
 *   The bus name the device is removed from.
 * @param devname
 *   The device name being removed.
 * @return
 *   0 on success, negative on error.
 */
int rte_eal_hotplug_remove(const char *busname, const char *devname);

#define RTE_EAL_UEVENT_MSG_LEN 4096
#define RTE_EAL_UEVENT_SUBSYSTEM_UIO 1
#define RTE_EAL_UEVENT_SUBSYSTEM_VFIO 2

/**
 * The eth device event type for interrupt, and maybe others in the future.
 */
enum rte_eal_uevent_type {
	RTE_EAL_UEVENT_UNKNOWN,  /**< unknown event type */
	RTE_EAL_UEVENT_ADD, /**< lsc interrupt event */
	RTE_EAL_UEVENT_REMOVE,
				/**< queue state event (enabled/disabled) */
	RTE_EAL_UEVENT_CHANGE,
			/**< reset interrupt event, sent to VF on PF reset */
	RTE_EAL_UEVENT_MOVE,  /**< message from the VF received by PF */
	RTE_EAL_UEVENT_ONLINE,   /**< MACsec offload related event */
	RTE_EAL_UEVENT_OFFLINE, /**< device removal event */
	RTE_EAL_UEVENT_MAX       /**< max value of this enum */
};

struct rte_eal_uevent {
	enum rte_eal_uevent_type type;	/**< uevent action type */
	int subsystem;				/**< subsystem id */
};

/**
 * create  the device uevent file descriptor.
 * @return
 *   - On success, the device uevent fd.
 *   - On failure, a negative value.
 */
int
rte_eal_uev_fd_new(void);

/**
 * Bind  the netlink to enable  uevent receiving.
 *
 * @param fd
 *   The fd which the uevent associated to
 * @return
 *   - On success, zero.
 *   - On failure, a negative value.
 */
int
rte_eal_uev_enable(int fd);

/**
 * It read out the uevent from the specific file descriptor.
 *
 * @param fd
 *   The fd which the uevent associated to
 * @param uevent
 *   Pointer to the uevent which read from the monitoring fd.
 * @return
 *   - On success, zero.
 *   - On failure, a negative value.
 */
int
rte_eal_uev_receive(int fd, struct rte_eal_uevent *uevent);

typedef int (*rte_eal_uev_cb_fn)(struct rte_device *dev,
		enum rte_eal_uevent_type event, void *cb_arg, void *ret_param);
/**< user application callback to be registered for interrupts */

/**
 * Register a callback function for specific device..
 *
 * @param dev
 *  Pointer to struct rte_device.
 * @param event
 *  Uevent interested.
 * @param cb_fn
 *  User supplied callback function to be called.
 * @param cb_arg
 *  Pointer to the parameters for the registered callback.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
int rte_eal_uev_callback_register(struct rte_device *dev,
			enum rte_eal_uevent_type event,
			rte_eal_uev_cb_fn cb_fn, void *cb_arg);

/**
 * Unregister a callback function for specific device.
 *
 * @param device
 *  Pointer to struct rte_device.
 * @param event
 *  Uevent interested.
 * @param cb_fn
 *  User supplied callback function to be called.
 * @param cb_arg
 *  Pointer to the parameters for the registered callback. -1 means to
 *  remove all for the same callback address and same event.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
int rte_eal_uev_callback_unregister(struct rte_device *dev,
			enum rte_eal_uevent_type event,
		rte_eal_uev_cb_fn cb_fn, void *cb_arg);

/**
 * @internal Executes all the user application registered callbacks for
 * the specific device. It is for DPDK internal user only. User
 * application should not call it directly.
 *
 * @param dev
 *  Pointer to struct rte_device.
 * @param event
 *  rte device uevent type.
 * @param cb_arg
 *  callback parameter.
 * @param ret_param
 *  To pass data back to user application.
 *  This allows the user application to decide if a particular function
 *  is permitted or not.
 *
 * @return
 *  int
 */
int _rte_eal_uev_callback_process(struct rte_device *dev,
		enum rte_eal_uevent_type event, void *cb_arg, void *ret_param);

/**
 * Device comparison function.
 *
 * This type of function is used to compare an rte_device with arbitrary
 * data.
 *
 * @param dev
 *   Device handle.
 *
 * @param data
 *   Data to compare against. The type of this parameter is determined by
 *   the kind of comparison performed by the function.
 *
 * @return
 *   0 if the device matches the data.
 *   !0 if the device does not match.
 *   <0 if ordering is possible and the device is lower than the data.
 *   >0 if ordering is possible and the device is greater than the data.
 */
typedef int (*rte_dev_cmp_t)(const struct rte_device *dev, const void *data);

#define RTE_PMD_EXPORT_NAME_ARRAY(n, idx) n##idx[]

#define RTE_PMD_EXPORT_NAME(name, idx) \
static const char RTE_PMD_EXPORT_NAME_ARRAY(this_pmd_name, idx) \
__attribute__((used)) = RTE_STR(name)

#define DRV_EXP_TAG(name, tag) __##name##_##tag

#define RTE_PMD_REGISTER_PCI_TABLE(name, table) \
static const char DRV_EXP_TAG(name, pci_tbl_export)[] __attribute__((used)) = \
RTE_STR(table)

#define RTE_PMD_REGISTER_PARAM_STRING(name, str) \
static const char DRV_EXP_TAG(name, param_string_export)[] \
__attribute__((used)) = str

/**
 * Advertise the list of kernel modules required to run this driver
 *
 * This string lists the kernel modules required for the devices
 * associated to a PMD. The format of each line of the string is:
 * "<device-pattern> <kmod-expression>".
 *
 * The possible formats for the device pattern are:
 *   "*"                     all devices supported by this driver
 *   "pci:*"                 all PCI devices supported by this driver
 *   "pci:v8086:d*:sv*:sd*"  all PCI devices supported by this driver
 *                           whose vendor id is 0x8086.
 *
 * The format of the kernel modules list is a parenthesed expression
 * containing logical-and (&) and logical-or (|).
 *
 * The device pattern and the kmod expression are separated by a space.
 *
 * Example:
 * - "* igb_uio | uio_pci_generic | vfio"
 */
#define RTE_PMD_REGISTER_KMOD_DEP(name, str) \
static const char DRV_EXP_TAG(name, kmod_dep_export)[] \
__attribute__((used)) = str

#ifdef __cplusplus
}
#endif

#endif /* _RTE_VDEV_H_ */
