/*-
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

#ifndef _RTE_COMPRESSDEV_PMD_H_
#define _RTE_COMPRESSDEV_PMD_H_

/** @file
 * RTE comp PMD APIs
 *
 * @note
 * These API are from comp PMD only and user applications should not call
 * them directly.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>

#include <rte_dev.h>
#include <rte_malloc.h>
#include <rte_mbuf.h>
#include <rte_mempool.h>
#include <rte_log.h>
#include <rte_common.h>

#include "rte_comp.h"
#include "rte_compressdev.h"


#define RTE_COMPRESSDEV_PMD_NAME_ARG			("name")
#define RTE_COMPRESSDEV_PMD_MAX_NB_QP_ARG		("max_nb_queue_pairs")
#define RTE_COMPRESSDEV_PMD_SOCKET_ID_ARG		("socket_id")


static const char * const compressdev_pmd_valid_params[] = {
	RTE_COMPRESSDEV_PMD_NAME_ARG,
	RTE_COMPRESSDEV_PMD_MAX_NB_QP_ARG,
	RTE_COMPRESSDEV_PMD_SOCKET_ID_ARG
};

/**
 * @internal
 * Initialisation parameters for comp devices
 */
struct rte_compressdev_pmd_init_params {
	char name[RTE_COMPRESSDEV_NAME_MAX_LEN];
	size_t private_data_size;
	int socket_id;
	unsigned int max_nb_queue_pairs;
};

/** Global structure used for maintaining state of allocated comp devices */
struct rte_compressdev_global {
	struct rte_compressdev *devs;	/**< Device information array */
	struct rte_compressdev_data *data[RTE_COMPRESS_MAX_DEVS];
	/**< Device private data */
	uint8_t nb_devs;		/**< Number of devices found */
	uint8_t max_devs;		/**< Max number of devices */
};

/* compressdev driver, containing the driver ID */
struct compressdev_driver {
	TAILQ_ENTRY(compressdev_driver) next; /**< Next in list. */
	const struct rte_driver *driver;
	uint8_t id;
};

/** pointer to global comp devices data structure. */
extern struct rte_compressdev_global *rte_compressdev_globals;

/**
 * Get the rte_compressdev structure device pointer for the device. Assumes a
 * valid device index.
 *
 * @param	dev_id	Device ID value to select the device structure.
 *
 * @return
 *   - The rte_compressdev structure pointer for the given device ID.
 */
struct rte_compressdev *
rte_compressdev_pmd_get_dev(uint8_t dev_id);

/**
 * Get the rte_compressdev structure device pointer for the named device.
 *
 * @param	name	device name to select the device structure.
 *
 * @return
 *   - The rte_compressdev structure pointer for the given device ID.
 */
struct rte_compressdev *
rte_compressdev_pmd_get_named_dev(const char *name);

/**
 * Validate if the comp device index is valid attached comp device.
 *
 * @param	dev_id	comp device index.
 *
 * @return
 *   - If the device index is valid (1) or not (0).
 */
unsigned int
rte_compressdev_pmd_is_valid_dev(uint8_t dev_id);

/**
 * The pool of rte_compressdev structures.
 */
extern struct rte_compressdev *rte_compressdevs;


/**
 * Definitions of all functions exported by a driver through the
 * the generic structure of type *comp_dev_ops* supplied in the
 * *rte_compressdev* structure associated with a device.
 */

/**
 *	Function used to configure device.
 *
 * @param	dev	comp device pointer
 *		config	comp device configurations
 *
 * @return	Returns 0 on success
 */
typedef int (*compressdev_configure_t)(struct rte_compressdev *dev,
		struct rte_compressdev_config *config);

/**
 * Function used to start a configured device.
 *
 * @param	dev	comp device pointer
 *
 * @return	Returns 0 on success
 */
typedef int (*compressdev_start_t)(struct rte_compressdev *dev);

/**
 * Function used to stop a configured device.
 *
 * @param	dev	comp device pointer
 */
typedef void (*compressdev_stop_t)(struct rte_compressdev *dev);

/**
 * Function used to close a configured device.
 *
 * @param	dev	comp device pointer
 * @return
 * - 0 on success.
 * - EAGAIN if can't close as device is busy
 */
typedef int (*compressdev_close_t)(struct rte_compressdev *dev);


/**
 * Function used to get statistics of a device.
 *
 * @param	dev	comp device pointer
 * @param	stats	Pointer to comp device stats structure to populate
 */
typedef void (*compressdev_stats_get_t)(struct rte_compressdev *dev,
				struct rte_compressdev_stats *stats);


/**
 * Function used to reset statistics of a device.
 *
 * @param	dev	comp device pointer
 */
typedef void (*compressdev_stats_reset_t)(struct rte_compressdev *dev);


/**
 * Function used to get specific information of a device.
 *
 * @param	dev	comp device pointer
 */
typedef void (*compressdev_info_get_t)(struct rte_compressdev *dev,
				struct rte_compressdev_info *dev_info);

/**
 * Start queue pair of a device.
 *
 * @param	dev	comp device pointer
 * @param	qp_id	Queue Pair Index
 *
 * @return	Returns 0 on success.
 */
typedef int (*compressdev_queue_pair_start_t)(struct rte_compressdev *dev,
				uint16_t qp_id);

/**
 * Stop queue pair of a device.
 *
 * @param	dev	comp device pointer
 * @param	qp_id	Queue Pair Index
 *
 * @return	Returns 0 on success.
 */
typedef int (*compressdev_queue_pair_stop_t)(struct rte_compressdev *dev,
				uint16_t qp_id);

/**
 * Setup a queue pair for a device.
 *
 * @param	dev		 comp device pointer
 * @param	qp_id		 Queue Pair Index
 * @param	max_inflight_ops Max inflight ops which qp must accommodate
 * @param	socket_id	 Socket Index
 *
 * @return	Returns 0 on success.
 */
typedef int (*compressdev_queue_pair_setup_t)(struct rte_compressdev *dev,
		uint16_t qp_id,	uint32_t max_inflight_ops, int socket_id);

/**
 * Release memory resources allocated by given queue pair.
 *
 * @param	dev	comp device pointer
 * @param	qp_id	Queue Pair Index
 *
 * @return
 * - 0 on success.
 * - EAGAIN if can't close as device is busy
 */
typedef int (*compressdev_queue_pair_release_t)(struct rte_compressdev *dev,
		uint16_t qp_id);

/**
 * Get number of available queue pairs of a device.
 *
 * @param	dev	comp device pointer
 *
 * @return	Returns number of queue pairs on success.
 */
typedef uint32_t (*compressdev_queue_pair_count_t)(struct rte_compressdev *dev);

/**
 * Create a session mempool to allocate sessions from
 *
 * @param	dev		comp device pointer
 * @param	nb_objs		number of sessions objects in mempool
 * @param	obj_cache	l-core object cache size, see *rte_ring_create*
 * @param	socket_id	Socket Id to allocate  mempool on.
 *
 * @return
 * - On success returns a pointer to a rte_mempool
 * - On failure returns a NULL pointer
 */
typedef int (*compressdev_create_session_pool_t)(
		struct rte_compressdev *dev, unsigned int nb_objs,
		unsigned int obj_cache_size, int socket_id);


/**
 * Get the size of a compressdev session
 *
 * @param	dev		comp device pointer
 *
 * @return
 *  - On success returns the size of the session structure for device
 *  - On failure returns 0
 */
typedef unsigned int (*compressdev_get_session_private_size_t)(
		struct rte_compressdev *dev);

/**
 * Configure a comp session on a device.
 *
 * @param	dev		comp device pointer
 * @param	xform		Single or chain of comp xforms
 * @param	priv_sess	Ptr to compressdev's private session structure
 * @param	mp		Mempool where the private session is allocated
 *
 * @return
 *  - Returns 0 if private session structure have been created successfully.
 *  - Returns -EINVAL if input parameters are invalid.
 *  - Returns -ENOTSUP if comp device does not support the comp transform.
 *  - Returns -ENOMEM if the private session could not be allocated.
 */
typedef int (*compressdev_configure_session_t)(struct rte_compressdev *dev,
		struct rte_comp_xform *xform,
		struct rte_comp_session *session,
		struct rte_mempool *mp);

/**
 * Free driver private session data.
 *
 * @param	dev		comp device pointer
 * @param	sess		compressdev session structure
 */
typedef void (*compressdev_free_session_t)(struct rte_compressdev *dev,
		struct rte_comp_session *sess);

/**
 * Optional API for drivers to attach sessions with queue pair.
 * @param	dev		comp device pointer
 * @param	qp_id		queue pair id for attaching session
 * @param	priv_sess       Ptr to compressdev's private session structure
 * @return
 *  - Return 0 on success
 */
typedef int (*compressdev_queue_pair_attach_session_t)(
		  struct rte_compressdev *dev,
		  uint16_t qp_id,
		  void *session_private);

/**
 * Optional API for drivers to detach sessions from queue pair.
 * @param	dev		comp device pointer
 * @param	qp_id		queue pair id for detaching session
 * @param	priv_sess       Ptr to compressdev's private session structure
 * @return
 *  - Return 0 on success
 */
typedef int (*compressdev_queue_pair_detach_session_t)(
		  struct rte_compressdev *dev,
		  uint16_t qp_id,
		  void *session_private);

/** comp device operations function pointer table */
struct rte_compressdev_ops {
	compressdev_configure_t dev_configure;	/**< Configure device. */
	compressdev_start_t dev_start;		/**< Start device. */
	compressdev_stop_t dev_stop;		/**< Stop device. */
	compressdev_close_t dev_close;		/**< Close device. */

	compressdev_info_get_t dev_infos_get;	/**< Get device info. */

	compressdev_stats_get_t stats_get;
	/**< Get device statistics. */
	compressdev_stats_reset_t stats_reset;
	/**< Reset device statistics. */

	compressdev_queue_pair_setup_t queue_pair_setup;
	/**< Set up a device queue pair. */
	compressdev_queue_pair_release_t queue_pair_release;
	/**< Release a queue pair. */
	compressdev_queue_pair_start_t queue_pair_start;
	/**< Start a queue pair. */
	compressdev_queue_pair_stop_t queue_pair_stop;
	/**< Stop a queue pair. */
	compressdev_queue_pair_count_t queue_pair_count;
	/**< Get count of the queue pairs. */

	compressdev_get_session_private_size_t session_get_size;
	/**< Return private session. */
	compressdev_configure_session_t session_configure;
	/**< Configure a comp session. */
	compressdev_free_session_t session_clear;
	/**< Clear a comp sessions private data. */
	compressdev_queue_pair_attach_session_t qp_attach_session;
	/**< Attach session to queue pair. */
	compressdev_queue_pair_detach_session_t qp_detach_session;
	/**< Detach session from queue pair. */
};


/**
 * Function for internal use by dummy drivers primarily, e.g. ring-based
 * driver.
 * Allocates a new compressdev slot for an comp device and returns the pointer
 * to that slot for the driver to use.
 *
 * @param	name		Unique identifier name for each device
 * @param	socket_id	Socket to allocate resources on.
 * @return
 *   - Slot in the rte_dev_devices array for a new device;
 */
struct rte_compressdev *
rte_compressdev_pmd_allocate(const char *name, int socket_id);

/**
 * Function for internal use by dummy drivers primarily, e.g. ring-based
 * driver.
 * Release the specified compressdev device.
 *
 * @param compressdev
 * The *compressdev* pointer is the address of the *rte_compressdev* structure.
 * @return
 *   - 0 on success, negative on error
 */
extern int
rte_compressdev_pmd_release_device(struct rte_compressdev *compressdev);


/**
 * @internal
 *
 * PMD assist function to parse initialisation arguments for comp driver
 * when creating a new comp PMD device instance.
 *
 * PMD driver should set default values for that PMD before calling function,
 * these default values will be over-written with successfully parsed values
 * from args string.
 *
 * @param	params	parsed PMD initialisation parameters
 * @param	args	input argument string to parse
 *
 * @return
 *  - 0 on success
 *  - errno on failure
 */
int
rte_compressdev_pmd_parse_input_args(
		struct rte_compressdev_pmd_init_params *params,
		const char *args);

/**
 * @internal
 *
 * PMD assist function to provide boiler plate code for comp driver to create
 * and allocate resources for a new comp PMD device instance.
 *
 * @param	name	comp device name.
 * @param	device	base device instance
 * @param	params	PMD initialisation parameters
 *
 * @return
 *  - comp device instance on success
 *  - NULL on creation failure
 */
struct rte_compressdev *
rte_compressdev_pmd_create(const char *name,
		struct rte_device *device,
		struct rte_compressdev_pmd_init_params *params);

/**
 * @internal
 *
 * PMD assist function to provide boiler plate code for comp driver to
 * destroy and free resources associated with a comp PMD device instance.
 *
 * @param	compressdev	comp device handle.
 *
 * @return
 *  - 0 on success
 *  - errno on failure
 */
int
rte_compressdev_pmd_destroy(struct rte_compressdev *compressdev);

/**
 * Executes all the user application registered callbacks for the specific
 * device.
 *  *
 * @param	dev	Pointer to compressdev struct
 * @param	event	comp device interrupt event type.
 *
 * @return
 *  void
 */
void rte_compressdev_pmd_callback_process(struct rte_compressdev *dev,
				enum rte_compressdev_event_type event);

/**
 * @internal
 * Create unique device name
 */
int
rte_compressdev_pmd_create_dev_name(char *name, const char *dev_name_prefix);

/**
 * @internal
 * Allocate compressdev driver.
 *
 * @param comp_drv
 *   Pointer to compressdev_driver.
 * @param drv
 *   Pointer to rte_driver.
 *
 * @return
 *  The driver type identifier
 */
uint8_t rte_compressdev_allocate_driver(struct compressdev_driver *comp_drv,
		const struct rte_driver *drv);


#define RTE_PMD_REGISTER_COMPRESSDEV_DRIVER(comp_drv, drv, driver_id)\
RTE_INIT(init_ ##driver_id);\
static void init_ ##driver_id(void)\
{\
	driver_id = rte_compressdev_allocate_driver(&comp_drv, &(drv).driver);\
}

static inline void *
get_session_private_data(const struct rte_comp_session *sess,
		uint8_t driver_id) {
	return sess->sess_private_data[driver_id];
}

static inline void
set_session_private_data(struct rte_comp_session *sess,
		uint8_t driver_id, void *private_data)
{
	sess->sess_private_data[driver_id] = private_data;
}

#ifdef __cplusplus
}
#endif

#endif /* _RTE_COMPRESSDEV_PMD_H_ */
