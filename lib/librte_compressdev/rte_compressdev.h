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

#ifndef _RTE_COMPRESSDEV_H_
#define _RTE_COMPRESSDEV_H_

/**
 * @file rte_compressdev.h
 *
 * RTE Compression Device APIs
 *
 * Defines RTE comp Device APIs for the provisioning of compression operations.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "rte_kvargs.h"
#include "rte_comp.h"
#include "rte_dev.h"
#include <rte_common.h>

extern const char **rte_cyptodev_names;

/* Logging Macros */

#define COMPDEV_LOG_ERR(...) \
	RTE_LOG(ERR, COMPRESSDEV, \
		RTE_FMT("%s() line %u: " RTE_FMT_HEAD(__VA_ARGS__, ) "\n", \
			__func__, __LINE__, RTE_FMT_TAIL(__VA_ARGS__,)))

#define COMPDEV_LOG_INFO(...) \
	RTE_LOG(INFO, COMPRESSDEV, \
		RTE_FMT(RTE_FMT_HEAD(__VA_ARGS__,) "\n", \
			RTE_FMT_TAIL(__VA_ARGS__,)))

#ifdef RTE_LIBRTE_COMPRESSDEV_DEBUG
#define COMPDEV_LOG_DEBUG(...) \
	RTE_LOG(DEBUG, COMPRESSDEV, \
		RTE_FMT("%s() line %u: " RTE_FMT_HEAD(__VA_ARGS__,) "\n", \
			__func__, __LINE__, RTE_FMT_TAIL(__VA_ARGS__,)))

#define COMPDEV_PMD_TRACE(...) \
	RTE_LOG(DEBUG, COMPRESSDEV, \
		RTE_FMT("[%s] %s: " RTE_FMT_HEAD(__VA_ARGS__,) "\n", \
			dev, __func__, RTE_FMT_TAIL(__VA_ARGS__,)))

#else
#define COMPDEV_LOG_DEBUG(...) (void)0
#define COMPDEV_PMD_TRACE(...) (void)0
#endif



/**
 * A macro that points to an offset from the start
 * of the comp operation structure (rte_comp_op)
 *
 * The returned pointer is cast to type t.
 *
 * @param c
 *   The comp operation.
 * @param o
 *   The offset from the start of the comp operation.
 * @param t
 *   The type to cast the result into.
 */
#define rte_comp_op_ctod_offset(c, t, o)	\
	((t)((char *)(c) + (o)))

/**
 * A macro that returns the physical address that points
 * to an offset from the start of the comp operation
 * (rte_comp_op)
 *
 * @param c
 *   The comp operation.
 * @param o
 *   The offset from the start of the comp operation
 *   to calculate address from.
 */
#define rte_comp_op_ctophys_offset(c, o)	\
	(rte_iova_t)((c)->phys_addr + (o))

/**
 * comp parameters range description
 */
struct rte_comp_param_range {
	uint16_t min;	/**< minimum size */
	uint16_t max;	/**< maximum size */
	uint16_t increment;
	/**< if a range of sizes are supported,
	 * this parameter is used to indicate
	 * increments in byte size that are supported
	 * between the minimum and maximum
	 */
};


/** Structure used to capture a capability of a comp device */
struct rte_compressdev_capabilities {
	/* TODO */
	enum rte_comp_algorithm algo;
	uint64_t comp_feature_flags;
	/**< bitmap of flags for compression service features*/
	struct rte_comp_param_range window_size;
	/**< window size range in bytes */
};


/** Macro used at end of comp PMD list */
#define RTE_COMP_END_OF_CAPABILITIES_LIST() \
	{ RTE_COMP_ALGO_LIST_END }


/**
 * compression device supported feature flags
 *
 * Note:
 * New features flags should be added to the end of the list
 *
 * Keep these flags synchronised with rte_compressdev_get_feature_name()
 */

#define	RTE_COMP_FF_HW_ACCELERATED		(1ULL << 0)
/**< Operations are off-loaded to an external hardware accelerator */
#define	RTE_COMP_FF_CPU_SSE			(1ULL << 1)
/**< Utilises CPU SIMD SSE instructions */
#define	RTE_COMP_FF_CPU_AVX			(1ULL << 2)
/**< Utilises CPU SIMD AVX instructions */
#define	RTE_COMP_FF_CPU_AVX2			(1ULL << 3)
/**< Utilises CPU SIMD AVX2 instructions */
#define	RTE_COMP_FF_CPU_AVX512		(1ULL << 4)
/**< Utilises CPU SIMD AVX512 instructions */
#define	RTE_COMP_FF_CPU_NEON			(1ULL << 5)
/**< Utilises CPU NEON instructions */

/**
 * compression service feature flags
 *
 * Note:
 * New features flags should be added to the end of the list
 *
 * Keep these flags synchronised with rte_comp_get_feature_name() TODO
 */
#define	RTE_COMP_FF_MBUF_SCATTER_GATHER		(1ULL << 0)
/**< Scatter-gather mbufs are supported */
#define RTE_COMP_FF_MULTI_PKT_CHECKSUM		(1ULL << 1)
/**< Generation of checksum across multiple stateless packets is supported */
#define RTE_COMP_FF_STATEFUL_COMPRESSION	(1ULL << 2)
/**< Stateful compression is supported */
#define RTE_COMP_FF_STATEFUL_DECOMPRESSION	(1ULL << 3)
/**< Stateful decompression is supported */

/**
 * Get the name of a comp device feature flag
 *
 * @param	flag	The mask describing the flag.
 *
 * @return
 *   The name of this flag, or NULL if it's not a valid feature flag.
 */

extern const char *
rte_compressdev_get_feature_name(uint64_t flag);
extern const char *
rte_comp_get_feature_name(uint64_t flag);

/**  comp device information */
struct rte_compressdev_info {
	const char *driver_name;		/**< Driver name. */
	uint8_t driver_id;			/**< Driver identifier */
	struct rte_pci_device *pci_dev;		/**< PCI information. */

	uint64_t feature_flags;			/**< Feature flags */

	const struct rte_compressdev_capabilities *capabilities;
	/**< Array of devices supported capabilities */

	unsigned int max_nb_queue_pairs;
	/**< Maximum number of queues pairs supported by device. */

	unsigned int max_nb_sessions_per_qp;
	/**< Maximum number of sessions per queue pair.
	 * Default 0 for infinite sessions
	 */
	unsigned int max_nb_streams_per_qp;
	/**< Maximum number of streams per queue pair.
	 * Default 0 for infinite streams
	 */

};

#define RTE_COMPRESSDEV_DETACHED  (0)
#define RTE_COMPRESSDEV_ATTACHED  (1)

/** Definitions of comp device event types */
enum rte_compressdev_event_type {
	RTE_COMPRESSDEV_EVENT_UNKNOWN,	/**< unknown event type */
	RTE_COMPRESSDEV_EVENT_ERROR,	/**< error interrupt event */
	RTE_COMPRESSDEV_EVENT_MAX		/**< max value of this enum */
};

/**
 * Typedef for application callback function to be registered by application
 * software for notification of device events
 *
 * @param	dev_id	comp device identifier
 * @param	event	comp device event to register for notification of.
 * @param	cb_arg	User specified parameter to be passed as to passed to
 *			users callback function.
 */
typedef void (*rte_compressdev_cb_fn)(uint8_t dev_id,
		enum rte_compressdev_event_type event, void *cb_arg);


/** comp device statistics */
struct rte_compressdev_stats {
	uint64_t enqueued_count;
	/**< Count of all operations enqueued */
	uint64_t dequeued_count;
	/**< Count of all operations dequeued */

	uint64_t enqueue_err_count;
	/**< Total error count on operations enqueued */
	uint64_t dequeue_err_count;
	/**< Total error count on operations dequeued */
};

#define RTE_COMPRESSDEV_NAME_MAX_LEN	(64)
/**< Max length of name of comp PMD */

/**
 * Get the device identifier for the named comp device.
 *
 * @param	name	device name to select the device structure.
 *
 * @return
 *   - Returns comp device identifier on success.
 *   - Return -1 on failure to find named comp device.
 */
extern int
rte_compressdev_get_dev_id(const char *name);

/**
 * Get the comp device name given a device identifier.
 *
 * @param dev_id
 *   The identifier of the device
 *
 * @return
 *   - Returns comp device name.
 *   - Returns NULL if comp device is not present.
 */
extern const char *
rte_compressdev_name_get(uint8_t dev_id);

/**
 * Get the total number of comp devices that have been successfully
 * initialised.
 *
 * @return
 *   - The total number of usable comp devices.
 */
extern uint8_t
rte_compressdev_count(void);

/**
 * Get number of comp device defined type.
 *
 * @param	driver_id	driver identifier.
 *
 * @return
 *   Returns number of comp device.
 */
extern uint8_t
rte_compressdev_device_count_by_driver(uint8_t driver_id);

/**
 * Get number and identifiers of attached comp devices that
 * use the same comp driver.
 *
 * @param	driver_name	driver name.
 * @param	devices		output devices identifiers.
 * @param	nb_devices	maximal number of devices.
 *
 * @return
 *   Returns number of attached comp device.
 */
uint8_t
rte_compressdev_devices_get(const char *driver_name, uint8_t *devices,
		uint8_t nb_devices);
/*
 * Return the NUMA socket to which a device is connected
 *
 * @param dev_id
 *   The identifier of the device
 * @return
 *   The NUMA socket id to which the device is connected or
 *   a default of zero if the socket could not be determined.
 *   -1 if returned is the dev_id value is out of range.
 */
extern int
rte_compressdev_socket_id(uint8_t dev_id);

/** comp device configuration structure */
struct rte_compressdev_config {
	int socket_id;
	/**< Socket on which to allocate resources */
	uint16_t nb_queue_pairs;
	/**< Total number of queue pairs to configure on a device */
};

/**
 * Configure a device.
 *
 * This function must be invoked first before any other function in the
 * API. This function can also be re-invoked when a device is in the
 * stopped state.
 *
 * @param	dev_id		The identifier of the device to configure.
 * @param	config		The comp device configuration structure.
 *
 * @return
 *   - 0: Success, device configured.
 *   - <0: Error code returned by the driver configuration function.
 */
extern int
rte_compressdev_configure(uint8_t dev_id,
			struct rte_compressdev_config *config);

/**
 * Start an device.
 *
 * The device start step is the last one and consists of setting the configured
 * offload features and in starting the transmit and the receive units of the
 * device.
 * On success, all basic functions exported by the API (link status,
 * receive/transmit, and so on) can be invoked.
 *
 * @param dev_id
 *   The identifier of the device.
 * @return
 *   - 0: Success, device started.
 *   - <0: Error code of the driver device start function.
 */
extern int
rte_compressdev_start(uint8_t dev_id);

/**
 * Stop an device. The device can be restarted with a call to
 * rte_compressdev_start()
 *
 * @param	dev_id		The identifier of the device.
 */
extern void
rte_compressdev_stop(uint8_t dev_id);

/**
 * Close an device. The device cannot be restarted!
 *
 * @param	dev_id		The identifier of the device.
 *
 * @return
 *  - 0 on successfully closing device
 *  - <0 on failure to close device
 */
extern int
rte_compressdev_close(uint8_t dev_id);

/**
 * Allocate and set up a receive queue pair for a device.
 *
 *
 * @param	dev_id		The identifier of the device.
 * @param	queue_pair_id	The index of the queue pairs to set up. The
 *				value must be in the range [0, nb_queue_pair
 *				- 1] previously supplied to
 *				rte_compressdev_configure().
 * @param	max_inflight_ops max number of ops which the qp will have to
 *				accommodate simultaneously.
 * @param	socket_id	The *socket_id* argument is the socket
 *				identifier in case of NUMA. The value can be
 *				*SOCKET_ID_ANY* if there is no NUMA constraint
 *				for the DMA memory allocated for the receive
 *				queue pair.
 * @return
 *   - 0: Success, queue pair correctly set up.
 *   - <0: Queue pair configuration failed
 */
extern int
rte_compressdev_queue_pair_setup(uint8_t dev_id, uint16_t queue_pair_id,
		uint32_t max_inflight_ops, int socket_id);

/**
 * Start a specified queue pair of a device. It is used
 * when deferred_start flag of the specified queue is true.
 *
 * @param	dev_id		The identifier of the device
 * @param	queue_pair_id	The index of the queue pair to start. The value
 *				must be in the range [0, nb_queue_pair - 1]
 *				previously supplied to
 *				rte_comp_dev_configure().
 * @return
 *   - 0: Success, the transmit queue is correctly set up.
 *   - -EINVAL: The dev_id or the queue_id out of range.
 *   - -ENOTSUP: The function not supported in PMD driver.
 */
extern int
rte_compressdev_queue_pair_start(uint8_t dev_id, uint16_t queue_pair_id);

/**
 * Stop specified queue pair of a device
 *
 * @param	dev_id		The identifier of the device
 * @param	queue_pair_id	The index of the queue pair to stop. The value
 *				must be in the range [0, nb_queue_pair - 1]
 *				previously supplied to
 *				rte_compressdev_configure().
 * @return
 *   - 0: Success, the transmit queue is correctly set up.
 *   - -EINVAL: The dev_id or the queue_id out of range.
 *   - -ENOTSUP: The function not supported in PMD driver.
 */
extern int
rte_compressdev_queue_pair_stop(uint8_t dev_id, uint16_t queue_pair_id);

/**
 * Get the number of queue pairs on a specific comp device
 *
 * @param	dev_id		comp device identifier.
 * @return
 *   - The number of configured queue pairs.
 */
extern uint16_t
rte_compressdev_queue_pair_count(uint8_t dev_id);


/**
 * Retrieve the general I/O statistics of a device.
 *
 * @param	dev_id		The identifier of the device.
 * @param	stats		A pointer to a structure of type
 *				*rte_compressdev_stats* to be filled with the
 *				values of device counters.
 * @return
 *   - Zero if successful.
 *   - Non-zero otherwise.
 */
extern int
rte_compressdev_stats_get(uint8_t dev_id, struct rte_compressdev_stats *stats);

/**
 * Reset the general I/O statistics of a device.
 *
 * @param	dev_id		The identifier of the device.
 */
extern void
rte_compressdev_stats_reset(uint8_t dev_id);

/**
 * Retrieve the contextual information of a device.
 *
 * @param	dev_id		The identifier of the device.
 * @param	dev_info	A pointer to a structure of type
 *				*rte_compressdev_info* to be filled with the
 *				contextual information of the device.
 *
 * @note The capabilities field of dev_info is set to point to the first
 * element of an array of struct rte_compressdev_capabilities. The element after
 * the last valid element has it's op field set to
 * RTE_COMP_ALGO_LIST_END.
 */
extern void
rte_compressdev_info_get(uint8_t dev_id, struct rte_compressdev_info *dev_info);


/**
 * Register a callback function for specific device id.
 *
 * @param	dev_id		Device id.
 * @param	event		Event interested.
 * @param	cb_fn		User supplied callback function to be called.
 * @param	cb_arg		Pointer to the parameters for the registered
 *				callback.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
extern int
rte_compressdev_callback_register(uint8_t dev_id,
		enum rte_compressdev_event_type event,
		rte_compressdev_cb_fn cb_fn, void *cb_arg);

/**
 * Unregister a callback function for specific device id.
 *
 * @param	dev_id		The device identifier.
 * @param	event		Event interested.
 * @param	cb_fn		User supplied callback function to be called.
 * @param	cb_arg		Pointer to the parameters for the registered
 *				callback.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
extern int
rte_compressdev_callback_unregister(uint8_t dev_id,
		enum rte_compressdev_event_type event,
		rte_compressdev_cb_fn cb_fn, void *cb_arg);


typedef uint16_t (*dequeue_pkt_burst_t)(void *qp,
		struct rte_comp_op **ops, uint16_t nb_ops);
/**< Dequeue processed packets from queue pair of a device. */

typedef uint16_t (*enqueue_pkt_burst_t)(void *qp,
		struct rte_comp_op **ops, uint16_t nb_ops);
/**< Enqueue packets for processing on queue pair of a device. */




struct rte_compressdev_callback;

/** Structure to keep track of registered callbacks */
TAILQ_HEAD(rte_compressdev_cb_list, rte_compressdev_callback);

/** The data structure associated with each comp device. */
struct rte_compressdev {
	dequeue_pkt_burst_t dequeue_burst;
	/**< Pointer to PMD receive function. */
	enqueue_pkt_burst_t enqueue_burst;
	/**< Pointer to PMD transmit function. */

	struct rte_compressdev_data *data;
	/**< Pointer to device data */
	struct rte_compressdev_ops *dev_ops;
	/**< Functions exported by PMD */
	uint64_t feature_flags;
	/**< Supported features */
	struct rte_device *device;
	/**< Backing device */

	uint8_t driver_id;
	/**< comp driver identifier*/

	struct rte_compressdev_cb_list link_intr_cbs;
	/**< User application callback for interrupts if present */

	__extension__
	uint8_t attached : 1;
	/**< Flag indicating the device is attached */
} __rte_cache_aligned;


/**
 *
 * The data part, with no function pointers, associated with each device.
 *
 * This structure is safe to place in shared memory to be common among
 * different processes in a multi-process configuration.
 */
struct rte_compressdev_data {
	uint8_t dev_id;
	/**< Device ID for this instance */
	uint8_t socket_id;
	/**< Socket ID where memory is allocated */
	char name[RTE_COMPRESSDEV_NAME_MAX_LEN];
	/**< Unique identifier name */

	__extension__
	uint8_t dev_started : 1;
	/**< Device state: STARTED(1)/STOPPED(0) */

	void **queue_pairs;
	/**< Array of pointers to queue pairs. */
	uint16_t nb_queue_pairs;
	/**< Number of device queue pairs. */

	void *dev_private;
	/**< PMD-specific private data */
} __rte_cache_aligned;

extern struct rte_compressdev *rte_compressdevs;
/**
 *
 * Dequeue a burst of processed compression operations from a queue on the comp
 * device. The dequeued operation are stored in *rte_comp_op* structures
 * whose pointers are supplied in the *ops* array.
 *
 * The rte_compressdev_dequeue_burst() function returns the number of ops
 * actually dequeued, which is the number of *rte_comp_op* data structures
 * effectively supplied into the *ops* array.
 *
 * A return value equal to *nb_ops* indicates that the queue contained
 * at least *nb_ops* operations, and this is likely to signify that other
 * processed operations remain in the devices output queue. Applications
 * implementing a "retrieve as many processed operations as possible" policy
 * can check this specific case and keep invoking the
 * rte_compressdev_dequeue_burst() function until a value less than
 * *nb_ops* is returned.
 *
 * The rte_compressdev_dequeue_burst() function does not provide any error
 * notification to avoid the corresponding overhead.
 *
 * @param	dev_id		The compression device identifier
 * @param	qp_id		The index of the queue pair from which to
 *				retrieve processed operations. The value must be
 *				in the range [0, nb_queue_pair - 1] previously
 *				supplied to rte_compressdev_configure().
 * @param	ops		The address of an array of pointers to
 *				*rte_comp_op* structures that must be
 *				large enough to store *nb_ops* pointers in it.
 * @param	nb_ops		The maximum number of operations to dequeue.
 *
 * @return
 *   - The number of operations actually dequeued, which is the number
 *   of pointers to *rte_comp_op* structures effectively supplied to the
 *   *ops* array.
 */
static inline uint16_t
rte_compressdev_dequeue_burst(uint8_t dev_id, uint16_t qp_id,
		struct rte_comp_op **ops, uint16_t nb_ops)
{
	struct rte_compressdev *dev = &rte_compressdevs[dev_id];

	nb_ops = (*dev->dequeue_burst)
			(dev->data->queue_pairs[qp_id], ops, nb_ops);

	return nb_ops;
}

/**
 * Enqueue a burst of operations for processing on a compression device.
 *
 * The rte_compressdev_enqueue_burst() function is invoked to place
 * comp operations on the queue *qp_id* of the device designated by
 * its *dev_id*.
 *
 * The *nb_ops* parameter is the number of operations to process which are
 * supplied in the *ops* array of *rte_comp_op* structures.
 *
 * The rte_compressdev_enqueue_burst() function returns the number of
 * operations it actually enqueued for processing. A return value equal to
 * *nb_ops* means that all packets have been enqueued.
 *
 * @param	dev_id		The identifier of the device.
 * @param	qp_id		The index of the queue pair on which operations
 *				are to be enqueued for processing. The value
 *				must be in the range [0, nb_queue_pairs - 1]
 *				previously supplied to
 *				 *rte_compressdev_configure*.
 * @param	ops		The address of an array of *nb_ops* pointers
 *				to *rte_comp_op* structures which contain
 *				the operations to be processed.
 * @param	nb_ops		The number of operations to process.
 *
 * @return
 * The number of operations actually enqueued on the device. The return
 * value can be less than the value of the *nb_ops* parameter when the
 * comp devices queue is full or if invalid parameters are specified in
 * a *rte_comp_op*.
 */
static inline uint16_t
rte_compressdev_enqueue_burst(uint8_t dev_id, uint16_t qp_id,
		struct rte_comp_op **ops, uint16_t nb_ops)
{
	struct rte_compressdev *dev = &rte_compressdevs[dev_id];

	return (*dev->enqueue_burst)(
			dev->data->queue_pairs[qp_id], ops, nb_ops);
}


/** compressdev session */
struct rte_comp_session {
	__extension__ void *sess_private_data[0];
	/**< Private session material */
};


/**
 * Create symmetric comp session header (generic with no private data)
 *
 * @param   mempool    Symmetric session mempool to allocate session
 *                     objects from
 * @return
 *  - On success return pointer to sym-session
 *  - On failure returns NULL
 */
struct rte_comp_session *
rte_compressdev_session_create(struct rte_mempool *mempool);

/**
 * Frees comp session header, after checking that all
 * the device private data has been freed, returning it
 * to its original mempool.
 *
 * @param   sess     Session header to be freed.
 *
 * @return
 *  - 0 if successful.
 *  - -EINVAL if session is NULL.
 *  - -EBUSY if not all device private data has been freed.
 */
int
rte_compressdev_session_terminate(struct rte_comp_session *sess);

/**
 * Fill out private session data for the device, based on its device id.
 * The same private session data is shared by all devices exposed by a driver
 * so this only needs to be called for one device per driver-type. 
 * All private data stored must be shareable across devices, so read-only.  
 * A session initialised for more than one device (of different driver types)
 * must used the same xform for each init. 
 *
 * @param   dev_id   ID of device that we want the session to be used on
 * @param   sess     Session where the private data will be attached to
 * @param   xforms   comp transform operations to apply on flow
 *                   processed with this session.
 * @param   mempool  Mempool from where the private data should be allocated.
 *
 * @return
 *  - On success, zero.
 *  - -EINVAL if input parameters are invalid.
 *  - -ENOTSUP if comp device does not support the comp transform.
 *  - -ENOMEM if the private session could not be allocated.
 */
int
rte_compressdev_session_init(uint8_t dev_id,
			struct rte_comp_session *sess,
			struct rte_comp_xform *xforms,
			struct rte_mempool *mempool);

/**
 * Frees private data for the device id, based on its device type,
 * returning it to its mempool.
 *
 * @param   dev_id   ID of device that uses the session.
 * @param   sess     Session containing the reference to the private data
 *
 * @return
 *  - 0 if successful.
 *  - -EINVAL if device is invalid or session is NULL.
 */
int
rte_compressdev_session_clear(uint8_t dev_id,
			struct rte_comp_session *sess);

/**
 * Get the size of the header session, for all registered drivers.
 *
 * @return
 *   Size of the header session.
 */
unsigned int
rte_compressdev_get_header_session_size(void);

/**
 * Get the size of the private session data for a device.
 *
 * @param	dev_id		The device identifier.
 *
 * @return
 *   - Size of the private data, if successful
 *   - 0 if device is invalid or does not have private session
 */
unsigned int
rte_compressdev_get_private_session_size(uint8_t dev_id);

/**
 * Attach queue pair with sym session.
 *
 * @param	dev_id		Device to which the session will be attached.
 * @param	qp_id		Queue pair to which the session will be attached
 * @param	session		Session pointer previously allocated by
 *				*rte_compressdev_session_create*.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
int
rte_compressdev_queue_pair_attach_session(uint8_t dev_id, uint16_t qp_id,
		struct rte_comp_session *session);

/**
 * Detach queue pair with comp session.
 *
 * @param	dev_id		Device to which the session is attached.
 * @param	qp_id		Queue pair to which the session is attached.
 * @param	session		Session pointer previously allocated by
 *				*rte_compressdev_session_create*.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */

int
rte_compressdev_queue_pair_detach_session(uint8_t dev_id, uint16_t qp_id,
		struct rte_comp_session *session);
/**
 * This should alloc a stream from the device’s mempool and initialise it.
 * This handle will be passed to the PMD with every op in the stream.
 *
 * @param	dev_id		The identifier of the device.
 * @param	session		Session pointer previously allocated by
 *				*rte_compressdev_session_create*.
 * @param	stream		ptr to where ptr to PMD's private stream data
 *				will be stored. 
 *
 * @return
 *
 * TODO: Should qp_id also be added, with constraint that all ops in the same
 * stream should be sent to the same qp?
 *
 */
int
rte_comp_stream_create(uint8_t dev_id, struct rte_comp_session *sess,
					void ** stream);

/**
 * This should clear the stream and return it to the device’s mempool.
 *
 * @param	dev_id		The identifier of the device.
 *
 * @param	stream		ptr to PMD's private stream data 
 * 
 * 
 * @return
 */
int
rte_comp_stream_free(uint8_t dev_id, void * stream);

/**
 * Provide driver identifier.
 *
 * @param name
 *   The pointer to a driver name.
 * @return
 *  The driver type identifier or -1 if no driver found
 */
int rte_compressdev_driver_id_get(const char *name);

/**
 * Provide driver name.
 *
 * @param driver_id
 *   The driver identifier.
 * @return
 *  The driver name or null if no driver found
 */
const char *rte_compressdev_driver_name_get(uint8_t driver_id);

#ifdef __cplusplus
}
#endif

#endif /* _RTE_COMPRESSDEV_H_ */
