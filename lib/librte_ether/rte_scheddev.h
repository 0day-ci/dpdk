/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
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

#ifndef __INCLUDE_RTE_SCHEDDEV_H__
#define __INCLUDE_RTE_SCHEDDEV_H__

/**
 * @file
 * RTE Generic Hierarchical Scheduler API
 *
 * This interface provides the ability to configure the hierarchical scheduler
 * feature in a generic way.
 */

#include <stdint.h>

#include <rte_red.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Ethernet framing overhead
  *
  * Overhead fields per Ethernet frame:
  * 1. Preamble:                                            7 bytes;
  * 2. Start of Frame Delimiter (SFD):                      1 byte;
  * 3. Inter-Frame Gap (IFG):                              12 bytes.
  */
#define RTE_SCHEDDEV_ETH_FRAMING_OVERHEAD                  20

/**
  * Ethernet framing overhead plus Frame Check Sequence (FCS). Useful when FCS
  * is generated and added at the end of the Ethernet frame on TX side without
  * any SW intervention.
  */
#define RTE_SCHEDDEV_ETH_FRAMING_OVERHEAD_FCS              24

/**< Invalid WRED profile ID */
#define RTE_SCHEDDEV_WRED_PROFILE_ID_NONE                  UINT32_MAX

/**< Invalid shaper profile ID */
#define RTE_SCHEDDEV_SHAPER_PROFILE_ID_NONE                UINT32_MAX

/**< Scheduler hierarchy root node ID */
#define RTE_SCHEDDEV_ROOT_NODE_ID                          UINT32_MAX


/**
  * Scheduler node capabilities
  */
struct rte_scheddev_node_capabilities {
	/**< Private shaper support. */
	int shaper_private_supported;

	/**< Dual rate shaping support for private shaper. Valid only when
	 * private shaper is supported.
	 */
	int shaper_private_dual_rate_supported;

	/**< Minimum committed/peak rate (bytes per second) for private
	 * shaper. Valid only when private shaper is supported.
	 */
	uint64_t shaper_private_rate_min;

	/**< Maximum committed/peak rate (bytes per second) for private
	 * shaper. Valid only when private shaper is supported.
	 */
	uint64_t shaper_private_rate_max;

	/**< Maximum number of supported shared shapers. The value of zero
	 * indicates that shared shapers are not supported.
	 */
	uint32_t shaper_shared_n_max;

	/**< Items valid only for non-leaf nodes. */
	struct {
		/**< Maximum number of children nodes. */
		uint32_t n_children_max;

		/**< Lowest priority supported. The value of 1 indicates that
		 * only priority 0 is supported, which essentially means that
		 * Strict Priority (SP) algorithm is not supported.
		 */
		uint32_t sp_priority_min;

		/**< Maximum number of sibling nodes that can have the same
		 * priority at any given time. When equal to *n_children_max*,
		 * it indicates that WFQ/WRR algorithms are not supported.
		 */
		uint32_t sp_n_children_max;

		/**< WFQ algorithm support. */
		int scheduling_wfq_supported;

		/**< WRR algorithm support. */
		int scheduling_wrr_supported;

		/**< Maximum WFQ/WRR weight. */
		uint32_t scheduling_wfq_wrr_weight_max;
	} nonleaf;

	/**< Items valid only for leaf nodes. */
	struct {
		/**< Head drop algorithm support. */
		int cman_head_drop_supported;

		/**< Private WRED context support. */
		int cman_wred_context_private_supported;

		/**< Maximum number of shared WRED contexts supported. The value
		 * of zero indicates that shared WRED contexts are not
		 * supported.
		 */
		uint32_t cman_wred_context_shared_n_max;
	} leaf;
};

/**
  * Scheduler capabilities
  */
struct rte_scheddev_capabilities {
	/**< Maximum number of nodes. */
	uint32_t n_nodes_max;

	/**< Maximum number of levels (i.e. number of nodes connecting the root
	 * node with any leaf node, including the root and the leaf).
	 */
	uint32_t n_levels_max;

	/**< Maximum number of shapers, either private or shared. In case the
	 * implementation does not share any resource between private and
	 * shared shapers, it is typically equal to the sum between
	 * *shaper_private_n_max* and *shaper_shared_n_max*.
	 */
	uint32_t shaper_n_max;

	/**< Maximum number of private shapers. Indicates the maximum number of
	 * nodes that can concurrently have the private shaper enabled.
	 */
	uint32_t shaper_private_n_max;

	/**< Maximum number of shared shapers. The value of zero indicates that
	  * shared shapers are not supported.
	  */
	uint32_t shaper_shared_n_max;

	/**< Maximum number of nodes that can share the same shared shaper. Only
	  * valid when shared shapers are supported.
	  */
	uint32_t shaper_shared_n_nodes_max;

	/**< Maximum number of shared shapers that can be configured with dual
	  * rate shaping. The value of zero indicates that dual rate shaping
	  * support is not available for shared shapers.
	  */
	uint32_t shaper_shared_dual_rate_n_max;

	/**< Minimum committed/peak rate (bytes per second) for shared
	  * shapers. Only valid when shared shapers are supported.
	  */
	uint64_t shaper_shared_rate_min;

	/**< Maximum committed/peak rate (bytes per second) for shared
	  * shaper. Only valid when shared shapers are supported.
	  */
	uint64_t shaper_shared_rate_max;

	/**< Minimum value allowed for packet length adjustment for
	  * private/shared shapers.
	  */
	int shaper_pkt_length_adjust_min;

	/**< Maximum value allowed for packet length adjustment for
	  * private/shared shapers.
	  */
	int shaper_pkt_length_adjust_max;

	/**< Maximum number of WRED contexts. */
	uint32_t cman_wred_context_n_max;

	/**< Maximum number of private WRED contexts. Indicates the maximum
	  * number of leaf nodes that can concurrently have the private WRED
	  * context enabled.
	  */
	uint32_t cman_wred_context_private_n_max;

	/**< Maximum number of shared WRED contexts. The value of zero indicates
	  * that shared WRED contexts are not supported.
	  */
	uint32_t cman_wred_context_shared_n_max;

	/**< Maximum number of leaf nodes that can share the same WRED context.
	  * Only valid when shared WRED contexts are supported.
	  */
	uint32_t cman_wred_context_shared_n_nodes_max;

	/**< Support for VLAN DEI packet marking. */
	int mark_vlan_dei_supported;

	/**< Support for IPv4/IPv6 ECN marking of TCP packets. */
	int mark_ip_ecn_tcp_supported;

	/**< Support for IPv4/IPv6 ECN marking of SCTP packets. */
	int mark_ip_ecn_sctp_supported;

	/**< Support for IPv4/IPv6 DSCP packet marking. */
	int mark_ip_dscp_supported;

	/**< Summary of node-level capabilities across all nodes. */
	struct rte_scheddev_node_capabilities node;
};

/**
  * Congestion management (CMAN) mode
  *
  * This is used for controlling the admission of packets into a packet queue or
  * group of packet queues on congestion. On request of writing a new packet
  * into the current queue while the queue is full, the *tail drop* algorithm
  * drops the new packet while leaving the queue unmodified, as opposed to *head
  * drop* algorithm, which drops the packet at the head of the queue (the oldest
  * packet waiting in the queue) and admits the new packet at the tail of the
  * queue.
  *
  * The *Random Early Detection (RED)* algorithm works by proactively dropping
  * more and more input packets as the queue occupancy builds up. When the queue
  * is full or almost full, RED effectively works as *tail drop*. The *Weighted
  * RED* algorithm uses a separate set of RED thresholds for each packet color.
  */
enum rte_scheddev_cman_mode {
	RTE_SCHEDDEV_CMAN_TAIL_DROP = 0, /**< Tail drop */
	RTE_SCHEDDEV_CMAN_HEAD_DROP, /**< Head drop */
	RTE_SCHEDDEV_CMAN_WRED, /**< Weighted Random Early Detection (WRED) */
};

/**
  * Color
  */
enum rte_scheddev_color {
	e_RTE_SCHEDDEV_GREEN = 0, /**< Green */
	e_RTE_SCHEDDEV_YELLOW,    /**< Yellow */
	e_RTE_SCHEDDEV_RED,       /**< Red */
	e_RTE_SCHEDDEV_COLORS     /**< Number of colors */
};

/**
  * WRED profile
  */
struct rte_scheddev_wred_params {
	/**< One set of RED parameters per packet color */
	struct rte_red_params red_params[e_RTE_SCHEDDEV_COLORS];
};

/**
  * Token bucket
  */
struct rte_scheddev_token_bucket {
	/**< Token bucket rate (bytes per second) */
	uint64_t rate;

	/**< Token bucket size (bytes), a.k.a. max burst size */
	uint64_t size;
};

/**
  * Shaper (rate limiter) profile
  *
  * Multiple shaper instances can share the same shaper profile. Each node has
  * zero or one private shaper (only one node using it) and/or zero, one or
  * several shared shapers (multiple nodes use the same shaper instance).
  *
  * Single rate shapers use a single token bucket. A single rate shaper can be
  * configured by setting the rate of the committed bucket to zero, which
  * effectively disables this bucket. The peak bucket is used to limit the rate
  * and the burst size for the current shaper.
  *
  * Dual rate shapers use both the committed and the peak token buckets. The
  * rate of the committed bucket has to be less than or equal to the rate of the
  * peak bucket.
  */
struct rte_scheddev_shaper_params {
	/**< Committed token bucket */
	struct rte_scheddev_token_bucket committed;

	/**< Peak token bucket */
	struct rte_scheddev_token_bucket peak;

	/**< Signed value to be added to the length of each packet for the
	 * purpose of shaping. Can be used to correct the packet length with
	 * the framing overhead bytes that are also consumed on the wire (e.g.
	 * RTE_SCHEDDEV_ETH_FRAMING_OVERHEAD_FCS).
	 */
	int32_t pkt_length_adjust;
};

/**
  * Node parameters
  *
  * Each scheduler hierarchy node has multiple inputs (children nodes of the
  * current parent node) and a single output (which is input to its parent
  * node). The current node arbitrates its inputs using Strict Priority (SP),
  * Weighted Fair Queuing (WFQ) and Weighted Round Robin (WRR) algorithms to
  * schedule input packets on its output while observing its shaping (rate
  * limiting) constraints.
  *
  * Algorithms such as byte-level WRR, Deficit WRR (DWRR), etc are considered
  * approximations of the ideal of WFQ and are assimilated to WFQ, although
  * an associated implementation-dependent trade-off on accuracy, performance
  * and resource usage might exist.
  *
  * Children nodes with different priorities are scheduled using the SP
  * algorithm, based on their priority, with zero (0) as the highest priority.
  * Children with same priority are scheduled using the WFQ or WRR algorithm,
  * based on their weight, which is relative to the sum of the weights of all
  * siblings with same priority, with one (1) as the lowest weight.
  *
  * Each leaf node sits on on top of a TX queue of the current Ethernet port.
  * Therefore, the leaf nodes are predefined with the node IDs of 0 .. (N-1),
  * where N is the number of TX queues configured for the current Ethernet port.
  * The non-leaf nodes have their IDs generated by the application.
  */
struct rte_scheddev_node_params {
	/**< Shaper profile for the private shaper. The absence of the private
	 * shaper for the current node is indicated by setting this parameter
	 * to RTE_SCHEDDEV_SHAPER_PROFILE_ID_NONE.
	 */
	uint32_t shaper_profile_id;

	/**< User allocated array of valid shared shaper IDs. */
	uint32_t *shared_shaper_id;

	/**< Number of shared shaper IDs in the *shared_shaper_id* array. */
	uint32_t n_shared_shapers;

	union {
		/**< Parameters only valid for non-leaf nodes. */
		struct {
			/**< For each priority, indicates whether the children
			 * nodes sharing the same priority are to be scheduled
			 * by WFQ or by WRR. When NULL, it indicates that WFQ
			 * is to be used for all priorities. When non-NULL, it
			 * points to a pre-allocated array of *n_priority*
			 * elements, with a non-zero value element indicating
			 * WFQ and a zero value element for WRR.
			 */
			int *scheduling_mode_per_priority;

			/**< Number of priorities. */
			uint32_t n_priorities;
		} nonleaf;

		/**< Parameters only valid for leaf nodes. */
		struct {
			/**< Congestion management mode */
			enum rte_scheddev_cman_mode cman;

			/**< WRED parameters (valid when *cman* is WRED). */
			struct {
				/**< WRED profile for private WRED context. */
				uint32_t wred_profile_id;

				/**< User allocated array of shared WRED context
				 * IDs. The absence of a private WRED context
				 * for current leaf node is indicated by value
				 * RTE_SCHEDDEV_WRED_PROFILE_ID_NONE.
				 */
				uint32_t *shared_wred_context_id;

				/**< Number of shared WRED context IDs in the
				 * *shared_wred_context_id* array.
				 */
				uint32_t n_shared_wred_contexts;
			} wred;
		} leaf;
	};
};

/**
  * Node statistics counter type
  */
enum rte_scheddev_stats_counter {
	/**< Number of packets scheduled from current node. */
	RTE_SCHEDDEV_STATS_COUNTER_N_PKTS = 1 << 0,

	/**< Number of bytes scheduled from current node. */
	RTE_SCHEDDEV_STATS_COUNTER_N_BYTES = 1 << 1,

	/**< Number of packets dropped by current node.  */
	RTE_SCHEDDEV_STATS_COUNTER_N_PKTS_DROPPED = 1 << 2,

	/**< Number of bytes dropped by current node.  */
	RTE_SCHEDDEV_STATS_COUNTER_N_BYTES_DROPPED = 1 << 3,

	/**< Number of packets currently waiting in the packet queue of current
	 * leaf node.
	 */
	RTE_SCHEDDEV_STATS_COUNTER_N_PKTS_QUEUED = 1 << 4,

	/**< Number of bytes currently waiting in the packet queue of current
	 * leaf node.
	 */
	RTE_SCHEDDEV_STATS_COUNTER_N_BYTES_QUEUED = 1 << 5,
};

/**
  * Node statistics counters
  */
struct rte_scheddev_node_stats {
	/**< Number of packets scheduled from current node. */
	uint64_t n_pkts;

	/**< Number of bytes scheduled from current node. */
	uint64_t n_bytes;

	/**< Statistics counters for leaf nodes only. */
	struct {
		/**< Number of packets dropped by current leaf node. */
		uint64_t n_pkts_dropped;

		/**< Number of bytes dropped by current leaf node. */
		uint64_t n_bytes_dropped;

		/**< Number of packets currently waiting in the packet queue of
		 * current leaf node.
		 */
		uint64_t n_pkts_queued;

		/**< Number of bytes currently waiting in the packet queue of
		 * current leaf node.
		 */
		uint64_t n_bytes_queued;
	} leaf;
};

/**
 * Verbose error types.
 *
 * Most of them provide the type of the object referenced by struct
 * rte_scheddev_error::cause.
 */
enum rte_scheddev_error_type {
	RTE_SCHEDDEV_ERROR_TYPE_NONE, /**< No error. */
	RTE_SCHEDDEV_ERROR_TYPE_UNSPECIFIED, /**< Cause unspecified. */
	RTE_SCHEDDEV_ERROR_TYPE_WRED_PROFILE,
	RTE_SCHEDDEV_ERROR_TYPE_WRED_PROFILE_GREEN,
	RTE_SCHEDDEV_ERROR_TYPE_WRED_PROFILE_YELLOW,
	RTE_SCHEDDEV_ERROR_TYPE_WRED_PROFILE_RED,
	RTE_SCHEDDEV_ERROR_TYPE_WRED_PROFILE_ID,
	RTE_SCHEDDEV_ERROR_TYPE_SHARED_WRED_CONTEXT_ID,
	RTE_SCHEDDEV_ERROR_TYPE_SHAPER_PROFILE,
	RTE_SCHEDDEV_ERROR_TYPE_SHARED_SHAPER_ID,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_PARAMS,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_PARAMS_PARENT_NODE_ID,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_PARAMS_PRIORITY,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_PARAMS_WEIGHT,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_PARAMS_SCHEDULING_MODE,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_PARAMS_SHAPER_PROFILE_ID,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_PARAMS_SHARED_SHAPER_ID,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_PARAMS_LEAF,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_PARAMS_LEAF_CMAN,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_PARAMS_LEAF_WRED_PROFILE_ID,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_PARAMS_LEAF_SHARED_WRED_CONTEXT_ID,
	RTE_SCHEDDEV_ERROR_TYPE_NODE_ID,
};

/**
 * Verbose error structure definition.
 *
 * This object is normally allocated by applications and set by PMDs, the
 * message points to a constant string which does not need to be freed by
 * the application, however its pointer can be considered valid only as long
 * as its associated DPDK port remains configured. Closing the underlying
 * device or unloading the PMD invalidates it.
 *
 * Both cause and message may be NULL regardless of the error type.
 */
struct rte_scheddev_error {
	enum rte_scheddev_error_type type; /**< Cause field and error type. */
	const void *cause; /**< Object responsible for the error. */
	const char *message; /**< Human-readable error message. */
};

/**
 * Scheduler capabilities get
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param cap
 *   Scheduler capabilities. Needs to be pre-allocated and valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_capabilities_get(uint8_t port_id,
	struct rte_scheddev_capabilities *cap,
	struct rte_scheddev_error *error);

/**
 * Scheduler node capabilities get
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid.
 * @param cap
 *   Scheduler node capabilities. Needs to be pre-allocated and valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_capabilities_get(uint8_t port_id,
	uint32_t node_id,
	struct rte_scheddev_node_capabilities *cap,
	struct rte_scheddev_error *error);

/**
 * Scheduler WRED profile add
 *
 * Create a new WRED profile with ID set to *wred_profile_id*. The new profile
 * is used to create one or several WRED contexts.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param wred_profile_id
 *   WRED profile ID for the new profile. Needs to be unused.
 * @param profile
 *   WRED profile parameters. Needs to be pre-allocated and valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_wred_profile_add(uint8_t port_id,
	uint32_t wred_profile_id,
	struct rte_scheddev_wred_params *profile,
	struct rte_scheddev_error *error);

/**
 * Scheduler WRED profile delete
 *
 * Delete an existing WRED profile. This operation fails when there is currently
 * at least one user (i.e. WRED context) of this WRED profile.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param wred_profile_id
 *   WRED profile ID. Needs to be the valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_wred_profile_delete(uint8_t port_id,
	uint32_t wred_profile_id,
	struct rte_scheddev_error *error);

/**
 * Scheduler shared WRED context add or update
 *
 * When *shared_wred_context_id* is invalid, a new WRED context with this ID is
 * created by using the WRED profile identified by *wred_profile_id*.
 *
 * When *shared_wred_context_id* is valid, this WRED context is no longer using
 * the profile previously assigned to it and is updated to use the profile
 * identified by *wred_profile_id*.
 *
 * A valid shared WRED context can be assigned to several scheduler hierarchy
 * leaf nodes configured to use WRED as the congestion management mode.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param shared_wred_context_id
 *   Shared WRED context ID
 * @param wred_profile_id
 *   WRED profile ID. Needs to be the valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_shared_wred_context_add_update(uint8_t port_id,
	uint32_t shared_wred_context_id,
	uint32_t wred_profile_id,
	struct rte_scheddev_error *error);

/**
 * Scheduler shared WRED context delete
 *
 * Delete an existing shared WRED context. This operation fails when there is
 * currently at least one user (i.e. scheduler hierarchy leaf node) of this
 * shared WRED context.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param shared_wred_context_id
 *   Shared WRED context ID. Needs to be the valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_shared_wred_context_delete(uint8_t port_id,
	uint32_t shared_wred_context_id,
	struct rte_scheddev_error *error);

/**
 * Scheduler shaper profile add
 *
 * Create a new shaper profile with ID set to *shaper_profile_id*. The new
 * shaper profile is used to create one or several shapers.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param shaper_profile_id
 *   Shaper profile ID for the new profile. Needs to be unused.
 * @param profile
 *   Shaper profile parameters. Needs to be pre-allocated and valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_shaper_profile_add(uint8_t port_id,
	uint32_t shaper_profile_id,
	struct rte_scheddev_shaper_params *profile,
	struct rte_scheddev_error *error);

/**
 * Scheduler shaper profile delete
 *
 * Delete an existing shaper profile. This operation fails when there is
 * currently at least one user (i.e. shaper) of this shaper profile.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param shaper_profile_id
 *   Shaper profile ID. Needs to be the valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_shaper_profile_delete(uint8_t port_id,
	uint32_t shaper_profile_id,
	struct rte_scheddev_error *error);

/**
 * Scheduler shared shaper add or update
 *
 * When *shared_shaper_id* is not a valid shared shaper ID, a new shared shaper
 * with this ID is created using the shaper profile identified by
 * *shaper_profile_id*.
 *
 * When *shared_shaper_id* is a valid shared shaper ID, this shared shaper is no
 * longer using the shaper profile previously assigned to it and is updated to
 * use the shaper profile identified by *shaper_profile_id*.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param shared_shaper_id
 *   Shared shaper ID
 * @param shaper_profile_id
 *   Shaper profile ID. Needs to be the valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_shared_shaper_add_update(uint8_t port_id,
	uint32_t shared_shaper_id,
	uint32_t shaper_profile_id,
	struct rte_scheddev_error *error);

/**
 * Scheduler shared shaper delete
 *
 * Delete an existing shared shaper. This operation fails when there is
 * currently at least one user (i.e. scheduler hierarchy node) of this shared
 * shaper.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param shared_shaper_id
 *   Shared shaper ID. Needs to be the valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_shared_shaper_delete(uint8_t port_id,
	uint32_t shared_shaper_id,
	struct rte_scheddev_error *error);

/**
 * Scheduler node add
 *
 * When *node_id* is not a valid node ID, a new node with this ID is created and
 * connected as child to the existing node identified by *parent_node_id*.
 *
 * When *node_id* is a valid node ID, this node is disconnected from its current
 * parent and connected as child to another existing node identified by
 * *parent_node_id *.
 *
 * This function can be called during port initialization phase (before the
 * Ethernet port is started) for building the scheduler start-up hierarchy.
 * Subject to the specific Ethernet port supporting on-the-fly scheduler
 * hierarchy updates, this function can also be called during run-time (after
 * the Ethernet port is started).
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID
 * @param parent_node_id
 *   Parent node ID. Needs to be the valid.
 * @param priority
 *   Node priority. The highest node priority is zero. Used by the SP algorithm
 *   running on the parent of the current node for scheduling this child node.
 * @param weight
 *   Node weight. The node weight is relative to the weight sum of all siblings
 *   that have the same priority. The lowest weight is one. Used by the WFQ/WRR
 *   algorithm running on the parent of the current node for scheduling this
 *   child node.
 * @param params
 *   Node parameters. Needs to be pre-allocated and valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_add(uint8_t port_id,
	uint32_t node_id,
	uint32_t parent_node_id,
	uint32_t priority,
	uint32_t weight,
	struct rte_scheddev_node_params *params,
	struct rte_scheddev_error *error);

/**
 * Scheduler node delete
 *
 * Delete an existing node. This operation fails when this node currently has at
 * least one user (i.e. child node).
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_delete(uint8_t port_id,
	uint32_t node_id,
	struct rte_scheddev_error *error);

/**
 * Scheduler node suspend
 *
 * Suspend an existing node.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_suspend(uint8_t port_id,
	uint32_t node_id,
	struct rte_scheddev_error *error);

/**
 * Scheduler node resume
 *
 * Resume an existing node that was previously suspended.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_resume(uint8_t port_id,
	uint32_t node_id,
	struct rte_scheddev_error *error);

/**
 * Scheduler hierarchy set
 *
 * This function is called during the port initialization phase (before the
 * Ethernet port is started) to freeze the scheduler start-up hierarchy.
 *
 * This function fails when the currently configured scheduler hierarchy is not
 * supported by the Ethernet port, in which case the user can abort or try out
 * another hierarchy configuration (e.g. a hierarchy with less leaf nodes),
 * which can be build from scratch (when *clear_on_fail* is enabled) or by
 * modifying the existing hierarchy configuration (when *clear_on_fail* is
 * disabled).
 *
 * Note that, even when the configured scheduler hierarchy is supported (so this
 * function is successful), the Ethernet port start might still fail due to e.g.
 * not enough memory being available in the system, etc.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param clear_on_fail
 *   On function call failure, hierarchy is cleared when this parameter is
 *   non-zero and preserved when this parameter is equal to zero.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_hierarchy_set(uint8_t port_id,
	int clear_on_fail,
	struct rte_scheddev_error *error);

/**
 * Scheduler node parent update
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid.
 * @param parent_node_id
 *   Node ID for the new parent. Needs to be valid.
 * @param priority
 *   Node priority. The highest node priority is zero. Used by the SP algorithm
 *   running on the parent of the current node for scheduling this child node.
 * @param weight
 *   Node weight. The node weight is relative to the weight sum of all siblings
 *   that have the same priority. The lowest weight is zero. Used by the WFQ/WRR
 *   algorithm running on the parent of the current node for scheduling this
 *   child node.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_parent_update(uint8_t port_id,
	uint32_t node_id,
	uint32_t parent_node_id,
	uint32_t priority,
	uint32_t weight,
	struct rte_scheddev_error *error);

/**
 * Scheduler node private shaper update
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid.
 * @param shaper_profile_id
 *   Shaper profile ID for the private shaper of the current node. Needs to be
 *   either valid shaper profile ID or RTE_SCHEDDEV_SHAPER_PROFILE_ID_NONE, with
 *   the latter disabling the private shaper of the current node.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_shaper_update(uint8_t port_id,
	uint32_t node_id,
	uint32_t shaper_profile_id,
	struct rte_scheddev_error *error);

/**
 * Scheduler node shared shapers update
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid.
 * @param shared_shaper_id
 *   Shared shaper ID. Needs to be valid.
 * @param add
 *   Set to non-zero value to add this shared shaper to current node or to zero
 *   to delete this shared shaper from current node.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_shared_shaper_update(uint8_t port_id,
	uint32_t node_id,
	uint32_t shared_shaper_id,
	int add,
	struct rte_scheddev_error *error);

/**
 * Scheduler node scheduling mode update
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid leaf node ID.
 * @param scheduling_mode_per_priority
 *   For each priority, indicates whether the children nodes sharing the same
 *   priority are to be scheduled by WFQ or by WRR. When NULL, it indicates that
 *   WFQ is to be used for all priorities. When non-NULL, it points to a
 *   pre-allocated array of *n_priority* elements, with a non-zero value element
 *   indicating WFQ and a zero value element for WRR.
 * @param n_priorities
 *   Number of priorities.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_scheduling_mode_update(uint8_t port_id,
	uint32_t node_id,
	int *scheduling_mode_per_priority,
	uint32_t n_priorities,
	struct rte_scheddev_error *error);

/**
 * Scheduler node congestion management mode update
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid leaf node ID.
 * @param cman
 *   Congestion management mode.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_cman_update(uint8_t port_id,
	uint32_t node_id,
	enum rte_scheddev_cman_mode cman,
	struct rte_scheddev_error *error);

/**
 * Scheduler node private WRED context update
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid leaf node ID.
 * @param wred_profile_id
 *   WRED profile ID for the private WRED context of the current node. Needs to
 *   be either valid WRED profile ID or RTE_SCHEDDEV_WRED_PROFILE_ID_NONE, with
 *   the latter disabling the private WRED context of the current node.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_wred_context_update(uint8_t port_id,
	uint32_t node_id,
	uint32_t wred_profile_id,
	struct rte_scheddev_error *error);

/**
 * Scheduler node shared WRED context update
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid leaf node ID.
 * @param shared_wred_context_id
 *   Shared WRED context ID. Needs to be valid.
 * @param add
 *   Set to non-zero value to add this shared WRED context to current node or to
 *   zero to delete this shared WRED context from current node.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_shared_wred_context_update(uint8_t port_id,
	uint32_t node_id,
	uint32_t shared_wred_context_id,
	int add,
	struct rte_scheddev_error *error);

/**
 * Scheduler packet marking - VLAN DEI (IEEE 802.1Q)
 *
 * IEEE 802.1p maps the traffic class to the VLAN Priority Code Point (PCP)
 * field (3 bits), while IEEE 802.1q maps the drop priority to the VLAN Drop
 * Eligible Indicator (DEI) field (1 bit), which was previously named Canonical
 * Format Indicator (CFI).
 *
 * All VLAN frames of a given color get their DEI bit set if marking is enabled
 * for this color; otherwise, their DEI bit is left as is (either set or not).
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param mark_green
 *   Set to non-zero value to enable marking of green packets and to zero to
 *   disable it.
 * @param mark_yellow
 *   Set to non-zero value to enable marking of yellow packets and to zero to
 *   disable it.
 * @param mark_red
 *   Set to non-zero value to enable marking of red packets and to zero to
 *   disable it.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_mark_vlan_dei(uint8_t port_id,
	int mark_green,
	int mark_yellow,
	int mark_red,
	struct rte_scheddev_error *error);

/**
 * Scheduler packet marking - IPv4 / IPv6 ECN (IETF RFC 3168)
 *
 * IETF RFCs 2474 and 3168 reorganize the IPv4 Type of Service (TOS) field
 * (8 bits) and the IPv6 Traffic Class (TC) field (8 bits) into Differentiated
 * Services Codepoint (DSCP) field (6 bits) and Explicit Congestion Notification
 * (ECN) field (2 bits). The DSCP field is typically used to encode the traffic
 * class and/or drop priority (RFC 2597), while the ECN field is used by RFC
 * 3168 to implement a congestion notification mechanism to be leveraged by
 * transport layer protocols such as TCP and SCTP that have congestion control
 * mechanisms.
 *
 * When congestion is experienced, as alternative to dropping the packet,
 * routers can change the ECN field of input packets from 2'b01 or 2'b10 (values
 * indicating that source endpoint is ECN-capable) to 2'b11 (meaning that
 * congestion is experienced). The destination endpoint can use the ECN-Echo
 * (ECE) TCP flag to relay the congestion indication back to the source
 * endpoint, which acknowledges it back to the destination endpoint with the
 * Congestion Window Reduced (CWR) TCP flag.
 *
 * All IPv4/IPv6 packets of a given color with ECN set to 2’b01 or 2’b10
 * carrying TCP or SCTP have their ECN set to 2’b11 if the marking feature is
 * enabled for the current color, otherwise the ECN field is left as is.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param mark_green
 *   Set to non-zero value to enable marking of green packets and to zero to
 *   disable it.
 * @param mark_yellow
 *   Set to non-zero value to enable marking of yellow packets and to zero to
 *   disable it.
 * @param mark_red
 *   Set to non-zero value to enable marking of red packets and to zero to
 *   disable it.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_mark_ip_ecn(uint8_t port_id,
	int mark_green,
	int mark_yellow,
	int mark_red,
	struct rte_scheddev_error *error);

/**
 * Scheduler packet marking - IPv4 / IPv6 DSCP (IETF RFC 2597)
 *
 * IETF RFC 2597 maps the traffic class and the drop priority to the IPv4/IPv6
 * Differentiated Services Codepoint (DSCP) field (6 bits). Here are the DSCP
 * values proposed by this RFC:
 *
 *                       Class 1    Class 2    Class 3    Class 4
 *                     +----------+----------+----------+----------+
 *    Low Drop Prec    |  001010  |  010010  |  011010  |  100010  |
 *    Medium Drop Prec |  001100  |  010100  |  011100  |  100100  |
 *    High Drop Prec   |  001110  |  010110  |  011110  |  100110  |
 *                     +----------+----------+----------+----------+
 *
 * There are 4 traffic classes (classes 1 .. 4) encoded by DSCP bits 1 and 2, as
 * well as 3 drop priorities (low/medium/high) encoded by DSCP bits 3 and 4.
 *
 * All IPv4/IPv6 packets have their color marked into DSCP bits 3 and 4 as
 * follows: green mapped to Low Drop Precedence (2’b01), yellow to Medium
 * (2’b10) and red to High (2’b11). Marking needs to be explicitly enabled
 * for each color; when not enabled for a given color, the DSCP field of all
 * packets with that color is left as is.
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param mark_green
 *   Set to non-zero value to enable marking of green packets and to zero to
 *   disable it.
 * @param mark_yellow
 *   Set to non-zero value to enable marking of yellow packets and to zero to
 *   disable it.
 * @param mark_red
 *   Set to non-zero value to enable marking of red packets and to zero to
 *   disable it.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_mark_ip_dscp(uint8_t port_id,
	int mark_green,
	int mark_yellow,
	int mark_red,
	struct rte_scheddev_error *error);

/**
 * Scheduler get statistics counter types enabled for all nodes
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param nonleaf_node_capability_stats_mask
 *   Statistics counter types available per node for all non-leaf nodes. Needs
 *   to be pre-allocated.
 * @param nonleaf_node_enabled_stats_mask
 *   Statistics counter types currently enabled per node for each non-leaf node.
 *   This is a subset of *nonleaf_node_capability_stats_mask*. Needs to be
 *   pre-allocated.
 * @param leaf_node_capability_stats_mask
 *   Statistics counter types available per node for all leaf nodes. Needs to
 *   be pre-allocated.
 * @param leaf_node_enabled_stats_mask
 *   Statistics counter types currently enabled for each leaf node. This is
 *   a subset of *leaf_node_capability_stats_mask*. Needs to be pre-allocated.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_stats_get_enabled(uint8_t port_id,
	uint64_t *nonleaf_node_capability_stats_mask,
	uint64_t *nonleaf_node_enabled_stats_mask,
	uint64_t *leaf_node_capability_stats_mask,
	uint64_t *leaf_node_enabled_stats_mask,
	struct rte_scheddev_error *error);

/**
 * Scheduler enable selected statistics counters for all nodes
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param nonleaf_node_enabled_stats_mask
 *   Statistics counter types to be enabled per node for each non-leaf node.
 *   This needs to be a subset of the statistics counter types available per
 *   node for all non-leaf nodes. Any statistics counter type not included in
 *   this set is to be disabled for all non-leaf nodes.
 * @param leaf_node_enabled_stats_mask
 *   Statistics counter types to be enabled per node for each leaf node. This
 *   needs to be a subset of the statistics counter types available per node for
 *   all leaf nodes. Any statistics counter type not included in this set is to
 *   be disabled for all leaf nodes.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_stats_enable(uint8_t port_id,
	uint64_t nonleaf_node_enabled_stats_mask,
	uint64_t leaf_node_enabled_stats_mask,
	struct rte_scheddev_error *error);

/**
 * Scheduler get statistics counter types enabled for current node
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid.
 * @param capability_stats_mask
 *   Statistics counter types available for the current node. Needs to be
 *   pre-allocated.
 * @param enabled_stats_mask
 *   Statistics counter types currently enabled for the current node. This is
 *   a subset of *capability_stats_mask*. Needs to be pre-allocated.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_stats_get_enabled(uint8_t port_id,
	uint32_t node_id,
	uint64_t *capability_stats_mask,
	uint64_t *enabled_stats_mask,
	struct rte_scheddev_error *error);

/**
 * Scheduler enable selected statistics counters for current node
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid.
 * @param enabled_stats_mask
 *   Statistics counter types to be enabled for the current node. This needs to
 *   be a subset of the statistics counter types available for the current node.
 *   Any statistics counter type not included in this set is to be disabled for
 *   the current node.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_stats_enable(uint8_t port_id,
	uint32_t node_id,
	uint64_t enabled_stats_mask,
	struct rte_scheddev_error *error);

/**
 * Scheduler node statistics counters read
 *
 * @param port_id
 *   The port identifier of the Ethernet device.
 * @param node_id
 *   Node ID. Needs to be valid.
 * @param stats
 *   When non-NULL, it contains the current value for the statistics counters
 *   enabled for the current node.
 * @param clear
 *   When this parameter has a non-zero value, the statistics counters are
 *   cleared (i.e. set to zero) immediately after they have been read, otherwise
 *   the statistics counters are left untouched.
 * @param error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int rte_scheddev_node_stats_read(uint8_t port_id,
	uint32_t node_id,
	struct rte_scheddev_node_stats *stats,
	int clear,
	struct rte_scheddev_error *error);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_RTE_SCHEDDEV_H__ */
