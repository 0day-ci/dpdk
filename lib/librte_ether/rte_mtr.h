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

#ifndef __INCLUDE_RTE_MTR_H__
#define __INCLUDE_RTE_MTR_H__

/**
 * @file
 * RTE Generic Traffic Metering and Policing API
 *
 * This interface provides the ability to configure the traffic metering and
 * policing (MTR) in a generic way.
 *
 * The processing done for each input packet hitting a MTR object is:
 *    A) Traffic metering: The packet is assigned a color (the meter output
 *       color), based on the previous history of the flow reflected in the
 *       current state of the MTR object, according to the specific traffic
 *       metering algorithm. The traffic metering algorithm can typically work
 *       in color aware mode, in which case the input packet already has an
 *       initial color (the input color), or in color blind mode, which is
 *       equivalent to considering all input packets initially colored as green.
 *    B) Policing: There is a separate policer action configured for each meter
 *       output color, which can:
 *          a) Drop the packet.
 *          b) Keep the same packet color: the policer output color matches the
 *             meter output color (essentially a no-op action).
 *          c) Recolor the packet: the policer output color is different than
 *             the meter output color.
 *       The policer output color is the output color of the packet, which is
 *       set in the packet meta-data (i.e. struct rte_mbuf::sched::color).
 *    C) Statistics: The set of counters maintained for each MTR object is
 *       configurable and subject to the implementation support. This set
 *       includes the number of packets and bytes dropped or passed for each
 *       output color.
 *
 * Once successfully created, an MTR object is linked to one or several flows
 * through the meter action of the flow API.
 *    A) Whether an MTR object is private to a flow or potentially shared by
 *       several flows has to be specified at creation time.
 *    B) Several meter actions can be potentially registered for the same flow.
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Color
 */
enum rte_mtr_color {
	RTE_MTR_GREEN = 0, /**< Green */
	RTE_MTR_YELLOW, /**< Yellow */
	RTE_MTR_RED, /**< Red */
	RTE_MTR_COLORS /**< Number of colors */
};

/**
 * Statistics counter type
 */
enum rte_mtr_stats_type {
	/**< Number of packets passed as green by the policer.  */
	RTE_MTR_STATS_N_PKTS_GREEN = 1 << 0,

	/**< Number of bytes passed as green by the policer.  */
	RTE_MTR_STATS_N_BYTES_GREEN = 1 << 1,

	/**< Number of packets passed as yellow by the policer.  */
	RTE_MTR_STATS_N_PKTS_YELLOW = 1 << 2,

	/**< Number of bytes passed as yellow by the policer.  */
	RTE_MTR_STATS_N_BYTES_YELLOW = 1 << 3,

	/**< Number of packets passed as red by the policer.  */
	RTE_MTR_STATS_N_PKTS_RED = 1 << 4,

	/**< Number of bytes passed as red by the policer.  */
	RTE_MTR_STATS_N_BYTES_RED = 1 << 5,

	/**< Number of packets dropped by the policer.  */
	RTE_MTR_STATS_N_PKTS_DROPPED = 1 << 6,

	/**< Number of bytes dropped by the policer.  */
	RTE_MTR_STATS_N_BYTES_DROPPED = 1 << 7,
};

/**
 * Statistics counters
 */
struct rte_mtr_stats {
	/**< Number of packets passed by the policer (per color). */
	uint64_t n_pkts[RTE_MTR_COLORS];

	/**< Number of bytes passed by the policer (per color). */
	uint64_t n_bytes[RTE_MTR_COLORS];

	/**< Number of packets dropped by the policer. */
	uint64_t n_pkts_dropped;

	/**< Number of bytes passed by the policer. */
	uint64_t n_bytes_dropped;
};

/**
 * Traffic metering algorithms
 */
enum rte_mtr_algorithm {
	/**< Single Rate Three Color Marker (srTCM) - IETF RFC 2697. */
	RTE_MTR_SRTCM_RFC2697 = 0,

	/**< Two Rate Three Color Marker (trTCM) - IETF RFC 2698. */
	RTE_MTR_TRTCM_RFC2698,
};

/**
 * Meter profile
 */
struct rte_mtr_meter_profile {
	/**<  Traffic metering algorithm. */
	enum rte_mtr_algorithm alg;

	union {
		/**< Items only valid when *alg* is set to srTCM - RFC2697. */
		struct {
			/**< Committed Information Rate (CIR) (bytes/second). */
			uint64_t cir;

			/**< Committed Burst Size (CBS) (bytes). */
			uint64_t cbs;

			/**< Excess Burst Size (EBS) (bytes). */
			uint64_t ebs;

			/**< Non-zero for color aware mode, zero for color blind
			 * mode. In color aware mode, the packet input color is
			 * read from the IPv4/IPv6 DSCP field, as defined by
			 * IETF RFC 2597 (low/medium/high drop precedence
			 * translates to green/yellow/red color respectively).
			 */
			int color_aware;
		} srtcm_rfc2697;

		/**< Items only valid when *alg* is set to trTCM - RFC2698. */
		struct {
			/**< Committed Information Rate (CIR) (bytes/second). */
			uint64_t cir;

			/**< Peak Information Rate (PIR) (bytes/second). */
			uint64_t pir;

			/**< Committed Burst Size (CBS) (byes). */
			uint64_t cbs;

			/**< Peak Burst Size (PBS) (bytes). */
			uint64_t pbs;

			/**< Non-zero for color aware mode, zero for color blind
			 * mode. In color aware mode, the packet input color is
			 * read from the IPv4/IPv6 DSCP field, as defined by
			 * IETF RFC 2597 (low/medium/high drop precedence
			 * translates to green/yellow/red color respectively).
			 */
			int color_aware;
		} trtcm_rfc2698;
	};
};

/**
 * Policer actions
 */
enum rte_mtr_policer_action {
	/**< Recolor the packet as green. */
	e_MTR_POLICER_ACTION_COLOR_GREEN = 0,

	/**< Recolor the packet as yellow. */
	e_MTR_POLICER_ACTION_COLOR_YELLOW,

	/**< Recolor the packet as red. */
	e_MTR_POLICER_ACTION_COLOR_RED,

	/**< Drop the packet. */
	e_MTR_POLICER_ACTION_DROP,
};

/**
 * Parameters for each traffic metering & policing object
 *
 * @see enum rte_mtr_stats_type
 */
struct rte_mtr_params {
	/**< Meter profile ID. */
	uint32_t meter_profile_id;

	/**< Policer actions (per meter output color). */
	enum rte_mtr_policer_action action[RTE_MTR_COLORS];

	/**< Set of stats counters to be enabled. */
	uint64_t stats_mask;
};

/**
 * Verbose error types.
 *
 * Most of them provide the type of the object referenced by struct
 * rte_mtr_error::cause.
 */
enum rte_mtr_error_type {
	RTE_MTR_ERROR_TYPE_NONE, /**< No error. */
	RTE_MTR_ERROR_TYPE_UNSPECIFIED, /**< Cause unspecified. */
	RTE_MTR_ERROR_TYPE_METER_PROFILE_ID,
	RTE_MTR_ERROR_TYPE_METER_PROFILE,
	RTE_MTR_ERROR_TYPE_MTR_ID,
	RTE_MTR_ERROR_TYPE_MTR_PARAMS,
	RTE_MTR_ERROR_TYPE_POLICER_ACTION_GREEN,
	RTE_MTR_ERROR_TYPE_POLICER_ACTION_YELLOW,
	RTE_MTR_ERROR_TYPE_POLICER_ACTION_RED,
	RTE_MTR_ERROR_TYPE_STATS_MASK,
	RTE_MTR_ERROR_TYPE_STATS,
	RTE_MTR_ERROR_TYPE_SHARED,
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
struct rte_mtr_error {
	enum rte_mtr_error_type type; /**< Cause field and error type. */
	const void *cause; /**< Object responsible for the error. */
	const char *message; /**< Human-readable error message. */
};

/**
 * Meter profile add
 *
 * Create a new meter profile with ID set to *meter_profile_id*. The new profile
 * is used to create one or several MTR objects.
 *
 * @param[in] port_id
 *   The port identifier of the Ethernet device.
 * @param[in] meter_profile_id
 *   ID for the new meter profile. Needs to be unused by any of the existing
 *   meter profiles added for the current port.
 * @param[in] profile
 *   Meter profile parameters. Needs to be pre-allocated and valid.
 * @param[out] error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int
rte_mtr_meter_profile_add(uint8_t port_id,
	uint32_t meter_profile_id,
	struct rte_mtr_meter_profile *profile,
	struct rte_mtr_error *error);

/**
 * Meter profile delete
 *
 * Delete an existing meter profile. This operation fails when there is
 * currently at least one user (i.e. MTR object) of this profile.
 *
 * @param[in] port_id
 *   The port identifier of the Ethernet device.
 * @param[in] meter_profile_id
 *   Meter profile ID. Needs to be the valid.
 * @param[out] error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int
rte_mtr_meter_profile_delete(uint8_t port_id,
	uint32_t meter_profile_id,
	struct rte_mtr_error *error);

/**
 * MTR object create
 *
 * Create a new MTR object for the current port.
 *
 * @param[in] port_id
 *   The port identifier of the Ethernet device.
 * @param[in] mtr_id
 *   MTR object ID. Needs to be unused by any of the existing MTR objects
 *   created for the current port.
 * @param[in] params
 *   MTR object params. Needs to be pre-allocated and valid.
 * @param[in] shared
 *   Non-zero when this MTR object can be shared by multiple flows, zero when
 *   this MTR object can be used by a single flow.
 * @param[out] error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int
rte_mtr_create(uint8_t port_id,
	uint32_t mtr_id,
	struct rte_mtr_params *params,
	int shared,
	struct rte_mtr_error *error);

/**
 * MTR object destroy
 *
 * Delete an existing MTR object. This operation fails when there is currently
 * at least one user (i.e. flow) of this MTR object.
 *
 * @param[in] port_id
 *   The port identifier of the Ethernet device.
 * @param[in] mtr_id
 *   MTR object ID. Needs to be unused by any of the existing MTR objects
 *   created for the current port.
 * @param[out] error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int
rte_mtr_destroy(uint8_t port_id,
	uint32_t mtr_id,
	struct rte_mtr_error *error);

/**
 * MTR object meter profile update
 *
 * @param[in] port_id
 *   The port identifier of the Ethernet device.
 * @param[in] mtr_id
 *   MTR object ID. Needs to be valid.
 * @param[in] meter_profile_id
 *   Meter profile ID for the current MTR object. Needs to be valid.
 * @param[out] error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int
rte_mtr_meter_profile_update(uint8_t port_id,
	uint32_t mtr_id,
	uint32_t meter_profile_id,
	struct rte_mtr_error *error);

/**
 * MTR object policer action update for given color
 *
 * @param[in] port_id
 *   The port identifier of the Ethernet device.
 * @param[in] mtr_id
 *   MTR object ID. Needs to be valid.
 * @param[in] color
 *   Color for which the policer action is updated.
 * @param[in] action
 *   Policer action for specified color.
 * @param[out] error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 */
int
rte_mtr_policer_action_update(uint8_t port_id,
	uint32_t mtr_id,
	enum rte_mtr_color color,
	enum rte_mtr_policer_action action,
	struct rte_mtr_error *error);

/**
 * MTR object enabled statistics counters update
 *
 * @param[in] port_id
 *   The port identifier of the Ethernet device.
 * @param[in] mtr_id
 *   MTR object ID. Needs to be valid.
 * @param[in] stats_mask
 *   Mask of statistics counter types to be enabled for the current MTR object.
 *   Any statistics counter type not included in this set is to be disabled for
 *   the current MTR object.
 * @param[out] error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 *
 * @see enum rte_mtr_stats_type
 */
int
rte_mtr_stats_update(uint8_t port_id,
	uint32_t mtr_id,
	uint64_t stats_mask,
	struct rte_mtr_error *error);

/**
 * MTR object statistics counters read
 *
 * @param[in] port_id
 *   The port identifier of the Ethernet device.
 * @param[in] mtr_id
 *   MTR object ID. Needs to be valid.
 * @param[out] stats
 *   When non-NULL, it contains the current value for the statistics counters
 *   enabled for the current MTR object.
 * @param[out] stats_mask
 *   When non-NULL, it contains the mask of statistics counter types that are
 *   currently enabled for this MTR object, indicating which of the counters
 *   retrieved with the *stats* structure are valid.
 * @param[in] clear
 *   When this parameter has a non-zero value, the statistics counters are
 *   cleared (i.e. set to zero) immediately after they have been read,
 *   otherwise the statistics counters are left untouched.
 * @param[out] error
 *   Error details. Filled in only on error, when not NULL.
 * @return
 *   0 on success, non-zero error code otherwise.
 *
 * @see enum rte_mtr_stats_type
 */
int
rte_mtr_stats_read(uint8_t port_id,
	uint32_t mtr_id,
	struct rte_mtr_stats *stats,
	uint64_t *stats_mask,
	int clear,
	struct rte_mtr_error *error);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_RTE_MTR_H__ */
