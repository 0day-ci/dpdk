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

#include <stdint.h>

#include <rte_errno.h>
#include "rte_ethdev.h"
#include "rte_mtr_driver.h"
#include "rte_mtr.h"

/* Get generic traffic metering and policing operations structure from a port. */
const struct rte_mtr_ops *
rte_mtr_ops_get(uint8_t port_id, struct rte_mtr_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	const struct rte_mtr_ops *ops;

	if (!rte_eth_dev_is_valid_port(port_id)) {
		rte_mtr_error_set(error,
			ENODEV,
			RTE_MTR_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENODEV));
		return NULL;
	}

	if ((dev->dev_ops->mtr_ops_get == NULL) ||
		(dev->dev_ops->mtr_ops_get(dev, &ops) != 0) ||
		(ops == NULL)) {
		rte_mtr_error_set(error,
			ENOSYS,
			RTE_MTR_ERROR_TYPE_UNSPECIFIED,
			NULL,
			rte_strerror(ENOSYS));
		return NULL;
	}

	return ops;
}

#define RTE_MTR_FUNC(port_id, func)			\
({							\
	const struct rte_mtr_ops *ops =			\
		rte_mtr_ops_get(port_id, error);		\
	if (ops == NULL)					\
		return -rte_errno;			\
							\
	if (ops->func == NULL)				\
		return -rte_mtr_error_set(error,		\
			ENOSYS,				\
			RTE_MTR_ERROR_TYPE_UNSPECIFIED,	\
			NULL,				\
			rte_strerror(ENOSYS));		\
							\
	ops->func;					\
})

/* MTR meter profile add */
int
rte_mtr_meter_profile_add(uint8_t port_id,
	uint32_t meter_profile_id,
	struct rte_mtr_meter_profile *profile,
	struct rte_mtr_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	return RTE_MTR_FUNC(port_id, meter_profile_add)(dev,
		meter_profile_id, profile, error);
}

/** MTR meter profile delete */
int
rte_mtr_meter_profile_delete(uint8_t port_id,
	uint32_t meter_profile_id,
	struct rte_mtr_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	return RTE_MTR_FUNC(port_id, meter_profile_delete)(dev,
		meter_profile_id, error);
}

/** MTR object create */
int
rte_mtr_create(uint8_t port_id,
	uint32_t mtr_id,
	struct rte_mtr_params *params,
	int shared,
	struct rte_mtr_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	return RTE_MTR_FUNC(port_id, create)(dev,
		mtr_id, params, shared, error);
}

/** MTR object destroy */
int
rte_mtr_destroy(uint8_t port_id,
	uint32_t mtr_id,
	struct rte_mtr_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	return RTE_MTR_FUNC(port_id, destroy)(dev,
		mtr_id, error);
}

/** MTR object meter profile update */
int
rte_mtr_meter_profile_update(uint8_t port_id,
	uint32_t mtr_id,
	uint32_t meter_profile_id,
	struct rte_mtr_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	return RTE_MTR_FUNC(port_id, meter_profile_update)(dev,
		mtr_id, meter_profile_id, error);
}

/** MTR object policer action update */
int
rte_mtr_policer_action_update(uint8_t port_id,
	uint32_t mtr_id,
	enum rte_mtr_color color,
	enum rte_mtr_policer_action action,
	struct rte_mtr_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	return RTE_MTR_FUNC(port_id, policer_action_update)(dev,
		mtr_id, color, action, error);
}

/** MTR object enabled stats update */
int
rte_mtr_stats_update(uint8_t port_id,
	uint32_t mtr_id,
	uint64_t stats_mask,
	struct rte_mtr_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	return RTE_MTR_FUNC(port_id, stats_update)(dev,
		mtr_id, stats_mask, error);
}

/** MTR object stats read */
int
rte_mtr_stats_read(uint8_t port_id,
	uint32_t mtr_id,
	struct rte_mtr_stats *stats,
	uint64_t *stats_mask,
	int clear,
	struct rte_mtr_error *error)
{
	struct rte_eth_dev *dev = &rte_eth_devices[port_id];
	return RTE_MTR_FUNC(port_id, stats_read)(dev,
		mtr_id, stats, stats_mask, clear, error);
}
