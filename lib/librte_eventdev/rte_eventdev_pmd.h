/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Intel Corporation. All rights reserved.
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

#ifndef _RTE_EVENTDEV_PMD_H_
#define _RTE_EVENTDEV_PMD_H_

/** @file
 * RTE EVENTDEV PMD API
 *
 * @note
 * This API is for the Event Dev PMD only, and user applications should never
 * call these functions directly.
 */

#include "rte_eventdev.h"
#include "rte_eventdev_ops.h"

/** Max number of chars in an eventdev name */
#define RTE_EVENTDEV_PMD_NAME_SIZE 64

/* Main struct that is passed around - contains pointers to all other structs
 * linked with this dev. A dev represents the PMD behind an event device.
 */
struct rte_event_dev {
	char name[RTE_EVENTDEV_PMD_NAME_SIZE];
	const struct rte_event_dev_ops *ops;
	struct rte_event_dev_info info;
	uint32_t socket_id;
	uint8_t id;
	bool configured;
	TAILQ_ENTRY(rte_event_dev) next;
};

/**
 * Registers a PMD in the list of available event dev PMDs
 */
int
rte_event_dev_register(struct rte_event_dev *new_dev);

#endif /* _RTE_EVENTDEV_PMD_H_ */
