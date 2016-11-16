/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 Intel Corporation. All rights reserved.
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

#ifndef _RTE_EVENT_DEV_OPS_
#define _RTE_EVENT_DEV_OPS_

#include <stdint.h>

struct rte_event;
struct rte_event_dev;
struct rte_event_link;
struct rte_event_dev_info;
struct rte_event_dev_config;
struct rte_event_queue_conf;
struct rte_event_port_conf;

/* Creation and info */
typedef int (*event_dev_configure)(
		struct rte_event_dev *dev,
		struct rte_event_dev_config *config);
typedef int (*event_dev_start)(struct rte_event_dev *dev);
typedef void (*event_dev_stop)(struct rte_event_dev *dev);
typedef int (*event_dev_close)(struct rte_event_dev *dev);

/* Queue control */
typedef void (*event_queue_default_conf_get)(
		struct rte_event_dev *dev,
		uint8_t queue_id,
		struct rte_event_queue_conf *queue_conf);
typedef int (*event_dev_queue_setup)(
		struct rte_event_dev *dev,
		uint8_t queue_id,
		const struct rte_event_queue_conf *queue_conf);
typedef uint16_t (*event_dev_queue_count)(struct rte_event_dev *dev);
typedef uint8_t (*event_queue_priority)(
		struct rte_event_dev *dev,
		uint8_t queue_id);


/* Port control */
typedef void (*event_port_default_conf_get)(
		struct rte_event_dev *dev,
		uint8_t port_id,
		struct rte_event_port_conf *port_conf);
typedef int (*event_dev_port_setup)(
		struct rte_event_dev *dev,
		uint8_t port_id,
		const struct rte_event_port_conf *port_conf);
typedef uint8_t (*event_dev_port_dequeue_depth)(
		struct rte_event_dev *dev,
		uint8_t port_id);
typedef uint8_t (*event_dev_port_enqueue_depth)(
		struct rte_event_dev *dev,
		uint8_t port_id);
typedef uint8_t (*event_dev_port_count)(struct rte_event_dev *dev);

/* Enqueue, dequeue and scheduling */
typedef int (*event_dev_enqueue)(
		struct rte_event_dev *dev,
		uint8_t port_id,
		struct rte_event *ev,
		bool pin_event);
typedef int (*event_dev_enqueue_burst)(
		struct rte_event_dev *dev,
		uint8_t port_id,
		struct rte_event ev[],
		int num,
		bool pin_event);
typedef bool (*event_dev_dequeue_wait_time)(
		struct rte_event_dev *dev,
		uint64_t ns);
typedef bool (*event_dev_dequeue)(
		struct rte_event_dev *dev,
		uint8_t port_id,
		struct rte_event *ev,
		uint64_t wait);
typedef int (*event_dev_dequeue_burst)(
		struct rte_event_dev *dev,
		uint8_t port_id,
		struct rte_event ev[],
		int num,
		uint64_t wait);
typedef int (*event_dev_schedule)(struct rte_event_dev *dev);
typedef void (*event_dev_release)(
		struct rte_event_dev *dev,
		uint8_t port_id,
		uint8_t index);

/* Mapping */
typedef int (*event_dev_port_link)(
		struct rte_event_dev *dev,
		uint8_t port_id,
		struct rte_event_queue_link link[],
		int num);
typedef int (*event_dev_port_unlink)(
		struct rte_event_dev *dev,
		uint8_t port_id,
		uint8_t queues[],
		int num);

/* stats */
typedef int (*event_dev_stats_get)(
		const struct rte_event_dev *dev,
		struct rte_event_dev_stats *stats);
typedef void (*event_dev_dump)(
		FILE *f,
		const struct rte_event_dev *dev);

struct rte_event_dev_ops {
	/* Creation and info */
	event_dev_configure	configure;
	event_dev_start		start;
	event_dev_stop		stop;
	event_dev_close		close;

	/* Port control */
	event_port_default_conf_get port_default_conf_get;
	event_dev_port_setup	port_setup;
	event_dev_port_dequeue_depth port_dequeue_depth;
	event_dev_port_enqueue_depth port_enqueue_depth;
	event_dev_port_count    port_count;

	/* Queue control */
	event_queue_default_conf_get queue_default_conf_get;
	event_dev_queue_setup	queue_setup;
	event_dev_queue_count   queue_count;
	event_queue_priority	queue_priority;

	/* Enqueue, dequeue and scheduling */
	event_dev_enqueue	enqueue;
	event_dev_enqueue_burst	enqueue_burst;
	event_dev_dequeue_wait_time	dequeue_wait_time;
	event_dev_dequeue	dequeue;
	event_dev_dequeue_burst	dequeue_burst;
	event_dev_schedule	schedule;
	event_dev_release	release;

	/* Mapping */
	event_dev_port_link	port_link;
	event_dev_port_unlink	port_unlink;

	/* Stats */
	event_dev_stats_get	stats_get;
	event_dev_dump          dump;
};

#endif /* _RTE_EVENT_DEV_OPS_ */
