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

#include <string.h>

#include <rte_eal.h>
#include <rte_eal_memconfig.h>
#include <rte_dev.h>
#include "rte_eventdev.h"
#include "rte_eventdev_pmd.h"
#include "rte_eventdev_ops.h"

#define RTE_MAX_EVENT_DEVS 32
struct rte_event_dev *event_devs[RTE_MAX_EVENT_DEVS];
static int rte_num_event_dev_pmds;

TAILQ_HEAD(rte_eventdev_list, rte_event_dev);

static struct rte_tailq_elem rte_eventdev_tailq = {
	.name = "EVENT_DEVS",
};
EAL_REGISTER_TAILQ(rte_eventdev_tailq)

#ifdef RTE_LIBRTE_EVENTDEV_DEBUG
#define RTE_PMD_DEBUG_TRACE(...) \
	rte_pmd_debug_trace(__func__, __VA_ARGS__)
#else
#define RTE_PMD_DEBUG_TRACE(...)
#endif


int
rte_event_dev_register(struct rte_event_dev *new_dev)
{
	uint8_t index = rte_num_event_dev_pmds;
	struct rte_eventdev_list *list;

	list = RTE_TAILQ_CAST(rte_eventdev_tailq.head, rte_eventdev_list);

	/* The primary process is responsible for adding the eventdevs to the
	 * tailQ, and the secondary processes simply look up the eventdev in
	 * the list and add it to their local array for fast access.
	 */
	if (rte_eal_process_type() == RTE_PROC_PRIMARY) {
		new_dev->id = index;

		rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);
		TAILQ_INSERT_TAIL(list, new_dev, next);
		rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);
	} else {
		struct rte_event_dev *dev;

		bool found = false;
		rte_rwlock_read_lock(RTE_EAL_TAILQ_RWLOCK);
		TAILQ_FOREACH(dev, list, next) {
			if (strcmp(dev->name, new_dev->name) == 0) {
				found = true;
				break;
			}
		}
		rte_rwlock_read_unlock(RTE_EAL_TAILQ_RWLOCK);

		if (!found)
			return -1;
	}

	event_devs[new_dev->id] = new_dev;

	rte_num_event_dev_pmds++;

	return 0;
}

uint8_t
rte_event_dev_count(void)
{
	return rte_num_event_dev_pmds;
}

int8_t
rte_event_dev_get_dev_id(const char *name)
{
	struct rte_event_dev *d;
	int i;

	/* FIXME: This loop only works on secondary processes when the same
	 * eventdev vdevs as the primary process are passed on the command
	 * line.
	 */
	for (i = 0; i < rte_num_event_dev_pmds; i++) {
		d = event_devs[i];
		if (strcmp(d->name, name) == 0)
			return d->id;
	}
	return -EINVAL;
}

int
rte_event_dev_socket_id(uint8_t dev_id)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev)
		return -EINVAL;

	return dev->socket_id;
}

int
rte_event_dev_info_get(uint8_t dev_id, struct rte_event_dev_info *dev_info)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev_info)
		return -EINVAL;

	*dev_info = dev->info;

	return 0;
}

int
rte_event_dev_configure(uint8_t dev_id, struct rte_event_dev_config *config)
{
	struct rte_event_dev *dev = event_devs[dev_id];
	int ret;

	if (!dev || dev->configured || !config)
		return -EINVAL;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->configure, -ENOTSUP);
	ret = dev->ops->configure(dev, config);
	if (ret)
		return ret;

	dev->configured = true;

	return 0;
}

void
rte_event_queue_default_conf_get(uint8_t dev_id,
				 uint8_t queue_id,
				 struct rte_event_queue_conf *queue_conf)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return;

	RTE_FUNC_PTR_OR_RET(dev->ops->queue_default_conf_get);
	return dev->ops->queue_default_conf_get(dev, queue_id, queue_conf);
}

int
rte_event_queue_setup(uint8_t dev_id,
		      uint8_t queue_id,
		      const struct rte_event_queue_conf *queue_conf)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return 0;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->queue_setup, -ENOTSUP);
	return dev->ops->queue_setup(dev, queue_id, queue_conf);
}

uint16_t
rte_event_queue_count(uint8_t dev_id)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return 0;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->queue_count, -ENOTSUP);
	return dev->ops->queue_count(dev);
}

uint8_t
rte_event_queue_priority(uint8_t dev_id, uint8_t queue_id)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return 0;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->queue_priority, -ENOTSUP);
	return dev->ops->queue_priority(dev, queue_id);
}

void
rte_event_port_default_conf_get(uint8_t dev_id,
				uint8_t port_id,
				struct rte_event_port_conf *port_conf)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return;

	RTE_FUNC_PTR_OR_RET(dev->ops->port_default_conf_get);
	return dev->ops->port_default_conf_get(dev, port_id, port_conf);
}

int
rte_event_port_setup(uint8_t dev_id,
		     uint8_t port_id,
		     const struct rte_event_port_conf *port_conf)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->port_setup, -ENOTSUP);
	return dev->ops->port_setup(dev, port_id, port_conf);
}

uint8_t
rte_event_port_dequeue_depth(uint8_t dev_id, uint8_t port_id)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->port_dequeue_depth, -ENOTSUP);
	return dev->ops->port_dequeue_depth(dev, port_id);
}

uint8_t
rte_event_port_enqueue_depth(uint8_t dev_id, uint8_t port_id)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->port_enqueue_depth, -ENOTSUP);
	return dev->ops->port_enqueue_depth(dev, port_id);
}

uint8_t
rte_event_port_count(uint8_t dev_id)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return 0;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->port_count, -ENOTSUP);
	return dev->ops->port_count(dev);
}

int
rte_event_dev_start(uint8_t dev_id)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->start, -ENOTSUP);
	return dev->ops->start(dev);
}

void
rte_event_dev_stop(uint8_t dev_id)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return;

	RTE_FUNC_PTR_OR_RET(dev->ops->stop);
	return dev->ops->stop(dev);
}

int
rte_event_dev_close(uint8_t dev_id)
{
	int ret;
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->close, -ENOTSUP);
	ret = dev->ops->close(dev);
	if (ret)
		return ret;

	dev->configured = false;
	return 0;
}

int
rte_event_enqueue(uint8_t dev_id,
		  uint8_t event_port_id,
		  struct rte_event *ev,
		  bool pin_event)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->enqueue, -ENOTSUP);
	return dev->ops->enqueue(dev, event_port_id, ev, pin_event);
}

int
rte_event_enqueue_burst(uint8_t dev_id, uint8_t event_port_id,
			struct rte_event ev[], int num, bool pin_event)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->enqueue_burst, -ENOTSUP);
	return dev->ops->enqueue_burst(dev, event_port_id, ev, num, pin_event);
}

uint64_t
rte_event_dequeue_wait_time(uint8_t dev_id, uint64_t ns)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return 0;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->dequeue_wait_time, -ENOTSUP);
	return dev->ops->dequeue_wait_time(dev, ns);
}

bool
rte_event_dequeue(uint8_t dev_id, uint8_t event_port_id,
		  struct rte_event *ev, uint64_t wait)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return false;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->dequeue, -ENOTSUP);
	return dev->ops->dequeue(dev, event_port_id, ev, wait);
}

int
rte_event_dequeue_burst(uint8_t dev_id, uint8_t event_port_id,
			struct rte_event ev[], int num, uint64_t wait)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->dequeue_burst, -ENOTSUP);
	return dev->ops->dequeue_burst(dev, event_port_id, ev, num, wait);
}

int
rte_event_schedule(uint8_t dev_id)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->schedule, -ENOTSUP);
	return dev->ops->schedule(dev);
}

void
rte_event_release(uint8_t dev_id, uint8_t event_port_id, uint8_t index)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return;

	RTE_FUNC_PTR_OR_RET(dev->ops->release);
	dev->ops->release(dev, event_port_id, index);
}

int
rte_event_port_link(uint8_t dev_id, uint8_t port_id,
		struct rte_event_queue_link link[], int num)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->port_link, -ENOTSUP);
	return dev->ops->port_link(dev, port_id, link, num);
}

int
rte_event_port_unlink(uint8_t dev_id, uint8_t port_id, uint8_t queues[], int num)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->port_unlink, -ENOTSUP);
	return dev->ops->port_unlink(dev, port_id, queues, num);
}

int
rte_event_dev_stats_get(uint8_t dev_id, struct rte_event_dev_stats *stats)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return -1;

	RTE_FUNC_PTR_OR_ERR_RET(dev->ops->stats_get, -ENOTSUP);
	return dev->ops->stats_get(dev, stats);
}

void
rte_event_dev_dump(FILE *f, uint8_t dev_id)
{
	struct rte_event_dev *dev = event_devs[dev_id];

	if (!dev || !dev->configured)
		return;

	RTE_FUNC_PTR_OR_RET(dev->ops->dump);
	dev->ops->dump(f, dev);
}
