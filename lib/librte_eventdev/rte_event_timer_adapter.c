/*-
 *   BSD LICENSE
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

#include <string.h>

#include <rte_memzone.h>
#include <rte_memory.h>
#include <rte_dev.h>
#include <rte_errno.h>

#include "rte_eventdev.h"
#include "rte_eventdev_pmd.h"
#include "rte_event_timer_adapter.h"
#include "rte_event_timer_adapter_driver.h"

#define MAX_EVENT_TIMER_ADAPTERS 64
#define DATA_MZ_NAME_MAX_LEN 64
#define DATA_MZ_NAME_FORMAT "rte_event_timer_adapter_data_%d"

static struct rte_event_timer_adapter adapters[MAX_EVENT_TIMER_ADAPTERS];

extern const struct rte_event_timer_adapter_ops sw_event_adapter_timer_ops;

static inline int
adapter_valid(const struct rte_event_timer_adapter *adapter)
{
	return adapter != NULL && adapter->allocated == 1;
}

#define ADAPTER_VALID_OR_ERR_RET(adapter, retval) do { \
	if (!adapter_valid(adapter))		       \
		return retval;			       \
} while (0)

#define FUNC_PTR_OR_ERR_RET(func, errval) do { \
	if ((func) == NULL)		       \
		return errval;		       \
} while (0)

#define FUNC_PTR_OR_NULL_RET_WITH_ERRNO(func, errval) do { \
	if ((func) == NULL) {				   \
		rte_errno = errval;			   \
		return NULL;				   \
	}						   \
} while (0)

static int
default_port_conf_cb(uint16_t id, uint8_t event_dev_id, uint8_t *event_port_id,
		     void *conf_arg)
{
	struct rte_event_timer_adapter *adapter;
	struct rte_eventdev *dev;
	struct rte_event_dev_config dev_conf;
	struct rte_event_port_conf *port_conf = conf_arg;
	int started;
	uint8_t port_id;
	uint8_t dev_id;
	int ret;

	RTE_SET_USED(event_dev_id);

	adapter = &adapters[id];
	dev = &rte_eventdevs[adapter->data->event_dev_id];
	dev_id = dev->data->dev_id;
	dev_conf = dev->data->dev_conf;

	started = dev->data->dev_started;
	if (started)
		rte_event_dev_stop(dev_id);

	port_id = dev_conf.nb_event_ports;
	dev_conf.nb_event_ports += 1;
	ret = rte_event_dev_configure(dev_id, &dev_conf);
	if (ret < 0) {
		if (started)
			rte_event_dev_start(dev_id);

		return ret;
	}

	ret = rte_event_port_setup(dev_id, port_id, port_conf);
	if (ret < 0)
		return ret;

	*event_port_id = port_id;

	if (started)
		rte_event_dev_start(dev_id);

	return 0;
}

struct rte_event_timer_adapter *
rte_event_timer_adapter_create(const struct rte_event_timer_adapter_conf *conf)
{
	/* default port conf values */
	struct rte_event_port_conf port_conf = {
		.new_event_threshold = 128,
		.dequeue_depth = 32,
		.enqueue_depth = 32
	};

	return rte_event_timer_adapter_create_ext(conf, default_port_conf_cb,
						  &port_conf);
}

struct rte_event_timer_adapter *
rte_event_timer_adapter_create_ext(
		const struct rte_event_timer_adapter_conf *conf,
		rte_event_timer_adapter_port_conf_cb_t conf_cb,
		void *conf_arg)
{
	uint16_t adapter_id;
	struct rte_event_timer_adapter *adapter;
	const struct rte_memzone *mz;
	char mz_name[DATA_MZ_NAME_MAX_LEN];
	int n, ret;
	struct rte_eventdev *dev;

	if (conf == NULL) {
		rte_errno = -EINVAL;
		return NULL;
	}

	/* Check eventdev ID */
	if (!rte_event_pmd_is_valid_dev(conf->event_dev_id)) {
		rte_errno = -EINVAL;
		return NULL;
	}
	dev = &rte_eventdevs[conf->event_dev_id];

	adapter_id = conf->timer_adapter_id;

	/* Check adapter ID not already allocated */
	adapter = &adapters[adapter_id];
	if (adapter->allocated) {
		rte_errno = -EEXIST;
		return NULL;
	}

	/* Create shared data area. */
	n = snprintf(mz_name, sizeof(mz_name), DATA_MZ_NAME_FORMAT, adapter_id);
	if (n >= (int)sizeof(mz_name)) {
		rte_errno = -EINVAL;
		return NULL;
	}
	mz = rte_memzone_reserve(mz_name,
				 sizeof(struct rte_event_timer_adapter_data),
				 conf->socket_id, 0);
	if (mz == NULL)
		/* rte_errno set by rte_memzone_reserve */
		return NULL;

	adapter->data = mz->addr;
	memset(adapter->data, 0, sizeof(struct rte_event_timer_adapter_data));

	adapter->data->mz = mz;
	adapter->data->event_dev_id = conf->event_dev_id;
	adapter->data->id = adapter_id;
	adapter->data->socket_id = conf->socket_id;
	adapter->data->conf = *conf;  /* copy conf structure */

	/* Query eventdev PMD for timer adapter capabilities and ops */
	ret = dev->dev_ops->timer_adapter_caps_get(dev,
						   &adapter->data->caps,
						   &adapter->ops);
	if (ret < 0) {
		rte_errno = -EINVAL;
		return NULL;
	}

	if (!(adapter->data->caps &
	      RTE_EVENT_TIMER_ADAPTER_CAP_INTERNAL_PORT)) {
		FUNC_PTR_OR_NULL_RET_WITH_ERRNO(conf_cb, -EINVAL);
		ret = conf_cb(adapter->data->id, adapter->data->event_dev_id,
			      &adapter->data->event_port_id, conf_arg);
		if (ret < 0) {
			rte_errno = -EINVAL;
			return NULL;
		}
	}

	/* If eventdev PMD did not provide ops, use default software
	 * implementation.
	 */
	if (adapter->ops == NULL)
		adapter->ops = &sw_event_adapter_timer_ops;

	/* Allow driver to do some setup */
	FUNC_PTR_OR_NULL_RET_WITH_ERRNO(adapter->ops->init, -ENOTSUP);
	ret = adapter->ops->init(adapter);
	if (ret < 0) {
		rte_errno = -EINVAL;
		return NULL;
	}

	/* Set fast-path function pointers */
	adapter->arm_burst = adapter->ops->arm_burst;
	adapter->arm_tmo_tick_burst = adapter->ops->arm_tmo_tick_burst;
	adapter->cancel_burst = adapter->ops->cancel_burst;

	adapter->allocated = 1;

	return adapter;
}

int
rte_event_timer_adapter_get_info(const struct rte_event_timer_adapter *adapter,
		struct rte_event_timer_adapter_info *adapter_info)
{
	ADAPTER_VALID_OR_ERR_RET(adapter, -EINVAL);

	if (adapter->ops->get_info)
		/* let driver set values it knows */
		adapter->ops->get_info(adapter, adapter_info);

	/* Set common values */
	adapter_info->conf = adapter->data->conf;
	adapter_info->event_dev_port_id = adapter->data->event_port_id;
	adapter_info->caps = adapter->data->caps;

	return 0;
}

int
rte_event_timer_adapter_start(const struct rte_event_timer_adapter *adapter)
{
	int ret;

	ADAPTER_VALID_OR_ERR_RET(adapter, -EINVAL);
	FUNC_PTR_OR_ERR_RET(adapter->ops->start, -EINVAL);

	ret = adapter->ops->start(adapter);
	if (ret < 0)
		return ret;

	adapter->data->started = 1;

	return 0;
}

int
rte_event_timer_adapter_stop(const struct rte_event_timer_adapter *adapter)
{
	int ret;

	ADAPTER_VALID_OR_ERR_RET(adapter, -EINVAL);
	FUNC_PTR_OR_ERR_RET(adapter->ops->stop, -EINVAL);

	ret = adapter->ops->stop(adapter);
	if (ret < 0)
		return ret;

	adapter->data->started = 0;

	return 0;
}

struct rte_event_timer_adapter *
rte_event_timer_adapter_lookup(uint16_t adapter_id)
{
	char name[DATA_MZ_NAME_MAX_LEN];
	const struct rte_memzone *mz;
	struct rte_event_timer_adapter_data *data;
	struct rte_event_timer_adapter *adapter;
	int ret;
	struct rte_eventdev *dev;

	if (adapters[adapter_id].allocated)
		return &adapters[adapter_id]; /* Adapter is already loaded */

	snprintf(name, DATA_MZ_NAME_MAX_LEN, DATA_MZ_NAME_FORMAT, adapter_id);
	mz = rte_memzone_lookup(name);
	if (mz == NULL) {
		rte_errno = -ENOENT;
		return NULL;
	}

	data = mz->addr;

	adapter = &adapters[data->id];
	adapter->data = data;

	dev = &rte_eventdevs[adapter->data->event_dev_id];

	/* Query eventdev PMD for timer adapter capabilities and ops */
	ret = dev->dev_ops->timer_adapter_caps_get(dev,
						   &adapter->data->caps,
						   &adapter->ops);
	if (ret < 0) {
		rte_errno = -EINVAL;
		return NULL;
	}

	/* If eventdev PMD did not provide ops, use default software
	 * implementation.
	 */
	if (adapter->ops == NULL)
		adapter->ops = &sw_event_adapter_timer_ops;

	/* Set fast-path function pointers */
	adapter->arm_burst = adapter->ops->arm_burst;
	adapter->arm_tmo_tick_burst = adapter->ops->arm_tmo_tick_burst;
	adapter->cancel_burst = adapter->ops->cancel_burst;

	adapter->allocated = 1;

	return adapter;
}

int
rte_event_timer_adapter_free(struct rte_event_timer_adapter *adapter)
{
	int ret;

	ADAPTER_VALID_OR_ERR_RET(adapter, -EINVAL);
	FUNC_PTR_OR_ERR_RET(adapter->ops->uninit, -EINVAL);

	/* free impl priv data */
	ret = adapter->ops->uninit(adapter);
	if (ret < 0)
		return ret;

	/* free shared data area */
	ret = rte_memzone_free(adapter->data->mz);
	if (ret < 0)
		return ret;

	adapter->data = NULL;
	adapter->allocated = 0;

	return 0;
}

int
rte_event_timer_arm_burst(const struct rte_event_timer_adapter *adapter,
			  struct rte_event_timer **event_timers,
			  uint16_t nb_event_timers)
{
	int ret;

	ADAPTER_VALID_OR_ERR_RET(adapter, -EINVAL);
	FUNC_PTR_OR_ERR_RET(adapter->arm_burst, -EINVAL);

	if (!adapter->data->started)
		return -EAGAIN;

	ret = adapter->arm_burst(adapter, event_timers, nb_event_timers);
	if (ret < 0)
		return ret;

	return 0;
}

int
rte_event_timer_arm_tmo_tick_burst(
			const struct rte_event_timer_adapter *adapter,
			struct rte_event_timer **event_timers,
			const uint64_t timeout_ticks,
			const uint16_t nb_event_timers)
{
	int ret;

	ADAPTER_VALID_OR_ERR_RET(adapter, -EINVAL);
	FUNC_PTR_OR_ERR_RET(adapter->arm_tmo_tick_burst, -EINVAL);

	if (!adapter->data->started)
		return -EAGAIN;

	for (int i = 0; i < nb_event_timers; i++)
		event_timers[i]->timeout_ticks = timeout_ticks;

	ret = adapter->arm_tmo_tick_burst(adapter, event_timers, timeout_ticks,
					  nb_event_timers);
	if (ret < 0)
		return ret;

	return 0;
}

int
rte_event_timer_cancel_burst(const struct rte_event_timer_adapter *adapter,
			     struct rte_event_timer **event_timers,
			     uint16_t nb_event_timers)
{
	int ret;

	ADAPTER_VALID_OR_ERR_RET(adapter, -EINVAL);
	FUNC_PTR_OR_ERR_RET(adapter->cancel_burst, -EINVAL);

	if (!adapter->data->started)
		return -EAGAIN;

	ret = adapter->cancel_burst(adapter, event_timers, nb_event_timers);
	if (ret < 0)
		return ret;

	return 0;
}
