/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <sys/queue.h>
#include <sys/signalfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/netlink.h>
#include <sys/epoll.h>
#include <unistd.h>

#include <rte_malloc.h>
#include <rte_bus.h>
#include <rte_dev.h>
#include <rte_devargs.h>
#include <rte_debug.h>
#include <rte_log.h>
#include <rte_spinlock.h>

#include "eal_private.h"

/* spinlock for uevent callbacks */
static rte_spinlock_t rte_eal_uev_cb_lock = RTE_SPINLOCK_INITIALIZER;

/**
 * The user application callback description.
 *
 * It contains callback address to be registered by user application,
 * the pointer to the parameters for callback, and the event type.
 */
struct rte_eal_uev_callback {
	TAILQ_ENTRY(rte_eal_uev_callback) next; /**< Callbacks list */
	rte_eal_uev_cb_fn cb_fn;                /**< Callback address */
	void *cb_arg;                           /**< Parameter for callback */
	void *ret_param;                        /**< Return parameter */
	enum rte_eal_uevent_type event;          /**< Interrupt event type */
	uint32_t active;                        /**< Callback is executing */
};

static int cmp_detached_dev_name(const struct rte_device *dev,
	const void *_name)
{
	const char *name = _name;

	/* skip attached devices */
	if (dev->driver != NULL)
		return 1;

	return strcmp(dev->name, name);
}

static int cmp_dev_name(const struct rte_device *dev, const void *_name)
{
	const char *name = _name;

	return strcmp(dev->name, name);
}

int rte_eal_dev_attach(const char *name, const char *devargs)
{
	struct rte_bus *bus;
	int ret;

	if (name == NULL || devargs == NULL) {
		RTE_LOG(ERR, EAL, "Invalid device or arguments provided\n");
		return -EINVAL;
	}

	bus = rte_bus_find_by_device_name(name);
	if (bus == NULL) {
		RTE_LOG(ERR, EAL, "Unable to find a bus for the device '%s'\n",
			name);
		return -EINVAL;
	}
	if (strcmp(bus->name, "pci") == 0)
		return rte_eal_hotplug_add("pci", name, devargs);
	if (strcmp(bus->name, "vdev") != 0) {
		RTE_LOG(ERR, EAL, "Device attach is only supported for PCI and vdev devices.\n");
		return -ENOTSUP;
	}

	/*
	 * If we haven't found a bus device the user meant to "hotplug" a
	 * virtual device instead.
	 */
	ret = rte_vdev_init(name, devargs);
	if (ret)
		RTE_LOG(ERR, EAL, "Driver cannot attach the device (%s)\n",
			name);
	return ret;
}

int rte_eal_dev_detach(struct rte_device *dev)
{
	struct rte_bus *bus;
	int ret;

	if (dev == NULL) {
		RTE_LOG(ERR, EAL, "Invalid device provided.\n");
		return -EINVAL;
	}

	bus = rte_bus_find_by_device(dev);
	if (bus == NULL) {
		RTE_LOG(ERR, EAL, "Cannot find bus for device (%s)\n",
			dev->name);
		return -EINVAL;
	}

	if (bus->unplug == NULL) {
		RTE_LOG(ERR, EAL, "Bus function not supported\n");
		return -ENOTSUP;
	}

	ret = bus->unplug(dev);
	if (ret)
		RTE_LOG(ERR, EAL, "Driver cannot detach the device (%s)\n",
			dev->name);
	return ret;
}

static char *
full_dev_name(const char *bus, const char *dev, const char *args)
{
	char *name;
	size_t len;

	len = snprintf(NULL, 0, "%s:%s,%s", bus, dev, args) + 1;
	name = calloc(1, len);
	if (name == NULL) {
		RTE_LOG(ERR, EAL, "Could not allocate full device name\n");
		return NULL;
	}
	snprintf(name, len, "%s:%s,%s", bus, dev, args);
	return name;
}

int rte_eal_hotplug_add(const char *busname, const char *devname,
			const char *devargs)
{
	struct rte_bus *bus;
	struct rte_device *dev;
	struct rte_devargs *da;
	char *name;
	int ret;

	bus = rte_bus_find_by_name(busname);
	if (bus == NULL) {
		RTE_LOG(ERR, EAL, "Cannot find bus (%s)\n", busname);
		return -ENOENT;
	}

	if (bus->plug == NULL) {
		RTE_LOG(ERR, EAL, "Function plug not supported by bus (%s)\n",
			bus->name);
		return -ENOTSUP;
	}

	name = full_dev_name(busname, devname, devargs);
	if (name == NULL)
		return -ENOMEM;

	da = calloc(1, sizeof(*da));
	if (da == NULL) {
		ret = -ENOMEM;
		goto err_name;
	}

	ret = rte_eal_devargs_parse(name, da);
	if (ret)
		goto err_devarg;

	ret = rte_eal_devargs_insert(da);
	if (ret)
		goto err_devarg;

	ret = bus->scan();
	if (ret)
		goto err_devarg;

	dev = bus->find_device(NULL, cmp_detached_dev_name, devname);
	if (dev == NULL) {
		RTE_LOG(ERR, EAL, "Cannot find unplugged device (%s)\n",
			devname);
		ret = -ENODEV;
		goto err_devarg;
	}

	ret = bus->plug(dev);
	if (ret) {
		RTE_LOG(ERR, EAL, "Driver cannot attach the device (%s)\n",
			dev->name);
		goto err_devarg;
	}
	free(name);
	return 0;

err_devarg:
	if (rte_eal_devargs_remove(busname, devname)) {
		free(da->args);
		free(da);
	}
err_name:
	free(name);
	return ret;
}

int rte_eal_hotplug_remove(const char *busname, const char *devname)
{
	struct rte_bus *bus;
	struct rte_device *dev;
	int ret;

	bus = rte_bus_find_by_name(busname);
	if (bus == NULL) {
		RTE_LOG(ERR, EAL, "Cannot find bus (%s)\n", busname);
		return -ENOENT;
	}

	if (bus->unplug == NULL) {
		RTE_LOG(ERR, EAL, "Function unplug not supported by bus (%s)\n",
			bus->name);
		return -ENOTSUP;
	}

	dev = bus->find_device(NULL, cmp_dev_name, devname);
	if (dev == NULL) {
		RTE_LOG(ERR, EAL, "Cannot find plugged device (%s)\n", devname);
		return -EINVAL;
	}

	ret = bus->unplug(dev);
	if (ret)
		RTE_LOG(ERR, EAL, "Driver cannot detach the device (%s)\n",
			dev->name);
	rte_eal_devargs_remove(busname, devname);
	return ret;
}

int
rte_eal_uev_fd_new(void)
{

	int netlink_fd = -1;

	netlink_fd = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_KOBJECT_UEVENT);
	if (netlink_fd < 0)
		return -1;

	return netlink_fd;
}

int
rte_eal_uev_enable(int netlink_fd)
{
	struct sockaddr_nl addr;
	int ret;
	int size = 64 * 1024;
	int nonblock = 1;
	memset(&addr, 0, sizeof(addr));
	addr.nl_family = AF_NETLINK;
	addr.nl_pid = 0;
	addr.nl_groups = 0xffffffff;

	setsockopt(netlink_fd, SOL_SOCKET, SO_RCVBUFFORCE, &size, sizeof(size));

	ret = ioctl(netlink_fd, FIONBIO, &nonblock);
	if (ret != 0) {
		RTE_LOG(ERR, EAL,
		"ioctl(FIONBIO) failed\n");
		close(netlink_fd);
		return -1;
	}

	if (bind(netlink_fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
		close(netlink_fd);
		return -1;
	}

	return 0;
}

static int
rte_eal_uev_parse(const char *buf, struct rte_eal_uevent *event)
{
	char action[RTE_EAL_UEVENT_MSG_LEN];
	char subsystem[RTE_EAL_UEVENT_MSG_LEN];
	char dev_path[RTE_EAL_UEVENT_MSG_LEN];
	int i = 0;

	memset(action, 0, RTE_EAL_UEVENT_MSG_LEN);
	memset(subsystem, 0, RTE_EAL_UEVENT_MSG_LEN);
	memset(dev_path, 0, RTE_EAL_UEVENT_MSG_LEN);

	while (i < RTE_EAL_UEVENT_MSG_LEN) {
		for (; i < RTE_EAL_UEVENT_MSG_LEN; i++) {
			if (*buf)
				break;
			buf++;
		}
		if (!strncmp(buf, "ACTION=", 7)) {
			buf += 7;
			i += 7;
			snprintf(action, sizeof(action), "%s", buf);
		} else if (!strncmp(buf, "DEVPATH=", 8)) {
			buf += 8;
			i += 8;
			snprintf(dev_path, sizeof(dev_path), "%s", buf);
		} else if (!strncmp(buf, "SUBSYSTEM=", 10)) {
			buf += 10;
			i += 10;
			snprintf(subsystem, sizeof(subsystem), "%s", buf);
		}
		for (; i < RTE_EAL_UEVENT_MSG_LEN; i++) {
			if (*buf == '\0')
				break;
			buf++;
		}
	}

	if ((!strncmp(subsystem, "uio", 3)) ||
		(!strncmp(subsystem, "pci", 3))) {
		event->subsystem = RTE_EAL_UEVENT_SUBSYSTEM_UIO;
		if (!strncmp(action, "add", 3))
			event->type = RTE_EAL_UEVENT_ADD;
		if (!strncmp(action, "remove", 6))
			event->type = RTE_EAL_UEVENT_REMOVE;
		return 0;
	}

	return -1;
}

int
rte_eal_uev_receive(int fd, struct rte_eal_uevent *uevent)
{
	int ret;
	char buf[RTE_EAL_UEVENT_MSG_LEN];

	memset(uevent, 0, sizeof(struct rte_eal_uevent));
	memset(buf, 0, RTE_EAL_UEVENT_MSG_LEN);

	ret = recv(fd, buf, RTE_EAL_UEVENT_MSG_LEN - 1, MSG_DONTWAIT);
	if (ret > 0)
		return rte_eal_uev_parse(buf, uevent);
	else if (ret < 0) {
		RTE_LOG(ERR, EAL,
		"Socket read error(%d): %s\n",
		errno, strerror(errno));
		return -1;
	} else
		/* connection closed */
		return -1;
}

int
rte_eal_uev_callback_register(struct rte_device *dev,
			enum rte_eal_uevent_type event,
			rte_eal_uev_cb_fn cb_fn, void *cb_arg)
{
	struct rte_eal_uev_callback *user_cb;

	if (!cb_fn)
		return -EINVAL;

	rte_spinlock_lock(&rte_eal_uev_cb_lock);

	TAILQ_FOREACH(user_cb, &(dev->uev_cbs), next) {
		if (user_cb->cb_fn == cb_fn &&
			user_cb->cb_arg == cb_arg &&
			user_cb->event == event) {
			break;
		}
	}

	/* create a new callback. */
	if (user_cb == NULL) {
		user_cb = rte_zmalloc("EAL_UEV_CALLBACK",
					sizeof(struct rte_eal_uev_callback), 0);
		if (user_cb != NULL) {
			user_cb->cb_fn = cb_fn;
			user_cb->cb_arg = cb_arg;
			user_cb->event = event;
			TAILQ_INSERT_TAIL(&(dev->uev_cbs), user_cb, next);
		}
	}

	rte_spinlock_unlock(&rte_eal_uev_cb_lock);
	return (user_cb == NULL) ? -ENOMEM : 0;
}

int
rte_eal_uev_callback_unregister(struct rte_device *dev,
			enum rte_eal_uevent_type event,
			rte_eal_uev_cb_fn cb_fn, void *cb_arg)
{
	int ret;
	struct rte_eal_uev_callback *cb, *next;

	if (!cb_fn)
		return -EINVAL;

	rte_spinlock_lock(&rte_eal_uev_cb_lock);

	ret = 0;
	for (cb = TAILQ_FIRST(&dev->uev_cbs); cb != NULL; cb = next) {

		next = TAILQ_NEXT(cb, next);

		if (cb->cb_fn != cb_fn || cb->event != event ||
				(cb->cb_arg != (void *)-1 &&
				cb->cb_arg != cb_arg))
			continue;

		/*
		 * if this callback is not executing right now,
		 * then remove it.
		 */
		if (cb->active == 0) {
			TAILQ_REMOVE(&(dev->uev_cbs), cb, next);
			rte_free(cb);
		} else {
			ret = -EAGAIN;
		}
	}

	rte_spinlock_unlock(&rte_eal_uev_cb_lock);
	return ret;
}

int
_rte_eal_uev_callback_process(struct rte_device *dev,
	enum rte_eal_uevent_type event, void *cb_arg, void *ret_param)
{
	struct rte_eal_uev_callback *cb_lst;
	struct rte_eal_uev_callback dev_cb;
	int rc = 0;

	rte_spinlock_lock(&rte_eal_uev_cb_lock);
	TAILQ_FOREACH(cb_lst, &(dev->uev_cbs), next) {
		if (cb_lst->cb_fn == NULL || cb_lst->event != event)
			continue;
		dev_cb = *cb_lst;
		cb_lst->active = 1;
		if (cb_arg != NULL)
			dev_cb.cb_arg = cb_arg;
		if (ret_param != NULL)
			dev_cb.ret_param = ret_param;

		rte_spinlock_unlock(&rte_eal_uev_cb_lock);
		rc = dev_cb.cb_fn(dev, dev_cb.event,
				dev_cb.cb_arg, dev_cb.ret_param);
		rte_spinlock_lock(&rte_eal_uev_cb_lock);
		cb_lst->active = 0;
	}
	rte_spinlock_unlock(&rte_eal_uev_cb_lock);
	return rc;
}
