/*-
 *   BSD LICENSE
 *
 *   Copyright 2017 6WIND S.A.
 *   Copyright 2017 Mellanox
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
 *     * Neither the name of 6WIND S.A. nor the names of its
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

#include <errno.h>
#include <fcntl.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <netinet/ip.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/queue.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rte_alarm.h>
#include <rte_bus.h>
#include <rte_bus_vdev.h>
#include <rte_common.h>
#include <rte_config.h>
#include <rte_dev.h>
#include <rte_errno.h>
#include <rte_ethdev.h>
#include <rte_ether.h>
#include <rte_kvargs.h>
#include <rte_log.h>

#define HYPERV_DRIVER net_hyperv
#define HYPERV_ARG_IFACE "iface"
#define HYPERV_ARG_MAC "mac"
#define HYPERV_ARG_FORCE "force"
#define HYPERV_PROBE_MS 1000

#define NETVSC_CLASS_ID "{f8615163-df3e-46c5-913f-f2d2f965ed0e}"

#ifdef RTE_LIBRTE_HYPERV_DEBUG

#define PMD_DRV_LOG(level, ...) \
	RTE_LOG(level, PMD, \
		RTE_FMT("%s:%u: %s(): " RTE_FMT_HEAD(__VA_ARGS__,) "\n", \
			strrchr("/" __FILE__, '/') + 1, \
			__LINE__, \
			__func__, \
			RTE_FMT_TAIL(__VA_ARGS__,)))

#else /* RTE_LIBRTE_HYPERV_DEBUG */

#define PMD_DRV_LOG(level, ...) \
	RTE_LOG(level, PMD, \
		RTE_FMT(RTE_STR(HYPERV_DRIVER) ": " \
			RTE_FMT_HEAD(__VA_ARGS__,) "\n", \
		RTE_FMT_TAIL(__VA_ARGS__,)))

#endif /* RTE_LIBRTE_HYPERV_DEBUG */

#define DEBUG(...) PMD_DRV_LOG(DEBUG, __VA_ARGS__)
#define INFO(...) PMD_DRV_LOG(INFO, __VA_ARGS__)
#define WARN(...) PMD_DRV_LOG(WARNING, __VA_ARGS__)
#define ERROR(...) PMD_DRV_LOG(ERR, __VA_ARGS__)

/**
 * Convert a MAC address string to binary form.
 *
 * Note: this function should be exposed by rte_ether.h as the reverse of
 * ether_format_addr().
 *
 * Several MAC string formats are supported on input for convenience:
 *
 * 1. "12:34:56:78:9a:bc"
 * 2. "12-34-56-78-9a-bc"
 * 3. "123456789abc"
 * 4. Upper/lowercase hexadecimal.
 * 5. Any combination of the above, e.g. "12:34-5678-9aBC".
 * 6. Partial addresses are allowed, with low-order bytes filled first:
 *    - "5:6:78c" translates to "00:00:05:06:07:8c",
 *    - "5678c" translates to "00:00:00:05:67:8c".
 *
 * Non-hexadecimal characters, unknown separators and strings specifying
 * more than 6 bytes are not allowed.
 *
 * @param[out] eth_addr
 *   Pointer to conversion result buffer.
 * @param[in] str
 *   MAC address string to convert.
 *
 * @return
 *   0 on success, -EINVAL in case of unsupported format.
 */
static int
ether_addr_from_str(struct ether_addr *eth_addr, const char *str)
{
	static const uint8_t conv[0x100] = {
		['0'] = 0x80, ['1'] = 0x81, ['2'] = 0x82, ['3'] = 0x83,
		['4'] = 0x84, ['5'] = 0x85, ['6'] = 0x86, ['7'] = 0x87,
		['8'] = 0x88, ['9'] = 0x89, ['a'] = 0x8a, ['b'] = 0x8b,
		['c'] = 0x8c, ['d'] = 0x8d, ['e'] = 0x8e, ['f'] = 0x8f,
		['A'] = 0x8a, ['B'] = 0x8b, ['C'] = 0x8c, ['D'] = 0x8d,
		['E'] = 0x8e, ['F'] = 0x8f, [':'] = 0x40, ['-'] = 0x40,
		['\0'] = 0x60,
	};
	uint64_t addr = 0;
	uint64_t buf = 0;
	unsigned int i = 0;
	unsigned int n = 0;
	uint8_t tmp;

	do {
		tmp = conv[(int)*(str++)];
		if (!tmp)
			return -EINVAL;
		if (tmp & 0x40) {
			i += (i & 1) + (!i << 1);
			addr = (addr << (i << 2)) | buf;
			n += i;
			buf = 0;
			i = 0;
		} else {
			buf = (buf << 4) | (tmp & 0xf);
			++i;
		}
	} while (!(tmp & 0x20));
	if (n > 12)
		return -EINVAL;
	i = RTE_DIM(eth_addr->addr_bytes);
	while (i) {
		eth_addr->addr_bytes[--i] = addr & 0xff;
		addr >>= 8;
	}
	return 0;
}

/** Context structure for a hyperv instance. */
struct hyperv_ctx {
	LIST_ENTRY(hyperv_ctx) entry; /**< Next entry in list. */
	unsigned int id; /**< ID used to generate unique names. */
	char name[64]; /**< Unique name for hyperv instance. */
	char devname[64]; /**< Fail-safe PMD instance name. */
	char devargs[256]; /**< Fail-safe PMD instance device arguments. */
	char if_name[IF_NAMESIZE]; /**< NetVSC netdevice name. */
	unsigned int if_index; /**< NetVSC netdevice index. */
	struct ether_addr if_addr; /**< NetVSC MAC address. */
	int pipe[2]; /**< Communication pipe with fail-safe instance. */
	char yield[256]; /**< Current device string used with fail-safe. */
};

/** Context list is common to all PMD instances. */
static LIST_HEAD(, hyperv_ctx) hyperv_ctx_list =
	LIST_HEAD_INITIALIZER(hyperv_ctx_list);

/** Number of entries in context list. */
static unsigned int hyperv_ctx_count;

/** Number of PMD instances relying on context list. */
static unsigned int hyperv_ctx_inst;

/**
 * Destroy a hyperv context instance.
 *
 * @param ctx
 *   Context to destroy.
 */
static void
hyperv_ctx_destroy(struct hyperv_ctx *ctx)
{
	if (ctx->pipe[0] != -1)
		close(ctx->pipe[0]);
	if (ctx->pipe[1] != -1)
		close(ctx->pipe[1]);
	/* Poisoning for debugging purposes. */
	memset(ctx, 0x22, sizeof(*ctx));
	free(ctx);
}

/**
 * Iterate over system network interfaces.
 *
 * This function runs a given callback function for each netdevice found on
 * the system.
 *
 * @param func
 *   Callback function pointer. List traversal is aborted when this function
 *   returns a nonzero value.
 * @param ...
 *   Variable parameter list passed as @p va_list to @p func.
 *
 * @return
 *   0 when the entire list is traversed successfully, a negative error code
 *   in case or failure, or the nonzero value returned by @p func when list
 *   traversal is aborted.
 */
static int
hyperv_foreach_iface(int (*func)(const struct if_nameindex *iface,
				 const struct ether_addr *eth_addr,
				 va_list ap), ...)
{
	struct if_nameindex *iface = if_nameindex();
	int s = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
	unsigned int i;
	int ret = 0;

	if (!iface) {
		ret = -ENOBUFS;
		ERROR("cannot retrieve system network interfaces");
		goto error;
	}
	if (s == -1) {
		ret = -errno;
		ERROR("cannot open socket: %s", rte_strerror(errno));
		goto error;
	}
	for (i = 0; iface[i].if_name; ++i) {
		struct ifreq req;
		struct ether_addr eth_addr;
		va_list ap;

		strncpy(req.ifr_name, iface[i].if_name, sizeof(req.ifr_name));
		if (ioctl(s, SIOCGIFHWADDR, &req) == -1) {
			WARN("cannot retrieve information about interface"
			     " \"%s\": %s",
			     req.ifr_name, rte_strerror(errno));
			continue;
		}
		memcpy(eth_addr.addr_bytes, req.ifr_hwaddr.sa_data,
		       RTE_DIM(eth_addr.addr_bytes));
		va_start(ap, func);
		ret = func(&iface[i], &eth_addr, ap);
		va_end(ap);
		if (ret)
			break;
	}
error:
	if (s != -1)
		close(s);
	if (iface)
		if_freenameindex(iface);
	return ret;
}

/**
 * Determine if a network interface is NetVSC.
 *
 * @param[in] iface
 *   Pointer to netdevice description structure (name and index).
 *
 * @return
 *   A nonzero value when interface is detected as NetVSC. In case of error,
 *   rte_errno is updated and 0 returned.
 */
static int
hyperv_iface_is_netvsc(const struct if_nameindex *iface)
{
	static const char temp[] = "/sys/class/net/%s/device/class_id";
	char path[snprintf(NULL, 0, temp, iface->if_name) + 1];
	FILE *f;
	int ret;
	int len = 0;

	snprintf(path, sizeof(path), temp, iface->if_name);
	f = fopen(path, "r");
	if (!f) {
		rte_errno = errno;
		return 0;
	}
	ret = fscanf(f, NETVSC_CLASS_ID "%n", &len);
	if (ret == EOF)
		rte_errno = errno;
	ret = len == (int)strlen(NETVSC_CLASS_ID);
	fclose(f);
	return ret;
}

/**
 * Retrieve the last component of a path.
 *
 * This is a simplified basename() that does not modify its input buffer to
 * handle trailing backslashes.
 *
 * @param[in] path
 *    Path to retrieve the last component from.
 *
 * @return
 *    Pointer to the last component.
 */
static const char *
hyperv_basename(const char *path)
{
	const char *tmp = path;

	while (*tmp)
		if (*(tmp++) == '/')
			path = tmp;
	return path;
}

/**
 * Retrieve network interface data from sysfs symbolic link.
 *
 * @param[out] buf
 *   Output data buffer.
 * @param size
 *   Output buffer size.
 * @param[in] if_name
 *   Netdevice name.
 * @param[in] relpath
 *   Symbolic link path relative to netdevice sysfs entry.
 *
 * @return
 *   0 on success, a negative error code otherwise.
 */
static int
hyperv_sysfs_readlink(char *buf, size_t size, const char *if_name,
		      const char *relpath)
{
	int ret;

	ret = snprintf(buf, size, "/sys/class/net/%s/%s", if_name, relpath);
	if (ret == -1 || (size_t)ret >= size - 1)
		return -ENOBUFS;
	ret = readlink(buf, buf, size);
	if (ret == -1)
		return -errno;
	if ((size_t)ret >= size - 1)
		return -ENOBUFS;
	buf[ret] = '\0';
	return 0;
}

/**
 * Probe a network interface to associate with hyperv context.
 *
 * This function determines if the network device matches the properties of
 * the NetVSC interface associated with the hyperv context and communicates
 * its bus address to the fail-safe PMD instance if so.
 *
 * It is normally used with hyperv_foreach_iface().
 *
 * @param[in] iface
 *   Pointer to netdevice description structure (name and index).
 * @param[in] eth_addr
 *   MAC address associated with @p iface.
 * @param ap
 *   Variable arguments list comprising:
 *
 *   - struct hyperv_ctx *ctx:
 *     Context to associate network interface with.
 *
 * @return
 *   A nonzero value when interface matches, 0 otherwise or in case of
 *   error.
 */
static int
hyperv_device_probe(const struct if_nameindex *iface,
		    const struct ether_addr *eth_addr,
		    va_list ap)
{
	struct hyperv_ctx *ctx = va_arg(ap, struct hyperv_ctx *);
	char buf[RTE_MAX(sizeof(ctx->yield), 256u)];
	const char *addr;
	size_t len;
	int ret;

	/* Skip non-matching or unwanted NetVSC interfaces. */
	if (ctx->if_index == iface->if_index) {
		if (!strcmp(ctx->if_name, iface->if_name))
			return 0;
		DEBUG("NetVSC interface \"%s\" (index %u) renamed \"%s\"",
		      ctx->if_name, ctx->if_index, iface->if_name);
		strncpy(ctx->if_name, iface->if_name, sizeof(ctx->if_name));
		return 0;
	}
	if (hyperv_iface_is_netvsc(iface))
		return 0;
	if (!is_same_ether_addr(eth_addr, &ctx->if_addr))
		return 0;
	/* Look for associated PCI device. */
	ret = hyperv_sysfs_readlink(buf, sizeof(buf), iface->if_name,
				    "device/subsystem");
	if (ret)
		return 0;
	if (strcmp(hyperv_basename(buf), "pci"))
		return 0;
	ret = hyperv_sysfs_readlink(buf, sizeof(buf), iface->if_name,
				    "device");
	if (ret)
		return 0;
	addr = hyperv_basename(buf);
	len = strlen(addr);
	if (!len)
		return 0;
	/* Send PCI device argument to fail-safe PMD instance if updated. */
	if (!strcmp(addr, ctx->yield))
		return 1;
	DEBUG("associating PCI device \"%s\" with NetVSC interface \"%s\""
	      " (index %u)",
	      addr, ctx->if_name, ctx->if_index);
	memmove(buf, addr, len + 1);
	addr = buf;
	buf[len] = '\n';
	ret = write(ctx->pipe[1], addr, len + 1);
	buf[len] = '\0';
	if (ret == -1) {
		if (errno == EINTR || errno == EAGAIN)
			return 1;
		WARN("cannot associate PCI device name \"%s\" with interface"
		     " \"%s\": %s",
		     addr, ctx->if_name, rte_strerror(errno));
		return 1;
	}
	if ((size_t)ret != len + 1) {
		/*
		 * Attempt to override previous partial write, no need to
		 * recover if that fails.
		 */
		ret = write(ctx->pipe[1], "\n", 1);
		(void)ret;
		return 1;
	}
	fsync(ctx->pipe[1]);
	memcpy(ctx->yield, addr, len + 1);
	return 1;
}

/**
 * Alarm callback that regularly probes system network interfaces.
 *
 * This callback runs at a frequency determined by HYPERV_PROBE_MS as long
 * as an hyperv context instance exists.
 *
 * @param arg
 *   Ignored.
 */
static void
hyperv_alarm(void *arg)
{
	struct hyperv_ctx *ctx;
	int ret;

	(void)arg;
	LIST_FOREACH(ctx, &hyperv_ctx_list, entry) {
		ret = hyperv_foreach_iface(hyperv_device_probe, ctx);
		if (ret)
			break;
	}
	if (!hyperv_ctx_count)
		return;
	ret = rte_eal_alarm_set(HYPERV_PROBE_MS * 1000, hyperv_alarm, NULL);
	if (ret < 0) {
		ERROR("unable to reschedule alarm callback: %s",
		      rte_strerror(-ret));
	}
}

/**
 * Probe a NetVSC interface to generate a hyperv context from.
 *
 * This function instantiates hyperv contexts either for all NetVSC devices
 * found on the system or only a subset provided as device arguments.
 *
 * It is normally used with hyperv_foreach_iface().
 *
 * @param[in] iface
 *   Pointer to netdevice description structure (name and index).
 * @param[in] eth_addr
 *   MAC address associated with @p iface.
 * @param ap
 *   Variable arguments list comprising:
 *
 *   - const char *name:
 *     Name associated with current driver instance.
 *
 *   - struct rte_kvargs *kvargs:
 *     Device arguments provided to current driver instance.
 *
 *   - int force:
 *     Accept specified interface even if not detected as NetVSC.
 *
 *   - unsigned int specified:
 *     Number of specific netdevices provided as device arguments.
 *
 *   - unsigned int *matched:
 *     The number of specified netdevices matched by this function.
 *
 * @return
 *   A nonzero value when interface matches, 0 otherwise or in case of
 *   error.
 */
static int
hyperv_netvsc_probe(const struct if_nameindex *iface,
		    const struct ether_addr *eth_addr,
		    va_list ap)
{
	const char *name = va_arg(ap, const char *);
	struct rte_kvargs *kvargs = va_arg(ap, struct rte_kvargs *);
	int force = va_arg(ap, int);
	unsigned int specified = va_arg(ap, unsigned int);
	unsigned int *matched = va_arg(ap, unsigned int *);
	unsigned int i;
	struct hyperv_ctx *ctx;
	uint16_t port_id;
	int ret;

	/* Probe all interfaces when none are specified. */
	if (specified) {
		for (i = 0; i != kvargs->count; ++i) {
			const struct rte_kvargs_pair *pair = &kvargs->pairs[i];

			if (!strcmp(pair->key, HYPERV_ARG_IFACE)) {
				if (!strcmp(pair->value, iface->if_name))
					break;
			} else if (!strcmp(pair->key, HYPERV_ARG_MAC)) {
				struct ether_addr tmp;

				if (ether_addr_from_str(&tmp, pair->value)) {
					ERROR("invalid MAC address format"
					      " \"%s\"",
					      pair->value);
					return -EINVAL;
				}
				if (!is_same_ether_addr(eth_addr, &tmp))
					break;
			}
		}
		if (i == kvargs->count)
			return 0;
		++(*matched);
	}
	/* Weed out interfaces already handled. */
	LIST_FOREACH(ctx, &hyperv_ctx_list, entry)
		if (ctx->if_index == iface->if_index)
			break;
	if (ctx) {
		if (!specified)
			return 0;
		WARN("interface \"%s\" (index %u) is already handled, skipping",
		     iface->if_name, iface->if_index);
		return 0;
	}
	if (!hyperv_iface_is_netvsc(iface)) {
		if (!specified)
			return 0;
		WARN("interface \"%s\" (index %u) is not NetVSC, %s",
		     iface->if_name, iface->if_index,
		     force ? "using anyway (forced)" : "skipping");
		if (!force)
			return 0;
	}
	/* Create interface context. */
	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		ret = -errno;
		ERROR("cannot allocate context for interface \"%s\": %s",
		      iface->if_name, rte_strerror(errno));
		goto error;
	}
	ctx->id = hyperv_ctx_count;
	strncpy(ctx->if_name, iface->if_name, sizeof(ctx->if_name));
	ctx->if_index = iface->if_index;
	ctx->if_addr = *eth_addr;
	ctx->pipe[0] = -1;
	ctx->pipe[1] = -1;
	ctx->yield[0] = '\0';
	if (pipe(ctx->pipe) == -1) {
		ret = -errno;
		ERROR("cannot allocate control pipe for interface \"%s\": %s",
		      ctx->if_name, rte_strerror(errno));
		goto error;
	}
	for (i = 0; i != RTE_DIM(ctx->pipe); ++i) {
		int flf = fcntl(ctx->pipe[i], F_GETFL);
		int fdf = fcntl(ctx->pipe[i], F_GETFD);

		if (flf != -1 &&
		    fcntl(ctx->pipe[i], F_SETFL, flf | O_NONBLOCK) != -1 &&
		    fdf != -1 &&
		    fcntl(ctx->pipe[i], F_SETFD,
			  i ? fdf | FD_CLOEXEC : fdf & ~FD_CLOEXEC) != -1)
			continue;
		ret = -errno;
		ERROR("cannot toggle non-blocking or close-on-exec flags on"
		      " control file descriptor #%u (%d): %s",
		      i, ctx->pipe[i], rte_strerror(errno));
		goto error;
	}
	/* Generate virtual device name and arguments. */
	i = 0;
	ret = snprintf(ctx->name, sizeof(ctx->name), "%s_id%u",
		       name, ctx->id);
	if (ret == -1 || (size_t)ret >= sizeof(ctx->name) - 1)
		++i;
	ret = snprintf(ctx->devname, sizeof(ctx->devname), "net_failsafe_%s",
		       ctx->name);
	if (ret == -1 || (size_t)ret >= sizeof(ctx->devname) - 1)
		++i;
	/*
	 * Note: bash replaces the default sh interpreter used by popen()
	 * because as seen with dash, POSIX-compliant shells do not
	 * necessarily support redirections with file descriptor numbers
	 * above 9.
	 */
	ret = snprintf(ctx->devargs, sizeof(ctx->devargs),
		       "exec(exec bash -c "
		       "'while read -r tmp <&%u 2> /dev/null;"
		       " do dev=$tmp; done;"
		       " echo $dev"
		       "'),dev(net_tap_%s,remote=%s)",
		       ctx->pipe[0], ctx->name, ctx->if_name);
	if (ret == -1 || (size_t)ret >= sizeof(ctx->devargs) - 1)
		++i;
	if (i) {
		ret = -ENOBUFS;
		ERROR("generated virtual device name or argument list too long"
		      " for interface \"%s\"", ctx->if_name);
		goto error;
	}
	/*
	 * Remove any competing rte_eth_dev entries sharing the same MAC
	 * address, fail-safe instances created by this PMD will handle them
	 * as sub-devices later.
	 */
	RTE_ETH_FOREACH_DEV(port_id) {
		struct rte_device *dev = rte_eth_devices[port_id].device;
		struct rte_bus *bus = rte_bus_find_by_device(dev);
		struct ether_addr tmp;

		rte_eth_macaddr_get(port_id, &tmp);
		if (!is_same_ether_addr(eth_addr, &tmp))
			continue;
		WARN("removing device \"%s\" with identical MAC address to"
		     " re-create it as a fail-safe sub-device",
		     dev->name);
		if (!bus)
			ret = -EINVAL;
		else
			ret = rte_eal_hotplug_remove(bus->name, dev->name);
		if (ret < 0) {
			ERROR("unable to remove device \"%s\": %s",
			      dev->name, rte_strerror(-ret));
			goto error;
		}
	}
	/* Request virtual device generation. */
	DEBUG("generating virtual device \"%s\" with arguments \"%s\"",
	      ctx->devname, ctx->devargs);
	ret = rte_eal_hotplug_add("vdev", ctx->devname, ctx->devargs);
	if (ret)
		goto error;
	LIST_INSERT_HEAD(&hyperv_ctx_list, ctx, entry);
	++hyperv_ctx_count;
	DEBUG("added NetVSC interface \"%s\" to context list", ctx->if_name);
	return 0;
error:
	if (ctx)
		hyperv_ctx_destroy(ctx);
	return ret;
}

/**
 * Probe NetVSC interfaces.
 *
 * This function probes system netdevices according to the specified device
 * arguments and starts a periodic alarm callback to notify the resulting
 * fail-safe PMD instances of their sub-devices whereabouts.
 *
 * @param dev
 *   Virtual device context for PMD instance.
 *
 * @return
 *    Always 0, even in case of errors.
 */
static int
hyperv_vdev_probe(struct rte_vdev_device *dev)
{
	static const char *const hyperv_arg[] = {
		HYPERV_ARG_IFACE,
		HYPERV_ARG_MAC,
		HYPERV_ARG_FORCE,
		NULL,
	};
	const char *name = rte_vdev_device_name(dev);
	const char *args = rte_vdev_device_args(dev);
	struct rte_kvargs *kvargs = rte_kvargs_parse(args ? args : "",
						     hyperv_arg);
	unsigned int specified = 0;
	unsigned int matched = 0;
	int force = 0;
	unsigned int i;
	int ret;

	DEBUG("invoked as \"%s\", using arguments \"%s\"", name, args);
	if (!kvargs) {
		ERROR("cannot parse arguments list");
		goto error;
	}
	for (i = 0; i != kvargs->count; ++i) {
		const struct rte_kvargs_pair *pair = &kvargs->pairs[i];

		if (!strcmp(pair->key, HYPERV_ARG_FORCE))
			force = !!atoi(pair->value);
		else if (!strcmp(pair->key, HYPERV_ARG_IFACE) ||
			 !strcmp(pair->key, HYPERV_ARG_MAC))
			++specified;
	}
	rte_eal_alarm_cancel(hyperv_alarm, NULL);
	/* Gather interfaces. */
	ret = hyperv_foreach_iface(hyperv_netvsc_probe, name, kvargs, force,
				   specified, &matched);
	if (ret < 0)
		goto error;
	if (matched < specified)
		WARN("some of the specified parameters did not match valid"
		     " network interfaces");
	ret = rte_eal_alarm_set(HYPERV_PROBE_MS * 1000, hyperv_alarm, NULL);
	if (ret < 0) {
		ERROR("unable to schedule alarm callback: %s",
		      rte_strerror(-ret));
		goto error;
	}
error:
	if (kvargs)
		rte_kvargs_free(kvargs);
	++hyperv_ctx_inst;
	return 0;
}

/**
 * Remove PMD instance.
 *
 * The alarm callback and underlying hyperv context instances are only
 * destroyed after the last PMD instance is removed.
 *
 * @param dev
 *   Virtual device context for PMD instance.
 *
 * @return
 *   Always 0.
 */
static int
hyperv_vdev_remove(struct rte_vdev_device *dev)
{
	(void)dev;
	if (--hyperv_ctx_inst)
		return 0;
	rte_eal_alarm_cancel(hyperv_alarm, NULL);
	while (!LIST_EMPTY(&hyperv_ctx_list)) {
		struct hyperv_ctx *ctx = LIST_FIRST(&hyperv_ctx_list);

		LIST_REMOVE(ctx, entry);
		--hyperv_ctx_count;
		hyperv_ctx_destroy(ctx);
	}
	return 0;
}

/** Virtual device descriptor. */
static struct rte_vdev_driver hyperv_vdev = {
	.probe = hyperv_vdev_probe,
	.remove = hyperv_vdev_remove,
};

RTE_PMD_REGISTER_VDEV(HYPERV_DRIVER, hyperv_vdev);
RTE_PMD_REGISTER_ALIAS(HYPERV_DRIVER, eth_hyperv);
RTE_PMD_REGISTER_PARAM_STRING(net_hyperv,
			      HYPERV_ARG_IFACE "=<string> "
			      HYPERV_ARG_MAC "=<string> "
			      HYPERV_ARG_FORCE "=<int>");
