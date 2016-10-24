/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2016 RehiveTech. All rights reserved.
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
 *     * Neither the name of RehiveTech nor the names of its
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
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <dirent.h>
#include <limits.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <rte_log.h>
#include <rte_soc.h>

#include <eal_internal_cfg.h>
#include <eal_filesystem.h>
#include <eal_private.h>

/** Pathname of SoC devices directory. */
#define SYSFS_SOC_DEVICES "/sys/bus/platform/devices"

static const char *
soc_get_sysfs_path(void)
{
	const char *path = NULL;

	path = getenv("SYSFS_SOC_DEVICES");
	if (path == NULL)
		return SYSFS_SOC_DEVICES;

	return path;
}

static char *
dev_read_uevent(const char *dirname)
{
	char filename[PATH_MAX];
	struct stat st;
	char *buf;
	ssize_t total = 0;
	int fd;

	snprintf(filename, sizeof(filename), "%s/uevent", dirname);
	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		RTE_LOG(WARNING, EAL, "Failed to open file %s\n", filename);
		return strdup("");
	}

	if (fstat(fd, &st) < 0) {
		RTE_LOG(ERR, EAL, "Failed to stat file %s\n", filename);
		close(fd);
		return NULL;
	}

	if (st.st_size == 0) {
		close(fd);
		return strdup("");
	}

	buf = malloc(st.st_size + 1);
	if (buf == NULL) {
		RTE_LOG(ERR, EAL, "Failed to alloc memory to read %s\n",
			filename);
		close(fd);
		return NULL;
	}

	while (total < st.st_size) {
		ssize_t rlen = read(fd, buf + total, st.st_size - total);
		if (rlen < 0) {
			if (errno == EINTR)
				continue;

			RTE_LOG(ERR, EAL, "Failed to read file %s\n", filename);

			free(buf);
			close(fd);
			return NULL;
		}
		if (rlen == 0) /* EOF */
			break;

		total += rlen;
	}

	buf[total] = '\0';
	close(fd);

	return buf;
}

static const char *
dev_uevent_find(const char *uevent, const char *key)
{
	const size_t keylen = strlen(key);
	const size_t total = strlen(uevent);
	const char *p = uevent;

	/* check whether it is the first key */
	if (!strncmp(uevent, key, keylen))
		return uevent + keylen;

	/* check 2nd key or further... */
	do {
		p = strstr(p, key);
		if (p == NULL)
			break;

		if (p[-1] == '\n') /* check we are at a new line */
			return p + keylen;

		p += keylen; /* skip this one */
	} while (p - uevent < (ptrdiff_t) total);

	return NULL;
}

static char *
strdup_until_nl(const char *p)
{
	const char *nl = strchr(p, '\n');
	if (nl == NULL)
		return strdup(p); /* no newline, copy until '\0' */

	return strndup(p, nl - p);
}

static int
dev_parse_uevent(struct rte_soc_device *dev, const char *uevent)
{
	const char *of;
	const char *compat_n;
	char *err;
	long n;
	char compat[strlen("OF_COMPATIBLE_NNNN=")];
	long i;

	of = dev_uevent_find(uevent, "OF_FULLNAME=");
	if (of == NULL)
		return 1; /* don't care about this device */

	dev->addr.fdt_path = strdup_until_nl(of);
	if (dev->addr.fdt_path == NULL) {
		RTE_LOG(ERR, PMD,
			"Failed to alloc memory for fdt_path\n");
		return -1;
	}

	RTE_LOG(DEBUG, EAL, "Detected device %s (%s)\n",
			dev->addr.name, dev->addr.fdt_path);

	compat_n = dev_uevent_find(uevent, "OF_COMPATIBLE_N=");
	if (compat_n == NULL) {
		RTE_LOG(ERR, EAL, "No OF_COMPATIBLE_N found\n");
		return -1;
	}

	n = strtoul(compat_n, &err, 0);
	if (*err != '\n' && err != NULL) {
		RTE_LOG(ERR, EAL, "Failed to parse OF_COMPATIBLE_N: %.10s\n",
			err);
		goto fail_fdt_path;
	}

	if (n == 0)
		return 1; /* cannot match anything */
	if (n > 9999) { /* match NNNN */
		RTE_LOG(ERR, EAL, "OF_COMPATIBLE_N is invalid: %ld\n", n);
		goto fail_fdt_path;
	}

	dev->id = calloc(n + 1, sizeof(*dev->id));
	if (dev->id == NULL) {
		RTE_LOG(ERR, PMD, "Failed to alloc memory for ID\n");
		free(dev->addr.fdt_path);
		return -1;
	}

	for (i = 0; i < n; ++i) {
		snprintf(compat, sizeof(compat), "OF_COMPATIBLE_%lu=", i);
		const char *val;

		val = dev_uevent_find(uevent, compat);
		if (val == NULL) {
			RTE_LOG(ERR, EAL, "%s was not found\n", compat);
			goto fail_id;
		}

		dev->id[i]._compatible = strdup_until_nl(val);
		if (dev->id[i]._compatible == NULL) {
			RTE_LOG(ERR, PMD,
				"Failed to alloc memory for compatible\n");
			goto fail_id;
		}

		RTE_LOG(DEBUG, EAL, "  compatible: %s\n",
				dev->id[i].compatible);
	}

	dev->id[n]._compatible = NULL; /* mark last one */

	return 0;

fail_id:
	while (i-- >= 0)
		free(dev->id[i]._compatible);
	free(dev->id);
fail_fdt_path:
	free(dev->addr.fdt_path);
	return -1;
}

static void
dev_content_free(struct rte_soc_device *dev)
{
	int i;

	if (dev->addr.fdt_path)
		free(dev->addr.fdt_path);

	if (dev->id != NULL) {
		for (i = 0; dev->id[i]._compatible; ++i)
			free(dev->id[i]._compatible);

		free(dev->id);
		dev->id = NULL;
	}
}

/**
 * Scan one SoC sysfs entry, and fill the devices list from it.
 * We require to have the uevent file with records: OF_FULLNAME and
 * OF_COMPATIBLE array (with at least one entry). Otherwise, such device
 * is skipped.
 */
static int
soc_scan_one(const char *dirname, const char *name)
{
	struct rte_soc_device *dev;
	char *uevent;
	int ret;

	uevent = dev_read_uevent(dirname);
	if (uevent == NULL)
		return -1;

	if (uevent[0] == '\0') {
		/* ignore directory without uevent file */
		free(uevent);
		return 1;
	}

	dev = malloc(sizeof(*dev) + strlen(name) + 1);
	if (dev == NULL) {
		RTE_LOG(ERR, PMD, "Failed to alloc memory for %s\n", name);
		free(uevent);
		return -1;
	}

	memset(dev, 0, sizeof(*dev));
	dev->addr.name = (char *) (dev + 1);
	strcpy(dev->addr.name, name);

	ret = dev_parse_uevent(dev, uevent);
	if (ret)
		goto fail;
	free(uevent); /* not needed anymore */

	/* device is valid, add in list (sorted) */
	if (TAILQ_EMPTY(&soc_device_list)) {
		TAILQ_INSERT_TAIL(&soc_device_list, dev, next);
	} else {
		struct rte_soc_device *dev2;

		TAILQ_FOREACH(dev2, &soc_device_list, next) {
			ret = rte_eal_compare_soc_addr(&dev->addr, &dev2->addr);
			if (ret > 0)
				continue;

			if (ret < 0) {
				TAILQ_INSERT_BEFORE(dev2, dev, next);
			} else { /* already registered */
				dev_content_free(dev2);
				dev2->addr.fdt_path = dev->addr.fdt_path;
				dev2->id = dev->id;
				free(dev);
			}
			return 0;
		}
		TAILQ_INSERT_TAIL(&soc_device_list, dev, next);
	}

	return 0;

fail:
	free(uevent);
	dev_content_free(dev);
	free(dev);
	return ret;
}

int
rte_eal_soc_scan_platform_bus(void)
{
	struct dirent *e;
	DIR *dir;
	char dirname[PATH_MAX];

	dir = opendir(soc_get_sysfs_path());
	if (dir == NULL) {
		RTE_LOG(ERR, EAL, "%s(): opendir failed: %s\n",
			__func__, strerror(errno));
		return -1;
	}

	while ((e = readdir(dir)) != NULL) {
		if (e->d_name[0] == '.')
			continue;

		snprintf(dirname, sizeof(dirname), "%s/%s",
				soc_get_sysfs_path(), e->d_name);
		if (soc_scan_one(dirname, e->d_name) < 0)
			goto error;
	}
	closedir(dir);
	return 0;

error:
	closedir(dir);
	return -1;
}

/* Init the SoC EAL subsystem */
int
rte_eal_soc_init(void)
{
	struct rte_soc_driver *drv;

	/* SoC is disabled by default */
	if (!internal_config.enable_soc)
		return 0;

	/* For each registered driver, call their scan routine to perform any
	 * custom scan for devices (for example, custom buses)
	 */
	TAILQ_FOREACH(drv, &soc_driver_list, next) {
		RTE_VERIFY(drv->scan_fn);
		drv->scan_fn();
		/* Ignore all errors from this */
	}

	return 0;
}
