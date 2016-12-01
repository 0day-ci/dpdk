/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
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
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <poll.h>
#include <unistd.h>

#include <rte_common.h>
#include <rte_malloc.h>
#include <rte_log.h>

#include "fd_man.h"

#define FDPOLLERR (POLLERR | POLLHUP | POLLNVAL)


/**
 * Adjusts the highest index populated in the array of fds
 * @return
 *   Index of highest position populated + 1.
 */
static int
fdset_adjust_num(struct fdset *pfdset)
{
	int idx;

	for (idx = pfdset->num - 1;
	     idx >= 0 && pfdset->fd[idx].fd == -1;
	     idx--)
		;

	pfdset->num = idx + 1;

	return pfdset->num;
}

/**
 * Returns the index in the fdset for a given fd.
 * If fd is -1, it means to search for a free entry.
 * @return
 *   index for the fd, or -1 if fd isn't in the fdset.
 */
static int
fdset_find_fd(struct fdset *pfdset, int fd)
{
	int i;

	for (i = 0; i < pfdset->num && pfdset->fd[i].fd != fd; i++)
		;

	return i == pfdset->num ? -1 : i;
}

static int
fdset_find_free_slot(struct fdset *pfdset)
{
	if (pfdset->num < MAX_FDS)
		return pfdset->num;
	else
		return fdset_find_fd(pfdset, -1);
}

static void
fdset_add_fd(struct fdset *pfdset, int idx, int fd,
	fd_cb rcb, fd_cb wcb, void *dat)
{
	struct fdentry *pfdentry;

	pfdentry = &pfdset->fd[idx];
	pfdentry->fd = fd;
	pfdentry->rcb = rcb;
	pfdentry->wcb = wcb;
	pfdentry->dat = dat;
}

/**
 * Compact the fdset and fill the read/write fds with the fds in the fdset.
 * @return
 *  the number of fds filled in the read/write fds.
 */
static int
fdset_fill(struct pollfd *rwfds, struct fdset *pfdset)
{
	struct fdentry *pfdentry;
	int i;
	int num;

	for (i = 0, num = pfdset->num; i < num; i++) {
		pfdentry = &pfdset->fd[i];

		if (pfdentry->fd < 0) {
			/* Hole in the list. Move the last one here */

			*pfdentry = pfdset->fd[num - 1];
			pfdset->fd[num - 1].fd = -1;
			num = fdset_adjust_num(pfdset);
		}
		rwfds[i].fd = pfdentry->fd;
		rwfds[i].events = pfdentry->rcb ? POLLIN : 0;
		rwfds[i].events |= pfdentry->wcb ? POLLOUT : 0;
	}

	return i;
}

void
fdset_init(struct fdset *pfdset)
{
	int i;

	if (pfdset == NULL)
		return;

	pthread_mutex_init(&pfdset->fd_mutex, NULL);

	for (i = 0; i < MAX_FDS; i++) {
		pfdset->fd[i].fd = -1;
		pfdset->fd[i].dat = NULL;
	}
	pfdset->num = 0;
}

/**
 * Register the fd in the fdset with read/write handler and context.
 */
int
fdset_add(struct fdset *pfdset, int fd, fd_cb rcb, fd_cb wcb, void *dat)
{
	int i;

	if (pfdset == NULL || fd == -1)
		return -1;

	pthread_mutex_lock(&pfdset->fd_mutex);

	i = fdset_find_free_slot(pfdset);
	if (i == -1) {
		pthread_mutex_unlock(&pfdset->fd_mutex);
		return -2;
	}

	fdset_add_fd(pfdset, i, fd, rcb, wcb, dat);
	if (i == pfdset->num)
		pfdset->num++;

	pthread_mutex_unlock(&pfdset->fd_mutex);

	return 0;
}

/**
 *  Unregister the fd from the fdset.
 *  Returns context of a given fd or NULL.
 */
void *
fdset_del(struct fdset *pfdset, int fd)
{
	int i;
	void *dat = NULL;

	if (pfdset == NULL || fd == -1)
		return NULL;

	do {
		pthread_mutex_lock(&pfdset->fd_mutex);

		i = fdset_find_fd(pfdset, fd);
		if (i != -1 && pfdset->fd[i].busy == 0) {
			/* busy indicates r/wcb is executing! */
			dat = pfdset->fd[i].dat;
			pfdset->fd[i].fd = -1;
			pfdset->fd[i].rcb = pfdset->fd[i].wcb = NULL;
			pfdset->fd[i].dat = NULL;
			(void) fdset_adjust_num(pfdset);
			i = -1;
		}
		pthread_mutex_unlock(&pfdset->fd_mutex);
	} while (i != -1);

	return dat;
}

/**
 *  Unregister the fd at the specified slot from the fdset.
 */
static void
fdset_del_slot(struct fdset *pfdset, int index)
{
	if (pfdset == NULL || index < 0 || index >= MAX_FDS)
		return;

	pthread_mutex_lock(&pfdset->fd_mutex);

	pfdset->fd[index].fd = -1;
	pfdset->fd[index].rcb = pfdset->fd[index].wcb = NULL;
	(void) fdset_adjust_num(pfdset);

	pthread_mutex_unlock(&pfdset->fd_mutex);
}


/**
 * This functions runs in infinite blocking loop until there is no fd in
 * pfdset. It calls corresponding r/w handler if there is event on the fd.
 *
 * Before the callback is called, we set the flag to busy status; If other
 * thread(now rte_vhost_driver_unregister) calls fdset_del concurrently, it
 * will wait until the flag is reset to zero(which indicates the callback is
 * finished), then it could free the context after fdset_del.
 */
void
fdset_event_dispatch(struct fdset *pfdset)
{
	int i;
	struct fdentry *pfdentry;
	int numfds;
	fd_cb rcb, wcb;
	void *dat;
	int fd;
	int remove1, remove2;
	int ret;
	int handled;

	if (pfdset == NULL)
		return;

	struct pollfd * const rwfds =
		rte_malloc("struct pollfd", MAX_FDS * sizeof(*rwfds), 0);

	while (1) {
		pthread_mutex_lock(&pfdset->fd_mutex);

		numfds = fdset_fill(rwfds, pfdset);

		pthread_mutex_unlock(&pfdset->fd_mutex);

		/*
		 * When poll is blocked, other threads might unregister
		 * listenfds from and register new listenfds into fdset.
		 * When poll returns, the entries for listenfds in the fdset
		 * might have been updated. It is ok if there is unwanted call
		 * for new listenfds.
		 */
		ret = poll(rwfds, numfds, 1000 /* millisecs */);

		if (ret <= 0)
			continue;

		for (i = handled = 0; i < numfds && handled < ret; i++) {
			if (!rwfds[i].revents)
				continue;

			handled++;
			remove1 = remove2 = 0;

			pthread_mutex_lock(&pfdset->fd_mutex);
			pfdentry = &pfdset->fd[i];
			fd = pfdentry->fd;
			rcb = pfdentry->rcb;
			wcb = pfdentry->wcb;
			dat = pfdentry->dat;
			pfdentry->busy = 1;
			pthread_mutex_unlock(&pfdset->fd_mutex);

			if (fd >= 0 && rcb &&
			    rwfds[i].revents & (POLLIN | FDPOLLERR))
				rcb(fd, dat, &remove1);
			if (fd >= 0 && wcb &&
			    rwfds[i].revents & (POLLOUT | FDPOLLERR))
				wcb(fd, dat, &remove2);
			pfdentry->busy = 0;
			/*
			 * fdset_del needs to check busy flag.
			 * We don't allow fdset_del to be called in callback
			 * directly.
			 */
			/*
			 * When we are to clean up the fd from fdset,
			 * because the fd is closed in the cb,
			 * the old fd val could be reused by when creates new
			 * listen fd in another thread, we couldn't call
			 * fd_set_del.
			 */
			if (remove1 || remove2)
				fdset_del_slot(pfdset, i);
		}
	}
}
