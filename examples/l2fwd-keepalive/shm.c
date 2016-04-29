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

#include <time.h>

#include <rte_common.h>
#include <rte_log.h>
#include <rte_keepalive.h>

#include "shm.h"

struct rte_keepalive_shm *rte_keepalive_shm_create(void)
{
	int fd;
	int idx_core;
	struct rte_keepalive_shm *ka_shm;

	/* If any existing object is not unlinked, it makes it all too easy
	 * for clients to end up with stale shared memory blocks when
	 * restarted. Unlinking makes sure subsequent shm_open by clients
	 * will get the new block mapped below.
	 */
	if (shm_unlink(RTE_KEEPALIVE_SHM_NAME) == -1 && errno != ENOENT)
		printf("Warning: Error unlinking stale %s (%s)\n",
			RTE_KEEPALIVE_SHM_NAME, strerror(errno));

	fd = shm_open(RTE_KEEPALIVE_SHM_NAME,
		O_CREAT | O_TRUNC | O_RDWR, 0666);
	if (fd < 0)
		RTE_LOG(INFO, EAL,
			"Failed to open %s as SHM (%s)\n",
			RTE_KEEPALIVE_SHM_NAME,
			strerror(errno));
	else if (ftruncate(fd, sizeof(struct rte_keepalive_shm)) != 0)
		RTE_LOG(INFO, EAL,
			"Failed to resize SHM (%s)\n", strerror(errno));
	else {
		ka_shm = (struct rte_keepalive_shm *) mmap(
			0, sizeof(struct rte_keepalive_shm),
			PROT_READ | PROT_WRITE,	MAP_SHARED, fd, 0);
		close(fd);
		if (ka_shm == MAP_FAILED)
			RTE_LOG(INFO, EAL,
				"Failed to mmap SHM (%s)\n", strerror(errno));
		else {
			memset(ka_shm, 0, sizeof(struct rte_keepalive_shm));

			/* Initialize the semaphores for IPC/SHM use */
			if (sem_init(&ka_shm->core_died, 1, 0) != 0) {
				RTE_LOG(INFO, EAL,
					"Failed to setup SHM semaphore (%s)\n",
					strerror(errno));
				return NULL;
			}

			/* Set all cores to 'not present' */
			for (idx_core = 0;
				idx_core < RTE_KEEPALIVE_MAXCORES;
				idx_core++)
				ka_shm->core_state[idx_core] = -1;

			/* Set magic number so agent knows setup
			 * has finished.
			 */
			ka_shm->magic = RTE_KEEPALIVE_SHM_MAGIC;
			ka_shm->time_of_init = time(NULL);

			return ka_shm;
		}
	}
return NULL;
}

void rte_keepalive_shm_alive(struct rte_keepalive_shm *shm, const int id_core)
{
	shm->core_state[id_core] = RTE_KEEPALIVE_SHM_ALIVE;
}

void rte_keepalive_shm_dead(struct rte_keepalive_shm *shm, const int id_core)
{
	int count;

	shm->core_state[id_core] = RTE_KEEPALIVE_SHM_DEAD;

	/* Limit number of times semaphore can be incremented, in case
	 * listening agent is not active.
	 */
	if (sem_getvalue(&shm->core_died, &count) == -1) {
		RTE_LOG(INFO, EAL, "Semaphore check failed(%s)\n",
			strerror(errno));
		return;
	}
	if (count > 1)
		return;

	if (sem_post(&shm->core_died) != 0)
		RTE_LOG(INFO, EAL,
			"Failed to increment semaphore (%s)\n",
			strerror(errno));
}
