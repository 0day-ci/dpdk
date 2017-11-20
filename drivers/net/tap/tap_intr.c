/*-
 *   BSD LICENSE
 *
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
 *     * Neither the name of the copyright holder nor the names of its
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

/**
 * @file
 * Interrupts handling for tap driver.
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include <rte_eth_tap.h>
#include <rte_errno.h>
#include <rte_interrupts.h>


/**
 * Unregister Rx interrupts free the queue interrupt vector.
 *
 * @param dev
 *   Pointer to the tap rte_eth_dev structure.
 */
static void
tap_rx_intr_vec_uninstall(struct rte_eth_dev *dev)
{
	struct pmd_internals *pmd = dev->data->dev_private;
	struct rte_intr_handle *intr_handle = &pmd->intr_handle;

	rte_intr_free_epoll_fd(intr_handle);
	free(intr_handle->intr_vec);
	intr_handle->intr_vec = NULL;
	intr_handle->nb_efd = 0;
}

/**
 * Allocate Rx queue interrupt vector and register Rx interrupts.
 *
 * @param dev
 *   Pointer to the tap rte_eth_dev device structure.
 *
 * @return
 *   0 on success, negative errno value otherwise and rte_errno is set.
 */
static int
tap_rx_intr_vec_install(struct rte_eth_dev *dev)
{
	struct pmd_internals *pmd = dev->data->dev_private;
	unsigned int rxqs_n = pmd->dev->data->nb_rx_queues;
	struct rte_intr_handle *intr_handle = &pmd->intr_handle;
	unsigned int n = RTE_MIN(rxqs_n, (uint32_t)RTE_MAX_RXTX_INTR_VEC_ID);
	unsigned int i;
	unsigned int count = 0;

	if (!dev->data->dev_conf.intr_conf.rxq)
		return 0;
	intr_handle->intr_vec = malloc(sizeof(intr_handle->intr_vec[rxqs_n]));
	if (intr_handle->intr_vec == NULL) {
		rte_errno = ENOMEM;
		RTE_LOG(ERR, PMD,
			"failed to allocate memory for interrupt vector,"
			" Rx interrupts will not be supported");
		return -rte_errno;
	}
	for (i = 0; i < n; i++) {
		struct rx_queue *rxq = pmd->dev->data->rx_queues[i];

		/* Skip queues that cannot request interrupts. */
		if (!rxq || (rxq->fd <= 0)) {
			/* Use invalid intr_vec[] index to disable entry. */
			intr_handle->intr_vec[i] =
				RTE_INTR_VEC_RXTX_OFFSET +
				RTE_MAX_RXTX_INTR_VEC_ID;
			continue;
		}
		if (count >= RTE_MAX_RXTX_INTR_VEC_ID) {
			rte_errno = E2BIG;
			RTE_LOG(ERR, PMD,
				"too many Rx queues for interrupt vector size"
				" (%d), Rx interrupts cannot be enabled",
				RTE_MAX_RXTX_INTR_VEC_ID);
			tap_rx_intr_vec_uninstall(dev);
			return -rte_errno;
		}
		intr_handle->intr_vec[i] = RTE_INTR_VEC_RXTX_OFFSET + count;
		intr_handle->efds[count] = rxq->fd;
		count++;
	}
	if (!count)
		tap_rx_intr_vec_uninstall(dev);
	else
		intr_handle->nb_efd = count;
	return 0;
}

/**
 * Register or unregister the Rx interrupts.
 *
 * @param dev
 *   Pointer to the tap rte_eth_dev device structure.
 * @param set
 *   should the operation be register or unregister the interrupts.
 *
 * @return
 *   0 on success, negative errno value otherwise and rte_errno is set.
 */
int
tap_rx_intr_vec_set(struct rte_eth_dev *dev, int set)
{
	tap_rx_intr_vec_uninstall(dev);
	if (set)
		return tap_rx_intr_vec_install(dev);
	return 0;
}
