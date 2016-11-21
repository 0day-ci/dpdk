/*-
 * Copyright (c) 2016 Solarflare Communications Inc.
 * All rights reserved.
 *
 * This software was jointly developed between OKTET Labs (under contract
 * for Solarflare) and Solarflare Communications, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <rte_cycles.h>

#include "efx.h"
#include "efx_mcdi.h"
#include "efx_regs_mcdi.h"

#include "sfc.h"
#include "sfc_log.h"

#define	SFC_MCDI_POLL_INTERVAL_MIN_US	10		/* 10us in 1us units */
#define	SFC_MCDI_POLL_INTERVAL_MAX_US	(US_PER_S / 10)	/* 100ms in 1us units */
#define	SFC_MCDI_WATCHDOG_INTERVAL_US	(10 * US_PER_S)	/* 10s in 1us units */

static void
sfc_mcdi_timeout(struct sfc_adapter *sa)
{
	sfc_warn(sa, "MC TIMEOUT");

	sfc_panic(sa, "MCDI timeout handling is not implemented\n");
}

static void
sfc_mcdi_poll(struct sfc_adapter *sa)
{
	efx_nic_t *enp;
	unsigned int delay_total;
	unsigned int delay_us;
	boolean_t aborted;

	delay_total = 0;
	delay_us = SFC_MCDI_POLL_INTERVAL_MIN_US;
	enp = sa->nic;

	do {
		if (efx_mcdi_request_poll(enp))
			return;

		if (delay_total > SFC_MCDI_WATCHDOG_INTERVAL_US) {
			aborted = efx_mcdi_request_abort(enp);
			SFC_ASSERT(aborted);
			sfc_mcdi_timeout(sa);
			return;
		}

		rte_delay_us(delay_us);

		delay_total += delay_us;

		/* Exponentially back off the poll frequency */
		RTE_BUILD_BUG_ON(SFC_MCDI_POLL_INTERVAL_MAX_US > UINT_MAX / 2);
		delay_us *= 2;
		if (delay_us > SFC_MCDI_POLL_INTERVAL_MAX_US)
			delay_us = SFC_MCDI_POLL_INTERVAL_MAX_US;

	} while (1);
}

static void
sfc_mcdi_execute(void *arg, efx_mcdi_req_t *emrp)
{
	struct sfc_adapter *sa = (struct sfc_adapter *)arg;
	struct sfc_mcdi *mcdi = &sa->mcdi;

	rte_spinlock_lock(&mcdi->lock);

	SFC_ASSERT(mcdi->state == SFC_MCDI_INITIALIZED);

	efx_mcdi_request_start(sa->nic, emrp, B_FALSE);
	sfc_mcdi_poll(sa);

	rte_spinlock_unlock(&mcdi->lock);
}

static void
sfc_mcdi_ev_cpl(void *arg)
{
	struct sfc_adapter *sa = (struct sfc_adapter *)arg;
	struct sfc_mcdi *mcdi = &sa->mcdi;

	SFC_ASSERT(mcdi->state == SFC_MCDI_INITIALIZED);

	/* MCDI is polled, completions are not expected */
	SFC_ASSERT(0);
}

static void
sfc_mcdi_exception(void *arg, efx_mcdi_exception_t eme)
{
	struct sfc_adapter *sa = (struct sfc_adapter *)arg;

	sfc_warn(sa, "MC %s",
	    (eme == EFX_MCDI_EXCEPTION_MC_REBOOT) ? "REBOOT" :
	    (eme == EFX_MCDI_EXCEPTION_MC_BADASSERT) ? "BADASSERT" : "UNKNOWN");

	sfc_panic(sa, "MCDI exceptions handling is not implemented\n");
}

int
sfc_mcdi_init(struct sfc_adapter *sa)
{
	struct sfc_mcdi *mcdi;
	size_t max_msg_size;
	efx_mcdi_transport_t *emtp;
	int rc;

	sfc_log_init(sa, "entry");

	mcdi = &sa->mcdi;

	SFC_ASSERT(mcdi->state == SFC_MCDI_UNINITIALIZED);

	rte_spinlock_init(&mcdi->lock);

	mcdi->state = SFC_MCDI_INITIALIZED;

	max_msg_size = sizeof(uint32_t) + MCDI_CTL_SDU_LEN_MAX_V2;
	rc = sfc_dma_alloc(sa, "mcdi", 0, max_msg_size, sa->socket_id,
			   &mcdi->mem);
	if (rc != 0)
		goto fail_dma_alloc;

	emtp = &mcdi->transport;
	emtp->emt_context = sa;
	emtp->emt_dma_mem = &mcdi->mem;
	emtp->emt_execute = sfc_mcdi_execute;
	emtp->emt_ev_cpl = sfc_mcdi_ev_cpl;
	emtp->emt_exception = sfc_mcdi_exception;

	sfc_log_init(sa, "init MCDI");
	rc = efx_mcdi_init(sa->nic, emtp);
	if (rc != 0)
		goto fail_mcdi_init;

	return 0;

fail_mcdi_init:
	memset(emtp, 0, sizeof(*emtp));
	sfc_dma_free(sa, &mcdi->mem);

fail_dma_alloc:
	mcdi->state = SFC_MCDI_UNINITIALIZED;
	return rc;
}

void
sfc_mcdi_fini(struct sfc_adapter *sa)
{
	struct sfc_mcdi *mcdi;
	efx_mcdi_transport_t *emtp;

	sfc_log_init(sa, "entry");

	mcdi = &sa->mcdi;
	emtp = &mcdi->transport;

	rte_spinlock_lock(&mcdi->lock);

	SFC_ASSERT(mcdi->state == SFC_MCDI_INITIALIZED);
	mcdi->state = SFC_MCDI_UNINITIALIZED;

	sfc_log_init(sa, "fini MCDI");
	efx_mcdi_fini(sa->nic);
	memset(emtp, 0, sizeof(*emtp));

	rte_spinlock_unlock(&mcdi->lock);

	sfc_dma_free(sa, &mcdi->mem);
}
