/*-
 * BSD LICENSE
 *
 * Copyright (c) 2015-2017 Atomic Rules LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * * Neither the name of copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ARK_DEBUG_H_
#define _ARK_DEBUG_H_

#include <rte_log.h>

/* Format specifiers for string data pairs */
#define FMT_SU32  "\n\t%-20s    %'20u"
#define FMT_SU64  "\n\t%-20s    %'20lu"
#define FMT_SPTR  "\n\t%-20s    %20p"

#define ARK_TRACE_ON(fmt, ...) \
  fprintf(stderr, fmt, ##__VA_ARGS__)

#define ARK_TRACE_OFF(fmt, ...) \
  do {if (0) fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)

/* Debug macro for reporting Packet stats */
#ifdef RTE_LIBRTE_ARK_DEBUG_STATS
#define ARK_DEBUG_STATS(fmt, ...) ARK_TRACE_ON(fmt, ##__VA_ARGS__)
#else
#define ARK_DEBUG_STATS(fmt, ...)  ARK_TRACE_OFF(fmt, ##__VA_ARGS__)
#endif

/* Debug macro for tracing full behavior*/
#ifdef RTE_LIBRTE_ARK_DEBUG_TRACE
#define ARK_DEBUG_TRACE(fmt, ...)  ARK_TRACE_ON(fmt, ##__VA_ARGS__)
#else
#define ARK_DEBUG_TRACE(fmt, ...)  ARK_TRACE_OFF(fmt, ##__VA_ARGS__)
#endif

#ifdef ARK_STD_LOG
#define PMD_DRV_LOG(level, fmt, args...) \
  fprintf(stderr, fmt, args)
#else
#define PMD_DRV_LOG(level, fmt, args...) \
	RTE_LOG(level, PMD, "%s(): " fmt, __func__, ## args)
#endif

#endif
