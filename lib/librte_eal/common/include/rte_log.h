/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2017 Intel Corporation. All rights reserved.
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

#ifndef _RTE_LOG_H_
#define _RTE_LOG_H_

/**
 * @file
 *
 * RTE Logs API
 *
 * This file provides a log API to RTE applications.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#include <rte_common.h>

struct rte_log_dynamic_type;

/** The rte_log structure. */
struct rte_logs {
	uint32_t type;  /**< Bitfield with enabled logs. */
	uint32_t level; /**< Log level. */
	FILE *file;     /**< Output file set by rte_openlog_stream, or NULL. */
	size_t dynamic_types_len;
	struct rte_log_dynamic_type *dynamic_types;
};

/** Global log informations */
extern struct rte_logs rte_logs;

/* SDK log type */
#define RTE_LOGTYPE_EAL        0 /**< Log related to eal. */
#define RTE_LOGTYPE_MALLOC     1 /**< Log related to malloc. */
#define RTE_LOGTYPE_RING       2 /**< Log related to ring. */
#define RTE_LOGTYPE_MEMPOOL    3 /**< Log related to mempool. */
#define RTE_LOGTYPE_TIMER      4 /**< Log related to timers. */
#define RTE_LOGTYPE_PMD        5 /**< Log related to poll mode driver. */
#define RTE_LOGTYPE_HASH       6 /**< Log related to hash table. */
#define RTE_LOGTYPE_LPM        7 /**< Log related to LPM. */
#define RTE_LOGTYPE_KNI        8 /**< Log related to KNI. */
#define RTE_LOGTYPE_ACL        9 /**< Log related to ACL. */
#define RTE_LOGTYPE_POWER     10 /**< Log related to power. */
#define RTE_LOGTYPE_METER     11 /**< Log related to QoS meter. */
#define RTE_LOGTYPE_SCHED     12 /**< Log related to QoS port scheduler. */
#define RTE_LOGTYPE_PORT      13 /**< Log related to port. */
#define RTE_LOGTYPE_TABLE     14 /**< Log related to table. */
#define RTE_LOGTYPE_PIPELINE  15 /**< Log related to pipeline. */
#define RTE_LOGTYPE_MBUF      16 /**< Log related to mbuf. */
#define RTE_LOGTYPE_CRYPTODEV 17 /**< Log related to cryptodev. */
#define RTE_LOGTYPE_EFD       18 /**< Log related to EFD. */
#define RTE_LOGTYPE_EVENTDEV  19 /**< Log related to eventdev. */
#define RTE_LOGTYPE_BUS       20 /**< Log related to Bus. */

/* these log types can be used in an application */
#define RTE_LOGTYPE_USER1     24 /**< User-defined log type 1. */
#define RTE_LOGTYPE_USER2     25 /**< User-defined log type 2. */
#define RTE_LOGTYPE_USER3     26 /**< User-defined log type 3. */
#define RTE_LOGTYPE_USER4     27 /**< User-defined log type 4. */
#define RTE_LOGTYPE_USER5     28 /**< User-defined log type 5. */
#define RTE_LOGTYPE_USER6     29 /**< User-defined log type 6. */
#define RTE_LOGTYPE_USER7     30 /**< User-defined log type 7. */
#define RTE_LOGTYPE_USER8     31 /**< User-defined log type 8. */

/** First identifier for extended logs */
#define RTE_LOGTYPE_FIRST_EXT_ID 32

/* Can't use 0, as it gives compiler warnings */
#define RTE_LOG_EMERG    1U  /**< System is unusable.               */
#define RTE_LOG_ALERT    2U  /**< Action must be taken immediately. */
#define RTE_LOG_CRIT     3U  /**< Critical conditions.              */
#define RTE_LOG_ERR      4U  /**< Error conditions.                 */
#define RTE_LOG_WARNING  5U  /**< Warning conditions.               */
#define RTE_LOG_NOTICE   6U  /**< Normal but significant condition. */
#define RTE_LOG_INFO     7U  /**< Informational.                    */
#define RTE_LOG_DEBUG    8U  /**< Debug-level messages.             */

/**
 * Change the stream that will be used by the logging system.
 *
 * This can be done at any time. The f argument represents the stream
 * to be used to send the logs. If f is NULL, the default output is
 * used (stderr).
 *
 * @param f
 *   Pointer to the stream.
 * @return
 *   - 0 on success.
 *   - Negative on error.
 */
int rte_openlog_stream(FILE *f);

/**
 * Set the global log level.
 *
 * After this call, logs with a level lower or equal than the level
 * passed as argument will be displayed.
 *
 * @param level
 *   Log level. A value between RTE_LOG_EMERG (1) and RTE_LOG_DEBUG (8).
 */
void rte_log_set_global_level(uint32_t level);

/**
 * Deprecated, replaced by rte_log_set_global_level().
 */
__rte_deprecated
void rte_set_log_level(uint32_t level);

/**
 * Get the global log level.
 *
 * @return
 *   The current global log level.
 */
uint32_t rte_log_get_global_level(void);

/**
 * Deprecated, replaced by rte_log_get_global_level().
 */
__rte_deprecated
uint32_t rte_get_log_level(void);

/**
 * Enable or disable the log type.
 *
 * @param type
 *   Log type, for example, RTE_LOGTYPE_EAL.
 * @param enable
 *   True for enable; false for disable.
 */
__rte_deprecated
void rte_set_log_type(uint32_t type, int enable);

/**
 * Get the global log type.
 */
__rte_deprecated
uint32_t rte_get_log_type(void);

/**
 * Set the log level for a given type.
 *
 * @param pattern
 *   The regexp identifying the log type.
 * @param level
 *   The level to be set.
 * @return
 *   0 on success, a negative value if level is invalid.
 */
int rte_log_set_level_regexp(const char *pattern, uint32_t level);

/**
 * Set the log level for a given type.
 *
 * @param logtype
 *   The log type identifier.
 * @param level
 *   The level to be set.
 * @return
 *   0 on success, a negative value if logtype or level is invalid.
 */
int rte_log_set_level(uint32_t logtype, uint32_t level);

/**
 * Get the current loglevel for the message being processed.
 *
 * Before calling the user-defined stream for logging, the log
 * subsystem sets a per-lcore variable containing the loglevel and the
 * logtype of the message being processed. This information can be
 * accessed by the user-defined log output function through this
 * function.
 *
 * @return
 *   The loglevel of the message being processed.
 */
int rte_log_cur_msg_loglevel(void);

/**
 * Get the current logtype for the message being processed.
 *
 * Before calling the user-defined stream for logging, the log
 * subsystem sets a per-lcore variable containing the loglevel and the
 * logtype of the message being processed. This information can be
 * accessed by the user-defined log output function through this
 * function.
 *
 * @return
 *   The logtype of the message being processed.
 */
int rte_log_cur_msg_logtype(void);

/**
 * Register a dynamic log type
 *
 * If a log is already registered with the same type, the returned value
 * is the same than the previous one.
 *
 * @param name
 *   The string identifying the log type.
 * @return
 *   - >0: success, the returned value is the log type identifier.
 *   - (-ENONEM): cannot allocate memory.
 */
int rte_log_register(const char *name);

/**
 * Dump log information.
 *
 * Dump the global level and the registered log types.
 *
 * @param f
 *   The output stream where the dump should be sent.
 */
void rte_log_dump(FILE *f);

/**
 * Generates a log message.
 *
 * The message will be sent in the stream defined by the previous call
 * to rte_openlog_stream().
 *
 * The level argument determines if the log should be displayed or
 * not, depending on the global rte_logs variable.
 *
 * The preferred alternative is the RTE_LOG() because it adds the
 * level and type in the logged string.
 *
 * @param level
 *   Log level. A value between RTE_LOG_EMERG (1) and RTE_LOG_DEBUG (8).
 * @param logtype
 *   The log type, for example, RTE_LOGTYPE_EAL.
 * @param format
 *   The format string, as in printf(3), followed by the variable arguments
 *   required by the format.
 * @return
 *   - 0: Success.
 *   - Negative on error.
 */
int rte_log(uint32_t level, uint32_t logtype, const char *format, ...)
#ifdef __GNUC__
#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 2))
	__attribute__((cold))
#endif
#endif
	__attribute__((format(printf, 3, 4)));

/**
 * Generates a log message.
 *
 * The message will be sent in the stream defined by the previous call
 * to rte_openlog_stream().
 *
 * The level argument determines if the log should be displayed or
 * not, depending on the global rte_logs variable. A trailing
 * newline may be added if needed.
 *
 * The preferred alternative is the RTE_LOG() because it adds the
 * level and type in the logged string.
 *
 * @param level
 *   Log level. A value between RTE_LOG_EMERG (1) and RTE_LOG_DEBUG (8).
 * @param logtype
 *   The log type, for example, RTE_LOGTYPE_EAL.
 * @param format
 *   The format string, as in printf(3), followed by the variable arguments
 *   required by the format.
 * @param ap
 *   The va_list of the variable arguments required by the format.
 * @return
 *   - 0: Success.
 *   - Negative on error.
 */
int rte_vlog(uint32_t level, uint32_t logtype, const char *format, va_list ap)
	__attribute__((format(printf,3,0)));

/**
 * Generates a log message.
 *
 * The RTE_LOG() is a helper that prefixes the string with the log level
 * and type, and call rte_log().
 *
 * @param l
 *   Log level. A value between EMERG (1) and DEBUG (8). The short name is
 *   expanded by the macro, so it cannot be an integer value.
 * @param t
 *   The log type, for example, EAL. The short name is expanded by the
 *   macro, so it cannot be an integer value.
 * @param ...
 *   The fmt string, as in printf(3), followed by the variable arguments
 *   required by the format.
 * @return
 *   - 0: Success.
 *   - Negative on error.
 */
#define RTE_LOG(l, t, ...)					\
	 rte_log(RTE_LOG_ ## l,					\
		 RTE_LOGTYPE_ ## t, # t ": " __VA_ARGS__)

/**
 * Generates a log message for data path.
 *
 * Similar to RTE_LOG(), except that it is removed at compilation time
 * if the RTE_LOG_DP_LEVEL configuration option is lower than the log
 * level argument.
 *
 * @param l
 *   Log level. A value between EMERG (1) and DEBUG (8). The short name is
 *   expanded by the macro, so it cannot be an integer value.
 * @param t
 *   The log type, for example, EAL. The short name is expanded by the
 *   macro, so it cannot be an integer value.
 * @param ...
 *   The fmt string, as in printf(3), followed by the variable arguments
 *   required by the format.
 * @return
 *   - 0: Success.
 *   - Negative on error.
 */
#define RTE_LOG_DP(l, t, ...)					\
	(void)((RTE_LOG_ ## l <= RTE_LOG_DP_LEVEL) ?		\
	 rte_log(RTE_LOG_ ## l,					\
		 RTE_LOGTYPE_ ## t, # t ": " __VA_ARGS__) :	\
	 0)

#ifdef __cplusplus
}
#endif

#endif /* _RTE_LOG_H_ */
