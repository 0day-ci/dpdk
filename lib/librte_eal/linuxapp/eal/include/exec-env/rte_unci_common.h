/*-
 *   This file is provided under a dual BSD/LGPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 *
 *   GNU LESSER GENERAL PUBLIC LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2.1 of the GNU Lesser General Public License
 *   as published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program;
 *
 *   Contact Information:
 *   Intel Corporation
 *
 *
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _RTE_UNCI_COMMON_H_
#define _RTE_UNCI_COMMON_H_

#define UNCI_DEVICE "unci"

#define UNCI_NL_GRP 31

#define UNCI_ETHTOOL_MSG_LEN 500
struct unci_ethtool_msg {
	uint32_t cmd_id;
	uint8_t port_id;
	uint32_t flag;
	uint8_t input_buffer[UNCI_ETHTOOL_MSG_LEN];
	uint8_t output_buffer[UNCI_ETHTOOL_MSG_LEN];
	size_t input_buffer_len;
	size_t output_buffer_len;
	int err;
};

enum unci_ethtool_msg_flag {
	UNCI_MSG_FLAG_NONE,
	UNCI_MSG_FLAG_REQUEST,
	UNCI_MSG_FLAG_RESPONSE,
};

enum {
	IFLA_UNCI_UNSPEC,
	IFLA_UNCI_PORTID,
	IFLA_UNCI_PID,
	__IFLA_UNCI_MAX,
};

#define IFLA_UNCI_MAX (__IFLA_UNCI_MAX - 1)

/*
 * Request id.
 */
enum rte_unci_req_id {
	RTE_UNCI_REQ_UNKNOWN = (1 << 16),
	RTE_UNCI_REQ_CHANGE_MTU,
	RTE_UNCI_REQ_CFG_NETWORK_IF,
	RTE_UNCI_REQ_GET_STATS,
	RTE_UNCI_REQ_GET_MAC,
	RTE_UNCI_REQ_SET_MAC,
	RTE_UNCI_REQ_START_PORT,
	RTE_UNCI_REQ_STOP_PORT,
	RTE_UNCI_REQ_SET_PROMISC,
	RTE_UNCI_REQ_SET_ALLMULTI,
	RTE_UNCI_REQ_MAX,
};

#endif /* _RTE_UNCI_COMMON_H_ */
