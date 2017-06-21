/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
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

#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <linux/netlink.h>

#include <rte_spinlock.h>
#include <rte_log.h>
#include "rte_ctrl_ethtool.h"
#include "rte_nl.h"
#include "rte_ctrl_if.h"

#define MAX_PAYLOAD sizeof(struct unci_ethtool_msg)

struct ctrl_if_nl {
	union {
		struct nlmsghdr nlh;
		uint8_t nlmsg[NLMSG_SPACE(MAX_PAYLOAD)];
	};
	struct msghdr msg;
	struct iovec iov;
	struct sockaddr_nl dest_addr;
};

struct ctrl_if_msg_sync {
	struct unci_ethtool_msg msg_storage;
	pthread_mutex_t msg_lock;
	uint32_t pending_process;
};


/**
 * Flags values for rte_eth_control_interface_process_msg() API
 */
enum control_interface_process_flag {
	/**< Process if msg available. */
	RTE_ETHTOOL_CTRL_IF_PROCESS_MSG,

	/**< Discard msg if available, respond with a error value. */
	RTE_ETHTOOL_CTRL_IF_DISCARD_MSG,
};

static int sock_fd = -1;
static pthread_t thread_id;

static struct ctrl_if_nl nl_s;
static struct ctrl_if_nl nl_r;

static struct ctrl_if_msg_sync ctrl_if_sync = {
	.msg_lock = PTHREAD_MUTEX_INITIALIZER,
};

static int
nl_send(void *buf, size_t len)
{
	int ret;

	if (nl_s.nlh.nlmsg_len < len) {
		RTE_LOG(ERR, CTRL_IF, "Message is too big, len:%zu\n", len);
		return -1;
	}

	if (!NLMSG_OK(&nl_s.nlh, NLMSG_SPACE(MAX_PAYLOAD))) {
		RTE_LOG(ERR, CTRL_IF, "Message is not OK\n");
		return -1;
	}

	/* Fill in the netlink message payload */
	memcpy(NLMSG_DATA(nl_s.nlmsg), buf, len);

	ret = sendmsg(sock_fd, &nl_s.msg, 0);

	if (ret < 0)
		RTE_LOG(ERR, CTRL_IF, "Failed nl msg send. ret:%d, err:%d\n",
				ret, errno);
	return ret;
}

static int
nl_ethtool_msg_send(struct unci_ethtool_msg *msg)
{
	return nl_send((void *)msg, sizeof(struct unci_ethtool_msg));
}

static void
process_msg(struct unci_ethtool_msg *msg)
{
	if (msg->cmd_id > RTE_UNCI_REQ_UNKNOWN) {
		msg->err = rte_eth_dev_control_process(msg->cmd_id,
				msg->port_id, msg->input_buffer,
				msg->output_buffer, &msg->output_buffer_len);
	} else {
		msg->err = rte_eth_dev_ethtool_process(msg->cmd_id,
				msg->port_id, msg->input_buffer,
				msg->output_buffer, &msg->output_buffer_len);
	}

	if (msg->err)
		memset(msg->output_buffer, 0, msg->output_buffer_len);

	nl_ethtool_msg_send(msg);
}

static int
control_interface_msg_process(uint32_t flag)
{
	struct unci_ethtool_msg msg_storage;
	int ret = 0;

	pthread_mutex_lock(&ctrl_if_sync.msg_lock);
	if (ctrl_if_sync.pending_process == 0) {
		pthread_mutex_unlock(&ctrl_if_sync.msg_lock);
		return 0;
	}

	memcpy(&msg_storage, &ctrl_if_sync.msg_storage,
			sizeof(struct unci_ethtool_msg));
	ctrl_if_sync.pending_process = 0;
	pthread_mutex_unlock(&ctrl_if_sync.msg_lock);

	switch (flag) {
	case RTE_ETHTOOL_CTRL_IF_PROCESS_MSG:
		process_msg(&msg_storage);
		break;

	case RTE_ETHTOOL_CTRL_IF_DISCARD_MSG:
		msg_storage.err = -1;
		nl_ethtool_msg_send(&msg_storage);
		break;

	default:
		ret = -1;
		break;
	}

	return ret;
}

static int
msg_add_and_process(struct nlmsghdr *nlh)
{
	pthread_mutex_lock(&ctrl_if_sync.msg_lock);

	if (ctrl_if_sync.pending_process) {
		pthread_mutex_unlock(&ctrl_if_sync.msg_lock);
		return -1;
	}

	memcpy(&ctrl_if_sync.msg_storage, NLMSG_DATA(nlh),
			sizeof(struct unci_ethtool_msg));
	ctrl_if_sync.msg_storage.flag = UNCI_MSG_FLAG_RESPONSE;
	ctrl_if_sync.pending_process = 1;

	pthread_mutex_unlock(&ctrl_if_sync.msg_lock);

	control_interface_msg_process(RTE_ETHTOOL_CTRL_IF_PROCESS_MSG);

	return 0;
}

static void *
nl_recv(void *arg)
{
	int ret;

	for (;;) {
		ret = recvmsg(sock_fd, &nl_r.msg, 0);
		if (ret < 0)
			continue;

		if ((unsigned int)ret < sizeof(struct unci_ethtool_msg)) {
			RTE_LOG(WARNING, CTRL_IF,
					"Received %d bytes, payload %zu\n",
					ret, sizeof(struct unci_ethtool_msg));
			continue;
		}

		msg_add_and_process(&nl_r.nlh);
	}

	return arg;
}

static void
nl_setup_header(struct ctrl_if_nl *nl)
{
	nl->dest_addr.nl_family = AF_NETLINK;
	nl->dest_addr.nl_pid = 0;   /*  For Linux Kernel */
	nl->dest_addr.nl_groups = 0;

	memset(nl->nlmsg, 0, NLMSG_SPACE(MAX_PAYLOAD));

	/* Fill the netlink message header */
	nl->nlh.nlmsg_len = NLMSG_LENGTH(MAX_PAYLOAD);
	nl->nlh.nlmsg_pid = getpid();  /* self pid */
	nl->nlh.nlmsg_flags = 0;

	nl->iov.iov_base = (void *)nl->nlmsg;
	nl->iov.iov_len = nl->nlh.nlmsg_len;
	memset(&nl->msg, 0, sizeof(struct msghdr));
	nl->msg.msg_name = (void *)&nl->dest_addr;
	nl->msg.msg_namelen = sizeof(struct sockaddr_nl);
	nl->msg.msg_iov = &nl->iov;
	nl->msg.msg_iovlen = 1;
}

static int
nl_socket_init(void)
{
	struct sockaddr_nl src_addr;
	int fd;
	int ret;

	fd = socket(PF_NETLINK, SOCK_RAW, UNCI_NL_GRP);
	if (fd < 0)
		return -1;

	src_addr.nl_family = AF_NETLINK;
	src_addr.nl_pid = getpid();
	ret = bind(fd, (struct sockaddr *)&src_addr, sizeof(src_addr));
	if (ret) {
		close(fd);
		return -1;
	}

	nl_setup_header(&nl_s);
	nl_setup_header(&nl_r);

	return fd;
}

int
control_interface_nl_init(void)
{
	int ret;

	sock_fd = nl_socket_init();
	if (sock_fd < 0) {
		RTE_LOG(ERR, CTRL_IF, "Failed to initialize netlink socket\n");
		return -1;
	}

	ret = pthread_create(&thread_id, NULL, nl_recv, NULL);
	if (ret != 0) {
		RTE_LOG(ERR, CTRL_IF, "Failed to create receive thread\n");
		return -1;
	}

	return 0;
}

void
control_interface_nl_release(void)
{
	pthread_cancel(thread_id);
	pthread_join(thread_id, NULL);
	close(sock_fd);
}
