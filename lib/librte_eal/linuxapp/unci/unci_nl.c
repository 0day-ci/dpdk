/*-
 * GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2016 Intel Corporation. All rights reserved.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 *
 *   Contact Information:
 *   Intel Corporation
 */

#include <net/sock.h>

#include "unci_dev.h"

#define UNCI_CMD_TIMEOUT 500 /* ms */

static struct ethtool_input_buffer {
	int magic;
	void *buffer;
	size_t length;
	struct completion *msg_received;
	int *err;
	u32 in_use;
} ethtool_input_buffer;

static struct sock *nl_sock;
static struct mutex sync_lock;

static int unci_input_buffer_register(int magic, void *buffer, size_t length,
		struct completion *msg_received, int *err)
{
	if (!ethtool_input_buffer.in_use) {
		ethtool_input_buffer.magic = magic;
		ethtool_input_buffer.buffer = buffer;
		ethtool_input_buffer.length = length;
		ethtool_input_buffer.msg_received = msg_received;
		ethtool_input_buffer.err = err;
		ethtool_input_buffer.in_use = 1;
		return 0;
	}

	return 1;
}

static void unci_input_buffer_unregister(int magic)
{
	if (ethtool_input_buffer.in_use) {
		if (magic == ethtool_input_buffer.magic) {
			ethtool_input_buffer.magic = -1;
			ethtool_input_buffer.buffer = NULL;
			ethtool_input_buffer.length = 0;
			ethtool_input_buffer.msg_received = NULL;
			ethtool_input_buffer.err = NULL;
			ethtool_input_buffer.in_use = 0;
		} else {
			pr_err("Unregister magic mismatch\n");
		}
	}
}

static void nl_recv_user_request(struct unci_ethtool_msg *ethtool_msg)
{
	UNCI_DBG("Request from userspace received\n");
}

static void nl_recv_user_response(struct unci_ethtool_msg *ethtool_msg)
{
	struct completion *msg_received;
	size_t recv_len;
	size_t expected_len;

	if (ethtool_input_buffer.in_use) {
		if (ethtool_input_buffer.buffer != NULL) {
			recv_len = ethtool_msg->output_buffer_len;
			expected_len = ethtool_input_buffer.length;

			memcpy(ethtool_input_buffer.buffer,
					ethtool_msg->output_buffer,
					ethtool_input_buffer.length);

			if (ethtool_msg->err == 0 && recv_len != expected_len)
				pr_info("Expected and received len not match "
					"%zu - %zu\n", recv_len, expected_len);
		}

		*ethtool_input_buffer.err = ethtool_msg->err;
		msg_received = ethtool_input_buffer.msg_received;
		unci_input_buffer_unregister(ethtool_input_buffer.magic);
		complete(msg_received);
	}
}

static void nl_recv(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;
	struct unci_ethtool_msg ethtool_msg;

	nlh = (struct nlmsghdr *)skb->data;

	memcpy(&ethtool_msg, NLMSG_DATA(nlh), sizeof(struct unci_ethtool_msg));
	UNCI_DBG("CMD: %u\n", ethtool_msg.cmd_id);

	if (ethtool_msg.flag & UNCI_MSG_FLAG_REQUEST) {
		nl_recv_user_request(&ethtool_msg);
		return;
	}

	nl_recv_user_response(&ethtool_msg);
}

static int unci_nl_send(u32 cmd_id, u8 port_id, u32 pid, void *in_data,
		size_t in_data_len)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	struct unci_ethtool_msg ethtool_msg;

	if (pid == 0)
		return -1;

	memset(&ethtool_msg, 0, sizeof(struct unci_ethtool_msg));
	ethtool_msg.cmd_id = cmd_id;
	ethtool_msg.port_id = port_id;

	if (in_data) {
		if (in_data_len == 0 || in_data_len > UNCI_ETHTOOL_MSG_LEN)
			return -EINVAL;
		ethtool_msg.input_buffer_len = in_data_len;
		memcpy(ethtool_msg.input_buffer, in_data, in_data_len);
	}

	skb = nlmsg_new(NLMSG_ALIGN(sizeof(struct unci_ethtool_msg)),
			GFP_ATOMIC);
	nlh = nlmsg_put(skb, 0, 0, NLMSG_DONE, sizeof(struct unci_ethtool_msg),
			0);

	NETLINK_CB(skb).dst_group = 0;

	memcpy(nlmsg_data(nlh), &ethtool_msg, sizeof(struct unci_ethtool_msg));

	nlmsg_unicast(nl_sock, skb, pid);
	UNCI_DBG("Sent cmd:%u port:%u pid:%u\n", cmd_id, port_id, pid);

	return 0;
}

int unci_nl_exec(u32 cmd, struct net_device *dev, void *in_data,
		size_t in_data_len, void *out_data, size_t out_data_len)
{
	struct unci_dev *unci = netdev_priv(dev);
	int err = -EINVAL;
	int ret;

	if (out_data_len > UNCI_ETHTOOL_MSG_LEN) {
		pr_err("Message is too big to receive:%zu\n", out_data_len);
		return err;
	}

	mutex_lock(&sync_lock);
	ret = unci_input_buffer_register(cmd, out_data, out_data_len,
			&unci->msg_received, &err);
	if (ret) {
		mutex_unlock(&sync_lock);
		return -EINVAL;
	}

	ret = unci_nl_send(cmd, unci->port_id, unci->pid, in_data, in_data_len);
	if (ret) {
		unci_input_buffer_unregister(ethtool_input_buffer.magic);
		mutex_unlock(&sync_lock);
		return ret;
	}

	ret = wait_for_completion_interruptible_timeout(&unci->msg_received,
			 msecs_to_jiffies(UNCI_CMD_TIMEOUT));
	if (ret == 0 || err < 0) {
		unci_input_buffer_unregister(ethtool_input_buffer.magic);
		mutex_unlock(&sync_lock);
		if (ret == 0) { /* timeout */
			unci->nb_timedout_msg++;
			pr_info("Command timed-out for port:%u cmd:%u (%u)\n",
				unci->port_id, cmd, unci->nb_timedout_msg);
			return -EINVAL;
		}
		UNCI_DBG("Command return error for port:%d cmd:%d err:%d\n",
				unci->port_id, cmd, err);
		return err;
	}
	mutex_unlock(&sync_lock);

	return 0;
}

static struct netlink_kernel_cfg cfg = {
	.input = nl_recv,
};

void unci_nl_init(void)
{
	nl_sock = netlink_kernel_create(&init_net, UNCI_NL_GRP, &cfg);
	mutex_init(&sync_lock);
}

void unci_nl_release(void)
{
	netlink_kernel_release(nl_sock);
}
