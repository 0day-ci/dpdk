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
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
 *   DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>

#include <rte_ethdev.h>

#include "rte_eth_softnic_internals.h"
#include "rte_eth_softnic.h"

#define UDEV(odev)						\
	((struct pmd_internals *) ((odev)->data->dev_private))->udev

static int
pmd_dev_configure(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	return dev->dev_ops->dev_configure(dev);
}

static int
pmd_dev_start(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	return dev->dev_ops->dev_start(dev);
}

static void
pmd_dev_stop(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	dev->dev_ops->dev_stop(dev);
}

static int
pmd_dev_set_link_up(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	return dev->dev_ops->dev_set_link_up(dev);
}

static int
pmd_dev_set_link_down(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	return dev->dev_ops->dev_set_link_down(dev);
}

static void
pmd_dev_close(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	dev->dev_ops->dev_close(dev);
}

static void
pmd_promiscuous_enable(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	dev->dev_ops->promiscuous_enable(dev);
}

static void
pmd_promiscuous_disable(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	dev->dev_ops->promiscuous_disable(dev);
}

static void
pmd_allmulticast_enable(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	dev->dev_ops->allmulticast_enable(dev);
}

static void
pmd_allmulticast_disable(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	dev->dev_ops->allmulticast_disable(dev);
}

static int
pmd_link_update(struct rte_eth_dev *dev, int wait_to_complete)
{
	dev = UDEV(dev);

	return dev->dev_ops->link_update(dev, wait_to_complete);
}

static void
pmd_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *igb_stats)
{
	dev = UDEV(dev);

	dev->dev_ops->stats_get(dev, igb_stats);
}

static void
pmd_stats_reset(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	dev->dev_ops->stats_reset(dev);
}

static int
pmd_xstats_get(struct rte_eth_dev *dev,
	struct rte_eth_xstat *stats, unsigned n)
{
	dev = UDEV(dev);

	return dev->dev_ops->xstats_get(dev, stats, n);
}

static int
pmd_xstats_get_by_id(struct rte_eth_dev *dev,
	const uint64_t *ids, uint64_t *values, unsigned int n)
{
	dev = UDEV(dev);

	return dev->dev_ops->xstats_get_by_id(dev, ids, values, n);
}

static void
pmd_xstats_reset(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	dev->dev_ops->xstats_reset(dev);
}

static int
pmd_xstats_get_names(struct rte_eth_dev *dev,
	struct rte_eth_xstat_name *xstats_names, unsigned size)
{
	dev = UDEV(dev);

	return dev->dev_ops->xstats_get_names(dev, xstats_names, size);
}

static int
pmd_xstats_get_names_by_id(struct rte_eth_dev *dev,
	struct rte_eth_xstat_name *xstats_names, const uint64_t *ids,
	unsigned int size)
{
	dev = UDEV(dev);

	return dev->dev_ops->xstats_get_names_by_id(dev, xstats_names, ids, size);
}

static int
pmd_queue_stats_mapping_set(struct rte_eth_dev *dev,
	uint16_t queue_id, uint8_t stat_idx, uint8_t is_rx)
{
	dev = UDEV(dev);

	return dev->dev_ops->queue_stats_mapping_set(dev, queue_id,
		stat_idx, is_rx);
}

static void
pmd_dev_infos_get(struct rte_eth_dev *dev,
	struct rte_eth_dev_info *dev_info)
{
	dev = UDEV(dev);

	dev->dev_ops->dev_infos_get(dev, dev_info);
}

static const uint32_t *
pmd_dev_supported_ptypes_get(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	return dev->dev_ops->dev_supported_ptypes_get(dev);
}

static int
pmd_rx_queue_start(struct rte_eth_dev *dev, uint16_t queue_id)
{
	dev = UDEV(dev);

	return dev->dev_ops->rx_queue_start(dev, queue_id);
}

static int
pmd_rx_queue_stop(struct rte_eth_dev *dev, uint16_t queue_id)
{
	dev = UDEV(dev);

	return dev->dev_ops->rx_queue_stop(dev, queue_id);
}

static int
pmd_tx_queue_start(struct rte_eth_dev *dev, uint16_t queue_id)
{
	dev = UDEV(dev);

	return dev->dev_ops->tx_queue_start(dev, queue_id);
}

static int
pmd_tx_queue_stop(struct rte_eth_dev *dev, uint16_t queue_id)
{
	dev = UDEV(dev);

	return dev->dev_ops->tx_queue_stop(dev, queue_id);
}

static int
pmd_rx_queue_setup(struct rte_eth_dev *dev,
	uint16_t rx_queue_id, uint16_t nb_rx_desc, unsigned int socket_id,
	const struct rte_eth_rxconf *rx_conf, struct rte_mempool *mb_pool)
{
	dev = UDEV(dev);

	return dev->dev_ops->rx_queue_setup(dev, rx_queue_id, nb_rx_desc,
		socket_id, rx_conf, mb_pool);
}

static int
pmd_tx_queue_setup(struct rte_eth_dev *dev,
	uint16_t tx_queue_id, uint16_t nb_tx_desc, unsigned int socket_id,
	const struct rte_eth_txconf *tx_conf)
{
	dev = UDEV(dev);

	return dev->dev_ops->tx_queue_setup(dev, tx_queue_id, nb_tx_desc,
		socket_id, tx_conf);
}

static int
pmd_rx_queue_intr_enable(struct rte_eth_dev *dev, uint16_t rx_queue_id)
{
	dev = UDEV(dev);

	return dev->dev_ops->rx_queue_intr_enable(dev, rx_queue_id);
}

static int
pmd_rx_queue_intr_disable(struct rte_eth_dev *dev, uint16_t rx_queue_id)
{
	dev = UDEV(dev);

	return dev->dev_ops->rx_queue_intr_disable(dev, rx_queue_id);
}

static uint32_t
pmd_rx_queue_count(struct rte_eth_dev *dev, uint16_t rx_queue_id)
{
	dev = UDEV(dev);

	return dev->dev_ops->rx_queue_count(dev, rx_queue_id);
}

static int
pmd_fw_version_get(struct rte_eth_dev *dev,
	char *fw_version, size_t fw_size)
{
	dev = UDEV(dev);

	return dev->dev_ops->fw_version_get(dev, fw_version, fw_size);
}

static void
pmd_rxq_info_get(struct rte_eth_dev *dev,
	uint16_t rx_queue_id, struct rte_eth_rxq_info *qinfo)
{
	dev = UDEV(dev);

	dev->dev_ops->rxq_info_get(dev, rx_queue_id, qinfo);
}

static void
pmd_txq_info_get(struct rte_eth_dev *dev,
	uint16_t tx_queue_id, struct rte_eth_txq_info *qinfo)
{
	dev = UDEV(dev);

	dev->dev_ops->txq_info_get(dev, tx_queue_id, qinfo);
}

static int
pmd_mtu_set(struct rte_eth_dev *dev, uint16_t mtu)
{
	dev = UDEV(dev);

	return dev->dev_ops->mtu_set(dev, mtu);
}

static int
pmd_vlan_filter_set(struct rte_eth_dev *dev, uint16_t vlan_id, int on)
{
	dev = UDEV(dev);

	return dev->dev_ops->vlan_filter_set(dev, vlan_id, on);
}

static int
pmd_vlan_tpid_set(struct rte_eth_dev *dev,
	enum rte_vlan_type type, uint16_t tpid)
{
	dev = UDEV(dev);

	return dev->dev_ops->vlan_tpid_set(dev, type, tpid);
}

static void
pmd_vlan_offload_set(struct rte_eth_dev *dev, int mask)
{
	dev = UDEV(dev);

	dev->dev_ops->vlan_offload_set(dev, mask);
}

static int
pmd_vlan_pvid_set(struct rte_eth_dev *dev, uint16_t vlan_id, int on)
{
	dev = UDEV(dev);

	return dev->dev_ops->vlan_pvid_set(dev, vlan_id, on);
}

static void
pmd_vlan_strip_queue_set(struct rte_eth_dev *dev,
	uint16_t rx_queue_id, int on)
{
	dev = UDEV(dev);

	dev->dev_ops->vlan_strip_queue_set(dev, rx_queue_id, on);
}

static int
pmd_flow_ctrl_get(struct rte_eth_dev *dev, struct rte_eth_fc_conf *fc_conf)
{
	dev = UDEV(dev);

	return dev->dev_ops->flow_ctrl_get(dev, fc_conf);
}

static int
pmd_flow_ctrl_set(struct rte_eth_dev *dev, struct rte_eth_fc_conf *fc_conf)
{
	dev = UDEV(dev);

	return dev->dev_ops->flow_ctrl_set(dev, fc_conf);
}

static int
pmd_priority_flow_ctrl_set(struct rte_eth_dev *dev,
	struct rte_eth_pfc_conf *pfc_conf)
{
	dev = UDEV(dev);

	return dev->dev_ops->priority_flow_ctrl_set(dev, pfc_conf);
}

static int
pmd_reta_update(struct rte_eth_dev *dev,
	struct rte_eth_rss_reta_entry64 *reta_conf, uint16_t reta_size)
{
	dev = UDEV(dev);

	return dev->dev_ops->reta_update(dev, reta_conf, reta_size);
}

static int
pmd_reta_query(struct rte_eth_dev *dev,
	struct rte_eth_rss_reta_entry64 *reta_conf, uint16_t reta_size)
{
	dev = UDEV(dev);

	return dev->dev_ops->reta_query(dev, reta_conf, reta_size);
}

static int
pmd_rss_hash_update(struct rte_eth_dev *dev,
	struct rte_eth_rss_conf *rss_conf)
{
	dev = UDEV(dev);

	return dev->dev_ops->rss_hash_update(dev, rss_conf);
}

static int
pmd_rss_hash_conf_get(struct rte_eth_dev *dev,
	struct rte_eth_rss_conf *rss_conf)
{
	dev = UDEV(dev);

	return dev->dev_ops->rss_hash_conf_get(dev, rss_conf);
}

static int
pmd_dev_led_on(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	return dev->dev_ops->dev_led_on(dev);
}

static int
pmd_dev_led_off(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	return dev->dev_ops->dev_led_off(dev);
}

static void
pmd_mac_addr_remove(struct rte_eth_dev *dev, uint32_t index)
{
	dev = UDEV(dev);

	dev->dev_ops->mac_addr_remove(dev, index);
}

static int
pmd_mac_addr_add(struct rte_eth_dev *dev,
	struct ether_addr *mac_addr, uint32_t index, uint32_t vmdq)
{
	dev = UDEV(dev);

	return dev->dev_ops->mac_addr_add(dev, mac_addr, index, vmdq);
}

static void
pmd_mac_addr_set(struct rte_eth_dev *dev, struct ether_addr *mac_addr)
{
	dev = UDEV(dev);

	dev->dev_ops->mac_addr_set(dev, mac_addr);
}

static int
pmd_uc_hash_table_set(struct rte_eth_dev *dev,
	struct ether_addr *mac_addr, uint8_t on)
{
	dev = UDEV(dev);

	return dev->dev_ops->uc_hash_table_set(dev, mac_addr, on);
}

static int
pmd_uc_all_hash_table_set(struct rte_eth_dev *dev, uint8_t on)
{
	dev = UDEV(dev);

	return dev->dev_ops->uc_all_hash_table_set(dev, on);
}

static int
pmd_set_queue_rate_limit(struct rte_eth_dev *dev,
	uint16_t queue_idx, uint16_t tx_rate)
{
	dev = UDEV(dev);

	return dev->dev_ops->set_queue_rate_limit(dev, queue_idx, tx_rate);
}

static int
pmd_mirror_rule_set(struct rte_eth_dev *dev,
	struct rte_eth_mirror_conf *mirror_conf, uint8_t rule_id, uint8_t on)
{
	dev = UDEV(dev);

	return dev->dev_ops->mirror_rule_set(dev, mirror_conf, rule_id, on);
}

static int
pmd_mirror_rule_reset(struct rte_eth_dev *dev, uint8_t rule_id)
{
	dev = UDEV(dev);

	return dev->dev_ops->mirror_rule_reset(dev, rule_id);
}

static int
pmd_udp_tunnel_port_add(struct rte_eth_dev *dev,
	struct rte_eth_udp_tunnel *tunnel_udp)
{
	dev = UDEV(dev);

	return dev->dev_ops->udp_tunnel_port_add(dev, tunnel_udp);
}

static int
pmd_udp_tunnel_port_del(struct rte_eth_dev *dev,
	struct rte_eth_udp_tunnel *tunnel_udp)
{
	dev = UDEV(dev);

	return dev->dev_ops->udp_tunnel_port_del(dev, tunnel_udp);
}

static int
pmd_set_mc_addr_list(struct rte_eth_dev *dev,
	struct ether_addr *mc_addr_set, uint32_t nb_mc_addr)
{
	dev = UDEV(dev);

	return dev->dev_ops->set_mc_addr_list(dev, mc_addr_set, nb_mc_addr);
}

static int
pmd_timesync_enable(struct rte_eth_dev *dev)
{
	return dev->dev_ops->timesync_enable(dev);
}

static int
pmd_timesync_disable(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	return dev->dev_ops->timesync_disable(dev);
}

static int
pmd_timesync_read_rx_timestamp(struct rte_eth_dev *dev,
               struct timespec *timestamp, uint32_t flags)
{
	dev = UDEV(dev);

	return dev->dev_ops->timesync_read_rx_timestamp(dev, timestamp, flags);
}

static int
pmd_timesync_read_tx_timestamp(struct rte_eth_dev *dev,
	struct timespec *timestamp)
{
	dev = UDEV(dev);

	return dev->dev_ops->timesync_read_tx_timestamp(dev, timestamp);
}

static int
pmd_timesync_adjust_time(struct rte_eth_dev *dev, int64_t delta)
{
	dev = UDEV(dev);

	return dev->dev_ops->timesync_adjust_time(dev, delta);
}

static int
pmd_timesync_read_time(struct rte_eth_dev *dev, struct timespec *timestamp)
{
	dev = UDEV(dev);

	return dev->dev_ops->timesync_read_time(dev, timestamp);
}

static int
pmd_timesync_write_time(struct rte_eth_dev *dev,
	const struct timespec *timestamp)
{
	dev = UDEV(dev);

	return dev->dev_ops->timesync_write_time(dev, timestamp);
}

static int
pmd_get_reg(struct rte_eth_dev *dev, struct rte_dev_reg_info *info)
{
	dev = UDEV(dev);

	return dev->dev_ops->get_reg(dev, info);
}

static int
pmd_get_eeprom_length(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	return dev->dev_ops->get_eeprom_length(dev);
}

static int
pmd_get_eeprom(struct rte_eth_dev *dev, struct rte_dev_eeprom_info *info)
{
	dev = UDEV(dev);

	return dev->dev_ops->get_eeprom(dev, info);
}

static int
pmd_set_eeprom(struct rte_eth_dev *dev, struct rte_dev_eeprom_info *info)
{
	dev = UDEV(dev);

	return dev->dev_ops->set_eeprom(dev, info);
}

static int
pmd_l2_tunnel_eth_type_conf(struct rte_eth_dev *dev,
               struct rte_eth_l2_tunnel_conf *l2_tunnel)
{
	dev = UDEV(dev);

	return dev->dev_ops->l2_tunnel_eth_type_conf(dev, l2_tunnel);
}

static int
pmd_l2_tunnel_offload_set(struct rte_eth_dev *dev,
               struct rte_eth_l2_tunnel_conf *l2_tunnel,
			   uint32_t mask,
			   uint8_t en)
{
	dev = UDEV(dev);

	return dev->dev_ops->l2_tunnel_offload_set(dev,l2_tunnel, mask, en);
}

#ifdef RTE_NIC_BYPASS

static void
pmd_bypass_init(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	dev->dev_ops->bypass_init(dev);
}

static int32_t
pmd_bypass_state_set(struct rte_eth_dev *dev, uint32_t *new_state)
{
	dev = UDEV(dev);

	return dev->dev_ops->bypass_state_set(dev, new_state);
}

static int32_t
pmd_bypass_state_show(struct rte_eth_dev *dev, uint32_t *state)
{
	dev = UDEV(dev);

	return dev->dev_ops->bypass_state_show(dev, state);
}

static int32_t
pmd_bypass_event_set(struct rte_eth_dev *dev,
	uint32_t state, uint32_t event)
{
	dev = UDEV(dev);

	return dev->dev_ops->bypass_event_set(dev, state, event);
}

static int32_t
pmd_bypass_event_show(struct rte_eth_dev *dev,
	uint32_t event_shift, uint32_t *event)
{
	dev = UDEV(dev);

	return dev->dev_ops->bypass_event_show(dev, event_shift, event);
}

static int32_t
pmd_bypass_wd_timeout_set(struct rte_eth_dev *dev, uint32_t timeout)
{
	dev = UDEV(dev);

	return dev->dev_ops->bypass_wd_timeout_set(dev, timeout);
}

static int32_t
pmd_bypass_wd_timeout_show(struct rte_eth_dev *dev, uint32_t *wd_timeout)
{
	dev = UDEV(dev);

	return dev->dev_ops->bypass_wd_timeout_show(dev, wd_timeout);
}

static int32_t
pmd_bypass_ver_show(struct rte_eth_dev *dev, uint32_t *ver)
{
	dev = UDEV(dev);

	return dev->dev_ops->bypass_ver_show(dev, ver);
}

static int32_t
pmd_bypass_wd_reset(struct rte_eth_dev *dev)
{
	dev = UDEV(dev);

	return dev->dev_ops->bypass_wd_reset(dev);
}

#endif

static int
pmd_filter_ctrl(struct rte_eth_dev *dev, enum rte_filter_type filter_type,
	enum rte_filter_op filter_op, void *arg)
{
	dev = UDEV(dev);

	return dev->dev_ops->filter_ctrl(dev, filter_type, filter_op, arg);
}

static int
pmd_get_dcb_info(struct rte_eth_dev *dev,
	struct rte_eth_dcb_info *dcb_info)
{
	dev = UDEV(dev);

	return dev->dev_ops->get_dcb_info(dev, dcb_info);
}

static int
pmd_tm_ops_get(struct rte_eth_dev *dev, void *ops)
{
	dev = UDEV(dev);

	return dev->dev_ops->tm_ops_get(dev, ops);
}

static const struct eth_dev_ops pmd_ops_default = {
	.dev_configure = pmd_dev_configure,
	.dev_start = pmd_dev_start,
	.dev_stop = pmd_dev_stop,
	.dev_set_link_up = pmd_dev_set_link_up,
	.dev_set_link_down = pmd_dev_set_link_down,
	.dev_close = pmd_dev_close,
	.link_update = pmd_link_update,

	.promiscuous_enable = pmd_promiscuous_enable,
	.promiscuous_disable = pmd_promiscuous_disable,
	.allmulticast_enable = pmd_allmulticast_enable,
	.allmulticast_disable = pmd_allmulticast_disable,
	.mac_addr_remove = pmd_mac_addr_remove,
	.mac_addr_add = pmd_mac_addr_add,
	.mac_addr_set = pmd_mac_addr_set,
	.set_mc_addr_list = pmd_set_mc_addr_list,
	.mtu_set = pmd_mtu_set,

	.stats_get = pmd_stats_get,
	.stats_reset = pmd_stats_reset,
	.xstats_get = pmd_xstats_get,
	.xstats_reset = pmd_xstats_reset,
	.xstats_get_names = pmd_xstats_get_names,
	.queue_stats_mapping_set = pmd_queue_stats_mapping_set,

	.dev_infos_get = pmd_dev_infos_get,
	.rxq_info_get = pmd_rxq_info_get,
	.txq_info_get = pmd_txq_info_get,
	.fw_version_get = pmd_fw_version_get,
	.dev_supported_ptypes_get = pmd_dev_supported_ptypes_get,

	.vlan_filter_set = pmd_vlan_filter_set,
	.vlan_tpid_set = pmd_vlan_tpid_set,
	.vlan_strip_queue_set = pmd_vlan_strip_queue_set,
	.vlan_offload_set = pmd_vlan_offload_set,
	.vlan_pvid_set = pmd_vlan_pvid_set,

	.rx_queue_start = pmd_rx_queue_start,
	.rx_queue_stop = pmd_rx_queue_stop,
	.tx_queue_start = pmd_tx_queue_start,
	.tx_queue_stop = pmd_tx_queue_stop,
	.rx_queue_setup = pmd_rx_queue_setup,
	.rx_queue_release = NULL,
	.rx_queue_count = pmd_rx_queue_count,
	.rx_descriptor_done = NULL,
	.rx_descriptor_status = NULL,
	.tx_descriptor_status = NULL,
	.rx_queue_intr_enable = pmd_rx_queue_intr_enable,
	.rx_queue_intr_disable = pmd_rx_queue_intr_disable,
	.tx_queue_setup = pmd_tx_queue_setup,
	.tx_queue_release = NULL,
	.tx_done_cleanup = NULL,

	.dev_led_on = pmd_dev_led_on,
	.dev_led_off = pmd_dev_led_off,

	.flow_ctrl_get = pmd_flow_ctrl_get,
	.flow_ctrl_set = pmd_flow_ctrl_set,
	.priority_flow_ctrl_set = pmd_priority_flow_ctrl_set,

	.uc_hash_table_set = pmd_uc_hash_table_set,
	.uc_all_hash_table_set = pmd_uc_all_hash_table_set,

	.mirror_rule_set = pmd_mirror_rule_set,
	.mirror_rule_reset = pmd_mirror_rule_reset,

	.udp_tunnel_port_add = pmd_udp_tunnel_port_add,
	.udp_tunnel_port_del = pmd_udp_tunnel_port_del,
	.l2_tunnel_eth_type_conf = pmd_l2_tunnel_eth_type_conf,
	.l2_tunnel_offload_set = pmd_l2_tunnel_offload_set,

	.set_queue_rate_limit = pmd_set_queue_rate_limit,

	.rss_hash_update = pmd_rss_hash_update,
	.rss_hash_conf_get = pmd_rss_hash_conf_get,
	.reta_update = pmd_reta_update,
	.reta_query = pmd_reta_query,

	.get_reg = pmd_get_reg,
	.get_eeprom_length = pmd_get_eeprom_length,
	.get_eeprom = pmd_get_eeprom,
	.set_eeprom = pmd_set_eeprom,

#ifdef RTE_NIC_BYPASS
	.bypass_init = pmd_bypass_init,
	.bypass_state_set = pmd_bypass_state_set,
	.bypass_state_show = pmd_bypass_state_show,
	.bypass_event_set = pmd_bypass_event_set,
	.bypass_event_show = pmd_bypass_event_show,
	.bypass_wd_timeout_set = pmd_bypass_wd_timeout_set,
	.bypass_wd_timeout_show = pmd_bypass_wd_timeout_show,
	.bypass_ver_show = pmd_bypass_ver_show,
	.bypass_wd_reset = pmd_bypass_wd_reset,
#endif

	.filter_ctrl = pmd_filter_ctrl,

	.get_dcb_info = pmd_get_dcb_info,

	.timesync_enable = pmd_timesync_enable,
	.timesync_disable = pmd_timesync_disable,
	.timesync_read_rx_timestamp = pmd_timesync_read_rx_timestamp,
	.timesync_read_tx_timestamp = pmd_timesync_read_tx_timestamp,
	.timesync_adjust_time = pmd_timesync_adjust_time,
	.timesync_read_time = pmd_timesync_read_time,
	.timesync_write_time = pmd_timesync_write_time,

	.xstats_get_by_id = pmd_xstats_get_by_id,
	.xstats_get_names_by_id = pmd_xstats_get_names_by_id,

	.tm_ops_get = pmd_tm_ops_get,
};

#define CHECK_AND_SET_NULL(o, u, func)			\
	if ((u)->func == NULL)				\
		(o)->func = NULL

#define CHECK_AND_SET_NONNULL(o, u, func)			\
	if ((u)->func != NULL)				\
		(o)->func = (u)->func

static void
pmd_ops_check_and_set_null(struct eth_dev_ops *o,
	const struct eth_dev_ops *u)
{
	CHECK_AND_SET_NULL(o, u, dev_configure);
	CHECK_AND_SET_NULL(o, u, dev_start);
	CHECK_AND_SET_NULL(o, u, dev_stop);
	CHECK_AND_SET_NULL(o, u, dev_set_link_up);
	CHECK_AND_SET_NULL(o, u, dev_set_link_down);
	CHECK_AND_SET_NULL(o, u, dev_close);
	CHECK_AND_SET_NULL(o, u, link_update);
	CHECK_AND_SET_NULL(o, u, promiscuous_enable);
	CHECK_AND_SET_NULL(o, u, promiscuous_disable);
	CHECK_AND_SET_NULL(o, u, allmulticast_enable);
	CHECK_AND_SET_NULL(o, u, allmulticast_disable);
	CHECK_AND_SET_NULL(o, u, mac_addr_remove);
	CHECK_AND_SET_NULL(o, u, mac_addr_add);
	CHECK_AND_SET_NULL(o, u, mac_addr_set);
	CHECK_AND_SET_NULL(o, u, set_mc_addr_list);
	CHECK_AND_SET_NULL(o, u, mtu_set);
	CHECK_AND_SET_NULL(o, u, stats_get);
	CHECK_AND_SET_NULL(o, u, stats_reset);
	CHECK_AND_SET_NULL(o, u, xstats_get);
	CHECK_AND_SET_NULL(o, u, xstats_reset);
	CHECK_AND_SET_NULL(o, u, xstats_get_names);
	CHECK_AND_SET_NULL(o, u, queue_stats_mapping_set);
	CHECK_AND_SET_NULL(o, u, dev_infos_get);
	CHECK_AND_SET_NULL(o, u, rxq_info_get);
	CHECK_AND_SET_NULL(o, u, txq_info_get);
	CHECK_AND_SET_NULL(o, u, fw_version_get);
	CHECK_AND_SET_NULL(o, u, dev_supported_ptypes_get);
	CHECK_AND_SET_NULL(o, u, vlan_filter_set);
	CHECK_AND_SET_NULL(o, u, vlan_tpid_set);
	CHECK_AND_SET_NULL(o, u, vlan_strip_queue_set);
	CHECK_AND_SET_NULL(o, u, vlan_offload_set);
	CHECK_AND_SET_NULL(o, u, vlan_pvid_set);
	CHECK_AND_SET_NULL(o, u, rx_queue_start);
	CHECK_AND_SET_NULL(o, u, rx_queue_stop);
	CHECK_AND_SET_NULL(o, u, tx_queue_start);
	CHECK_AND_SET_NULL(o, u, tx_queue_stop);
	CHECK_AND_SET_NULL(o, u, rx_queue_setup);
	CHECK_AND_SET_NULL(o, u, rx_queue_release);
	CHECK_AND_SET_NULL(o, u, rx_queue_count);
	CHECK_AND_SET_NULL(o, u, rx_descriptor_done);
	CHECK_AND_SET_NULL(o, u, rx_descriptor_status);
	CHECK_AND_SET_NULL(o, u, tx_descriptor_status);
	CHECK_AND_SET_NULL(o, u, rx_queue_intr_enable);
	CHECK_AND_SET_NULL(o, u, rx_queue_intr_disable);
	CHECK_AND_SET_NULL(o, u, tx_queue_setup);
	CHECK_AND_SET_NULL(o, u, tx_queue_release);
	CHECK_AND_SET_NULL(o, u, tx_done_cleanup);
	CHECK_AND_SET_NULL(o, u, dev_led_on);
	CHECK_AND_SET_NULL(o, u, dev_led_off);
	CHECK_AND_SET_NULL(o, u, flow_ctrl_get);
	CHECK_AND_SET_NULL(o, u, flow_ctrl_set);
	CHECK_AND_SET_NULL(o, u, priority_flow_ctrl_set);
	CHECK_AND_SET_NULL(o, u, uc_hash_table_set);
	CHECK_AND_SET_NULL(o, u, uc_all_hash_table_set);
	CHECK_AND_SET_NULL(o, u, mirror_rule_set);
	CHECK_AND_SET_NULL(o, u, mirror_rule_reset);
	CHECK_AND_SET_NULL(o, u, udp_tunnel_port_add);
	CHECK_AND_SET_NULL(o, u, udp_tunnel_port_del);
	CHECK_AND_SET_NULL(o, u, l2_tunnel_eth_type_conf);
	CHECK_AND_SET_NULL(o, u, l2_tunnel_offload_set);
	CHECK_AND_SET_NULL(o, u, set_queue_rate_limit);
	CHECK_AND_SET_NULL(o, u, rss_hash_update);
	CHECK_AND_SET_NULL(o, u, rss_hash_conf_get);
	CHECK_AND_SET_NULL(o, u, reta_update);
	CHECK_AND_SET_NULL(o, u, reta_query);
	CHECK_AND_SET_NULL(o, u, get_reg);
	CHECK_AND_SET_NULL(o, u, get_eeprom_length);
	CHECK_AND_SET_NULL(o, u, get_eeprom);
	CHECK_AND_SET_NULL(o, u, set_eeprom);

	#ifdef RTE_NIC_BYPASS

	CHECK_AND_SET_NULL(o, u, bypass_init);
	CHECK_AND_SET_NULL(o, u, bypass_state_set);
	CHECK_AND_SET_NULL(o, u, bypass_state_show);
	CHECK_AND_SET_NULL(o, u, bypass_event_set);
	CHECK_AND_SET_NULL(o, u, bypass_event_show);
	CHECK_AND_SET_NULL(o, u, bypass_wd_timeout_set);
	CHECK_AND_SET_NULL(o, u, bypass_wd_timeout_show);
	CHECK_AND_SET_NULL(o, u, bypass_ver_show);
	CHECK_AND_SET_NULL(o, u, bypass_wd_reset);

	#endif

	CHECK_AND_SET_NULL(o, u, filter_ctrl);
	CHECK_AND_SET_NULL(o, u, get_dcb_info);
	CHECK_AND_SET_NULL(o, u, timesync_enable);
	CHECK_AND_SET_NULL(o, u, timesync_disable);
	CHECK_AND_SET_NULL(o, u, timesync_read_rx_timestamp);
	CHECK_AND_SET_NULL(o, u, timesync_read_tx_timestamp);
	CHECK_AND_SET_NULL(o, u, timesync_adjust_time);
	CHECK_AND_SET_NULL(o, u, timesync_read_time);
	CHECK_AND_SET_NULL(o, u, timesync_write_time);
	CHECK_AND_SET_NULL(o, u, xstats_get_by_id);
	CHECK_AND_SET_NULL(o, u, xstats_get_names_by_id);
	CHECK_AND_SET_NULL(o, u, tm_ops_get);
}

void
pmd_ops_inherit(struct eth_dev_ops *o, const struct eth_dev_ops *u)
{
	/* Rules:
	 *    1. u->func == NULL => o->func = NULL
	 *    2. u->func != NULL => o->func = pmd_ops_default.func
	 *    3. queue related func => o->func = u->func
	 */

	memcpy(o, &pmd_ops_default, sizeof(struct eth_dev_ops));
	pmd_ops_check_and_set_null(o, u);

	/* Copy queue related functions */
	o->rx_queue_release = u->rx_queue_release;
	o->tx_queue_release = u->tx_queue_release;
	o->rx_descriptor_done = u->rx_descriptor_done;
	o->rx_descriptor_status = u->rx_descriptor_status;
	o->tx_descriptor_status = u->tx_descriptor_status;
	o->tx_done_cleanup = u->tx_done_cleanup;
}

void
pmd_ops_derive(struct eth_dev_ops *o, const struct eth_dev_ops *u)
{
	CHECK_AND_SET_NONNULL(o, u, dev_configure);
	CHECK_AND_SET_NONNULL(o, u, dev_start);
	CHECK_AND_SET_NONNULL(o, u, dev_stop);
	CHECK_AND_SET_NONNULL(o, u, dev_set_link_up);
	CHECK_AND_SET_NONNULL(o, u, dev_set_link_down);
	CHECK_AND_SET_NONNULL(o, u, dev_close);
	CHECK_AND_SET_NONNULL(o, u, link_update);
	CHECK_AND_SET_NONNULL(o, u, promiscuous_enable);
	CHECK_AND_SET_NONNULL(o, u, promiscuous_disable);
	CHECK_AND_SET_NONNULL(o, u, allmulticast_enable);
	CHECK_AND_SET_NONNULL(o, u, allmulticast_disable);
	CHECK_AND_SET_NONNULL(o, u, mac_addr_remove);
	CHECK_AND_SET_NONNULL(o, u, mac_addr_add);
	CHECK_AND_SET_NONNULL(o, u, mac_addr_set);
	CHECK_AND_SET_NONNULL(o, u, set_mc_addr_list);
	CHECK_AND_SET_NONNULL(o, u, mtu_set);
	CHECK_AND_SET_NONNULL(o, u, stats_get);
	CHECK_AND_SET_NONNULL(o, u, stats_reset);
	CHECK_AND_SET_NONNULL(o, u, xstats_get);
	CHECK_AND_SET_NONNULL(o, u, xstats_reset);
	CHECK_AND_SET_NONNULL(o, u, xstats_get_names);
	CHECK_AND_SET_NONNULL(o, u, queue_stats_mapping_set);
	CHECK_AND_SET_NONNULL(o, u, dev_infos_get);
	CHECK_AND_SET_NONNULL(o, u, rxq_info_get);
	CHECK_AND_SET_NONNULL(o, u, txq_info_get);
	CHECK_AND_SET_NONNULL(o, u, fw_version_get);
	CHECK_AND_SET_NONNULL(o, u, dev_supported_ptypes_get);
	CHECK_AND_SET_NONNULL(o, u, vlan_filter_set);
	CHECK_AND_SET_NONNULL(o, u, vlan_tpid_set);
	CHECK_AND_SET_NONNULL(o, u, vlan_strip_queue_set);
	CHECK_AND_SET_NONNULL(o, u, vlan_offload_set);
	CHECK_AND_SET_NONNULL(o, u, vlan_pvid_set);
	CHECK_AND_SET_NONNULL(o, u, rx_queue_start);
	CHECK_AND_SET_NONNULL(o, u, rx_queue_stop);
	CHECK_AND_SET_NONNULL(o, u, tx_queue_start);
	CHECK_AND_SET_NONNULL(o, u, tx_queue_stop);
	CHECK_AND_SET_NONNULL(o, u, rx_queue_setup);
	CHECK_AND_SET_NONNULL(o, u, rx_queue_release);
	CHECK_AND_SET_NONNULL(o, u, rx_queue_count);
	CHECK_AND_SET_NONNULL(o, u, rx_descriptor_done);
	CHECK_AND_SET_NONNULL(o, u, rx_descriptor_status);
	CHECK_AND_SET_NONNULL(o, u, tx_descriptor_status);
	CHECK_AND_SET_NONNULL(o, u, rx_queue_intr_enable);
	CHECK_AND_SET_NONNULL(o, u, rx_queue_intr_disable);
	CHECK_AND_SET_NONNULL(o, u, tx_queue_setup);
	CHECK_AND_SET_NONNULL(o, u, tx_queue_release);
	CHECK_AND_SET_NONNULL(o, u, tx_done_cleanup);
	CHECK_AND_SET_NONNULL(o, u, dev_led_on);
	CHECK_AND_SET_NONNULL(o, u, dev_led_off);
	CHECK_AND_SET_NONNULL(o, u, flow_ctrl_get);
	CHECK_AND_SET_NONNULL(o, u, flow_ctrl_set);
	CHECK_AND_SET_NONNULL(o, u, priority_flow_ctrl_set);
	CHECK_AND_SET_NONNULL(o, u, uc_hash_table_set);
	CHECK_AND_SET_NONNULL(o, u, uc_all_hash_table_set);
	CHECK_AND_SET_NONNULL(o, u, mirror_rule_set);
	CHECK_AND_SET_NONNULL(o, u, mirror_rule_reset);
	CHECK_AND_SET_NONNULL(o, u, udp_tunnel_port_add);
	CHECK_AND_SET_NONNULL(o, u, udp_tunnel_port_del);
	CHECK_AND_SET_NONNULL(o, u, l2_tunnel_eth_type_conf);
	CHECK_AND_SET_NONNULL(o, u, l2_tunnel_offload_set);
	CHECK_AND_SET_NONNULL(o, u, set_queue_rate_limit);
	CHECK_AND_SET_NONNULL(o, u, rss_hash_update);
	CHECK_AND_SET_NONNULL(o, u, rss_hash_conf_get);
	CHECK_AND_SET_NONNULL(o, u, reta_update);
	CHECK_AND_SET_NONNULL(o, u, reta_query);
	CHECK_AND_SET_NONNULL(o, u, get_reg);
	CHECK_AND_SET_NONNULL(o, u, get_eeprom_length);
	CHECK_AND_SET_NONNULL(o, u, get_eeprom);
	CHECK_AND_SET_NONNULL(o, u, set_eeprom);

	#ifdef RTE_NIC_BYPASS

	CHECK_AND_SET_NONNULL(o, u, bypass_init);
	CHECK_AND_SET_NONNULL(o, u, bypass_state_set);
	CHECK_AND_SET_NONNULL(o, u, bypass_state_show);
	CHECK_AND_SET_NONNULL(o, u, bypass_event_set);
	CHECK_AND_SET_NONNULL(o, u, bypass_event_show);
	CHECK_AND_SET_NONNULL(o, u, bypass_wd_timeout_set);
	CHECK_AND_SET_NONNULL(o, u, bypass_wd_timeout_show);
	CHECK_AND_SET_NONNULL(o, u, bypass_ver_show);
	CHECK_AND_SET_NONNULL(o, u, bypass_wd_reset);

	#endif

	CHECK_AND_SET_NONNULL(o, u, filter_ctrl);
	CHECK_AND_SET_NONNULL(o, u, get_dcb_info);
	CHECK_AND_SET_NONNULL(o, u, timesync_enable);
	CHECK_AND_SET_NONNULL(o, u, timesync_disable);
	CHECK_AND_SET_NONNULL(o, u, timesync_read_rx_timestamp);
	CHECK_AND_SET_NONNULL(o, u, timesync_read_tx_timestamp);
	CHECK_AND_SET_NONNULL(o, u, timesync_adjust_time);
	CHECK_AND_SET_NONNULL(o, u, timesync_read_time);
	CHECK_AND_SET_NONNULL(o, u, timesync_write_time);
	CHECK_AND_SET_NONNULL(o, u, xstats_get_by_id);
	CHECK_AND_SET_NONNULL(o, u, xstats_get_names_by_id);
	CHECK_AND_SET_NONNULL(o, u, tm_ops_get);
}
