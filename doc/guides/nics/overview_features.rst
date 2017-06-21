..  BSD LICENSE
    Copyright(c) 2017 Intel Corporation. All rights reserved.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    * Neither the name of Intel Corporation nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

:orphan:

.. _NIC_Overview_Features:

Overview features
=================

.. _nic_features_speed_capabilities:

Speed capabilities
------------------

eth_dev_ops: dev_infos_get:speed_capa

Advertise the speed capabilities that the current device capable to offer.


.. _nic_features_link_status:

Link status
-----------
eth_dev_ops: link_update

Getting link speed, duplex mode and state (up/down) support.


.. _nic_features_link_status_event:

Link status event
-----------------
user config requires: dev_conf.intr_conf.lsc
rte_pci_driver.drv_flags = RTE_PCI_DRV_INTR_LSC

Link Status Change interrupts support.


.. _nic_features_removal_event:

Removal event
-------------
user config requires: dev_conf.intr_conf.rmv
rte_pci_driver.drv_flags = RTE_PCI_DRV_INTR_RMV

Device removal interrupts support.


.. _nic_features_queue_status_event:

Queue status event
------------------
eth_dev_ops: link_update


.. _nic_features_rx_interrupt:

Rx interrupt
------------
eth_dev_ops: rx_queue_intr_enable, rx_queue_intr_disable

Rx interrupts support.


.. _nic_features_free_tx_mbuf_on_demand:

Free Tx mbuf on demand
----------------------
eth_dev_ops: tx_done_cleanup

Forcing free consumed buffers on Tx ring support.


.. _nic_features_queue_start_stop:

Queue start/stop
----------------
eth_dev_ops: rx_queue_start, rx_queue_stop, tx_queue_start, tx_queue_stop

start/stop specified Rx/Tx queue of a port.


.. _nic_features_mtu_update:

MTU update
----------
eth_dev_ops: mtu_set

Updating port MTU support.


.. _nic_features_jumbo_frame:

Jumbo frame
-----------

Receiving jumbo frames support.


.. _nic_features_scattered_rx:

Scattered Rx
------------

Receiving segmented mbufs support.


.. _nic_features_lro:

LRO
---
Large Receive Offload.


.. _nic_features_tso:

TSO
---
TCP Segmentation Offloading.


.. _nic_features_promiscuous_mode:

Promiscuous mode
----------------
eth_dev_ops: promiscuous_enable, promiscuous_disable

enable/disable port promiscuous mode.


.. _nic_features_allmulticast_mode:

Allmulticast mode
-----------------
eth_dev_ops: allmulticast_enable, allmulticast_disable

enable/disable receiving multicast frames.


.. _nic_features_unicast_mac_filter:

Unicast MAC filter
------------------
eth_dev_ops: mac_addr_add, mac_addr_remove

Adding MAC address to enable whitelist filtering to accept packets.


.. _nic_features_multicast_mac_filter:

Multicast MAC filter
--------------------
eth_dev_ops: set_mc_addr_list

Setting the list of multicast addresses to filter.


.. _nic_features_rss_hash:

RSS hash
--------
eth_dev_ops: rss_hash_update, rss_hash_conf_get

Configuration of Receive Side Scaling (RSS) hash computation.


.. _nic_features_rss_key_update:

RSS key update
--------------
eth_dev_ops: rss_hash_update, rss_hash_conf_get

Updating Receive Side Scaling (RSS) hash key.

.. _nic_features_rss_reta_update:

RSS reta update
---------------
eth_dev_ops: reta_update, reta_query

Updating Redirection Table of the Receive Side Scaling (RSS).


.. _nic_features_vmdq:

VMDq
----

Virtual Machine Device Queues (VMDq) support.


.. _nic_features_sriov:

SR-IOV
------

Driver supports creating Virtual Functions.


.. _nic_features_dcb:

DCB
---
eth_dev_ops: get_dcb_info

Data Center Bridging (DCB) support.


.. _nic_features_vlan_filter:

VLAN filter
-----------
eth_dev_ops: vlan_filter_set

Filtering of a VLAN Tag Identifier.


.. _nic_features_ethertype_filter:

Ethertype filter
----------------
eth_dev_ops: filter_ctrl:RTE_ETH_FILTER_ETHERTYPE


.. _nic_features_ntuple_filter:

N-tuple filter
--------------
eth_dev_ops: filter_ctrl:RTE_ETH_FILTER_NTUPLE


.. _nic_features_syn_filter:

SYN filter
----------
eth_dev_ops: filter_ctrl:RTE_ETH_FILTER_SYN


.. _nic_features_tunnel_filter:

Tunnel filter
-------------
eth_dev_ops: filter_ctrl:RTE_ETH_FILTER_TUNNEL


.. _nic_features_filexible_filter:

Flexible filter
---------------
eth_dev_ops: filter_ctrl:RTE_ETH_FILTER_FLEXIBLE


.. _nic_features_hash_filter:

Hash filter
-----------
eth_dev_ops: filter_ctrl:RTE_ETH_FILTER_HASH


.. _nic_features_flow_director:

Flow director
-------------
eth_dev_ops: filter_ctrl:RTE_ETH_FILTER_FDIR


.. _nic_features_flow_control:

Flow control
------------
eth_dev_ops: flow_ctrl_get, flow_ctrl_set, priority_flow_ctrl_set

Configuring link flow control.


.. _nic_features_flow_api:

Flow API
--------
eth_dev_ops: filter_ctrl:RTE_ETH_FILTER_GENERIC
rte_flow_ops: *

Generic filtering API support.


.. _nic_features_rate_limitation:

Rate limitation
---------------
eth_dev_ops: set_queue_rate_limit

Tx rate limitation for a queue.


.. _nic_features_traffic_mirroring:

Traffic mirroring
-----------------
eth_dev_ops: mirror_rule_set, mirror_rule_reset

Adding traffic mirroring rule support.


.. _nic_features_crc_offload:

CRC offload
-----------


.. _nic_features_vlan_offload:

VLAN offload
------------
eth_dev_ops: vlan_offload_set


.. _nic_features_qinq_offload:

QinQ offload
------------


.. _nic_features_l3_checksum_offload:

L3 checksum offload
-------------------


.. _nic_features_l4_checksum_offload:

L4 checksum offload
-------------------


.. _nic_features_macsec_offload:

MACsec offload
--------------


.. _nic_features_inner_l3_checksum:

Inner L3 checksum
-----------------


.. _nic_features_inner_l4_checksum:

Inner L4 checksum
-----------------


.. _nic_features_packet_type_parsing:

Packet type parsing
-------------------
eth_dev_ops: dev_supported_ptypes_get

Packet type parsing and returns list of supported types.

.. _nic_features_timesync:

Timesync
--------
eth_dev_ops: timesync_enable, timesync_disable
eth_dev_ops: timesync_read_rx_timestamp, timesync_read_tx_timestamp
eth_dev_ops: timesync_adjust_time, timesync_read_time, timesync_write_time



.. _nic_features_rx_descriptor_status:

Rx descriptor status
--------------------
eth_dev_ops: rx_descriptor_status

Check the status of a Rx descriptor.


.. _nic_features_tx_descriptor_status:

Tx descriptor status
--------------------
eth_dev_ops: tx_descriptor_status

Check the status of a Tx descriptor.


.. _nic_features_basic_stats:

Basic stats
-----------
eth_dev_ops: stats_get, stats_reset

Basic statistics, same for all drivers.


.. _nic_features_extended_stats:

Extended stats
--------------
eth_dev_ops: xstats_get, xstats_reset, xstats_get_names
eth_dev_ops: xstats_get_by_id, xstats_get_names_by_id

Extended statistics, changes from driver to driver.


.. _nic_features_stats_per_queue:

Stats per queue
---------------
eth_dev_ops: queue_stats_mapping_set

Ability to get stats per queue.


.. _nic_features_fw_version:

FW version
----------
eth_dev_ops: fw_version_get

Get device hardware firmware information.


.. _nic_features_eeprom_dump:

EEPROM dump
-----------
eth_dev_ops: get_eeprom_length, get_eeprom, set_eeprom

Get device eeprom data.


.. _nic_features_register_dump:

Registers dump
--------------
eth_dev_ops: get_reg

Get device registers.


.. _nic_features_led:

LED
---
eth_dev_ops: dev_led_on, dev_led_off


.. _nic_features_multiprocess_aware:

Multiprocess aware
------------------

Driver can be used for primary-secondary process model.


.. _nic_features_bsd_nic_uio:

BSD nic_uio
-----------

BSD nic_uio module supported.


.. _nic_features_linux_uio:

Linux UIO
---------

Works with igb_uio kernel module.


.. _nic_features_linux_vfio:

Linux VFIO
----------

Works with vfio-pci kernel module.


.. _nic_features_other_kdrv:

Other kdrv
----------


.. _nic_features_armv7:

ARMv7
-----
defconfig_arm-armv7a-\*-\*

Support armv7 architecture.


.. _nic_features_armv8:

ARMv8
-----
defconfig_arm64-armv8a-\*-\*

Support armv8a (64bit) architecture.


.. _nic_features_power8:

Power8
------
defconfig_ppc_64-power8-\*-\*

Support PowerPC architecture.


.. _nic_features_x86-32:

x86-32
------
defconfig_x86_x32-native-\*-\*
defconfig_i686-native-\*-\*

Support 32bits x86 architecture.


.. _nic_features_x86-64:

x86-64
------
defconfig_x86_64-native-\*-\*

Support 64bits x86 architecture.


.. _nic_features_usage_doc:

Usage doc
---------
doc/guides/nics/\*.rst

Documentation describes usage.


.. _nic_features_design_doc:

Design doc
----------

doc/guides/nics/\*.rst

Documentation describes design.

.. _nic_features_perf_doc:

Perf doc
--------
http://dpdk.org/doc/perf/*

Document performance values.

.. _nic_features_other:

Other dev ops not represented by a Feature
------------------------------------------
rxq_info_get
txq_info_get
vlan_tpid_set
vlan_strip_queue_set
vlan_pvid_set
rx_queue_setup
rx_queue_release
rx_queue_count
rx_descriptor_done
tx_queue_setup
tx_queue_release
l2_tunnel_offload_set
uc_hash_table_set
uc_all_hash_table_set
udp_tunnel_port_add
udp_tunnel_port_del
l2_tunnel_eth_type_conf
l2_tunnel_offload_set
