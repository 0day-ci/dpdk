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


VF daemon (VFd) How-to Guide - EXPERIMENTAL
===========================================

VFd is a mechanism which is used to configure the features on VF. It's an
EXPERIMENTAL feature that can only be used in this scenario, DPDK PF + DPDK
VF. If the PF port is driven by the Linux kernel driver, this VFd feature is
lost.

Now VFd is only supported by ixgbe and i40e.

If the users want to configure a feature on VF, most of the time, it cannot
be configured by the VF directly. Because most of the VF features are under
the control of the PF.

Normally, to configure a feature on VF, the APP should call the API provided
by VF driver. If this feature cannot be configured by the VF directly (most
probably), the VF sends a message to PF, through the mailbox on ixgbe and i40e.
It means that wether the features can be configured or not depends on if the
appropriate mailbox messages are defined.

DPDK leverages the mailbox interface defined by the Linux kernel driver. So the
DPDK compatibility with kernel driver can be guaranteed. But the shortcoming is
that the messages become a limitation when the users want more features on VF.

VFd is a new way to control the features on VF. By this way, the VF driver
doesn't talk directly to the PF driver when configuring a feature on VF.
When a VF APP (Means the application using the VF ports.) want to enable a
VF feature, it can send a message to the PF APP (Means the application using
the PF port. It can be the same applcation with the VF APP.). The PF APP will
configure the feature for the VF. Obviously, the PF APP also can configure
the VF features without the request from the VF APP.

.. _VF_daemon_overview:

.. figure:: img/vf_daemon_overview.*

   VF daemon Overview

Comparing with the traditional way, VFd move the negotiation between VF and
PF from the driver level to APP level. So APP should define how to negotiation
between VF and PF. Or even not let the VFs have the capability to configure
the features, all the configuration can be done on PF.

It's the APP's responsibility to use VFd. Considering the KVM migration, VF APP
may transfer from one VM to another. It's recommanded to let the PF control the
VF features without VFs' participation. Then VF APP has no capability to
configure the features. So the users need not define the interface between the
VF APP and the PF APP. It's hard to let the APPs follow a interface
specification. The service provider should take the control of all the features.

Although VFd is supported by both ixgbe and i40e. Please aware as the HW
capability is different, the functions supported by ixgbe and i40e are not the
same. Below chapters will show the VFd functions.


Preparing
---------

VFd only can be used in the scenario, DPDK PF + DPDK VF. The users should bind
the PF port to igb_uio, then create the VFs based on the DPDK PF host.

The typical procedure to achieve this is as follows:

#. Boot the system without iommu, or with ``iommu=pt``.

#. Bind the PF port to igb_uio:

   .. code-block:: console

       dpdk-devbind.py -b igb_uio 01:00.0

#. Create a Virtual Function:

   .. code-block:: console

       echo 1 > /sys/bus/pci/devices/0000:01:00.0/max_vfs

#. Start a VM with the new VF port bypassed to it.

#. Run a DPDK application on the PF in the host:

   .. code-block:: console

       testpmd -l 0-7 -n 4 -- -i --txqflags=0

#. Bind the VF port to igb_uio in the VM:

   .. code-block:: console

       dpdk-devbind.py -b igb_uio 03:00.0

#. Run a DPDK application on the VF in the VM:

   .. code-block:: console

       testpmd -l 0-7 -n 4 -- -i --txqflags=0


Common functions on IXGBE and I40E
----------------------------------


TX loopback
~~~~~~~~~~~

   Run a CLI on the PF to set TX loopback:

   .. code-block:: console

       set tx loopback 0 on|off

   Set if the PF port and all the VF ports belongs to it are allowed to send the
   packets to other virtual ports.

   Although it's a VFd functioon, it's the global setting for the whole physical
   port. When using this function, the PF and all the VFs' TX loopback will be
   enabled/disabled.


VF MAC address setting
~~~~~~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to set the MAC address for a VF port:

   .. code-block:: console

       set vf mac addr 0 0 A0:36:9F:7B:C3:51

   This CLI will change the MAC address of the VF port to this new address.
   If any other addresses are set before, they will be removed.


VF MAC anti-spoofing
~~~~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to enable/disable the MAC anti-spoofing for a VF port:

   .. code-block:: console

       set vf mac antispoof 0 0 on|off

   When enabling the MAC anti-spoofing, the port will not sent the packets
   which's source MAC address is not this port's own.


VF VLAN anti-spoofing
~~~~~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to enable/disable the VLAN anti-spoofing for a VF port:

   .. code-block:: console

       set vf vlan antispoof 0 0 on|off

   When enabling the VLAN anti-spoofing, the port will not sent the packets
   which's VLAN ID does not belong to VLAN IDs that this port can receive.


VF VLAN insertion
~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to set the VLAN insertion for a VF port:

   .. code-block:: console

       set vf vlan insert 0 0 1

   When using this CLI, an assigned VLAN ID can be inserted to the transmitted
   packets by the HW.

   The assigned VLAN ID can be 0. It means disabling the VLAN insertion.


VF VLAN stripping
~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to enable/disable the VLAN stripping for a VF port:

   .. code-block:: console

       set vf vlan stripq 0 0 on|off

   This CLI is used to enable/disable the RX VLAN stripping for a specific
   VF port.


VF VLAN filtering
~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to set the VLAN filtering for a VF port:

   .. code-block:: console

       rx_vlan add 1 port 0 vf 1
       rx_vlan rm 1 port 0 vf 1

   There're 2 CLIs to add and remove the VLAN filter for several VF ports.
   When the VLAN filters is added, only the packets have the assigned VLAN
   IDs can be received. Other packets will be dropped by HW.


The IXGBE specific VFd functions
--------------------------------


All queues drop
~~~~~~~~~~~~~~~

   Run a CLI on the PF to enable/disable the all queues drop:

   .. code-block:: console

       set all queues drop on|off

   It's a global setting for the PF and all the VF ports of the physical
   port.

   Enabling the all queues drop means when there's no available descriptor for
   the received packets, drop them directly. The all queues drop should be
   enabled in SRIOV mode to avoid one queue blocking others.


VF packet drop
~~~~~~~~~~~~~~

   Run a CLI on the PF to enable/disable the packet drop for a specific VF:

   .. code-block:: console

       set vf split drop 0 0 on|off

   It's a similar function as the all queues drop. The difference is this
   function is per VF setting and the previous function is a global setting.


VF rate limit
~~~~~~~~~~~~~

   Run a CLI on the PF to all queues' rate limit for a specific VF:

   .. code-block:: console

       set port 0 vf 0 rate 10 queue_mask 1

   It's a function to set the rate limit for all the queues in the queue_mask
   bitmap. It's not used to set the summary of the rate limit. The rate limit
   of every queue will be set equally to the assigned rate limit.


VF rate limit
~~~~~~~~~~~~~

   Run a CLI on the PF to all queues' rate limit for a specific VF:

   .. code-block:: console

       set port 0 vf 0 rate 10 queue_mask 1

   It's a function to set the rate limit for all the queues in the queue_mask
   bitmap. It's not used to set the summary of the rate limit. The rate limit
   of every queue will be set equally to the assigned rate limit.


VF RX enabling
~~~~~~~~~~~~~~

   Run a CLI on the PF to enable/disable packet receiving for a specific VF:

   .. code-block:: console

       set port 0 vf 0 rx on|off

   This function can be used to stop/start packet receiving on VF.


VF TX enabling
~~~~~~~~~~~~~~

   Run a CLI on the PF to enable/disable packet transmitting for a specific VF:

   .. code-block:: console

       set port 0 vf 0 tx on|off

   This function can be used to stop/start packet transmitting on VF.


VF RX mode setting
~~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to set the RX mode for a specific VF:

   .. code-block:: console

       set port 0 vf 0 rxmode AUPE|ROPE|BAM|MPE on|off

   This function can be used to enable/disable some RX mode on VF, inclduing
   if accept untagged packets, if accept the packets matching the MAC filters,
   if accept MAC broadcast packets, if enable MAC multicast promiscuous mode.


The I40E specific VFd functions
-------------------------------


VF statistics
~~~~~~~~~~~~~

   Provide an API to get the a specific VF's statistic from PF.

   No testpmd example yet.


VF statistics resetting
~~~~~~~~~~~~~~~~~~~~~~~

   Provide an API to rest the a specific VF's statistic from PF.

   No testpmd example yet.


VF link status change notification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   Provide an API to let a specific VF know the physical link status changed.

   Normally if a VF received this notification, the driver should notify the
   APP to reset the VF port.

   No testpmd example yet.


VF MAC broadcast setting
~~~~~~~~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to enable/disable MAC broadcast packet receiving for
   a specific VF:

   .. code-block:: console

       set vf broadcast 0 0 on|off


VF MAC multicast promiscuous mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to enable/disable MAC multicast promiscuous mode for
   a specific VF:

   .. code-block:: console

       set vf allmulti 0 0 on|off


VF MAC unicast promiscuous mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to enable/disable MAC unicast promiscuous mode for
   a specific VF:

   .. code-block:: console

       set vf promisc 0 0 on|off


VF max bandwidth
~~~~~~~~~~~~~~~~

   Run a CLI on the PF to set the TX maximum bandwidth for a specific VF:

   .. code-block:: console

       set vf tx max-bandwidth 0 0 2000

   The maximum bandwidth is an absolute value in Mbps.


VF TC bandwidth allocation
~~~~~~~~~~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to set the TCs' TX bandwidth allocation for a specific
   VF:

   .. code-block:: console

       set vf tc tx min-bandwidth 0 0 (20,20,20,40)

   The allocated bandwidth should be set for all the TCs. The allocated
   bandwidth is a relative value in %. The summary of all the bandwidth should
   be 100.


VF TC max bandwidth
~~~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to set the TCs' TX maximum bandwidth for a specific VF:

   .. code-block:: console

       set vf tc tx max-bandwidth 0 0 0 10000

   The maximum bandwidth is an absolute value in Mbps.


TC strict priority scheduling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   Run a CLI on the PF to enable/disable several TCs' TX strict priority
   scheduling:

   .. code-block:: console

       set tx strict-link-priority 0 0x3

   The 0 in the TC bitmap means disabling the strict priority scheduling for
   this TC. The 1 means enabling the strict priority scheduling for this TC.
