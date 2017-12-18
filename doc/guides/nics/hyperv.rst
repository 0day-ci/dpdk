..  BSD LICENSE
    Copyright 2017 6WIND S.A.
    Copyright 2017 Mellanox

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    * Neither the name of 6WIND S.A. nor the names of its
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

HYPERV poll mode driver
=======================

The HYPERV PMD (librte_pmd_hyperv) provides support for NetVSC interfaces
and associated SR-IOV virtual function (VF) devices found in Linux virtual
machines running on Microsoft Hyper-V_ (including Azure) platforms.

.. _Hyper-V: https://docs.microsoft.com/en-us/windows-hardware/drivers/network/overview-of-hyper-v

Implementation details
----------------------

Each instance of this driver effectively needs to drive two devices: the
NetVSC interface proper and its SR-IOV VF (referred to as "physical" from
this point on) counterpart sharing the same MAC address.

Physical devices are part of the host system and cannot be maintained during
VM migration. From a VM standpoint they appear as hot-plug devices that come
and go without prior notice.

When the physical device is present, egress and most of the ingress traffic
flows through it; only multicasts and other hypervisor control still flow
through NetVSC. Otherwise, NetVSC acts as a fallback for all traffic.

To avoid unnecessary code duplication and ensure maximum performance,
handling of physical devices is left to their original PMDs; this virtual
device driver (also known as *vdev*) manages other PMDs as summarized by the
following block diagram::

         .------------------.
         | DPDK application |
         `--------+---------'
                  |
           .------+------.
           | DPDK ethdev |
           `------+------'       Control
                  |                 |
     .------------+------------.    v    .------------.
     |       failsafe PMD      +---------+ hyperv PMD |
     `--+-------------------+--'         `------------'
        |                   |
        |          .........|.........
        |          :        |        :
   .----+----.     :   .----+----.   :
   | tap PMD |     :   | any PMD |   :
   `----+----'     :   `----+----'   : <-- Hot-pluggable
        |          :        |        :
 .------+-------.  :  .-----+-----.  :
 | NetVSC-based |  :  | SR-IOV VF |  :
 |   netdevice  |  :  |   device  |  :
 `--------------'  :  `-----------'  :
                   :.................:

Build options
-------------

- ``CONFIG_RTE_LIBRTE_HYPERV_PMD`` (default ``y``)

   Toggle compilation of this driver.

- ``CONFIG_RTE_LIBRTE_HYPERV_DEBUG`` (default ``n``)

   Toggle additional debugging code.

Run-time parameters
-------------------

To invoke this PMD, applications have to explicitly provide the
``--vdev=net_hyperv`` EAL option.

The following device parameters are supported:

- ``iface`` [string]

  Provide a specific NetVSC interface (netdevice) name to attach this PMD
  to. Can be provided multiple times for additional instances.

- ``mac`` [string]

  Same as ``iface`` except a suitable NetVSC interface is located using its
  MAC address.

- ``force`` [int]

  If nonzero, forces the use of specified interfaces even if not detected as
  NetVSC.

Not specifying either ``iface`` or ``mac`` makes this PMD attach itself to
all NetVSC interfaces found on the system.
