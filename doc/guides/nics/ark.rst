.. BSD LICENSE

    Copyright (c) 2015-2017 Atomic Rules LLC
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
    * Neither the name of Atomic Rules LLC nor the names of its
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

ARK Poll Mode Driver
====================

The ARK PMD is a DPDK poll-mode driver for the Atomic Rules Arkville
(ARK) family of devices.

More information can be found at the `Atomic Rules website
<http://atomicrules.com>`_.

Overview
--------

The Atomic Rules Arkville product is DPDK and AXI compliant product
that marshals packets across a PCIe conduit between host DPDK mbufs and
FPGA AXI streams.

The ARK PMD, and the spirit of the overall Arkville product,
has been to take the DPDK API/ABI as a fixed specification;
then implement much of the business logic in FPGA RTL circuits.
The approach of *working backwards* from the DPDK API/ABI and having
the GPP host software *dictate*, while the FPGA hardware *copes*,
results in significant performance gains over a naive implementation.

While this document describes the ARK PMD software, it is helpful to
understand what the FPGA hardware is and is not. The Arkville RTL
component provides a single PCIe Physical Function (PF) supporting
some number of RX/Ingress and TX/Egress Queues. The ARK PMD controls
the Arkville core through a dedicated opaque Core BAR (CBAR).
To allow users full freedom for their own FPGA application IP,
an independent FPGA Application BAR (ABAR) is provided.

One popular way to imagine Arkville's FPGA hardware aspect is as the
FPGA PCIe-facing side of a so-called Smart NIC. The Arkville core does
not contain any MACs, and is link-speed independent, as well as
agnostic to the number of physical ports the application chooses to
use. The ARK driver exposes the familiar PMD interface to allow packet
movement to and from mbufs across multiple queues.

However FPGA RTL applications could contain a universe of added
functionality that an Arkville RTL core does not provide or can
not anticipate. To allow for this expectation of user-defined
innovation, the ARK PMD provides a dynamic mechanism of adding
capabilities without having to modify the ARK PMD.

The ARK PMD is intended to support all instances of the Arkville
RTL Core, regardless of configuration, FPGA vendor, or target
board. While specific capabilities such as number of physical
hardware queue-pairs are negotiated; the driver is designed to
remain constant over a broad and extendable feature set.

Intentionally, Arkville by itself DOES NOT provide common NIC
capabilities such as offload or receive-side scaling (RSS).
These capabilities would be viewed as a gate-level "tax" on
Green-box FPGA applications that do not require such function.
Instead, they can be added as needed with essentially no
overhead to the FPGA Application.

Data Path Interface
-------------------

Ingress RX and Egress TX operation is by the nominal DPDK API .
The driver supports single-port, multi-queue for both RX and TX.

Refer to ``ark_ethdev.h`` for the list of supported methods to
act upon RX and TX Queues.

Configuration Information
-------------------------

**DPDK Configuration Parameters**

  The following configuration options are available for the ARK PMD:

   * **CONFIG_RTE_LIBRTE_ARK_PMD** (default y): Enables or disables inclusion
     of the ARK PMD driver in the DPDK compilation.

   * **CONFIG_RTE_LIBRTE_ARK_DEBUG_RX** (default n): Enables or disables debug
     logging and internal checking of RX ingress logic within the ARK PMD driver.

   * **CONFIG_RTE_LIBRTE_ARK_DEBUG_TX** (default n): Enables or disables debug
     logging and internal checking of TX egress logic within the ARK PMD driver.

   * **CONFIG_RTE_LIBRTE_ARK_DEBUG_STATS** (default n): Enables or disables debug
     logging of detailed packet and performance statistics gathered in
     the PMD and FPGA.

   * **CONFIG_RTE_LIBRTE_ARK_DEBUG_TRACE** (default n): Enables or disables debug
     logging of detailed PMD events and status.


Building DPDK
-------------

See the :ref:`DPDK Getting Started Guide for Linux <linux_gsg>` for
instructions on how to build DPDK.

By default the ARK PMD library will be built into the DPDK library.

For configuring and using UIO and VFIO frameworks, please also refer :ref:`the
documentation that comes with DPDK suite <linux_gsg>`.

Supported ARK RTL PCIe Instances
--------------------------------

ARK PMD supports the following Arkville RTL PCIe instances including:

* ``1d6c:100d`` - AR-ARKA-FX0 [Arkville 32B DPDK Data Mover]
* ``1d6c:100e`` - AR-ARKA-FX1 [Arkville 64B DPDK Data Mover]

Supported Operating Systems
---------------------------

Any Linux distribution fulfilling the conditions described in ``System Requirements``
section of :ref:`the DPDK documentation <linux_gsg>` or refer to *DPDK Release Notes*.

Supported Features
------------------

* Dynamic ARK PMD extensions
* Multiple receive and transmit queues
* Jumbo frames up to 9K
* Hardware Statistics

Unsupported Features
--------------------

Features that may be part of, or become part of, the Arkville RTL IP that are
not currently supported or exposed by the ARK PMD include:

* PCIe SR-IOV Virtual Functions (VFs)
* Arkville's Packet Generator Control and Status
* Arkville's Packet Director Control and Status
* Arkville's Packet Checker Control and Status
* Arkville's Timebase Management

Pre-Requisites
--------------

#. Prepare the system as recommended by DPDK suite.  This includes environment
   variables, hugepages configuration, tool-chains and configuration

#. Insert igb_uio kernel module using the command 'modprobe igb_uio'

#. Bind the intended ARK device to igb_uio module

At this point the system should be ready to run DPDK applications. Once the
application runs to completion, the ARK PMD can be detached from igb_uio if necessary.

Usage Example
-------------

This section demonstrates how to launch **testpmd** with Atomic Rules ARK
devices managed by librte_pmd_ark.

#. Load the kernel modules:

   .. code-block:: console

      modprobe uio
      insmod ./x86_64-native-linuxapp-gcc/kmod/igb_uio.ko

   .. note::

      The ARK PMD driver depends upon the igb_uio user space I/O kernel module

#. Mount and request huge pages:

   .. code-block:: console

      mount -t hugetlbfs nodev /mnt/huge
      echo 256 > /sys/devices/system/node/node0/hugepages/hugepages-2048kB/nr_hugepages

#. Bind UIO driver to ARK device at 0000:01:00.0 (using dpdk-devbind.py):

   .. code-block:: console

      ./usertools/dpdk-devbind.py --bind=igb_uio 0000:01:00.0

   .. note::

      The last argument to dpdk-devbind.py is the 4-tuple that indentifies a specific PCIe
      device. You can use lspci -d 1d6c: to indentify all Atomic Rules devices in the system,
      and thus determine the correct 4-tuple argument to dpdk-devbind.py

#. Start testpmd with basic parameters:

   .. code-block:: console

      ./x86_64-native-linuxapp-gcc/app/testpmd -l 0-3 -n 4 -- -i

   Example output:

   .. code-block:: console

      [...]
      EAL: PCI device 0000:01:00.0 on NUMA socket -1
      EAL:   probe driver: 1d6c:100e rte_ark_pmd
      EAL:   PCI memory mapped at 0x7f9b6c400000
      PMD: eth_ark_dev_init(): Initializing 0:2:0.1
      ARKP PMD CommitID: 378f3a67
      Configuring Port 0 (socket 0)
      Port 0: DC:3C:F6:00:00:01
      Checking link statuses...
      Port 0 Link Up - speed 100000 Mbps - full-duplex
      Done
      testpmd>
