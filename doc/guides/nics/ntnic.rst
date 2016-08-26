..  BSD LICENSE
    Copyright (c) 2016 Napatech A/S
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
    * Neither the name of Napatech nor the names of its
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

NTNIC Poll Mode Driver
======================

The NTNIC poll mode driver library (**librte_pmd_ntnic**) implements support
for **Napatech NIC** 40/50 Gbps adapters.
This PMD is implemented as a pure software virtual device and must be created
by using the EAL --vdev=parameter (parameters are explained i detail later).
It runs on top of the Napatech NIC Driver Suite that must be installed and
started.

Supported Features
------------------

- RSS (Receive Side Scaling)
- TSS (Transmit Side Scaling)
- Promiscuous mode
- Basic statistics


Prerequisites
-------------

Requires Napatech NIC and Napatech NIC Driver Suite installed and
running in version **0.3.0** or higher.
This includes external libraries and kernel driver for resources
allocations and initialization.

Pre-Installation Configuration
------------------------------

Environment variables
~~~~~~~~~~~~~~~~~~~~~

In order to compile the Napatech NIC PMD, user must:

* Export the environmental variable NAPATECH3_PATH with the path where
  the Napatech Driver suite was installed (default location is
  /opt/napatech3).

Config File Options
~~~~~~~~~~~~~~~~~~~

- ``CONFIG_RTE_LIBRTE_PMD_NTNIC`` (default **n**)

Using the NTNIC PMD from a DPDK application using EAL vdev parameter
--------------------------------------------------------------------

Napatech NIC PMD VDEV parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Napatech NIC PMD vdev has the following command line parameters to enable
its features.

* port
  Configures which NT port the vdev should use.

      --vdev eth_ntnic0,port=4,rxqs=1,txqs=1

  This will create a DPDK port0 using NT port 4

* rxqs
  Control how many receive queues a vdev should have. The traffic from a vdev
  will be load balanced to this amount of queues and each queue can be handled
  by its own lcore.
  Note: The ntservice.ini HostBuffersRx must be configured to the sum of all
  receive queues across all vdevs.

      --vdev eth_ntnic0,port=0,rxqs=8,txqs=1

  Traffic from NT port 0 will be split across
  8 queues meaning that 8 lcores can be
  opened on DPDK port 0 and each will be
  their individual traffic.

* hash
  Control which load balancing scheme should be used when running with multiple
  receive queues.
  The possible values are:
    1 = HashRoundRobin
    2 = Hash2TupleSorted
    3 = Hash5TupleSorted

      --vdev eth_ntnic0,port=0,rxqs=4,hash=3,txqs=1

  Create 4 queues from NT port 0 and
  load balance the traffic using the
  5-tuple sorted algorithm. Each of the
  4 queues can be opened via DPDK port 0

* txqs
  Control how many transmit queues a vdev should have. Each queue can be used
  by a lcore and each queue is independant for other queues.
  Note: The ntservice.ini HostBuffersTx must be configured to the sum of all
  transmit queues across all vdevs.

      --vdev eth_ntnic0,port=0,txqs=32,rxqs=1

  Enable 32 transmit queues on NT port 0


Changes to Napatech driver configuration file ntservice.ini
-----------------------------------------------------------
Depending on the number of queues that is created, more hostbuffers may be
needed pre-allocated by the Napatech NIC Driver. This is controlled in the
ntservice.ini file (default locations is /opt/napatech3/config).
The Napatech NIC Driver must be re-started when this configuration file is
changed.

