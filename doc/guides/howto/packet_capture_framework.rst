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


This doc describes how the Data Plane Development Kit (DPDK) packet capture
framework is used for capturing packets on the DPDK ports. It is written with
users of DPDK in mind who want to know more about the feature and for those
who want to monitor traffic on DPDK-controlled devices.

The DPDK packet capture framework was introduced in DPDK v16.07.
The DPDK packet capture framework consists of the DPDK pdump library and
DPDK pdump tool.


DPDK pdump Library and pdump Tool
=================================

The ``librte_pdump`` library provides the APIs required to allow users to
initialize the packet capture framework and to enable or disable packet
capture. The library works on a client/server model and its usage is
recommended for debugging purposes.

The ``dpdk-pdump`` tool is developed based on the ``librte_pdump`` library.
The ``dpdk-pdump`` tool runs as a DPDK secondary process and is capable
of enabling or disabling packet capture on the DPDK ports.
The ``dpdk-pdump`` tool provides command-line options with which users can
request enabling or disabling of the packet capture on DPDK ports.

The application which initializes the packet capture framework will act as a
server and the application that enables or disables the packet capture will
act as a client. The server sends the Rx and Tx packets from the DPDK ports
to the client.

In DPDK the ``testpmd`` application is modified to initialize the packet
capture framework and act as a server, and the ``dpdk-pdump`` tool acts
as a client. To view Rx or Tx packets of ``testpmd``, the application should
be launched first, and then the ``dpdk-pdump`` tool. Packets from the
``testpmd`` will be sent to the tool, which then sends them on to the pcap
pmd device and that device writes them to the pcap file or to an external
interface depending on the command-line option used.

   .. Note::
      * The ``dpdk-pdump`` tool can only be used in conjunction with a primary
        application which has the packet capture framework initialized already.
        In dpdk, only the ``testpmd`` is modified to initialize packet capture
        framework, other applications remain untouched. So, if the ``dpdk-pdump``
        tool has to be used with any application other than the testpmd, user
        needs to explicitly modify that application to call packet capture
        framework initialization code. Refer ``app/test-pmd/testpmd.c``
        code and look for ``pdump`` keyword to see how this is done.

      * The ``dpdk-pdump`` tool depends on libpcap based PMD which is disabled
        by default in the build configuration files,
        owing to an external dependency on the libpcap development files
        which must be installed on the board.
        Once the libpcap development files are installed, the libpcap based PMD
        can be enabled by setting CONFIG_RTE_LIBRTE_PMD_PCAP=y and recompiling the DPDK.


Test Environment
----------------

The overview of using packet capture framework and dpdk-pdump tool
for packet capturing on the DPDK port
in :numref:`figure_packet_capture_framework`.

.. _figure_packet_capture_framework:

.. figure:: img/packet_capture_framework.*

   Packet capturing on DPDK port using the dpdk-pdump tool.

Configuration Steps
-------------------

The following steps demonstrate how to run the ``dpdk-pdump`` tool to
capture Rx side packets of dpdk_port0
in :numref:`figure_packet_capture_framework` and inspect them using
``tcpdump``.

#. Modify the DPDK primary application to initialize the packet capture framework
   as mentioned in above note, enable below config options and build the DPDK::

     CONFIG_RTE_LIBRTE_PMD_PCAP=y
     CONFIG_RTE_LIBRTE_PDUMP=y

#. Launch testpmd as the primary application::

     sudo ./app/testpmd -c 0xf0 -n 4 -- -i --port-topology=chained

#. Launch the pdump tool as follows::

     sudo ./build/app/dpdk-pdump -- \
                              --pdump 'port=0,queue=*,rx-dev=/tmp/capture.pcap'

#. Send traffic to dpdk_port0 from traffic generator.
   Inspect packets captured in the file capture.pcap using a tool
   that can interpret pcap files, for example tcpdump::

     $tcpdump -nr /tmp/capture.pcap
     reading from file /tmp/capture.pcap, link-type EN10MB (Ethernet)
     11:11:36.891404 IP 4.4.4.4.whois++ > 3.3.3.3.whois++: UDP, length 18
     11:11:36.891442 IP 4.4.4.4.whois++ > 3.3.3.3.whois++: UDP, length 18
     11:11:36.891445 IP 4.4.4.4.whois++ > 3.3.3.3.whois++: UDP, length 18

Conclusion
----------

In this doc we described the ``librte_pdump`` library and ``dpdk-pdump`` tool
usage for capturing traffic of DPDK ports.

Additional Information
----------------------

More details on ``librte_pdump`` library and ``dpdk-pdump`` tool can be found
at following links.

#. http://dpdk.org/doc/guides/prog_guide/pdump_lib.html
#. http://dpdk.org/doc/guides/tools/pdump.html
