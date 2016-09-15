..  BSD LICENSE
    Copyright(c) 2016 Intel Corporation. All rights reserved.
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

Tun/Tap Poll Mode Driver
========================================

The rte_eth_tap.c PMD creates a device using TUN/TAP interfaces on the local
host. The PMD allows for DPDK and the host to communicate using a raw device
interface on the host and in the DPDK application.

The device created is a TAP device, which sends/receives packet in a raw format
with a L2 header. The usage for a TAP PMD is for connectivity to the local host
using a TAP interface. When the TAP PMD is initialized it will create a number
of tap devices in the host accessed via 'ifconfig -a' or 'ip' command. The
commands can be used to assign and query the virtual like device.

These TAP interfaces can be used with wireshark or tcpdump or Pktgen-DPDK along
with being able to be used as a network connection to the DPDK application. The
method enable one or more interfaces is to use the --vdev=eth_tap option on the
DPDK application  command line. Each --vdev=eth_tap option give will create an
interface named dtap0, dtap1, ... and so forth.

.. code-block:: console

   The interfaced name can be changed by adding the iface=foo0
   e.g. --vedv=eth_tap,iface=foo0 --vdev=eth_tap,iface=foo1, ...

.. code-block:: console

   Also the speed of the interface can be changed from 10G to whatever number
   needed, but the interface does not enforce that speed.
   e.g. --vdev=eth_tap,iface=foo0,speed=25000

After the DPDK application is started you can send and receive packets on the
interface using the standard rx_burst/tx_burst APIs in DPDK. From the host point
of view you can use any host tool like tcpdump, wireshark, ping, Pktgen and
others to communicate with the DPDK application. The DPDK application may not
understand network protocols like IPv4/6, UDP or TCP unless the application has
been written to understand these protocols.

If you need the interface as a real network interface meaning running and has
a valid IP address then you can do this with the following commands:

.. code-block:: console

   sudo ip link set dtap0 up; sudo ip addr add 192.168.0.250/24 dev dtap0
   sudo ip link set dtap1 up; sudo ip addr add 192.168.1.250/24 dev dtap1

Please change the IP addresses as you see fit.

If routing is enabled on the host you can also communicate with the DPDK App
over the internet via a standard socket layer application as long as you account
for the protocol handing in the application.

If you have a Network Stack in your DPDK application or something like it you
can utilize that stack to handle the network protocols. Plus you would be able
to address the interface using an IP address assigned to the internal interface.
