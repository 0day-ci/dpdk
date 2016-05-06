
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


dpdk_pdump Application
======================
The dpdk_pdump application is a Data Plane Development Kit (DPDK) application
that runs as a DPDK secondary process and is capable of enabling packet capturing
on dpdk ports and capturing the packets.

Running the Application
-----------------------
The application has a pdump command line option with various sub arguments inside:
Parameters inside the parenthesis represents the mandatory parameters.
Parameters inside the square brackets represents optional parameters.
User has to pass on packet capture parameters under --pdump parameters, multiples of
--pdump can be passed to capture packets on different port and queue combinations.

.. code-block:: console

   ./$(RTE_TARGET)/app/pdump -- --pdump '(port=<port_id> |
   device_id=<pci address or device name>),
   (queue=2), (rx-dev=<iface/path to pcap file> |
   tx-dev=<iface/path to pcap file> |
   rxtx-dev=<iface/path to pcap file>),
   [ring-size=1024], [mbuf-size=2048], [total-num-mbufs=8191]'

Parameters
~~~~~~~~~~
``--pdump``: Specifies arguments needed for packet capturing.

``port``
Port id of the eth device on which packets should be captured.

``device_id``
PCI address (or) name of the eth device on which packets should be captured.

``queue``
Queue id of the eth device on which packets should be captured.
User can pass on queue value as ‘*’ if packets capturing has to be enabled
on all queues of the eth device.

``rx-dev``
Can be either pcap file name or any linux iface onto which ingress side packets of
dpdk eth device will be sent on for users to view.

``tx-dev``
Can be either pcap file name or any linux iface onto which egress side packets of
dpdk eth device will be sent on for users to view.

``rxtx-dev``
Can be either pcap file name or any linux iface onto which both ingress &
egress side packets of dpdk eth device will be sent on for users to view.

Note:
To receive ingress packets only, rx-dev should be passed.
To receive egress packets only, tx-dev should be passed.
To receive ingress and egress packets separately should pass on both rx-dev and tx-dev.
To receive both ingress and egress packets on same device, should pass only rxtx-dev.

Pdump tool uses these devices internally to create PCAPPMD vdev having ``tx_stream``
as either of these devices.

``ring-size``
Size of the ring. This value is used internally for ring creation.
The ring will be used to enqueue the packets from primary application to secondary.

``mbuf-size``
Size of the mbuf data room size. This is used internally for mempool creation.
Ideally this value must be same as primary application's mempool which is used for
packet rx.

``total-num-mbufs``
Total number mbufs in mempool. This is used internally for mempool creation.

Example
-------

.. code-block:: console

        $ sudo ./x86_64-native-linuxapp-gcc/app/dpdk_pdump -- --pdump 'device_id=0000:02:00.0,queue=*,rx-dev=/tmp/rx-file.pcap,tx-dev=/tmp/tx-file.pcap,ring-size=8192,mbuf-size=2176,total-num-mbufs=16384' --pdump 'device_id=0000:01:00.0,queue=*,rx-dev=/tmp/rx2-file.pcap,tx-dev=/tmp/tx2-file.pcap,ring-size=16384,mbuf-size=2176,total-num-mbufs=32768'
