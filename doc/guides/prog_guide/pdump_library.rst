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

.. _Pdump_Library:

pdump Library
=============

Pdump library provides framework for packet capturing on DPDK.

Operation
---------

Pdump library provides APIs to support packet capturing on dpdk ethernet devices.
Library provides APIs to initialize the packet capture framework, enable/disable
the packet capture and un initialize the packet capture framework.

Pdump library works on server and client based model.

Sever is responsible for enabling/disabling the packet captures.
Clients are responsible for requesting enable/disable of the
packet captures.

As part of packet capture framework initialization, pthread and
the server socket is created. Only one server socket is allowed on the system.
As part of enabling/disabling the packet capture, client sockets are created
and multiple client sockets are allowed.
Who ever calls initialization first they will succeed with the initialization,
next subsequent calls of initialization are not allowed. So next users can only
request enabling/disabling the packet capture.

Library provides below APIs

``rte_pdump_init()``
This API initializes the packet capture framework.

``rte_pdump_enable()``
This API enables the packet capturing on a given port and queue.
Note: filter option in the API is place holder for future use.

``rte_pdump_enable_by_deviceid()``
This API enables the packet capturing on a given device id
(device name or pci address) and queue.
Note: filter option in the API is place holder for future use.

``rte_pdump_disable()``
This API disables the packet capturing on a given port and queue.

``rte_pdump_disable_by_deviceid()``
This API disables the packet capturing on a given device_id and queue.

``rte_pdump_uninit()``
This API un initializes the packet capture framework.


Implementation Details
----------------------

On a call to library API ``rte_pdump_init()``, library creates pthread and server socket.
Server socket in pthread context will be listening to the client requests to enable/disable
the packet capture.

Who ever calls this API first will have server socket created,
subsequent calls to this APIs will not create any further server sockets. i.e only one server
socket is allowed.

On each call to library APIs ``rte_pdump_enable()/rte_pdump_enable_by_deviceid()``
to enable the packet capture, library creates separate client sockets,
builds up enable request and sends the request to the server.
Server listening on the socket will serve the request, enable the packet capture
by registering ethernet rx/tx callbacks for the given port/device_id and queue combinations.
Server mirrors the packets to new mempool and enqueue them to the ring that clients has passed
in these APIs.
Server sends the response back to the client about the status of the request that was processed.
After the response is received from the server, client sockets will be closed.

On each call to library APIs ``rte_pdump_disable()/rte_pdump_disable_by_deviceid()``
to disable packet capture, library creates separate client sockets,
builds up disable request and sends the request to the server.
Server listening on the socket will serve the request, disable the packet capture
by removing the ethernet rx/tx callbacks for the given port/device_id and queue combinations.
Server sends the response back to the client about the status of the request that was processed.
After the response is received from the server, client sockets will be closed.

On a call to library API ``rte_pdump_uninit()``, library closes the pthread and the server socket.


Use Case: Packet Capturing
--------------------------

app/pdump tool is developed based on this library to capture the packets
in DPDK.
Users can develop their own packet capturing application using new library
if they wish to do so.
