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

The ``pdump`` library provides the framework for the packet capturing on DPDK.
Library provides the below APIs to initialize the packet capture framework, to enable
or disable the packet capture and to un initialize the packet capture framework.

``rte_pdump_init()``:
This API initializes the packet capture framework.

``rte_pdump_enable()``:
This API enables the packet capture on a given port and the queue.
Note: filter option in the API is the place holder for the future enhancements.

``rte_pdump_enable_by_deviceid()``:
This API enables the packet capture on a given device id(``vdev name or pci address``) and the queue.
Note: filter option in the API is the place holder for the future enhancements.

``rte_pdump_disable()``:
This API disables the packet capture on a given port and the queue.

``rte_pdump_disable_by_deviceid()``:
This API disables the packet capture on a given device id(``vdev name or pci address``) and the queue.

``rte_pdump_uninit()``:
This API un initializes the packet capture framework.


Operation
---------

The ``pdump`` library works on the server and the client based model. The sever is responsible for enabling or
disabling the packet capture and the clients are responsible to request enable or disable the packet capture.

The packet capture framework, as part of it's initialization, creates the pthread and creates the server socket in
the pthread. The application who calls the framework initialization first, will have the server socket created and
the further calls to the framework initialization by same application or other applications is not allowed i.e. only
one server socket is allowed on the system. So the other applications, can only request for enabling or disabling of
the packet capture and the client socket is created to send the request to the server. The server socket will be
listening to the client requests for enabling or disabling the packet capture.


Implementation Details
----------------------

The library API ``rte_pdump_init()``, initializes the packet capture framework by creating the pthread and the server
socket.The server socket in the pthread context will be listening to the client requests to enable or disable the
packet capture. Who ever calls this API first will have the server socket created, the subsequent calls to this APIs
will not create any further server socket. i.e. only one server socket is allowed.

These library APIs ``rte_pdump_enable()/rte_pdump_enable_by_deviceid()`` enables the packet capture, on each call to
these APIs, library creates the separate client socket, creates the pdump enable request and send the request to the
server. Server who is listening on the socket will take the request, enable the packet capture by registering the
Ethernet rx/tx callbacks for the given port or device_id and queue combinations. Then server will mirror the packets
to the new mempool and enqueue them to the ring that clients has passed in to these APIs, server also sends the response
back to the client about the status of the request that was processed. After the response is received from the server,
client socket is closed.

The library APIs ``rte_pdump_disable()/rte_pdump_disable_by_deviceid()`` disables the packet capture, on each call to
these APIs, library creates the separate client socket, creates the pdump disable request and send the request to the
server. Server who is listening on the socket will take the request, disable the packet capture by removing the
Ethernet rx/tx callbacks for the given port or device_id and queue combinations. Server sends the response back to the
client about the status of the request that was processed. After the response is received from the server, client
socket is closed.

The library API ``rte_pdump_uninit()``, un initializes the packet capture framework by closing the pthread and the
server socket.


Use Case: Packet Capturing
--------------------------

DPDK ``app/pdump`` tool is developed based on this library to capture the packets in DPDK.
Users can use this library to develop their own packet capturing application.
