..  BSD LICENSE
    Copyright(c) 2010-2016 Intel Corporation. All rights reserved.
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

Link Reset Sample Application (in Virtualized Environments)
===========================================================

The Link Reset sample application is a simple example of VF traffic recovery
using the Data Plane Development Kit (DPDK) which also takes advantage of Single
Root I/O Virtualization (SR-IOV) features in a virtualized environment.

Overview
--------

The Link Reset sample application, which should operate in virtualized
environments, performs L2 forwarding for each packet that is received on an
RX_PORT.
This example is extended from the L2 forwarding example. Please reference the
example of L2 forwarding in virtualized environments for more details and
explanation about the behavior of forwarding and how to setup the test.
The purpose of this example is to show when the PF port is down and up, the VF
port can recover and the traffic can recover too.

Virtual Function Setup Instructions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This application can use the virtual function available in the system and
therefore can be used in a virtual machine without passing through
the whole Network Device into a guest machine in a virtualized scenario.
The virtual functions can be enabled in the host machine or the hypervisor
with the respective physical function driver.

For example, in a Linux* host machine, it is possible to enable a virtual
function using the following command:

.. code-block:: console

    modprobe ixgbe max_vfs=2,2

This command enables two Virtual Functions on each of Physical Function of the
NIC, with two physical ports in the PCI configuration space.
It is important to note that enabled Virtual Function 0 and 2 would belong to
Physical Function 0 and Virtual Function 1 and 3 would belong to Physical
Function 1, in this case enabling a total of four Virtual Functions.

Compiling the Application
-------------------------

#.  Go to the example directory:

    .. code-block:: console

        export RTE_SDK=/path/to/rte_sdk
        cd ${RTE_SDK}/examples/link_reset

#.  Set the target (a default target is used if not specified). For example:

    .. code-block:: console

        export RTE_TARGET=x86_64-native-linuxapp-gcc

    *See the DPDK Getting Started Guide* for possible RTE_TARGET values.

#.  Build the application:

    .. code-block:: console

        make

Running the Application
-----------------------

The application requires a number of command line options:

.. code-block:: console

    ./build/link_reset [EAL options] -- -p PORTMASK [-q NQ]

where,

*   p PORTMASK: A hexadecimal bitmask of the ports to configure

*   q NQ: A number of queues (=ports) per lcore (default is 1)

To run the application in linuxapp environment with 4 lcores, 16 ports and 8 RX
queues per lcore, issue the command:

.. code-block:: console

    $ ./build/link_reset -c f -n 4 -- -q 8 -p ffff

Refer to the *DPDK Getting Started Guide* for general information on running applications
and the Environment Abstraction Layer (EAL) options.

Explanation
-----------

Handle VF link reset event
~~~~~~~~~~~~~~~~~~~~~~~~~~

In main function, when initialising each port, register a callback for reset
event.

.. code-block:: c

        /* Initialise each port */
        for (portid = 0; portid < nb_ports; portid++) {
        ......
                /* register reset interrupt callback */
                rte_eth_dev_callback_register(portid,
                        RTE_ETH_EVENT_INTR_RESET, reset_event_callback, NULL);
        ......
        }

The callback function *reset_event_callback* will be executed in the
interruption thread. But we want the event to be handled in the management
thread. So in the callback function only a reset flag *stop_forwarding* is set.

.. code-block:: c

        static void
        reset_event_callback(uint8_t port_id, enum rte_eth_event_type type, void *param)
        {
                RTE_SET_USED(param);

                printf("\n\nIn registered callback...\n");
                printf("Event type: %s on port %d\n",
                        type == RTE_ETH_EVENT_INTR_RESET ? "RESET interrupt" :
                        "unknown event", port_id);
               reset_port = port_id;
               rte_compiler_barrier(); /* prevent compiler reordering */
                stop_forwarding = 1;
        }

The management thread keeps checking the reset flag *stop_forwarding* to see
if VF port reset is needed. If so, it should call the API *rte_eth_dev_reset*
to reset VF port. After that, the traffic will recover.

.. code-block:: c

        while (1) {
                rte_delay_ms(1000);
                printf("..");
                if (stop_forwarding == 1) {
                        printf("\nreset\n");
                        rte_eth_dev_reset(reset_port);
                        stop_forwarding = 0;
                }
                if (force_quit)
                        break;
        }
