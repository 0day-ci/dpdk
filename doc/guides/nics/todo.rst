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


PMDs internal TODO list
=======================

This is the list for tracking required PMD changes triggered by library modifications.

.. table:: PMDs internal TODO list

 +-------------------+--------------------------------+----------+----------------+-----------------------------------+
 | TODO              | PMDs                           | Deadline | Related Commit | Note                              |
 +===================+================================+==========+================+===================================+
 | mbuf organisation | | af_packet, ark, avp, bnx2x,  | v18.02   | 8f094a9ac5d7   | Remove unnecessary mbuf field     |
 |                   | | bnxt, bonding, cxgbe, dpaa,  |          |                | initialization.                   |
 |                   | | dpaa2, e1000, ena,           |          |                |                                   |
 |                   | | failsafe, fm10k,             |          |                |                                   |
 |                   | | kni, mlx4,                   |          |                |                                   |
 |                   | | mrvl, nfp, octeontx,         |          |                |                                   |
 |                   | | pcap, qede, ring, sfc,       |          |                |                                   |
 |                   | | softnic, szedata2, tap,      |          |                |                                   |
 |                   | | thunderx, vhost,             |          |                |                                   |
 |                   | | vmxnet3                      |          |                |                                   |
 +-------------------+--------------------------------+----------+----------------+-----------------------------------+
 | dynamic logging   | | af_packet, ark, avp, bnx2x,  | v18.08   | c1b5fa94a46f   | Switch to dynamic logging         |
 |                   | | bnxt, bonding, cxgbe, dpaa,  |          |                | functions, remove static debug    |
 |                   | | dpaa2, e1000, ena, enic,     |          |                | config options.                   |
 |                   | | failsafe, fm10k, ixgbe,      |          |                |                                   |
 |                   | | kni, liquidio, mlx4, mlx5,   |          |                |                                   |
 |                   | | mrvl, nfp, null, octeontx,   |          |                |                                   |
 |                   | | pcap, qede, ring, sfc,       |          |                |                                   |
 |                   | | softnic, szedata2, tap,      |          |                |                                   |
 |                   | | thunderx, vhost, virtio,     |          |                |                                   |
 |                   | | vmxnet3                      |          |                |                                   |
 +-------------------+--------------------------------+----------+----------------+-----------------------------------+
 | | descriptor      | | af_packet, ark, avp, bnx2x,  | v19.02   | b1b700ce7d6f   | Replace descriptor_done API with  |
 | | status API      | | bnxt, bonding, cxgbe, dpaa,  |          |                | rx_descriptor_status and          |
 |                   | | dpaa2, ena, enic,            |          |                | tx_descriptor_status APIs.        |
 |                   | | failsafe, fm10k,             |          |                |                                   |
 |                   | | kni, liquidio, mlx4,         |          |                |                                   |
 |                   | | mrvl, nfp, null, octeontx,   |          |                |                                   |
 |                   | | pcap, qede, ring, sfc,       |          |                |                                   |
 |                   | | softnic, szedata2, tap,      |          |                |                                   |
 |                   | | thunderx, vhost, virtio,     |          |                |                                   |
 |                   | | vmxnet3                      |          |                |                                   |
 +-------------------+--------------------------------+----------+----------------+-----------------------------------+
 | new offload flags | | af_packet, ark, avp, bnx2x,  | v18.05   | ce17eddefc20   | Use new ethdev offloads filed     |
 |                   | | bnxt, bonding, cxgbe, dpaa,  |          | cba7f53b717d   | to get requested offload list     |
 |                   | | dpaa2, e1000, ena, enic,     |          |                | instead of bitfield values.       |
 |                   | | failsafe, fm10k, i40e, ixgbe,|          |                |                                   |
 |                   | | kni, liquidio, mlx4, mlx5,   |          |                |                                   |
 |                   | | mrvl, nfp, null, octeontx,   |          |                |                                   |
 |                   | | pcap, qede, ring, sfc,       |          |                |                                   |
 |                   | | softnic, szedata2, tap,      |          |                |                                   |
 |                   | | thunderx, vhost, virtio,     |          |                |                                   |
 |                   | | vmxnet3                      |          |                |                                   |
 +-------------------+--------------------------------+----------+----------------+-----------------------------------+
 | | check new mbuf  | | af_packet, ark, avp, bnx2x,  | v18.02   | 380a7aab1ae2   | mbuf flag PKT_RX_VLAN_PKT renamed |
 | | VLAN flag       | | bnxt, bonding, cxgbe, dpaa,  |          |                | to PKT_RX_VLAN and its meaning    |
 | | PKT_RX_VLAN     | | dpaa2, e1000, ena, enic,     |          |                | changed, confirm PMD uses new     |
 |                   | | failsafe, fm10k, i40e, ixgbe,|          |                | flag correct.                     |
 |                   | | kni, liquidio, mlx4, mlx5,   |          |                |                                   |
 |                   | | mrvl, nfp, null, octeontx,   |          |                |                                   |
 |                   | | pcap, qede, ring, sfc,       |          |                |                                   |
 |                   | | softnic, szedata2, tap,      |          |                |                                   |
 |                   | | thunderx, vhost, virtio,     |          |                |                                   |
 |                   | | vmxnet3                      |          |                |                                   |
 +-------------------+--------------------------------+----------+----------------+-----------------------------------+
