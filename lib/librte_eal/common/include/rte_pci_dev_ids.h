/*-
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 *
 *   GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2010-2015 Intel Corporation. All rights reserved.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 *
 *   Contact Information:
 *   Intel Corporation
 *
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef RTE_PCI_DEV_ID_DECL_IGB
#define RTE_PCI_DEV_ID_DECL_IGB(vend, dev)
#endif

#ifndef RTE_PCI_DEV_ID_DECL_IGBVF
#define RTE_PCI_DEV_ID_DECL_IGBVF(vend, dev)
#endif

#ifndef PCI_VENDOR_ID_INTEL
/** Vendor ID used by Intel devices */
#define PCI_VENDOR_ID_INTEL 0x8086
#endif

/******************** Physical IGB devices from e1000_hw.h ********************/

#define E1000_DEV_ID_82576                      0x10C9
#define E1000_DEV_ID_82576_FIBER                0x10E6
#define E1000_DEV_ID_82576_SERDES               0x10E7
#define E1000_DEV_ID_82576_QUAD_COPPER          0x10E8
#define E1000_DEV_ID_82576_QUAD_COPPER_ET2      0x1526
#define E1000_DEV_ID_82576_NS                   0x150A
#define E1000_DEV_ID_82576_NS_SERDES            0x1518
#define E1000_DEV_ID_82576_SERDES_QUAD          0x150D
#define E1000_DEV_ID_82575EB_COPPER             0x10A7
#define E1000_DEV_ID_82575EB_FIBER_SERDES       0x10A9
#define E1000_DEV_ID_82575GB_QUAD_COPPER        0x10D6
#define E1000_DEV_ID_82580_COPPER               0x150E
#define E1000_DEV_ID_82580_FIBER                0x150F
#define E1000_DEV_ID_82580_SERDES               0x1510
#define E1000_DEV_ID_82580_SGMII                0x1511
#define E1000_DEV_ID_82580_COPPER_DUAL          0x1516
#define E1000_DEV_ID_82580_QUAD_FIBER           0x1527
#define E1000_DEV_ID_I350_COPPER                0x1521
#define E1000_DEV_ID_I350_FIBER                 0x1522
#define E1000_DEV_ID_I350_SERDES                0x1523
#define E1000_DEV_ID_I350_SGMII                 0x1524
#define E1000_DEV_ID_I350_DA4                   0x1546
#define E1000_DEV_ID_I210_COPPER                0x1533
#define E1000_DEV_ID_I210_COPPER_OEM1           0x1534
#define E1000_DEV_ID_I210_COPPER_IT             0x1535
#define E1000_DEV_ID_I210_FIBER                 0x1536
#define E1000_DEV_ID_I210_SERDES                0x1537
#define E1000_DEV_ID_I210_SGMII                 0x1538
#define E1000_DEV_ID_I210_COPPER_FLASHLESS      0x157B
#define E1000_DEV_ID_I210_SERDES_FLASHLESS      0x157C
#define E1000_DEV_ID_I211_COPPER                0x1539
#define E1000_DEV_ID_I354_BACKPLANE_1GBPS       0x1F40
#define E1000_DEV_ID_I354_SGMII                 0x1F41
#define E1000_DEV_ID_I354_BACKPLANE_2_5GBPS     0x1F45
#define E1000_DEV_ID_DH89XXCC_SGMII             0x0438
#define E1000_DEV_ID_DH89XXCC_SERDES            0x043A
#define E1000_DEV_ID_DH89XXCC_BACKPLANE         0x043C
#define E1000_DEV_ID_DH89XXCC_SFP               0x0440

RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82576)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82576_FIBER)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82576_SERDES)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82576_QUAD_COPPER)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82576_QUAD_COPPER_ET2)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82576_NS)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82576_NS_SERDES)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82576_SERDES_QUAD)

RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82575EB_COPPER)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82575EB_FIBER_SERDES)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82575GB_QUAD_COPPER)

RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82580_COPPER)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82580_FIBER)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82580_SERDES)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82580_SGMII)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82580_COPPER_DUAL)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82580_QUAD_FIBER)

RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I350_COPPER)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I350_FIBER)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I350_SERDES)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I350_SGMII)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I350_DA4)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I210_COPPER)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I210_COPPER_OEM1)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I210_COPPER_IT)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I210_FIBER)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I210_SERDES)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I210_SGMII)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I211_COPPER)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I354_BACKPLANE_1GBPS)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I354_SGMII)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I354_BACKPLANE_2_5GBPS)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_DH89XXCC_SGMII)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_DH89XXCC_SERDES)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_DH89XXCC_BACKPLANE)
RTE_PCI_DEV_ID_DECL_IGB(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_DH89XXCC_SFP)

/****************** Virtual IGB devices from e1000_hw.h ******************/

#define E1000_DEV_ID_82576_VF                   0x10CA
#define E1000_DEV_ID_82576_VF_HV                0x152D
#define E1000_DEV_ID_I350_VF                    0x1520
#define E1000_DEV_ID_I350_VF_HV                 0x152F

RTE_PCI_DEV_ID_DECL_IGBVF(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82576_VF)
RTE_PCI_DEV_ID_DECL_IGBVF(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_82576_VF_HV)
RTE_PCI_DEV_ID_DECL_IGBVF(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I350_VF)
RTE_PCI_DEV_ID_DECL_IGBVF(PCI_VENDOR_ID_INTEL, E1000_DEV_ID_I350_VF_HV)

/*
 * Undef all RTE_PCI_DEV_ID_DECL_* here.
 */
#undef RTE_PCI_DEV_ID_DECL_IGB
#undef RTE_PCI_DEV_ID_DECL_IGBVF
