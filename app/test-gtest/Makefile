#   BSD LICENSE
#   BSD LICENSE
#
#   Copyright(c) 2016 Intel Corporation. All rights reserved.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     * Neither the name of Intel Corporation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


ifneq ($(MAKECMDGOALS),clean)
ifeq ($(GTEST_DIR),)
$(error "Please define GTEST_DIR environment variable")
endif

ifeq ($(GMOCK_DIR),)
$(error "Please define GMOCK_DIR environment variable")
endif
endif

include $(RTE_SDK)/mk/rte.vars.mk

# binary name
APP = test-gtest

# all source are stored in SRCS-y
SRCS-y := main.cpp
SRCS-y += test_mempool.cpp

CFLAGS += -O0
CFLAGS += $(WERROR_FLAGS)

CXXFLAGS += -std=c++11
CXXFLAGS += -I$(GTEST_DIR)/include 
CXXFLAGS += -I$(GMOCK_DIR)/include

LDFLAGS += -lstdc++
LDFLAGS += -L$(GMOCK_DIR) -lgmock

# this application needs libraries first
DEPDIRS-y += lib drivers

include $(RTE_SDK)/mk/rte.app.mk

