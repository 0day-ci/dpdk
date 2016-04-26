#    Copyright (c) 2016 QLogic Corporation.
#    All rights reserved.
#    www.qlogic.com
#
#    See LICENSE.qede_pmd for copyright and licensing details.

include $(RTE_SDK)/mk/rte.vars.mk

#
# library name
#
LIB = librte_pmd_qede.a

CFLAGS += -O3
CFLAGS += $(WERROR_FLAGS)

EXPORT_MAP := rte_pmd_qede_version.map

LIBABIVER := 1

#
# OS
#
OS_TYPE := $(shell uname -s)

#
# CLANG VERSION
#
IS_CLANG_GT_362 := $(shell \
			CLANG_MAJOR=`echo | clang -dM -E - 2>/dev/null | grep clang_major | cut -d" " -f3`; \
			CLANG_MINOR=`echo | clang -dM -E - 2>/dev/null | grep clang_minor | cut -d" " -f3`; \
			CLANG_PATCH=`echo | clang -dM -E - 2>/dev/null | grep clang_patch | cut -d" " -f3`; \
			if [ "0$$CLANG_MAJOR" -gt "03" ]; then \
				echo 1; \
			elif [ "0$$CLANG_MAJOR" -eq "03" -a "0$$CLANG_MINOR" -gt "06" ]; then \
				echo 1; \
			elif [ "0$$CLANG_MAJOR" -eq "03" -a "0$$CLANG_MINOR" -eq "06" -a "0$$CLANG_PATCH" -gt "02" ]; then \
				echo 1; \
			fi)

#
# CFLAGS
#
CFLAGS_BASE_DRIVER = -Wno-unused-parameter
CFLAGS_BASE_DRIVER += -Wno-unused-value
CFLAGS_BASE_DRIVER += -Wno-sign-compare
CFLAGS_BASE_DRIVER += -Wno-missing-prototypes
CFLAGS_BASE_DRIVER += -Wno-cast-qual
CFLAGS_BASE_DRIVER += -Wno-unused-function
CFLAGS_BASE_DRIVER += -Wno-unused-variable
CFLAGS_BASE_DRIVER += -Wno-strict-aliasing
CFLAGS_BASE_DRIVER += -Wno-missing-prototypes
CFLAGS_BASE_DRIVER += -Wno-format-nonliteral
ifeq ($(OS_TYPE),Linux)
ifeq ($(IS_CLANG_GT_362),1)
CFLAGS_BASE_DRIVER += -Wno-shift-negative-value # Support added after clang 3.6
else
CFLAGS_BASE_DRIVER += -Wno-shift-sign-overflow
endif
endif

ifneq (,$(filter gcc gcc48,$(CC)))
CFLAGS_BASE_DRIVER += -Wno-unused-but-set-variable
CFLAGS_BASE_DRIVER += -Wno-missing-declarations
CFLAGS_BASE_DRIVER += -Wno-maybe-uninitialized
CFLAGS_BASE_DRIVER += -Wno-strict-prototypes
else ifeq ($(CC), clang)
CFLAGS_BASE_DRIVER += -Wno-format-extra-args
CFLAGS_BASE_DRIVER += -Wno-visibility
CFLAGS_BASE_DRIVER += -Wno-empty-body
CFLAGS_BASE_DRIVER += -Wno-invalid-source-encoding
CFLAGS_BASE_DRIVER += -Wno-sometimes-uninitialized
CFLAGS_BASE_DRIVER += -Wno-pointer-bool-conversion
else
#icc flags
endif

#
# Add extra flags for base ecore driver files
# to disable warnings in them
#
#
BASE_DRIVER_OBJS=$(patsubst %.c,%.o,$(notdir $(wildcard $(SRCDIR)/base/*.c)))
$(foreach obj, $(BASE_DRIVER_OBJS), $(eval CFLAGS+=$(CFLAGS_BASE_DRIVER)))

#
# all source are stored in SRCS-y
#
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += base/ecore_dev.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += base/ecore_hw.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += base/ecore_cxt.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += base/ecore_sp_commands.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += base/ecore_init_fw_funcs.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += base/ecore_spq.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += base/ecore_init_ops.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += base/ecore_mcp.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += base/ecore_int.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += base/bcm_osal.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += qede_ethdev.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += qede_main.c
SRCS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += qede_rxtx.c

# dependent libs:
DEPDIRS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += lib/librte_eal lib/librte_ether
DEPDIRS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += lib/librte_mempool lib/librte_mbuf
DEPDIRS-$(CONFIG_RTE_LIBRTE_QEDE_PMD) += lib/librte_net lib/librte_malloc

include $(RTE_SDK)/mk/rte.lib.mk
