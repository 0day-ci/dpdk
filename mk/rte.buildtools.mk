#   BSD LICENSE
#
#   Copyright(c) 2010-2016 Intel Corporation. All rights reserved.
#   Copyright(c) 2014-2015 6WIND S.A.
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

include $(RTE_SDK)/mk/internal/rte.compile-pre.mk
include $(RTE_SDK)/mk/internal/rte.install-pre.mk
include $(RTE_SDK)/mk/internal/rte.clean-pre.mk
include $(RTE_SDK)/mk/internal/rte.build-pre.mk
include $(RTE_SDK)/mk/internal/rte.depdirs-pre.mk

# VPATH contains at least SRCDIR
VPATH += $(SRCDIR)

_BUILD = $(APP)
_INSTALL = $(INSTALL-FILES-y) $(SYMLINK-FILES-y)
_INSTALL += $(RTE_OUTPUT)/buildtools/$(APP) $(RTE_OUTPUT)/buildtools/$(APP).map
POSTINSTALL += target-appinstall
_CLEAN = doclean
POSTCLEAN += target-appclean

.PHONY: all
all: install

.PHONY: install
install: build _postinstall

_postinstall: build

.PHONY: build
build: _postbuild

exe2cmd = $(strip $(call dotfile,$(patsubst %,%.cmd,$(1))))

ifeq ($(LINK_USING_CC),1)
override EXTRA_LDFLAGS := $(call linkerprefix,$(EXTRA_LDFLAGS))
O_TO_EXE = $(CC) $(CFLAGS) $(LDFLAGS_$(@)) \
	-Wl,-Map=$(@).map,--cref -o $@ $(OBJS-y) $(call linkerprefix,$(LDFLAGS)) \
	$(EXTRA_LDFLAGS) $(call linkerprefix,$(LDLIBS))
else
O_TO_EXE = $(LD) $(LDFLAGS) $(LDFLAGS_$(@)) $(EXTRA_LDFLAGS) \
	-Map=$(@).map --cref -o $@ $(OBJS-y) $(LDLIBS)
endif
O_TO_EXE_STR = $(subst ','\'',$(O_TO_EXE)) #'# fix syntax highlight
O_TO_EXE_DISP = $(if $(V),"$(O_TO_EXE_STR)","  LD $(@)")
O_TO_EXE_CMD = "cmd_$@ = $(O_TO_EXE_STR)"
O_TO_EXE_DO = @set -e; \
	echo $(O_TO_EXE_DISP); \
	$(O_TO_EXE) && \
	echo $(O_TO_EXE_CMD) > $(call exe2cmd,$(@))

-include .$(APP).cmd

# path where libraries are retrieved
LDLIBS_PATH := $(subst -Wl$(comma)-L,,$(filter -Wl$(comma)-L%,$(LDLIBS)))
LDLIBS_PATH += $(subst -L,,$(filter -L%,$(LDLIBS)))

# list of .a files that are linked to this application
LDLIBS_NAMES := $(patsubst -l%,lib%.a,$(filter -l%,$(LDLIBS)))
LDLIBS_NAMES += $(patsubst -Wl$(comma)-l%,lib%.a,$(filter -Wl$(comma)-l%,$(LDLIBS)))

# list of found libraries files (useful for deps). If not found, the
# library is silently ignored and dep won't be checked
LDLIBS_FILES := $(wildcard $(foreach dir,$(LDLIBS_PATH),\
	$(addprefix $(dir)/,$(LDLIBS_NAMES))))

#
# Compile executable file if needed
#
$(APP): $(OBJS-y) $(LDLIBS_FILES) $(DEP_$(APP)) $(LDSCRIPT) FORCE
	@[ -d $(dir $@) ] || mkdir -p $(dir $@)
	$(if $(D),\
		@echo -n "$< -> $@ " ; \
		echo -n "file_missing=$(call boolean,$(file_missing)) " ; \
		echo -n "cmdline_changed=$(call boolean,$(call cmdline_changed,$(O_TO_EXE_STR))) " ; \
		echo -n "depfile_missing=$(call boolean,$(depfile_missing)) " ; \
		echo "depfile_newer=$(call boolean,$(depfile_newer)) ")
	$(if $(or \
		$(file_missing),\
		$(call cmdline_changed,$(O_TO_EXE_STR)),\
		$(depfile_missing),\
		$(depfile_newer)),\
		$(O_TO_EXE_DO))

#
# install app in $(RTE_OUTPUT)/app
#
$(RTE_OUTPUT)/buildtools/$(APP): $(APP)
	@echo "  INSTALL-APP $(APP)"
	@[ -d $(RTE_OUTPUT)/buildtools ] || mkdir -p $(RTE_OUTPUT)/buildtools
	$(Q)cp -f $(APP) $(RTE_OUTPUT)/buildtools

#
# install app map file in $(RTE_OUTPUT)/app
#
$(RTE_OUTPUT)/buildtools/$(APP).map: $(APP)
	@echo "  INSTALL-MAP $(APP).map"
	@[ -d $(RTE_OUTPUT)/buildtools ] || mkdir -p $(RTE_OUTPUT)/buildtools
	$(Q)cp -f $(APP).map $(RTE_OUTPUT)/buildtools

#
# Clean all generated files
#
.PHONY: clean
clean: _postclean
	$(Q)rm -f $(_BUILD_TARGETS) $(_INSTALL_TARGETS) $(_CLEAN_TARGETS)

.PHONY: doclean
doclean:
	$(Q)rm -rf $(APP) $(OBJS-all) $(DEPS-all) $(DEPSTMP-all) \
	  $(CMDS-all) $(INSTALL-FILES-all) .$(APP).cmd


include $(RTE_SDK)/mk/internal/rte.compile-post.mk
include $(RTE_SDK)/mk/internal/rte.install-post.mk
include $(RTE_SDK)/mk/internal/rte.clean-post.mk
include $(RTE_SDK)/mk/internal/rte.build-post.mk
include $(RTE_SDK)/mk/internal/rte.depdirs-post.mk

.PHONY: FORCE
FORCE:
