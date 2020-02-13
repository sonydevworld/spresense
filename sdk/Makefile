############################################################################
# Makefile
#
#   Copyright (C) 2007-2012, 2014-2015 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#   Copyright 2018 Sony Semiconductor Solutions Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# XXX: Sorry, this make file only for Linux/Cygwin

# Path to NuttX kernel source tree, it must be relative path.

NUTTXDIR ?= ../nuttx

# Setup for Windows vs Linux/Cygwin/OSX environments

ifeq ("$(shell uname -o 2>/dev/null)","Cygwin")
    WINTOOL = y
endif

ifeq ($(WINTOOL),y)
  SDKDIR := $(shell pwd | cygpath -m -f - | sed -e 's/ /\\ /g')
  TOPDIR := $(shell pwd | cygpath -m -f - | sed -e 's/ /\\ /g')/$(NUTTXDIR)
else
  SDKDIR := $(shell pwd | sed -e 's/ /\\ /g')
  TOPDIR := $(shell pwd | sed -e 's/ /\\ /g')/$(NUTTXDIR)
endif

-include $(TOPDIR)/.config
-include $(SDKDIR)/.config
include $(TOPDIR)/tools/Config.mk
-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

ifeq ($(V),1)
export Q :=
else
ifeq ($(V),2)
export Q :=
else
export Q := @
endif
endif

BOARD_DIR = $(SDKDIR)$(DELIM)bsp$(DELIM)board$(DELIM)$(CONFIG_BOARD_NAME)
BOARD_COMMON_DIR = $(SDKDIR)$(DELIM)bsp$(DELIM)board$(DELIM)common

# Dummy. It needs libapps.a to be a member of EXPORTLIBS.

APPDIR := ../apps

ifeq ($(CONFIG_BUILD_PROTECTED),y)
include $(TOPDIR)/tools/ProtectedLibs.mk
else
ifeq ($(CONFIG_BUILD_KERNEL),y)
include $(TOPDIR)/tools/KernelLibs.mk
else
include $(TOPDIR)/tools/FlatLibs.mk
endif
endif

# Default tools

ifeq ($(DIRLINK),)
DIRLINK = $(TOPDIR)/tools/link.sh
DIRUNLINK = $(TOPDIR)/tools/unlink.sh
endif

# Export tool definitions

MKEXPORT = tools/mkexport.sh
MKEXPORT_ARGS = -w$(WINTOOL) -t "$(TOPDIR)" -s "$(SDKDIR)"

PHONY :=

LDSTARTGROUP ?= --start-group
LDENDGROUP ?= --end-group

NUTTXNAME = nuttx
BIN       = $(NUTTXNAME)$(EXEEXT)
SPK       = $(NUTTXNAME).spk

KDEFINE = ${shell $(TOPDIR)/tools/define.sh "$(CC)" __KERNEL__}

# Process architecture and board-specific directories

ARCH_DIR = arch/$(CONFIG_ARCH)
ARCH_SRC = $(ARCH_DIR)/src
ARCH_INC = $(ARCH_DIR)/include

all: $(BIN)

# buildkernel
#
# Separated build target for NuttX kernel libraries. If KERNCONF specified by
# option, then reconfigure and build them all.
# After install them, we need to replace nsh builtin list to ours, so
# remove builtin_list.o from libapps.a if exists.
# If nsh disabled at NuttX configuration, then our builtin_list will be removed
# automatically.

buildkernel:
	$(Q) TOPDIR=$(TOPDIR) $(SDKDIR)/tools/mkversion.sh
ifneq ($(KERNCONF),)
	$(Q) $(SDKDIR)/tools/config.py --kernel $(KERNCONF)
endif
	$(Q) $(MAKE) $(MAKEOPTS) -C $(TOPDIR) pass2deps
PHONY += buildkernel

# installkernel
#
# installkernel target copies NuttX kernel libraries. And remove specific object files from
# copied archives. It is replacement for use SDK specific code.
# Additionally, remove links to board directory for avoid conflict the same name header
# inclusion.

installkernel:
	$(Q) install $(sort $(patsubst %,$(TOPDIR)/%,$(EXPORTLIBS))) lib
	$(Q) if [ -f lib/libarch.a ]; then \
		$(ARCROSSDEV)ar d lib/libarch.a cxd56_allocateheap.o; \
		$(ARCROSSDEV)ar d lib/libarch.a cxd56_serial.o; \
		$(ARCROSSDEV)ar d lib/libarch.a cxd56_uart.o; \
		$(ARCROSSDEV)ar d lib/libarch.a cxd56_timerisr.o; \
		$(ARCROSSDEV)ar d lib/libarch.a cxd56_idle.o; \
		$(ARCROSSDEV)ar d lib/libarch.a cxd56_start.o; \
		$(ARCROSSDEV)ar d lib/libarch.a up_udelay.o; \
		$(ARCROSSDEV)ar d lib/libarch.a up_mdelay.o; \
	fi
PHONY += installkernel

SDKMODDIRS =
CONTEXTDIRS =
EXTLIBS =
EXTSUBDIRS =

# Include library build recipes

include LibTargets.mk

SDKDEPDIRS = $(SDKMODDIRS) $(EXTSUBDIRS)
SDKCLEANDIRS += $(EXTSUBDIRS) lib
CONTEXTDIRS += $(EXTSUBDIRS)

ifeq ($(WINTOOL),y)
  LIBPATHS += -L"${shell cygpath -w "$(SDKDIR)/lib"}"
else
  LIBPATHS += -L"$(SDKDIR)/lib"
endif

LINKLIBS = $(patsubst lib/%,%,$(SDKLIBS) $(EXTLIBS) $(NUTTXLIBS))
LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))

$(BIN): depend sdklibs
	$(Q) $(MAKE) -C bsp TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" LINKLIBS="$(LINKLIBS)" EXTRADEFINES=$(KDEFINE) $(BIN)
	@echo "SPK: $(BIN).spk"
	$(Q) if [ -x $(MKSPK) ]; then \
		$(MKSPK) -c 2 $(BIN) nuttx $(BIN).spk >/dev/null ; \
	else \
		echo mkspk tool not found. ; \
	fi

sdklibs: $(EXPORTLIBS) $(SDKLIBS) $(EXTLIBS)

$(EXPORTLIBS): installkernel

export: $(BIN)
	$(Q) MAKE=$(MAKE) $(MKEXPORT) $(MKEXPORT_ARGS) -l "$(EXPORTLIBS) $(SDKLIBS) $(EXTLIBS)"

bsp/include/arch/board: .config
	@echo "LN: bsp/include/arch/board to $(BOARD_DIR)/include"
	$(Q) $(DIRLINK) $(BOARD_DIR)/include $@

bsp/include/arch/board/common: bsp/include/arch/board .config
	@echo "LN: bsp/include/arch/board/common to $(BOARD_COMMON_DIR)/include"
	$(Q) $(DIRLINK) $(BOARD_COMMON_DIR)/include $@

dirlinks: bsp/include/arch/board bsp/include/arch/board/common
PHONY += dirlinks

context: check_context bsp/include/sdk/config.h dirlinks
	$(Q) $(DIRUNLINK) $(TOPDIR)/include/arch/board
	$(Q) $(DIRUNLINK) $(TOPDIR)/$(ARCH_SRC)/board
	$(Q) for dir in $(CONTEXTDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" context; \
	done
PHONY += context

# check_context
#
# This target checks if NuttX has been configured.  NuttX is configured using
# the script tools/configure.sh.  That script will install certain files in
# the top-level NuttX build directory.  This target verifies that those
# configuration files have been installed and that NuttX is ready to be built.

check_context:
	$(Q) if [ ! -e $(TOPDIR)/.config -o ! -e $(TOPDIR)/Make.defs ]; then \
		echo "" ; echo "Nuttx has not been configured:" ; \
		echo "  tools/config.py -k <target>" ; echo "" ; \
		exit 1 ; \
	fi
	$(Q) if [ ! -e $(SDKDIR)/.config ]; then \
		echo "" ; echo "SDK has not been configured:" ; \
		echo "  tools/config.py <target>" ; echo "" ; \
		exit 1 ; \
	fi
	$(Q) if [ ! -e $(TOPDIR)/include/nuttx/config.h ]; then \
		echo ""; echo "NuttX has not been built:" ; \
		echo "  make buildkernel" ; echo "" ; \
		exit 1; \
	fi

clean_context:
	$(call DELFILE, bsp/include/sdk/config.h)
	$(Q) $(DIRUNLINK) bsp/include/arch/board/common
	$(Q) $(DIRUNLINK) bsp/include/arch/board

PHONY += clean_context

depend: context
	$(Q) for dir in $(SDKDEPDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" EXTRADEFINES=$(KDEFINE) depend; \
	done
PHONY += depend

# clean_kernel
#
#

cleankernel:
	$(Q) $(MAKE) -C $(TOPDIR) clean
PHONY += cleankernel

distcleankernel: distclean
	$(Q) $(MAKE) -C $(TOPDIR) distclean
PHONY += distcleankernel

subdir_clean:
	$(Q) for dir in $(SDKCLEANDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" clean; \
		find $$dir -name "Make.dep" -delete; \
		find $$dir -name ".depend" -delete; \
	done
PHONY += subdir_clean

depend_clean:
	$(Q)find $(SDKDIR) -name "Make.dep" -delete
	$(Q)find $(SDKDIR) -name ".depend" -delete
PHONY += depend_clean

clean: subdir_clean depend_clean
	$(call DELFILE, .kconfig.tmp)
	$(call DELFILE, bsp/include/sdk/config.h)
	$(call DELFILE, bsp/include/sdk/version.h)
	$(call DELFILE, $(SDKLIBS))
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(SPK))
	$(call DELFILE, System.map)
	$(call DELFILE, nuttx.map)
PHONY += clean

subdir_distclean:
	$(Q) for dir in $(SDKCLEANDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" distclean; \
	done
PHONY += subdir_distclean

distclean: clean subdir_distclean clean_context
	$(call DELFILE, .config)
	$(call DELFILE, .config.old)

bsp/include/sdk/config.h: $(SDKDIR)/.config tools/mkconfig.py
	$(Q) tools/mkconfig.py > $@

bsp/include/sdk/version.h: $(SDKDIR)/.version
# TBD

# For NuttX configuration

configkernel oldconfigkernel olddefconfigkernel menuconfigkernel qconfigkernel gconfigkernel:
	$(Q) $(MAKE) -C $(TOPDIR) $(patsubst %kernel,%,$@)
PHONY += configkernel oldconfigkernel olddefconfigkernel menuconfigkernel qconfigkernel gconfigkernel

# config targets
#
#

config: .kconfig.tmp clean_context
	$(Q) SDKDIR=$(SDKDIR) kconfig-conf $<
PHONY += config

oldconfig: .kconfig.tmp clean_context
	$(Q) SDKDIR=$(SDKDIR) kconfig-conf --oldconfig $<
PHONY += oldconfig

olddefconfig: .kconfig.tmp clean_context
	$(Q) SDKDIR=$(SDKDIR) kconfig-conf --olddefconfig $<
PHONY += olddefconfig

menuconfig: .kconfig.tmp clean_context
	$(Q) SDKDIR=$(SDKDIR) kconfig-mconf $<
PHONY += menuconfig

qconfig: .kconfig.tmp clean_context
	$(Q) SDKDIR=$(SDKDIR) kconfig-qconf $<
PHONY += qconfig

gconfig: .kconfig.tmp clean_context
	$(Q) SDKDIR=$(SDKDIR) kconfig-gconf $<
PHONY += gconfig

savedefconfig: .kconfig.tmp clean_context
	$(Q) SDKDIR=$(SDKDIR) kconfig-conf --savedefconfig defconfig $<
PHONY += savedefconfig

.kconfig.tmp: preconfig ext_preconfig
	$(Q) tools/gentmpkconfig.py $@

preconfig:
	$(Q) $(MAKE) -C system TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" preconfig

ext_preconfig:
ifneq ($(EXTSUBDIRS),)
	$(Q) for dir in $(EXTSUBDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" preconfig; \
	done
endif

ext_clean:
ifneq ($(EXTSUBDIRS),)
	$(Q) for dir in $(EXTSUBDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" clean; \
	done
endif

ext_distclean:
ifneq ($(EXTSUBDIRS),)
	$(Q) for dir in $(EXTSUBDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" distclean; \
	done
endif
PHONY += ext_preconfig ext_clean ext_distclean

help:
	@echo  ''
	@echo  'SDK targets:'
	@echo  '  (all)           - Build sdk (if buildkernel is done in advance)'
	@echo  '  clean           - Remove most generated sdk files but keep the sdk config'
	@echo  '  distclean       - Remove all generated sdk files + sdk config'
	@echo  ''
	@echo  'Kernel targets:'
	@echo  '  buildkernel     - Build kernel'
	@echo  '  cleankernel     - Remove most generated kernel files but keep the kernel config'
	@echo  '  distcleankernel - Remove all generated kernel files + kernel config'
	@echo  ''
	@echo  'Configuration targets:'
	@echo  '  xxxconfig       - Update the sdk config'
	@echo  '  xxxconfigkernel - Update the kernel config'
	@echo  ''
	@echo  '    There are various configuration targets, but instead of these targets,'
	@echo  '    Execute configuration by using "./tools/config.py" tool.'
	@echo  '    For more details refer to help of "./tools/config.py -h".'
	@echo  ''
	@echo  'Make options:'
	@echo  '  make V=0|1 [targets] 0 => quiet build (default), 1 => verbose build'
	@echo  ''

.PHONY: $(PHONY)
