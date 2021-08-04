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

-include Make.defs

TOPDIR ?= ../nuttx
DELIM ?= $(strip /)

# Check if the system has been configured

ifeq ($(wildcard $(TOPDIR)/.config),)
.DEFAULT:
	@echo
	@echo "SDK has not been configured!"
	@echo "To configure the project:"
	@echo "  tools/config.py <config> [<config> ...]"
	@echo "For a list of available configurations:"
	@echo "  tools/config.py -l"
	@echo

else

-include $(TOPDIR)/Make.defs

# XXX: Sorry, this make file only for Linux/Cygwin

# Setup for Windows vs Linux/Cygwin/OSX environments

ifeq ($(WINTOOL),)
ifeq ("$(shell uname -o 2>/dev/null)","Cygwin")
    WINTOOL = y
endif
endif

ifeq ($(WINTOOL),y)
  SDKDIR := $(shell pwd | cygpath -m -f - | sed -e 's/ /\\ /g')
else
  SDKDIR := $(shell pwd | sed -e 's/ /\\ /g')
endif

ifeq ($(V),1)
export Q :=
else
ifeq ($(V),2)
export Q :=
else
export Q := @
endif
endif

define MAKE_template
	+$(Q) $(MAKE) -C $(1) $(2) TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" SDKDIR="$(SDKDIR)"

endef

define SDIR_template
$(1)_$(2):
	+$(Q) $(MAKE) -C $(1) $(2) TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" SDKDIR="$(SDKDIR)"

endef

# Export tool definitions

MKEXPORT = tools/mkexport.sh
MKEXPORT_ARGS = -w$(WINTOOL) -t "$(TOPDIR)" -s "$(SDKDIR)"

PHONY :=

NUTTXNAME = nuttx
BIN       = $(NUTTXNAME)$(EXEEXT)
SPK       = $(NUTTXNAME).spk

KDEFINE = ${shell $(TOPDIR)/tools/define.sh "$(CC)" __KERNEL__}

# If TOPDIR's origin is command line, 'make' called from NuttX build system.
# If not, 'make' called directly from user, then bypass to NuttX build.

ifeq ($(MAKELEVEL),0)
all:
	$(Q) TOPDIR="$(TOPDIR)" ./tools/mkversion.sh
	@ $(MAKE) -C ../nuttx && cp $(TOPDIR)/$(BIN) $(TOPDIR)/$(SPK) $(TOPDIR)/*.map .
else
all: .built
endif

SDKSUBDIRS = modules system

EXTSUBDIRS = $(dir $(wildcard ../*/.sdksubdir)) $(SPRESENSE_HOME)

TARGETDIRS = $(SDKSUBDIRS) $(EXTSUBDIRS)
SDKCLEANDIRS = $(EXTSUBDIRS)

$(foreach SDIR, $(TARGETDIRS), $(eval $(call SDIR_template,$(SDIR),all)))
$(foreach SDIR, $(TARGETDIRS), $(eval $(call SDIR_template,$(SDIR),install)))
$(foreach SDIR, $(TARGETDIRS), $(eval $(call SDIR_template,$(SDIR),preconfig)))
$(foreach SDIR, $(TARGETDIRS), $(eval $(call SDIR_template,$(SDIR),context)))
$(foreach SDIR, $(TARGETDIRS), $(eval $(call SDIR_template,$(SDIR),depend)))
$(foreach SDIR, $(TARGETDIRS), $(eval $(call SDIR_template,$(SDIR),clean)))
$(foreach SDIR, $(TARGETDIRS), $(eval $(call SDIR_template,$(SDIR),distclean)))

ifeq ($(MAKELEVEL),0)
MKSDKEXPORT = $(SDKDIR)/tools/mksdkexport.sh
export:
	$(Q) $(MAKE) -C $(TOPDIR) export && mv $(TOPDIR)/nuttx-export-*.zip nuttx-export.zip
exportsdk: export
	$(Q) $(MKSDKEXPORT) -i nuttx-export.zip -o sdk-export.tar.gz
	$(Q) rm -rf nuttx-export.zip
PHONY += export
PHONY += exportsdk
endif

dirlinks:
PHONY += dirlinks

$(TOPDIR)/tools/cxd56:
	$(Q) mkdir -p $@

ifeq ($(CONFIG_HOST_LINUX),y)
  MKSPK_SRC = $(SDKDIR)/tools/linux/mkspk
endif
ifeq ($(CONFIG_HOST_MACOS),y)
  MKSPK_SRC = $(SDKDIR)/tools/macos/mkspk
endif
ifeq ($(CONFIG_HOST_WINDOWS),y)
  MKSPK_SRC = $(SDKDIR)/tools/windows/mkspk$(HOSTEXEEXT)
endif
MKSPK = $(TOPDIR)/tools/cxd56/mkspk$(HOSTEXEEXT)
$(MKSPK): $(TOPDIR)/tools/cxd56
	$(call COPYFILE,$(MKSPK_SRC),$@)

context: dirlinks $(TOPDIR)/tools/cxd56/mkspk$(HOSTEXEEXT) $(foreach SDIR,$(TARGETDIRS),$(SDIR)_context)
PHONY += context

clean_context:
PHONY += clean_context

depend: context $(foreach SDIR,$(TARGETDIRS),$(SDIR)_depend)
PHONY += depend

#
# Build rules
#

.built: $(foreach SDIR,$(TARGETDIRS),$(SDIR)_all)
	@ touch $@

subdir_clean: $(foreach SDIR,$(TARGETDIRS),$(SDIR)_clean)
	$(Q) for dir in $(SDKCLEANDIRS); do \
		find $$dir -name "Make.dep" -print | xargs rm -f; \
		find $$dir -name ".depend" -print | xargs rm -f; \
	done
PHONY += subdir_clean

depend_clean:
	$(Q)find $(SDKDIR) -name "Make.dep" -print | xargs rm -f
	$(Q)find $(SDKDIR) -name ".depend" -print | xargs rm -f
PHONY += depend_clean

ifeq ($(MAKELEVEL),0)
clean:
	$(Q) $(MAKE) -C $(TOPDIR) clean

distclean:
	$(Q) $(MAKE) -C $(TOPDIR) distclean CONFIG_APPS_DIR="$(SDKDIR)/apps"
else
clean: subdir_clean depend_clean
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(SPK))
	$(call DELFILE, System.map)
	$(call DELFILE, nuttx.map)
	$(call DELFILE, .built)
	$(call DELFILE, Kconfig)
PHONY += clean

subdir_distclean: $(foreach SDIR,$(TARGETDIRS),$(SDIR)_distclean)
PHONY += subdir_distclean

distclean: subdir_distclean clean_context

endif

install: $(foreach SDIR, $(TARGETDIRS), $(SDIR)_install)

# Bypass config targets

config oldconfig olddefconfig menuconfig nconfig qconfig gconfig:
	$(Q) $(MAKE) -C $(TOPDIR) $@

PHONY += config oldconfig olddefconfig menuconfig nconfig qconfig gconfig

savedefconfig:
	$(Q) $(MAKE) -C $(TOPDIR) $@
	$(Q) mv $(TOPDIR)/defconfig .
PHONY += savedefconfig

preconfig: $(foreach SDIR,$(TARGETDIRS),$(SDIR)_preconfig)
	$(Q) python3 tools/mkkconfig.py -m "Spresense SDK" $(TARGETDIRS)

archive:

register:

endif # ifeq ($(wildcard $(TOPDIR)/.config),)

help:
	@echo  ''
	@echo  'SDK targets:'
	@echo  '  (all)           - Build sdk (if tools/config.py is done in advance)'
	@echo  '  clean           - Remove most generated files but keep the sdk config'
	@echo  '  distclean       - Remove all generated files + sdk config'
	@echo  ''
	@echo  'Configuration targets:'
	@echo  '  config'
	@echo  '  oldconfig'
	@echo  '  olddefconfig'
	@echo  '  menuconfig'
	@echo  '  nconfig'
	@echo  '  qconfig'
	@echo  '  gconfig'
	@echo  '  savedefconfig'
	@echo  ''
	@echo  '    There are various configuration targets, but instead of these targets,'
	@echo  '    Execute configuration by using "./tools/config.py" tool.'
	@echo  '    For more details refer to help of "./tools/config.py -h".'
	@echo  ''
	@echo  'Make options:'
	@echo  '  make V=0|1 [targets] 0 => quiet build (default), 1 => verbose build'
	@echo  ''

.PHONY: $(PHONY)
