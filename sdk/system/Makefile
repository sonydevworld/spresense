############################################################################
# system/Makefile
#
#   Copyright 2020 Sony Semiconductor Solutions Corporation
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
# 3. Neither the name of Sony Semiconductor Solutions Corporation nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written
#    permission.
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

include $(APPDIR)/Make.defs
include $(SDKDIR)/Make.defs

MODULEDIRS := $(dir $(wildcard $(SDKDIR)$(DELIM)system$(DELIM)*$(DELIM)Make.defs))
CLEANDIRS  := $(dir $(wildcard $(SDKDIR)$(DELIM)system$(DELIM)*$(DELIM)Makefile))

define SDIR_template
$(1)_$(2):
	+$(Q) $(MAKE) -C $(1) $(2) TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" SDKDIR="$(SDKDIR)"

endef

# CONFIGURED_APPS is the application directories that should be built in
#   the current configuration.

CONFIGURED_APPS :=

define Add_Module
  include $(1)Make.defs
endef

$(foreach SDIR, $(MODULEDIRS), $(eval $(call Add_Module,$(SDIR))))

all: $(foreach SDIR,$(CONFIGURED_APPS),$(SDIR)_all)

$(foreach SDIR, $(CONFIGURED_APPS), $(eval $(call SDIR_template,$(SDIR),all)))
$(foreach SDIR, $(CONFIGURED_APPS), $(eval $(call SDIR_template,$(SDIR),context)))
$(foreach SDIR, $(CONFIGURED_APPS), $(eval $(call SDIR_template,$(SDIR),depend)))
$(foreach SDIR, $(CLEANDIRS), $(eval $(call SDIR_template,$(SDIR),clean)))
$(foreach SDIR, $(CLEANDIRS), $(eval $(call SDIR_template,$(SDIR),distclean)))

context: $(foreach SDIR,$(CONFIGURED_APPS),$(SDIR)_context)

depend: $(foreach SDIR,$(CONFIGURED_APPS),$(SDIR)_depend)

clean: $(foreach SDIR,$(CLEANDIRS),$(SDIR)_clean)

distclean: $(foreach SDIR,$(CLEANDIRS),$(SDIR)_distclean)
	$(call DELFILE, Kconfig)

install:

preconfig: Kconfig

Kconfig:
	$(Q) $(MKKCONFIG) -m "System tools"
