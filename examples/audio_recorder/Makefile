############################################################################
# audio_recorder/Makefile
#
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

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs
include $(APPDIR)/Make.defs

# Audio application info

CONFIG_EXAMPLES_AUDIO_RECORDER_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_EXAMPLES_AUDIO_RECORDER_STACKSIZE ?= 2048

APPNAME   = recorder
PRIORITY  = $(CONFIG_EXAMPLES_AUDIO_RECORDER_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_AUDIO_RECORDER_STACKSIZE)

# Audio Example

MAINSRC = audio_recorder_main.cxx

# Audio Example paths

AUDIODIR = $(SDKDIR)$(DELIM)modules$(DELIM)audio

# Audio Example flags

ifeq ($(WINTOOL),y)
CXXFLAGS += -I "${shell cygpath -w $(AUDIODIR)}"
ifeq ($(CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC),y)
CXXFLAGS += -I "${shell cygpath -w worker/userproc/include}"
endif
else
CXXFLAGS += -I $(AUDIODIR)
ifeq ($(CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC),y)
CXXFLAGS += -I worker/userproc/include
endif
endif

CXXFLAGS += -D_POSIX
CXXFLAGS += -DUSE_MEMMGR_FENCE
CXXFLAGS += -DATTENTION_USE_FILENAME_LINE

CONFIG_EXAMPLES_AUDIO_RECORDER_PROGNAME ?= audio_recorder$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_AUDIO_RECORDER_PROGNAME)

ifeq ($(CONFIG_EXAMPLES_AUDIO_RECORDER_USEPREPROC),y)

# Instead of including "Applicaion.mk", write "make rules" here.
# (Cannot use "Application.mk" because targe "clean" will duplicate and be ignored.)

CXXEXT ?= .cxx

CXXOBJS = $(CXXSRCS:$(CXXEXT)=$(OBJEXT))
MAINOBJ = $(MAINSRC:$(CXXEXT)=$(OBJEXT))

SRCS = $(CXXSRCS) $(MAINSRC)
OBJS = $(CXXOBJS)

ifneq ($(CONFIG_BUILD_KERNEL),y)
  OBJS += $(MAINOBJ)
endif

ifeq ($(WINTOOL),y)
  BIN = "${shell cygpath -w $(APPDIR)$(DELIM)libexamples$(LIBEXT)}"
  INSTALL_DIR = "${shell cygpath -w $(BIN_DIR)}"
else
  BIN = $(APPDIR)$(DELIM)libexamples$(LIBEXT)
  INSTALL_DIR = $(BIN_DIR)
endif

ROOTDEPPATH = --dep-path .

VPATH = worker

all: .built
.PHONY: .built worker_build clean

$(CXXOBJS): %$(OBJEXT): %$(CXXEXT)
	$(call COMPILEXX, $<, $@)

$(MAINOBJ): %$(OBJEXT): %$(CXXEXT)
	$(call COMPILEXX, $<, $@)

worker_build:
	$(Q) $(MAKE) -C worker TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

.built: $(OBJS) worker_build

ifeq ($(CONFIG_BUILD_KERNEL),y)
$(BIN_DIR)$(DELIM)$(PROGNAME): $(OBJS) $(MAINOBJ)
	@echo "LD: $(PROGNAME)"
	$(Q) $(LD) $(LDELFFLAGS) $(LDLIBPATH) -o $(INSTALL_DIR)$(DELIM)$(PROGNAME) $(ARCHCRT0OBJ) $(MAINOBJ) $(LDLIBS)
	$(Q) $(NM) -u $(INSTALL_DIR)$(DELIM)$(PROGNAME)

install: $(BIN_DIR)$(DELIM)$(PROGNAME)
else
install: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
endif

ifeq ($(CONFIG_NSH_BUILTIN_APPS),y)
$(BUILTIN_REGISTRY)$(DELIM)$(APPNAME)_main.bdat: $(DEPCONFIG) Makefile
	$(call REGISTER,$(APPNAME),$(PRIORITY),$(STACKSIZE),$(APPNAME)_main)

context: $(BUILTIN_REGISTRY)$(DELIM)$(APPNAME)_main.bdat
else
context:
endif

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(ROOTDEPPATH) "$(CXX)" -- $(CXXFLAGS) -- $(SRCS) >Make.dep
	$(Q) $(MAKE) -C worker TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) depend
	$(Q) touch $@

depend: .depend

clean_worker:
	@$(MAKE) -C worker TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

clean: clean_worker
	$(call DELFILE, .worker)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
.PHONY: preconfig
preconfig:

else

include $(APPDIR)/Application.mk

endif

