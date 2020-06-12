############################################################################
# bsp/scripts/Board.mk
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
#   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#           Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))
CXXOBJS = $(CXXSRCS:.cxx=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

SCHEDSRCDIR = $(TOPDIR)$(DELIM)sched
ARCHSRCDIR = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src
BSPSRCDIR = $(SDKDIR)$(DELIM)bsp$(DELIM)src

ifneq ($(CONFIG_ARCH_FAMILY),)
  ARCH_FAMILY = $(patsubst "%",%,$(CONFIG_ARCH_FAMILY))
endif

ifeq ($(WINTOOL),y)
  CFLAGS += -I "${shell cygpath -w $(SCHEDSRCDIR)}"
  CFLAGS += -I "${shell cygpath -w $(ARCHSRCDIR)$(DELIM)chip}"
  CFLAGS += -I "${shell cygpath -w $(ARCHSRCDIR)$(DELIM)common}"
ifneq ($(ARCH_FAMILY),)
  CFLAGS += -I "${shell cygpath -w $(ARCHSRCDIR)$(DELIM)$(ARCH_FAMILY)}"
endif
  CFLAGS += -I "${shell cygpath -w $(BSPSRCDIR)}"
else
  CFLAGS += -I$(SCHEDSRCDIR)
  CFLAGS += -I$(ARCHSRCDIR)$(DELIM)chip
  CFLAGS += -I$(ARCHSRCDIR)$(DELIM)common
ifneq ($(ARCH_FAMILY),)
  CFLAGS += -I$(ARCHSRCDIR)$(DELIM)$(ARCH_FAMILY)
endif
  CFLAGS += -I$(BSPSRCDIR)
endif

all: libboard$(LIBEXT)

$(ASRCS) $(HEAD_ASRC): %$(ASMEXT): %.S
ifeq ($(WINTOOL),y)
	$(Q) $(CPP) $(CPPFLAGS) `cygpath -w $<` -o $@.tmp
else
	$(Q) $(CPP) $(CPPFLAGS) $< -o $@.tmp
endif
	$(Q) cat $@.tmp | sed -e "s/^#/;/g" > $@
	$(Q) rm $@.tmp

$(AOBJS): %$(OBJEXT): %$(ASMEXT)
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS) $(LINKOBJS): %$(OBJEXT): %.cxx
	$(call COMPILEXX, $<, $@)

libboard$(LIBEXT): $(OBJS) $(CXXOBJS)
	$(Q) $(AR) $@
ifneq ($(OBJS),)
	$(call ARCHIVE, $@, $(OBJS) $(CXXOBJS))
endif

.depend: Makefile $(SRCS) $(CXXSRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
ifneq ($(CXXSRCS),)
	$(Q) $(MKDEP) $(DEPPATH) "$(CXX)" -- $(CXXFLAGS) -- $(CXXSRCS) >>Make.dep
endif
	$(Q) touch $@

depend: .depend

ifneq ($(BOARD_CONTEXT),y)
context:
endif

clean:
	$(call DELFILE, libboard$(LIBEXT))
	$(call CLEAN)
	$(EXTRA_CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(EXTRA_DISTCLEAN)

-include Make.dep
