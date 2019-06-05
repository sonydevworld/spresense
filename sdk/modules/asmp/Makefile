############################################################################
# modules/asmp/Makefile
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

-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs
DELIM ?= $(strip /)

VPATH   = supervisor
SUBDIRS =
DEPPATH = --dep-path supervisor

CSRCS  = asmp_init.c
CSRCS += mptask.c mptask_sighandler.c mptask_exec.c mptask_destroy.c
CSRCS += mptask_map.c
CSRCS += mptask_secure.c
CSRCS += mpmq.c
CSRCS += mpshm.c
CSRCS += mpmutex.c

ifeq ($(CONFIG_CXD56_SUBCORE),)
include rawelf/Make.defs
include mm_tile/Make.defs
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

LIB_OBJS = $(AOBJS) $(COBJS)

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libasmp$(LIBEXT)

ARCHSRCDIR = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src
BSPSRCDIR = $(SDKDIR)$(DELIM)bsp$(DELIM)src

ifeq ($(WINTOOL),y)
  CFLAGS += -I "${shell cygpath -w $(ARCHSRCDIR)$(DELIM)chip}"
  CFLAGS += -I "${shell cygpath -w $(ARCHSRCDIR)$(DELIM)common}"
  CFLAGS += -I "${shell cygpath -w $(BSPSRCDIR)}"
ifneq ($(CONFIG_ARCH_FAMILY),)
  CFLAGS += -I "${shell cygpath -w $(ARCHSRCDIR)$(DELIM)$(CONFIG_ARCH_FAMILY)}"
endif
else
  CFLAGS += -I$(ARCHSRCDIR)$(DELIM)chip
  CFLAGS += -I$(ARCHSRCDIR)$(DELIM)common
  CFLAGS += -I$(BSPSRCDIR)
ifneq ($(CONFIG_ARCH_FAMILY),)
  CFLAGS += -I$(ARCHSRCDIR)$(DELIM)$(CONFIG_ARCH_FAMILY)
endif
endif

all: $(BIN)
.PHONY: context depend clean distclean clean_worker

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(LIB_OBJS)
	$(call ARCHIVE, $@, $(LIB_OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

.context:
	$(Q) touch $@

context: .context

clean_worker:
	$(Q) $(MAKE) -C worker TOPDIR="$(TOPDIR)" clean

distclean_worker:
	$(Q) $(MAKE) -C worker TOPDIR="$(TOPDIR)" distclean

clean: clean_worker
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean distclean_worker
	$(call DELFILE, .context)
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

