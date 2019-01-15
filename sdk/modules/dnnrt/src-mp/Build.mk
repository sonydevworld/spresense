############################################################################
# modules/dnnrt/src-mp/Makefile
#
#   Copyright 2018 Sony Corporation
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
# 3. Neither the name of Sony Corporation nor the names of its contributors
#    may be used to endorse or promote products derived from this software
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

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

DELIM ?= $(strip /)
RUNTIME_EXTERN_DIR := $(SDKDIR)/../externals/nnabla-c-runtime

RUNTIME_EXTERN_SRCDIR := $(RUNTIME_EXTERN_DIR)/src

INCLUDES := -I$(RUNTIME_EXTERN_DIR)/include
INCLUDES += -I$(RUNTIME_EXTERN_SRCDIR)/runtime
INCLUDES += -I$(RUNTIME_EXTERN_SRCDIR)/functions
INCLUDES += -Isrc-mp

CSRC_PATH += src-mp/runtime
CSRCS += runtime_client.c
CSRCS += mp_manager.c

VPATH += $(CSRC_PATH)
ROOTDEPPATH =$(foreach dir,$(CSRC_PATH), --dep-path $(dir))

ifeq ($(WINTOOL),y)
  INCLUDES += -I"${shell cygpath -w $(SDKDIR)/modules/include/dnnrt}"
  INCLUDES += -I"${shell cygpath -w $(SDKDIR)/modules/include}"
else
  INCLUDES += -I$(SDKDIR)/modules/include/dnnrt
  INCLUDES += -I$(SDKDIR)/modules/include
endif # ($(WINTOOL),y)
CFLAGS += $(INCLUDES) -std=c99
CFLAGS := $(patsubst -O%,-O3,$(CFLAGS)) # don't follow the -O. option in CFLAGS

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

LIB = libdnnrt$(LIBEXT)

all: .built

.PHONY: all context depend clean distclean preconfig

$(COBJS) $(MAINOBJ): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(OBJS): Makefile

$(LIB): $(OBJS)
	$(call ARCHIVE, $(LIB), $(OBJS))

.built: $(LIB)
	$(Q) touch .built

install:

context:
# make apps refer to the same version of network.h
	$(Q) install -m 0664 -C \
		$(RUNTIME_EXTERN_DIR)/include/nnablart/network.h \
		$(SDKDIR)/modules/include/dnnrt/nnablart/network.h

# Create dependencies

.depend: $(SDKDIR)/.config Makefile $(SRCS)
	$(Q) $(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) > Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call CLEAN)
	$(call DELFILE, .built)
	$(call DELFILE, Make.dep)
	$(call DELFILE, *.o)
	$(call DELFILE, *.d)
	$(call DELFILE, $(LIB))

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(call DELFILE, $(SDKDIR)/modules/include/dnnrt/nnablart/network.h)

-include Make.dep
preconfig:
