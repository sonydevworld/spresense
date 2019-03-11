############################################################################
# modules/dnnrt/src/Make.defs
#
#   Copyright 2019 Sony Semiconductor Solutions Corporation
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

DELIM ?= $(strip /)

# like "uniq" command

uniq = $(if $1,$(firstword $1) $(call uniq,$(filter-out $(firstword $1),$1)))

RUNTIMEDIR ?= $(SDKDIR)/../externals/nnabla-c-runtime

# C source files are separated because many files are exists

include Make.defs

CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(RUNTIMEDIR)/include}
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(RUNTIMEDIR)/src/functions}
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(RUNTIMEDIR)/src/runtime}

# Runtime patch for missing fminf, fmaxf math functions

CFLAGS += -include fminf_fmaxf.h
CFLAGS += -std=c99

# Don't follow the -O. option in CFLAGS

CFLAGS := $(patsubst -O%,-O3,$(CFLAGS))

FUNC_OBJS = $(FUNC_SRCS:.c=$(OBJEXT))
RT_OBJS = $(RT_SRCS:.c=$(OBJEXT))

SRCS = $(FUNC_SRCS) $(RT_SRCS)
OBJS = $(FUNC_OBJS) $(RT_OBJS)

LIBFUNC = libnnablart_functions$(LIBEXT)
LIBRT = libnnablart_runtime$(LIBEXT)

all: $(LIBFUNC) $(LIBRT)

$(OBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(LIBFUNC): $(FUNC_OBJS)
	$(call ARCHIVE, $@, $(FUNC_OBJS))

$(LIBRT): $(RT_OBJS)
	$(call ARCHIVE, $@, $(RT_OBJS))

.context:
	$(Q) touch $@

context: .context

.depend: $(SDKDIR)/.config Makefile Make.defs $(SRCS)
	$(Q) $(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) > Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(LIBRUNC))
	$(call DELFILE, $(LIBRT))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, .context)
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
