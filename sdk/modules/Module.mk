############################################################################
# modules/Module.mk
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
#   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
#   Authors: Gregory Nutt <gnutt@nuttx.org>
#            Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

DELIM ?= $(strip /)

# The GNU make CURDIR will always be a POSIX-like path with forward slashes
# as path segment separators.  If we know that this is a native build, then
# we need to fix up the path so the DELIM will match the actual delimiter.

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
CWD = $(strip ${shell echo %CD% | cut -d: -f2})
else
CWD = $(CURDIR)
endif

SUFFIX = $(subst $(DELIM),.,$(CWD))

# Object files

AOBJS = $(ASRCS:.S=$(SUFFIX)$(OBJEXT))
COBJS = $(CSRCS:.c=$(SUFFIX)$(OBJEXT))
CXXOBJS = $(CXXSRCS:$(CXXEXT)=$(SUFFIX)$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS) $(CXXSRCS)
OBJS = $(AOBJS) $(COBJS) $(CXXOBJS)

DEPPATH += --dep-path .
DEPPATH += --obj-path .
DEPPATH += --obj-suffix $(SUFFIX)$(OBJEXT)
VPATH += :.

BIN ?= $(APPDIR)$(DELIM)libapps$(LIBEXT)

# Targets follow

all:: $(OBJS)
.PHONY: clean preconfig depend distclean

$(AOBJS): %$(SUFFIX)$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(SUFFIX)$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS): %$(SUFFIX)$(OBJEXT): %$(CXXEXT)
	$(call COMPILEXX, $<, $@)

archive:
ifeq ($(WINTOOL),y)
	$(call ARCHIVE_ADD, "${shell cygpath -w $(BIN)}", $(OBJS))
else
	$(call ARCHIVE_ADD, $(BIN), $(OBJS))
endif

install::

preconfig::

context::

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
ifeq ($(filter %$(CXXEXT),$(SRCS)),)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
else
	$(Q) $(MKDEP) $(DEPPATH) "$(CXX)" -- $(CXXFLAGS) -- $(SRCS) >Make.dep
endif
	$(Q) touch $@

depend:: .depend

clean::
	$(call CLEAN)

distclean:: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
