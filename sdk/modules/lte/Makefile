############################################################################
# modules/lte/Makefile
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

-include $(SDKDIR)/modules/Make.defs

DELIM ?= $(strip /)
CXXEXT ?= .cpp

MODNAME = lte

# Add sub directory if exists

VPATH   =
SUBDIRS =
DEPPATH =

CSRCS  =
CXXSRCS =

# Include sub directory source files

include $(wildcard */Make.defs)

# Here is a example of adding include paths.
# Note that include path addtion will be affected to build time.
# Don't add unnecessary include paths for module.

LTESRCDIR = $(SDKDIR)$(DELIM)modules$(DELIM)lte
LTEINCDIR = $(SDKDIR)$(DELIM)modules$(DELIM)lte$(DELIM)include
LTEALTCOMINCDIR = $(SDKDIR)$(DELIM)modules$(DELIM)lte$(DELIM)altcom$(DELIM)include
LTEALTCOMAPIINCDIR = $(SDKDIR)$(DELIM)modules$(DELIM)include$(DELIM)lte

ifeq ($(WINTOOL),y)
  CFLAGS += -I "${shell cygpath -w $(LTEINCDIR)$(DELIM)opt}"
  CFLAGS += -I "${shell cygpath -w $(LTEINCDIR)$(DELIM)osal}"
  CFLAGS += -I "${shell cygpath -w $(LTEINCDIR)$(DELIM)util}"
  CFLAGS += -I "${shell cygpath -w $(LTEALTCOMINCDIR)}"
  CFLAGS += -I "${shell cygpath -w $(LTEALTCOMINCDIR)$(DELIM)api}"
  CFLAGS += -I "${shell cygpath -w $(LTEALTCOMINCDIR)$(DELIM)api$(DELIM)lte}"
  CFLAGS += -I "${shell cygpath -w $(LTEALTCOMINCDIR)$(DELIM)api$(DELIM)socket}"
  CFLAGS += -I "${shell cygpath -w $(LTEALTCOMINCDIR)$(DELIM)api$(DELIM)mbedtls}"
  CFLAGS += -I "${shell cygpath -w $(LTEALTCOMINCDIR)$(DELIM)evtdisp}"
  CFLAGS += -I "${shell cygpath -w $(LTEALTCOMINCDIR)$(DELIM)gw}"
  CFLAGS += -I "${shell cygpath -w $(LTEALTCOMAPIINCDIR)$(DELIM)altcom}"
  CFLAGS += -I "${shell cygpath -w $(LTEALTCOMAPIINCDIR)$(DELIM)altcom$(DELIM)net}"
ifeq ($(CONFIG_NET),y)
    CFLAGS += -I "${shell cygpath -w $(TOPDIR)$(DELIM)net}"
endif
else
  CFLAGS += -I$(LTEINCDIR)$(DELIM)opt
  CFLAGS += -I$(LTEINCDIR)$(DELIM)osal
  CFLAGS += -I$(LTEINCDIR)$(DELIM)util
  CFLAGS += -I$(LTEALTCOMINCDIR)
  CFLAGS += -I$(LTEALTCOMINCDIR)$(DELIM)api
  CFLAGS += -I$(LTEALTCOMINCDIR)$(DELIM)api$(DELIM)lte
  CFLAGS += -I$(LTEALTCOMINCDIR)$(DELIM)api$(DELIM)socket
  CFLAGS += -I$(LTEALTCOMINCDIR)$(DELIM)api$(DELIM)mbedtls
  CFLAGS += -I$(LTEALTCOMINCDIR)$(DELIM)evtdisp
  CFLAGS += -I$(LTEALTCOMINCDIR)$(DELIM)gw
  CFLAGS += -I$(LTEALTCOMAPIINCDIR)$(DELIM)altcom
  CFLAGS += -I$(LTEALTCOMAPIINCDIR)$(DELIM)altcom$(DELIM)net
ifeq ($(CONFIG_NET),y)
    CFLAGS += -I$(TOPDIR)$(DELIM)net
endif
endif

include $(SDKDIR)/modules/Module.mk

