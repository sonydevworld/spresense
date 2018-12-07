############################################################################
# examples/tlstest/Makefile
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
# 3. Neither the name NuttX nor Sony nor the names of its contributors
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

-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

-include $(SDKDIR)/.config

ifeq ($(WINTOOL),y)
LTEINC = "${shell cygpath -w $(SDKDIR)$(DELIM)modules$(DELIM)lte$(DELIM)include}"
NETINC = "${shell cygpath -w $(SDKDIR)$(DELIM)..$(DELIM)externals$(DELIM)netutils}"
else
LTEINC = "$(SDKDIR)$(DELIM)modules$(DELIM)lte$(DELIM)include"
NETINC = "$(SDKDIR)$(DELIM)..$(DELIM)externals$(DELIM)netutils"
endif

CFLAGS += -I$(LTEINC) -I$(NETINC)

# lte built-in application info

CONFIG_EXAMPLES_TLSTEST_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_EXAMPLES_TLSTEST_STACKSIZE ?= 2048

APPNAME = tlstest
PRIORITY = $(CONFIG_EXAMPLES_TLSTEST_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_TLSTEST_STACKSIZE)

# tlstest Example

ASRCS =
CSRCS += lte_sub.c ssl_sub.c
MAINSRC = tlstest_main.c

CONFIG_EXAMPLES_TLSTEST_PROGNAME ?= tlstest$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_TLSTEST_PROGNAME)

include $(APPDIR)/Application.mk
