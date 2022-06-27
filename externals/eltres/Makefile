############################################################################
# externals/eltres/Makefile
#
#   Copyright 2022 Sony Semiconductor Solutions Corporation
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

-include $(APPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

BIN = libeltres$(LIBEXT)

ROOTDEPPATH += --dep-path CXM150x_SDK/CXM150x_core
ROOTDEPPATH += --dep-path CXM150x_SDK/CXM150x_port

VPATH += :CXM150x_SDK/CXM150x_core
VPATH += :CXM150x_SDK/CXM150x_port

# CXM150x_core
CSRCS += CXM150x_CTRL_FW_UPDATE.c
CSRCS += CXM150x_GNSS.c
CSRCS += CXM150x_GNSS_FW_UPDATE.c
CSRCS += CXM150x_LIB.c
CSRCS += CXM150x_SYS.c
CSRCS += CXM150x_TEST.c
CSRCS += CXM150x_TIME.c
CSRCS += CXM150x_TX.c
CSRCS += CXM150x_Utility.c

# CXM150x_port
# TODO: support fw update
CSRCS += CXM150x_CTRL_FW_UPDATE_Port.c
CSRCS += CXM150x_GNSS_FW_UPDATE_Port.c
CSRCS += CXM150x_Port.c

include $(APPDIR)/Application.mk
