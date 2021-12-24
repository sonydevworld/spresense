############################################################################
# modules/dnnrt/Makefile
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

-include $(SDKDIR)/modules/Make.defs

RUNTIMEDIR := $(SDKDIR)/../externals/nnablart/nnabla-c-runtime

MODNAME = dnnrt

CSRCS :=

ifeq ($(CONFIG_DNN_RT_MP),y)
ifeq ($(CONFIG_DNN_RT_MPCOMM),y)
CSRCS += runtime_client.c

VPATH += src-mpcomm/runtime
DEPPATH = --dep-path src-mpcomm/runtime
else
CSRCS += runtime_client.c
CSRCS += mp_manager.c

VPATH += src-mp/runtime
DEPPATH = --dep-path src-mp/runtime
endif
else
CSRCS +=  runtime_nnabla.c
CSRCS +=  shared_chunk.c
CSRCS +=  affine.c
CSRCS +=  convolution.c

CFLAGS += -Isrc

VPATH = src/functions src/runtime
DEPPATH = --dep-path src/functions --dep-path src/runtime
endif

CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(RUNTIMEDIR)/include}
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(RUNTIMEDIR)/src/functions}
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(RUNTIMEDIR)/src/runtime}

CFLAGS += -std=c99

# Don't follow the -O. option in CFLAGS
CFLAGS := $(patsubst -O%,-O3,$(CFLAGS))

include $(SDKDIR)/modules/Module.mk

$(SDKDIR)/modules/include/dnnrt/nnablart/network.h: $(RUNTIMEDIR)/include/nnablart/network.h
	$(call COPYFILE,$<,$@)

context:: $(SDKDIR)/modules/include/dnnrt/nnablart/network.h

ifeq ($(CONFIG_DNN_RT_MPCOMM),y)

build_worker:
	@$(MAKE) -C src-mpcomm/worker TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

$(OBJS): build_worker

clean:: clean_worker

clean_worker:
	@$(MAKE) -C src-mpcomm/worker TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

endif
