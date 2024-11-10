############################################################################
# modules/audiolite/Makefile
#
#   Copyright 2023 Sony Semiconductor Solutions Corporation
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

# Set 1 FORACE_WORKER_BUILD when you want to make spk

MODNAME = audiolite
CXXEXT = .cxx

CXXSRCS  = src/base/al_component.cxx
CXXSRCS += src/base/al_evthandler.cxx
CXXSRCS += src/base/al_inputnode.cxx
CXXSRCS += src/base/al_memalloc.cxx
CXXSRCS += src/base/al_outputnode.cxx
CXXSRCS += src/base/al_thread.cxx
CXXSRCS += src/base/al_workercomp.cxx
CXXSRCS += src/base/al_stream.cxx
CXXSRCS += src/base/al_decoder.cxx
CXXSRCS += src/base/al_worker.cxx
CXXSRCS += src/base/al_workercmd.cxx
CXXSRCS += src/driver/al_audiodrv.cxx
CXXSRCS += src/components/al_outputcomp.cxx
CXXSRCS += src/components/al_inputcomp.cxx
CXXSRCS += src/components/al_wavdec.cxx
CXXSRCS += src/components/al_wavenc.cxx
CXXSRCS += src/components/al_mp3dec.cxx

CSRCS =
CSRCS += worker/common/alworker_comm.c

CFLAGS += -I include -DBUILD_TGT_SUPERVISOR
CXXFLAGS += -I include -DBUILD_TGT_SUPERVISOR

ifneq ($(CONFIG_AUDIO_LITE_MP3DEC_SUBCORE_ELF),)
CSRCS += dummy.c
else ifeq ($(CONFIG_AUDIO_LITE_MP3DEC_SUBCORE_SPK_BUILD),y)
CSRCS += dummy.c
endif

include $(SDKDIR)/modules/Module.mk

.PHONY: build_mp3dec
build_mp3dec:
	@$(MAKE) -C worker/mp3dec TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

dummy.c: build_mp3dec

clean_mp3dec:
	@$(MAKE) -C worker/mp3dec TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

clean:: clean_mp3dec
