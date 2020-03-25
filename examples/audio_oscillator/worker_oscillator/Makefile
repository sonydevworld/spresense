############################################################################
# audio_oscillator/worker_oscillator/Makefile
#
#   Copyright 2020 Sony Semiconductor Solutions Corporation
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
include $(SDKDIR)/Make.defs

ifeq ($(WINTOOL),y)
LIB_DIR = "${shell cygpath -w lib}"
else
LIB_DIR = "lib"
endif

VPATH = userproc/src

LDLIBPATH += -L $(LIB_DIR)
LDLIBPATH += -L $(TOPDIR)/staging

LDLIBS += -lasmpw

# Setup to build and linking CMSIS DSP library

CMSIS_DSP = libarm_cortexM4lf_math$(LIBEXT)
CMSIS_DSP_DIR = $(EXTERNAL_DIR)$(DELIM)cmsis$(DELIM)dsp
LDLIBPATH += -L $(CMSIS_DSP_DIR)
LDLIBS += -larm_cortexM4lf_math

WORKER_ELF = OSCPROC

CXXSRCS = main.cpp oscillator.cpp

CXXELFFLAGS += -Os
ifeq ($(WINTOOL),y)
CXXELFFLAGS += -I"$(shell cygpath -w userproc$(DELIM)include)"
CXXELFFLAGS += -I"$(shell cygpath -w $(APPDIR))"
else
CXXELFFLAGS += -Iuserproc/include
CXXELFFLAGS += -I$(APPDIR)
CXXELFFLAGS += -I$(SDKDIR)/modules/audio/include
endif

CXXFLAGS += -D_POSIX

CXXOBJS = $(CXXSRCS:.cpp=$(OBJEXT))

# Build

all: lib $(WORKER_ELF)

# Build ASMP worker library

libasmpw.a:
	$(Q) $(MAKE) -C lib TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

lib: libasmpw.a $(CMSIS_DSP_DIR)/$(CMSIS_DSP)

$(CMSIS_DSP_DIR)/$(CMSIS_DSP):
	$(Q) $(MAKE) -C $(CMSIS_DSP_DIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

# Complile

$(CXXOBJS): %$(OBJEXT):%.cpp
	@echo "CXX: $<"
	$(Q) $(CXX) -c $(CXXELFFLAGS) $< -o $@

# Build workers

$(WORKER_ELF): $(CXXOBJS)
	@echo "LD: $@"
	$(Q) $(LD) $(LDRAWELFFLAGS) -e main --gc-sections $(LDLIBPATH) -o $@ \
		--start-group $(ARCHCRT0OBJ) $^ $(LDLIBS) $(LIBGCC) --end-group
	$(Q) $(STRIP) -d $(WORKER_ELF)

.depend:
	$(Q) $(MAKE) -C lib TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) depend
	@touch $@

depend: .depend

lib_clean:
	$(Q) $(MAKE) -C lib TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

clean: lib_clean
	$(call DELFILE, $(WORKER_ELF))
	$(call CLEAN)

