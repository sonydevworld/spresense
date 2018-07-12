############################################################################
# modules/audio/dsp/worker/Makefile
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

ifeq ($(WINTOOL),y)
LIB_DIR = "${shell cygpath -w lib}"
else
LIB_DIR = "lib"
endif

LDLIBPATH += -L $(LIB_DIR)
LDLIBPATH += -L "$(SDKDIR)/lib"

LDLIBS += -lasmpw -lc
#LDLIBS += -lasmpw "/usr/arm-none-eabi/lib/thumb/v7e-m/fpv4-sp/hard/libc.a" "/usr/arm-none-eabi/lib/thumb/v7e-m/fpv4-sp/hard/libnosys.a" "/usr/bin/../lib/gcc/arm-none-eabi/7.2.1/thumb/v7e-m/fpv4-sp/hard/libgcc.a"
#LDLIBS += -lasmpw -lcxx -lmm -lc -lsched -larch -lbsp -lfs -ldrivers -lapps -lbinfmt "/usr/bin/../lib/gcc/arm-none-eabi/7.2.1/thumb/v7e-m/fpv4-sp/hard/libgcc.a"

BIN = POSTFILTER

CXXSRCS = postfilter_ctrl.cpp main.cpp

CXXELFFLAGS += -Os
ifeq ($(WINTOOL),y)
#CXXELFFLAGS += -I"$(shell cygpath -w $(APPDIR))"
CXXELFFLAGS += -I"$(shell cygpath -w $(SDKDIR)$(DELIM)modules$(DELIM)asmp$(DELIM)worker)"
CXXELFFLAGS += -I"$(shell cygpath -w $(SDKDIR)$(DELIM)modules$(DELIM)audio$(DELIM)include)"
else
#CXXELFFLAGS += -I$(APPDIR)
CXXELFFLAGS += -I$(SDKDIR)/modules/asmp/worker
CXXELFFLAGS += -I$(SDKDIR)/modules/audio/include
endif

CXXELFFLAGS += -D_POSIX

ifneq ($(CONFIG_EXAMPLES_ASMP_WORKER_STACKSIZE),)
WORKERSTACKSIZE = $(CONFIG_EXAMPLES_ASMP_WORKER_STACKSIZE)
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
CXXOBJS = $(CXXSRCS:.cpp=$(OBJEXT))

all: $(BIN)

$(CXXOBJS): %$(OBJEXT): %.cpp
	@echo "CXX: $<"
	$(Q) $(CXX) -c $(CXXELFFLAGS) $< -o $@

$(AOBJS): %$(OBJEXT): %.S
	@echo "AS: $<"
	$(Q) $(CC) -c $(AFLAGS) $< -o $@

lib/libasmpw.a:
	$(Q) $(MAKE) -C lib TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" CROSSDEV=$(CROSSDEV)

$(BIN): $(CXXOBJS) $(AOBJS) lib/libasmpw.a
	@echo "LD: $@"
	$(Q) $(LD) $(LDRAWELFFLAGS) $(LDLIBPATH) -o $@ $(ARCHCRT0OBJ) $^ $(LDLIBS)

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, .context)
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

install:
	$(Q) mkdir -p $(ROMFS_DIR)
ifeq ($(WINTOOL),y)
	$(Q) $(STRIP) -d -o "$(shell cygpath -w $(ROMFS_DIR)/$(BIN))" $(BIN)
else
	$(Q) $(STRIP) -d -o $(ROMFS_DIR)/$(BIN) $(BIN)
endif
