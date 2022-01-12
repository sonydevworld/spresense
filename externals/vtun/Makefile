############################################################################
# externals/vtun/Makefile
#
#   Copyright 2021 Sony Semiconductor Solutions Corporation
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

include $(APPDIR)/Make.defs
include $(SDKDIR)/Make.defs

# vtun built-in application info

PROGNAME  = $(CONFIG_EXTERNALS_VTUN_PROGNAME)
PRIORITY  = $(CONFIG_EXTERNALS_VTUN_PRIORITY)
STACKSIZE = $(CONFIG_EXTERNALS_VTUN_STACKSIZE)
MODULE    = $(CONFIG_EXTERNALS_VTUN)

# Set up build environment

WGET   ?= wget
UNPACK := unzip

LEX      := lex
LEXFLAGS := -t

YACC      := yacc
YACCFLAGS := -d

VTUN_SH1 := cbc46c9ad44e6c02196d5ea632fb41536331a200
VTUN_DIR := VTun-$(VTUN_SH1)
VTUN_URL := "https://codeload.github.com/SPRESENSE/VTun/zip/$(VTUN_SH1)"
VTUN_DL_NAME := archive.zip

# Patch files

VTUN_PATCH := patch
VTUN_PATCHFILES = $(shell cd $(VTUN_PATCH); find -name "*.patch")

$(VTUN_DIR):
	$(Q) echo "Downloading: $(VTUN_URL)"
	$(Q) mkdir -p $(VTUN_DIR)
	$(Q) $(WGET) -O $(VTUN_DL_NAME) $(VTUN_URL)
	$(Q) $(UNPACK) $(VTUN_DL_NAME)

ASRCS =
CSRCS += compat_nuttx.c
CSRCS += nuttx/tun_dev.c
CSRCS += $(VTUN_DIR)/cfg_file.tab.c
CSRCS += $(VTUN_DIR)/cfg_file.lex.c
CSRCS += $(VTUN_DIR)/server.c
CSRCS += $(VTUN_DIR)/client.c
CSRCS += $(VTUN_DIR)/lib.c
CSRCS += $(VTUN_DIR)/llist.c
CSRCS += $(VTUN_DIR)/auth.c
CSRCS += $(VTUN_DIR)/tunnel.c
CSRCS += $(VTUN_DIR)/lock.c
CSRCS += $(VTUN_DIR)/netlib.c
CSRCS += $(VTUN_DIR)/generic/pty_dev.c
CSRCS += $(VTUN_DIR)/generic/pipe_dev.c
CSRCS += $(VTUN_DIR)/generic/tcp_proto.c
CSRCS += $(VTUN_DIR)/generic/udp_proto.c
CSRCS += $(VTUN_DIR)/linkfd.c
CSRCS += $(VTUN_DIR)/lfd_shaper.c
CSRCS += $(VTUN_DIR)/lfd_zlib.c
CSRCS += $(VTUN_DIR)/lfd_lzo.c
CSRCS += $(VTUN_DIR)/lfd_encrypt.c
CSRCS += $(VTUN_DIR)/lfd_legacy_encrypt.c
MAINSRC = $(VTUN_DIR)/main.c

CFLAGS += -I$(VTUN_DIR)
CFLAGS += -DVTUN_CONFIG_FILE=\"$(CONFIG_EXTERNALS_VTUN_CFG_FILE)\"
CFLAGS += -DVTUN_PID_FILE=\"$(CONFIG_EXTERNALS_VTUN_PID_FILE)\"
CFLAGS += -DVTUN_STAT_DIR=\"$(CONFIG_EXTERNALS_VTUN_STAT_DIR)\"
CFLAGS += -DVTUN_LOCK_DIR=\"$(CONFIG_EXTERNALS_VTUN_LOCK_DIR)\"
CFLAGS += -DVTUN_NUTTX

context:: $(VTUN_DIR)

distclean::
	$(call DELFILE, $(VTUN_DL_NAME))
	$(call DELDIR, $(VTUN_DIR))

include $(APPDIR)/Application.mk
