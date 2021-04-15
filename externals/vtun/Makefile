############################################################################
# system/vtun/Makefile
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

-include $(TOPDIR)/Make.defs
include $(SDKDIR)/Make.defs

CSRCS =
MAINSRC =

# vtun built-in application info

PROGNAME  = $(CONFIG_EXTERNALS_VTUN_PROGNAME)
PRIORITY  = $(CONFIG_EXTERNALS_VTUN_PRIORITY)
STACKSIZE = $(CONFIG_EXTERNALS_VTUN_STACKSIZE)
MODULE    = $(CONFIG_EXTERNALS_VTUN)

# Set up build environment

WGET   ?= wget
TAR    ?= tar 
TAROPT ?= xzf

VTUN_VER := 3.0.4
VTUN_DIR := vtun_src
VTUN_URL := "https://jaist.dl.sourceforge.net/project/vtun/vtun/$(VTUN_VER)/vtun-$(VTUN_VER).tar.gz"
VTUN_DL_NAME := archive.tar.gz

# Patch files

VTUN_PATCH := patch
VTUN_PATCHFILES = $(shell cd $(VTUN_PATCH); find -name "*.patch")

$(VTUN_DIR):
	$(Q) echo "Downloading: $(VTUN_URL)"
	$(Q) mkdir -p $(VTUN_DIR)
	$(Q) $(WGET) -O $(VTUN_DL_NAME) $(VTUN_URL)
	$(Q) cd $(VTUN_DIR); $(TAR) $(TAROPT) ..$(DELIM)$(VTUN_DL_NAME) --strip=1
	$(Q) files="$(VTUN_PATCHFILES)"; \
	     cd $(VTUN_DIR); \
	     for ff in $$files; do echo patch $$ff; patch -p0 < ..$(DELIM)$(VTUN_PATCH)$(DELIM)$$ff; done;

context:: $(VTUN_DIR)

distclean::
	$(call DELFILE, $(VTUN_DL_NAME))
	$(call DELDIR, $(VTUN_DIR))

include $(APPDIR)/Application.mk
