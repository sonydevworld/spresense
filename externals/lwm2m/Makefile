############################################################################
# externals/lwm2m/Makefile
#
#   Copyright 2019 Sony Corporation
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

# Do not change

-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs
DELIM ?= $(strip /)
CXXEXT ?= .cpp

M2MCORE  := wakaama$(DELIM)core
M2MSHARE := wakaama$(DELIM)examples$(DELIM)shared

DEPPATH += --dep-path .
DEPPATH += --dep-path $(M2MCORE)
DEPPATH += --dep-path $(M2MCORE)$(DELIM)er-coap-13
DEPPATH += --dep-path $(M2MSHARE)
DEPPATH += --dep-path $(M2MSHARE)$(DELIM)tinydtls
DEPPATH += --dep-path $(M2MSHARE)$(DELIM)tinydtls$(DELIM)aes
DEPPATH += --dep-path $(M2MSHARE)$(DELIM)tinydtls$(DELIM)ecc
DEPPATH += --dep-path $(M2MSHARE)$(DELIM)tinydtls$(DELIM)sha2

VPATH += :.
VPATH += :$(M2MCORE)
VPATH += :$(M2MCORE)$(DELIM)er-coap-13
VPATH += :$(M2MSHARE)
VPATH += :$(M2MSHARE)$(DELIM)tinydtls
VPATH += :$(M2MSHARE)$(DELIM)tinydtls$(DELIM)aes
VPATH += :$(M2MSHARE)$(DELIM)tinydtls$(DELIM)ecc
VPATH += :$(M2MSHARE)$(DELIM)tinydtls$(DELIM)sha2

# Define
CFLAGS += -DHAVE_ASSERT_H
CFLAGS += -DLWM2M_CLIENT_MODE
CFLAGS += -DLWM2M_BOOTSTRAP
CFLAGS += -DLWM2M_LITTLE_ENDIAN
CFLAGS += -DLWM2M_SUPPORT_JSON
CFLAGS += -DWITH_TINYDTLS
CFLAGS += -DUIP_BUFSIZE=1294
CFLAGS += -DUIP_LLH_LEN=14
CFLAGS += -DUIP_IPUDPH_LEN=48

# LwM2M
CSRCS += block1.c
CSRCS += bootstrap.c
CSRCS += data.c
CSRCS += utils.c
CSRCS += tlv.c
CSRCS += discover.c
CSRCS += json.c
CSRCS += json_common.c
CSRCS += liblwm2m.c
CSRCS += list.c
CSRCS += management.c
CSRCS += objects.c
CSRCS += observe.c
CSRCS += packet.c
CSRCS += registration.c
CSRCS += transaction.c
CSRCS += uri.c
# COAP
CSRCS += er-coap-13.c
# tinyDTLS
CSRCS += platform.c
CSRCS += memtrace.c
CSRCS += netq.c
CSRCS += peer.c
CSRCS += session.c
CSRCS += ccm.c
CSRCS += commandline.c
#CSRCS += connection.c
CSRCS += crypto.c
CSRCS += dtls.c
CSRCS += dtls_time.c
CSRCS += dtls_debug.c
CSRCS += dtlsconnection.c
CSRCS += hmac.c
CSRCS += rijndael.c
CSRCS += ecc.c
CSRCS += sha2.c

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = liblwm2m.a

# Common build

all: $(BIN)
.PHONY: context depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

# Create dependencies

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

# context target proceeds before depend. User can be used this rule
# to prerequisite process (e.g. generate header file).

.context:
	$(Q) touch $@

context:

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, .context)
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
.PHONY: preconfig
preconfig:
