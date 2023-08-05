############################################################################
# externals/lwm2m/Makefile
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
-include $(SDKDIR)/Make.defs

BIN = liblwm2m$(LIBEXT)

# Wakaama configuration

include wakaama.mk

CFLAGS += -DLWM2M_LITTLE_ENDIAN

ifeq ($(CONFIG_EXTERNALS_LWM2M_WITH_TINYDTLS),y)
CFLAGS += -DHAVE_ASSERT_H
CFLAGS += -DWITH_SHA256
endif

LWM2MCORDIR = wakaama$(DELIM)core
LWM2MCOADIR = wakaama$(DELIM)coap
LWM2MDATDIR = wakaama$(DELIM)data
LWM2MSHADIR = wakaama$(DELIM)examples$(DELIM)shared

CFLAGS      += ${INCDIR_PREFIX}$(LWM2MCORDIR)
CFLAGS      += ${INCDIR_PREFIX}$(LWM2MCOADIR)

ROOTDEPPATH += --dep-path $(LWM2MCORDIR)
ROOTDEPPATH += --dep-path $(LWM2MCOADIR)
ROOTDEPPATH += --dep-path $(LWM2MCOADIR)$(DELIM)er-coap-13
ROOTDEPPATH += --dep-path $(LWM2MDATDIR)
ROOTDEPPATH += --dep-path $(LWM2MSHADIR)

VPATH       += :$(LWM2MCORDIR)
VPATH       += :$(LWM2MCOADIR)
VPATH       += :$(LWM2MCOADIR)$(DELIM)er-coap-13
VPATH       += :$(LWM2MDATDIR)
VPATH       += :$(LWM2MSHADIR)

ifeq ($(CONFIG_EXTERNALS_LWM2M_WITH_TINYDTLS),y)
ROOTDEPPATH += --dep-path $(LWM2MSHADIR)$(DELIM)tinydtls
ROOTDEPPATH += --dep-path $(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)aes
ROOTDEPPATH += --dep-path $(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)ecc
ROOTDEPPATH += --dep-path $(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)sha2

VPATH       += :$(LWM2MSHADIR)$(DELIM)tinydtls
VPATH       += :$(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)aes
VPATH       += :$(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)ecc
VPATH       += :$(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)sha2
endif

# core
CSRCS += liblwm2m.c
CSRCS += uri.c
CSRCS += utils.c
CSRCS += objects.c
CSRCS += list.c
CSRCS += packet.c
CSRCS += registration.c
CSRCS += bootstrap.c
CSRCS += management.c
CSRCS += observe.c
CSRCS += discover.c
# data
CSRCS += data.c
CSRCS += tlv.c
CSRCS += json.c
CSRCS += senml_json.c
CSRCS += json_common.c
# coap
CSRCS += transaction.c
CSRCS += block.c
CSRCS += er-coap-13.c
# shared
CSRCS += commandline.c
CSRCS += platform.c
CSRCS += memtrace.c
# tinydtls
ifeq ($(CONFIG_EXTERNALS_LWM2M_WITH_TINYDTLS),y)
CSRCS += dtlsconnection.c
CSRCS += dtls.c
CSRCS += crypto.c
CSRCS += ccm.c
CSRCS += hmac.c
CSRCS += dtls_debug.c
CSRCS += netq.c
CSRCS += peer.c
CSRCS += dtls_time.c
CSRCS += session.c
CSRCS += rijndael.c
CSRCS += sha2.c
CSRCS += ecc.c
else
CSRCS += connection.c
endif

include $(APPDIR)/Application.mk
