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

# define

CFLAGS += -DHAVE_ASSERT_H
CFLAGS += -DLWM2M_CLIENT_MODE
CFLAGS += -DLWM2M_BOOTSTRAP
CFLAGS += -DLWM2M_LITTLE_ENDIAN
CFLAGS += -DLWM2M_SUPPORT_TLV
CFLAGS += -DLWM2M_SUPPORT_JSON
CFLAGS += -DWITH_TINYDTLS
CFLAGS += -DWITH_SHA256
CFLAGS += -DUIP_BUFSIZE=1294
CFLAGS += -DUIP_LLH_LEN=14
CFLAGS += -DUIP_IPUDPH_LEN=48
CFLAGS += -DLWM2M_COAP_DEFAULT_BLOCK_SIZE=1024
#CFLAGS += -DLWM2M_WITH_LOGS

LWM2MCORDIR = wakaama$(DELIM)core
LWM2MCOADIR = wakaama$(DELIM)coap
LWM2MDATDIR = wakaama$(DELIM)data
LWM2MSHADIR = wakaama$(DELIM)examples$(DELIM)shared
LWM2MCLIDIR = wakaama$(DELIM)examples$(DELIM)client

ROOTDEPPATH += --dep-path $(LWM2MCORDIR)
ROOTDEPPATH += --dep-path $(LWM2MCOADIR)
ROOTDEPPATH += --dep-path $(LWM2MCOADIR)$(DELIM)er-coap-13
ROOTDEPPATH += --dep-path $(LWM2MDATDIR)
ROOTDEPPATH += --dep-path $(LWM2MSHADIR)
ROOTDEPPATH += --dep-path $(LWM2MSHADIR)$(DELIM)tinydtls
ROOTDEPPATH += --dep-path $(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)aes
ROOTDEPPATH += --dep-path $(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)ecc
ROOTDEPPATH += --dep-path $(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)sha2
ROOTDEPPATH += --dep-path $(LWM2MCLIDIR)

VPATH       += :$(LWM2MCORDIR)
VPATH       += :$(LWM2MCOADIR)
VPATH       += :$(LWM2MCORDIR)$(DELIM)er-coap-13
VPATH       += :$(LWM2MCOADIR)$(DELIM)er-coap-13
VPATH       += :$(LWM2MDATDIR)
VPATH       += :$(LWM2MSHADIR)
VPATH       += :$(LWM2MSHADIR)$(DELIM)tinydtls
VPATH       += :$(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)aes
VPATH       += :$(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)ecc
VPATH       += :$(LWM2MSHADIR)$(DELIM)tinydtls$(DELIM)sha2
VPATH       += :$(LWM2MCLIDIR)

# LwM2M
CSRCS += block.c
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

include $(APPDIR)/Application.mk
