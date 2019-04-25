############################################################################
# externals/awsiot/Makefile
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

AWSCORE := aws-iot-device-sdk-embedded-C
AWSPLAT := aws-iot-device-sdk-embedded-C$(DELIM)platform$(DELIM)linux

DEPPATH += --dep-path .
DEPPATH += --dep-path $(AWSCORE)$(DELIM)src
DEPPATH += --dep-path $(AWSCORE)$(DELIM)include
DEPPATH += --dep-path $(AWSCORE)$(DELIM)external_libs$(DELIM)jsmn
DEPPATH += --dep-path $(AWSPLAT)$(DELIM)common
DEPPATH += --dep-path $(AWSPLAT)$(DELIM)mbedtls
DEPPATH += --dep-path $(AWSPLAT)$(DELIM)pthread

VPATH += :.
VPATH += :$(AWSCORE)$(DELIM)src
VPATH += :$(AWSCORE)$(DELIM)include
VPATH += :$(AWSCORE)$(DELIM)external_libs$(DELIM)jsmn
VPATH += :$(AWSPLAT)$(DELIM)common
VPATH += :$(AWSPLAT)$(DELIM)mbedtls
VPATH += :$(AWSPLAT)$(DELIM)pthread

CFLAGS += -I$(AWSCORE)$(DELIM)external_libs$(DELIM)jsmn

# Define
ifdef CONFIG_EXTERNALS_AWSIOT_IOT_DEBUG
 CFLAGS += -DENABLE_IOT_DEBUG
endif
ifdef CONFIG_EXTERNALS_AWSIOT_IOT_ERROR
 CFLAGS += -DENABLE_IOT_ERROR
endif

# AWSIoT
CSRCS += aws_iot_jobs_interface.c
CSRCS += aws_iot_jobs_json.c
CSRCS += aws_iot_jobs_topics.c
CSRCS += aws_iot_jobs_types.c
CSRCS += aws_iot_json_utils.c
CSRCS += aws_iot_mqtt_client.c
CSRCS += aws_iot_mqtt_client_common_internal.c
CSRCS += aws_iot_mqtt_client_connect.c
CSRCS += aws_iot_mqtt_client_publish.c
CSRCS += aws_iot_mqtt_client_subscribe.c
CSRCS += aws_iot_mqtt_client_unsubscribe.c
CSRCS += aws_iot_mqtt_client_yield.c
CSRCS += aws_iot_shadow_actions.c
CSRCS += aws_iot_shadow.c
CSRCS += aws_iot_shadow_json.c
CSRCS += aws_iot_shadow_records.c
CSRCS += timer.c
CSRCS += network_mbedtls_wrapper.c
CSRCS += threads_pthread_wrapper.c
CSRCS += jsmn.c

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libawsiot.a

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
