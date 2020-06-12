############################################################################
# externals/awsiot/Makefile
#
#   Copyright 2019 Sony Semiconductor Solutions Corporation
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

BIN = libawsiot$(LIBEXT)

AWSCOREDIR = aws-iot-device-sdk-embedded-C
AWSPLATDIR = $(AWSCOREDIR)$(DELIM)platform$(DELIM)linux

ROOTDEPPATH += --dep-path $(AWSCOREDIR)$(DELIM)src
ROOTDEPPATH += --dep-path $(AWSCOREDIR)$(DELIM)external_libs$(DELIM)jsmn
ROOTDEPPATH += --dep-path $(AWSPLATDIR)$(DELIM)common
ROOTDEPPATH += --dep-path $(AWSPLATDIR)$(DELIM)mbedtls
ROOTDEPPATH += --dep-path $(AWSPLATDIR)$(DELIM)pthread

VPATH += :$(AWSCOREDIR)$(DELIM)src
VPATH += :$(AWSCOREDIR)$(DELIM)external_libs$(DELIM)jsmn
VPATH += :$(AWSPLATDIR)$(DELIM)common
VPATH += :$(AWSPLATDIR)$(DELIM)mbedtls
VPATH += :$(AWSPLATDIR)$(DELIM)pthread

# Log option
ifeq ($(CONFIG_EXTERNALS_AWSIOT_ENABLE_IOT_DEBUG),y)
  CFLAGS += -DENABLE_IOT_DEBUG
endif

ifeq ($(CONFIG_EXTERNALS_AWSIOT_ENABLE_IOT_TRACE),y)
  CFLAGS += -DENABLE_IOT_TRACE
endif

ifeq ($(CONFIG_EXTERNALS_AWSIOT_ENABLE_IOT_INFO),y)
  CFLAGS += -DENABLE_IOT_INFO
endif

ifeq ($(CONFIG_EXTERNALS_AWSIOT_ENABLE_IOT_WARN),y)
  CFLAGS += -DENABLE_IOT_WARN
endif

ifeq ($(CONFIG_EXTERNALS_AWSIOT_ENABLE_IOT_ERROR),y)
  CFLAGS += -DENABLE_IOT_ERROR
endif

# AWS-IoT
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

include $(APPDIR)/Application.mk
