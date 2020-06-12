############################################################################
# externals/mqtt/Makefile
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

BIN = libmqtt$(LIBEXT)

ROOTDEPPATH += --dep-path paho.mqtt.embedded-c$(DELIM)MQTTPacket$(DELIM)src
ROOTDEPPATH += --dep-path paho.mqtt.embedded-c$(DELIM)MQTTClient-C$(DELIM)src
ROOTDEPPATH += --dep-path paho.mqtt.embedded-c$(DELIM)MQTTClient-C$(DELIM)src$(DELIM)nuttx

VPATH += :paho.mqtt.embedded-c$(DELIM)MQTTPacket$(DELIM)src
VPATH += :paho.mqtt.embedded-c$(DELIM)MQTTClient-C$(DELIM)src
VPATH += :paho.mqtt.embedded-c$(DELIM)MQTTClient-C$(DELIM)src$(DELIM)nuttx

CSRCS += MQTTClient.c
CSRCS += MQTTSocket.c
CSRCS += MQTTSslSocket.c
CSRCS += MQTTConnectClient.c
CSRCS += MQTTConnectServer.c
CSRCS += MQTTDeserializePublish.c
CSRCS += MQTTFormat.c
CSRCS += MQTTPacket.c
CSRCS += MQTTSerializePublish.c
CSRCS += MQTTSubscribeClient.c
CSRCS += MQTTSubscribeServer.c
CSRCS += MQTTUnsubscribeClient.c
CSRCS += MQTTUnsubscribeServer.c

include $(APPDIR)/Application.mk
