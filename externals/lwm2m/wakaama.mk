############################################################################
# externals/lwm2m/wakaama.mk
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

ifeq ($(CONFIG_EXTERNALS_LWM2M_CLIENT_MODE),y)
WAKAAMA_DEFS += -DLWM2M_CLIENT_MODE
endif

ifeq ($(CONFIG_EXTERNALS_LWM2M_BOOTSTRAP),y)
WAKAAMA_DEFS += -DLWM2M_BOOTSTRAP
endif

ifeq ($(CONFIG_EXTERNALS_LWM2M_SERVER_MODE),y)
WAKAAMA_DEFS += -DLWM2M_SERVER_MODE
endif

ifeq ($(CONFIG_EXTERNALS_LWM2M_BOOTSTRAP_SERVER_MODE),y)
WAKAAMA_DEFS += -DLWM2M_BOOTSTRAP_SERVER_MODE
endif

ifeq ($(CONFIG_EXTERNALS_LWM2M_SUPPORT_TLV),y)
WAKAAMA_DEFS += -DLWM2M_SUPPORT_TLV
endif

ifeq ($(CONFIG_EXTERNALS_LWM2M_SUPPORT_JSON),y)
WAKAAMA_DEFS += -DLWM2M_SUPPORT_JSON
endif

ifeq ($(CONFIG_EXTERNALS_LWM2M_SUPPORT_SENML_JSON),y)
WAKAAMA_DEFS += -DLWM2M_SUPPORT_SENML_JSON
endif

ifeq ($(CONFIG_EXTERNALS_LWM2M_WITH_TINYDTLS),y)
WAKAAMA_DEFS += -DWITH_TINYDTLS
endif

ifeq ($(CONFIG_EXTERNALS_LWM2M_OLD_CONTENT_FORMAT_SUPPORT),y)
WAKAAMA_DEFS += -DLWM2M_OLD_CONTENT_FORMAT_SUPPORT
endif

ifeq ($(CONFIG_EXTERNALS_LWM2M_VERSION_1_0),y)
WAKAAMA_DEFS += -DLWM2M_VERSION_1_0
endif

ifeq ($(CONFIG_EXTERNALS_LWM2M_RAW_BLOCK1_REQUESTS),y)
WAKAAMA_DEFS += -DLWM2M_RAW_BLOCK1_REQUESTS
endif

WAKAAMA_DEFS += -DLWM2M_COAP_DEFAULT_BLOCK_SIZE=$(CONFIG_EXTERNALS_LWM2M_COAP_DEFAULT_BLOCK_SIZE)

ifeq ($(CONFIG_EXTERNALS_LWM2M_WITH_LOGS),y)
WAKAAMA_DEFS += -DLWM2M_WITH_LOGS
endif

ifeq ($(CONFIG_EXTERNALS_LWM2M_MEMORY_TRACE),y)
WAKAAMA_DEFS += -DLWM2M_MEMORY_TRACE
endif

CFLAGS   += $(WAKAAMA_DEFS)
CXXFLAGS += $(WAKAAMA_DEFS)
