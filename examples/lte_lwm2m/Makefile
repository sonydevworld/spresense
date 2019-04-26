############################################################################
# lte_lwm2m/Makefile
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

-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

# lte_lwm2m built-in application info

CONFIG_EXAMPLES_LTE_LWM2M_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_EXAMPLES_LTE_LWM2M_STACKSIZE ?= 2048
CONFIG_EXAMPLES_LTE_LWM2M_STACKSIZE_IN_USING_MBEDTLS ?= 5120

APPNAME = lte_lwm2m
PRIORITY = $(CONFIG_EXAMPLES_LTE_LWM2M_PRIORITY)
ifeq ($(CONFIG_EXTERNALS_MBEDTLS),y)
STACKSIZE = $(CONFIG_EXAMPLES_LTE_LWM2M_STACKSIZE_IN_USING_MBEDTLS)
else
STACKSIZE = $(CONFIG_EXAMPLES_LTE_LWM2M_STACKSIZE)
endif

# define

CFLAGS += -DHAVE_ASSERT_H
CFLAGS += -DLWM2M_CLIENT_MODE
CFLAGS += -DLWM2M_BOOTSTRAP
CFLAGS += -DLWM2M_LITTLE_ENDIAN
CFLAGS += -DLWM2M_SUPPORT_JSON
CFLAGS += -DWITH_TINYDTLS

# lte_lwm2m Example

LWM2MCLIDIR = $(EXTERNAL_DIR)$(DELIM)lwm2m$(DELIM)wakaama$(DELIM)examples$(DELIM)client
ROOTDEPPATH = --dep-path $(LWM2MCLIDIR)
VPATH       = :$(LWM2MCLIDIR)

ASRCS =
CSRCS += object_access_control.c
CSRCS += object_device.c
CSRCS += object_firmware.c
CSRCS += object_location.c
CSRCS += object_connectivity_moni.c
CSRCS += object_security.c
CSRCS += object_server.c
CSRCS += object_connectivity_stat.c
CSRCS += system_api.c
CSRCS += test_object.c
CSRCS += lwm2m_lte_connection.c
MAINSRC = lwm2mclient.c

ifeq ($(WINTOOL),y)
CFLAGS += -I "${shell cygpath -w $(LWM2MCLIDIR)}"
else
CFLAGS += -I$(LWM2MCLIDIR)
endif

CONFIG_EXAMPLES_LTE_LWM2M_PROGNAME ?= lte_lwm2m$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_LTE_LWM2M_PROGNAME)

include $(APPDIR)/Application.mk
