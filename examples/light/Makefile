############################################################################
# light/Makefile
#
#   Copyright 2018 Sony Semiconductor Solutions Corporation
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

# sensor built-in application info

PROGNAME  = $(CONFIG_EXAMPLES_LIGHT_PROGNAME)
PRIORITY  = $(CONFIG_EXAMPLES_LIGHT_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_LIGHT_STACKSIZE)
MODULE    = $(CONFIG_EXAMPLES_LIGHT)

# sensor Example

ifeq ($(CONFIG_SENSORS_APDS9930_SCU),y)
MAINSRC = light_main_apds9930.c
endif

ifeq ($(CONFIG_SENSORS_LT1PA01_SCU),y)
MAINSRC = light_main_lt1pa01.c
endif

ifeq ($(CONFIG_SENSORS_BH1721FVC_SCU),y)
MAINSRC = light_main_bh1721fvc.c
endif

ifeq ($(CONFIG_SENSORS_RPR0521RS_SCU),y)
MAINSRC = light_main_rpr0521rs.c
endif

include $(APPDIR)/Application.mk
