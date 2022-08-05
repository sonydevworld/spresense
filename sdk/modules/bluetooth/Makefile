############################################################################
# bluetooth/Makefile
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

-include $(SDKDIR)/modules/Make.defs

MODNAME = bluetooth

CSRCS  =  bluetooth_common.c
ifeq ($(CONFIG_BLUETOOTH_A2DP),y)
CSRCS +=  bluetooth_a2dp.c
endif
ifeq ($(CONFIG_BLUETOOTH_AVRCP),y)
CSRCS +=  bluetooth_avrcp.c
endif
ifeq ($(CONFIG_BLUETOOTH_HFP),y)
CSRCS +=  bluetooth_hfp.c
endif
ifeq ($(CONFIG_BLUETOOTH_SPP),y)
CSRCS +=  bluetooth_spp.c
endif
ifeq ($(CONFIG_BLUETOOTH_LE_GATT),y)
CSRCS +=  bluetooth_le_gatt.c
endif
CSRCS +=  bluetooth_hal_init.c

ifeq ($(CONFIG_BCM20706),y)
include hal/bcm20706/Make.defs
endif

ifeq ($(CONFIG_BLUETOOTH_NRF52),y)
include hal/nrf52/Make.defs
endif

include $(SDKDIR)/modules/Module.mk
