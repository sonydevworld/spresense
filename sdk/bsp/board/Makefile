############################################################################
# bsp/board/Makefile
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
-include $(SDKDIR)/Make.defs

BOARDNAME := $(patsubst "%",%,$(strip $(CONFIG_BOARD_NAME)))

# For ASRCS and CSRCS, use the file specified by $(BOARDNAME)/Make.defs 
# in preference.

ASRCS =
CSRCS =
DEPPATH =

ifneq ($(BOARDNAME),)
include $(BOARDNAME)/Make.defs
endif

VPATH   += common/src
DEPPATH += --dep-path common/src

CSRCS += cxd56_boot.c

ifeq ($(CONFIG_ARCH_FPU),y)
CSRCS += cxd56_ostest.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += cxd56_usbmsc.c
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
CSRCS += cxd56_composite.c
endif

ifeq ($(CONFIG_SENSORS),y)
CSRCS += cxd56_sensors.c
endif

ifeq ($(CONFIG_BMI160),y)
ifeq ($(CONFIG_BMI160_I2C),y)
CSRCS += cxd56_bmi160_i2c.c
else
CSRCS += cxd56_bmi160_spi.c
endif
endif

ifeq ($(CONFIG_KX022),y)
CSRCS += cxd56_kx022.c
endif

ifeq ($(CONFIG_BMP280),y)
CSRCS += cxd56_bmp280.c
endif

ifeq ($(CONFIG_BM1383GLV),y)
CSRCS += cxd56_bm1383glv.c
endif

ifeq ($(CONFIG_AK09912),y)
CSRCS += cxd56_ak09912.c
endif

ifeq ($(CONFIG_BM1422GMV),y)
CSRCS += cxd56_bm1422gmv.c
endif

ifeq ($(CONFIG_APDS9930),y)
CSRCS += cxd56_apds9930.c
endif

ifeq ($(CONFIG_LT1PA01),y)
CSRCS += cxd56_lt1pa01.c
endif

ifeq ($(CONFIG_BH1721FVC),y)
CSRCS += cxd56_bh1721fvc.c
endif

ifeq ($(CONFIG_RPR0521RS),y)
CSRCS += cxd56_rpr0521rs.c
endif

ifeq ($(CONFIG_APDS9960),y)
CSRCS += cxd56_apds9960.c
endif

ifeq ($(CONFIG_BH1745NUC),y)
CSRCS += cxd56_bh1745nuc.c
endif

ifeq ($(CONFIG_SPI),y)
CSRCS += cxd56_spi.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += cxd56_pwm.c
endif

ifeq ($(CONFIG_LCD_LPM013M091A),y)
CSRCS += cxd56_lpm013m091a.c
endif

ifeq ($(CONFIG_LCD_ET014TT1),y)
CSRCS += cxd56_et014tt1.c
endif

ifeq ($(CONFIG_BOARDCTL_UNIQUEID),y)
CSRCS += cxd56_uid.c
endif

ifeq ($(CONFIG_BOARD_CRASHDUMP),y)
CSRCS += cxd56_crashdump.c
endif

ifeq ($(CONFIG_LCD_ILI9340),y)
CSRCS += cxd56_ili9340.c
endif

ifeq ($(CONFIG_SYSTEM_I2CTOOL),y)
CSRCS += cxd56_i2cdev.c
endif

ifeq ($(CONFIG_CXD56_EMMC),y)
CSRCS += cxd56_emmcdev.c
endif

ifeq ($(CONFIG_CXD56_SFC),y)
CSRCS += cxd56_flash.c
endif

ifeq ($(CONFIG_VIDEO_ISX012),y)
CSRCS += cxd56_isx012.c
endif

ifeq ($(CONFIG_CXD56_AUDIO), y)
CSRCS += cxd56_audio.c
endif

ifeq ($(CONFIG_CXD56_SPISD),y)
CSRCS += cxd56_spisd.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += cxd56_autoleds.c
else
ifeq ($(CONFIG_USERLED),y)
CSRCS += cxd56_userleds.c
endif
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += cxd56_buttons.c
endif

ifeq ($(CONFIG_CXD56_GPIO_IRQ),y)
CSRCS += cxd56_gpioif.c
endif

ifeq ($(CONFIG_CXD56_GAUGE),y)
CSRCS += cxd56_gauge.c
endif

ifeq ($(CONFIG_CXD56_CHARGER),y)
CSRCS += cxd56_charger.c
endif

include $(SDKDIR)/bsp/scripts/Board.mk
