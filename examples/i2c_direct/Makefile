############################################################################
# i2c_direct/Makefile
############################################################################

include $(APPDIR)/Make.defs

# i2c_direct built-in application info
PROGNAME  = $(CONFIG_EXAMPLES_I2C_DIRECT_PROGNAME)
PRIORITY  = $(CONFIG_EXAMPLES_I2C_DIRECT_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_I2C_DIRECT_STACKSIZE)
MODULE    = $(CONFIG_EXAMPLES_I2C_DIRECT)

# I2C direct Example
ASRCS =
CSRCS = i2c_common.c i2c_bmi270.c bmi270.c
MAINSRC = i2c_direct_main.c

CFLAGS += -I ./include
CXXFLAGS += -I ./include

include $(APPDIR)/Application.mk
