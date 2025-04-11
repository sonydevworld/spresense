include $(APPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

ALWORKER_COMMON = $(SDKDIR)/modules/audiolite/worker/common

BUILD_EXECUTE = 1

# ALWORKER_USE_CMSIS = 1
# ALWORKER_USE_RESAMPLER = 1

BIN = imufft
SPK = $(BIN).spk

CSRCS = imufft_worker_main.c

CFLAGS +=
LDLIBPATH =
LDLIBS =

VPATH_DIRS =

INCDIRS =

include $(ALWORKER_COMMON)/mkfiles/alworker.mk
