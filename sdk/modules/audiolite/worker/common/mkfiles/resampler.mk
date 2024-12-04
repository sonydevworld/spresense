ifneq ($(CONFIG_ASMP_WORKER_CMSIS),y)
$(error resampler uses CMSIS, 
        you must set CONFIG_ASMP_WORKER_CMSIS and CONFIG_EXTERNAL_CMSIS.)
endif

SPEEXDSP_DIR = $(ALWORKER_COMMON)/../ext_libs/speexdsp_resample

CSRCS += resample.c
VPATH_DIR += $(SPEEXDSP_DIR)
INCDIRS += $(SPEEXDSP_DIR)
CFLAGS += -DUSE_CMSIS
