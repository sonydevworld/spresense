CMSISDIR = $(SDKDIR)$(DELIM)..$(DELIM)externals$(DELIM)cmsis
CMSIS_DSP_DIR = $(CMSISDIR)$(DELIM)dsp

# Setup to build and linking CMSIS DSP library

include $(CMSISDIR)/LibIncludes.mk

CMSIS_DSP_LIB = $(CMSIS_DSP_DIR)$(DELIM)libarm_cortexM4lf_math$(LIBEXT)
LDLIBPATH += -L $(CMSIS_DSP_DIR)
LDLIBS += -larm_cortexM4lf_math

$(CMSIS_DSP_LIB):
	$(Q) $(MAKE) -C $(CMSIS_DSP_DIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)"
