
include $(APPDIR)/Make.defs
include $(SDKDIR)/Make.defs

PROGNAME = $(CONFIG_EXAMPLES_FFT_PWBIMU_PROGNAME)
PRIORITY = $(CONFIG_EXAMPLES_FFT_PWBIMU_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_FFT_PWBIMU_STACKSIZE)
MODULE = $(CONFIG_EXAMPLES_FFT_PWBIMU)

ASRCS =
CSRCS =
MAINSRC = fft_pwbimu_main.cxx

CXXSRCS += alusr_imufft.cxx imu_hydrant.cxx

include $(APPDIR)/Application.mk

build_imufft_worker:
	@$(MAKE) -C imufft TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

clean_imufft_worker:
	@$(MAKE) -C imufft TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

$(OBJS): build_imufft_worker

clean:: clean_imufft_worker
