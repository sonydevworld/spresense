
include $(APPDIR)/Make.defs
include $(SDKDIR)/Make.defs

PROGNAME = $(CONFIG_EXAMPLES_AUDIOLITE_FFT_PROGNAME)
PRIORITY = $(CONFIG_EXAMPLES_AUDIOLITE_FFT_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_AUDIOLITE_FFT_STACKSIZE)
MODULE = $(CONFIG_EXAMPLES_AUDIOLITE_FFT)

ASRCS =
CSRCS =
MAINSRC = audiolite_fft_main.cxx

CXXSRCS += alusr_fftworker.cxx

include $(APPDIR)/Application.mk

build_fftworker_worker:
	@$(MAKE) -C fftworker TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

clean_fftworker_worker:
	@$(MAKE) -C fftworker TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

$(OBJS): build_fftworker_worker

clean:: clean_fftworker_worker
