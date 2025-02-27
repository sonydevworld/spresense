
include $(APPDIR)/Make.defs
include $(SDKDIR)/Make.defs

PROGNAME = $(CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_PROGNAME)
PRIORITY = $(CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_STACKSIZE)
MODULE = $(CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL)

ASRCS   =
CSRCS   =

ifneq ($(CONFIG_SENSORS_CXD5610_GNSS),)
CSRCS   += gnss_addon.c
endif

CXXSRCS = netserver.cxx alusr_pll.cxx alusr_filter.cxx
MAINSRC = audiolite_gnss1pps_dpll_main.cxx

ifneq ($(CONFIG_EXAMPLES_AUDIOLITE_GNSS1PPS_DPLL_PSEUDO1PPS),)
PROGNAME  += pseudo1pps
PRIORITY  += 100
STACKSIZE += 1024
MAINSRC   += pseudo1pps_main.c
endif

include $(APPDIR)/Application.mk

build_filter_worker:
	@$(MAKE) -C filter TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

clean_filter_worker:
	@$(MAKE) -C filter TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

build_pll_worker:
	@$(MAKE) -C pll TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

clean_pll_worker:
	@$(MAKE) -C pll TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

$(OBJS): build_pll_worker build_filter_worker

clean:: clean_pll_worker clean_filter_worker
