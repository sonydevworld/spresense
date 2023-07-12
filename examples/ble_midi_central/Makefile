
include $(APPDIR)/Make.defs
include $(SDKDIR)/Make.defs

PROGNAME = $(CONFIG_EXAMPLES_BLE_MIDI_CENTRAL_PROGNAME)
PRIORITY = $(CONFIG_EXAMPLES_BLE_MIDI_CENTRAL_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_BLE_MIDI_CENTRAL_STACKSIZE)
MODULE = $(CONFIG_EXAMPLES_BLE_MIDI_CENTRAL)

ASRCS =
CSRCS = midi_parser.c alworker_comm.c led_indicator.c
CXXSRCS = midi_play.cxx 
MAINSRC = ble_midi_central_main.c

CFLAGS += -Iworker
CXXFLAGS += -Iworker

include $(APPDIR)/Application.mk

.PHONY: build_synthworker

build_synthworker:
	@$(MAKE) -C worker TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

$(OBJS): build_synthworker

clean_synthworker:
	@$(MAKE) -C worker TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

clean:: clean_synthworker 

