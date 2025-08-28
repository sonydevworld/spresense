
include $(APPDIR)/Make.defs
include $(SDKDIR)/Make.defs

PROGNAME = $(CONFIG_EXAMPLES_AHRS_PWBIMU_PROGNAME)
PRIORITY = $(CONFIG_EXAMPLES_AHRS_PWBIMU_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_AHRS_PWBIMU_STACKSIZE)

ASRCS =
CSRCS = net_client.c
MAINSRC = ahrs_pwbimu_main.c

ifeq ($(CONFIG_EXAMPLES_AHRS_PWBIMU_RECV),y)
PROGNAME  += ahrs_recv
PRIORITY  += 100
STACKSIZE += 2048
MAINSRC   += ahrs_receiver_main.c
endif

MODULE = $(CONFIG_EXAMPLES_AHRS_PWBIMU)

#########################################################################
## HOST Side build rule

ifeq ($(CONFIG_EXAMPLES_AHRS_PWBIMU_HOST_RECV),y)
HOST_BIN = host_recvserver

$(HOST_BIN): ahrs_receiver_main.c
	@echo "LD: $@"
	$(Q) $(HOSTCC) $(HOSTLDFLAGS) $^ -o $@

context:: $(HOST_BIN)

clean::
	$(call DELFILE, $(HOST_BIN))

endif

include $(APPDIR)/Application.mk
