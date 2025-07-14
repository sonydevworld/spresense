ifeq ($(CONFIG_EXTERNALS_COBS_C),y)
CFLAGS   += ${INCDIR_PREFIX}$(SDKDIR)/../externals/cobs-c
CXXFLAGS += ${INCDIR_PREFIX}$(SDKDIR)/../externals/cobs-c
endif
