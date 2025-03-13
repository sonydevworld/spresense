ifeq ($(CONFIG_EXTERNALS_COBS_C),y)
  EXTRA_LIBPATHS += -L "$(EXTLIBDIR)$(DELIM)cobs-c"
  EXTRA_LIBS     += -lcobs
endif
