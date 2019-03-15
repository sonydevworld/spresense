EXTERNAL_DIR := $(SDKDIR)$(DELIM)..$(DELIM)externals

include $(wildcard $(EXTERNAL_DIR)/*/LibIncludes.mk)

