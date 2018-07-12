
-include $(TOPDIR)/Make.defs

BIN = libasmpw$(LIBEXT)
WORKER_DIR = $(SDKDIR)$(DELIM)modules$(DELIM)asmp$(DELIM)worker
LIB = $(WORKER_DIR)$(DELIM)libasmpw$(LIBEXT)

all: $(BIN)
.PHONY: depend clean distclean

depend:
	$(Q) $(MAKE) -C $(WORKER_DIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" depend

$(LIB): depend
	$(Q) $(MAKE) -C $(WORKER_DIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" $(BIN)

$(BIN): $(LIB)
	$(Q) install $< $@

clean:
	$(call CLEAN)

distclean: clean
