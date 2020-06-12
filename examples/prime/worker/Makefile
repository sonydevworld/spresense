############################################################################
# prime/worker/Makefile
#
#   Copyright 2018 Sony Semiconductor Solutions Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name of Sony Semiconductor Solutions Corporation nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

WORKER_ELFS = hello/hello

SUBDIRS = $(dir $(WORKER_ELFS))

define DIR_template
$(1)_$(2):
	$(Q) $(MAKE) -C $(1) $(3) TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" SDKDIR="$(SDKDIR)" CROSSDEV=$(CROSSDEV)
endef

all: romfs.h
.PHONY: all clean depend lib

# Create the romfs.img file from the populated romfs directory

romfs.img: $(WORKER_ELFS)
	$(Q) mkdir -p romfs
	$(Q) cp $^ romfs
	$(Q) genromfs -f $@ -d romfs -V "WORKER"

# Create the romfs.h header file from the romfs.img file

romfs.h: romfs.img
	$(Q) xxd -i $< | sed -e "s/^unsigned/static const unsigned/g" >$@

# Build prime worker library

lib/libasmpw.a:
	$(Q) $(MAKE) -C lib TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

lib: lib/libasmpw.a

# Build workers

$(WORKER_ELFS): lib
	$(Q) $(MAKE) -C $(dir $@) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)

.depend:
	$(Q) $(MAKE) -C lib TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) depend
	@touch $@

depend: .depend

# Clean each subdirectory

lib_clean:
	$(Q) $(MAKE) -C lib TOPDIR="$(TOPDIR)" clean

clean: $(foreach DIR, $(SUBDIRS), $(DIR)_clean) lib_clean
	$(Q) rm -rf romfs romfs.img romfs.h

$(foreach DIR, $(SUBDIRS), $(eval $(call DIR_template,$(DIR),clean,clean)))
