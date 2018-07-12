############################################################################
# modules/memutils/simple_fifo/LibTargets.mk
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
# Simple fifo library

ifeq ($(CONFIG_MEMUTILS),y)
ifeq ($(CONFIG_MEMUTILS_SIMPLE_FIFO),y)
SDKLIBS += lib$(DELIM)libsimplefifo$(LIBEXT)
SDKMODDIRS += modules$(DELIM)memutils$(DELIM)simple_fifo
endif
endif
SDKCLEANDIRS += modules$(DELIM)memutils$(DELIM)simple_fifo

modules$(DELIM)memutils$(DELIM)simple_fifo$(DELIM)libsimplefifo$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)memutils$(DELIM)simple_fifo TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libsimplefifo$(LIBEXT)

lib$(DELIM)libsimplefifo$(LIBEXT): modules$(DELIM)memutils$(DELIM)simple_fifo$(DELIM)libsimplefifo$(LIBEXT)
	$(Q) install $< $@
