############################################################################
# modules/dnnrt/LibTargets.mk
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
# Dnnrt Library

ifeq ($(CONFIG_DNN_RT),y)

LIBDNN  = libdnnrt$(LIBEXT)
DNNDIR  = modules$(DELIM)dnnrt

SDKLIBS += lib$(DELIM)$(LIBDNN)
SDKCLEANDIRS += $(DNNDIR)
SDKMODDIRS += $(DNNDIR)
CONTEXTDIRS += $(DNNDIR)

lib$(DELIM)$(LIBDNN): $(DNNDIR)$(DELIM)$(LIBDNN)
	install $< $@

ifeq ($(CONFIG_DNN_RT_MP),y)
$(DNNDIR)$(DELIM)$(LIBDNN): context
	$(Q) $(MAKE) -C $(DNNDIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)"
else
LIBRT = libnnablart_runtime$(LIBEXT)
LIBFUNC = libnnablart_functions$(LIBEXT)
SDKLIBS += lib$(DELIM)$(LIBRT)
SDKLIBS += lib$(DELIM)$(LIBFUNC)

lib$(DELIM)$(LIBRT): $(DNNDIR)$(DELIM)$(LIBRT)
	install $< $@

lib$(DELIM)$(LIBFUNC): $(DNNDIR)$(DELIM)$(LIBFUNC)
	install $< $@

$(DNNDIR)$(DELIM)$(LIBRT) $(DNNDIR)$(DELIM)$(LIBFUNC) $(DNNDIR)$(DELIM)$(LIBDNN): context
	$(Q) $(MAKE) -C $(DNNDIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)"

endif

endif
