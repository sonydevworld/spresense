############################################################################
# externals/cmsis/LibTarget.mk
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

# CMSIS DSP
CMSIS_DSP := libarm_cortexM4lf_math$(LIBEXT)
CMSIS_DSP_DIR := $(EXTERNAL_DIR)$(DELIM)cmsis$(DELIM)dsp

ifeq ($(CONFIG_EXTERNALS_CMSIS_DSP),y)
EXTLIBS += lib$(DELIM)$(CMSIS_DSP)
EXTSUBDIRS += $(CMSIS_DSP_DIR)
endif
SDKCLEANDIRS += $(CMSIS_DSP_DIR)

$(CMSIS_DSP_DIR)$(DELIM)$(CMSIS_DSP): context
	$(Q) $(MAKE) -C $(dir $@) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" $(notdir $@)

lib$(DELIM)$(CMSIS_DSP): $(CMSIS_DSP_DIR)$(DELIM)$(CMSIS_DSP)
	$(Q) install $< $@

# CMSIS NN
CMSIS_NN := libcmsis_nn$(LIBEXT)
CMSIS_NN_DIR := $(EXTERNAL_DIR)$(DELIM)cmsis$(DELIM)nn

ifeq ($(CONFIG_EXTERNALS_CMSIS_NN),y)
EXTLIBS += lib$(DELIM)$(CMSIS_NN)
EXTSUBDIRS += $(CMSIS_NN_DIR)
endif
SDKCLEANDIRS += $(CMSIS_NN_DIR)

$(CMSIS_NN_DIR)$(DELIM)$(CMSIS_NN): context
	$(Q) $(MAKE) -C $(dir $@) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" $(notdir $@)

lib$(DELIM)$(CMSIS_NN): $(CMSIS_NN_DIR)$(DELIM)$(CMSIS_NN)
	$(Q) install $< $@

