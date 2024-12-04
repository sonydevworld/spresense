############################################################################
# externals/tensorflow/Makefile
#
#   Copyright 2021 Sony Semiconductor Solutions Corporation
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

include $(APPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

CUR_DIR=$(shell pwd)

BIN = libtensorflow-microlite$(LIBEXT)

#######################################################################
# Wrapper source codes
# dummy_src.c is a dummy source code for building Tensorflow microlite.

CSRCS = dummy_src.c

## For Audio Provider in Tensorflow example
ifeq ($(CONFIG_EXTERNALS_TENSORFLOW_EXAMPLE_MICROSPEECH),y)
CXXSRCS = tf_audio_provider.cxx tf_audio_util.cxx

CXXFLAGS += -D_POSIX
CXXFLAGS += -DUSE_MEMMGR_FENCE
CXXFLAGS += -DATTENTION_USE_FILENAME_LINE
endif

ifeq ($(CONFIG_EXTERNALS_TENSORFLOW_EXAMPLE_PERSONDETECTION),y)
CSRCS += tf_image_provider.c
endif

CFLAGS += -I wrapper_include
CXXFLAGS += -I wrapper_include

#######################################################################
# Build script of Tensorflow microlite
#

WGET ?= wget
RM ?= rm
MV ?= mv
UNPACK := unzip

## For tflite-micro-spresense-examples repository

include TensorflowTag.mk

TENSORFLOW_URL = https://github.com/sonydevworld/tflite-micro-spresense-examples/archive/refs/tags/$(TENSORFLOW_TAG).zip
TENSORFLOW_ARCHIVE = $(TENSORFLOW_TAG).zip

TENSORFLOW_DIR = tflite-micro-spresense-examples-$(TENSORFLOW_TAG)
TENSORFLOW_MAKEFILE = tensorflow$(DELIM)lite$(DELIM)micro$(DELIM)tools$(DELIM)make$(DELIM)Makefile
TENSORFLOW_OPTKERNEL = cmsis_nn
TENSORFLOW_TARGET = spresense
TENSORFLOW_LIBOUT = $(TENSORFLOW_DIR)$(DELIM)gen
SPRESENSE_MAKEDEF = $(TOPDIR)$(DELIM)Make.defs
SPRESENSE_TF_C_RUNTIME = $(CUR_DIR)$(DELIM)c-runtime$(DELIM)Runtime.mk

TF_MAKECMD := make OPTIMIZED_KERNEL_DIR=$(TENSORFLOW_OPTKERNEL) \
					TARGET=$(TENSORFLOW_TARGET) \
					SPRESENSE_DEFS=$(SPRESENSE_MAKEDEF) \
					SPRESENSE_CONFIG_H=$(TOPDIR)$(DELIM)include$(DELIM)nuttx$(DELIM)config.h \
					SPRESENSE_CURDIR=$(CUR_DIR) \
					SPRESENSE_APP_TFMAKE=$(SPRESENSE_TF_C_RUNTIME)

context:: $(TENSORFLOW_DIR)

clean:: tensorflow_clean

distclean:: delete_tensorflow

$(TENSORFLOW_DIR):
	$(Q) if [ ! -d $(TENSORFLOW_DIR) ]; then \
		$(WGET) -O $(TENSORFLOW_ARCHIVE) $(TENSORFLOW_URL); \
		$(UNPACK) $(TENSORFLOW_ARCHIVE); \
		$(RM) $(TENSORFLOW_ARCHIVE); \
		cd $(CUR_DIR); \
	fi

$(BIN): $(TENSORFLOW_DIR)
	$(Q) cd $(TENSORFLOW_DIR); $(TF_MAKECMD)
	$(Q) cp $(TENSORFLOW_LIBOUT)$(DELIM)$(BIN) $(CUR_DIR)

spresense_audio_provider.cxx: $(BIN)
	$(Q) touch spresense_audio_provider.cxx

tf_wrapper_audio_util.cxx: $(BIN)
	$(Q) touch tf_wrapper_audio_util.cxx

spresense_image_provider.c: $(BIN)
	$(Q) touch spresense_image_provider.c

dummy_src.c: $(BIN)
	$(Q) touch dummy_src.c

tensorflow_clean:
	$(Q) if [ -d $(TENSORFLOW_DIR) ]; then \
		cd $(TENSORFLOW_DIR); $(TF_MAKECMD) clean; \
	fi
	$(Q) rm -f dummy_src.c

delete_tensorflow: tensorflow_clean
	$(Q) if [ -d $(TENSORFLOW_DIR) ]; then \
		rm -rf $(TENSORFLOW_DIR); \
	fi

include $(APPDIR)$(DELIM)Application.mk
