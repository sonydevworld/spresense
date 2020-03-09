############################################################################
# externals/nnablart/Makefile
#
#   Copyright 2020 Sony Semiconductor Solutions Corporation
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

-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

BIN = libnnablart$(LIBEXT)

RUNTIMEDIR = nnabla-c-runtime
SRCDIR = $(RUNTIMEDIR)/src

CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(RUNTIMEDIR)/include}
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(SRCDIR)/functions}
CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(SRCDIR)/runtime}

# Runtime patch for missing fminf, fmaxf math functions in NuttX libm.

ifeq ($(CONFIG_LIBM),y)
CFLAGS += -include fminf_fmaxf.h
endif
CFLAGS += -std=c99

# Don't follow the -O. option in CFLAGS

CFLAGS := $(patsubst -O%,-O3,$(CFLAGS))

CSRCS    =   accessor.c list.c shape.c
ROOTDEPPATH  = --dep-path $(SRCDIR)/functions/utilities
VPATH    = $(SRCDIR)/functions/utilities

CSRCS   += sigmoid.c relu.c tanh.c softmax.c selu.c elu.c prelu.c leakyrelu.c
CSRCS   += crelu.c celu.c swish.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/activation
VPATH   += $(SRCDIR)/functions/implements/activation

CSRCS   += abs.c batch_matmul.c exp.c identity.c log.c round.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/math
VPATH   += $(SRCDIR)/functions/implements/math

CSRCS   += binary_tanh.c binary_sigmoid.c binary_connect_affine.c binary_weight_affine.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/quantization
VPATH   += $(SRCDIR)/functions/implements/quantization

CSRCS   += add_scalar.c arithmetic.c arithmetic_generic.c div2.c mul2.c mul_scalar.c
CSRCS   += pow2.c pow_scalar.c r_div_scalar.c r_pow_scalar.c r_sub_scalar.c
CSRCS   += sub2.c add2.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/arithmetic
VPATH   += $(SRCDIR)/functions/implements/arithmetic

CSRCS   += maximum_scalar.c minimum_scalar.c maximum2.c minimum2.c sign.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/logical
VPATH   += $(SRCDIR)/functions/implements/logical

CSRCS   += matrix_diag.c matrix_diag_part.c reshape.c concatenate.c split.c stack.c
CSRCS   += shift.c slice.c flip.c transpose.c pad.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/array
VPATH   += $(SRCDIR)/functions/implements/array

CSRCS   += pooling.c max_pooling.c sum_pooling.c average_pooling.c unpooling.c deconvolution.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/neural_network
VPATH   += $(SRCDIR)/functions/implements/neural_network

CSRCS   += affine.c affine_generic.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/neural_network/affine
VPATH   += $(SRCDIR)/functions/implements/neural_network/affine

CSRCS   += convolution.c convolution_generic.c convolution_float.c convolution_common.c
CSRCS   += binary_connect_convolution.c binary_weight_convolution.c depthwise_convolution.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/neural_network/convolution
VPATH   += $(SRCDIR)/functions/implements/neural_network/convolution

CSRCS   += batch_normalization.c mean_subtraction.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/normalization
VPATH   += $(SRCDIR)/functions/implements/normalization

CSRCS   += dropout.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/stochasticity
VPATH   += $(SRCDIR)/functions/implements/stochasticity

CSRCS   += sum.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements/reduction
VPATH   += $(SRCDIR)/functions/implements/reduction

CSRCS   += unimplemented.c
ROOTDEPPATH += --dep-path $(SRCDIR)/functions/implements
VPATH   += $(SRCDIR)/functions/implements

CSRCS += function_context.c runtime.c runtime_internal.c
ROOTDEPPATH += --dep-path $(SRCDIR)/runtime
VPATH   += $(SRCDIR)/runtime

include $(APPDIR)/Application.mk
