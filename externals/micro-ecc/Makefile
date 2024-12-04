############################################################################
# externals/micro-ecc/Makefile
#
#   Copyright 2023 Sony Semiconductor Solutions Corporation
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

BIN = libuecc$(LIBEXT)

UECC = micro-ecc

CFLAGS += ${INCDIR_PREFIX}$(UECC)

CFLAGS += -D__ARM_ARCH_7M__=0
CFLAGS += -DuECC_PLATFORM=uECC_arm_thumb2
CFLAGS += -Dasm_clear=0 -Dasm_set=0 -Dasm_rshift1=0
CFLAGS += -Dasm_mmod_fast_secp160r1=0 -Dasm_mmod_fast_secp256r1=0 -Dasm_mmod_fast_secp256k1=0
CFLAGS += -D__clang_major__=0 -D__clang_minor__=0
CFLAGS += -Ddefault_RNG_defined=0
CFLAGS += -DuECC_OPTIMIZATION_LEVEL=3

CSRCS    =   uECC.c
ROOTDEPPATH  = --dep-path $(UECC)
VPATH    = $(UECC)

include $(APPDIR)/Application.mk
