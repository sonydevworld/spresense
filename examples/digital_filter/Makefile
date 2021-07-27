############################################################################
# examples/fir_filter/Makefile
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

# For "fir" command

PROGNAME = $(CONFIG_EXAMPLES_FIR_FIR_PROGNAME)
PRIORITY = $(CONFIG_EXAMPLES_FIR_FIR_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_FIR_FIR_STACKSIZE)

ASRCS =
CSRCS = wav.c
MAINSRC = fir_filter_main.c

# For "decimate" command

PROGNAME += $(CONFIG_EXAMPLES_FIR_DECIMATOR_PROGNAME)
PRIORITY += $(CONFIG_EXAMPLES_FIR_DECIMATOR_PRIORITY)
STACKSIZE += $(CONFIG_EXAMPLES_FIR_DECIMATOR_STACKSIZE)

MAINSRC += fir_decimator_main.c

# For "envelope" command

PROGNAME += $(CONFIG_EXAMPLES_ENVELOPE_PROGNAME)
PRIORITY += $(CONFIG_EXAMPLES_ENVELOPE_PRIORITY)
STACKSIZE += $(CONFIG_EXAMPLES_ENVELOPE_STACKSIZE)

MAINSRC += fir_envelope_main.c

# For "edge" command

PROGNAME += $(CONFIG_EXAMPLES_EDGE_PROGNAME)
PRIORITY += $(CONFIG_EXAMPLES_EDGE_PRIORITY)
STACKSIZE += $(CONFIG_EXAMPLES_EDGE_STACKSIZE)

MAINSRC += edge_main.c

# For "edge16" command

PROGNAME += $(CONFIG_EXAMPLES_EDGE16_PROGNAME)
PRIORITY += $(CONFIG_EXAMPLES_EDGE16_PRIORITY)
STACKSIZE += $(CONFIG_EXAMPLES_EDGE16_STACKSIZE)

MAINSRC += edge16_main.c

MODULE = $(CONFIG_EXAMPLES_DIGITAL_FILTER)

include $(APPDIR)/Application.mk
