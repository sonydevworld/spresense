#!/usr/bin/env bash
# tools/mkmsys2deps.sh
#
#   Copyright (C) 2016 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
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
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
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

# Uncomment to enable debug options
# set -x
# DEBUG=--dep-debug

# Make sure that we know where we are

TOOLDIR=$(cd $(dirname $0) && pwd)
NXTOOLSDIR=$(cd ${TOOLDIR}/../../nuttx/tools && pwd)

if [ ! -x ${TOOLDIR}/mkmsys2deps.sh ]; then
  echo "# ERROR: tools/ directory not found"
  exit 1
fi

# Make sure that executables are ready

MKDEPS=${NXTOOLSDIR}/mkdeps.exe

if [ ! -x ${MKDEPS} ]; then
  echo "# ERROR: tools/mkdeps.exe does not exist"
  exit 1
fi

# Run the mkdeps.exe program to generate a Windows dependency file

${MKDEPS} ${DEBUG} $* | iconv -f cp932 -t utf-8

