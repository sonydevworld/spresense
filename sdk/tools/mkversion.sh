#!/bin/bash
############################################################################
# tools/mkversion.sh
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

TAG=${1:-HEAD}

if [ "X${TOPDIR}" = "X" ]; then
    echo "Please specify a TOPDIR variable"
    exit 1
fi

APP_VERSION="0.0.0"
SDK_VERSION="SDK1.4.0"
if [ -r sdk_version ]; then
    SDK_VERSION="SDK`cat sdk_version`"
fi

NUTTX_VERSION="7.22"

# Get short hash for specified tag
GIT_REVISION=`git rev-parse ${TAG} | cut -b -7`

if [ "${GIT_REVISION}" != "" ]; then
    BUILD_ID="${APP_VERSION}-${SDK_VERSION}-${GIT_REVISION}"
else
    BUILD_ID="${APP_VERSION}-${SDK_VERSION}"
fi

# BUILD_ID must be 40 characters or less
if [ ${#BUILD_ID} -gt 40 ]; then
    echo "BUILD_ID too long! ${BUILD_ID}"
    exit 1
fi

${TOPDIR}/tools/version.sh -v ${NUTTX_VERSION} -b "${BUILD_ID}" ${TOPDIR}/.version
