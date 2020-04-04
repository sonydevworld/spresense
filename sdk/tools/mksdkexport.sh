#!/bin/bash
# tools/mksdkexport.sh
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

CURRENT_DIR=`pwd`
SCRIPT_NAME=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)/$(basename "${BASH_SOURCE[0]}")
SCRIPT_DIR=`dirname "$SCRIPT_NAME"`

# function   : show_help
# description: Show help and exit.
function show_help()
{
	echo "  Usage:"
	echo "    $0 -i <NuttX exported zip archive> -o <output tar archive>"
	echo ""
	echo "  Optional arguments:"
	echo "       -i: Input nuttx exported zip archive file (default: nuttx-export.zip)"
	echo "       -o: Output tar archive file (default: sdk-export.tar.gz)"
	echo "       -h: Show help (This message)"
	exit
}

# Option handler
# -i: Exported nuttx zip archive (default: nuttx-export.zip)
# -o: SDK export output tar archive file (default: sdk-export.tar.gz)
# -h: Show Help
NUTTX_EXPORT="nuttx-export.zip"
SDK_EXPORT="sdk-export.tar.gz"
SDK_DIR=`cd ${SCRIPT_DIR}/.. && pwd`
while getopts i:o:h OPT
do
	case $OPT in
		'i' ) NUTTX_EXPORT=$OPTARG;;
		'o' ) SDK_EXPORT=$OPTARG;;
		'h' ) show_help;;
	esac
done

SDK_EXP_ROOT=sdk-export
SDK_EXP_NUTTX=${SDK_EXP_ROOT}/nuttx
SDK_EXP_SDK=${SDK_EXP_ROOT}/sdk

# Check exported nuttx archive
if [ ! -f ${NUTTX_EXPORT} ]; then
	echo "Error: Exported nuttx archive ${NUTTX_EXPORT} not found."
	exit -1;
fi

# Create temporary directory
TMP_DIR=`mktemp -d`

# Extract exported nuttx archive
unzip -d ${TMP_DIR} ${NUTTX_EXPORT} > /dev/null

# Change exported file structure
# -- sdk-export
#    |-- nuttx   : Exported NuttX
#    |-- sdk     : SDK extended files
#    |-- LICENSE : Spresense SDK license file

mkdir -p ${TMP_DIR}/${SDK_EXP_ROOT}

# Move exported nuttx
mv ${TMP_DIR}/nuttx-export* ${TMP_DIR}/${SDK_EXP_NUTTX}

# Copy SDK modules header files
mkdir -p ${TMP_DIR}/${SDK_EXP_SDK}/modules
cp -a ${SDK_DIR}/modules/include ${TMP_DIR}/${SDK_EXP_SDK}/modules/

# Copy license file
cp -a ${SDK_DIR}/../LICENSE ${TMP_DIR}/${SDK_EXP_ROOT}/

# Archive files
tar czf ${SDK_EXPORT} -C ${TMP_DIR} ${SDK_EXP_ROOT}

# Remove temporary directory
rm -rf ${TMP_DIR}

echo "Done."
