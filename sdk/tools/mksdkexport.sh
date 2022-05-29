#!/usr/bin/env bash
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

# Copy SDK include header files
mkdir -p ${TMP_DIR}/${SDK_EXP_SDK}
cp -a ${SDK_DIR}/include ${TMP_DIR}/${SDK_EXP_SDK}/

# Copy SDK modules header files
mkdir -p ${TMP_DIR}/${SDK_EXP_SDK}/modules
cp -a ${SDK_DIR}/modules/include ${TMP_DIR}/${SDK_EXP_SDK}/modules/

# Copy SDK apps header files
mkdir -p ${TMP_DIR}/${SDK_EXP_SDK}/apps
cp -a ${SDK_DIR}/apps/include ${TMP_DIR}/${SDK_EXP_SDK}/apps/

# Copy SDK system header files
mkdir -p ${TMP_DIR}/${SDK_EXP_SDK}/system
cp -a ${SDK_DIR}/system/include ${TMP_DIR}/${SDK_EXP_SDK}/system/

# Copy External header files
mkdir -p ${TMP_DIR}/${SDK_EXP_SDK}/externals/include

# CMSIS header files
mkdir -p ${TMP_DIR}/${SDK_EXP_SDK}/externals/include/cmsis
cp -a ${SDK_DIR}/../externals/cmsis/CMSIS_5/CMSIS/Core/Include/* ${TMP_DIR}/${SDK_EXP_SDK}/externals/include/cmsis/
cp -a ${SDK_DIR}/../externals/cmsis/CMSIS_5/CMSIS/DSP/Include/* ${TMP_DIR}/${SDK_EXP_SDK}/externals/include/cmsis/
cp -a ${SDK_DIR}/../externals/cmsis/CMSIS_5/CMSIS/NN/Include/* ${TMP_DIR}/${SDK_EXP_SDK}/externals/include/cmsis/

# MBEDTLS header files
mkdir -p ${TMP_DIR}/${SDK_EXP_SDK}/externals/include/mbedtls
cp -a ${SDK_DIR}/../externals/alt_stubs/mbedtls/include/mbedtls/* ${TMP_DIR}/${SDK_EXP_SDK}/externals/include/mbedtls/

# Add external library objects into libapps.a
CROSSDEV=arm-none-eabi-
ARTMPDIR=`mktemp -d`
AR=${CROSSDEV}ar
STRIP=${CROSSDEV}strip

LIBLIST=(\
	${SDK_DIR}/../externals/cmsis/dsp/libarm_cortexM4lf_math.a\
	${SDK_DIR}/../externals/cmsis/nn/libcmsis_nn.a\
	${SDK_DIR}/../externals/sslutils/libsslutils.a\
	${SDK_DIR}/modules/sensing/gnss/cxd56nmea/libcxd56nmea.a\
	${SDK_DIR}/modules/libmodules.a\
)

for lib in ${LIBLIST[@]}
do
	if [ ! -f "${lib}" ]; then
		continue
	fi

	# Get some shorter names for the library
	libname=`basename ${lib} .a`
	shortname=`echo ${libname} | sed -e "s/^lib//g"`

	# Copy the application library unmodified
	echo "Merging archive: `basename ${lib} .a`"

	# Create a temporary directory and extract all of the objects there
	# Hmmm.. this probably won't work if the archiver is not 'ar'
	cd "${ARTMPDIR}"
	${AR} x "${lib}"

	# Rename each object file (to avoid collision when they are combined)
	# and add the file to libnuttx
	for file in `${AR} t ${lib} | sed -e "s/[\r\n]\+//g"`; do
		mv "${file}" "${shortname}-${file}"
		${AR} rcs "${TMP_DIR}/${SDK_EXP_NUTTX}/libs/libapps.a" "${shortname}-${file}"
	done

	# Remove all of extracted object files
	rm -f *.o

	cd - > /dev/null
done

# Remove debugging symbols
for lib in ${TMP_DIR}/${SDK_EXP_NUTTX}/libs/lib*
do
	${STRIP} -g "${lib}"
done

# Remove .gitignore files
find ${TMP_DIR}/${SDK_EXP_ROOT} -name .gitignore -delete

# Copy license file
cp -a ${SDK_DIR}/../LICENSE ${TMP_DIR}/${SDK_EXP_ROOT}/

# Archive files
tar czf ${SDK_EXPORT} -C ${TMP_DIR} ${SDK_EXP_ROOT}

# Remove temporary directory
rm -rf ${TMP_DIR}

echo "Done."
