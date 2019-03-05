#!/bin/bash
# tools/mkexport.sh
#
#   Copyright (C) 2011-2012, 2014, 2016 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
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

# Get the input parameter list

USAGE="USAGE: $0 [-d] [-z] -t <top-dir> -s <sdk-dir> [-x <lib-ext>] -l \"lib1 [lib2 [lib3 ...]]\""
unset TOPDIR
unset LIBLIST
unset TGZ
USRONLY=n
LIBEXT=.a

while [ ! -z "$1" ]; do
	case $1 in
		-d )
			set -x
			;;
		-l )
			shift
			LIBLIST=$1
			;;
		-t )
			shift
			TOPDIR=$1
			;;
		-s )
			shift
			SDKDIR=$1
			;;
		-u )
			USRONLY=y
			;;
		-w | -wn | -wy)
			# TODO: ignore now
			;;
		-x )
			shift
			LIBEXT=$1
			;;
		-z )
			TGZ=y
			;;
		-h )
			echo $USAGE
			exit 0
			;;
		* )
			echo "Unrecognized argument: $1"
			echo $USAGE
			exit 1
			;;
		esac
	shift
done

# Check arguments

if [ -z "${TOPDIR}" -o -z "${LIBLIST}" ]; then
	echo "MK: Missing required arguments"
	echo $USAGE
	exit 1
fi

if [ ! -d "${TOPDIR}" ]; then
	echo "MK: Directory ${TOPDIR} does not exist"
	exit 1
fi

# Check configuration
# Verify that we have Make.defs, .config, and .version files.

if [ ! -f "${TOPDIR}/Make.defs" ]; then
	echo "MK: Directory ${TOPDIR}/Make.defs does not exist"
	exit 1
fi

if [ ! -f "${TOPDIR}/include/nuttx/config.h" ]; then
	echo "MK: File ${TOPDIR}/include/nuttx/config.h does not exist"
	exit 1
fi

if [ ! -f "${TOPDIR}/.version" ]; then
	echo "MK: File ${TOPDIR}/.version does not exist"
	exit 1
fi

if [ ! -f "${SDKDIR}/.config" ]; then
	echo "MK: File ${SDKDIR}/.config does not exist"
	exit 1
fi

# Check if the make environment variable has been defined

if [ -z "${MAKE}" ]; then
	MAKE=`which make`
fi

# Get the version string

#source "${SDKDIR}/.version"
#if [ ! -z "${CONFIG_VERSION_STRING}" -a "${CONFIG_VERSION_STRING}" != "0.0" ]; then
#	VERSION="-${CONFIG_VERSION_STRING}"
#fi
if [ -f "${SDKDIR}"/sdk_version ]; then
	VERSION="-`cat ${SDKDIR}/sdk_version`"
fi

# Check whether the environment is Cygwin and create the export directory

if [ $(uname -o 2>/dev/null) = "Cygwin" ]; then
	TMPDIR=`mktemp -d | cygpath -m -f -`
else
	TMPDIR=`mktemp -d`
fi
OUTPUTDIR="sdk-export${VERSION}"
EXPORTDIR="${TMPDIR}/${OUTPUTDIR}"
EXPORTNXDIR="${EXPORTDIR}/nuttx"
EXPORTSDKDIR="${EXPORTDIR}/sdk"

# If the export directory already exists, then remove it and create a new one

if [ -d "${EXPORTDIR}" ]; then
	echo "MK: Removing old export directory"
	rm -rf "${EXPORTDIR}"
fi

# Remove any possible previous results

rm -f "${OUTPUTDIR}.tar"
rm -f "${OUTPUTDIR}.zip"
rm -f "${OUTPUTDIR}.tar.gz"

# Create the export directory and some of its subdirectories

mkdir "${EXPORTDIR}" || { echo "MK: 'mkdir ${EXPORTDIR}' failed"; exit 1; }
mkdir "${EXPORTNXDIR}" || { echo "MK: 'mkdir ${EXPORTNXDIR}' failed"; exit 1; }
mkdir "${EXPORTNXDIR}/startup" || { echo "MK: 'mkdir ${EXPORTNXDIR}/startup' failed"; exit 1; }
mkdir "${EXPORTNXDIR}/build" || { echo "MK: 'mkdir ${EXPORTNXDIR}/build' failed"; exit 1; }
mkdir "${EXPORTNXDIR}/arch" || { echo "MK: 'mkdir ${EXPORTNXDIR}/arch' failed"; exit 1; }
mkdir "${EXPORTNXDIR}/tools" || { echo "MK: 'mkdir ${EXPORTNXDIR}/tools' failed"; exit 1; }
mkdir "${EXPORTSDKDIR}" || { echo "MK: 'mkdir ${EXPORTSDKDIR}' failed"; exit 1; }
mkdir -p "${EXPORTSDKDIR}/bsp/include/sdk" || { echo "MK: 'mkdir ${EXPORTSDKDIR}/bsp/include/sdk' failed"; exit 1; }
mkdir -p "${EXPORTSDKDIR}/bsp/include/sdk/chip" || { echo "MK: 'mkdir ${EXPORTSDKDIR}/bsp/include/sdk/chip' failed"; exit 1; }
mkdir -p "${EXPORTSDKDIR}/modules/include" || { echo "MK: 'mkdir ${EXPORTSDKDIR}/modules/include' failed"; exit 1; }
mkdir "${EXPORTSDKDIR}/libs" || { echo "MK: 'mkdir ${EXPORTSDKDIR}/libs' failed"; exit 1; }
mkdir "${EXPORTSDKDIR}/tools" || { echo "MK: 'mkdir ${EXPORTSDKDIR}/tools' failed"; exit 1; }
mkdir -p "${EXPORTSDKDIR}/bsp/scripts" || { echo "MK: 'mkdir ${EXPORTSDKDIR}/bsp/scripts' failed"; exit 1; }
mkdir -p "${EXPORTSDKDIR}/system/builtin/registry" || { echo "MK: 'mkdir ${EXPORTSDKDIR}/system/builtin/registry' failed"; exit 1; }

# Copy the .config file

cp -a "${TOPDIR}/.config" "${EXPORTNXDIR}/.config" ||
  { echo "MK: Failed to copy ${TOPDIR}/.config to ${EXPORTNXDIR}/.config"; exit 1; }
cp -a "${SDKDIR}/.config" "${EXPORTSDKDIR}/.config" ||
  { echo "MK: Failed to copy ${SDKDIR}/.config to ${EXPORSDKTDIR}/.config"; exit 1; }

# Copy the Make.defs files, but disable windows path conversions

grep -v "WINTOOL[ \t]*=[ \t]y" "${TOPDIR}/Make.defs"  > "${EXPORTNXDIR}/Make.defs"

# Extract information from the Make.defs file.  A Makefile can do this best

${MAKE} -C "${TOPDIR}/tools" -f Makefile.export TOPDIR="${TOPDIR}" EXPORTDIR="${EXPORTNXDIR}"
source "${EXPORTNXDIR}/makeinfo.sh"
rm -f "${EXPORTNXDIR}/makeinfo.sh"
rm -f "${EXPORTNXDIR}/Make.defs"

# Verify the build info that we got from makeinfo.sh

if [ ! -d "${ARCHDIR}" ]; then
	echo "MK: Directory ${ARCHDIR} does not exist"
	exit 1
fi

# Is there a linker script in this configuration?

if [ ! -z "${LDPATH}" ]; then

	# Apparently so.  Verify that the script exists

	if [ ! -f "${LDPATH}" ]; then
		echo "MK: File ${LDPATH} does not exist"
		exit 1
	fi

	# Copy the linker script

	cp -p "${LDPATH}" "${EXPORTNXDIR}/build/." || \
		{ echo "MK: cp ${LDPATH} failed"; exit 1; }
fi

# Copy the NuttX include directory (retaining attributes and following symbolic links)

cp -LR -p "${TOPDIR}/include" "${EXPORTNXDIR}/." || \
	{ echo "MK: 'cp ${TOPDIR}/include' failed"; exit 1; }

# Copy architecture-specific header files into the arch export sub-directory.
# This is tricky because each architecture does things in a little different
# way.
#
# First copy any header files in the architecture src/ sub-directory (some
# architectures keep all of the header files there, some a few, and others
# none

cp -LR -f "${ARCHDIR}"/*.h "${EXPORTNXDIR}"/arch/. 2>/dev/null

# Copy SDK header files
cp -LR -pf "${SDKDIR}"/bsp/include/arch "${EXPORTNXDIR}"/include/.
cp -rp "${SDKDIR}"/bsp/include/sdk/*.h "${EXPORTSDKDIR}"/bsp/include/sdk/.
cp -rp "${SDKDIR}"/modules/include/* "${EXPORTSDKDIR}"/modules/include/.

# An unnecessary copy, but copying driver headers so using in Arduino library.
cp -rp "${SDKDIR}"/bsp/src/*.h "${EXPORTSDKDIR}"/bsp/include/sdk/.
cp -rp "${SDKDIR}"/bsp/src/chip/*.h "${EXPORTSDKDIR}"/bsp/include/sdk/chip/.

# Import build related tools and files

cp -f "${TOPDIR}"/tools/Config.mk "${EXPORTNXDIR}"/tools/.
cp -f "${TOPDIR}"/tools/Makefile.host "${EXPORTNXDIR}"/tools/.
cp -f "${TOPDIR}"/tools/mkdeps.c "${EXPORTNXDIR}"/tools/.
cp -f "${TOPDIR}"/tools/incdir.sh "${EXPORTNXDIR}"/tools/.
cp -f "${TOPDIR}"/tools/incdir.bat "${EXPORTNXDIR}"/tools/.
cp "${ARCHDIR}"/armv7-m/Toolchain.defs "${EXPORTNXDIR}"/build/.
cat "${TOPDIR}"/Make.defs | sed -e 's,.*/Toolchain.defs,include ${TOPDIR}/build/Toolchain.defs,' >"${EXPORTNXDIR}"/Make.defs

cp -f "${SDKDIR}"/Make.defs "${EXPORTSDKDIR}"/.
cp -f "${SDKDIR}"/bsp/scripts/Make.defs.userapps "${EXPORTSDKDIR}"/Make.defs
cp -frp "${SDKDIR}"/tools/* "${EXPORTSDKDIR}"/tools/

# Copy builtin command source
#
# This process needs to be able to user adds his application to nsh builtin command.

cp -f "${SDKDIR}"/system/builtin/Make.defs "${EXPORTSDKDIR}"/system/builtin/.
cp -f "${SDKDIR}"/system/builtin/Makefile "${EXPORTSDKDIR}"/system/builtin/Makefile
cp -f "${SDKDIR}"/system/builtin/*.c "${EXPORTSDKDIR}"/system/builtin/.
cp -f "${SDKDIR}"/system/builtin/*.h "${EXPORTSDKDIR}"/system/builtin/.
cp -f "${SDKDIR}"/system/builtin/registry/*dat "${EXPORTSDKDIR}"/system/builtin/registry/. 2>/dev/null
cp -f "${SDKDIR}"/system/builtin/registry/Makefile.external "${EXPORTSDKDIR}"/system/builtin/registry/Makefile

# Then look a list of possible places where other architecture-specific
# header files might be found.  If those places exist (as directories or
# as symbolic links to directories, then copy the header files from
# those directories into the EXPORTDIR

ARCH_HDRDIRS="arm armv7-m avr avr32 board common chip mips32"
for hdir in $ARCH_HDRDIRS; do

	# Does the directory (or symbolic link) exist?

	if [ -d "${ARCHDIR}/${hdir}" -o -h "${ARCHDIR}/${hdir}" ]; then

		# Yes.. create a export sub-directory of the same name

		mkdir "${EXPORTNXDIR}/arch/${hdir}" || \
			{ echo "MK: 'mkdir ${EXPORTNXDIR}/arch/${hdir}' failed"; exit 1; }

		# Then copy the header files (only) into the new directory

		cp -f "${ARCHDIR}"/${hdir}/*.h "${EXPORTNXDIR}"/arch/${hdir}/. 2>/dev/null

		# One architecture has low directory called "chip" that holds the
		# header files

		if [ -d "${ARCHDIR}/${hdir}/chip" ]; then

			# Yes.. create a export sub-directory of the same name

			mkdir "${EXPORTNXDIR}/arch/${hdir}/chip" || \
				{ echo "MK: 'mkdir ${EXPORTNXDIR}/arch/${hdir}/chip' failed"; exit 1; }

			# Then copy the header files (only) into the new directory

			cp -f "${ARCHDIR}"/${hdir}/chip/*.h "${EXPORTNXDIR}"/arch/${hdir}/chip/. 2>/dev/null
		fi
	fi
done

# Copy OS internal header files as well.  They are used by some architecture-
# specific header files.

mkdir "${EXPORTNXDIR}/arch/os" || \
	{ echo "MK: 'mkdir ${EXPORTNXDIR}/arch/os' failed"; exit 1; }

OSDIRS="clock environ errno group init irq mqueue paging pthread sched semaphore signal task timer wdog"

for dir in ${OSDIRS}; do
	mkdir "${EXPORTNXDIR}/arch/os/${dir}" || \
		{ echo "MK: 'mkdir ${EXPORTNXDIR}/arch/os/${dir}' failed"; exit 1; }
	cp -f "${TOPDIR}"/sched/${dir}/*.h "${EXPORTNXDIR}"/arch/os/${dir}/. 2>/dev/null
done

# Copy External CMSIS header files

mkdir "${EXPORTNXDIR}/include/cmsis" || { echo "MK: 'mkdir ${EXPORTNXDIR}/include/cmsis' failed"; exit 1; }
cp -rp "${SDKDIR}"/../externals/cmsis/CMSIS_5/CMSIS/NN/Include/* "${EXPORTNXDIR}/include/cmsis"
cp -rp "${SDKDIR}"/../externals/cmsis/CMSIS_5/CMSIS/DSP/Include/* "${EXPORTNXDIR}/include/cmsis"
cp -rp "${SDKDIR}"/../externals/cmsis/CMSIS_5/CMSIS/Core/Include/* "${EXPORTNXDIR}/include/cmsis"

# Add the board library to the list of libraries

LIBLIST="${LIBLIST} bsp/board/libboard${LIBEXT}"

# Then process each library

ARTMPDIR=`mktemp -d`
AR=${CROSSDEV}ar
STRIP=${CROSSDEV}strip
for lib in ${LIBLIST}; do
	if [ ! -f "${SDKDIR}/${lib}" ]; then
		echo "MK: Library ${SDKDIR}/${lib} does not exist"
		exit 1
	fi

	# Get some shorter names for the library

	libname=`basename ${lib} ${LIBEXT}`
	shortname=`echo ${libname} | sed -e "s/^lib//g"`

	# Copy the application library unmodified

	if [ "X${libname}" = "Xlibapps" ]; then
		cp -p "${SDKDIR}/${lib}" "${EXPORTSDKDIR}/libs/." || \
			{ echo "MK: cp ${TOPDIR}/${lib} failed"; exit 1; }
	else
		echo "Merging archive: `basename ${lib} ${LIBEXT}`"
		# Save current working directory

		PREVDIR="$PWD"

		# Create a temporary directory and extract all of the objects there
		# Hmmm.. this probably won't work if the archiver is not 'ar'

		cd "${ARTMPDIR}"
		${AR} x "${SDKDIR}/${lib}"

		# Rename each object file (to avoid collision when they are combined)
		# and add the file to libnuttx

		for file in `${AR} t ${SDKDIR}/${lib} | sed -e "s/[\r\n]\+//g"`; do
			mv "${file}" "${shortname}-${file}"
			${AR} rcs "${EXPORTSDKDIR}/libs/libsdk${LIBEXT}" "${shortname}-${file}"
		done

		# Remove debugging symbols

		${STRIP} -g "${EXPORTSDKDIR}/libs/libsdk${LIBEXT}"

		# Remove all of extracted object files

		rm -f *.o

		cd "${PREVDIR}"
	fi
done

# Now tar up the whole export directory

cd "${TMPDIR}" || \
	{ echo "MK: 'cd ${TMPDIR}' failed"; exit 1; }

if [ "X${TGZ}" = "Xy" ]; then
	tar cvf "${OUTPUTDIR}.tar" "${OUTPUTDIR}" 1>/dev/null 2>&1
	gzip -f "${OUTPUTDIR}.tar"
	cp "${OUTPUTDIR}.tar.gz" "${SDKDIR}"
else
	zip -r "${OUTPUTDIR}.zip" "${OUTPUTDIR}" 1>/dev/null 2>&1
	cp "${OUTPUTDIR}.zip" "${SDKDIR}"
fi

# Clean up after ourselves

rm -rf "${TMPDIR}"
