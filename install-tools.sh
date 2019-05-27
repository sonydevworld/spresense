#!/usr/bin/env bash
############################################################################
# install-tools.sh
#
#   Copyright 2019 Sony Semiconductor Solutions Corporation
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

#set -x

SPRROOT=$HOME/spresenseenv

NXTOOLURL=https://bitbucket.org/nuttx/tools
NXTOOL=nuttx-tools
NXTOOLARCHIVE=${NXTOOL}.tar.gz
GENROMFS=genromfs-0.5.2

CROSSBASEURL=https://developer.arm.com/-/media/Files/downloads/gnu-rm
CROSSTOOLDIR=7-2018q2
CROSSTOOLFILE=gcc-arm-none-eabi-7-2018-q2-update

# MD5 check sums
TOOLCHAINSUM_win=bc8ae26d7c429f30d583a605a4bcf9bc
TOOLCHAINSUM_mac=a66be9828cf3c57d7d21178e07cd8904
TOOLCHAINSUM_linux=299ebd3f1c2c90930d28ab82e5d8d6c0

OPENOCDBASEURL=https://github.com/gnu-mcu-eclipse/openocd/releases/download
OPENOCDRELEASE=v0.10.0-12-20190422
OPENOCDVERDATE=0.10.0-12-20190422-2015
OPENOCDFILE=gnu-mcu-eclipse-openocd-${OPENOCDVERDATE}

VERBOSE=

# download <URL> <output file name>
download()
{
    # No download again

    [ -e $2 ] && return

    echo "=== Download $2"
    curl -L $1 > $2

    if [ "$?" -ne "0" ]; then
	echo "Download $2 failed."
	rm -f $2  # remove broken file
	exit 1
    fi
}

run_progress()
{
    if [ -n "$VERBOSE" ]; then
        $*
    else
        echo -n "=== $*  "
        $* 2>&1 | awk '
            BEGIN {ORS=""}
            NR % 10 == 0 { print "."; fflush() }
            END { print "\n"}'
    fi
    if [ "$?" -ne "0" ]; then
        echo command "$*" failed.
        exit 1
    fi
}

# Base command line tools installation for each systems

linux_install_tools()
{
    local _packages="git gperf libncurses5-dev flex bison genromfs pkg-config autoconf automake curl"
    local _needed
    for p in ${_packages}; do
        dpkg -s $p >/dev/null 2>&1 || _needed="${_needed} ${p}"
    done
    
    if [ -n "${_needed}" ]; then
        sudo apt install -y ${_needed}
    fi
}

win_install_tools()
{
    pacman -S --noconfirm --needed base-devel gcc git python3 ncurses-devel unzip 2>/dev/null
}

mac_install_tools()
{
    :  # do nothing
}

install_nxtools()
{
    local _kconfig=${SPRROOT}/usr/bin/kconfig-conf
    local _genromfs=${SPRROOT}/usr/bin/genromfs

    # Skip when both of kconfig and genromfs are already installed

    [ -e ${_kconfig} -a -e ${_genromfs} ] && return

    # Download and install kconfig-frontends

    download ${NXTOOLURL}/get/master.tar.gz ${NXTOOLARCHIVE}

    rm -rf ${NXTOOL}
    mkdir ${NXTOOL}
    run_progress tar xvzf ${NXTOOLARCHIVE} --strip-components=1 -C ${NXTOOL}

    local _confopts="--prefix=${SPRROOT}/usr --disable-shared --disable-nconf"
    if [ ! -e ${_kconfig} ]; then
        cd ${NXTOOL}/kconfig-frontends
        run_progress ./configure ${_confopts}
        run_progress make install
        cd - >/dev/null 2>&1
    fi

    # And install genromfs except linux

    if [ "${OS}" != "linux" -a ! -e ${_genromfs} ]; then
        rm -rf ${GENROMFS}
        tar xzf ${NXTOOL}/${GENROMFS}.tar.gz || exit 1
        cd ${GENROMFS}
        run_progress make install PREFIX=${SPRROOT}
        cd - >/dev/null 2>&1
    fi

    # Clean source files

    rm -rf ${NXTOOL} ${GENROMFS}
}

linux_install_toolchain()
{
    # Check the cross toolchain already exists

    [ -e ${SPRROOT}/usr/bin/arm-none-eabi-gcc ] && return

    # Download cross toolchain

    local _fn=${CROSSTOOLFILE}-${OS}.tar.bz2

    download ${CROSSBASEURL}/${CROSSTOOLDIR}/${_fn} ${_fn}

    echo ${TOOLCHAINSUM_linux} ${_fn} > .sum
    eval md5sum -c .sum
    rm -f .sum
    if [ "$?" -ne "0" ]; then
        echo "Archive validation failed."
        exit 1
    fi

    # Extract toolchain directly

    run_progress tar vjxf ${_fn} --strip-components=1 -C ${SPRROOT}/usr
}

win_install_toolchain()
{
    # Check the cross toolchain already exists

    [ -e ${SPRROOT}/usr/bin/arm-none-eabi-gcc ] && return

    local _fn=${CROSSTOOLFILE}-win32.zip

    download ${CROSSBASEURL}/${CROSSTOOLDIR}/${_fn} ${_fn}

    run_progress unzip -o -d ${SPRROOT}/usr ${_fn}
}

mac_install_toolchain()
{
    # Check the cross toolchain already exists

    [ -e ${SPRROOT}/usr/bin/arm-none-eabi-gcc ] && return

    # Download cross toolchain

    local _fn=${CROSSTOOLFILE}-${OS}.tar.bz2

    download ${CROSSBASEURL}/${CROSSTOOLDIR}/${_fn} ${_fn}

    local _sum=`md5 -q ${_fn}`
    if [ "${_sum}" != "${TOOLCHAINSUM_mac}" ]; then
        echo "Archive validation failed."
        exit 1
    fi

    # Extract toolchain directly

    run_progress tar vjxf ${_fn} --strip-components=1 -C ${SPRROOT}/usr
}

linux_install_openocd()
{
    local _fn=${OPENOCDFILE}
    local _sha

    [ -e ${SPRROOT}/usr/bin/openocd ] && return
	
    if [ "`uname -m 2>/dev/null`" = "x86_64" ]; then
        _fn=${_fn}-centos64.tgz
    else
        _fn=${_fn}-centos32.tgz
    fi
    _sha=${_fn}.sha

    download ${OPENOCDBASEURL}/${OPENOCDRELEASE}/${_fn} ${_fn}
    download ${OPENOCDBASEURL}/${OPENOCDRELEASE}/${_sha} ${_sha}

    shasum -c ${_sha} || exit 1

    run_progress tar vzxf ${_fn}  --strip-components=3 -C ${SPRROOT}/usr || exit 1
}

win_install_openocd()
{
    local _fn=${OPENOCDFILE}
    local _sha=${_fn}.sha

    [ -e ${SPRROOT}/usr/bin/openocd ] && return
	
    if [ "`uname -m 2>/dev/null`" = "x86_64" ]; then
        _fn=${_fn}-win64.zip
    else
        _fn=${_fn}-win32.zip
    fi
    _sha=${_fn}.sha

    download ${OPENOCDBASEURL}/${OPENOCDRELEASE}/${_fn} ${_fn}
    download ${OPENOCDBASEURL}/${OPENOCDRELEASE}/${_sha} ${_sha}

    sha256sum -c ${_sha} || exit 1

    # Unfortunately, zip can't change archived paths, and GNU MCU Eclipse OpenOCD
    # directory depends on the date time. So extract archive to temporary directory,
    # and move from appropriate directory level to SPRROOT.

    run_progress unzip -o  ${_fn}
    cp -ar "GNU MCU Eclipse/OpenOCD/${OPENOCDVERDATE}/"* ${SPRROOT}/usr || exit 1
    rm -rf "GNU MCU Eclipse"
}

mac_install_openocd()
{
    local _fn=${OPENOCDFILE}-macos.tgz
    local _sha=${_fn}.sha

    [ -e ${SPRROOT}/usr/bin/openocd ] && return
	
    download ${OPENOCDBASEURL}/${OPENOCDRELEASE}/${_fn} ${_fn}
    download ${OPENOCDBASEURL}/${OPENOCDRELEASE}/${_sha} ${_sha}

    shasum -c ${_sha} || exit 1

    run_progress tar vzxf ${_fn}  --strip-components=3 -C ${SPRROOT}/usr
}

while [ ! -z "$*" ]
do
    case "$1" in
        -r)
            REINSTALL=y
            ;;
        -v)
            VERBOSE=y
            ;;
	    *)
            break
            ;;
    esac
    shift
done

if [ -n "$REINSTALL" ]; then
    rm -rf ${SPRROOT}
    rm -rf *.zip *.tar.* *.tgz
fi

case "`uname -s`" in
    Linux)
        OS=linux
        ;;
    MSYS*)
        OS=win
        ;;
    Darwin)
        OS=mac
        ;;
    *)
        echo Sorry, this platform is not supported.
        exit 1
        ;;
esac

# Setup base tools for current system

echo "== Install base command line tools"
${OS}_install_tools

# Install kconfig-frontends

echo "== Install additional tools"
install_nxtools

# Install cross toolchain

echo "== Install cross toolchain"
${OS}_install_toolchain

# Install openocd prebuilt binary

${OS}_install_openocd
# TODO: Download config for CXD5602 into ${SPRROOT}/usr/scripts/target

# Create PATH environment setup support script

_setup=${SPRROOT}/setup

echo "#!/usr/bin/env bash" > ${_setup}
echo "PATH=${SPRROOT}/usr/bin:"'${PATH}' >> ${_setup}
echo "export PATH" >> ${_setup}

echo "Installation is done."
