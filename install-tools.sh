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

SPRBASENAME=spresenseenv

NXTOOLURL=https://bitbucket.org/nuttx/tools
NXTOOL=nuttx-tools
NXTOOLARCHIVE=${NXTOOL}.tar.gz
GENROMFS=genromfs-0.5.2

CROSSBASEURL=https://developer.arm.com/-/media/Files/downloads/gnu-rm
CROSSTOOLDIR=10.3-2021.10
CROSSTOOLFILE=gcc-arm-none-eabi-10.3-2021.10

# MD5 check sums
TOOLCHAINSUM_win=2bc8f0c4c4659f8259c8176223eeafc1
TOOLCHAINSUM_mac=7f2a7b7b23797302a9d6182c6e482449
TOOLCHAINSUM_linux_x86_64=2383e4eb4ea23f248d33adc70dc3227e
TOOLCHAINSUM_linux_aarch64=3fe3d8bb693bd0a6e4615b6569443d0d

OPENOCDBASEURL=https://github.com/sonydevworld/spresense-openocd-prebuilt/releases/download
OPENOCDRELEASE=v0.11.0-spr1
OPENOCDVERDATE=openocd-0.11.0-spr1-20230313

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
    local _packages="git gperf libncurses5 libncurses6 libncurses5-dev libncurses-dev flex bison genromfs pkg-config autoconf automake curl make minicom unzip"
    local _needed
    for p in ${_packages}; do
        if LANG=C apt-cache policy $p | grep -q "Candidate:"; then
            dpkg -s $p >/dev/null 2>&1 || _needed="${_needed} ${p}"
        fi
    done
    
    if [ -n "${_needed}" ]; then
        sudo apt update
        sudo apt install -y ${_needed}
    fi
}

win_install_tools()
{
    pacman -S --noconfirm --needed base-devel gcc gperf git python3 ncurses-devel unzip tio vim 2>/dev/null
}

mac_install_tools()
{
    :  # do nothing
}

wsl_install_tools()
{
    linux_install_tools # XXX: We expects that Ubuntu runs on WSL
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

    local _fn
    local _mach=`uname -m 2>/dev/null`

    if [ "${_mach}" != "x86_64" ] && [ "${_mach}" != "aarch64" ]; then
        echo Sorry, this machine ${_mach} is not supported.
        return
    fi

    _fn=${CROSSTOOLFILE}-${_mach}-linux.tar.bz2

    download ${CROSSBASEURL}/${CROSSTOOLDIR}/${_fn} ${_fn}

    local _csum=TOOLCHAINSUM_linux_${_mach}
    echo ${!_csum} ${_fn} > .sum
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
    local _target=${1:-${SPRROOT}/usr}

    # Check the cross toolchain already exists

    [ -e ${_target}/bin/arm-none-eabi-gcc ] && return

    local _fn=${CROSSTOOLFILE}-win32.zip

    download ${CROSSBASEURL}/${CROSSTOOLDIR}/${_fn} ${_fn}

    run_progress unzip ${_fn}

    mkdir -p ${_target}
    cp -ar ${CROSSTOOLFILE}/* ${_target}
    rm -rf ${CROSSTOOLFILE}
}

mac_install_toolchain()
{
    # Check the cross toolchain already exists

    [ -e ${SPRROOT}/usr/bin/arm-none-eabi-gcc ] && return

    # Download cross toolchain

    local _fn=${CROSSTOOLFILE}-mac.tar.bz2

    download ${CROSSBASEURL}/${CROSSTOOLDIR}/${_fn} ${_fn}

    local _sum=`md5 -q ${_fn}`
    if [ "${_sum}" != "${TOOLCHAINSUM_mac}" ]; then
        echo "Archive validation failed."
        exit 1
    fi

    # Extract toolchain directly

    run_progress tar vjxf ${_fn} --strip-components=1 -C ${SPRROOT}/usr
}

wsl_install_toolchain()
{
    linux_install_toolchain
}

linux_install_openocd()
{
    local _fn=${OPENOCDVERDATE}
    local _sha
    local _target=${1:-${SPRROOT}/usr}

    local _mach=`uname -m 2>/dev/null`

    if [ "${_mach}" == "x86_64" ] || [ "${_mach}" == "aarch64" ]; then
        _fn=${_fn}-linux64.tar.bz2
    else
        _fn=${_fn}-linux32.tar.bz2
    fi
    _sha=${_fn}.sha

    download ${OPENOCDBASEURL}/${OPENOCDRELEASE}/${_fn} ${_fn}
    download ${OPENOCDBASEURL}/${OPENOCDRELEASE}/${_sha} ${_sha}

    shasum -c ${_sha} || exit 1

    run_progress tar vjxf ${_fn}  --strip-components=1 -C ${SPRROOT}/usr || exit 1
}

win_install_openocd()
{
    local _fn=${OPENOCDVERDATE}
    local _sha=${_fn}.sha
    local _target=${1:-${SPRROOT}/usr}

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
    mkdir -p ${_target}
    cp -ar "${OPENOCDVERDATE}/"* ${_target} || exit 1
    rm -rf "${OPENOCDVERDATE}"
}

mac_install_openocd()
{
    local _fn=${OPENOCDVERDATE}-macosx.tar.bz2
    local _sha=${_fn}.sha
    local _target=${1:-${SPRROOT}/usr}

    download ${OPENOCDBASEURL}/${OPENOCDRELEASE}/${_fn} ${_fn}
    download ${OPENOCDBASEURL}/${OPENOCDRELEASE}/${_sha} ${_sha}

    shasum -c ${_sha} || exit 1

    run_progress tar vjxf ${_fn}  --strip-components=1 -C ${SPRROOT}/usr
}

wsl_install_openocd()
{
    win_install_openocd ${SPRROOT}/windows
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

ARCH=`uname -m 2>/dev/null`
# 32bit linux is not supported
if [ "${OS}" == "linux" ]; then
    if [ "${ARCH}" != "x86_64" ] && [ "${ARCH}" != "aarch64" ]; then
        echo Sorry, this platform is not supported.
        exit 1
    fi

    if [ "$(grep enabled /proc/sys/fs/binfmt_misc/WSLInterop 2>/dev/null)" = "enabled" ]; then
        OS="wsl"
    fi
fi

# Switch SPRROOT by platform
if [ "${OS}" == "win" ]; then
    SPRROOT=/opt/${SPRBASENAME}
else
    SPRROOT=${HOME}/${SPRBASENAME}
fi

# Re-install

if [ -n "$REINSTALL" ]; then
    rm -rf ${SPRROOT}
    rm -rf *.zip *.tar.* *.tgz
fi

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

# On WSL, we need to symbolic links to .exe files and add x permission

if [ "$OS" = "wsl" ]; then
    cd ${SPRROOT}/windows/bin
    for f in *
    do
        chmod +x $f 2>/dev/null
        ln -sf $f $(basename $f .exe) 2>/dev/null
    done
    cd - >/dev/null 2>&1
fi

# Create PATH environment setup support script

_setup=${HOME}/${SPRBASENAME}/setup
mkdir -p ${HOME}/${SPRBASENAME}

echo "#!/usr/bin/env bash" > ${_setup}
echo "PATH=${SPRROOT}/usr/bin:"'${PATH}' >> ${_setup}
echo "export PATH" >> ${_setup}

echo "Installation is done."
