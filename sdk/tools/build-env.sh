#!/bin/bash
############################################################################
# tools/envsetup.sh
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

CURRENT_DIR=`pwd`
SCRIPT_NAME=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)/$(basename "${BASH_SOURCE[0]}")
SCRIPT_DIR=`dirname "$SCRIPT_NAME"`

############################################################################
# Public function                                                          #
############################################################################

# Name: spresense_create_apps_root
# Note: Create application root didectory that will contain user applications.
# Usage: $ spresense_create_apps_root <application home directory>
function spresense_create_apps_root() {
	if [ "$#" != 1 ]; then
		echo "Usage: ${FUNCNAME[0]} <application home directory>"
	else
		if [ "${1:0:1}" == "/" ]; then
			SPRESENSE_HOME=${1}
		else
			SPRESENSE_HOME="$(cd ${1}; pwd)"
		fi
		if [ -d ${SPRESENSE_HOME} ]; then
			echo "Warning: Directory ${SPRESENSE_HOME} is already exists,"
			echo -n "         Overwrite makefiles ? (Y/N): "
			read input
			input=`echo ${input} | tr '[:lower:]' '[:upper:]'`
			if [ "${input}" == "Y" ]; then
				echo "Creating application directory into ${SPRESENSE_HOME}"
				cd ${SPRESENSE_SDK}/sdk
				${SCRIPT_DIR}/mkappsdir.py -f -s ${SPRESENSE_HOME} "User application"
				cd - &> /dev/null
			else
				echo "Create application home directory canceled."
			fi
		else
			echo "Creating application directory into ${SPRESENSE_HOME}"
			cd ${SPRESENSE_SDK}/sdk
			${SCRIPT_DIR}/mkappsdir.py -s ${SPRESENSE_HOME} "User application"
			cd - &> /dev/null
		fi
		# Save current variable
		_save_spresense_environment

		# Print current variable
		_print_current_spresense_environment
	fi
}

# Name: spresense_set_apps_root
# Note: Select application root didectory.
# Usage: $ spresense_set_apps_root <application home directory>
function spresense_set_apps_root() {
	if [ "$#" != 1 ]; then
		echo "Usage: ${FUNCNAME[0]} <application home directory>"
	else
		if [ "${1:0:1}" == "/" ]; then
			_SPRESENSE_HOME=${1}
		else
			_SPRESENSE_HOME="$(cd ${1}; pwd)"
		fi
		if [ -d ${_SPRESENSE_HOME} ]; then
			if [ -f ${_SPRESENSE_HOME}/Application.mk ]; then
				SPRESENSE_HOME=${_SPRESENSE_HOME}

				# Save current variable
				_save_spresense_environment
			else
				echo "Warning: Your environment(${_SPRESENSE_HOME}) doesn't have makefiles."
				echo "         Please run next command for create makefiles."
				echo "         $ spresense_create_apps_root ${_SPRESENSE_HOME}"
			fi
		else
			echo "Warning: ${_SPRESENSE_HOME} does not exist."
			echo "         Please run"
			echo "         $ spresense_create_apps_root ${_SPRESENSE_HOME}"
		fi

		# Print current variable
		_print_current_spresense_environment
	fi
}

# Name: spresense_create_app
# Note: Create user application into application root didectory.
# Usage: $ spresense_create_app <application name>
function spresense_create_app() {
	if [ "$#" != 1 ]; then
		echo "Usage: ${FUNCNAME[0]} <application name>"
	elif [ "${SPRESENSE_HOME}" == "" ]; then
		echo "Warning: Spresense user application directory is not set."
		echo "         Please run"
		echo "         $ spresense_set_apps_root <application home directory>"
	else
		cd ${SPRESENSE_SDK}/sdk
		rm -f ${SPRESENSE_HOME}/Kconfig
		./tools/mkcmd.py -c -d ${SPRESENSE_HOME} ${1}
		cd - &> /dev/null
	fi
}

# Name: spresense_config
# Note: Create user application into application root didectory.
# Usage: $ spresense_config <configuration name>...
function spresense_config() {
	cd ${SPRESENSE_SDK}/sdk
	./tools/config.py $@
	cd - &> /dev/null
}

# Name: spresense_goto_sdk
# Note: Move current directory to SDK directory.
# Usage: $ spresense_goto_sdk
function spresense_goto_sdk() {
	if [ "${SPRESENSE_SDK}" != "" ]; then
		cd ${SPRESENSE_SDK}/sdk
	else
		echo "Warning: SPRESENSE_SDK is not set."
		echo "         Please run 'source tools/envsetup.sh' again."
	fi
}

# Name: spresense_goto_apps_root
# Note: Move current directory to user application root directory.
# Usage: $ spresense_goto_apps_root
function spresense_goto_apps_root() {
	if [ "${SPRESENSE_HOME}" != "" ]; then
		cd ${SPRESENSE_HOME}
	else
		echo "Warning: SPRESENSE_HOME is not set."
		echo "         Please run 'spresense_set_apps_root' first."
	fi
}

# Name: spresense_make
# Note: Build SDK and user application same as make command.
# Usage: $ spresense_make [build options]
function spresense_make() {
	make -C ${SPRESENSE_SDK}/sdk $@
}

# Load current variable
function _load_spresense_environment() {
	if [ -f ${HOME}/.spresense_env ]; then
		source ${HOME}/.spresense_env
	fi
	export SPRESENSE_HOME
}

# Save current variable
function _save_spresense_environment() {
	# Export parameters into configuration file
	echo "SPRESENSE_HOME=${SPRESENSE_HOME}" > ${HOME}/.spresense_env
	echo "SPRESENSE_SDK=${SPRESENSE_SDK}" >> ${HOME}/.spresense_env
}

# Print current variable
function _print_current_spresense_environment() {
	echo "======================================="
	echo "   SDK_VERSION = `cat ${SPRESENSE_SDK}/sdk/tools/mkversion.sh | grep '^SDK_VERSION' | cut -d '\"' -f 2`"
	echo " NUTTX_VERSION = `cat ${SPRESENSE_SDK}/sdk/tools/mkversion.sh | grep '^NUTTX_VERSION' | cut -d '\"' -f 2`"
	echo " SPRESENSE_SDK = ${SPRESENSE_SDK}"
	echo "SPRESENSE_HOME = ${SPRESENSE_HOME}"
	echo "   GCC_VERSION = `arm-none-eabi-gcc --version | head -n 1`"
	echo "          HOST = `uname -sm`"
	echo "======================================="
}

############################################################################
# Public parameter definition                                              #
############################################################################

############################################################################
# Environment setup                                                        #
############################################################################

#
# User application setup
#

# Load current variable
_load_spresense_environment

if [ "${SPRESENSE_HOME}" == "" ]; then
	echo "Warning: Spresense user application directory is not set."
    echo "         Please run"
    echo "         $ spresense_set_apps_root <application home directory>"
elif [ ! -d ${SPRESENSE_HOME} ]; then
    echo "Warning: ${SPRESENSE_HOME} does not exist."
    echo "         Please run"
    echo "         $ spresense_create_apps_root ${SPRESENSE_HOME}"
fi

if [ -d "${SPRESENSE_HOME}" -a ! -f "${SPRESENSE_HOME}/Application.mk" ]; then
    echo "Warning: Your environment(${SPRESENSE_HOME}) doesn't have makefiles."
    echo "         Please run next command for create makefiles."
    echo "         $ spresense_create_apps_root ${SPRESENSE_HOME}"
fi

# Set repository root to SPRESENSE_SDK
export SPRESENSE_SDK=$(dirname $(dirname ${SCRIPT_DIR}))

# Print current variable
_print_current_spresense_environment

# Save current variable
_save_spresense_environment

#
# TAB completion
#

if [ "${BASH_VERSINFO[0]}" -ge "4" ]; then
	source ${SCRIPT_DIR}/completion.sh
else
	echo "Info: Your system cannot use Spresense SDK completion."
	echo "      If you want to use Spresense SDK completion,"
	echo "      please update bash version 4 or later."
fi
