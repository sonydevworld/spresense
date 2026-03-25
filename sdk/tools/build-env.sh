#!/usr/bin/env bash
############################################################################
# tools/build-env.sh
#
#   Copyright 2019, 2026 Sony Semiconductor Solutions Corporation
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

SCRIPT_NAME=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)/$(basename "${BASH_SOURCE[0]}")
SCRIPT_DIR=`dirname "$SCRIPT_NAME"`

############################################################################
# Public function                                                          #
############################################################################

# Name: spr-create-approot
# Note: Create application root directory that will contain user applications.
# Usage: $ spr-create-approot <application home directory>
function spr-create-approot() {
	if [ "$#" != 1 ]; then
		echo "Usage: ${FUNCNAME[0]} <application home directory>"
	else
		if [ "${1:0:1}" == "/" ]; then
			SPRESENSE_HOME=${1}
		else
			SPRESENSE_HOME=$(pwd -P)/${1}
		fi
		SPRESENSE_HOME_DIR=$(dirname ${SPRESENSE_HOME})
		SPRESENSE_HOME_BASE=$(basename ${SPRESENSE_HOME})
		mkdir -p ${SPRESENSE_HOME_DIR}
		SPRESENSE_HOME=$(cd ${SPRESENSE_HOME_DIR}; pwd -P)/${SPRESENSE_HOME_BASE}
		if [ -d ${SPRESENSE_HOME} ]; then
			echo "Warning: Directory ${SPRESENSE_HOME} is already exists,"
			echo -n "         Overwrite makefiles ? (Y/N): "
			read input
			input=`echo ${input} | tr '[:lower:]' '[:upper:]'`
			if [ "${input}" == "Y" ]; then
				echo "Creating application directory into ${SPRESENSE_HOME}"
				cd ${SPRESENSE_SDK}/sdk
				${SCRIPT_DIR}/mkappsdir.py -f ${SPRESENSE_HOME} "User application"
				cd - &> /dev/null
			else
				echo "Create application home directory canceled."
			fi
		else
			echo "Creating application directory into ${SPRESENSE_HOME}"
			cd ${SPRESENSE_SDK}/sdk
			${SCRIPT_DIR}/mkappsdir.py ${SPRESENSE_HOME} "User application"
			cd - &> /dev/null
		fi
		# Save current variable
		_save_spresense_environment
		_load_spresense_environment

		# Print current variable
		_print_current_spresense_environment
	fi
}

# Name: spr-set-approot
# Note: Select application root directory.
# Usage: $ spr-set-approot <application home directory>
function spr-set-approot() {
	if [ "$#" != 1 ]; then
		echo "Usage: ${FUNCNAME[0]} <application home directory>"
		# Print current variable
		_print_current_spresense_environment
	else
		if [ "${1:0:1}" == "/" ]; then
			_SPRESENSE_HOME=${1}
		else
			_SPRESENSE_HOME="$(cd ${1}; pwd)"
		fi
		if [ -d ${_SPRESENSE_HOME} ]; then
			if [ -f ${_SPRESENSE_HOME}/.sdksubdir ]; then
				SPRESENSE_HOME=${_SPRESENSE_HOME}

				# Save current variable
				_save_spresense_environment
				_load_spresense_environment
			else
				if [ -f "${SPRESENSE_HOME}/Application.mk" ]; then
					echo "Warning: Your environment(${SPRESENSE_HOME}) is created for Spresense SDK1.x version."
					echo "         Please create a new project directory with next command."
					echo "         $ spr-create-approot <new place>"
				else
					echo "Warning: Your environment(${_SPRESENSE_HOME}) doesn't have makefiles."
					echo "         Please run next command to create makefiles."
					echo "         $ spr-create-approot ${_SPRESENSE_HOME}"
				fi
				unset SPRESENSE_HOME
			fi
		else
			echo "Warning: ${_SPRESENSE_HOME} does not exist."
			echo "         Please run"
			echo "         $ spr-create-approot ${_SPRESENSE_HOME}"
		fi

		# Print current variable
		_print_current_spresense_environment
	fi
}

# Name: spr-unset-approot
# Note: Unset application root directory.
# Usage: $ spr-unset-approot
function spr-unset-approot() {
	unset SPRESENSE_HOME

	# Save current variable
	_save_spresense_environment

	# Print current variable
	_print_current_spresense_environment
}

# Name: spr-set-port
# Note: Set serial port
# Usage: $ spr-set-port <serial port>
function spr-set-port() {
	if [ "$#" != 1 ]; then
		echo "Usage: ${FUNCNAME[0]} <serial port>"
		# Print current variable
		_print_current_spresense_environment
	else
		SPRESENSE_PORT=${1}

		# Save current variable
		_save_spresense_environment
		_load_spresense_environment

		# Print current variable
		_print_current_spresense_environment
	fi
}

# Name: spr-unset-port
# Note: Unset serial port.
# Usage: $ spr-unset-port
function spr-unset-port() {
	unset SPRESENSE_PORT

	# Save current variable
	_save_spresense_environment

	# Print current variable
	_print_current_spresense_environment
}

# Name: spr-set-baud
# Note: Set serial port baud rate
# Usage: $ spr-set-baud <baud rate>
function spr-set-baud() {
	if [ "$#" != 1 ]; then
		echo "Usage: ${FUNCNAME[0]} <baud rate>"
		# Print current variable
		_print_current_spresense_environment
	else
		SPRESENSE_BAUD=${1}

		# Save current variable
		_save_spresense_environment
		_load_spresense_environment

		# Print current variable
		_print_current_spresense_environment
	fi
}

# Name: spr-unset-baud
# Note: Unset serial port baud rate.
# Usage: $ spr-unset-baud
function spr-unset-baud() {
	unset SPRESENSE_BAUD

	# Save current variable
	_save_spresense_environment

	# Print current variable
	_print_current_spresense_environment
}

# Name: spr-info
# Note: Show current environment information
# Usage: $ spr-info
function spr-info() {
	# Print current variable
	_print_current_spresense_environment
}

# Name: spr-create-app
# Note: Create user application into application root directory.
# Usage: $ spr-create-app <application name>
function spr-create-app() {
	if [ "$#" != 1 ]; then
		echo "Usage: ${FUNCNAME[0]} <application name>"
	elif [ "${SPRESENSE_HOME}" == "" ]; then
		echo "Warning: Spresense user application directory is not set."
		echo "         Please run"
		echo "         $ spr-set-approot <application home directory>"
	else
		cd ${SPRESENSE_SDK}/sdk
		KCONFIG_EXISTED=0
		if [ -f ${SPRESENSE_SDK}/sdk/Kconfig ]; then
			KCONFIG_EXISTED=1
			rm -f ${SPRESENSE_HOME}/Kconfig
			rm -f ${SPRESENSE_SDK}/sdk/apps/Kconfig
			rm -f ${SPRESENSE_SDK}/sdk/apps/spresense/Kconfig
			rm -f ${SPRESENSE_SDK}/sdk/Kconfig
		fi
		./tools/mkcmd.py -d ${SPRESENSE_HOME} ${1}
		if [ ${KCONFIG_EXISTED} -eq 1 ]; then
			make olddefconfig &> /dev/null
		fi
		cd - &> /dev/null
	fi
}

# Name: spr-import-example
# Note: Import an example application into application root directory.
# Usage: $ spr-import-example <example application path>
function spr-import-example() {
	# Validate arguments
	if [ "$#" != 1 ]; then
		echo "Usage: ${FUNCNAME[0]} <example application path>"
		return 1
	fi

	# Check if SPRESENSE_HOME is set
	if [ -z "${SPRESENSE_HOME}" ]; then
		echo "Warning: Spresense user application directory is not set."
		echo "         Please run"
		echo "         $ spr-set-approot <application home directory>"
		return 1
	fi

	local src_app="${1}"
	local app_basename=$(basename "${src_app}")
	local dest_path="${SPRESENSE_HOME}/${app_basename}"

	# Check if source application exists
	if [ ! -e "${src_app}" ]; then
		echo "Error: Application '${src_app}' does not exist."
		return 1
	fi

	# Ensure the source lives under an 'examples' directory
	local src_parent_abs="$(cd "$(dirname "${src_app}")" && pwd -P)"
	local src_parent_name="$(basename "${src_parent_abs}")"
	if [ "${src_parent_name}" != "examples" ]; then
		echo "Error: Application '${src_app}' must reside under an 'examples' directory."
		return 1
	fi

	# Copy application to SPRESENSE_HOME
	if ! cp -r "${src_app}" "${SPRESENSE_HOME}"; then
		echo "Error: Failed to copy '${src_app}' to '${SPRESENSE_HOME}'."
		return 1
	fi

	# Generate approot names
	local approot=$(basename "${SPRESENSE_HOME}")
	local APPROOT="${approot^^}"
	local SED_INPLACE=(-i)

	# macOS (BSD sed) requires explicit empty backup suffix with -i
	if [ "$(uname -s)" = "Darwin" ]; then
		SED_INPLACE=(-i "")
	fi

	# Copy configs/examples/${approot} if it exists
	local configs_src="${SPRESENSE_SDK}/sdk/configs/examples/${app_basename}"
	if [ -d "${configs_src}" ]; then
		mkdir -p "${dest_path}/configs"
		cp -r "${configs_src}" "${dest_path}/configs/"

		# Replace +EXAMPLES_ with +${APPROOT}_ in defconfig
		local defconfig_file="${dest_path}/configs/${app_basename}/defconfig"
		if [ -f "${defconfig_file}" ]; then
			if ! sed "${SED_INPLACE[@]}" "s/+EXAMPLES_/+${APPROOT}_/g" "${defconfig_file}"; then
				echo "Warning: Failed to replace +EXAMPLES_ with +${APPROOT}_ in ${defconfig_file}."
			fi
		fi
	fi

	# Replace EXAMPLES_ with APPROOT_ in all files
	# Change to destination directory to avoid path issues
	cd "${SPRESENSE_HOME}" || return 1
	if ! find "${app_basename}" -type f -exec sed "${SED_INPLACE[@]}" "s/EXAMPLES_/${APPROOT}_/g" {} +; then
		echo "Warning: Failed to replace EXAMPLES with ${APPROOT} in some files."
		cd - &> /dev/null
		return 1
	fi
	cd - &> /dev/null

	# Replace $(APPDIR)/examples/ with nothing in Make.defs
	local makedefs_file="${dest_path}/Make.defs"
	if [ -f "${makedefs_file}" ]; then
		if ! sed "${SED_INPLACE[@]}" 's|\$(APPDIR)/examples/||g' "${makedefs_file}"; then
			echo "Warning: Failed to replace \$(APPDIR)/examples/ in ${makedefs_file}."
		fi
	fi

	if [ -f ${SPRESENSE_SDK}/sdk/Kconfig ]; then
		cd ${SPRESENSE_SDK}/sdk
		rm -f ${SPRESENSE_HOME}/Kconfig
		rm -f ${SPRESENSE_SDK}/sdk/apps/Kconfig
		rm -f ${SPRESENSE_SDK}/sdk/apps/spresense/Kconfig
		rm -f ${SPRESENSE_SDK}/sdk/Kconfig
		make olddefconfig &> /dev/null
		cd - &> /dev/null
	fi

	echo "Application '${app_basename}' successfully imported to ${SPRESENSE_HOME}"
	return 0
}

# Name: spr-config
# Note: Configure SDK and user application same as config.py command.
# Usage: $ spr-config <configuration name>...
function spr-config() {
	local appname
	local configname
	local config_arg="$1"
	local approot=$(basename "${SPRESENSE_HOME}")

	if [ "$#" -eq 0 ]; then
		echo "Usage: ${FUNCNAME[0]} <config name>..."
		echo "  -l             List available configurations"
		echo "  -m             Open menuconfig after configuration"
		echo "  -v <config>    Verbose output"
		echo "  -i <config>    Show configuration"
		return 1
	fi

	cd ${SPRESENSE_SDK}/sdk

	# Check if config_arg contains / and configname is "default"
	if [[ "$config_arg" == *"/default" ]]; then
		# Parse appname/configname format
		appname="${config_arg%/*}"
		configname="${config_arg#*/}"

		if [[ "$approot" == "$appname" ]]; then
			./tools/config.py -d "${SPRESENSE_HOME}/configs" "${configname}"
		else
			./tools/config.py -d "${SPRESENSE_HOME}/${appname}/configs" "${configname}"
		fi
	else
		# Normal config.py execution
		./tools/config.py $@
	fi
	cd - &> /dev/null
}

# Name: spr-mkdefconfig
# Note: Create user configuration into application root directory.
# Usage: $ spr-mkdefconfig <configuration name>...
function spr-mkdefconfig() {
	# Validate arguments
	if [ "$#" != 1 ]; then
		echo "Usage: ${FUNCNAME[0]} <configuration name>"
		return 1
	fi

	# Check if SPRESENSE_HOME is set
	if [ -z "${SPRESENSE_HOME}" ]; then
		echo "Warning: Spresense user application directory is not set."
		echo "         Please run"
		echo "         $ spr-set-approot <application home directory>"
		return 1
	fi

	# Check SDK path
	if [ -z "${SPRESENSE_SDK}" ]; then
		echo "Warning: SPRESENSE_SDK is not set."
		echo "         Please run 'source tools/build-env.sh' again."
		return 1
	fi

	local config_name="$1"
	local sdk_dir="${SPRESENSE_SDK}/sdk"
	local current_dir="$(pwd -P)"
	local approot=$(basename "${SPRESENSE_HOME}")

	if ! cd "${sdk_dir}"; then
		echo "Error: Failed to enter SDK directory: ${sdk_dir}"
		return 1
	fi

	# Copy default/defconfig to SPRESENSE_HOME/configs/default
	mkdir -p "${SPRESENSE_HOME}/configs"
	cp -r "${sdk_dir}/configs/default" "${SPRESENSE_HOME}/configs/"

	if ! ./tools/mkdefconfig.py -d "${SPRESENSE_HOME}" "${approot}/${config_name}"; then
		echo "Error: mkdefconfig failed for configuration: ${config_name}"
		cd "${current_dir}" &> /dev/null
		return 1
	fi

	cd "${current_dir}" &> /dev/null
	echo "Config '${config_name}' successfully created in ${SPRESENSE_HOME}/configs"
	return 0
}

# Name: spr-flash
# Note: Flash nuttx.spk via flash.sh wrapper.
# Usage: $ spr-flash <options>
function spr-flash() {
	cd ${SPRESENSE_SDK}/sdk
	# Check if arguments contain .spk or .espk files, or -w/-r/-B option
	local has_spk=0
	for arg in "$@"; do
		if [[ "$arg" =~ \.(spk|espk)$ ]] || [[ "$arg" == "-w" ]] || [[ "$arg" == "-r" ]] || [[ "$arg" == "-B" ]]; then
			has_spk=1
			break
		fi
	done

	local port;
	if [ "${SPRESENSE_PORT}" != "" ]; then
		port="-c ${SPRESENSE_PORT}"
	fi

	local baud;
	if [ "${SPRESENSE_BAUD}" != "" ]; then
		baud="-b ${SPRESENSE_BAUD}"
	fi

	if [ $has_spk -eq 1 ]; then
		./tools/flash.sh ${port} ${baud} $@
	else
		./tools/flash.sh ${port} ${baud} $@ ${SPRESENSE_HOME}/build/nuttx.spk
	fi
	cd - &> /dev/null
}

# Name: spr-terminal
# Note: Open serial terminal via terminal.sh wrapper.
# Usage: $ spr-terminal <options>
function spr-terminal() {
	cd ${SPRESENSE_SDK}/sdk

	local port;
	if [ "${SPRESENSE_PORT}" != "" ]; then
		port="-c ${SPRESENSE_PORT}"
	fi

	./tools/terminal.sh ${port} $@
	cd - &> /dev/null
}

# Name: spr-go-sdk
# Note: Move current directory to SDK directory.
# Usage: $ spr-go-sdk
function spr-go-sdk() {
	if [ "${SPRESENSE_SDK}" != "" ]; then
		cd ${SPRESENSE_SDK}/sdk
	else
		echo "Warning: SPRESENSE_SDK is not set."
		echo "         Please run 'source tools/build-env.sh' again."
	fi
}

# Name: spr-go-approot
# Note: Move current directory to user application root directory.
# Usage: $ spr-go-approot
function spr-go-approot() {
	if [ "${SPRESENSE_HOME}" != "" ]; then
		cd ${SPRESENSE_HOME}
	else
		echo "Warning: SPRESENSE_HOME is not set."
		echo "         Please run 'spr-set-approot' first."
	fi
}

# Name: spr-make
# Note: Build SDK and user application same as make command.
# Usage: $ spr-make [build options]
function spr-make() {
	make -j -C ${SPRESENSE_SDK}/sdk $@
	if [[ $? -eq 0 ]] && [[ -d ${SPRESENSE_HOME} ]]; then
		# Check if arguments contain clean, config or help
		local skip_copy=0
		local remove_build=0
		for arg in "$@"; do
			if [[ "$arg" == *config* ]] || [[ "$arg" == help ]]; then
				skip_copy=1
			fi
			if [[ "$arg" == *clean* ]]; then
				skip_copy=1
				remove_build=1
			fi
			if [[ "$arg" == all ]]; then
				skip_copy=0
				remove_build=0
			fi
		done

		if [[ $skip_copy -eq 0 ]] && [[ -f ${SPRESENSE_SDK}/sdk/nuttx.spk ]]; then
			echo "Build successful. Copying build artifacts to ${SPRESENSE_HOME}/build"
			mkdir -p ${SPRESENSE_HOME}/build
			cp ${SPRESENSE_SDK}/sdk/nuttx.spk ${SPRESENSE_HOME}/build/
			cp ${SPRESENSE_SDK}/sdk/nuttx ${SPRESENSE_HOME}/build/
			cp ${SPRESENSE_SDK}/sdk/nuttx.map ${SPRESENSE_HOME}/build/
			cp ${SPRESENSE_SDK}/sdk/System.map ${SPRESENSE_HOME}/build/
		fi

		if [[ $remove_build -eq 1 ]]; then
			echo "Clean successful. Removing build artifacts from ${SPRESENSE_HOME}/build"
			rm -f ${SPRESENSE_HOME}/build/nuttx.spk
			rm -f ${SPRESENSE_HOME}/build/nuttx
			rm -f ${SPRESENSE_HOME}/build/nuttx.map
			rm -f ${SPRESENSE_HOME}/build/System.map
		fi
	fi
}

# Load current variable
function _load_spresense_environment() {
	if [ -f ${HOME}/.spresense_env ]; then
		source ${HOME}/.spresense_env
	fi
	export SPRESENSE_HOME
	export SPRESENSE_PORT
	export SPRESENSE_BAUD
}

# Save current variable
function _save_spresense_environment() {
	# Export parameters into configuration file
	echo "SPRESENSE_HOME=${SPRESENSE_HOME}" > ${HOME}/.spresense_env
	echo "SPRESENSE_SDK=${SPRESENSE_SDK}" >> ${HOME}/.spresense_env
	echo "SPRESENSE_PORT=${SPRESENSE_PORT}" >> ${HOME}/.spresense_env
	echo "SPRESENSE_BAUD=${SPRESENSE_BAUD}" >> ${HOME}/.spresense_env
}

# Print current variable
function _print_current_spresense_environment() {
	echo "======================================="
	echo "   SDK_VERSION = `cat ${SPRESENSE_SDK}/sdk/tools/mkversion.sh | grep '^SDK_VERSION' | cut -d '\"' -f 2`"
	echo " NUTTX_VERSION = `cat ${SPRESENSE_SDK}/sdk/tools/mkversion.sh | grep '^NUTTX_VERSION' | cut -d '\"' -f 2`"
	echo " SPRESENSE_SDK = ${SPRESENSE_SDK}"
	echo "SPRESENSE_HOME = ${SPRESENSE_HOME}"
	echo "SPRESENSE_PORT = ${SPRESENSE_PORT}"
	echo "SPRESENSE_BAUD = ${SPRESENSE_BAUD}"
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
    echo "         $ spr-create-approot <application home directory>"
    echo "            or"
    echo "         $ spr-set-approot <application home directory>"
elif [ ! -d ${SPRESENSE_HOME} ]; then
    echo "Warning: ${SPRESENSE_HOME} does not exist."
    echo "         Please run"
    echo "         $ spr-create-approot ${SPRESENSE_HOME}"
fi

if [ -d "${SPRESENSE_HOME}" -a ! -f "${SPRESENSE_HOME}/.sdksubdir" ]; then
    if [ -f "${SPRESENSE_HOME}/Application.mk" ]; then
        echo "Warning: Your environment(${SPRESENSE_HOME}) is created for Spresense SDK1.x version."
        echo "         Please create a new project directory with next command."
        echo "         $ spr-create-approot <new place>"
    else
        echo "Warning: Your environment(${SPRESENSE_HOME}) doesn't have makefiles."
        echo "         Please run next command to create makefiles."
        echo "         $ spr-create-approot ${SPRESENSE_HOME}"
    fi
    unset SPRESENSE_HOME
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
	echo "Info: Please update the version to 4.0 or later"
	echo "      The spresense tools completion can not work well"
	echo "      because of old bash version."
fi

#
# Tool path setup
#

export PATH=${SCRIPT_DIR}/bin:$PATH
