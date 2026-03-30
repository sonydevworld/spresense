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
# Usage: $ spr-create-approot <application root directory>
function spr-create-approot() {
	if [ "$#" == 0 ] || [ "${1:0:1}" == "-" ]; then
		echo "Usage: ${FUNCNAME[0]} <approot> [description]"
		echo ""
		echo "Create and set your new application root directory."
		echo ""
		echo "Arguments:"
		echo "  approot         Path to your new application root directory."
		echo "                  Absolute path or relative path from the current directory."
		echo "                  Use alphanumeric characters with '.', '~', '_', '-' and '/'."
		echo "                  (e.g. ~/myproject, ../../myproject)"
		echo ""
		echo "Options:"
		echo "  description     Optional description of the application root."
		echo "                  (e.g. \"My Project\")"
		echo "                  Leading or trailing spaces are not allowed."
		echo ""
		return 1
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	# Save previous SPRESENSE_HOME to restore on error
	local prev_spresense_home="${SPRESENSE_HOME}"

	if [ -f "${SPRESENSE_SDK}/nuttx/.config" ]; then
		echo "Cleaning build output before creating application root directory..."
		if ! spr-make distclean > /dev/null 2>&1; then
			echo "Error: Failed to run 'spr-make distclean'."
		fi
	fi

	local mkappsdir_flag=""
	local mkappsdir_desc="User application"
	if [ "$#" -ge 2 ]; then
		mkappsdir_desc="$2"
		if [[ "${mkappsdir_desc}" == *\"* ]]; then
			echo "Error: Invalid description '${mkappsdir_desc}'."
			echo "       Double quotes (\") are not allowed in description."
			return 1
		fi
		if [[ "${mkappsdir_desc}" =~ ^[[:space:]] || "${mkappsdir_desc}" =~ [[:space:]]$ ]]; then
			echo "Error: Invalid description '${mkappsdir_desc}'."
			echo "       Leading or trailing spaces are not allowed in description."
			return 1
		fi
	fi
	if [[ "$1" =~ [^A-Za-z0-9./~_-] ]]; then
		echo "Error: Invalid approot '$1'."
		echo "       Use alphanumeric characters with '.', '~', '_', '-' and '/' only."
		return 1
	fi
	if [[ "$1" == *"~"* ]] && [[ "${1:0:1}" != "~" ]]; then
		echo "Error: Invalid approot '$1'."
		echo "       '~' is only allowed at the beginning of approot."
		return 1
	fi
	if [ "${1:0:1}" == "/" ]; then
		SPRESENSE_HOME=${1}
	elif [ "${1:0:1}" == "~" ]; then
		SPRESENSE_HOME=${HOME}${1:1}
	else
		SPRESENSE_HOME=$(pwd -P)/${1}
	fi
	SPRESENSE_HOME_DIR=$(dirname ${SPRESENSE_HOME})
	SPRESENSE_HOME_BASE=$(basename ${SPRESENSE_HOME})
	SPRESENSE_HOME_BASE_LOWER=$(echo "${SPRESENSE_HOME_BASE}" | tr '[:upper:]' '[:lower:]')
	if [ "${SPRESENSE_HOME_BASE_LOWER}" == "examples" ] || \
	   [ "${SPRESENSE_HOME_BASE_LOWER}" == "feature" ] || \
	   [ "${SPRESENSE_HOME_BASE_LOWER}" == "device" ]; then
		echo "Error: Invalid approot '${SPRESENSE_HOME}'."
		echo "       Directory names 'examples', 'feature', and 'device' are reserved and cannot be used."
		SPRESENSE_HOME="${prev_spresense_home}"
		return 1
	fi
	mkdir -p ${SPRESENSE_HOME_DIR}
	SPRESENSE_HOME=$(cd ${SPRESENSE_HOME_DIR}; pwd -P)/${SPRESENSE_HOME_BASE}
	if [ -d ${SPRESENSE_HOME} ]; then
		echo "Warning: Directory '${SPRESENSE_HOME}' already exists,"
		echo -n "         Overwrite Makefile ? (Y/N): "
		read input
		input=`echo ${input} | tr '[:lower:]' '[:upper:]'`
		if [ "${input}" == "Y" ]; then
			mkappsdir_flag="-f"
		else
			echo "Canceled to create '${SPRESENSE_HOME}'."
			# Restore previous SPRESENSE_HOME on cancel
			SPRESENSE_HOME="${prev_spresense_home}"
			_save_spresense_environment
			_load_spresense_environment
			return 1
		fi
	fi
	cd ${SPRESENSE_SDK}/sdk || return 1
	./tools/mkappsdir.py ${mkappsdir_flag} ${SPRESENSE_HOME} "${mkappsdir_desc}"
	if [ $? -ne 0 ]; then
		echo "Error: Failed to create '${SPRESENSE_HOME}'."
		# Restore previous SPRESENSE_HOME on error
		SPRESENSE_HOME="${prev_spresense_home}"
		_save_spresense_environment
		_load_spresense_environment
		cd - &> /dev/null
		return 1
	fi
	# Create a marker file to indicate this is an application root directory
	touch "${SPRESENSE_HOME}/.approotdir"
	echo "Created and set application root directory to '${SPRESENSE_HOME}'"
	cd - &> /dev/null
	# Save current variable
	_save_spresense_environment
	_load_spresense_environment
	# Print current variable
	_print_current_spresense_environment
}

# Name: spr-set-approot
# Note: Select application root directory.
# Usage: $ spr-set-approot <application root directory>
function spr-set-approot() {
	if [ "$#" == 0 ] || [ "${1:0:1}" == "-" ]; then
		echo "Usage: ${FUNCNAME[0]} <approot>"
		echo ""
		echo "Set your application root directory as SPRESENSE_HOME."
		echo "Specify the directory created by 'spr-create-approot' command."
		echo ""
		echo "Arguments:"
		echo "  approot      Your application root directory."
		echo "               Absolute path or relative path from the current directory."
		echo "               Use alphanumeric characters with '.', '~', '_', '-' and '/' only."
		echo "               (e.g. ~/myproject, ../../myproject)"
		echo ""
		return 1
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	if [[ "$1" =~ [^A-Za-z0-9./~_-] ]]; then
		echo "Error: Invalid approot '$1'."
		echo "       Use alphanumeric characters with '.', '~', '_', '-' and '/' only."
		return 1
	fi
	if [[ "$1" == *"~"* ]] && [[ "${1:0:1}" != "~" ]]; then
		echo "Error: Invalid approot '$1'."
		echo "       '~' is only allowed at the beginning of approot."
		return 1
	fi

	if [ "${1:0:1}" == "/" ]; then
		_SPRESENSE_HOME=${1}
	elif [ "${1:0:1}" == "~" ]; then
		_SPRESENSE_HOME=${HOME}${1:1}
	else
		if [ -d "${1}" ]; then
			_SPRESENSE_HOME="$(cd ${1} &> /dev/null; pwd)"
		else
			_SPRESENSE_HOME="$(pwd -P)/${1}"
		fi
	fi
	if [ -d "${_SPRESENSE_HOME}" ]; then
		if [ -f "${_SPRESENSE_HOME}/.sdksubdir" -a -f "${_SPRESENSE_HOME}/.approotdir" ]; then
			if [ -f "${SPRESENSE_SDK}/nuttx/.config" ]; then
				echo "Cleaning build output before setting application root directory..."
				if ! spr-make distclean > /dev/null 2>&1; then
					echo "Error: Failed to run 'spr-make distclean'."
				fi
			fi
			SPRESENSE_HOME=${_SPRESENSE_HOME}
			# Save current variable
			_save_spresense_environment
			_load_spresense_environment
			# Print current variable
			_print_current_spresense_environment
		else
			if [ -f "${_SPRESENSE_HOME}/Application.mk" ]; then
				echo "Warning: Your environment('${_SPRESENSE_HOME}') is created for Spresense SDK1.x version."
				echo "         Please create a new application root directory with next command."
				echo ""
				echo "         $ spr-create-approot <new place>"
				echo ""
			else
				echo "Warning: Your environment('${_SPRESENSE_HOME}') is invalid."
				echo "         Please run next command to create the application root directory."
				echo ""
				echo "         $ spr-create-approot ${_SPRESENSE_HOME}"
				echo ""
			fi
			unset SPRESENSE_HOME
			# Save current variable
			_save_spresense_environment
			_load_spresense_environment
		fi
	else
		echo "Warning: '${_SPRESENSE_HOME}' does not exist."
		echo "         Please run next command to create the application root directory."
		echo ""
		echo "         $ spr-create-approot ${_SPRESENSE_HOME}"
		echo ""
	fi
}

# Name: spr-unset-approot
# Note: Unset application root directory.
# Usage: $ spr-unset-approot
function spr-unset-approot() {
	if [ "$#" != 0 ]; then
		echo "Usage: ${FUNCNAME[0]}"
		echo ""
		echo "Unset your application root directory."
		echo ""
		return 1
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	if [ -f "${SPRESENSE_SDK}/nuttx/.config" ]; then
		echo "Cleaning build output before unsetting application root directory..."
		if ! spr-make distclean > /dev/null 2>&1; then
			echo "Error: Failed to run 'spr-make distclean'."
		fi
	fi

	unset SPRESENSE_HOME
	# Save current variable
	_save_spresense_environment
	_load_spresense_environment
	# Print current variable
	_print_current_spresense_environment
}

# Name: spr-distclean
# Note: Clean generated files under sdk/apps by git clean.
# Usage: $ spr-distclean
function spr-distclean() {
	local force=0
	if [ "$#" -gt 1 ] || { [ "$#" -eq 1 ] && [ "$1" != "-f" ]; }; then
		echo "Usage: ${FUNCNAME[0]} [-f]"
		echo ""
		echo "Clean generated files under sdk/apps by git clean."
		echo "Then run 'spr-make distclean' automatically."
		echo ""
		echo "Options:"
		echo "  -f          Run cleanup without confirmation prompt."
		echo ""
		return 1
	fi
	if [ "$#" -eq 1 ]; then
		force=1
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	local apps_dir="${SPRESENSE_SDK}/sdk/apps"
	if [ ! -d "${apps_dir}" ]; then
		echo "Error: '${apps_dir}' directory does not exist."
		return 1
	fi
	if ! command -v git > /dev/null 2>&1; then
		echo "Error: 'git' command is not available."
		return 1
	fi

	if [ ${force} -eq 0 ]; then
		local preview
		preview=$(git -C "${apps_dir}" clean -ndx)
		if [ $? -ne 0 ]; then
			echo "Error: Failed to list cleanup targets under '${apps_dir}'."
			return 1
		fi
		if [ -z "${preview}" ]; then
			echo "No files to clean under '${apps_dir}'."
			if [ -f "${SPRESENSE_SDK}/nuttx/.config" ]; then
				if ! spr-make distclean > /dev/null 2>&1; then
					echo "Error: Failed to run 'spr-make distclean'."
				fi
			fi
			return 0
		fi

		echo "This command removes untracked and ignored files under '${apps_dir}'."
		echo "Targets to remove:"
		echo "${preview}"
		echo -n "Run 'git -C ${apps_dir} clean -fdx' ? (Y/N): "
		read input
		input=`echo ${input} | tr '[:lower:]' '[:upper:]'`
		if [ "${input}" != "Y" ]; then
			echo "Canceled to clean '${apps_dir}'."
			return 1
		fi
	fi

	git -C "${apps_dir}" clean -fdx
	if [ $? -ne 0 ]; then
		echo "Error: Failed to clean '${apps_dir}'."
		return 1
	fi
	if [ -f "${SPRESENSE_SDK}/nuttx/.config" ]; then
		if ! spr-make distclean > /dev/null 2>&1; then
			echo "Error: Failed to run 'spr-make distclean'."
		fi
	fi
	echo "Finished cleaning '${apps_dir}' and running 'spr-make distclean'."
}

# Name: spr-set-port
# Note: Set serial port
# Usage: $ spr-set-port <serial port>
function spr-set-port() {
	if [ "$#" == 0 ] || [ "${1:0:1}" == "-" ]; then
		echo "Usage: ${FUNCNAME[0]} <port>"
		echo ""
		echo "Set your serial port used by 'spr-flash' and 'spr-terminal' command."
		echo ""
		echo "Arguments:"
		echo "  port         Serial port name"
		echo "               (e.g. COM3, /dev/ttyS0)."
		echo ""
		return 1
	fi

	SPRESENSE_PORT=${1}
	# Save current variable
	_save_spresense_environment
	_load_spresense_environment
	# Print current variable
	_print_current_spresense_environment
}

# Name: spr-unset-port
# Note: Unset serial port.
# Usage: $ spr-unset-port
function spr-unset-port() {
	if [ "$#" != 0 ]; then
		echo "Usage: ${FUNCNAME[0]}"
		echo ""
		echo "Unset your serial port."
		echo ""
		return 1
	fi

	unset SPRESENSE_PORT
	# Save current variable
	_save_spresense_environment
	_load_spresense_environment
	# Print current variable
	_print_current_spresense_environment
}

# Name: spr-set-baud
# Note: Set serial port baud rate
# Usage: $ spr-set-baud <baud rate>
function spr-set-baud() {
	if [ "$#" == 0 ] || [ "${1:0:1}" == "-" ]; then
		echo "Usage: ${FUNCNAME[0]} <baud>"
		echo ""
		echo "Set your serial baud rate used by 'spr-flash' command."
		echo ""
		echo "Arguments:"
		echo "  baud         Baud rate for serial communication."
		echo "               (e.g. 115200, 230400, 460800, 500000, 576000, 921600, 1000000, 1152000)."
		echo ""
		return 1
	fi

	if [[ ! "$1" =~ ^[0-9]+$ ]]; then
		echo "Error: Invalid baud '$1'."
		echo "       Baud must contain digits only."
		return 1
	fi

	SPRESENSE_BAUD=${1}
	# Save current variable
	_save_spresense_environment
	_load_spresense_environment
	# Print current variable
	_print_current_spresense_environment
}

# Name: spr-unset-baud
# Note: Unset serial port baud rate.
# Usage: $ spr-unset-baud
function spr-unset-baud() {
	if [ "$#" != 0 ]; then
		echo "Usage: ${FUNCNAME[0]}"
		echo ""
		echo "Unset your serial baud rate."
		echo "The default 115200 is used by 'spr-flash' command."
		echo ""
		return 1
	fi

	unset SPRESENSE_BAUD
	# Save current variable
	_save_spresense_environment
	_load_spresense_environment
	# Print current variable
	_print_current_spresense_environment
}

# Name: spr-info
# Note: Show current environment information
# Usage: $ spr-info
function spr-info() {
	if [ "$#" != 0 ]; then
		echo "Usage: ${FUNCNAME[0]}"
		echo ""
		echo "Show your current environment information."
		echo ""
		return 1
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	# Print current variable
	_print_current_spresense_environment
}

# Name: spr-create-app
# Note: Create user application into application root directory.
# Usage: $ spr-create-app <application name>
function spr-create-app() {
	if [ "$#" == 0 ] || [ "${1:0:1}" == "-" ]; then
		echo "Usage: ${FUNCNAME[0]} <appname> [description] [-x]"
		echo ""
		echo "Create your application into application root directory."
		echo ""
		echo "Arguments:"
		echo "  appname         Application name to create."
		echo "                  It must be an alphanumeric string (A-Z, a-z, 0-9, _)."
		echo ""
		echo "Options:"
		echo "  description     Optional description of the application."
		echo "                  (e.g. \"My Application\")"
		echo "                  Leading or trailing spaces are not allowed."
		echo "  -x              Create C++ application."
		echo ""
		return 1
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi
	_check_spresense_home_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	local cpp_flag=""
	local appname=""
	local desc=""
	local arg_count=0
	local approot=$(basename "${SPRESENSE_HOME}")
	local SED_INPLACE=(-i)

	# macOS (BSD sed) requires explicit empty backup suffix with -i
	if [ "$(uname -s)" = "Darwin" ]; then
		SED_INPLACE=(-i "")
	fi

	# Parse arguments and remove -x flag
	for arg in "$@"; do
		if [ "$arg" == "-x" ]; then
			cpp_flag="-x"
		else
			arg_count=$((arg_count + 1))
			if [ $arg_count -eq 1 ]; then
				appname="$arg"
			elif [ $arg_count -eq 2 ]; then
				desc="$arg"
				if [[ "$desc" == -* ]]; then
					desc=""
					arg_count=$((arg_count - 1))
				fi
			fi
		fi
	done
	if ! [[ "$appname" =~ ^[A-Za-z0-9_]+$ ]]; then
		echo "Error: Invalid appname '${appname}'."
		echo "       Use alphanumeric characters only (A-Z, a-z, 0-9, _)."
		return 1
	fi
	if [[ "$appname" =~ ^[0-9] ]]; then
		echo "Error: Invalid appname '${appname}'."
		echo "       appname cannot start with a digit."
		return 1
	fi
	if [[ "${desc}" == *\"* ]]; then
		echo "Error: Invalid description '${desc}'."
		echo "       Double quotes (\") are not allowed in description."
		return 1
	fi
	if [[ "${desc}" =~ ^[[:space:]] || "${desc}" =~ [[:space:]]$ ]]; then
		echo "Error: Invalid description '${desc}'."
		echo "       Leading or trailing spaces are not allowed in description."
		return 1
	fi

	cd ${SPRESENSE_SDK}/sdk || return 1
	KCONFIG_EXISTED=0
	if [ -f ${SPRESENSE_SDK}/sdk/Kconfig ]; then
		KCONFIG_EXISTED=1
		rm -f ${SPRESENSE_HOME}/Kconfig
		rm -f ${SPRESENSE_SDK}/sdk/apps/Kconfig
		rm -f ${SPRESENSE_SDK}/sdk/apps/spresense/Kconfig
		rm -f ${SPRESENSE_SDK}/sdk/Kconfig
	fi
	./tools/mkcmd.py -d ${SPRESENSE_HOME} ${cpp_flag} ${appname} "${desc}"
	if [ $? -ne 0 ]; then
		echo "Error: Failed to create application '${appname}'."
		cd - &> /dev/null
		return 1
	fi
	# Replace the generated APPROOT_ prefix token with USERAPP_ in created files
	if ! find "${SPRESENSE_HOME}/${appname}" -type f -exec sed "${SED_INPLACE[@]}" "s/${approot^^}_/USERAPP_/g" {} +; then
		echo "Error: Failed to replace ${approot^^}_ with USERAPP_ in '${SPRESENSE_HOME}/${appname}'."
		echo "       'find' or 'sed' is not available. Please check your environment settings."
		cd - &> /dev/null
		return 1
	fi
	# Replace "default y" with "default n" in generated Kconfig
	if [ -f "${SPRESENSE_HOME}/${appname}/Kconfig" ]; then
		sed "${SED_INPLACE[@]}" 's/^\([[:space:]]*\)default y$/\1default n/' "${SPRESENSE_HOME}/${appname}/Kconfig"
	fi
	# Copy default/defconfig to SPRESENSE_HOME/configs/default
	mkdir -p "${SPRESENSE_HOME}/configs/userapp/${appname}"
	cp -rf "${SPRESENSE_SDK}/sdk/configs/default" "${SPRESENSE_HOME}/configs/"
	# Create defconfig with +USERAPP_${APPNAME}=y
	echo "+USERAPP_${appname^^}=y" > "${SPRESENSE_HOME}/configs/userapp/${appname}/defconfig"
	echo "New 'userapp/${appname}' config successfully created into ${SPRESENSE_HOME}/configs"
	if [ ${KCONFIG_EXISTED} -eq 1 ]; then
		make olddefconfig &> /dev/null
	fi
	cd - &> /dev/null
}

# Name: spr-import-example
# Note: Import an example application into application root directory.
# Usage: $ spr-import-example <example application path>
function spr-import-example() {
	if [ "$#" == 0 ] || [ "${1:0:1}" == "-" ]; then
		echo "Usage: ${FUNCNAME[0]} <example>"
		echo ""
		echo "Import the specified example application into the application root directory."
		echo "Specify an example name completed with tab completion."
		echo ""
		echo "Arguments:"
		echo "  example         The example application name to import."
		echo "                  (e.g. examples/gnss, apps/examples/hello)"
		echo ""
		return 1
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi
	_check_spresense_home_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	local src_app="${1}"
	local src_arg_for_match="${1}"
	src_arg_for_match="${src_arg_for_match#./}"
	src_arg_for_match="${src_arg_for_match%/}"

	# Allow completion to pass SPRESENSE_SDK-relative paths.
	if [[ "${src_app}" != /* ]]; then
		if [[ "${src_app}" == examples/* ]]; then
			src_app="${SPRESENSE_SDK}/${src_app}"
		elif [[ "${src_app}" == apps/examples/* ]]; then
			src_app="${SPRESENSE_SDK}/sdk/${src_app}"
		fi
	fi

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

	local app_basename=$(basename "${src_app}")
	local approot=$(basename "${SPRESENSE_HOME}")
	local SED_INPLACE=(-i)

	# macOS (BSD sed) requires explicit empty backup suffix with -i
	if [ "$(uname -s)" = "Darwin" ]; then
		SED_INPLACE=(-i "")
	fi

	# Copy application to SPRESENSE_HOME
	if ! cp -rf "${src_app}" "${SPRESENSE_HOME}"; then
		echo "Error: Failed to copy '${src_app}' to '${SPRESENSE_HOME}'."
		return 1
	fi
	echo "Application '${app_basename}' successfully imported to ${SPRESENSE_HOME}"

	# Replace EXAMPLES_ with USERAPP_ in all files
	if ! find "${SPRESENSE_HOME}/${app_basename}" -type f -exec sed "${SED_INPLACE[@]}" "s/EXAMPLES_/USERAPP_/g" {} +; then
		echo "Error: Failed to replace EXAMPLES_ with USERAPP_ in ${SPRESENSE_HOME}/${app_basename}."
		echo "       'find' or 'sed' is not available. Please check your environment settings."
		return 1
	fi

	# Replace $(APPDIR)/examples/ with nothing in Make.defs
	local makedefs_file="${SPRESENSE_HOME}/${app_basename}/Make.defs"
	if [ -f "${makedefs_file}" ]; then
		if ! sed "${SED_INPLACE[@]}" '/CONFIGURED_APPS/s|\$(APPDIR)/examples/||g' "${makedefs_file}"; then
			echo "Warning: Failed to replace \$(APPDIR)/examples/ in ${makedefs_file}."
		fi
		if ! sed "${SED_INPLACE[@]}" '/CONFIGURED_APPS/s|examples/||g' "${makedefs_file}"; then
			echo "Warning: Failed to replace examples/ in ${makedefs_file}."
		fi
	fi

	# Find matching config directories from README [Source path]
	local config_names=()
	while IFS= read -r readme_path; do
		if awk -v target="${src_arg_for_match}" '
			function normalize(s) {
				gsub(/\r/, "", s)
				sub(/^[[:space:]]+/, "", s)
				sub(/[[:space:]]+$/, "", s)
				sub(/^sdk\//, "", s)
				sub(/^\.\//, "", s)
				sub(/\/$/, "", s)
				return s
			}
			BEGIN {
				in_source = 0
				matched = 0
				target = normalize(target)
			}
			/^\[Source path\][[:space:]]*$/ {
				in_source = 1
				next
			}
			in_source && /^\[[^]]+\][[:space:]]*$/ {
				in_source = 0
			}
			in_source {
				path = normalize($0)
				if (path == "") {
					next
				}
				if (path == target) {
					matched = 1
					exit
				}
			}
			END {
				exit(matched ? 0 : 1)
			}
		' "${readme_path}"; then
			config_names+=("$(basename "$(dirname "${readme_path}")")")
		fi
	done < <(find "${SPRESENSE_SDK}/sdk/configs/examples" -mindepth 2 -maxdepth 2 -name README.txt)

	if [ ${#config_names[@]} -eq 0 ]; then
		echo "Warning: No matching config was found in sdk/configs/examples/*/README.txt for source path '${src_arg_for_match}'."
	fi

	# Copy each matched config directory
	local config_copied=0
	for config_name in "${config_names[@]}"; do
		local configs_src="${SPRESENSE_SDK}/sdk/configs/examples/${config_name}"
		if [ -d "${configs_src}" ]; then
			if [ $config_copied -eq 0 ]; then
				mkdir -p "${SPRESENSE_HOME}/configs/userapp"
				cp -rf "${SPRESENSE_SDK}/sdk/configs/default" "${SPRESENSE_HOME}/configs/"
				config_copied=1
			fi
			cp -rf "${configs_src}" "${SPRESENSE_HOME}/configs/userapp"
			echo "Config 'userapp/${config_name}' successfully imported to ${SPRESENSE_HOME}/configs"

			# Replace +EXAMPLES_ with +USERAPP_ in defconfig
			local defconfig_file="${SPRESENSE_HOME}/configs/userapp/${config_name}/defconfig"
			if [ -f "${defconfig_file}" ]; then
				if ! sed "${SED_INPLACE[@]}" "s/+EXAMPLES_/+USERAPP_/g" "${defconfig_file}"; then
					echo "Warning: Failed to replace +EXAMPLES_ with +USERAPP_ in ${defconfig_file}."
				fi
			fi

			# Replace sdk/apps/examples/ or examples/ with userapp/ in README.txt
			local readme_file="${SPRESENSE_HOME}/configs/userapp/${config_name}/README.txt"
			if [ -f "${readme_file}" ]; then
				if ! sed "${SED_INPLACE[@]}" "s|sdk/apps/examples/|${approot}/|g" "${readme_file}"; then
					echo "Warning: Failed to replace sdk/apps/examples/ with ${approot}/ in ${readme_file}."
				fi
				if ! sed "${SED_INPLACE[@]}" "s|examples/|${approot}/|g" "${readme_file}"; then
					echo "Warning: Failed to replace examples/ with ${approot}/ in ${readme_file}."
				fi
			fi
		fi
	done

	if [ -f ${SPRESENSE_SDK}/sdk/Kconfig ]; then
		cd ${SPRESENSE_SDK}/sdk || return 1
		rm -f ${SPRESENSE_HOME}/Kconfig
		rm -f ${SPRESENSE_SDK}/sdk/apps/Kconfig
		rm -f ${SPRESENSE_SDK}/sdk/apps/spresense/Kconfig
		rm -f ${SPRESENSE_SDK}/sdk/Kconfig
		make olddefconfig &> /dev/null
		cd - &> /dev/null
	fi
}

# Name: spr-config
# Note: Configure SDK and user application same as config.py command.
# Usage: $ spr-config <configuration name>...
function spr-config() {
	local config_args=()

	for arg in "$@"; do
		if [[ "$arg" == "-h" ]] || [[ "$arg" == "--help" ]]; then
			config_args=()
			break
		elif [[ "$arg" == *"/default" ]]; then
			config_args+=("default")
		else
			config_args+=("$arg")
		fi
	done

	if [ ${#config_args[@]} == 0 ]; then
		echo "Usage: ${FUNCNAME[0]} [-m] <config>... [-- [+|-]tweak...]"
		echo "       ${FUNCNAME[0]} [-i] <config>..."
		echo "       ${FUNCNAME[0]} [-l]"
		echo ""
		echo "Configure SDK and your application."
		echo "Specify one or more configs listed with the -l option or completed with tab completion."
		echo ""
		echo "Arguments:"
		echo "  config         Configuration name"
		echo "                 (e.g. default, device/sdcard, feature/debug, examples/hello)"
		echo ""
		echo "Options:"
		echo "  -l             List available configurations"
		echo "  -m config      Open menuconfig with configuration"
		echo "  -i config      Show configuration"
		echo "  -- +tweak      Enable the specified configuration without 'CONFIG_' prefix"
		echo "  -- -tweak      Disable the specified configuration without 'CONFIG_' prefix"
		echo "                 (e.g. -- +DEBUG_FEATURES -USBDEV)"
		echo ""
		return 1
	fi
	cd ${SPRESENSE_SDK}/sdk || return 1
	./tools/config.py "${config_args[@]}"
	if [ $? -ne 0 ]; then
		echo "Error: Failed to configure '${config_args[@]}'."
		cd - &> /dev/null
		return 1
	fi
	# Show finished message only when not listing or showing info
	local show_finished=1
	for arg in "${config_args[@]}"; do
		if [[ "$arg" == "-l" ]] || [[ "$arg" == "-i" ]]; then
			show_finished=0
			break
		fi
	done

	if [ $show_finished -eq 1 ]; then
		echo "Configuration '${config_args[@]}' finished."
		if [[ -n "${SPRESENSE_HOME}" ]] && [[ -f "${SPRESENSE_SDK}/nuttx/.config" ]]; then
			cp -f "${SPRESENSE_SDK}/nuttx/.config" "${SPRESENSE_HOME}/sdk.config"
		fi
	fi
	cd - &> /dev/null
}

# Name: spr-mkdefconfig
# Note: Create user configuration into application root directory.
# Usage: $ spr-mkdefconfig <configuration name>...
function spr-mkdefconfig() {
	if [ "$#" == 0 ] || [ "${1:0:1}" == "-" ]; then
		echo "Usage: ${FUNCNAME[0]} <configname>"
		echo ""
		echo "Save the current configuration into your application root directory."
		echo ""
		echo "Arguments:"
		echo "  configname      New configuration name to save."
		echo "                  It must be an alphanumeric string (A-Z, a-z, 0-9, _)."
		echo ""
		return 1
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi
	_check_spresense_home_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	if [[ ! -f "${SPRESENSE_SDK}/nuttx/.config" ]]; then
		echo "Error: Your application is not configured yet."
		echo "       Please run 'spr-config'."
		return 1
	fi

	local config_name="$1"

	if ! [[ "$config_name" =~ ^[A-Za-z0-9_]+$ ]]; then
		echo "Error: Invalid configname '${config_name}'."
		echo "       Use alphanumeric characters only (A-Z, a-z, 0-9, _)."
		return 1
	fi

	# Copy default/defconfig to SPRESENSE_HOME/configs/default
	mkdir -p "${SPRESENSE_HOME}/configs"
	cp -rf "${SPRESENSE_SDK}/sdk/configs/default" "${SPRESENSE_HOME}/configs/"

	cd ${SPRESENSE_SDK}/sdk || return 1
	./tools/mkdefconfig.py -d "${SPRESENSE_HOME}" "userapp/${config_name}"
	if [ $? -ne 0 ]; then
		echo "Error: Failed to create config 'userapp/${config_name}'."
		cd - &> /dev/null
		return 1
	fi
	echo "Config 'userapp/${config_name}' successfully created in ${SPRESENSE_HOME}/configs"
	cd - &> /dev/null
}

# Name: spr-flash
# Note: Flash nuttx.spk via flash.sh wrapper.
# Usage: $ spr-flash <options>
function spr-flash() {
	# Check if first argument is valid
	if [ "$#" -gt 0 ]; then
		if ! [[ "$1" =~ \.(spk|espk)$ ]] && [[ "$1" != "-w" ]] && [[ "$1" != "-r" ]] && [[ "$1" != "-B" ]]; then
			echo "Usage: ${FUNCNAME[0]}"
			echo "       ${FUNCNAME[0]} <spk>..."
			echo "       ${FUNCNAME[0]} [-B]"
			echo "       ${FUNCNAME[0]} [-w <file>]"
			echo ""
			echo "Flash your built application into your board."
			echo "Please set the serial port name connected to the main board by 'spr-set-port'."
			echo "The serial baud rate can be set by 'spr-set-baud'. If not set, the default 115200 is used."
			echo ""
			echo "If you see \"WARNING: New bootloader is required\" when running 'spr-flash',"
			echo "please flash the bootloader by 'spr-flash -B'."
			echo ""
			echo "Options:"
			echo "  spk           Flash the specified .spk or .espk file(s) into the board."
			echo "  -B            Flash bootloader into the board."
			echo "  -w file       Upload the specified file into user file system ('/mnt/spif') on the board."
			echo ""
			return 1
		fi
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	if [ -z "${SPRESENSE_PORT}" ]; then
		echo "Error: Serial port is not set."
		echo "       Please run"
		echo ""
		echo "       $ spr-set-port <port>"
		echo ""
		return 1
	fi

	local port="-c ${SPRESENSE_PORT}";

	local baud="";
	if [ "${SPRESENSE_BAUD}" != "" ]; then
		baud="-b ${SPRESENSE_BAUD}"
	fi

	if [ "$#" -gt 0 ]; then
		${SPRESENSE_SDK}/sdk/tools/flash.sh ${port} ${baud} "$@"
	else
		_check_spresense_home_environment
		if [ $? -ne 0 ]; then
			return 1
		fi
		local approot=$(basename "${SPRESENSE_HOME}")
		if [ ! -f "${SPRESENSE_HOME}/out/${approot}.nuttx.spk" ]; then
			echo "Error: '${SPRESENSE_HOME}/out/${approot}.nuttx.spk' does not exist."
			echo "       Please run 'spr-make' first or specify a .spk/.espk file."
			return 1
		fi
		local worker_spk_glob="${SPRESENSE_HOME}/out/worker/*.spk"
		if compgen -G "${worker_spk_glob}" > /dev/null; then
			${SPRESENSE_SDK}/sdk/tools/flash.sh ${port} ${baud} "${SPRESENSE_HOME}/out/*.spk" "${worker_spk_glob}"
		else
			${SPRESENSE_SDK}/sdk/tools/flash.sh ${port} ${baud} "${SPRESENSE_HOME}/out/*.spk"
		fi
	fi
}

# Name: spr-terminal
# Note: Open serial terminal via terminal.sh wrapper.
# Usage: $ spr-terminal
function spr-terminal() {
	if [ "$#" != 0 ]; then
		echo "Usage: ${FUNCNAME[0]}"
		echo ""
		echo "Open serial terminal."
		echo "Please set the serial port name connected to the main board by 'spr-set-port'."
		echo "If not set, list the available ports."
		echo ""
		return 1
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	local port="";
	if [ "${SPRESENSE_PORT}" != "" ]; then
		port="-c ${SPRESENSE_PORT}"
	fi
	${SPRESENSE_SDK}/sdk/tools/terminal.sh ${port}
}

# Name: spr-go-sdk
# Note: Move current directory to SDK directory.
# Usage: $ spr-go-sdk
function spr-go-sdk() {
	if [ "$#" != 0 ]; then
		echo "Usage: ${FUNCNAME[0]}"
		echo ""
		echo "Change directory to your Spresense SDK directory."
		echo ""
		return 1
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	if [ ! -d "${SPRESENSE_SDK}/sdk" ]; then
		echo "Error: '${SPRESENSE_SDK}/sdk' directory does not exist."
		return 1
	fi

	echo "Change directory to ${SPRESENSE_SDK}/sdk"
	cd ${SPRESENSE_SDK}/sdk
}

# Name: spr-go-approot
# Note: Move current directory to user application root directory.
# Usage: $ spr-go-approot
function spr-go-approot() {
	if [ "$#" != 0 ]; then
		echo "Usage: ${FUNCNAME[0]}"
		echo ""
		echo "Change directory to your application root directory."
		echo ""
		return 1
	fi

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi
	_check_spresense_home_environment
	if [ $? -ne 0 ]; then
		return 1
	fi

	echo "Change directory to ${SPRESENSE_HOME}"
	cd ${SPRESENSE_HOME}
}

# Name: spr-make
# Note: Build SDK and user application same as make command.
# Usage: $ spr-make [build options]
function spr-make() {
	local job="-j"
	local make_args=()

	for arg in "$@"; do
		if [[ "$arg" == "-h" ]] || [[ "$arg" == "--help" ]]; then
			echo "Usage: ${FUNCNAME[0]} [(all)|distclean|clean|menuconfig] [V=1|2] [-jN]"
			echo ""
			echo "Build and clean your application."
			echo ""
			echo "Options:"
			echo "  all            Build (Must be configured in advance by 'spr-config')."
			echo "  distclean      Remove all build artifacts and configuration files."
			echo "  clean          Remove all build artifacts but keep configuration files."
			echo "  menuconfig     Open menuconfig to configure your application."
			echo "  V=1|2          Show verbose build output."
			echo "  -jN            Specify the N number of jobs to run simultaneously."
			echo "                 If this option is not specified, -j is used by default."
			echo "                 (e.g. -j4, -j8. Note that -j0/-j1 disables parallel build)."
			echo ""
			return 1
		elif [[ "$arg" =~ ^-j[0-9]+$ ]]; then
			if [[ "$arg" == "-j0" ]] || [[ "$arg" == "-j1" ]]; then
				job=""
			else
				job="$arg"
			fi
		else
			make_args+=("$arg")
		fi
	done

	# Check current environment
	_check_spresense_sdk_environment
	if [ $? -ne 0 ]; then
		return 1
	fi
	if [[ -n "${SPRESENSE_HOME}" ]] && [[ ! -d "${SPRESENSE_HOME}" ]]; then
		unset SPRESENSE_HOME
		# Save current variable
		_save_spresense_environment
	fi

	if [[ ! -f "${SPRESENSE_SDK}/nuttx/.config" ]]; then
		echo "Error: Your application is not configured yet."
		echo "       Please run 'spr-config'."
		return 1
	fi

	make ${job} -C ${SPRESENSE_SDK}/sdk "${make_args[@]}"
	if [[ $? -eq 0 ]] && [[ -d "${SPRESENSE_HOME}" ]]; then
		# Check if arguments contain clean, config or help
		local skip_copy=0
		local remove_build=0
		local is_distclean=0
		for arg in "$@"; do
			if [[ "$arg" == *config ]] || [[ "$arg" == help ]]; then
				skip_copy=1
			fi
			if [[ "$arg" == *clean* ]]; then
				skip_copy=1
				remove_build=1
			fi
			if [[ "$arg" == distclean ]]; then
				is_distclean=1
			fi
			if [[ "$arg" == all ]]; then
				skip_copy=0
				remove_build=0
				is_distclean=0
			fi
		done

		if [[ $skip_copy -eq 0 ]] && [[ -f "${SPRESENSE_SDK}/sdk/nuttx.spk" ]]; then
			local approot=$(basename "${SPRESENSE_HOME}")
			echo "Build successful. Copy build artifacts to ${SPRESENSE_HOME}/out"
			mkdir -p ${SPRESENSE_HOME}/out
			cp -f ${SPRESENSE_SDK}/sdk/nuttx.spk ${SPRESENSE_HOME}/out/${approot}.nuttx.spk
			cp -f ${SPRESENSE_SDK}/sdk/nuttx ${SPRESENSE_HOME}/out/${approot}.nuttx
			cp -f ${SPRESENSE_SDK}/sdk/nuttx.map ${SPRESENSE_HOME}/out/${approot}.nuttx.map
			cp -f ${SPRESENSE_SDK}/sdk/System.map ${SPRESENSE_HOME}/out/

			if [[ -d "${SPRESENSE_SDK}/sdk/workerspks" ]]; then
				local worker_file
				for worker_file in "${SPRESENSE_SDK}/sdk/workerspks"/*; do
					mkdir -p ${SPRESENSE_HOME}/out/worker
					if [[ -f "${worker_file}" ]]; then
						cp -f "${worker_file}" "${SPRESENSE_HOME}/out/worker/"
					fi
				done
			fi
		fi

		if [[ $remove_build -eq 1 ]]; then
			if [[ $is_distclean -eq 1 ]]; then
				echo "Distclean successful. Remove build artifacts from ${SPRESENSE_HOME}/out"
			else
				echo "Clean successful. Remove build artifacts from ${SPRESENSE_HOME}/out"
			fi
			rm -rf ${SPRESENSE_HOME}/out
		fi
	fi
}

# Load current variable
function _load_spresense_environment() {
	if [ -f ${SPRESENSE_SDK}/.spresense_env ]; then
		source ${SPRESENSE_SDK}/.spresense_env
	fi
	export SPRESENSE_HOME
	export SPRESENSE_PORT
	export SPRESENSE_BAUD
}

# Save current variable
function _save_spresense_environment() {
	# Export parameters into configuration file
	echo "SPRESENSE_HOME=${SPRESENSE_HOME}" > ${SPRESENSE_SDK}/.spresense_env
	echo "SPRESENSE_SDK=${SPRESENSE_SDK}" >> ${SPRESENSE_SDK}/.spresense_env
	echo "SPRESENSE_PORT=${SPRESENSE_PORT}" >> ${SPRESENSE_SDK}/.spresense_env
	echo "SPRESENSE_BAUD=${SPRESENSE_BAUD}" >> ${SPRESENSE_SDK}/.spresense_env
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
	if ! command -v arm-none-eabi-gcc &> /dev/null; then
		echo "   GCC_VERSION = 'arm-none-eabi-gcc' is not found in PATH."
	else
		echo "   GCC_VERSION = `arm-none-eabi-gcc --version | head -n 1`"
	fi
	echo "          HOST = `uname -sm`"
	echo "======================================="
}

# Check current variable
function _check_spresense_sdk_environment() {
	if [[ -z "${SPRESENSE_SDK}" ]] || [[ ! -d "${SPRESENSE_SDK}" ]]; then
		echo "Error: Your Spresense SDK directory is invalid."
		echo "       Please source tools/build-env.sh under spresense/sdk."
		echo ""
		echo "       $ source tools/build-env.sh"
		echo ""
		return 1
	fi
	return 0
}

function _check_spresense_home_environment() {
	if [ -z "${SPRESENSE_HOME}" ]; then
		echo "Error: Your application root directory is not set."
		echo "       Please create or set the application root directory."
		echo ""
		echo "       $ spr-create-approot <application root directory>"
		echo "          or"
		echo "       $ spr-set-approot <application root directory>"
		echo ""
		return 1
	elif [ ! -d "${SPRESENSE_HOME}" ]; then
		echo "Error: '${SPRESENSE_HOME}' directory does not exist."
		echo "       Your application root directory was cleared."
		echo ""
		unset SPRESENSE_HOME
		# Save current variable
		_save_spresense_environment
		return 1
	fi
	return 0
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

# Set repository root to SPRESENSE_SDK
export SPRESENSE_SDK=$(dirname $(dirname ${SCRIPT_DIR}))

# Check and source spresenseenv setup if GCC is not in PATH
if ! command -v arm-none-eabi-gcc &> /dev/null; then
	if [ -f "${HOME}/spresenseenv/setup" ]; then
		source "${HOME}/spresenseenv/setup"
	else
		echo "Warning: The tools required for development are not installed."
		echo "         Please install using install-tools.sh."
		echo ""
		echo "         $ bash ${SPRESENSE_SDK}/install-tools.sh"
		echo ""
	fi
fi

# Load current variable
_load_spresense_environment

if [ -z "${SPRESENSE_HOME}" ]; then
	echo "Warning: Your application root directory is not set."
	echo "         If you create user applications by spr-command,"
	echo ""
	echo "         $ spr-create-approot <application root directory>"
	echo "            or"
	echo "         $ spr-set-approot <application root directory>"
	echo ""
elif [ ! -d "${SPRESENSE_HOME}" ]; then
	echo "Warning: '${SPRESENSE_HOME}' directory does not exist."
	echo "          Your application root directory was cleared."
	echo ""
	unset SPRESENSE_HOME
fi

if [ -d "${SPRESENSE_HOME}" -a \( ! -f "${SPRESENSE_HOME}/.sdksubdir" -o ! -f "${SPRESENSE_HOME}/.approotdir" \) ]; then
	if [ -f "${SPRESENSE_HOME}/Application.mk" ]; then
		echo "Warning: '${SPRESENSE_HOME}' is created for Spresense SDK1.x version."
		echo "         Please create a new project directory with next command."
		echo ""
		echo "         $ spr-create-approot <new application root directory>"
		echo ""
	else
		echo "Warning: '${SPRESENSE_HOME}' is invalid."
		echo "         Please run next command to create the application root directory."
		echo ""
		echo "         $ spr-create-approot ${SPRESENSE_HOME}"
		echo ""
	fi
	unset SPRESENSE_HOME
fi

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
