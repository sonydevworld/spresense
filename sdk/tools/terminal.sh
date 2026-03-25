#!/usr/bin/env bash
############################################################################
# tools/terminal.sh
#
#   Copyright 2026 Sony Semiconductor Solutions Corporation
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

# function   : show_help
# description: Show help and exit.
function show_help()
{
	echo "  Usage:"
	echo "       $0 [-c <port>] [miniterm options]"
	echo ""
	echo "  Optional arguments:"
	echo "       -c: Serial port"
	echo "       -h: Show help (This message)"
	exit
}

# Get platform type
case "$(uname -s)" in
	Linux*)
		PLATFORM=linux
		;;
	Darwin*)
		PLATFORM=macos
		;;
	CYGWIN*|MINGW32*|MSYS*)
		PLATFORM=windows
		;;
	*)
		echo "ERROR: Unknown platform"
		echo ""
		show_help
		;;
esac

# Exec file extension postfix
EXEEXT=""

# Option handler
# -c: UART Port
# -h: Show Help
UART_PORT=
while getopts c:h OPT
do
	case $OPT in
		'c' ) UART_PORT=$OPTARG;;
		'h' ) show_help;;
	esac
done

# Shift argument position after option
shift $(($OPTIND - 1))

# WSL/WSL2 detection
if [ "${PLATFORM}" == "linux" ]; then
	if [ "$(uname -r | grep -i microsoft)" != "" ]; then
		if [[ "$UART_PORT" != /dev/* ]]; then
			# WSL/WSL2 is a linux but USB related SDK tools
			# should use windows binary.
			PLATFORM=windows
			EXEEXT=".exe"
		fi
	fi
fi

# Open serial terminal
${SCRIPT_DIR}/${PLATFORM}/miniterm${EXEEXT} --raw --eol LF ${UART_PORT} "$@"
