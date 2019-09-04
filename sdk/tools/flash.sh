#!/bin/bash
############################################################################
# tools/flash.sh
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

CURRENT_DIR=`pwd`
SCRIPT_NAME=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)/$(basename "${BASH_SOURCE[0]}")
SCRIPT_DIR=`dirname "$SCRIPT_NAME"`

# function   : show_help
# description: Show help and exit.
function show_help()
{
	echo "  Usage:"
	echo "       $0 [-c <UART Port> -b <UART Baudrate>] <(e)spk file> [<(e)spk file> ...]"
	echo ""
	echo "  Mandatory argument:"
	echo "       (e)spk file path"
	echo ""
	echo "  Optional arguments:"
	echo "       -c: Serial port (default: /dev/ttyUSB0)"
	echo "       -b: Serial baudrate (default: 115200)"
	echo "       -e: Extract loader archive"
	echo "       -l: Flash loader"
	echo "       -w: Worker load mode"
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

# Option handler
# -b: UART Baudrate (default: 115200)
# -c: UART Port (default: /dev/ttyUSB0)
# -h: Show Help
UART_BAUDRATE="115200"
UART_PORT="/dev/ttyUSB0"
UPDATE_ZIP=""
LOADR_PATH=""
FLASH_MODE="SPK"
while getopts b:c:s:e:l:wh OPT
do
	case $OPT in
		'b' ) UART_BAUDRATE=$OPTARG;;
		'c' ) UART_PORT=$OPTARG;;
		'e' ) UPDATE_ZIP=$OPTARG;;
		'l' ) LOADR_PATH=$OPTARG;;
		'w' ) FLASH_MODE="ELF";;
		'h' ) show_help;;
	esac
done

# Shift argument position after option
shift $(($OPTIND - 1))

if [ "${UPDATE_ZIP}" != "" ]; then
	${SCRIPT_DIR}/eula.py -i ${UPDATE_ZIP}
	exit
fi

# Check loader version
${SCRIPT_DIR}/eula.py -c

if [ "${FLASH_MODE}" == "SPK" ]; then
	# Pickup spk and espk files
	ESPK_FILES=""
	SPK_FILES=""

	for arg in $@
	do
		if [ "`echo ${arg} | grep "\.spk$"`" ]; then
			SPK_FILES="${SPK_FILES} ${arg}"
		elif [ "`echo ${arg} | grep "\.espk$"`" ]; then
			ESPK_FILES="${ESPK_FILES} -S ${arg}"
		fi
	done

	if [ "${LOADR_PATH}" != "" ]; then
		ESPK_FILES="`find ${LOADR_PATH} -name "*.espk"`"
		SPK_FILES="`find ${LOADR_PATH} -name "*.spk"`"
	fi

	if [ "${SPK_FILES}${ESPK_FILES}" == "" ]; then
		echo "ERROR: No (e)spk files are contains."
		echo ""
		show_help
	fi

	# Flash spk files into spresense board
	${SCRIPT_DIR}/${PLATFORM}/flash_writer -s -c ${UART_PORT} -d -b ${UART_BAUDRATE} -n ${ESPK_FILES} ${SPK_FILES}
elif [ "${FLASH_MODE}" == "ELF" ]; then
	if [ "$#" == "0" ]; then
		echo "ERROR: No elf files are contains."
		echo ""
		show_help
	fi

	# Flash elf files into spresense board
	${SCRIPT_DIR}/${PLATFORM}/xmodem_writer -d -c ${UART_PORT} $@
fi
