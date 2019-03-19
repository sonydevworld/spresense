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
SCRIPT_NAME=`readlink -e "${BASH_SOURCE[0]}"`
SCRIPT_DIR=`dirname "$SCRIPT_NAME"`

############################################################################
# Public function                                                          #
############################################################################

# Create Spresense home environmet with mkapp
function create_spresense_home()
{
    echo "Creating application directory into ${SPRESENSE_HOME}"
    ${SCRIPT_DIR}/mkappsdir.py -s ${SPRESENSE_HOME} "User application"
}

# TAB completion for ./tools/config.py
function _spresense_config_completion() {
	local cur prev cword

	# get command line arguments
	_get_comp_words_by_ref -n : cur prev cword

	if [ "${prev}" == "-d" -o "${prev}" == "--dir" ]; then
		# If use '-d' or '--dir' option, use filename completion
		compopt -o nospace
		COMPREPLY=($(compgen -d -- "${cur}" | sed 's#$#/#g'))
	else
		compopt +o nospace
		if [ "${cur:0:1}" == "-" ]; then
			# For option prediction
			SOPT=`${SPRESENSE_SDK}/sdk/tools/config.py -h | grep -oE "\--[a-zA-Z]+"`
			LOPT=`${SPRESENSE_SDK}/sdk/tools/config.py -h | grep -oE "^[ ]+-[a-zA-Z]{1}"  | tr -d " "`
			LIST="${SOPT} ${LOPT}"
		else
			LIST=`${COMP_WORDS[@]} -l | tail -n +2 | tr -d "\t"`
		fi
		COMPREPLY=($(compgen -W "${LIST}" -- "${cur}"))
	fi
}

# Add configuration command behalf of tools/config.py
function spresense_config() {
	cd ${SPRESENSE_SDK}/sdk
	./tools/config.py $@
}

############################################################################
# Public parameter definition                                              #
############################################################################

# Set <user home>/Spresense to SPRESENSE_HOME
export SPRESENSE_HOME=${HOME}/Spresense

# Set repository root to SPRESENSE_SDK
export SPRESENSE_SDK=$(dirname $(dirname ${SCRIPT_DIR}))

############################################################################
# Environment setup                                                        #
############################################################################

#
# User application setup
#

if [ ! -d ${SPRESENSE_HOME} ]; then
    echo "Warning: Spresense user application directory is not exists."
    echo "         Please run"
    echo "         $ create_spresense_home"
fi

# Export parameters into configuration file
echo "SPRESENSE_HOME=${SPRESENSE_HOME}" > ${HOME}/.spresense_env
echo "SPRESENSE_SDK=${SPRESENSE_SDK}" >> ${HOME}/.spresense_env

if [ -f ${SPRESENSE_HOME}/Application.mk ]; then
    # Echo result
    echo "Set user application root directory to ${SPRESENSE_HOME}."
    echo "You can put application directory into ${SPRESENSE_HOME}"
else
    if [ -d ${SPRESENSE_HOME} ]; then
        # Echo warning
        echo "Warning: Your environment(${SPRESENSE_HOME}) doesn't have makefiles."
        echo "         Please move ${SPRESENSE_HOME} to other place and run"
        echo "         $ create_spresense_home"
    fi
fi

#
# TAB completion
#

complete -F _spresense_config_completion tools/config.py ./tools/config.py spresense_config

