#!/bin/bash
############################################################################
# tools/completion.sh
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

############################################################################
# Public function                                                          #
############################################################################

# TAB completion for ./tools/config.py
function _spresense_config_completion() {
	local cur prev

	# get command line arguments
	cur=${2}
	prev=${3}

	if [ "${prev}" == "-d" -o "${prev}" == "--dir" ]; then
		# If use '-d' or '--dir' option, use filename completion
		compopt -o nospace
		COMPREPLY=($(compgen -d -- "${cur}" | sed 's#$#/#g'))
	else
		compopt +o nospace
		if [ "${cur:0:1}" == "-" ]; then
			# For option prediction
			SOPT=`${COMP_WORDS[0]} -h | grep -oE "\--[a-zA-Z]+"`
			LOPT=`${COMP_WORDS[0]} -h | grep -oE "^[ ]+-[a-zA-Z]{1}"  | tr -d " "`
			LIST="${SOPT} ${LOPT}"
		else
			LIST=`${COMP_WORDS[@]} -l | tail -n +2 | tr -d "\t"`
		fi
		COMPREPLY=($(compgen -W "${LIST}" -- "${cur}"))
	fi
}

############################################################################
# Environment setup                                                        #
############################################################################

#
# TAB completion
#

complete -F _spresense_config_completion tools/config.py ./tools/config.py spr-config

