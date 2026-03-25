#!/usr/bin/env bash
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
		if [ "${cur:0:1}" == "-" -a "${cur}" != "--info" -a "${cur}" != "-i" ]; then
			# For option prediction
			SOPT=`${COMP_WORDS[0]} -h | grep -oE "\--[a-zA-Z]+"`
			LOPT=`${COMP_WORDS[0]} -h | grep -oE "^[ ]+-[a-zA-Z]{1}"  | tr -d " "`
			LIST="${SOPT} ${LOPT}"
		else
			LIST=`${COMP_WORDS[0]} -l | tail -n +2 | tr -d "\t"`
		fi
		COMPREPLY=($(compgen -W "${LIST}" -- "${cur}"))
	fi
}

# TAB completion for spr-import-example
function _spresense_import_example_completion() {
	local cur prev base1 base2 old_nullglob rel suffix name
	local -a list

	cur="${COMP_WORDS[COMP_CWORD]}"
	prev=""
	if [ ${COMP_CWORD} -gt 0 ]; then
		prev="${COMP_WORDS[COMP_CWORD-1]}"
	fi
	base1="${SPRESENSE_SDK}/examples"
	base2="${SPRESENSE_SDK}/sdk/apps/examples"
	list=()

	compopt -o nospace

	if [ -z "${SPRESENSE_SDK}" ]; then
		return 0
	fi

	old_nullglob="$(shopt -p nullglob)"
	shopt -s nullglob

	# Keep progressing completion when the shell already inserted a space.
	if [ -z "${cur}" ]; then
		if [[ "${prev}" == "examples/" ]] || [[ "${prev}" == "apps/examples/" ]]; then
			cur="${prev}"
		elif [[ "${prev}" == "${base1}/" ]] || [[ "${prev}" == "${base2}/" ]]; then
			cur="${prev}"
		fi
	fi

	if [[ "${cur}" == /* ]]; then
		if [[ "${cur}" == "${base1}"* ]]; then
			for d in "${cur}"*; do
				[ -d "${d}" ] && list+=("${d}")
			done
		elif [[ "${cur}" == "${base2}"* ]]; then
			for d in "${cur}"*; do
				[ -d "${d}" ] && list+=("${d}")
			done
		fi
	else
		# Step 1: complete only the root candidates while typing prefix (e.g. exa -> examples/)
		if [[ "${cur}" != */* ]]; then
			list+=($(compgen -W "examples/ apps/examples/" -- "${cur}"))
		else
			# Step 2: after root is selected, complete only first-level app names.
			if [[ "${cur}" == examples/* ]] && [ -d "${base1}" ]; then
				suffix="${cur#examples/}"
				if [[ "${suffix}" != */* ]]; then
					for d in "${base1}"/*; do
						if [ -d "${d}" ]; then
							name="${d##*/}"
							if [[ "${name}" == "${suffix}"* ]]; then
								rel="${d#${SPRESENSE_SDK}/}"
								list+=("${rel}")
							fi
						fi
					done
				fi
			elif [[ "${cur}" == apps/examples/* ]] && [ -d "${base2}" ]; then
				suffix="${cur#apps/examples/}"
				if [[ "${suffix}" != */* ]]; then
					for d in "${base2}"/*; do
						if [ -d "${d}" ]; then
							name="${d##*/}"
							if [[ "${name}" == "${suffix}"* ]]; then
								rel="${d#${SPRESENSE_SDK}/sdk/}"
								list+=("${rel}")
							fi
						fi
					done
				fi
			else
				# Keep root completion available for partial hierarchical inputs.
				list+=($(compgen -W "examples/ apps/examples/" -- "${cur}"))
			fi
		fi
	fi

	eval "${old_nullglob}"
	COMPREPLY=($(printf "%s\n" "${list[@]}" | sort -u))
}

############################################################################
# Environment setup                                                        #
############################################################################

#
# TAB completion
#

complete -F _spresense_config_completion tools/config.py ./tools/config.py spr-config
complete -F _spresense_import_example_completion spr-import-example

