#!/bin/bash
PROMT_PREFIX=SpresenseSDK:
echo "This script has to be sourced." 
echo ". ./spresense_env.sh or source spresense_env.sh"
echo " "
docker pull sa7bkk/sdk-env-minimal
pwd=$(pwd)
alias spresense='docker run --rm -it -u `id -u`:`id -g`  -v $pwd:/spresense -w /spresense/sdk sa7bkk/sdk-env-minimal'
if [[ $PS1 !=  *$PROMT_PREFIX* ]]; then
	PS1=$PROMT_PREFIX$PS1
fi

echo "After this script executed successfully:"
echo "Usage: spresense [command]"
echo "This will run command in the Spresense SDK docker environment."
