#!/bin/bash
PROMT_PREFIX=SpresenseSDK:
REPO=devworldsony/spresense-sdk-env
echo "This script has to be sourced." 
echo ". ./spresense_env.sh or source spresense_env.sh"
echo " "
docker pull $REPO
pwd=$(pwd)
alias spresense='docker run --rm -it -u `id -u`:`id -g`  -v '"$pwd:/spresense -w /spresense/sdk $REPO"
if [[ $PS1 !=  *$PROMT_PREFIX* ]]; then
	PS1=$PROMT_PREFIX$PS1
fi

echo "After this script executed successfully:"
echo "Usage: spresense [command]"
echo "This will run command in the Spresense SDK docker environment."
