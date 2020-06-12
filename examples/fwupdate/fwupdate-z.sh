#!/bin/bash

if [ `uname -o` = "Cygwin" ]; then
  HOSTEXEEXT=.exe
fi

SCRDIR=`dirname $0`
TOPDIR=${SCRDIR}/../../nuttx
APPDIR=${SCRDIR}/../../sdk/apps

. ${TOPDIR}/.config

usage () {
  echo "Usage:"
  echo "  ${0} [-d /dev/ttyS#] [FILE]..."
  echo "Options:"
  echo "    [FILE] : /path/to/spk file"
  echo "Description:"
  echo "  After \"fwupdate -z\" on the Target Board, run this script."
  echo "Examples:"
  echo "  $0 -d /dev/ttySx nuttx.spk"
  echo "  $0 -d /dev/ttySx nuttx.spk ../firmware/spresense/bin/{loader,gnssfw}.espk"
  exit 1
}

while [ ! -z "$1" ]; do
  case $1 in
  -d )
    shift
    DEV="-d $1"
    ;;
  -h )
    usage
    ;;
  * )
    break;
    ;;
  esac
  shift
done

if [ $# -eq 0 ];then
  usage
fi

${SCRDIR}/package.sh $@

if [ -f ${APPDIR}/system/zmodem/sz${HOSTEXEEXT} ]; then
  echo "Send package.bin to the Target Board..."
  ${APPDIR}/system/zmodem/sz${HOSTEXEEXT} ${DEV} $PWD/package.bin
  \rm -f $PWD/package.bin
else
  echo "ERROR: Not exist \"sz${HOSTEXEEXT}\" tool and please compile it"
  echo " % cd $(cd ${APPDIR} && pwd)/system/zmodem"
  echo " % make -f Makefile.host clean all TOPDIR=$(cd ${TOPDIR} && pwd) APPDIR=$(cd ${APPDIR} && pwd)"
  echo " See $(cd ${APPDIR} && pwd)/system/zmodem/README.txt for details"
fi

