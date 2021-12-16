#!/bin/bash

OUTPUT=package.bin

usage () {
  echo "Usage:"
  echo "  ${0} [FILE]..."
  echo "Options:"
  echo "    [FILE] : /path/to/spk file"
  echo "Description:"
  echo "  Output a package file named as \"$OUTPUT\""
  echo "Examples:"
  echo "  ../examples/fwupdate/package.sh nuttx.spk"
  echo "  ../examples/fwupdate/package.sh nuttx.spk ../firmware/spresense/{loader,gnssfw}.espk"
  exit 1
}

put_int32 () {
  v3=$(( ($1 >> 24) & 0xff ))
  v2=$(( ($1 >> 16) & 0xff ))
  v1=$(( ($1 >>  8) & 0xff ))
  v0=$((  $1        & 0xff ))
  hv=$(printf '\\x%02x\\x%02x\\x%02x\\x%02x' $v0 $v1 $v2 $v3)
  printf "%b%b%b%b" $hv
}

get_fw_size ()
{
  wc -c $1 | awk '{print $1}'
}

# 0 FW_APP
# 1 FW_SYS
# 2 FW_UPDATER
# 3 FW_SBL

get_fw_type () {
  filename=`basename $1`
  case "$filename" in
    nuttx*)        echo "0" ;;
    sysutil*)      echo "0" ;;
    sbl*)          echo "3" ;;
    updater*|usb*) echo "2" ;;
    *)             echo "1" ;;
  esac
}

add_header () {
  fwtype=$1
  fwsize=$2
  put_int32 $fwtype
  put_int32 $fwsize
}

if [ $# -eq 0 ];then
  usage
  exit 1
fi

cp -f /dev/null $OUTPUT

for file in "$@"
do
  if [ -f ${file} ]; then
    type=`get_fw_type $file`
    size=`get_fw_size $file`
    echo "Pack: $type $size $file"
    add_header $type $size >> $OUTPUT
    cat $file >> $OUTPUT
  else
    echo "Error: File Not Found!! (${file})"
    \rm $OUTPUT
    exit 1
  fi
done

echo "===================="
echo "Created ${OUTPUT}"
echo "===================="
