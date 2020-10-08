#!/bin/bash

set -e

#/bin/bash

show_help() {
  echo "firmware programming tool"
  echo "usage: $0 [-P PROGRAMMER_OPTIONS] [-f FIRMWARE]"
  echo "Options:"
  echo -e "\t-f\tFirmware file to be programmed."
  echo -e "\t-c\topenocd config file."
}

# read optionss
OPTIND=1
# package destination only works when installed
firmware=$(rospack find perimeter_wire_sensor_firmware)/firmware.bin
version=$(rospack find perimeter_wire_sensor_firmware)/version
while getopts "h?f:" opt; do
    case "$opt" in
    h|\?)
        show_help_ctr
        exit 0
        ;;
    f)
        version=
        firmware=$OPTARG
        ;;
    esac
done

shift $((OPTIND-1))
[ "${1:-}" = "--" ] && shift

bossac --offset 0x2000 -i -U -e -w $firmware -v -R

if [ -f $version ]; then
  echo "Programmed firmware $(cat $version)"
else
  echo "Programmed firmware (unknown version)"
fi