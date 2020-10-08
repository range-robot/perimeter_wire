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
firmware=$(rospack find perimeter_wire_bootloader)/bootloader.hex
version=$(rospack find perimeter_wire_bootloader)/version
cfg=$(rospack find perimeter_wire_bootloader)/openocd.cfg
clear=false
while getopts "h?f:c:C" opt; do
    case "$opt" in
    h|\?)
        show_help_ctr
        exit 0
        ;;
    f)
        version=
        firmware=$OPTARG
        ;;
    c)
        cfg=$OPTARG
        ;;
    C)
        clear=true
    esac
done

shift $((OPTIND-1))
[ "${1:-}" = "--" ] && shift

if [ $clear == true ]; then
    echo "Clearing flash"
    # disable lock (first section) and erase all
    openocd -f $cfg -c "init; reset init; flash protect 0 0 15 off; exit;"
    openocd -f $cfg -c "init; reset init; flash erase_address unlock 0 0x00020000; exit;"
    exit
fi

# Having problems with ATMEL-ICE? Read this:
# https://community.atmel.com/forum/samd21-fuse-settings-causing-programming-failure
# Also make sure your Atmel Studio configuration is correct
# Dont't overwrite bootloader. Use the bl linker script and incremental uploading.

echo "Programming bootlaoder (Using Programmer)"


# disable lock (first section)
openocd -f $cfg -c "init; reset init; flash protect 0 0 0 off; exit;"

openocd -f $cfg -c "program $firmware verify reset exit"

# enable lock (first section)
echo "Enable lock for bootloader section"
openocd -f $cfg -c "init; reset init; flash protect 0 0 0 on; reset; exit;"

if [ -f $version ]; then
  echo "Programmed firmware $(cat $version)"
else
  echo "Programmed firmware (unknown version)"
fi