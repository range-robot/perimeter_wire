#!/bin/bash

set -e


echo "Programming firmware (Using Programmer)"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR

# erase nv storage
openocd -f ../openocd.cfg -c "init; reset init; flash erase_address pad 0x1FF60 160; exit;"

catkin build --this --make-args upload_firmware
