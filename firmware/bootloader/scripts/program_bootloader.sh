#!/bin/bash

set -e


echo "Programming bootlaoder (Using Programmer)"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR

# disable lock (first section)
openocd -f ../openocd.cfg -c "init; reset init; flash protect 0 0 0 off; exit;"


catkin build --this --make-args upload_bootloader

# enable lock (first section)
echo "Enable lock for bootloader section"
openocd -f ../openocd.cfg -c "init; reset init; flash protect 0 0 0 on; exit;"