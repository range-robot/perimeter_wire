#!/bin/bash

set -e

# Having problems with ATMEL-ICE? Read this:
# https://community.atmel.com/forum/samd21-fuse-settings-causing-programming-failure
# Also make sure your Atmel Studio configuration is correct
# Dont't overwrite bootloader. Use the bl linker script and incremental uploading.

echo "Programming bootlaoder (Using Programmer)"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR

# disable lock (first section)
openocd -f ../openocd.cfg -c "init; reset init; flash protect 0 0 0 off; exit;"


catkin build --this --make-args upload_bootloader

# enable lock (first section)
echo "Enable lock for bootloader section"
openocd -f ../openocd.cfg -c "init; reset init; flash protect 0 0 0 on; reset; exit;"