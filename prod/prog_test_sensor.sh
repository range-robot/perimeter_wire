#!/bin/bash

set -e


echo "Building..."

# pre build, for speed up programming
catkin build perimeter_wire_bootloader perimeter_wire_sensor_firmware perimeter_wire_sensor_driver


echo "Programming..."

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR


echo "Bootloader"
../firmware/bootloader/scripts/program_bootloader.sh

echo "Firmware"
# only 10 seconds ...
../firmware/sensor/scripts/program_with_bootloader.sh


echo "Programming finished successfull."
echo "Waiting for firmware startup"
sleep 11

echo "Testing..."
rosrun perimeter_wire_sensor_driver console -p /dev/ttyACM0 -d 4 -D -r 1 -c 13651


popd