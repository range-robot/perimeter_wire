#!/bin/bash


echo "Programming using bootloader"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR

catkin build --this --make-args upload_with_bootloader
