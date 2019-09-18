cmake_minimum_required(VERSION 2.8.3)

project(perimeter_wire_generator_firmware)

# TODO BUILD AND INSTALL

find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS include
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
