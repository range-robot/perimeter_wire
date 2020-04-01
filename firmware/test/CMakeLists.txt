cmake_minimum_required(VERSION 2.8.3)
project(perimeter_wire_firmware_tests)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  perimeter_wire_generator_firmware
  perimeter_wire_sensor_firmware
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
)

###########
## Build ##
###########
add_definitions("-D__SAMD21E17D__")
include_directories(
  # DFP
  sensor/samd21/samd21d/include
  sensor/cmsis/CMSIS/Core/Include

  # Project
  sensor/include
  sensor
  sensor/config
  sensor/hal/include
  sensor/hal/utils/include
  sensor/hpl/adc
  sensor/hpl/core
  sensor/hpl/gclk
  sensor/hpl/port
  sensor/hpl/pm
  sensor/hpl/tc
  sensor/hri
  sensor/usb
  sensor/usb/class/cdc
  sensor/usb/class/cdc/device
  sensor/usb/device
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  # gtest
  catkin_add_gtest(${PROJECT_NAME}_test
    test/matching_filter_test.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(${PROJECT_NAME}_test
    ${catkin_LIBRARIES}
  )
endif()
