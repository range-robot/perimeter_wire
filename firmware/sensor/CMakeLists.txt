cmake_minimum_required(VERSION 2.8.3)

##########################################################################
# tools to beused for programming the ARM
##########################################################################
set(ARM_UPLOADTOOL openocd)
# jtag ice mkII settings
set(ARM_PROGRAMMER jtag2isp)
set(ARM_UPLOADTOOL_PORT usb)

##########################################################################
# ARM and fuses needs to be set
##########################################################################
set(ARM_MCU ATSAMD21E17D)
set(ARM_CPU cortex-m0plus)
set(ARM_LINKER_OPTIONS "--specs=nano.specs -Wl,--start-group -lm  -Wl,--end-group -L${CMAKE_SOURCE_DIR}/Device_Startup -Tsamd21e17d_flash_bl.ld")

### END TOOLCHAIN SETUP AREA #############################################


project(perimeter_wire_sensor_firmware)

find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS include
)

include(cmake/generic-gcc-arm-none-eabi.cmake)


##########################################################################
# status messages for generating
##########################################################################
message(STATUS "Set CMAKE_FIND_ROOT_PATH to ${CMAKE_FIND_ROOT_PATH}")
message(STATUS "Set CMAKE_SYSTEM_INCLUDE_PATH to ${CMAKE_SYSTEM_INCLUDE_PATH}")
message(STATUS "Set CMAKE_SYSTEM_LIBRARY_PATH to ${CMAKE_SYSTEM_LIBRARY_PATH}")
message(STATUS "Set CMAKE_C_COMPILER to ${CMAKE_C_COMPILER}")


##########################################################################
# set compiler options for build types
##########################################################################
if(CMAKE_BUILD_TYPE MATCHES Release)
   set(CMAKE_C_FLAGS_RELEASE "-Os")
   set(CMAKE_CXX_FLAGS_RELEASE "-Os")
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
   set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Os -save-temps -g3")
   set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-Os -save-temps -g3")
endif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)

if(CMAKE_BUILD_TYPE MATCHES Debug)
   set(CMAKE_C_FLAGS_DEBUG "-O0 -save-temps -g3")
   set(CMAKE_CXX_FLAGS_DEBUG "-O0 -save-temps -g3")
endif(CMAKE_BUILD_TYPE MATCHES Debug)


##########################################################################
# compiler options for all build types
##########################################################################
add_definitions("-D__SAMD21E17D__")
add_definitions("-x c")

add_definitions("-Wall")
#add_definitions("-Werror")
add_definitions(-Wmissing-prototypes -Wpointer-arith)
#add_definitions(-Werror-implicit-function-declaration)
#add_definitions(-Wstrict-prototypes)
#add_definitions("-pedantic")
#add_definitions("-pedantic-errors")

#add_definitions("-fpack-struct")
#add_definitions("-fshort-enums")
#add_definitions("-funsigned-char")
#add_definitions("-funsigned-bitfields")
#add_definitions(-fdata-sections -fdata-sections -g3 -fno-strict-aliasing)
#add_definitions(-save-temps)
#add_definitions(-mrelax -save-temps)

add_definitions("-ffunction-sections")
add_definitions(-mlong-calls)
add_definitions("-mthumb")
add_definitions("-std=gnu99")

#-x c -mthumb -D__SAMD11C14A__  -Os -ffunction-sections -mlong-calls -g3 -Wall -mcpu=cortex-m0plus -c

###########
## Build ##
###########

include_directories(
  # DFP
  samd21/samd21d/include
  cmsis/CMSIS/Core/Include

  # Project
  include
  .
  Config
  hal/include
  hal/utils/include
  hpl/core
  hpl/gclk
  hpl/port
  hpl/pm
  hpl/rtc
  hri
  usb
  usb/class/cdc
  usb/class/cdc/device
  usb/device
)

add_arm_executable(firmware
./atmel_start.c
./com/app_layer.c
./com/usb_serial.c
./Device_Startup/startup_samd21.c
./Device_Startup/system_samd21.c
./driver_init.c
./error.c
./examples/driver_examples.c
./hal/src/hal_adc_dma.c
./hal/src/hal_atomic.c
./hal/src/hal_delay.c
./hal/src/hal_gpio.c
./hal/src/hal_init.c
./hal/src/hal_io.c
./hal/src/hal_sleep.c
./hal/src/hal_timer.c
./hal/src/hal_usb_device.c
./hal/utils/src/utils_assert.c
./hal/utils/src/utils_event.c
./hal/utils/src/utils_list.c
./hal/utils/src/utils_syscalls.c
./hpl/adc/hpl_adc.c
./hpl/core/hpl_core_m0plus_base.c
./hpl/core/hpl_init.c
./hpl/dmac/hpl_dmac.c
./hpl/gclk/hpl_gclk.c
./hpl/pm/hpl_pm.c
./hpl/rtc/hpl_rtc.c
./hpl/sysctrl/hpl_sysctrl.c
./hpl/usb/hpl_usb.c
./main.c
./perimeter_wire_sensor.c
./uplink.c
./usb/class/cdc/device/cdcdf_acm.c
./usb/device/usbdc.c
./usb/usb_protocol.c
./usb_start.c
)

add_custom_target(
  upload_with_bootloader
  bossac --offset 0x2000 -i -U -e -w ${firmware_BIN} -v -R
  DEPENDS ${firmware_BIN}
  COMMENT "Uploading ${firmware_BIN}"
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
