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
set(ARM_LINKER_OPTIONS "--specs=nano.specs -Wl,--start-group -lm  -Wl,--end-group -L${CMAKE_SOURCE_DIR}/src -Tbootloader_samd21x17.ld")

### END TOOLCHAIN SETUP AREA #############################################


project(perimeter_wire_bootloader)

find_package(catkin REQUIRED)
catkin_package()

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
   set(CMAKE_C_FLAGS_DEBUG "-Os -save-temps -g3")
   set(CMAKE_CXX_FLAGS_DEBUG "-Os -save-temps -g3")
endif(CMAKE_BUILD_TYPE MATCHES Debug)


##########################################################################
# compiler options for all build types
##########################################################################
add_definitions("-D__SAMD21E17D__")
add_definitions("-DSAM_BA_USBCDC_ONLY")
add_definitions("-x c")
add_definitions("-Wall")
add_definitions(-Wmissing-prototypes -Wpointer-arith)
add_definitions("-ffunction-sections")
add_definitions(-mlong-calls)
add_definitions("-mthumb")
add_definitions("-std=gnu99")

###########
## Build ##
###########

include_directories(
  # DFP
  samd21/samd21d/include
  cmsis/CMSIS/Core/Include
)

add_arm_executable(bootloader
  src/board_driver_i2c.c
  src/board_driver_serial.c
  src/board_startup.c
  src/sam_ba_monitor.c
  src/board_driver_led.c
  src/board_driver_usb.c
  src/main.c
  src/sam_ba_serial.c
  src/board_driver_pmic.c
  src/board_init.c
  src/sam_ba_cdc.c
  src/sam_ba_usb.c
)

add_custom_target(version ALL
  COMMAND git -C ${CMAKE_SOURCE_DIR} describe --tags > ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/version
  VERBATIM
  USES_TERMINAL)

#############
## Install ##
#############


install(FILES ${bootloader_HEX}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  RENAME bootloader.hex
)

install(FILES ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/version openocd.cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES scripts/program.sh
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
