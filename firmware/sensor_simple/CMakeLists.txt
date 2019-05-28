cmake_minimum_required(VERSION 2.8.3)

### TOOLCHAIN SETUP AREA #################################################
# Set any variables used in the toolchain prior project() call. In that
# case they are already set and used.
##########################################################################

##########################################################################
# tools to beused for programming the AVR
##########################################################################
set(AVR_UPLOADTOOL avrdude)
# jtag ice mkII settings
set(AVR_PROGRAMMER jtag2isp)
set(AVR_UPLOADTOOL_PORT usb)
# bootloader settings
#set(AVR_PROGRAMMER avr109)
#set(AVR_UPLOADTOOL_PORT /dev/ttyUSB0 -b 9600)

##########################################################################
# AVR and fuses needs to be set
##########################################################################
set(AVR_MCU atmega328p)
#set(MCU_SPEED "16000000UL")

#set(AVR_H_FUSE 0xd8)
#set(AVR_L_FUSE 0xff)

### END TOOLCHAIN SETUP AREA #############################################


project(perimeter_wire_firmware)

find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS include
)

include(cmake/generic-gcc-avr.cmake)

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
   set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Os -save-temps -g -gdwarf-3 -gstrict-dwarf")
   set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-Os -save-temps -g -gdwarf-3 -gstrict-dwarf")
endif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)

if(CMAKE_BUILD_TYPE MATCHES Debug)
   set(CMAKE_C_FLAGS_DEBUG "-O0 -save-temps -g -gdwarf-3 -gstrict-dwarf")
   set(CMAKE_CXX_FLAGS_DEBUG "-O0 -save-temps -g -gdwarf-3 -gstrict-dwarf")
endif(CMAKE_BUILD_TYPE MATCHES Debug)


##########################################################################
# compiler options for all build types
##########################################################################
#add_definitions("-DF_CPU=${MCU_SPEED}")

add_definitions("-Wall")
#add_definitions("-Werror")
add_definitions(-Wmissing-prototypes -Wpointer-arith)
#add_definitions(-Werror-implicit-function-declaration)
#add_definitions(-Wstrict-prototypes)
#add_definitions("-pedantic")
#add_definitions("-pedantic-errors")

add_definitions("-fpack-struct")
add_definitions("-fshort-enums")
add_definitions("-funsigned-char")
add_definitions("-funsigned-bitfields")
add_definitions("-ffunction-sections")
add_definitions(-fdata-sections -fdata-sections -g3 -fno-strict-aliasing)
add_definitions("-c")
add_definitions("-std=gnu99")
add_definitions(-mrelax -save-temps)

###########
## Build ##
###########

include_directories(
  include
  src
)

add_avr_executable(firmware
  src/main.c
  src/error.c
  src/com/app_layer.c
  src/com/async_serial.c
)

#find_library(C_LIB c)
#avr_target_link_libraries(firmware
#  ${C_LIB})

#############
## Install ##
#############

install(FILES ${firmware_ELF}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  RENAME firmware_em.elf
)

install(FILES program.sh
  PERMISSIONS OWNER_EXECUTE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
