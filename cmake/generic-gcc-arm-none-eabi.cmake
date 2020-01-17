##########################################################################
# The toolchain requires some variables set.
#
# ARM_MCU
#     the type of ARM the application is built for
# ARM_CPU
#     cpu core of the mcu
# ARM_L_FUSE (NO DEFAULT)
#     the LOW fuse value for the MCU used
# ARM_H_FUSE (NO DEFAULT)
#     the HIGH fuse value for the MCU used
# ARM_UPLOADTOOL (default: openocd)
#     the application used to upload to the MCU
#     NOTE: The toolchain is currently quite specific about
#           the commands used, so it needs tweaking.
# ARM_UPLOADTOOL_PORT (default: usb)
#     the port used for the upload tool, e.g. usb
# ARM_PROGRAMMER (default: avrispmkII)
#     the programmer hardware used, e.g. avrispmkII
##########################################################################

##########################################################################
# options
##########################################################################
option(WITH_MCU "Add the mCU type to the target file name." ON)

##########################################################################
# executables in use
##########################################################################
find_program(ARM_CC arm-none-eabi-gcc)
find_program(ARM_CXX  arm-none-eabi-g++)
find_program(ARM_OBJCOPY  arm-none-eabi-objcopy)
find_program(ARM_SIZE_TOOL arm-none-eabi-size)
find_program(ARM_OBJDUMP  arm-none-eabi-objdump)

##########################################################################
# toolchain starts with defining mandatory variables
##########################################################################
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER ${ARM_CC})
set(CMAKE_CXX_COMPILER ${ARM_CXX})
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS )

##########################################################################
# Identification
##########################################################################
set(ARM 1)

##########################################################################
# some necessary tools and variables for ARM builds, which may not
# defined yet
# - ARM_UPLOADTOOL
# - ARM_MCU
# - ARM_CPU
# - ARM_SIZE_ARGS
##########################################################################

# default upload tool
if(NOT ARM_UPLOADTOOL)
   set(
      ARM_UPLOADTOOL openocd
      CACHE STRING "Set default upload tool: openocd"
   )
   find_program(ARM_UPLOADTOOL openocd)
endif(NOT ARM_UPLOADTOOL)

# default MCU (chip)
if(NOT ARM_MCU)
   message(FATAL_ERROR "Please set ARM_MCU.")
endif(NOT ARM_MCU)

# default MCU (chip)
if(NOT ARM_CPU)
   message(FATAL_ERROR "Please set ARM_CPU.")
endif(NOT ARM_CPU)

#default arm-size args
if(NOT ARM_SIZE_ARGS)
   if(APPLE)
      set(ARM_SIZE_ARGS)
   else(APPLE)
      set(ARM_SIZE_ARGS)
   endif(APPLE)
endif(NOT ARM_SIZE_ARGS)

##########################################################################
# check build types:
# - Debug
# - Release
# - RelWithDebInfo
#
# Release is chosen, because of some optimized functions in the
# ARM toolchain, e.g. _delay_ms().
##########################################################################
if(NOT ((CMAKE_BUILD_TYPE MATCHES Release) OR
        (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo) OR
        (CMAKE_BUILD_TYPE MATCHES Debug) OR
        (CMAKE_BUILD_TYPE MATCHES MinSizeRel)))
   set(
      CMAKE_BUILD_TYPE Release
      CACHE STRING "Choose cmake build type: Debug Release RelWithDebInfo MinSizeRel"
      FORCE
   )
endif(NOT ((CMAKE_BUILD_TYPE MATCHES Release) OR
           (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo) OR
           (CMAKE_BUILD_TYPE MATCHES Debug) OR
           (CMAKE_BUILD_TYPE MATCHES MinSizeRel)))

##########################################################################

##########################################################################
# some cmake cross-compile necessities
##########################################################################
if(DEFINED ENV{ARM_FIND_ROOT_PATH})
    set(CMAKE_FIND_ROOT_PATH $ENV{ARM_FIND_ROOT_PATH})
else(DEFINED ENV{ARM_FIND_ROOT_PATH})
    if(EXISTS "/opt/local/arm-none-eabi")
      set(CMAKE_FIND_ROOT_PATH "/opt/local/arm-none-eabi")
    elseif(EXISTS "/usr/arm-none-eabi")
      set(CMAKE_FIND_ROOT_PATH "/usr/arm-none-eabi")
    elseif(EXISTS "/usr/lib/arm-none-eabi")
      set(CMAKE_FIND_ROOT_PATH "/usr/lib/arm-none-eabi")
    else(EXISTS "/opt/local/arm-none-eabi")
      message(FATAL_ERROR "Please set ARM_FIND_ROOT_PATH in your environment.")
    endif(EXISTS "/opt/local/arm-none-eabi")
endif(DEFINED ENV{ARM_FIND_ROOT_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# not added automatically, since CMAKE_SYSTEM_NAME is "generic"
set(CMAKE_SYSTEM_INCLUDE_PATH "${CMAKE_FIND_ROOT_PATH}/include")
set(CMAKE_SYSTEM_LIBRARY_PATH "${CMAKE_FIND_ROOT_PATH}/lib")

##########################################################################
# target file name add-on
##########################################################################
if(WITH_MCU)
   set(MCU_TYPE_FOR_FILENAME "-${ARM_MCU}")
else(WITH_MCU)
   set(MCU_TYPE_FOR_FILENAME "")
endif(WITH_MCU)

##########################################################################
# add_arm_executable
# - IN_VAR: EXECUTABLE_NAME
#
# Creates targets and dependencies for ARM toolchain, building an
# executable. Calls add_executable with ELF file as target name, so
# any link dependencies need to be using that target, e.g. for
# target_link_libraries(<EXECUTABLE_NAME>-${ARM_MCU}.elf ...).
##########################################################################
function(add_arm_executable EXECUTABLE_NAME)
   if(NOT ARGN)
      message(FATAL_ERROR "No source files given for ${EXECUTABLE_NAME}.")
   endif(NOT ARGN)

   # set file names
   set(elf_file ${EXECUTABLE_NAME}${MCU_TYPE_FOR_FILENAME}.elf)
   set(elf_file_path ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${elf_file})
   set(${EXECUTABLE_NAME}_ELF ${elf_file_path} PARENT_SCOPE)
   set(hex_file ${EXECUTABLE_NAME}${MCU_TYPE_FOR_FILENAME}.hex)
   set(hex_file_path ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${hex_file})
   set(${EXECUTABLE_NAME}_HEX ${hex_file_path} PARENT_SCOPE)
   set(map_file ${EXECUTABLE_NAME}${MCU_TYPE_FOR_FILENAME}.map)
   set(map_file_path ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${map_file})
   set(${EXECUTABLE_NAME}_MAP ${map_file_path} PARENT_SCOPE)
   set(eeprom_image ${EXECUTABLE_NAME}${MCU_TYPE_FOR_FILENAME}-eeprom.hex)
   set(eeprom_image_path ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${eeprom_image})
   set(${EXECUTABLE_NAME}_EEPROM ${eeprom_image_path} PARENT_SCOPE)

   # elf file
   add_executable(${elf_file} EXCLUDE_FROM_ALL ${ARGN})

   set_target_properties(
      ${elf_file}
      PROPERTIES
         COMPILE_FLAGS "-mcpu=${ARM_CPU} -mthumb"
	      LINK_FLAGS "-mcpu=${ARM_CPU} -mthumb -Wl,--gc-sections -Wl,-Map,${map_file_path} ${ARM_LINKER_OPTIONS}"
   )
   add_custom_command(
      OUTPUT ${hex_file_path}
      COMMAND
         ${ARM_OBJCOPY} -O ihex -R .eeprom -R .fuse -R .lock -R .signature ${elf_file_path} ${hex_file_path}
      COMMAND
         ${ARM_SIZE_TOOL} ${ARM_SIZE_ARGS} ${elf_file_path}
      DEPENDS ${elf_file_path}
   )

   # eeprom
   add_custom_command(
      OUTPUT ${eeprom_image_path}
      COMMAND
         ${ARM_OBJCOPY} -j .eeprom --set-section-flags=.eeprom=alloc,load
            --change-section-lma .eeprom=0 --no-change-warnings
            -O ihex ${elf_file_path} ${eeprom_image_path}
      DEPENDS ${elf_file_path}
   )

   add_custom_target(
      ${EXECUTABLE_NAME}
      ALL
      DEPENDS ${hex_file_path} ${eeprom_image_path}
   )

   set_target_properties(
      ${EXECUTABLE_NAME}
      PROPERTIES
         OUTPUT_NAME "${elf_file_path}"
   )

   # clean
   get_directory_property(clean_files ADDITIONAL_MAKE_CLEAN_FILES)
   set_directory_properties(
      PROPERTIES
         ADDITIONAL_MAKE_CLEAN_FILES "${map_file_path}"
   )

   # upload
   add_custom_target(
      upload_${EXECUTABLE_NAME}
      ${ARM_UPLOADTOOL}  -f ${CMAKE_SOURCE_DIR}/openocd.cfg -c "program ${hex_file_path} verify reset exit"
      DEPENDS ${hex_file_path}
      COMMENT "Uploading ${hex_file}"
   )

endfunction(add_arm_executable)

##########################################################################
# add_arm_library
# - IN_VAR: LIBRARY_NAME
#
# Calls add_library with an optionally concatenated name
# <LIBRARY_NAME>${MCU_TYPE_FOR_FILENAME}.
# This needs to be used for linking against the library, e.g. calling
# target_link_libraries(...).
##########################################################################
function(add_arm_library LIBRARY_NAME)
   if(NOT ARGN)
      message(FATAL_ERROR "No source files given for ${LIBRARY_NAME}.")
   endif(NOT ARGN)

   set(lib_file ${LIBRARY_NAME}${MCU_TYPE_FOR_FILENAME})

   add_library(${lib_file} STATIC ${ARGN})

   set_target_properties(
      ${lib_file}
      PROPERTIES
         COMPILE_FLAGS "-mmcu=${ARM_MCU}"
         OUTPUT_NAME "${lib_file}"
   )

   if(NOT TARGET ${LIBRARY_NAME})
      add_custom_target(
         ${LIBRARY_NAME}
         ALL
         DEPENDS ${lib_file}
      )

      set_target_properties(
         ${LIBRARY_NAME}
         PROPERTIES
            OUTPUT_NAME "${lib_file}"
      )
   endif(NOT TARGET ${LIBRARY_NAME})

endfunction(add_arm_library)

##########################################################################
# arm_target_link_libraries
# - IN_VAR: EXECUTABLE_TARGET
# - ARGN  : targets and files to link to
#
# Calls target_link_libraries with ARM target names (concatenation,
# extensions and so on.
##########################################################################
function(arm_target_link_libraries EXECUTABLE_TARGET)
   if(NOT ARGN)
      message(FATAL_ERROR "Nothing to link to ${EXECUTABLE_TARGET}.")
   endif(NOT ARGN)

   get_target_property(TARGET_LIST ${EXECUTABLE_TARGET} OUTPUT_NAME)

   foreach(TGT ${ARGN})
      if(TARGET ${TGT})
         get_target_property(ARG_NAME ${TGT} OUTPUT_NAME)
         list(APPEND TARGET_LIST ${ARG_NAME})
      else(TARGET ${TGT})
         list(APPEND NON_TARGET_LIST ${TGT})
      endif(TARGET ${TGT})
   endforeach(TGT ${ARGN})

   target_link_libraries(${TARGET_LIST} ${NON_TARGET_LIST})
endfunction(arm_target_link_libraries EXECUTABLE_TARGET)

##########################################################################
# arm_target_include_directories
#
# Calls target_include_directories with ARM target names
##########################################################################

function(arm_target_include_directories EXECUTABLE_TARGET)
   if(NOT ARGN)
      message(FATAL_ERROR "No include directories to add to ${EXECUTABLE_TARGET}.")
   endif()

   get_target_property(TARGET_LIST ${EXECUTABLE_TARGET} OUTPUT_NAME)
   set(extra_args ${ARGN})

   target_include_directories(${TARGET_LIST} ${extra_args})
endfunction()

##########################################################################
# arm_target_compile_definitions
#
# Calls target_compile_definitions with ARM target names
##########################################################################

function(arm_target_compile_definitions EXECUTABLE_TARGET)
   if(NOT ARGN)
      message(FATAL_ERROR "No compile definitions to add to ${EXECUTABLE_TARGET}.")
   endif()

   get_target_property(TARGET_LIST ${EXECUTABLE_TARGET} OUTPUT_NAME)
   set(extra_args ${ARGN})

   target_compile_definitions(${TARGET_LIST} ${extra_args})
endfunction()
