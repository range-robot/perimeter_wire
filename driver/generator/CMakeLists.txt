cmake_minimum_required(VERSION 2.8.3)
project(perimeter_wire_generator_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  perimeter_wire_generator_firmware
)

find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################

# From roscpp:
set(PTHREAD_LIB "")
find_package(Threads)
if(CMAKE_THREAD_LIBS_INIT)
  string(LENGTH ${CMAKE_THREAD_LIBS_INIT} _length)
  if(_length GREATER 2)
    string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 0 2 _prefix)
    if(${_prefix} STREQUAL "-l")
      math(EXPR _rest_len "${_length} - 2")
      string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 2 ${_rest_len} PTHREAD_LIB)
    endif()
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PTHREAD_LIB}
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  src
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(console
  src/driver.cpp
  src/console.cpp)
add_dependencies(console
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(console
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PTHREAD_LIB}
)
#############
## Install ##
#############

install(TARGETS console
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


#############
## Testing ##
#############


if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  set(ROSLINT_CPP_OPTS "--filter=-whitespace/braces,-runtime/references,-legal/copyright")
  file(GLOB_RECURSE roslint_cpp_SRC
    "include/*.h"
    "src/*.h"
    "src/*.cpp"
    "test/*.h"
    "test/*.cpp"
  )
  roslint_cpp(${roslint_cpp_SRC})
  roslint_add_test()
endif()
