cmake_minimum_required(VERSION 2.8.3)
project(perimeter_wire_sensor_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  perimeter_wire_sensor_firmware
  diagnostic_updater
)

find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  src
  src-gen
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(perimeter_wire_driver
  src/driver.cpp
  src/node.cpp)
add_dependencies(perimeter_wire_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(perimeter_wire_driver 
  ${catkin_LIBRARIES}
)

add_executable(console
  src/driver.cpp
  src/console.cpp)
add_dependencies(console
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(console
  ${catkin_LIBRARIES}
)
#############
## Install ##
#############

install(TARGETS perimeter_wire_driver console
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
