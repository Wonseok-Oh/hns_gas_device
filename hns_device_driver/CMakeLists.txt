cmake_minimum_required(VERSION 2.8.3)
project(hns_device_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hns_msgs
  roscpp
  serial
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hns_device_driver
#  CATKIN_DEPENDS hns_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/HNSDeviceDriver.cpp
)

add_library(hns_analog_test
   src/HNSAnalogTest.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)



## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(hns_analog_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hns_analog_test
   ${catkin_LIBRARIES}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/hns_device_driver_node.cpp)

add_executable(hns_analog_test_node src/hns_analog_test_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(hns_analog_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node hns_device_driver
   ${catkin_LIBRARIES}
)

target_link_libraries(hns_analog_test_node hns_analog_test
   ${catkin_LIBRARIES}
)

