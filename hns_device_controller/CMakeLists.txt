cmake_minimum_required(VERSION 2.8.3)
project(hns_device_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  hns_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hns_device_controller
#  CATKIN_DEPENDS roscpp
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

add_library(${PROJECT_NAME}
   src/HNSDeviceController.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROEJCT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(hns_device_controller_node src/hns_device_controller_node.cpp)

add_dependencies(hns_device_controller_node ${${PROEJCT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node hns_device_controller
   ${catkin_LIBRARIES}
)
