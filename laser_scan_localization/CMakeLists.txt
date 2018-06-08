cmake_minimum_required(VERSION 2.8.3)
project(laser_scan_localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LaserLocalization.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_scan_localization
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/laser_scan_localization.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}
	src/LaserScan.cpp
)

add_executable(${PROJECT_NAME}_talker
	src/Talker.cpp
)

add_executable(${PROJECT_NAME}_listener
	src/Listener.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME} ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_talker
   ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_listener
   ${catkin_LIBRARIES}
)
