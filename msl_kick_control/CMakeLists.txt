cmake_minimum_required(VERSION 2.8.3)
project(msl_kick_control)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED can_connection system_config roscpp usb_can_proxy msl_actuator_msgs msl_sensor_msgs msl_robot_id)

find_package(Boost REQUIRED COMPONENTS system thread)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES msl_kick_control
  CATKIN_DEPENDS can_connection system_config roscpp usb_can_proxy msl_actuator_msgs msl_sensor_msgs msl_robot_id
  DEPENDS ${Boost_LIBRARIES}
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(msl_kick_control src/Kicker.cpp)
target_link_libraries(msl_kick_control ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(msl_kick_control msl_actuator_msgs_generate_messages_cpp)

