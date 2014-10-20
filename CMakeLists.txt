set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(s8_ir_sensors)

find_package(catkin REQUIRED COMPONENTS roscpp ras_arduino_msgs s8_msgs s8_common_node)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp ras_arduino_msgs s8_msgs s8_common_node
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(s8_ir_sensors_node src/ir_sensors_node.cpp)
target_link_libraries(s8_ir_sensors_node ${catkin_LIBRARIES})
