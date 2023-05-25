cmake_minimum_required(VERSION 3.0.2)
project(z1_ros_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  control_msgs
  actionlib_msgs
  actionlib
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES z1_ros_control
   CATKIN_DEPENDS controller_manager hardware_interface roscpp control_msgs actionlib_msgs
#  DEPENDS system_lib 
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(lib)

add_executable(z1_ros_control_node src/z1_ros_control.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} Z1_SDK_x86_64)
