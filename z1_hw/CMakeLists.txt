cmake_minimum_required(VERSION 3.0.2)
project(z1_hw)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  control_msgs
  actionlib_msgs
  actionlib
  z1_sdk
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    controller_manager 
    hardware_interface 
    roscpp 
    control_msgs 
    actionlib_msgs
    z1_sdk
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${z1_sdk_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_node src/z1_hw.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${z1_sdk_LIBRARIES})