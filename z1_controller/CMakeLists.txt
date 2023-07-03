cmake_minimum_required(VERSION 3.0)
project(z1_controller
  VERSION 3.0.0
  LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -std=c++14 -DNDEBUG -fPIC")

## Dependencies
find_package(Boost REQUIRED)
find_package(pinocchio REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  std_msgs
  unitree_legged_msgs
  roslib
)

catkin_package(
  CATKIN_DEPENDS unitree_legged_msgs roslib
)  

include_directories(
  ${pinocchio_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
  sim
)

link_directories(lib)
add_executable(z1_ctrl main.cpp sim/IOROS.cpp)
target_link_libraries(z1_ctrl
  ${catkin_LIBRARIES}
  UnitreeArm_${CMAKE_SYSTEM_PROCESSOR}
  pinocchio::pinocchio
)