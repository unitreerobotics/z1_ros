cmake_minimum_required(VERSION 3.0)
project(z1_controller LANGUAGES CXX)
add_definitions(-std=c++14)
set(PACKAGE_NAME z1_controller)
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3")

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS 
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

link_directories(lib)

add_executable(z1_control_real src/real_control.cpp)
target_link_libraries(z1_control_real ${catkin_LIBRARIES} Z1_UDP_Linux64)

add_executable(z1_control_sim src/sim_control.cpp)
target_link_libraries(z1_control_sim ${catkin_LIBRARIES} Z1_ROS_Linux64)