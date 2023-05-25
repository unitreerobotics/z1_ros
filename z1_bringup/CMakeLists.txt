cmake_minimum_required(VERSION 3.0.2)
project(z1_bringup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES unitree_z1_bringup
#  CATKIN_DEPENDS unitree_gazebo z1_controller z1_moveit_config z1_ros_control z1_viz
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
