cmake_minimum_required(VERSION 3.0.2)
project(z1_rviz)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
 CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
