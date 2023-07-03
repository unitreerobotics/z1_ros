cmake_minimum_required(VERSION 3.1.3)
project(z1_examples)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM
 ${Boost_INCLUDE_DIRS}
 ${z1_sdk_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
 include
)

# examples
set(EXAMPLE_FILES
  examples/ros_gripper_ctrl.cpp
  examples/move_group_interface.cpp
)

foreach(EXAMPLE_FILE IN LISTS EXAMPLE_FILES)
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
  target_link_libraries(${EXAMPLE_NAME} ${catkin_LIBRARIES} )
endforeach()