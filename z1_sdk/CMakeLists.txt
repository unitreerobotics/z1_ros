cmake_minimum_required(VERSION 3.0)
project(z1_sdk
  VERSION 3.0.0
  LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -std=c++14 -DNDEBUG -fPIC -pthread")

## Dependencies
find_package(Boost REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(unitree_arm_sdk SHARED src/UdpPort.cpp)

find_package(catkin QUIET)
if(${catkin_FOUND})
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_SOURCE_DIR}/lib/libunitree_arm_sdk.so
  )
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# examples
set(EXAMPLE_FILES
  examples/example_joint_ctrl.cpp
  examples/example_lowcmd.cpp
)

foreach(EXAMPLE_FILE IN LISTS EXAMPLE_FILES)
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
  target_link_libraries(${EXAMPLE_NAME} ${catkin_LIBRARIES} unitree_arm_sdk)
endforeach()

find_package(pybind11 QUIET)
if(${pybind11_FOUND})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
  include_directories(${pybind11_INCLUDE_DIRS})
  pybind11_add_module(unitree_arm_interface src/arm_python_interface.cpp)
  target_link_libraries(unitree_arm_interface PRIVATE unitree_arm_sdk)
endif()