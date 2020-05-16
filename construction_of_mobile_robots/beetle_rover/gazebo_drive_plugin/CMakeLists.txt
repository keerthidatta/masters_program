cmake_minimum_required(VERSION 2.8.3)
project(gazebo_drive_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
#add_executable(gazebo_drive_plugin src/gazebo_drive_plugin.cpp)
# For set_speed_joint_plugin plugin

add_library(gazebo_drive_plugin src/gazebo_drive_plugin.cpp)
target_link_libraries(gazebo_drive_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

