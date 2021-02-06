cmake_minimum_required(VERSION 2.8.3)
project(mr_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS 
roscpp
rospy
std_msgs
sensor_msgs
tf
rviz
mr_description
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mr_simulation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
include
${catkin_INCLUDE_DIRS}
./src/
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/new_modular_robots_simulation.cpp
  src/generate_environment.cpp
)
target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
)

add_executable(static_world_A_broadcaster src/static_world_A_broadcaster.cpp)
target_link_libraries(static_world_A_broadcaster
   ${catkin_LIBRARIES})

add_executable(climbot6d_control_robot_by_conf_file src/climbot6d_control_robot_by_conf_file.cpp)
target_link_libraries(climbot6d_control_robot_by_conf_file
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )
add_executable(generate_environment_by_yaml src/generate_environment_by_yaml.cpp)
target_link_libraries(generate_environment_by_yaml
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )
