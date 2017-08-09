cmake_minimum_required(VERSION 2.8.3)
project(a_start)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  geometry_msgs
  rospy
)


catkin_package(
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(a_star
  src/main.cpp

)

target_link_libraries(a_star ${catkin_LIBRARIES})
add_dependencies(a_star ${catkin_EXPORTED_TARGETS})
add_definitions(-std=c++11)

