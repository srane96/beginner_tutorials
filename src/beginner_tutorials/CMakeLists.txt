cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package()
include_directories( include ${catkin_INCLUDE_DIRS})

