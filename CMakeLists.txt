cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tf
)
add_service_files(
  FILES
  strManipulator.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package()
include_directories( include ${catkin_INCLUDE_DIRS})
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_gencpp)
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_gencpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
  talkerTest
  test/talkerTest.launch
  test/talkerTest.cpp
  test/main.cpp
  )

  target_link_libraries(
  talkerTest
  ${catkin_LIBRARIES}
  )

  add_dependencies(
  talkerTest
  talker
  listener
  ${catkin_EXPORTED_TARGETS}
  )
endif()
