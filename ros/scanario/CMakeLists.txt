cmake_minimum_required(VERSION 3.0.2)
project(scanario)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  object_msgs
  roslib
)

catkin_package(
  CATKIN_DEPENDS std_msgs visualization_msgs object_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## target_objects
add_executable(target_objects src/target_objects/main.cpp src/target_objects/node.cpp)
target_link_libraries(target_objects ${catkin_LIBRARIES})
