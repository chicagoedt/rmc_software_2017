cmake_minimum_required(VERSION 2.8.3)
project(robot_setup_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(

 CATKIN_DEPENDS geometry_msgs roscpp tf

)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_tf_publisher src/broadcaster/tf_broadcaster.cpp)

target_link_libraries(robot_tf_publisher ${catkin_LIBRARIES})
