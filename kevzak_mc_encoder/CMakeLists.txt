cmake_minimum_required(VERSION 2.8.3)
project(kevzak_mc_encoder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(encoder src/encoder.cpp)
target_link_libraries(encoder ${catkin_LIBRARIES})
