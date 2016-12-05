cmake_minimum_required(VERSION 2.8.3)
project(state_machine)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(rmc_simulation)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  roscpp
  std_msgs
  tf
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


add_message_files(
  FILES
  SensorStatus.msg
)

add_service_files(
  FILES
  ValidateSensors.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES state_machine
  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(state_machine
#   src/${PROJECT_NAME}/state_machine.cpp
# )

## Declare a cpp executable
add_executable(state_machine_node src/stateMachine/state_machine.cpp src/stateMachine/main.cpp)
add_executable(mission_validator src/missionValidator/mission_validator.cpp src/missionValidator/main.cpp)
add_executable(pose_publisher_node src/posePublisher/posePublisher.cpp ../rmcDecode/rmcEnDecoder.cpp ../rmcDecode/rmcData.cpp src/posePublisher/serialPort.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(state_machine_node state_machine_generate_messages_cpp)
add_dependencies(mission_validator state_machine_generate_messages_cpp)
add_dependencies(state_machine_node rmc_simulation_generate_messages_cpp)
add_dependencies(pose_publisher_node rmc_simulation_generate_messages_cpp)
add_dependencies(mission_validator rmc_simulation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(state_machine_node ${catkin_LIBRARIES} )
target_link_libraries(mission_validator ${catkin_LIBRARIES} )
target_link_libraries(pose_publisher_node ${catkin_LIBRARIES} )

set(CMAKE_BUILD_TYPE Debug)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS state_machine state_machine_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_state_machine.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
