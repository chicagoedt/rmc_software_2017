cmake_minimum_required(VERSION 2.8.3)
project(ml)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS bin/classifier
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
