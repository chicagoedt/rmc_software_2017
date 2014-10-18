cmake_minimum_required(VERSION 2.8.3)
project(base_controller)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)


add_message_files(
        FILES
  	wheels_msg.msg
	Xbox_Button_Msg.msg
	diagnostics_msg.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
       	CATKIN_DEPENDS
	message_runtime
)



include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(keyboard_teleop src/keyboard_teleop.cpp)
#target_link_libraries(keyboard_teleop ${catkin_LIBRARIES})

add_executable(roboteq_driver src/roboteq_driver.cpp src/serialDriver/Exception.cpp src/serialDriver/SerialPort.cpp)
target_link_libraries(roboteq_driver ${catkin_LIBRARIES})



add_executable(joystick_teleop src/joystick_teleop.cpp)
target_link_libraries(joystick_teleop ${catkin_LIBRARIES})

#add_dependencies(keyboard_teleop base_controller_gencpp)
