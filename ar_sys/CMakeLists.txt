cmake_minimum_required(VERSION 2.8.3)
project(ar_sys)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	dynamic_reconfigure
	geometry_msgs
	image_transport
	roscpp
	tf
	visualization_msgs
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
	LIBRARIES aruco
	CATKIN_DEPENDS cv_bridge dynamic_reconfigure geometry_msgs image_transport visualization_msgs roscpp tf
#	DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(aruco
	src/aruco/arucofidmarkers.cpp
	src/aruco/cvdrawingutils.cpp
	src/aruco/cameraparameters.cpp
	src/aruco/board.cpp
	src/aruco/marker.cpp
	src/aruco/boarddetector.cpp
	src/aruco/markerdetector.cpp
	src/aruco/ar_omp.cpp
	src/aruco/subpixelcorner.cpp
)
target_link_libraries(aruco ${catkin_LIBRARIES})

add_executable(single_board src/single_board.cpp
	src/utils.cpp)
add_dependencies(single_board ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_board aruco ${catkin_LIBRARIES})

add_executable(multi_boards src/multi_boards.cpp
	src/utils.cpp)
add_dependencies(multi_boards ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_boards aruco ${catkin_LIBRARIES})

add_executable(system_viewer src/system_viewer.cpp)
add_dependencies(system_viewer ${catkin_EXPORTED_TARGETS})
target_link_libraries(system_viewer ${catkin_LIBRARIES})

install(DIRECTORY include/
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY camera_info/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/camera_info
	FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY data/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	FILES_MATCHING PATTERN "*.launch"
)
install(TARGETS aruco single_board multi_boards system_viewer
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

