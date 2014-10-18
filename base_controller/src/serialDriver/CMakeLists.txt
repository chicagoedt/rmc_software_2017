project(log)
FIND_PACKAGE ( Threads REQUIRED )

set(CMAKE_CXX_FLAGS "-Wall -Wextra -O3")
add_executable(log OutputConsole.cpp Exception.cpp RemoteConsole.cpp SerialPort.cpp Time.cpp DataLogger.cpp Mutex.cpp Thread.cpp main.cpp)
TARGET_LINK_LIBRARIES ( log ${CMAKE_THREAD_LIBS_INIT} )
install(TARGETS log RUNTIME DESTINATION bin)
