# Thanks to https://github.com/dmonopoly/gtest-cmake-example/blob/master/CMakeLists.txt
# for providing a basic setup to use gtest in a cmake project
# Another solution would make use of ExternalProject_Add to downloads and installs
# googletest in the build/ dir:
# http://stackoverflow.com/questions/9689183/cmake-googletest/9695234#9695234
# However, this requires an active internet connection


cmake_minimum_required(VERSION 2.8)
option(test "Build all tests." OFF) # Makes boolean 'test' available.

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
	pkg_search_module(LIBBSON REQUIRED libbson-1.0)
else()
	message(STATUS "Didn't fount PkgConfig, please specify the path yourself")
	set(LIBBSON_DIR "" CACHE PATH "Install directory of libbson. Should contain /include and /lib")
	set(LIBBSON_INCLUDE_DIRS "${LIBBSON_DIR}/include")
	set(LIBBSON_LIBRARY_DIRS "${LIBBSON_DIR}/lib")
	set(LIBBSON_CFLAGS_OTHER "-I${LIBBSON_INCLUDE_DIRS}/libbson-1.0")
	set(LIBBSON_LIBRARIES "bson-static-1.0.lib" CACHE STRING "Lib file. Different between platforms")
endif()

set(PROJECT_NAME rosbridge2cpp)
project(${PROJECT_NAME})

if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "-g -Wall")
endif()

set(CXX_STANDARD C++11)
add_definitions(-DRAPIDJSON_HAS_STDSTRING=1)
add_definitions(-DBSON_STATIC=1)

include_directories(${LIBBSON_INCLUDE_DIRS})
link_directories(${LIBBSON_LIBRARY_DIRS})
add_definitions(${LIBBSON_CFLAGS_OTHER})
set(LIBS ${LIBS} ${LIBBSON_LIBRARIES})

if(WIN32 AND NOT PkgConfig_FOUND)
	set(LIBS ${LIBS} "ws2_32") # Needed by libbson on windows 
endif()

INCLUDE_DIRECTORIES( include)
ADD_EXECUTABLE( rosbridge2cpp-client
	src/client/client.cpp 
	src/client/socket_tcp_connection.cpp 
	src/ros_bridge.cpp 
	src/ros_topic.cpp 
	src/ros_service.cpp 
	src/ros_tf_broadcaster.cpp 
)

ADD_LIBRARY( rosbridge2cpp SHARED
#	src/client/socket_tcp_connection.cpp #Not needed for the lib
	src/ros_bridge.cpp 
	src/ros_topic.cpp 
	src/ros_service.cpp 
	src/ros_tf_broadcaster.cpp 
)
target_link_libraries(rosbridge2cpp-client ${LIBS})
target_link_libraries(rosbridge2cpp ${LIBS})
#
#################################
## Testing
#################################
#if (test)
#	# This adds another subdirectory, which has 'project(gtest)'.
#	add_subdirectory(lib/googletest/googletest)
#
#	enable_testing()
#
#	# Include the gtest library. gtest_SOURCE_DIR is available due to
#	# 'project(gtest)' above.
#	include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
#
#	##############
#	# Unit Tests
#	##############
#	add_executable(runUnitTests tests/tests.cpp)
#
#	# Standard linking to gtest stuff.
#	target_link_libraries(runUnitTests gtest gtest_main rosbridge2cpp)
#
#	# Extra linking for the project.
#	#target_link_libraries(runUnitTests project1_lib)
#
#	# This is so you can do 'make test' to see all your tests run, instead of
#	# manually running the executable runUnitTests to see those specific tests.
#  #add_test(NAME that-test-I-made COMMAND runUnitTests)
#
#	# You can also omit NAME and COMMAND. The second argument could be some other
#	# test executable.
#	add_test(that-other-test-I-made runUnitTests)
#endif()
