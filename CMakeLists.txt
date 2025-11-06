cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME rosbridge2cpp)
project(${PROJECT_NAME})

message(STATUS "Ready to build ${PROJECT_NAME}")

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

if(PkgConfig_FOUND)
	pkg_search_module(LIBBSON REQUIRED libbson-1.0)
else()
	message(STATUS "Didn't found PkgConfig, please specify the path yourself")
	set(LIBBSON_DIR "" CACHE PATH "Install directory of libbson. Should contain /include and /lib")
	set(LIBBSON_INCLUDE_DIRS "${LIBBSON_DIR}/include")
	set(LIBBSON_LIBRARY_DIRS "${LIBBSON_DIR}/lib")
	set(LIBBSON_CFLAGS_OTHER "-I${LIBBSON_INCLUDE_DIRS}/libbson-1.0")
	set(LIBBSON_LIBRARIES "bson-static-1.0.lib" CACHE STRING "Lib file. Different between platforms")
endif()

if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Libraries
set(LIBS ${LIBS} ${LIBBSON_LIBRARIES})
set(LIBS ${LIBS} ${CMAKE_THREAD_LIBS_INIT})
set(LIBS ${LIBS} ${Boost_LIBRARIES})

if(WIN32 AND NOT PkgConfig_FOUND)
	set(LIBS ${LIBS} "ws2_32") # Needed by libbson on windows 
endif()

# Example executables (only built when this is the main project)
# These compile sources directly for demonstration purposes
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
	#WebSocket Client
	ADD_EXECUTABLE( websocket_client
		src/client/websocket_client.cpp 
		src/client/socket_websocket_connection.cpp 
		src/ros_bridge.cpp 
		src/ros_topic.cpp 
		src/ros_service.cpp 
		src/ros_tf_broadcaster.cpp 
	)
	target_include_directories(websocket_client PRIVATE include ${LIBBSON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
	target_compile_definitions(websocket_client PRIVATE RAPIDJSON_HAS_STDSTRING=1 BSON_STATIC=1 ${LIBBSON_CFLAGS_OTHER})

	# Alternative TCP client (for ROS1 compatibility)
	ADD_EXECUTABLE( tcp_client
		src/client/tcp_client.cpp 
		src/client/socket_tcp_connection.cpp 
		src/ros_bridge.cpp 
		src/ros_topic.cpp 
		src/ros_service.cpp 
		src/ros_tf_broadcaster.cpp 
	)
	target_include_directories(tcp_client PRIVATE include ${LIBBSON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
	target_compile_definitions(tcp_client PRIVATE RAPIDJSON_HAS_STDSTRING=1 BSON_STATIC=1 ${LIBBSON_CFLAGS_OTHER})

	# Comprehensive demo client showing multiple features
	# Links against the rosbridge2cpp library instead of compiling all sources
	ADD_EXECUTABLE( demo_client
		src/client/demo_client.cpp 
	)
endif()

# Main library target
ADD_LIBRARY( rosbridge2cpp SHARED
	src/ros_bridge.cpp 
	src/ros_topic.cpp 
	src/ros_service.cpp 
	src/ros_tf_broadcaster.cpp 
	src/client/socket_websocket_connection.cpp 
	src/client/socket_tcp_connection.cpp 
)

# Configure the library target with proper include directories and definitions
target_include_directories(rosbridge2cpp
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	PRIVATE
		${LIBBSON_INCLUDE_DIRS}
		${Boost_INCLUDE_DIRS}
)

target_link_directories(rosbridge2cpp PRIVATE ${LIBBSON_LIBRARY_DIRS})

target_compile_definitions(rosbridge2cpp
	PUBLIC
		RAPIDJSON_HAS_STDSTRING=1
	PRIVATE
		BSON_STATIC=1
		${LIBBSON_CFLAGS_OTHER}
)

target_link_libraries(rosbridge2cpp
	PRIVATE
		${LIBBSON_LIBRARIES}
		${CMAKE_THREAD_LIBS_INIT}
		${Boost_LIBRARIES}
)

if(WIN32 AND NOT PkgConfig_FOUND)
	target_link_libraries(rosbridge2cpp PRIVATE "ws2_32")
endif()

# Link example executables (only if they were built)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
	target_link_libraries(websocket_client ${LIBS})
	target_link_libraries(tcp_client ${LIBS})
	target_link_libraries(demo_client rosbridge2cpp ${LIBS})
endif()