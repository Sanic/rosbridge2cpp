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
	set(CMAKE_CXX_FLAGS "-g -Wall")
endif()

set(CXX_STANDARD C++11)
add_definitions(-DRAPIDJSON_HAS_STDSTRING=1)
add_definitions(-DBSON_STATIC=1)

# Include directories
include_directories(${LIBBSON_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${LIBBSON_LIBRARY_DIRS})
add_definitions(${LIBBSON_CFLAGS_OTHER})

# Libraries
set(LIBS ${LIBS} ${LIBBSON_LIBRARIES})
set(LIBS ${LIBS} ${CMAKE_THREAD_LIBS_INIT})
set(LIBS ${LIBS} ${Boost_LIBRARIES})

if(WIN32 AND NOT PkgConfig_FOUND)
	set(LIBS ${LIBS} "ws2_32") # Needed by libbson on windows 
endif()

INCLUDE_DIRECTORIES( include)

#WebSocket Client
ADD_EXECUTABLE( websocket_client
	src/client/websocket_client.cpp 
	src/client/socket_websocket_connection.cpp 
	src/ros_bridge.cpp 
	src/ros_topic.cpp 
	src/ros_service.cpp 
	src/ros_tf_broadcaster.cpp 
)

# Alternative TCP client (for ROS1 compatibility)
ADD_EXECUTABLE( tcp_client
	src/client/tcp_client.cpp 
	src/client/socket_tcp_connection.cpp 
	src/ros_bridge.cpp 
	src/ros_topic.cpp 
	src/ros_service.cpp 
	src/ros_tf_broadcaster.cpp 
)

ADD_LIBRARY( rosbridge2cpp SHARED
	src/ros_bridge.cpp 
	src/ros_topic.cpp 
	src/ros_service.cpp 
	src/ros_tf_broadcaster.cpp 
)
target_link_libraries(websocket_client ${LIBS})
target_link_libraries(tcp_client ${LIBS})
target_link_libraries(rosbridge2cpp ${LIBS})