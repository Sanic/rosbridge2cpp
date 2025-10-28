# rosbridge2cpp [![Build Status](https://travis-ci.org/Sanic/rosbridge2cpp.svg?branch=master)](https://travis-ci.org/Sanic/rosbridge2cpp)
A C++11 library to interface ROS via rosbridge

This library can be used to talk to [ROS](http://www.ros.org/) and [ROS2](https://www.ros.org/) via [rosbridge](http://wiki.ros.org/rosbridge_suite).
It enables you to communicate with ROS even if you don't have the full ROS Stack on your machine available (for example, when you are using Windows). The network communication of this library uses WebSocket protocol for compatibility with ROS2 rosbridge.

Please note that this library is in an early development stage and features are added as needed. 
Even though the library supports JSON and BSON, it's strongly recommended to use BSON.

## Dependencies

Before you can compile the library, you need to install [websocketpp](https://github.com/zaphoyd/websocketpp), [jsoncpp](https://github.com/open-source-parsers/jsoncpp), [libbson](https://github.com/mongodb/libbson), and [Boost](https://www.boost.org/).

```
sudo apt update
sudo apt install -y libwebsocketpp-dev libjsoncpp-dev libbson-dev libboost-all-dev
```

If you want to run rosbridge on the same machine:
```
# For ROS1 and ROS2
sudo apt install -y ros-${ROS_DISTRO}-rosbridge-suite
```

## Compiling the library

Now you can clone this repo. Please use --recursive if you want to use the unit tests.
After cloning the repo, change into that directory and execute:
```
mkdir build
cd build
cmake .. # or 'cmake .. -Dtest=on' if you build the unit tests
make 
```

## Usage

The library provides two transport options:

### WebSocket Client (ROS2 Compatible)
Checkout [src/client/websocket_client.cpp](src/client/websocket_client.cpp) for an example implementation using WebSocket connections.

**For ROS2 (Recommended):**
```
ros2 run rosbridge_server rosbridge_websocket
# or
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**For ROS1:**
```
roslaunch rosbridge_server rosbridge_websocket.launch bson_only_mode:=True
```

### TCP Client (ROS1 Compatible)
Checkout [src/client/tcp_client.cpp](src/client/tcp_client.cpp) for an example implementation using TCP connections.

**For ROS1:**
```
roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True
```

## Building Both Clients

The CMakeLists.txt builds both clients:
- `websocket_client` - WebSocket client (default)
- `tcp_client` - TCP client (for ROS1 compatibility)

### ROS2 Example
```
# Start the rosbridge websocket server:
ros2 run rosbridge_server rosbridge_websocket

# Publish a simple string message to a topic:
ros2 topic pub /test std_msgs/String "data: 'hello from ROS2'"
```

### ROS1 Example
```
# Start the rosbridge websocket server:
roslaunch rosbridge_server rosbridge_websocket.launch

# Publish a simple string message to a topic:
rostopic pub /test std_msgs/String "data: 'hello from ROS1'"
```

