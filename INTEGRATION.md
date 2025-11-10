# Using rosbridge2cpp in Another Project

This guide explains how to use the `rosbridge2cpp` library in your own CMake project by including it as a subdirectory.

## Method 1: Using as a Subdirectory (Recommended)

If you place this repository in a `resources` directory of your project, you can use it as follows:

### Project Structure
```
your_project/
├── CMakeLists.txt
├── src/
│   └── your_code.cpp
└── resources/
    └── rosbridge_client_cpp/  (this repository)
        ├── CMakeLists.txt
        ├── include/
        └── src/
```

### Your Project's CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(YourProject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the rosbridge2cpp library as a subdirectory
add_subdirectory(resources/rosbridge_client_cpp)

# Your executable
add_executable(your_executable
    src/your_code.cpp
)

# Link against rosbridge2cpp
# This automatically includes the header directories and links the library
target_link_libraries(your_executable
    PRIVATE
        rosbridge2cpp
)
```

### Example Usage in Your Code

```cpp
#include "ros_bridge.h"
#include "client/socket_websocket_connection.h"
#include "ros_topic.h"

using namespace rosbridge2cpp;

int main() {
    // Create a WebSocket connection
    SocketWebSocketConnection transport;
    
    // Create the ROS bridge
    ROSBridge bridge(transport);
    
    // Connect to rosbridge server
    if (!bridge.Init("127.0.0.1", 9090)) {
        std::cerr << "Failed to connect!" << std::endl;
        return 1;
    }
    
    // Create a topic
    ROSTopic topic(bridge, "/my_topic", "std_msgs/String");
    
    // Publish a message
    rapidjson::Document msg;
    msg.SetObject();
    msg.AddMember("data", "Hello from my project!", msg.GetAllocator());
    topic.Publish(msg);
    
    return 0;
}
```

## Method 2: Installing and Using find_package

If you want to install the library system-wide or in a specific location:

### Build and Install

```bash
cd resources/rosbridge_client_cpp
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/install
make install
```

### Use in Your Project

```cmake
cmake_minimum_required(VERSION 3.10)
project(YourProject)

set(CMAKE_CXX_STANDARD 11)

# Find the installed library
find_package(rosbridge2cpp REQUIRED)

add_executable(your_executable src/your_code.cpp)
target_link_libraries(your_executable PRIVATE rosbridge2cpp)
```

## Dependencies

Make sure your system has the required dependencies installed:

```bash
sudo apt update
sudo apt install -y \
    libwebsocketpp-dev \
    libjsoncpp-dev \
    libbson-dev \
    libboost-all-dev \
    pkg-config
```

## Notes

- The library uses C++11 standard
- When used as a subdirectory, the example executables (`websocket_client`, `tcp_client`, `demo_client`) are only built when this is the main project
- The library target automatically propagates include directories and compile definitions to your project
- All dependencies (Boost, libbson, etc.) are linked privately, so you don't need to link them in your project


