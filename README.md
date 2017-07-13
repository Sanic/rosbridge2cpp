# rosbridge2cpp [![Build Status](https://travis-ci.org/Sanic/rosbridge2cpp.svg?branch=master)](https://travis-ci.org/Sanic/rosbridge2cpp)
A C++11 library to interface ROS via rosbridge

This library can be used to talk to [ROS](http://www.ros.org/) via [rosbridge](http://wiki.ros.org/rosbridge_suite).
It enables you to communicate with ROS even if you don't have the full ROS Stack on your machine available (for example, when you are using Windows).
The network communication of this library is abstracted from a specific network implementation.
This abstraction allows you to use this library in different contexts, for example in Windows Applications or even in Game Engines like [Unreal](https://www.unrealengine.com/).

Please note that this library is in an early development stage and features are added as needed. 
Even though the library supports JSON and BSON, it's strongly recommended to use BSON.

## Compiling the library

Before you can compile the library, you need to install [libbson](https://github.com/mongodb/libbson). In short, this can be accomplished by:
```
wget https://github.com/mongodb/libbson/releases/download/1.5.3/libbson-1.5.3.tar.gz
tar xzf libbson-1.5.3.tar.gz
cd libbson-1.5.3
./configure --prefix=$HOME/deps/libbson && make -j$(grep -c ^processor /proc/cpuinfo) && make install
```
Please note that libbson will not be installed system-wide to avoid interference with existing installations that might need libbson in a different version.
To be able to find this library at link-time, we'll add the pkgconfig to our .bashrc to persist the change:
```
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/deps/libbson/lib/pkgconfig/ >> ~/.bashrc
source ~/.bashrc
```
Now you can clone this repo. Please use --recursive if you want to use the unit tests.
After cloning the repo, change into that directory and execute:
```
mkdir build
cd build
cmake .. # or 'cmake .. -Dtest=on' if you build the unit tests
make 
```

## Usage
Checkout [src/client/client.cpp](src/client/client.cpp) for an example implementation based on UNIX sockets.
On the server-side, please ensure that you're starting the TCP variant of the rosbridge server.
Websockets are currently not supported.

In order to use the recommended, full-duplex BSON variant of the library, you need a recent rosbridge version that has been released after March 15 2017. At the time of writing this README, there is no release after that date. So right now, you need to install  [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) from the repository. Everything after commit [f0844e2](https://github.com/RobotWebTools/rosbridge_suite/commit/f0844e24d05ded3c4ab803dc235c339e854175e8) should be fine.
When you've rosbridge_suite downloaded and put in your ROS workspace, you can launch rosbridge with BSON-Mode like this:
```
roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True
```

## Running the unit tests
Please ensure that the you executed cmake with '-Dtest=on' before you continue.
When the library and the unit tests are compiled, execute the following commands on a machine running ROS to setup a minimal testing environment:
```
roslaunch rosbridge_server rosbridge_tcp.launch # plus bson_only_mode:=True if you want to use BSON
rostopic pub /test std_msgs/String a5424890996794277159554918
rosrun rospy_tutorials add_two_ints_server
```
Write down the IP address of the machine where you executed these commands mentally and change to your 'build/' directory.
Call the Unit Tests like this:
```
export rosb2_cpp_ip=THE_IP_ADDRESS_OF_YOUR_ROS_MACHINE; export rosb2_cpp_port=9090; ./runUnitTests
```

