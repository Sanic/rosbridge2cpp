/*
 * This is an example file to get a feeling for the usage of this library.
 * You may also check our unit tests in tests/tests.cpp to see more functions in action.
 *
 * Please note that this example file uses the socket_tcp_connection class to talk
 * to the rosbridge server. socket_tcp_connection relies on traditional unix sockets and may
 * not work on your individual system.
 */
#include <iostream>
#include <thread>

#include "types.h"
#include "ros_bridge.h"
#include "ros_topic.h"
#include "ros_service.h"
#include "client/socket_tcp_connection.h"

using namespace rosbridge2cpp;

void connection_error_handler(TransportError err) {
  if(err == TransportError ::CONNECTION_CLOSED)
    std::cout << "Connection closed - You should reinit ROSBridge" << std::endl;
  if(err == TransportError ::SOCKET_ERROR)
    std::cout << "Error on ROSBridge Socket - You should reinit ROSBridge" << std::endl;
}


int main()
{
  SocketTCPConnection t;
  t.RegisterErrorCallback(connection_error_handler);


  ROSBridge ros(t);
  ros.enable_bson_mode();
  if( !ros.Init("127.0.0.1", 9090))
  {
    std::cerr << "Failed to connect to ROSBridge" << std::endl;
    return 0;
  }

  ROSTopic test_topic(ros, "/test_rosbridge2cpp", "std_msgs/String");

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  bson_t *message = BCON_NEW(
      "data", "Publish from Test Client"
      );
  test_topic.Publish(message);

  while(true){
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  
}
