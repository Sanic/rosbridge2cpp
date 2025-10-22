/*
 * This is an example file to get a feeling for the usage of this library.
 * You may also check our unit tests in tests/tests.cpp to see more functions in action.
 *
 * This example file uses the socket_websocket_connection class to talk
 * to the rosbridge server via WebSocket protocol, which is compatible with ROS2.
 */
#include <iostream>
#include <thread>

#include "types.h"
#include "ros_bridge.h"
#include "ros_topic.h"
#include "client/socket_websocket_connection.h"

using namespace rosbridge2cpp;

void connection_error_handler(TransportError err) {
  if(err == TransportError ::R2C_CONNECTION_CLOSED)
    std::cout << "Connection closed - You should reinit ROSBridge" << std::endl;
  if(err == TransportError ::R2C_SOCKET_ERROR)
    std::cout << "Error on ROSBridge Socket - You should reinit ROSBridge" << std::endl;
}


int main()
{
  SocketWebSocketConnection t;
  t.RegisterErrorCallback(connection_error_handler);


  ROSBridge ros(t);
  // ros.enable_bson_mode();
  
  std::cout << "Attempting to connect to ROSBridge server at ws://127.0.0.1:9090..." << std::endl;
  if( !ros.Init("127.0.0.1", 9090))
  {
    std::cerr << "Failed to connect to ROSBridge server!" << std::endl;
    std::cerr << "Please ensure that:" << std::endl;
    std::cerr << "1. ROS2 is running (ros2 run rosbridge_server rosbridge_websocket)" << std::endl;
    std::cerr << "2. ROSBridge WebSocket server is running:" << std::endl;
    std::cerr << "   ros2 run rosbridge_server rosbridge_websocket" << std::endl;
    std::cerr << "3. The server is listening on port 9090" << std::endl;
    return 1;
  }
  
  std::cout << "Successfully connected to ROSBridge server!" << std::endl;

  ROSTopic test_topic(ros, "/test_rosbridge2cpp", "std_msgs/String");
  std::cout << "Created topic: " << test_topic.TopicName() << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // std::cout << "Publishing BSON message..." << std::endl;
  // bson_t *message = BCON_NEW(
  //     "data", "Publish from Test Client"
  //     );
  // test_topic.Publish(message);



  std::cout << "Starting continuous publishing loop..." << std::endl;
  while(true){
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    // Create a JSON message and publish it to the test_topic
    rapidjson::Document json_msg;
    json_msg.SetObject();
    json_msg.AddMember("data", "Publish from Test Client (JSON)", json_msg.GetAllocator());
    test_topic.Publish(json_msg);
    
    std::cout << "Published JSON message to " << test_topic.TopicName() << std::endl;
  }
  
}
