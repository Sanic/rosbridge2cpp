/*
 * Comprehensive example demonstrating various features of the rosbridge2cpp library.
 * 
 * This example shows:
 * - Connecting to ROSBridge server (WebSocket or TCP)
 * - Publishing messages to topics
 * - Subscribing to topics with callbacks
 * - Calling ROS services
 * - Advertising ROS services
 * 
 * Usage:
 *   For WebSocket (ROS2): ros2 run rosbridge_server rosbridge_websocket
 *   For TCP (ROS1): roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>

#include "types.h"
#include "ros_bridge.h"
#include "ros_topic.h"
#include "ros_service.h"
#include "client/socket_websocket_connection.h"
#include "client/socket_tcp_connection.h"

using namespace rosbridge2cpp;

// Global flag for graceful shutdown
volatile bool g_running = true;

// Signal handler for graceful shutdown
void signal_handler(int signum) {
    std::cout << "\nReceived signal " << signum << ", shutting down gracefully..." << std::endl;
    g_running = false;
}

// Connection error handler
void connection_error_handler(TransportError err) {
    if(err == TransportError::R2C_CONNECTION_CLOSED)
        std::cout << "[ERROR] Connection closed - You should reinit ROSBridge" << std::endl;
    if(err == TransportError::R2C_SOCKET_ERROR)
        std::cout << "[ERROR] Error on ROSBridge Socket - You should reinit ROSBridge" << std::endl;
}

// Callback for subscribed topic messages
void message_callback(const ROSBridgePublishMsg &message) {
    std::cout << "[SUBSCRIBE] Received message on topic:" << std::endl;
    
    if (message.msg_json_.HasMember("data")) {
        if (message.msg_json_["data"].IsString()) {
            std::string data = message.msg_json_["data"].GetString();
            std::cout << "  Data: " << data << std::endl;
        }
    }
    
    // Print full JSON for debugging (optional)
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    message.msg_json_.Accept(writer);
    std::cout << "  Full message: " << buffer.GetString() << std::endl;
}

// Callback for service responses
void service_response_callback(ROSBridgeServiceResponseMsg &response) {
    std::cout << "[SERVICE] Received service response:" << std::endl;
    
    if (!response.values_json_.IsNull()) {
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        response.values_json_.Accept(writer);
        std::cout << "  Values: " << buffer.GetString() << std::endl;
    }
    
    std::cout << "  Result: " << (response.result_ ? "true" : "false") << std::endl;
}

// Note: Service request callback is now implemented as a lambda in main() 
// to have access to the ROSBridge instance for sending responses

int main(int argc, char** argv)
{
    // Register signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Choose transport type based on command line argument
    // Default to WebSocket, use TCP if "tcp" argument is provided
    bool use_tcp = (argc > 1 && std::string(argv[1]) == "tcp");
    std::string transport_type = use_tcp ? "TCP" : "WebSocket";
    std::string server_address = "127.0.0.1";
    int server_port = 9090;
    
    std::cout << "================================================" << std::endl;
    std::cout << "  ROSBridge2CPP Demo Client" << std::endl;
    std::cout << "  Transport: " << transport_type << std::endl;
    std::cout << "================================================" << std::endl;
    
    // Create transport layer
    SocketWebSocketConnection* ws_connection = nullptr;
    SocketTCPConnection* tcp_connection = nullptr;
    ITransportLayer* transport = nullptr;
    
    if (use_tcp) {
        tcp_connection = new SocketTCPConnection();
        tcp_connection->RegisterErrorCallback(connection_error_handler);
        transport = tcp_connection;
    } else {
        ws_connection = new SocketWebSocketConnection();
        ws_connection->RegisterErrorCallback(connection_error_handler);
        transport = ws_connection;
    }
    
    // Create ROSBridge instance
    ROSBridge ros(*transport);
    // Uncomment to enable BSON mode (recommended for performance)
    // ros.enable_bson_mode();
    
    // Connect to ROSBridge server
    std::cout << "\nAttempting to connect to ROSBridge server at " 
              << server_address << ":" << server_port << "..." << std::endl;
    
    if (!ros.Init(server_address, server_port)) {
        std::cerr << "[ERROR] Failed to connect to ROSBridge server!" << std::endl;
        std::cerr << "\nPlease ensure that:" << std::endl;
        if (use_tcp) {
            std::cerr << "1. ROS1 is running (roscore)" << std::endl;
            std::cerr << "2. ROSBridge TCP server is running:" << std::endl;
            std::cerr << "   roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True" << std::endl;
        } else {
            std::cerr << "1. ROS2 is running" << std::endl;
            std::cerr << "2. ROSBridge WebSocket server is running:" << std::endl;
            std::cerr << "   ros2 run rosbridge_server rosbridge_websocket" << std::endl;
        }
        std::cerr << "3. The server is listening on port " << server_port << std::endl;
        
        delete transport;
        return 1;
    }
    
    std::cout << "[SUCCESS] Connected to ROSBridge server!" << std::endl;
    std::cout << "[INFO] Health check: " << (ros.IsHealthy() ? "OK" : "NOT OK") << std::endl;
    
    // Give some time for connection to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // ============================================
    // Example 1: Publishing to a topic
    // ============================================
    std::cout << "\n--- Example 1: Publishing to a topic ---" << std::endl;
    ROSTopic publish_topic(ros, "/demo_publish", "std_msgs/String");
    publish_topic.Advertise();
    std::cout << "[INFO] Advertised publisher on topic: " << publish_topic.TopicName() << std::endl;
    
    // ============================================
    // Example 2: Subscribing to a topic
    // ============================================
    std::cout << "\n--- Example 2: Subscribing to a topic ---" << std::endl;
    ROSTopic subscribe_topic(ros, "/demo_subscribe", "std_msgs/String");
    auto callback_handle = subscribe_topic.Subscribe(message_callback);
    std::cout << "[INFO] Subscribed to topic: " << subscribe_topic.TopicName() << std::endl;
    
    // ============================================
    // Example 3: Calling a ROS service
    // ============================================
    std::cout << "\n--- Example 3: Calling a ROS service ---" << std::endl;
    ROSService service_client(ros, "/rosout/get_loggers", "roscpp/GetLoggers");
    
    // Prepare service request
    rapidjson::Document request;
    request.SetObject();
    request.AddMember("level", "DEBUG", request.GetAllocator());
    
    std::cout << "[INFO] Calling service: " << service_client.ServiceName() << std::endl;
    service_client.CallService(request, service_response_callback);
    
    // ============================================
    // Example 4: Advertising a service
    // ============================================
    std::cout << "\n--- Example 4: Advertising a service ---" << std::endl;
    ROSService service_server(ros, "/demo_service", "std_srvs/Empty");
    
    // Create a lambda that captures the ROSBridge reference to send the response
    auto service_handler = [&ros](ROSBridgeCallServiceMsg &request, 
                                   rapidjson::Document::AllocatorType &allocator) {
        std::cout << "[SERVICE] Received service request for /demo_service:" << std::endl;
        
        // Process the request
        if (!request.args_json_.IsNull()) {
            rapidjson::StringBuffer buffer;
            rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
            request.args_json_.Accept(writer);
            std::cout << "  Request args: " << buffer.GetString() << std::endl;
        }
        
        // Create and send response
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = true;
        
        // Echo back the args as values
        if (!request.args_json_.IsNull()) {
            response.values_json_.CopyFrom(request.args_json_, allocator);
        }
        
        std::cout << "  Sending service response..." << std::endl;
        ros.SendMessage(response);
    };
    
    service_server.Advertise(service_handler);
    std::cout << "[INFO] Advertised service: " << service_server.ServiceName() << std::endl;
    
    // ============================================
    // Main loop: Publish messages periodically
    // ============================================
    std::cout << "\n--- Starting main loop ---" << std::endl;
    std::cout << "[INFO] Publishing messages every 2 seconds..." << std::endl;
    std::cout << "[INFO] Press Ctrl+C to stop" << std::endl;
    
    int message_count = 0;
    
    while (g_running && ros.IsHealthy()) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Publish a message
        rapidjson::Document json_msg;
        json_msg.SetObject();
        
        std::string message_text = "Demo message #" + std::to_string(message_count++);
        rapidjson::Value data_value;
        data_value.SetString(message_text.c_str(), json_msg.GetAllocator());
        json_msg.AddMember("data", data_value, json_msg.GetAllocator());
        
        if (publish_topic.Publish(json_msg)) {
            std::cout << "[PUBLISH] Sent: " << message_text 
                      << " to " << publish_topic.TopicName() << std::endl;
        } else {
            std::cerr << "[ERROR] Failed to publish message!" << std::endl;
        }
    }
    
    // ============================================
    // Cleanup
    // ============================================
    std::cout << "\n--- Shutting down ---" << std::endl;
    
    // Unsubscribe from topic
    subscribe_topic.Unsubscribe(callback_handle);
    std::cout << "[INFO] Unsubscribed from topic" << std::endl;
    
    // Unadvertise publisher
    publish_topic.Unadvertise();
    std::cout << "[INFO] Unadvertised publisher" << std::endl;
    
    // Unadvertise service
    service_server.Unadvertise();
    std::cout << "[INFO] Unadvertised service" << std::endl;
    
    // Cleanup transport
    delete transport;
    
    std::cout << "[INFO] Shutdown complete. Goodbye!" << std::endl;
    
    return 0;
}

