#pragma once
#include <iostream>
#include <string>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/client.hpp>
#include <bson.h>

#include "rapidjson/document.h"

#include "itransport_layer.h"
#include "types.h"

using json = rapidjson::Document;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

namespace rosbridge2cpp{
  class SocketWebSocketConnection : public ITransportLayer{
    public:
      SocketWebSocketConnection() = default;
      
      ~SocketWebSocketConnection() {
        std::cout << "[WebSocketConnection] Destructor called" << std::endl;
        if (is_connected_) {
          Disconnect();
        } else {
          // Clean up threads even if connection failed
          terminate_receiver_thread_ = true;
          terminate_reconnect_thread_ = true;
          if (receiver_thread_set_up_ && receiver_thread_.joinable()) {
            receiver_thread_.join();
          }
          if (reconnect_thread_set_up_ && reconnect_thread_.joinable()) {
            reconnect_thread_.join();
          }
          // Clean up ASIO thread if it was created
          if (asio_thread_ && asio_thread_->joinable()) {
            c_.stop_perpetual();
            c_.stop();
            asio_thread_->join();
          }
        }
      }

      bool Init(std::string p_ip_addr, int p_port);
      bool SendMessage(std::string data);
      bool SendMessage(const uint8_t *data, unsigned int length);
      std::string GetLastSentMessage() const;
      bool IsConnected() const;
      int ReceiverThreadFunction();
      void ReconnectThreadFunction();
      bool AttemptReconnect();
      void RegisterIncomingMessageCallback(std::function<void(json&)> fun);
      void RegisterIncomingMessageCallback(std::function<void(bson_t&)> fun);
      void RegisterErrorCallback(std::function<void(TransportError)> fun);
      void ReportError(TransportError err);
      void SetTransportMode(ITransportLayer::TransportMode mode);
      void Disconnect();

    private:
      typedef websocketpp::client<websocketpp::config::asio> client;
      typedef websocketpp::connection_hdl connection_hdl;
      typedef websocketpp::config::asio::message_type::ptr message_ptr;
      
      std::string ip_addr_;
      int port_;
      std::string uri_;
      
      client c_;
      connection_hdl hdl_;
      websocketpp::lib::shared_ptr<websocketpp::lib::thread> asio_thread_;
      
      std::thread receiver_thread_;
      bool terminate_receiver_thread_ = false;
      bool receiver_thread_set_up_ = false;
      bool is_connected_ = false;
      bool is_reconnecting_ = false;
      bool callback_function_defined_ = false;
      bool bson_only_mode_ = false;
      bool auto_reconnect_ = true;
      bool terminate_reconnect_thread_ = false;
      std::thread reconnect_thread_;
      bool reconnect_thread_set_up_ = false;
      
      std::function<void(json&)> incoming_message_callback_;
      std::function<void(bson_t&)> incoming_message_callback_bson_;
      std::function<void(TransportError)> error_callback_ = nullptr;
      
      mutable std::mutex connection_mutex_;
      std::condition_variable connection_cv_;
      
      mutable std::string last_sent_message_;
      mutable std::mutex last_message_mutex_;
      
      // Reconnection state tracking
      mutable std::mutex reconnect_mutex_;
      bool last_send_failed_logged_ = false;
      
      // Helper functions
      void RegisterWebSocketHandlers();
      void SetupASIOThread();
      bool IsConnectedOrReconnecting() const;
      
      // WebSocket event handlers
      void on_open(connection_hdl hdl);
      void on_close(connection_hdl hdl);
      void on_fail(connection_hdl hdl);
      void on_message(connection_hdl hdl, message_ptr msg);
  };
}