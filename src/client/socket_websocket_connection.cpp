#include "client/socket_websocket_connection.h"
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>

namespace rosbridge2cpp{

/******************************************************************************************
 * HELPER FUNCTIONS
 *****************************************************************************************/

void SocketWebSocketConnection::RegisterWebSocketHandlers() {
  c_.set_message_handler(bind(&SocketWebSocketConnection::on_message, this, ::_1, ::_2));
  c_.set_open_handler(bind(&SocketWebSocketConnection::on_open, this, ::_1));
  c_.set_close_handler(bind(&SocketWebSocketConnection::on_close, this, ::_1));
  c_.set_fail_handler(bind(&SocketWebSocketConnection::on_fail, this, ::_1));
}

void SocketWebSocketConnection::SetupASIOThread() {
  if (!asio_thread_ || !asio_thread_->joinable()) {
    c_.init_asio();
    RegisterWebSocketHandlers();
    c_.start_perpetual();
    asio_thread_ = websocketpp::lib::make_shared<websocketpp::lib::thread>(&client::run, &c_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Give ASIO thread time to start
  } else {
    RegisterWebSocketHandlers();
  }
}

bool SocketWebSocketConnection::IsConnectedOrReconnecting() const {
  std::lock_guard<std::mutex> lock(connection_mutex_);
  return is_connected_ || is_reconnecting_;
}

/******************************************************************************************
 * INITIALIZATION
 *****************************************************************************************/

bool SocketWebSocketConnection::Init(std::string p_ip_addr, int p_port){
  ip_addr_ = p_ip_addr;
  port_ = p_port;
  uri_ = "ws://" + ip_addr_ + ":" + std::to_string(port_);
  
  std::cout << "[WebSocketConnection] Initializing connection to " << uri_ << std::endl;
  
  try {
    // Configure logging
    c_.clear_access_channels(websocketpp::log::alevel::all);
    c_.set_access_channels(websocketpp::log::alevel::connect);
    c_.set_access_channels(websocketpp::log::alevel::disconnect);
    c_.set_error_channels(websocketpp::log::elevel::all);
    
    // Setup ASIO and handlers
    SetupASIOThread();
    
    // Create and initiate connection
    websocketpp::lib::error_code ec;
    client::connection_ptr con = c_.get_connection(uri_, ec);
    if (ec) {
      std::cout << "[WebSocketConnection] Could not create connection: " << ec.message() << std::endl;
      return false;
    }
    
    hdl_ = con->get_handle();
    c_.connect(con);
    
    // Wait for connection
    std::unique_lock<std::mutex> lock(connection_mutex_);
    if (!connection_cv_.wait_for(lock, std::chrono::seconds(5), [this] { return is_connected_; })) {
      std::cout << "[WebSocketConnection] Connection timeout" << std::endl;
      return false;
    }
    
    std::cout << "[WebSocketConnection] Connected successfully" << std::endl;
    
    // Start receiver thread
    receiver_thread_ = std::thread([=]() {ReceiverThreadFunction(); return 1; });
    receiver_thread_set_up_ = true;
    
    // Start reconnection thread if auto-reconnect is enabled
    if (auto_reconnect_) {
      terminate_reconnect_thread_ = false;
      reconnect_thread_ = std::thread([=]() {ReconnectThreadFunction(); return 1; });
      reconnect_thread_set_up_ = true;
    }
    
    return true;
    
  } catch (websocketpp::exception const & e) {
    std::cout << "[WebSocketConnection] Exception: " << e.what() << std::endl;
    return false;
  }
}

/******************************************************************************************
 * MESSAGE SENDING
 *****************************************************************************************/

bool SocketWebSocketConnection::SendMessage(std::string data){
  // Check connection state (thread-safe)
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    if (is_reconnecting_) {
      return false; // Don't send while reconnecting
    }
    if (!is_connected_) {
      // Only log once to avoid spam
      std::lock_guard<std::mutex> reconnect_lock(reconnect_mutex_);
      if (!last_send_failed_logged_) {
        std::cout << "[WebSocketConnection] Not connected, waiting for reconnection..." << std::endl;
        last_send_failed_logged_ = true;
      }
      return false;
    }
  }
  
  try {
    websocketpp::lib::error_code ec;
    c_.send(hdl_, data, websocketpp::frame::opcode::text, ec);
    if (ec) {
      std::cout << "[WebSocketConnection] Send failed: " << ec.message() << std::endl;
      return false;
    }
    
    {
      std::lock_guard<std::mutex> lock(last_message_mutex_);
      last_sent_message_ = data;
    }
    
    // Reset failure logging flag on successful send
    {
      std::lock_guard<std::mutex> lock(reconnect_mutex_);
      last_send_failed_logged_ = false;
    }
    
    return true;
    
  } catch (websocketpp::exception const & e) {
    std::cout << "[WebSocketConnection] Send exception: " << e.what() << std::endl;
    return false;
  }
}

bool SocketWebSocketConnection::SendMessage(const uint8_t *data, unsigned int length){
  // Check connection state (thread-safe)
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    if (is_reconnecting_) {
      return false; // Don't send while reconnecting
    }
    if (!is_connected_) {
      return false;
    }
  }
  
  try {
    websocketpp::lib::error_code ec;
    c_.send(hdl_, data, length, websocketpp::frame::opcode::binary, ec);
    if (ec) {
      std::cout << "[WebSocketConnection] Send failed: " << ec.message() << std::endl;
      return false;
    }
    
    {
      std::lock_guard<std::mutex> lock(last_message_mutex_);
      std::ostringstream oss;
      oss << "[Binary: " << length << " bytes]";
      for (unsigned int i = 0; i < length && i < 32; i++) {
        oss << ":" << std::setw(2) << std::setfill('0') << std::hex << (int)(data[i]);
      }
      if (length > 32) {
        oss << "...";
      }
      last_sent_message_ = oss.str();
    }
    
    return true;
    
  } catch (websocketpp::exception const & e) {
    std::cout << "[WebSocketConnection] Send exception: " << e.what() << std::endl;
    return false;
  }
}

std::string SocketWebSocketConnection::GetLastSentMessage() const {
  std::lock_guard<std::mutex> lock(last_message_mutex_);
  return last_sent_message_;
}

bool SocketWebSocketConnection::IsConnected() const {
  std::lock_guard<std::mutex> lock(connection_mutex_);
  return is_connected_ && !is_reconnecting_;
}

/******************************************************************************************
 * THREAD FUNCTIONS
 *****************************************************************************************/

int SocketWebSocketConnection::ReceiverThreadFunction(){
  std::cout << "[WebSocketConnection] Receiver thread started" << std::endl;
  std::cout << "[WebSocketConnection] bson_only_mode: " << bson_only_mode_ << std::endl;
  
  while (!terminate_receiver_thread_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  std::cout << "[WebSocketConnection] Receiver thread terminated" << std::endl;
  return 0;
}

void SocketWebSocketConnection::ReconnectThreadFunction(){
  std::cout << "[WebSocketConnection] Reconnection thread started" << std::endl;
  
  while (!terminate_reconnect_thread_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Check if we need to reconnect
    {
      std::lock_guard<std::mutex> lock(connection_mutex_);
      if (is_connected_ || is_reconnecting_ || !auto_reconnect_ || terminate_reconnect_thread_) {
        continue;
      }
    }
    
    // Attempt reconnection
    std::cout << "[WebSocketConnection] Attempting to reconnect..." << std::endl;
    if (AttemptReconnect()) {
      std::cout << "[WebSocketConnection] Reconnection successful" << std::endl;
      // Reset failure logging flag on successful reconnect
      {
        std::lock_guard<std::mutex> lock(reconnect_mutex_);
        last_send_failed_logged_ = false;
      }
    } else {
      std::cout << "[WebSocketConnection] Reconnection failed, will retry in 2 seconds" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }
  
  std::cout << "[WebSocketConnection] Reconnection thread terminated" << std::endl;
}

bool SocketWebSocketConnection::AttemptReconnect(){
  // Check if already connected
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    if (is_connected_) {
      return true;
    }
    is_reconnecting_ = true;
  }
  
  try {
    // Setup ASIO thread and handlers
    SetupASIOThread();
    
    // Create new connection
    websocketpp::lib::error_code ec;
    client::connection_ptr con = c_.get_connection(uri_, ec);
    if (ec) {
      std::cout << "[WebSocketConnection] Could not create reconnection: " << ec.message() << std::endl;
      {
        std::lock_guard<std::mutex> lock(connection_mutex_);
        is_reconnecting_ = false;
      }
      return false;
    }
    
    hdl_ = con->get_handle();
    c_.connect(con);
    
    // Wait for connection
    std::unique_lock<std::mutex> lock(connection_mutex_);
    if (!connection_cv_.wait_for(lock, std::chrono::seconds(3), [this] { return is_connected_; })) {
      std::cout << "[WebSocketConnection] Reconnection timeout" << std::endl;
      is_reconnecting_ = false;
      return false;
    }
    
    // Restart receiver thread if needed
    if (!receiver_thread_set_up_ || !receiver_thread_.joinable()) {
      terminate_receiver_thread_ = false;
      receiver_thread_ = std::thread([=]() {ReceiverThreadFunction(); return 1; });
      receiver_thread_set_up_ = true;
      std::cout << "[WebSocketConnection] Receiver thread restarted" << std::endl;
    }
    
    is_reconnecting_ = false;
    return true;
    
  } catch (websocketpp::exception const & e) {
    std::cout << "[WebSocketConnection] Reconnection exception: " << e.what() << std::endl;
    {
      std::lock_guard<std::mutex> lock(connection_mutex_);
      is_reconnecting_ = false;
    }
    return false;
  }
}

/******************************************************************************************
 * CALLBACK REGISTRATION
 *****************************************************************************************/

void SocketWebSocketConnection::RegisterIncomingMessageCallback(std::function<void(json&)> fun){
  incoming_message_callback_ = fun;
  callback_function_defined_ = true;
}

void SocketWebSocketConnection::RegisterIncomingMessageCallback(std::function<void(bson_t&)> fun){
  incoming_message_callback_bson_ = fun;
  callback_function_defined_ = true;
}

void SocketWebSocketConnection::RegisterErrorCallback(std::function<void(TransportError)> fun){
  error_callback_ = fun;
}

void SocketWebSocketConnection::SetTransportMode(ITransportLayer::TransportMode mode){
  switch(mode){
    case ITransportLayer::JSON:
      bson_only_mode_ = false;
      break;
    case ITransportLayer::BSON:
      bson_only_mode_ = true;
      break;
    default:
      std::cerr << "[WebSocketConnection] Given TransportMode not implemented" << std::endl;
  }
}

/******************************************************************************************
 * DISCONNECTION
 *****************************************************************************************/

void SocketWebSocketConnection::Disconnect(){
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    if (!is_connected_) {
      return;
    }
  }
  
  auto_reconnect_ = false;
  terminate_reconnect_thread_ = true;
  
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    is_reconnecting_ = false;
  }
  
  try {
    websocketpp::lib::error_code ec;
    c_.close(hdl_, websocketpp::close::status::normal, "", ec);
    if (ec) {
      std::cout << "[WebSocketConnection] Error on close: " << ec.message() << std::endl;
    }
  } catch (websocketpp::exception const & e) {
    std::cout << "[WebSocketConnection] Exception on close: " << e.what() << std::endl;
  }
  
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    is_connected_ = false;
  }
  
  terminate_receiver_thread_ = true;
  
  if (reconnect_thread_set_up_ && reconnect_thread_.joinable()) {
    reconnect_thread_.join();
    reconnect_thread_set_up_ = false;
  }
  
  if (asio_thread_ && asio_thread_->joinable()) {
    c_.stop_perpetual();
    c_.stop();
    asio_thread_->join();
  }
}

/******************************************************************************************
 * WEBSOCKET EVENT HANDLERS
 *****************************************************************************************/

void SocketWebSocketConnection::on_open(connection_hdl hdl) {
  std::cout << "[WebSocketConnection] Connection opened" << std::endl;
  std::unique_lock<std::mutex> lock(connection_mutex_);
  is_connected_ = true;
  is_reconnecting_ = false;
  connection_cv_.notify_all();
}

void SocketWebSocketConnection::on_close(connection_hdl hdl) {
  std::cout << "[WebSocketConnection] Connection closed" << std::endl;
  std::unique_lock<std::mutex> lock(connection_mutex_);
  is_connected_ = false;
  connection_cv_.notify_all();
  
  if (!terminate_receiver_thread_ && auto_reconnect_) {
    ReportError(TransportError::R2C_CONNECTION_CLOSED);
  }
}

void SocketWebSocketConnection::on_fail(connection_hdl hdl) {
  std::cout << "[WebSocketConnection] Connection failed" << std::endl;
  std::unique_lock<std::mutex> lock(connection_mutex_);
  is_connected_ = false;
  connection_cv_.notify_all();
  
  if (!terminate_receiver_thread_) {
    ReportError(TransportError::R2C_SOCKET_ERROR);
  }
}

void SocketWebSocketConnection::on_message(connection_hdl hdl, message_ptr msg) {
  if (bson_only_mode_) {
    const std::string& payload = msg->get_payload();
    const uint8_t* data = reinterpret_cast<const uint8_t*>(payload.c_str());
    size_t length = payload.size();
    
    bson_t b;
    if (!bson_init_static(&b, data, length)) {
      std::cout << "[WebSocketConnection] Error on BSON parse - Ignoring message" << std::endl;
      return;
    }
    
    if (incoming_message_callback_bson_) {
      incoming_message_callback_bson_(b);
    }
    
    bson_destroy(&b);
  } else {
    const std::string& payload = msg->get_payload();
    json j;
    j.Parse(payload.c_str());
    
    if (j.HasParseError()) {
      std::cout << "[WebSocketConnection] JSON parse error - Ignoring message" << std::endl;
      return;
    }
    
    if (incoming_message_callback_) {
      incoming_message_callback_(j);
    }
  }
}

void SocketWebSocketConnection::ReportError(TransportError err){
  if (error_callback_ != nullptr) {
    error_callback_(err);
  }
}

}