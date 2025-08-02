#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <thread>
#include <atomic>
#include <string>
#include <mutex>
#include <chrono>
#include <cstring>
#include <fstream> 

using namespace std::chrono_literals;

class TcpServerNode : public rclcpp::Node
{
public:
  TcpServerNode() : Node("tcp_server_node"), running_(true), mode_(0.0), velocity_(0.0), direction_(0.0)
  {
    port_ = 1234;
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ == -1) {
      RCLCPP_FATAL(this->get_logger(), "Failed to create socket");
      rclcpp::shutdown();
      return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_);

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      RCLCPP_FATAL(this->get_logger(), "Bind failed");
      close(server_fd_);
      rclcpp::shutdown();
      return;
    }

    if (listen(server_fd_, 1) < 0) {
      RCLCPP_FATAL(this->get_logger(), "Listen failed");
      close(server_fd_);
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for ESP8266 on port %d...", port_);

    logfile_.open("esp8266_log.txt", std::ios::out | std::ios::app);
    if (!logfile_.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Failed to open log file for writing");
    }

    cmdvel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/cmdvel", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          mode_ = msg->data[0];
          direction_ = msg->data[1];
          velocity_ = msg->data[2];
        }
      });

    accept_thread_ = std::thread(&TcpServerNode::acceptConnection, this);
  }

  ~TcpServerNode()
  {
    running_ = false;
    if (conn_fd_ != -1) close(conn_fd_);
    if (server_fd_ != -1) close(server_fd_);

    if (accept_thread_.joinable()) accept_thread_.join();
    if (recv_thread_.joinable()) recv_thread_.join();
    if (send_timer_ != nullptr) send_timer_->cancel();

    if (logfile_.is_open()) {
      logfile_.close();
    }
  }

private:
  void acceptConnection()
  {
    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);

    conn_fd_ = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
    if (conn_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Accept failed");
      return;
    }

    char client_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
    RCLCPP_INFO(this->get_logger(), "Connected by %s:%d", client_ip, ntohs(client_addr.sin_port));

    recv_thread_ = std::thread(&TcpServerNode::receiveData, this);

    send_timer_ = this->create_wall_timer(
      100ms, std::bind(&TcpServerNode::sendDataToESP, this));
  }

  void receiveData()
  {
    char buffer[1024];
    while (running_) {
      ssize_t bytes_received = recv(conn_fd_, buffer, sizeof(buffer) - 1, 0);
      if (bytes_received > 0) {
        buffer[bytes_received] = '\0';
        RCLCPP_INFO(this->get_logger(), "ESP8266 says: %s", buffer);
        if (logfile_.is_open()) {
          logfile_ << "[INFO] ESP8266 says: " << buffer << std::endl;
        }
      } else if (bytes_received == 0) {
        RCLCPP_WARN(this->get_logger(), "Connection closed by ESP8266");
        break;
      } else {
        RCLCPP_ERROR(this->get_logger(), "recv error");
        break;
      }
    }
  }

  void sendDataToESP()
  {
    if (conn_fd_ == -1) return;

    double mode, direction, velocity;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      mode = mode_;
      direction = direction_;
      velocity = velocity_;
    }

    char message[100];
    snprintf(message, sizeof(message), "mode=%.0f,direction=%.0f,velocity=%.0f\n", mode, direction, velocity);

    ssize_t bytes_sent = send(conn_fd_, message, strlen(message), 0);
    if (bytes_sent < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to send data to ESP8266");
    }
  }

  int server_fd_{-1};
  int conn_fd_{-1};
  int port_;

  std::thread accept_thread_;
  std::thread recv_thread_;
  rclcpp::TimerBase::SharedPtr send_timer_;

  std::atomic<bool> running_;

  std::mutex data_mutex_;
  double mode_;
  double direction_;
  double velocity_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmdvel_sub_;

  std::ofstream logfile_; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TcpServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
