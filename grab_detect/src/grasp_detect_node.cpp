#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>

#include "grab_detect/msg/grasp_result.hpp"
#include "grab_detect/srv/grasp_detect.hpp"

#include <arpa/inet.h>
#include <cstring>
#include <jsoncpp/json/json.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

class GraspDetectNode : public rclcpp::Node {
public:
  GraspDetectNode() : Node("grasp_detect_node") {
    // 声明参数
    this->declare_parameter("server_host", "localhost");
    this->declare_parameter("server_port", 5555);
    this->declare_parameter("socket_timeout", 30.0);
    this->declare_parameter("trigger_topic", "/grasp_detect/trigger");
    this->declare_parameter("color_image_topic", "/color/image_raw");
    this->declare_parameter("depth_image_topic", "/depth/image_raw");
    this->declare_parameter("mask_image_topic", "");
    this->declare_parameter("service_name", "grasp_detect");

    // 获取参数
    server_host_ = this->get_parameter("server_host").as_string();
    server_port_ = this->get_parameter("server_port").as_int();
    socket_timeout_ = this->get_parameter("socket_timeout").as_double();

    auto trigger_topic = this->get_parameter("trigger_topic").as_string();
    auto color_topic = this->get_parameter("color_image_topic").as_string();
    auto depth_topic = this->get_parameter("depth_image_topic").as_string();
    auto mask_topic = this->get_parameter("mask_image_topic").as_string();
    auto service_name = this->get_parameter("service_name").as_string();

    // 创建订阅者
    trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        trigger_topic, 10,
        std::bind(&GraspDetectNode::trigger_callback, this,
                  std::placeholders::_1));

    color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        color_topic, 10,
        std::bind(&GraspDetectNode::color_callback, this,
                  std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, 10,
        std::bind(&GraspDetectNode::depth_callback, this,
                  std::placeholders::_1));

    if (!mask_topic.empty()) {
      mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          mask_topic, 10,
          std::bind(&GraspDetectNode::mask_callback, this,
                    std::placeholders::_1));
    }

    // 创建服务
    service_ = this->create_service<grab_detect::srv::GraspDetect>(
        service_name, std::bind(&GraspDetectNode::service_callback, this,
                                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "GraspDetectNode initialized");
    RCLCPP_INFO(this->get_logger(), "Server: %s:%d", server_host_.c_str(),
                server_port_);
    RCLCPP_INFO(this->get_logger(), "Listening on trigger topic: %s",
                trigger_topic.c_str());
  }

private:
  void trigger_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
    RCLCPP_INFO(this->get_logger(),
                "Received trigger, processing grasp detection...");

    if (!latest_color_ || !latest_depth_) {
      RCLCPP_WARN(this->get_logger(), "Missing color or depth image!");
      return;
    }

    // 执行检测
    auto request = std::make_shared<grab_detect::srv::GraspDetect::Request>();
    request->color_image = *latest_color_;
    request->depth_image = *latest_depth_;
    if (latest_mask_) {
      request->mask_image = *latest_mask_;
    }

    auto response = std::make_shared<grab_detect::srv::GraspDetect::Response>();
    process_detection(request, response);

    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Detection successful! Found %d grasps",
                  response->num_grasps);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Detection failed: %s",
                   response->message.c_str());
    }
  }

  void color_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    latest_color_ = msg;
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    latest_depth_ = msg;
  }

  void mask_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    latest_mask_ = msg;
  }

  void service_callback(
      const std::shared_ptr<grab_detect::srv::GraspDetect::Request> request,
      std::shared_ptr<grab_detect::srv::GraspDetect::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Service called");
    process_detection(request, response);
  }

  void process_detection(
      const std::shared_ptr<grab_detect::srv::GraspDetect::Request> request,
      std::shared_ptr<grab_detect::srv::GraspDetect::Response> response) {

    int sock = -1;
    try {
      // 创建socket
      sock = socket(AF_INET, SOCK_STREAM, 0);
      if (sock < 0) {
        throw std::runtime_error("Failed to create socket");
      }

      // 设置超时
      struct timeval timeout;
      timeout.tv_sec = static_cast<int>(socket_timeout_);
      timeout.tv_usec =
          static_cast<int>((socket_timeout_ - timeout.tv_sec) * 1e6);
      setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

      // 连接服务器
      struct sockaddr_in server_addr;
      server_addr.sin_family = AF_INET;
      server_addr.sin_port = htons(server_port_);
      inet_pton(AF_INET, server_host_.c_str(), &server_addr.sin_addr);

      RCLCPP_INFO(this->get_logger(), "Connecting to %s:%d...",
                  server_host_.c_str(), server_port_);

      if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) <
          0) {
        throw std::runtime_error("Failed to connect to server");
      }

      RCLCPP_INFO(this->get_logger(), "Connected! Sending images...");

      // 发送图像数据
      send_image(sock, request->color_image);
      send_image(sock, request->depth_image);

      // 发送mask（如果有的话）
      if (request->mask_image.data.empty()) {
        uint32_t zero = 0;
        send(sock, &zero, sizeof(zero), 0);
      } else {
        send_image(sock, request->mask_image);
      }

      RCLCPP_INFO(this->get_logger(), "Images sent, waiting for results...");

      // 接收结果
      receive_results(sock, response);

      close(sock);

    } catch (const std::exception &e) {
      if (sock >= 0) {
        close(sock);
      }
      response->success = false;
      response->num_grasps = 0;
      response->message = std::string("Error: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  void send_image(int sock, const sensor_msgs::msg::Image &img) {
    // 转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, img.encoding);
    } catch (cv_bridge::Exception &e) {
      throw std::runtime_error(std::string("cv_bridge exception: ") + e.what());
    }

    // 编码为PNG
    std::vector<uint8_t> buffer;
    cv::imencode(".png", cv_ptr->image, buffer);

    // 发送大小（网络字节序）
    uint32_t size = htonl(buffer.size());
    if (send(sock, &size, sizeof(size), 0) != sizeof(size)) {
      throw std::runtime_error("Failed to send image size");
    }

    // 发送数据
    size_t sent = 0;
    while (sent < buffer.size()) {
      ssize_t n = send(sock, buffer.data() + sent, buffer.size() - sent, 0);
      if (n <= 0) {
        throw std::runtime_error("Failed to send image data");
      }
      sent += n;
    }

    RCLCPP_DEBUG(this->get_logger(), "Sent image: %zu bytes", buffer.size());
  }

  void receive_results(
      int sock,
      std::shared_ptr<grab_detect::srv::GraspDetect::Response> response) {
    // 接收结果大小
    uint32_t result_size;
    if (recv(sock, &result_size, sizeof(result_size), 0) !=
        sizeof(result_size)) {
      throw std::runtime_error("Failed to receive result size");
    }
    result_size = ntohl(result_size);

    RCLCPP_INFO(this->get_logger(), "Receiving results: %u bytes", result_size);

    // 接收结果数据
    std::vector<char> buffer(result_size);
    size_t received = 0;
    while (received < result_size) {
      ssize_t n =
          recv(sock, buffer.data() + received, result_size - received, 0);
      if (n <= 0) {
        throw std::runtime_error("Failed to receive result data");
      }
      received += n;
    }

    // 解析JSON
    Json::Value root;
    Json::CharReaderBuilder reader_builder;
    std::string errors;
    std::istringstream iss(std::string(buffer.data(), result_size));

    if (!Json::parseFromStream(reader_builder, iss, &root, &errors)) {
      throw std::runtime_error("Failed to parse JSON: " + errors);
    }

    // 填充响应
    response->success = root["success"].asBool();
    response->num_grasps = root["num_grasps"].asInt();
    response->message = root.get("message", "").asString();

    if (response->success && root.isMember("grasps")) {
      const Json::Value &grasps = root["grasps"];
      for (const auto &grasp_json : grasps) {
        grab_detect::msg::GraspResult grasp;

        grasp.score = grasp_json["score"].asDouble();
        grasp.width = grasp_json["width"].asDouble();
        grasp.height = grasp_json["height"].asDouble();
        grasp.depth = grasp_json["depth"].asDouble();

        const Json::Value &trans = grasp_json["translation"];
        for (int i = 0; i < 3; i++) {
          grasp.translation[i] = trans[i].asDouble();
        }

        const Json::Value &rot = grasp_json["rotation_matrix"];
        for (int i = 0; i < 9; i++) {
          int row = i / 3;
          int col = i % 3;
          grasp.rotation_matrix[i] = rot[row][col].asDouble();
        }

        response->grasps.push_back(grasp);
      }
    }
  }

  // 成员变量
  std::string server_host_;
  int server_port_;
  double socket_timeout_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Service<grab_detect::srv::GraspDetect>::SharedPtr service_;

  sensor_msgs::msg::Image::SharedPtr latest_color_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::Image::SharedPtr latest_mask_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspDetectNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
