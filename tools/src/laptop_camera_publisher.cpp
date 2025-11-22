#include <chrono>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace utils {

class LaptopCameraPublisher : public rclcpp::Node {
public:
  LaptopCameraPublisher()
      : Node("laptop_camera_publisher"),
        camera_index_(declare_parameter<int>("camera_index", 0)),
        topic_name_(declare_parameter<std::string>("topic_name", "/camera/color/image_raw")),
        frame_id_(declare_parameter<std::string>("frame_id", "camera_color_frame")),
        publish_rate_(declare_parameter<double>("publish_rate", 30.0)),
        show_window_(declare_parameter<bool>("show_window", true)),
        window_name_(declare_parameter<std::string>("window_name", "Laptop Camera")) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_name_, qos);

    if (!capture_.open(camera_index_)) {
      RCLCPP_FATAL(get_logger(), "Failed to open camera index %d", camera_index_);
      throw std::runtime_error("Failed to open camera");
    }

    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, publish_rate_)));
    timer_ = create_wall_timer(period, std::bind(&LaptopCameraPublisher::capture_and_publish, this));

    RCLCPP_INFO(get_logger(), "Publishing laptop camera frames to %s", topic_name_.c_str());
    if (show_window_) {
      cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    }
  }

  ~LaptopCameraPublisher() override {
    timer_.reset();
    capture_.release();
    if (show_window_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  void capture_and_publish() {
    cv::Mat frame;
    if (!capture_.read(frame)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read frame from camera");
      return;
    }

    auto message = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
    message->header.stamp = now();
    message->header.frame_id = frame_id_;
    publisher_->publish(*message);

    if (show_window_) {
      cv::imshow(window_name_, frame);
      cv::waitKey(1);
    }
  }

  int camera_index_;
  std::string topic_name_;
  std::string frame_id_;
  double publish_rate_;
  bool show_window_;
  std::string window_name_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture capture_;
};

} // namespace utils

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<utils::LaptopCameraPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("laptop_camera_publisher"), "Unhandled exception: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
