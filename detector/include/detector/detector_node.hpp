#ifndef DETECTOR_NODE_HPP
#define DETECTOR_NODE_HPP
// # define USING_YOLOV8_car
#define USING_LW_DETR_CAR
#define USING_LW_DETR_ARMOR

#include "detector/yolov8.hpp"
#include "detector/msg/cars.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "detector/lw_detr.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <sstream>

namespace detector {
struct ROI {
  int x = 0;
  int y = 0;
  int w = 640;
  int h = 640;
};

enum detector_type { CAR, ARMOR };

class detector_node : public rclcpp::Node {

// car
//  YOLOv8推理对象
#ifdef USING_YOLOV8_CAR
  std::unique_ptr<YoloV8> detector_car_;
#endif
// LwDetr推理对象
#ifdef USING_LW_DETR_CAR
  std::unique_ptr<LwDetr> detector_car_;
#endif

// armor
// YOLOv8推理对象
#ifdef USING_YOLOV8_ARMOR
  std::unique_ptr<YoloV8> detector_armor_;
#endif
// LwDetr推理对象
#ifdef USING_LW_DETR_ARMOR
  std::unique_ptr<LwDetr> detector_armor_;
#endif

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_detector_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_armor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_car_pub_;
  rclcpp::Publisher<msg::Cars>::SharedPtr cars_with_armor_pub;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      rosbag_img_sub;

  float nms_threshold_; // NMS阈值

  double confidence_threshold_;
  bool rosbag_flag_;
  bool show_image_detector_;
  bool show_image_armor_;
  bool show_image_car_;
  bool ROI_open_;
  bool debug_img_clone_ = false;

  int ROI_point_x_;
  int ROI_point_y_;
  int num_ROI_;
  int ROI_overlap_;

  rclcpp::Publisher<msg::Cars>::SharedPtr
      cars_without_armor_pub;
  cv::Mat img_;
  std::vector<std::vector<cv::Mat>> img_ROI_;

  std::vector<ROI> rois_;
  std::string model_path_car_;
  std::string plugin_path_car_;
  std::string model_path_armor_;
  std::string plugin_path_armor_;

  bool flag = true;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::chrono::duration<double, std::micro> total_ms_img_clone;

public:
  explicit detector_node(const rclcpp::NodeOptions &options);
  ~detector_node() = default;

private:
  void rosbag_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void adjustROIPosition();

  std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
  detect_car();

  std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
  post_precess(std::vector<std::vector<float>> boxes,
               std::vector<std::vector<float>> scores,
               std::vector<std::vector<int>> classID);

  std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
  non_max_suppression(const std::vector<float> &boxes,
                      std::vector<float> scores, std::vector<int> classID,
                      float threshold);

  cv::Mat showROI();
  float bbox_iou(const cv::Rect &box1, const cv::Rect &box2);
};

} // namespace detector

#endif // DETECTOR_NODE_HPP