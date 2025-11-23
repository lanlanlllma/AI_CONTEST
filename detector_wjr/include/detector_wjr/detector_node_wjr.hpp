#ifndef DETECTOR_NODE_HPP
#define DETECTOR_NODE_HPP
// # define USING_YOLOV8_car
#define USING_LW_DETR_CAR
// #define USING_YOLOV8_ARMOR

#include "detector_wjr/yolov8.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <iomanip>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "detector_wjr/lw_detr.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <sstream>

namespace detector_wjr {

enum detector_type { CAR, ARMOR };

class detector_node_wjr : public rclcpp::Node {

// car
//  YOLOv8推理对象
#ifdef USING_YOLOV8_CAR
  std::unique_ptr<YoloV8> detector_car_;
#endif
// LwDetr推理对象
#ifdef USING_LW_DETR_CAR
  std::unique_ptr<LwDetr> detector_car_;
#endif

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_car_pub_;
  // rclcpp::Publisher<lidar_interfaces::msg::Cars>::SharedPtr cars_with_armor_pub;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      rosbag_img_sub;

  float nms_threshold_; // NMS阈值

  double confidence_threshold_;
  bool rosbag_flag_;
  bool show_image_detector_;
  bool show_image_armor_;
  bool show_image_car_;
  bool ROI_open_;
  bool debug_img_clone_ = true;

  int ROI_point_x_;
  int ROI_point_y_;
  int num_ROI_;
  int ROI_overlap_;

  // rclcpp::Publisher<lidar_interfaces::msg::Cars>::SharedPtr
  //     cars_without_armor_pub;
  cv::Mat img_;

  std::string model_path_car_;
  std::string plugin_path_car_;

  bool flag = true;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::chrono::duration<double, std::micro> total_ms_img_clone;

public:
  explicit detector_node_wjr(const rclcpp::NodeOptions &options);
  ~detector_node_wjr() = default;

private:
  void rosbag_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void adjustROIPosition();

  std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
  detect_defect();

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

} // namespace detector_wjr

#endif // DETECTOR_NODE_HPP
