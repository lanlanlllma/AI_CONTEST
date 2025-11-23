#include "detector_wjr/detector_node_wjr.hpp"

detector_wjr::detector_node_wjr::detector_node_wjr(const rclcpp::NodeOptions &options)
    : Node("detector_node_wjr", options) {
  RCLCPP_INFO(this->get_logger(), "detector_node_wjr start");
// 声明参数
#ifdef USING_YOLOV8_CAR
  this->declare_parameter("model_path_car",
                          "/home/phoenix/roboarm/FairinoDualArm/"
                          "src/detector/asserts/best_l.enging");
  this->declare_parameter(
      "plugin_path_car",
      "/home/phoenix/tensorrtx/yolov8/build/libmyplugins.so");
#endif
#ifdef USING_LW_DETR_CAR
  this->declare_parameter("model_path_car",
                          "/home/ma/roboarm/LW-DETR/output/lwdetr_medium_coco/inference_model.engine");
  this->declare_parameter("plugin_path_car",
                          "/home/phoenix/roboarm/FairinoDualArm/src/detector/"
                          "asserts/layernorm_plugin.so");
#endif

  this->declare_parameter("confidence_threshold", 0.65);
  this->declare_parameter("nms_threshold", 0.4);

  this->declare_parameter("image_input_topic", "/camera/color/image_raw");
  this->declare_parameter("image_car_topic", "/detector/detections/iamge");
  this->declare_parameter("detections_topic", "/detections");

  this->declare_parameter("show_pic_detector", true);
  this->declare_parameter("show_pic_armor", true);
  this->declare_parameter("show_pic_car", true);
  this->declare_parameter("rosbag_flag", true);
  this->declare_parameter("ROI_open", false);

  this->declare_parameter("ROI_point_x", 0);
  this->declare_parameter("ROI_point_y", 1150);
  this->declare_parameter("num_ROI", 6);
  this->declare_parameter("ROI_overlap", 150);

  // 获取参数
  model_path_car_ = this->get_parameter("model_path_car").as_string();
  plugin_path_car_ = this->get_parameter("plugin_path_car").as_string();
  confidence_threshold_ =
      this->get_parameter("confidence_threshold").as_double();
  nms_threshold_ = this->get_parameter("nms_threshold").as_double();

  std::string image_input_topic =
      this->get_parameter("image_input_topic").as_string();
  std::string image_car_topic =
      this->get_parameter("image_car_topic").as_string();
  std::string detections_topic =
      this->get_parameter("detections_topic").as_string();

  show_image_detector_ = this->get_parameter("show_pic_detector").as_bool();
  show_image_armor_ = this->get_parameter("show_pic_armor").as_bool();
  rosbag_flag_ = this->get_parameter("rosbag_flag").as_bool();
  show_image_car_ = this->get_parameter("show_pic_car").as_bool();
  ROI_open_ = this->get_parameter("ROI_open").as_bool();

  ROI_point_x_ = this->get_parameter("ROI_point_x").as_int();
  ROI_point_y_ = this->get_parameter("ROI_point_y").as_int();
  num_ROI_ = this->get_parameter("num_ROI").as_int();
  ROI_overlap_ = this->get_parameter("ROI_overlap").as_int();

//  初始化推理对象 car
#ifdef USING_YOLOV8_CAR
  detector_car_ = std::make_unique<detector_wjr::YoloV8>();
  int ret_car = detector_car_->init(model_path_car_, plugin_path_car_);
  if (ret_car != 0) {
    RCLCPP_ERROR(this->get_logger(), "car的YOLOv8初始化失败，错误码: %d",
                 ret_car);
    throw std::runtime_error("car的YOLOv8初始化失败");
  }
  RCLCPP_INFO(this->get_logger(), "car的YOLOv8初始化成功，模型路径: %s",
              model_path_car_.c_str());
  RCLCPP_INFO(this->get_logger(), "car模型初始化完成，正在加载armor模型...");
#endif
#ifdef USING_LW_DETR_CAR
  detector_car_ = std::make_unique<detector_wjr::LwDetr>();
  int ret_car = detector_car_->init(model_path_car_);
  if (ret_car != 0) {
    RCLCPP_ERROR(this->get_logger(), "car的LwDetr初始化失败，错误码: %d",
                 ret_car);
    throw std::runtime_error("car的LwDetr初始化失败");
  }
  RCLCPP_INFO(this->get_logger(), "car的LwDetr初始化成功，模型路径: %s",
              model_path_car_.c_str());
  RCLCPP_INFO(this->get_logger(), "car模型初始化完成，正在加载armor模型...");
#endif

  // 参数监控回调
  param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        for (const auto &param : params) {
          RCLCPP_INFO(this->get_logger(), "参数 [%s] 已修改，新值 = %s",
                      param.get_name().c_str(),
                      param.value_to_string().c_str());
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

  // 打印所有参数初始值
  auto param_list = this->list_parameters({}, 10);
  for (const auto &name : param_list.names) {
    auto value = this->get_parameter(name);
    RCLCPP_INFO(this->get_logger(), "[参数初始化] %s = %s", name.c_str(),
                value.value_to_string().c_str());
  }

  // 订阅图像
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_input_topic, rclcpp::SensorDataQoS(),
      std::bind(&detector_node_wjr::image_callback, this, std::placeholders::_1));

  //发布图像
  // cars_with_armor_pub =
  //     this->create_publisher<lidar_interfaces::msg::Cars>(detections_topic,
  //     10);

  img_car_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>(image_car_topic, 10);
}

void detector_wjr::detector_node_wjr::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  auto total_start = std::chrono::high_resolution_clock::now();
  // 输入验证
  if (msg->data.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Empty image data");
    return;
  }

  // 图像转换
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    img_ = cv_ptr->image;
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (img_.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Received image is empty, skipping detection.");
    return;
  }

  cv::Mat img_for_car;
  img_for_car = img_.clone();

  // 车辆检测
  std::vector<cv::Mat> raw_image_generator = {img_for_car};
  auto car_result = detector_car_->infer(raw_image_generator, flag);

  // 解析车辆检测结果
  cv::Mat processed_img;
  double car_duration;
  std::vector<float> car_boxes;
  std::vector<float> car_scores; //置信度
  std::vector<int> car_classIDs;
  try {
    processed_img = std::get<0>(car_result)[0]; // batch_size = 1
    car_duration = std::get<1>(car_result);
    car_boxes = std::get<2>(car_result)[0];  // batch_size = 1
    car_scores = std::get<3>(car_result)[0]; // batch_size = 1
    car_classIDs = std::get<4>(car_result)[0]; // batch_size = 1
    RCLCPP_DEBUG(this->get_logger(), "Car detection time: %f s", car_duration);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to parse car detection results: %s", e.what());
    return;
  }

  // 检查是否有车辆检测结果
  if (car_boxes.empty() || car_boxes.size() % 4 != 0) {
    RCLCPP_INFO(this->get_logger(), "No car detection result");
    
    // Cars.is_empty = true;
    // cars_with_armor_pub->publish(Cars);
    // if (get_parameter("show_pic_car").as_bool()) {
    //   //准备消息
    //   std_msgs::msg::Header header;
    //   header.stamp = this->now();
    //   header.frame_id = "camera";

    //   auto img_to_send =
    //       cv_bridge::CvImage(header, "bgr8", img_.clone()).toImageMsg();
    //   img_car_pub_->publish(*img_to_send);
    // }
    
    return;
  }

  // 处理每个检测到的车辆
  size_t car_count = car_boxes.size() / 4;
  std::cout << "car number: " << car_count << std::endl;

  // 发布车辆图像
  if (get_parameter("show_pic_car").as_bool()) {
    cv::Mat img_for_public_ = img_.clone();

    for (int i = 0; i < car_count; i++) {
      // 提取车辆边界框
      float x1 = car_boxes[i * 4 + 0];
      float y1 = car_boxes[i * 4 + 1];
      float x2 = car_boxes[i * 4 + 2];
      float y2 = car_boxes[i * 4 + 3];

      cv::Rect rect(x1, y1, x2 - x1, y2 - y1);
      if(car_classIDs[i]==0){
        // 绘制边界框（绿色表示车辆）
        cv::rectangle(img_for_public_, rect, cv::Scalar(0, 255, 0), 2);
      }else if(car_classIDs[i]==1){
        // 绘制边界框（蓝色表示脏）
        cv::rectangle(img_for_public_, rect, cv::Scalar(255, 0, 0), 2);
      }

      //左上角显示文字
      std::ostringstream oss;
      oss << "SCORE:" << std::fixed << std::setprecision(2) << car_scores[i];
      std::string label = oss.str();
      int font = cv::FONT_HERSHEY_SIMPLEX;
      double scale = 0.7;
      int thickness = 2;
      if(car_classIDs[i]==0){
        // 绘制边界框（绿色表示车辆）
        cv::putText(img_for_public_, label, cv::Point(x1 + 5, y1 + 25), font,
                  scale, cv::Scalar(0, 255, 0), thickness);
      }else if(car_classIDs[i]==1){
        // 绘制边界框（蓝色表示脏）
        cv::putText(img_for_public_, label, cv::Point(x1 + 5, y1 + 25), font,
                  scale, cv::Scalar(255, 0, 0), thickness);
      }
    }

    //准备消息
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera";

    auto img_to_send =
        cv_bridge::CvImage(header, "bgr8", img_for_public_).toImageMsg();
    img_car_pub_->publish(*img_to_send);
  }

  
    // // 设置结果状态
    // Cars.is_empty = Cars.cars.empty();
    // cars_with_armor_pub->publish(Cars);ss

    // //显示车辆及装甲板结果
    // // cars消息=car数组( xyxy + armor(id+score+xyxy) + is_empty )  +  header +
    // // is_empty
  

  //总时间
  auto total_end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      total_end - total_start);
  double total_ms_detection_double = duration.count();
  RCLCPP_INFO(this->get_logger(), "total detection time: %.3f ms",
              total_ms_detection_double);
};

// 注册组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detector_wjr::detector_node_wjr)