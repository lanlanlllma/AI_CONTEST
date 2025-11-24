#include "detector/detector_node.hpp"

detector::detector_node::detector_node(const rclcpp::NodeOptions &options)
    : Node("detector_node", options) {
  RCLCPP_INFO(this->get_logger(), "detector_node start");
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
  this->declare_parameter(
      "model_path_car",
      "/home/phoenix/ws_AImatch/model/lwdetr_medium_coco_1119/"
      "inference_model.engine"); // /home/phoenix/Desktop/lwdetr_medium_coco_0729/inference_model.engine
  this->declare_parameter("plugin_path_car",
                          "/home/phoenix/roboarm/FairinoDualArm/src/detector/"
                          "asserts/layernorm_plugin.so");
#endif
#ifdef USING_YOLOV8_ARMOR
  this->declare_parameter(
      "model_path_armor",
      "/home/phoenix/tensorrtx/yolov8/build/yolov8s.engine");
  this->declare_parameter(
      "plugin_path_armor",
      "/home/phoenix/tensorrtx/yolov8/build/libmyplugins.so");
#endif
#ifdef USING_LW_DETR_ARMOR
  this->declare_parameter("model_path_armor",
                          "/home/phoenix/ws_AImatch/model/"
                          "lwdetr_medium_coco_1015/inference_model.engine");
  this->declare_parameter("plugin_path_armor",
                          "/home/phoenix/roboarm/FairinoDualArm/src/detector/"
                          "asserts/layernorm_plugin.so");
#endif

  this->declare_parameter("confidence_threshold", 0.65);
  this->declare_parameter("nms_threshold", 0.4);

  this->declare_parameter("image_input_topic", "/image_for_radar");
  this->declare_parameter("image_car_topic", "/detector/img_car");
  this->declare_parameter("image_detector_topic", "/detector/img_detect");
  this->declare_parameter("image_armor_topic", "/detector/img_armor");
  this->declare_parameter("detections_topic", "/detections");

  this->declare_parameter("show_pic_detector", true);
  this->declare_parameter("show_pic_armor", false);
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
  model_path_armor_ = this->get_parameter("model_path_armor").as_string();
  plugin_path_armor_ = this->get_parameter("plugin_path_armor").as_string();
  confidence_threshold_ =
      this->get_parameter("confidence_threshold").as_double();
  nms_threshold_ = this->get_parameter("nms_threshold").as_double();

  std::string image_input_topic =
      this->get_parameter("image_input_topic").as_string();
  std::string image_car_topic =
      this->get_parameter("image_car_topic").as_string();
  std::string image_detector_topic =
      this->get_parameter("image_detector_topic").as_string();
  std::string image_armor_topic =
      this->get_parameter("image_armor_topic").as_string();
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
  detector_car_ = std::make_unique<detector::YoloV8>();
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
  detector_car_ = std::make_unique<detector::LwDetr>();
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

//   初始化推理对象 armor
#ifdef USING_YOLOV8_ARMOR
  detector_armor_ = std::make_unique<detector::YoloV8>();
  int ret_armor = detector_armor_->init(model_path_armor_, plugin_path_armor_);
  if (ret_armor != 0) {
    RCLCPP_ERROR(this->get_logger(), "armor的YOLOv8初始化失败，错误码: %d",
                 ret_armor);
    throw std::runtime_error("armor的YOLOv8初始化失败");
  }
  RCLCPP_INFO(this->get_logger(), "armor的YOLOv8初始化成功，模型路径: %s",
              model_path_armor_.c_str());
#endif
#ifdef USING_LW_DETR_ARMOR
  detector_armor_ = std::make_unique<detector::LwDetr>();
  int ret_armor = detector_armor_->init(model_path_armor_);
  if (ret_armor != 0) {
    RCLCPP_ERROR(this->get_logger(), "armor的LwDetr初始化失败，错误码: %d",
                 ret_armor);
    throw std::runtime_error("armor的LwDetr初始化失败");
  }
  RCLCPP_INFO(this->get_logger(), "armor的LwDetr初始化成功，模型路径: %s",
              model_path_armor_.c_str());
#endif

  //初始化ROI
  rois_ = std::vector<ROI>(num_ROI_);

  if (ROI_open_) {
    RCLCPP_INFO(this->get_logger(), "ROI is open");
    for (int i = 0; i < num_ROI_; i++) {
      rois_[i] =
          ROI{ROI_point_x_ + i * (640 - ROI_overlap_), ROI_point_y_, 640, 640};
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "ROI is closed");
    num_ROI_ = 0;
    for (int i = 0; i < num_ROI_; i++) {
      rois_[i] = ROI{0, 0, 320, 320};
    }
  }

  // 参数监控回调
  param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        for (const auto &param : params) {
          RCLCPP_INFO(this->get_logger(), "参数 [%s] 已修改，新值 = %s",
                      param.get_name().c_str(),
                      param.value_to_string().c_str());

          // 如果是 ROI_point_x 或 y，顺便更新 ROI
          if (param.get_name() == "ROI_point_x") {
            ROI_point_x_ = param.as_int();
          } else if (param.get_name() == "ROI_point_y") {
            ROI_point_y_ = param.as_int();
          } else if (param.get_name() == "num_ROI") {
            num_ROI_ = param.as_int();
            rois_.resize(num_ROI_);
          } else if (param.get_name() == "ROI_overlap") {
            ROI_overlap_ = param.as_int();
          } else if (param.get_name() == "ROI_open") {
            ROI_open_ = param.as_bool();
            num_ROI_ = ROI_open_ ? this->get_parameter("num_ROI").as_int() : 0;
            if (ROI_open_) {
              // 初始化ROI
              rois_.clear();
              rois_ = std::vector<ROI>(num_ROI_);
            } else {
              rois_.clear();
            }
          }
        }

        // 统一更新 ROI（防止参数变动后ROI不刷新）
        for (int i = 0; i < num_ROI_; i++) {
          rois_[i].x = ROI_point_x_ + i * (640 - ROI_overlap_);
          rois_[i].y = ROI_point_y_;
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
      std::bind(&detector_node::image_callback, this, std::placeholders::_1));

  //发布图像
  cars_with_armor_pub =
      this->create_publisher<msg::Cars>(detections_topic, 10);
  img_car_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>(image_car_topic, 10);

  img_detector_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>(image_detector_topic, 10);

  img_armor_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>(image_armor_topic, 10);
};

void detector::detector_node::rosbag_callback(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "compressed_image");
  // trans into sensor_msgs::msg::Image::SharedPtr
  auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
  img_msg->header = msg->header;
  auto img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  img_msg->height = img->image.rows;
  img_msg->width = img->image.cols;
  img_msg->encoding = "bgr8";
  img_msg->is_bigendian = 0;
  img_msg->step = img->image.step;
  img_msg->data = std::vector<uint8_t>(
      img->image.data,
      img->image.data + img->image.total() * img->image.channels());
  image_callback(img_msg);
};

cv::Mat detector::detector_node::showROI() {

  const int num_rois = rois_.size();
  const int max_x = img_.cols - 640;
  const int max_y = img_.rows - 640;

  cv::Mat preview;
  if (debug_img_clone_) {
    auto start_img = std::chrono::high_resolution_clock::now();
    preview = img_.clone();
    auto end_img = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_img - start_img);
    total_ms_img_clone += duration;
  } else {
    preview = img_.clone();
  }

  //显示ROI
  for (int i = 0; i < num_rois; ++i) {
    int x = std::min(rois_[i].x, max_x);
    int y = std::min(rois_[i].y, max_y);

    cv::Rect rect(x, y, 640, 640);

    cv::Scalar color;
    switch (i % 2) {
    case 0:
      color = cv::Scalar(255, 0, 0);
      break;
    case 1:
      color = cv::Scalar(0, 0, 255);
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "ROI number is wrong!");
      break;
    }

    cv::rectangle(preview, rect, color, 2);

    //左上角显示文字
    std::string label = "ROI " + std::to_string(i + 1);
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.7;
    int thickness = 2;
    cv::putText(preview, label, cv::Point(x + 5, y + 25), font, scale, color,
                thickness);
  }

  return preview;
}

void detector::detector_node::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  auto total_start = std::chrono::high_resolution_clock::now();
  if (debug_img_clone_) {
    total_ms_img_clone = std::chrono::duration<double, std::micro>(0);
  }
  // 输入验证
  if (msg->data.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Empty image data");
    return;
  }

  // 准备消息
  auto Cars = msg::Cars(); // cars消息=car数组( xyxy +
                                             // armor(id+score+xyxy) + is_empty
                                             // )  +  header  +  is_empty
  Cars.cars.clear();
  if (rosbag_flag_) {
    Cars.header.stamp = this->now();
  } else {
    Cars.header = msg->header;
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

  // 缩小图像？？？

  cv::Mat processed_img; // for armor
  if (debug_img_clone_) {
    auto start_img = std::chrono::high_resolution_clock::now();
    processed_img = img_.clone();
    auto end_img = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_img - start_img);
    total_ms_img_clone += duration;
  } else {
    processed_img = img_.clone();
  }

  cv::Mat img_for_car;
  if (debug_img_clone_) {
    auto start_img = std::chrono::high_resolution_clock::now();
    img_for_car = img_.clone();
    auto end_img = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_img - start_img);
    total_ms_img_clone += duration;
  } else {
    img_for_car = img_.clone();
  }

  // 车辆检测
  std::vector<float> car_boxes, car_scores;
  std::vector<int> car_ids;
  try {

    std::tie(car_boxes, car_scores, car_ids) = detect_car();

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to parse car detection results: %s", e.what());
    return;
  }

  // 检查是否有车辆检测结果
  if (car_boxes.empty() || car_boxes.size() % 4 != 0) {
    RCLCPP_DEBUG(this->get_logger(), "No car detection result");
    Cars.is_empty = true;
    cars_with_armor_pub->publish(Cars);
    if (get_parameter("show_pic_car").as_bool()) {
      //准备消息
      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = "camera";

      auto img_to_send =
          cv_bridge::CvImage(header, "bgr8", img_.clone()).toImageMsg();
      img_car_pub_->publish(*img_to_send);
    }
    return;
  }

  // 处理每个检测到的车辆
  size_t car_count = car_boxes.size() / 4;
  RCLCPP_INFO(this->get_logger(), "car number: %ld", car_count);

  // 发布车辆图像
  if (get_parameter("show_pic_car").as_bool()) {
    cv::Mat img_ROI = showROI();

    for (int i = 0; i < car_count; i++) {
      // 提取车辆边界框
      float x1 = car_boxes[i * 4 + 0];
      float y1 = car_boxes[i * 4 + 1];
      float x2 = car_boxes[i * 4 + 2];
      float y2 = car_boxes[i * 4 + 3];

      cv::Rect rect(x1, y1, x2 - x1, y2 - y1);
      cv::rectangle(img_ROI, rect, cv::Scalar(0, 255, 0), 2);

      //左上角显示文字
      std::ostringstream oss;
      oss << "SCORE:" << std::fixed << std::setprecision(2) << car_scores[i];
      std::string label = oss.str();
      int font = cv::FONT_HERSHEY_SIMPLEX;
      double scale = 0.7;
      int thickness = 2;
      cv::putText(img_ROI, label, cv::Point(x1 + 5, y1 + 25), font, scale,
                  cv::Scalar(0, 255, 0), thickness);
    }

    //准备消息
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera";

    auto img_to_send = cv_bridge::CvImage(header, "bgr8", img_ROI).toImageMsg();
    img_car_pub_->publish(*img_to_send);
  }

  // 装甲板检测 采用串行 由于单个车辆图像较小，推理时间较短，串行开销较小
  // 线程池？？
  auto start_armor = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < car_count; i++) {
    // 提取车辆边界框
    float x1 = car_boxes[i * 4 + 0];
    float y1 = car_boxes[i * 4 + 1];
    float x2 = car_boxes[i * 4 + 2];
    float y2 = car_boxes[i * 4 + 3];

    // 裁剪装甲板图像
    cv::Mat armor_img =
        processed_img(cv::Rect(x1, y1, x2 - x1, y2 - y1)).clone();

    RCLCPP_DEBUG(this->get_logger(), "开始 armor 检测");

    // 装甲板检测
    std::vector<cv::Mat> armor_image_generator = {armor_img};
    auto armor_result = detector_armor_->infer(armor_image_generator, false);

    // 解析装甲板检测结果
    std::vector<float> armor_boxes;
    std::vector<float> armor_scores;
    std::vector<int> armor_classID;

    try {
      armor_boxes = std::get<2>(armor_result)[0];   // batch_size = 1
      armor_scores = std::get<3>(armor_result)[0];  // batch_size = 1
      armor_classID = std::get<4>(armor_result)[0]; // batch_size = 1
      RCLCPP_DEBUG(this->get_logger(), "Armor detection time: %f ms",
                   std::get<1>(armor_result));
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Error processing armor detection: %s",
                  e.what());
      continue;
    }

    // 检查是否有装甲板检测结果(一般会在try里报错，这里是防止万一)
    if (armor_boxes.empty() || armor_scores.empty() || armor_classID.empty()) {
      continue;
    }

    // 找到最高分数的装甲板
    float max_score = -1;
    size_t max_index = 0;
    for (size_t j = 0; j < armor_scores.size(); j++) {
      if (armor_scores[j] > max_score) {
        max_score = armor_scores[j];
        max_index = j;
      }
    }

    // 创建车辆消息
    auto car_msg = msg::Car();
    auto armor_msg = msg::Armor();

    // 设置装甲板信息
    armor_msg.id = armor_classID[max_index];
    armor_msg.score = armor_scores[max_index];

    // 将装甲板坐标转换回原图坐标系
    float armor_x1 = armor_boxes[max_index * 4 + 0] + x1;
    float armor_y1 = armor_boxes[max_index * 4 + 1] + y1;
    float armor_x2 = armor_boxes[max_index * 4 + 2] + x1;
    float armor_y2 = armor_boxes[max_index * 4 + 3] + y1;

    armor_msg.bbox =
        std::array<float, 4>{armor_x1, armor_y1, armor_x2, armor_y2};
    car_msg.armor = armor_msg;

    // 设置车辆边界框
    car_msg.xyxy = std::array<float, 4>{x1, y1, x2, y2};
    car_msg.is_empty = false;

    Cars.cars.emplace_back(car_msg);
  }

  RCLCPP_DEBUG(this->get_logger(), "get armor result done");
  auto end_armor = std::chrono::high_resolution_clock::now();
  auto duration_armor = std::chrono::duration_cast<std::chrono::microseconds>(
      end_armor - start_armor);
  double total_ms_armor_double = duration_armor.count() / 1000.0;
  RCLCPP_DEBUG(this->get_logger(), "total armor detection time: %.3f ms",
               total_ms_armor_double);

  // 设置结果状态
  Cars.is_empty = Cars.cars.empty();
  cars_with_armor_pub->publish(Cars);

  // 显示装甲板结果
  if (get_parameter("show_pic_armor").as_bool()) {
    cv::Mat display_img = processed_img.clone();

    for (const auto &car : Cars.cars) {
      // 绘制装甲板框
      cv::rectangle(display_img,
                    cv::Point(car.armor.bbox[0], car.armor.bbox[1]),
                    cv::Point(car.armor.bbox[2], car.armor.bbox[3]),
                    cv::Scalar(0, 255, 0), 2);

      // 添加标签
      std::string label = "ID: " + std::to_string(car.armor.id) +
                          " Score: " + std::to_string(car.armor.score);
      cv::putText(display_img, label,
                  cv::Point(car.armor.bbox[0], car.armor.bbox[1] - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
    }

    cv::resize(display_img, display_img, cv::Size(1920, 1080));
    auto img_to_send =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", display_img)
            .toImageMsg();
    img_to_send->header.frame_id = "camera";
    img_armor_pub_->publish(*img_to_send);
  }

  //显示车辆及装甲板结果
  // cars消息=car数组( xyxy + armor(id+score+xyxy) + is_empty )  +  header  +
  // is_empty
  if (get_parameter("show_pic_detector").as_bool()) {
    cv::Mat display_img = processed_img.clone();

    for (const auto &car : Cars.cars) {
      // 绘制车辆框
      cv::rectangle(display_img, cv::Point(car.xyxy[0], car.xyxy[1]),
                    cv::Point(car.xyxy[2], car.xyxy[3]), cv::Scalar(255, 0, 0),
                    2);

      if (car.armor.id == -1)
        continue;

      // 绘制装甲板框
      cv::rectangle(display_img,
                    cv::Point(car.armor.bbox[0], car.armor.bbox[1]),
                    cv::Point(car.armor.bbox[2], car.armor.bbox[3]),
                    cv::Scalar(0, 255, 0), 2);

      // 添加标签
      std::string label = "ID: " + std::to_string(car.armor.id) +
                          " Score: " + std::to_string(car.armor.score);
      cv::putText(display_img, label,
                  cv::Point(car.armor.bbox[0], car.armor.bbox[1] - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
    }

    cv::resize(display_img, display_img, cv::Size(1920, 1080));
    auto img_to_send =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", display_img)
            .toImageMsg();
    img_to_send->header.frame_id = "camera";
    img_detector_pub_->publish(*img_to_send);
  }

  //总时间
  auto total_end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      total_end - total_start);
  if (debug_img_clone_) {
    double total_ms_img_clone_double = total_ms_img_clone.count() / 1000.0;
    RCLCPP_DEBUG(this->get_logger(), "total img_clone time: %.3f ms",
                 total_ms_img_clone_double);
  }
  double total_ms_detection_double = duration.count() / 1000.0;
  RCLCPP_INFO(this->get_logger(), "total detection time: %.3f ms",
              total_ms_detection_double);
};

std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
detector::detector_node::detect_car() {
  auto start = std::chrono::high_resolution_clock::now();
  int count_context_ = num_ROI_ + 1; // 计数上下文
  cv::Mat img_copy;
  if (debug_img_clone_) {
    auto start_img = std::chrono::high_resolution_clock::now();
    img_copy = img_.clone();
    auto end_img = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_img - start_img);
    total_ms_img_clone += duration;
  } else {
    img_copy = img_.clone();
  }

  std::vector<std::vector<cv::Mat>> input;

  // 提取ROI
  for (int i = 0; i < num_ROI_; i++) {
    // 边界检查，防止越界
    rois_[i].x = std::min(std::max(rois_[i].x, 0), img_.cols - rois_[i].w);
    rois_[i].y = std::min(std::max(rois_[i].y, 0), img_.rows - rois_[i].h);
    std::vector<cv::Mat> input_img;
    if (debug_img_clone_) {
      auto start_img = std::chrono::high_resolution_clock::now();
      input_img.push_back(
          img_(cv::Rect(rois_[i].x, rois_[i].y, rois_[i].w, rois_[i].h))
              .clone());
      auto end_img = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
          end_img - start_img);
      total_ms_img_clone += duration;
    } else {
      input_img.push_back(
          img_(cv::Rect(rois_[i].x, rois_[i].y, rois_[i].w, rois_[i].h))
              .clone());
    }

    input.push_back(input_img);
  }

  // 原图
  std::vector<cv::Mat> input_img;
  if (debug_img_clone_) {
    auto start_img = std::chrono::high_resolution_clock::now();
    input_img.push_back(img_copy.clone());
    auto end_img = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_img - start_img);
    total_ms_img_clone += duration;
  } else {
    input_img.push_back(img_copy.clone());
  }
  input.push_back(input_img);

  std::vector<bool> is_raw_image(input.size(), false); // 标记是否为原图

  // input重整
  for (int i = 0; i < input.size(); i++) {
    if (input[i].size() > 1) {
      input[i].resize(1);
    }
    if (input[i].empty() && i > 0) {
      is_raw_image[i] = true;
      if (debug_img_clone_) {
        auto start_img = std::chrono::high_resolution_clock::now();
        input[i].push_back(img_copy.clone());
        auto end_img = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_img - start_img);
        total_ms_img_clone += duration;
      } else {
        input[i].push_back(img_copy.clone());
      }
      count_context_--;
      RCLCPP_WARN(this->get_logger(),
                  "第 %d 个 ROI 图像为空，已用原图替代，实际上下文数量: %d",
                  i + 1, count_context_);
    }
  }

  is_raw_image[input.size() - 1] = true; // 原图标记为true

  RCLCPP_DEBUG(this->get_logger(),
               "实际用于检测的上下文数量: %d,input size: %zu", count_context_,
               input.size());

  std::vector<std::vector<float>> boxes(input.size());
  std::vector<std::vector<float>> scores(input.size());
  std::vector<std::vector<int>> class_ids(input.size());

  auto mid = std::chrono::high_resolution_clock::now();

  std::vector<std::thread> infer_threads;

  for (size_t i = 0; i < input.size(); ++i) {
    size_t idx = i;

    // 启动线程进行推理
    infer_threads.emplace_back([this, &input, &boxes, &scores, &class_ids, idx,
                                is_raw_image]() {
      try {
        auto start = std::chrono::high_resolution_clock::now();
        auto result = detector_car_->infer(input[idx], idx);
        if (std::get<2>(result).empty()) {
          boxes[idx] = {};
          scores[idx] = {};
          class_ids[idx] = {};
          RCLCPP_DEBUG(this->get_logger(),
                       "thread %zu : No car detection result", idx);

          return;
        }
        RCLCPP_DEBUG(this->get_logger(),
                     "[LW-DETR] infer finished, box count: %ld",
                     std::get<2>(result)[0].size() / 4);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        RCLCPP_DEBUG(this->get_logger(), "thread %zu inference time: %.3f ms",
                     idx, duration.count() / 1000.0);

        boxes[idx] = std::get<2>(result)[0];
        scores[idx] = std::get<3>(result)[0];
        class_ids[idx] = std::get<4>(result)[0];

        if (!is_raw_image[idx]) {
          // 转换ROI坐标
          for (size_t j = 0; j < boxes[idx].size(); j += 4) {
            boxes[idx][j] += rois_[idx].x;
            boxes[idx][j + 1] += rois_[idx].y;
            boxes[idx][j + 2] += rois_[idx].x;
            boxes[idx][j + 3] += rois_[idx].y;
          }
        }
      } catch (const std::exception &e) {
        std::cerr << "推理线程 " << idx << " 异常: " << e.what() << std::endl;
      }
    });
  }

  for (auto &t : infer_threads)
    t.join();

  //推理时间
  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  auto mid_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - mid);
  RCLCPP_DEBUG(this->get_logger(), "Car detection time: %.3f ms",
               mid_duration.count() / 1000.0);
  RCLCPP_DEBUG(this->get_logger(), "total car detection time: %.3f ms",
               duration.count() / 1000.0);

  // 非极大值抑制前的数据预处理
  auto [precessed_boxes, precessed_scores, precessed_class_ids] =
      post_precess(boxes, scores, class_ids);

  return non_max_suppression(precessed_boxes, precessed_scores,
                             precessed_class_ids, nms_threshold_);
}

std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
detector::detector_node::post_precess(std::vector<std::vector<float>> boxes,
                                      std::vector<std::vector<float>> scores,
                                      std::vector<std::vector<int>> classID) {
  std::vector<float> precessed_boxes, precessed_score;
  std::vector<int> precessed_classID;

  try {
    for (size_t i = 0; i < boxes.size(); ++i) {
      if (!boxes[i].empty()) {
        precessed_boxes.insert(precessed_boxes.end(), boxes[i].begin(),
                               boxes[i].end());
        precessed_score.insert(precessed_score.end(), scores[i].begin(),
                               scores[i].end());
        precessed_classID.insert(precessed_classID.end(), classID[i].begin(),
                                 classID[i].end());
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "合并后共获得 %zu 个检测框",
                 precessed_boxes.size() / 4);

  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "Error merging car detection results: %s",
                e.what());
    return {};
  }

  return std::make_tuple(precessed_boxes, precessed_score, precessed_classID);
}

std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
detector::detector_node::non_max_suppression(const std::vector<float> &boxes,
                                             std::vector<float> scores,
                                             std::vector<int> classID,
                                             float threshold) {

  std::vector<float> keep_boxes, keep_scores;
  std::vector<int> keep_IDs;

  // 如果没有检测框，直接返回空结果
  if (boxes.size() == 0) {
    return std::make_tuple(keep_boxes, keep_scores, keep_IDs);
  }

  // 执行非极大值抑制
  for (size_t i = 0; i < scores.size(); ++i) {
    if (scores[i] < confidence_threshold_) {
      scores[i] = 0;
      continue;
    }

    // 与后续所有框计算IoU，抑制重叠框
    for (size_t j = i + 1; j < scores.size(); ++j) {
      if (scores[j] < confidence_threshold_ || classID[i] != classID[j]) {
        continue; // 跳过低于阈值的框或不同类别的框
      }

      // 计算两个框的IoU
      cv::Rect box1(boxes[i * 4], boxes[i * 4 + 1],
                    boxes[i * 4 + 2] - boxes[i * 4],
                    boxes[i * 4 + 3] - boxes[i * 4 + 1]);

      cv::Rect box2(boxes[j * 4], boxes[j * 4 + 1],
                    boxes[j * 4 + 2] - boxes[j * 4],
                    boxes[j * 4 + 3] - boxes[j * 4 + 1]);

      float iou = bbox_iou(box1, box2);

      // 如果IoU大于阈值，保留置信度更高的框
      if (iou > threshold) {
        if (scores[i] > scores[j]) {
          scores[j] = 0;
        } else {
          scores[i] = 0;
          break; // 当前框已被抑制，不需要继续比较
        }
      }
    }
  }

  // 保留有效的检测框
  for (size_t i = 0; i < scores.size(); ++i) {
    if (scores[i] > confidence_threshold_) {
      keep_boxes.push_back(boxes[i * 4]);
      keep_boxes.push_back(boxes[i * 4 + 1]);
      keep_boxes.push_back(boxes[i * 4 + 2]);
      keep_boxes.push_back(boxes[i * 4 + 3]);
      keep_scores.push_back(scores[i]);
      keep_IDs.push_back(classID[i]);
    }
  }

  return std::make_tuple(keep_boxes, keep_scores, keep_IDs);
}

// 辅助函数：计算两个矩形的IoU
float detector::detector_node::bbox_iou(const cv::Rect &box1,
                                        const cv::Rect &box2) {
  int x1 = std::max(box1.x, box2.x);
  int y1 = std::max(box1.y, box2.y);
  int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
  int y2 = std::min(box1.y + box1.height, box2.y + box2.height);

  int inter_area = std::max(0, x2 - x1) * std::max(0, y2 - y1);
  int box1_area = box1.width * box1.height;
  int box2_area = box2.width * box2.height;

  float iou =
      static_cast<float>(inter_area) / (box1_area + box2_area - inter_area);
  return iou;
}

// 注册组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detector::detector_node)