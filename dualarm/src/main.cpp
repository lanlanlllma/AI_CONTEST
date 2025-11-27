#include "dualarm/headers.hpp"
#include "dualarm/service_server_template.hpp"
#include <thread>
#include <future>
#include "main.h"

#define GRIPPER_ID_L    10
#define GRIPPER_ID_R    10
#define GRIPPER_DISABLE 0
#define GRIPPER_ENABLE  1
#define GRIPPER_SET     2
#define GRIPPER_OPEN    0
#define GRIPPER_CLOSE   255

#define GRIPPER_LIST \
    { GRIPPER_ID_L, GRIPPER_ID_R }

// inline void wait_until_key_pressed() {
//     // 等待用户按下任意键
//     std::cout << "Press any key to continue..." << std::endl;
//     std::cin.get();
// }

// clang-format off
/*
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // ();
    ████████╗  ██╗   ██╗  █████╗  ██╗       █████╗  ██████╗  ███╗   ███╗
    ██╔══  ██║ ██║   ██║ ██╔══██╗ ██║      ██╔══██╗ ██╔══██╗ ████╗ ████║
    ██║    ██║ ██║   ██║ ███████║ ██║      ███████║ ██████╔╝ ██╔████╔██║
    ██║    ██║ ██║   ██║ ██╔══██║ ██║      ██╔══██║ ██╔══██╗ ██║╚██╔╝██║
    ████████║  ╚██████╔╝ ██║  ██║ ███████╗ ██║  ██║ ██║  ██║ ██║ ╚═╝ ██║
    ╚═══════╝   ╚═════╝  ╚═╝  ╚═╝ ╚══════╝ ╚═╝  ╚═╝ ╚═╝  ╚═╝ ╚═╝     ╚═╝
        // f. u. c. k.  a.l.l.  t.h.e.  v.t.a.b.l.e.s. //
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    */
/*
TODO: #3 写个随便什么模版类把request和response的处理逻辑封装起来，
      这样就可以直接使用模版类来处理请求和响应了。
      例如：
      template<typename RequestT, typename ResponseT>
      class ServiceHandler {
          // ...
      };
      这样就可以在RobotMain类中使用ServiceHandler<robo_ctrl::srv::RobotAct, robo_ctrl::srv::RobotAct::Response>来处理RobotAct服务的请求和响应。
      这样可以减少重复代码，提高代码的可读性和可维护性。
      另外，考虑到可能会有多个服务需要处理，可以使用一个map来存储不同服务的处理器，
      这样就可以根据服务名动态选择处理器进行处理了。
      例如：
      std::map<std::string, std::shared_ptr<ServiceHandlerBase>> service_handlers_;
      这样就可以在RobotMain类的构造函数中注册不同服务的处理器了。
      例如：
      service_handlers_["robot_act"] = std::make_shared<ServiceHandler<robo_ctrl::srv::RobotAct, robo_ctrl::srv::RobotAct::Response>>(this, "robot_act");
      这样就可以在RobotMain类中使用service_handlers_["robot_act"]->handle_request(request)来处理RobotAct服务的请求了。
      这样可以大大简化服务处理的代码逻辑，
      使得RobotMain类的代码更加清晰和易于维护。
*/

/*
TODO: 把一些固定的位置转换成关节的角度。

*/

/*
marble703:
    这个包我编不过，写了扔robo_ctrl包里了，在
    robo_ctrl/include/robo_ctrl/service_server_template.hpp
    能编过，里面看着没问题
 */

// clang-format on

RobotMain::RobotMain(const rclcpp::NodeOptions& options): Node("robot_main", options) {
    // Initialize tf2
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize subscribers
    L_robot_state_sub_ = this->create_subscription<robo_ctrl::msg::RobotState>(
        "/L/robot_state",
        10,
        [this](const robo_ctrl::msg::RobotState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(L_robot_state_mutex_);
            L_robot_state_ = msg;
            // std::cout << "Received L robot state update" << std::endl;
        }
    );
    L_gripper_status_sub_ = this->create_subscription<epg50_gripper_ros::msg::GripperStatus>(
        "gripper_status",
        10,
        [this](const epg50_gripper_ros::msg::GripperStatus::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(L_gripper_status_mutex_);
            L_gripper_status_ = msg;
        }
    );
    obj_detect_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = obj_detect_cb_group_;

    L_bbox2d_array_sub_ = this->create_subscription<detector::msg::Bbox2dArray>(
        "/detector/detections",
        10,
        std::bind(&RobotMain::handleObjDetection, this, std::placeholders::_1),
        sub_options
    );

    // Initialize clients
    L_robot_servo_line_client_ = this->create_client<robo_ctrl::srv::RobotServoLine>(ROBOT_L + "/robot_servo_line");
    L_robot_act_client_        = this->create_client<robo_ctrl::srv::RobotAct>(ROBOT_L + "/robot_act");
    L_robot_move_client_       = this->create_client<robo_ctrl::srv::RobotMove>(ROBOT_L + "/robot_move");
    L_robot_move_cart_client_  = this->create_client<robo_ctrl::srv::RobotMoveCart>(ROBOT_L + "/robot_move_cart");
    L_robot_servo_client_      = this->create_client<robo_ctrl::srv::RobotServo>(ROBOT_L + "/robot_servo");
    L_robot_set_speed_client_  = this->create_client<robo_ctrl::srv::RobotSetSpeed>(ROBOT_L + "/robot_set_speed");
    L_robot_act_j_client_      = this->create_client<robo_ctrl::srv::RobotActJ>(ROBOT_L + "/robot_act_j");
    gripper_command_client_    = this->create_client<epg50_gripper_ros::srv::GripperCommand>("/epg50_gripper/command");
    L_grasp_detect_mask_client_ = this->create_client<grab_detect::srv::GraspDetectTriggerMask>("/grasp_detect_trigger_mask");

    this->declare_parameter("init_tcp_pose", std::vector<double> { 168.0, -102.0, 394.0, -111.556, 0.0, -90.0 });
    this->declare_parameter("open_cap_joint_pose", std::vector<double> { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    this->declare_parameter("desk_height", 213.7);
    this->declare_parameter("cola_height", 91);
    this->declare_parameter("cestbon_height", 91);
    this->declare_parameter("robot_speed", 0.5);
    init_tcp_pose_vec_ = this->get_parameter("init_tcp_pose").as_double_array();
    rclcpp::sleep_for(std::chrono::seconds(2));
}
// 定义析构函数，生成 vtable
RobotMain::~RobotMain() {}

std::vector<double> RobotMain::calculateTcpToObjectIncrement(const std::vector<double>& object_position) {
    // 获取当前TCP位置（在基坐标系下）
    robo_ctrl::msg::TCPPose current_tcp_pose;
    {
        std::lock_guard<std::mutex> lock(L_robot_state_mutex_);
        if (L_robot_state_) {
            current_tcp_pose = L_robot_state_->tcp_pose;
        } else {
            RCLCPP_WARN(this->get_logger(), "Robot state not available");
            return { 0.0, 0.0, 0.0 };
        }
    }

    // 计算从当前TCP位置到物体位置的增量（都在基坐标系下）
    std::vector<double> increment = { (object_position[0] * 1000) - current_tcp_pose.x,
                                      (object_position[1] * 1000) - current_tcp_pose.y,
                                      (object_position[2] * 1000) - current_tcp_pose.z };

    RCLCPP_INFO(
        this->get_logger(),
        "Current TCP position (baselink): [%.3f, %.3f, %.3f]",
        current_tcp_pose.x,
        current_tcp_pose.y,
        current_tcp_pose.z
    );

    RCLCPP_INFO(
        this->get_logger(),
        "Object position (baselink): [%.3f, %.3f, %.3f]",
        object_position[0],
        object_position[1],
        object_position[2]
    );

    RCLCPP_INFO(
        this->get_logger(),
        "TCP to object increment: [%.3f, %.3f, %.3f]",
        increment[0],
        increment[1],
        increment[2]
    );

    return increment;
}

void RobotMain::handleObjDetection(const detector::msg::Bbox2dArray::SharedPtr bbox2d_array_msg) {
    // 异步处理避免阻塞
    std::async(std::launch::async, [this, bbox2d_array_msg]() {
        // 检查更新开关状态
        if (!isObjectUpdateEnabled()) {
            RCLCPP_DEBUG(this->get_logger(), "Object detection update is disabled");
            return;
        }

        // 使用互斥锁保护整个更新过程
        std::lock_guard<std::mutex> lock(detected_objects_mutex_);

        // 首先增加所有现有物体的年龄
        for (auto& obj: detected_objects_) {
            obj.age++;
        }

        // 处理检测到的物体（从 2D bbox 转换为位置）
        for (const auto& bbox2d: bbox2d_array_msg->results) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Received bbox2d with class_id: %d at pixel (%.1f, %.1f)",
                bbox2d.class_id,
                bbox2d.x,
                bbox2d.y
            );

            // 将像素坐标转换为机器人坐标系中的位置
            // 注意：这里需要根据相机标定参数进行转换
            // 为简化，这里假设已经有一个 TF 转换或者直接使用像素坐标
            // 实际应用中需要使用 TF 将 camera_frame -> base_link

            // TODO: 这里需要实现从像素坐标到机器人坐标系的转换
            // 临时使用像素坐标作为位置（单位：米）
            // 实际中需要结合深度信息或其他方式转换
            double x_pos = bbox2d.x / 1000.0; // 假设像素坐标需要缩放
            double y_pos = bbox2d.y / 1000.0;
            double z_pos = 0.0; // 2D 检测不提供 Z 信息

            // 提取物体信息
            object_detection new_obj;
            new_obj.id          = bbox2d.class_id;
            new_obj.position    = { x_pos, y_pos, z_pos };
            new_obj.age         = 0;
            new_obj.valid_count = 1;
            new_obj.bbox_x      = bbox2d.x;
            new_obj.bbox_y      = bbox2d.y;
            new_obj.bbox_width  = bbox2d.width;
            new_obj.bbox_height = bbox2d.height;
            new_obj.add_class_observation(bbox2d.class_id);

            // 查找是否已存在相同位置的物体（仅考虑 X-Y 平面距离）
            bool found_existing = false;
            int best_match_idx  = -1;
            double min_distance = distance_threshold_;

            for (size_t i = 0; i < detected_objects_.size(); ++i) {
                // 计算二维平面距离（仅 X-Y）
                double distance = std::sqrt(
                    std::pow(detected_objects_[i].position[0] - new_obj.position[0], 2)
                    + std::pow(detected_objects_[i].position[1] - new_obj.position[1], 2)
                );

                // 找到距离最近且在阈值内的物体
                if (distance < min_distance) {
                    min_distance   = distance;
                    best_match_idx = i;
                    found_existing = true;
                }
            }

            if (found_existing && best_match_idx >= 0) {
                // 找到了匹配的物体，更新其信息
                auto& existing_obj = detected_objects_[best_match_idx];

                // 重置年龄
                existing_obj.age = 0;
                existing_obj.valid_count++;
                
                // 更新边界框信息
                existing_obj.bbox_x      = bbox2d.x;
                existing_obj.bbox_y      = bbox2d.y;
                existing_obj.bbox_width  = bbox2d.width;
                existing_obj.bbox_height = bbox2d.height;

                // 添加类别观测（如果是 class_id=1 会在此函数内自动设置 id=1）
                existing_obj.add_class_observation(bbox2d.class_id);

                // 使用卡尔曼滤波更新位置
                std::vector<double> filtered_position = kalman_filters_[best_match_idx].filterPose(new_obj.position);
                existing_obj.position                 = filtered_position;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Updated object %d position: [%.3f, %.3f, %.3f], age: %d, valid_count: %d, class_stable: %s, has_seen_class_1: %s",
                    existing_obj.id,
                    filtered_position[0],
                    filtered_position[1],
                    filtered_position[2],
                    existing_obj.age,
                    existing_obj.valid_count,
                    existing_obj.is_class_stable() ? "YES" : "NO",
                    existing_obj.has_seen_class_1 ? "YES" : "NO"
                );
            } else {
                // 新物体，添加到列表中
                RobotKalmanFilter new_filter;

                // 初始化滤波器位置
                std::vector<double> filtered_position = new_filter.filterPose(new_obj.position);
                new_obj.position                      = filtered_position;

                // 添加到列表
                detected_objects_.push_back(new_obj);
                kalman_filters_.push_back(new_filter);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Added new object %d at position: [%.3f, %.3f, %.3f], age: %d, valid_count: %d",
                    new_obj.id,
                    filtered_position[0],
                    filtered_position[1],
                    filtered_position[2],
                    new_obj.age,
                    new_obj.valid_count
                );
            }
        }

        // 移除过期的物体（年龄超过阈值的物体）
        for (size_t i = 0; i < detected_objects_.size();) {
            if (detected_objects_[i].age > age_threshold_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Removing expired object %d (age: %d > threshold: %d)",
                    detected_objects_[i].id,
                    detected_objects_[i].age,
                    age_threshold_
                );
                detected_objects_.erase(detected_objects_.begin() + i);
                kalman_filters_.erase(kalman_filters_.begin() + i);
            } else {
                ++i;
            }
        }

        // 输出当前检测到的所有物体数量和状态
        RCLCPP_INFO(this->get_logger(), "Total detected objects: %zu", detected_objects_.size());
        for (const auto& obj: detected_objects_) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Object %d: age=%d, valid_count=%d, class_stable=%s, valid=%s",
                obj.id,
                obj.age,
                obj.valid_count,
                obj.is_class_stable() ? "YES" : "NO",
                obj.is_valid() ? "true" : "false"
            );
        }
    });
}

template<typename ServiceType>
class ServiceCaller {
public:
    using Request  = typename ServiceType::Request;
    using Response = typename ServiceType::Response;
    using Client   = rclcpp::Client<ServiceType>;

    static std::shared_ptr<Response> callServiceSync(
        std::shared_ptr<Client> client,
        std::shared_ptr<Request> request,
        rclcpp::Node::SharedPtr node,
        std::chrono::seconds timeout    = std::chrono::seconds(5),
        const std::string& service_name = "unknown"
    ) {
        if (!client) {
            auto response     = std::make_shared<Response>();
            response->success = false;
            response->message = "Client is null for service: " + service_name;
            RCLCPP_ERROR(node->get_logger(), "Client is null for service: %s", service_name.c_str());
            return response;
        }

        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, future, timeout) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "Service '%s' succeeded: %s",
                    service_name.c_str(),
                    response->message.c_str()
                );
            } else {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "Service '%s' failed: %s",
                    service_name.c_str(),
                    response->message.c_str()
                );
            }
            return response;
        } else {
            auto response     = std::make_shared<Response>();
            response->success = false;
            response->message = "Service call timeout for: " + service_name;
            RCLCPP_ERROR(node->get_logger(), "Service call timeout for: %s", service_name.c_str());
            return response;
        }
    }
};

/**
 * @brief 机器人服务调用的便捷宏定义
 */
// 原有的宏定义已被上面的ServiceCaller模板替代

/**
 * @brief 增强的等待服务函数
 */
template<typename ClientType>
bool waitForServiceEnhanced(
    std::shared_ptr<ClientType> client,
    rclcpp::Node::SharedPtr node,
    const std::string& service_name,
    std::chrono::seconds timeout = std::chrono::seconds(30)
) {
    RCLCPP_INFO(node->get_logger(), "Waiting for service: %s", service_name.c_str());

    auto start_time = std::chrono::steady_clock::now();
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > timeout) {
            RCLCPP_ERROR(node->get_logger(), "Timeout waiting for service: %s", service_name.c_str());
            return false;
        }

        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service: %s", service_name.c_str());
            return false;
        }

        RCLCPP_INFO(node->get_logger(), "Still waiting for service: %s", service_name.c_str());
    }

    RCLCPP_INFO(node->get_logger(), "Service '%s' is available", service_name.c_str());
    return true;
}
// auto open_gripper (rclcpp::Node::SharedPtr node) {
//     auto gripper_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
//     gripper_request->slave_id = GRIPPER_ID_L;
//     gripper_request->command  = GRIPPER_SET;
//     gripper_request->position = GRIPPER_OPEN;
//     gripper_request->speed    = 255; // 设置速度为255
//     gripper_request->torque   = 255; // 设置扭矩为255
//     auto gripper_future       = node->gripper_command_client_->async_send_request(gripper_request);
//     if (rclcpp::spin_until_future_complete(node, gripper_future) != rclcpp::FutureReturnCode::SUCCESS) {
//         RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper command");
//         return 1;
//     }
//     auto gripper_response = gripper_future.get();
//     if (!gripper_response->success) {
//         RCLCPP_ERROR(node->get_logger(), "Failed to open gripper");
//         return 1;
//     }
// };

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor executor();
    auto node = std::make_shared<RobotMain>(options);
    RCLCPP_INFO(node->get_logger(), "Waiting for object detection data...");
    auto start_time = node->get_clock()->now();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    }
    // 等待所有必要的服务可用 - 使用增强的等待函数
    RCLCPP_INFO(node->get_logger(), "Waiting for all required services...");

    // 等待机器人移动服务
    if (!waitForServiceEnhanced(node->L_robot_move_client_, node, "L/robot_move")) {
        return 1;
    }

    // 等待机器人操作服务
    if (!waitForServiceEnhanced(node->L_robot_act_client_, node, "L/robot_act")) {
        return 1;
    }

    // 等待机器人笛卡尔移动服务
    if (!waitForServiceEnhanced(node->L_robot_move_cart_client_, node, "L/robot_move_cart")) {
        return 1;
    }

    // 等待夹爪命令服务
    if (!waitForServiceEnhanced(node->gripper_command_client_, node, "gripper_command")) {
        return 1;
    }
    // 停止检测更新
    node->setObjectUpdateEnabled(true);
    RCLCPP_INFO(node->get_logger(), "All required services are now available!");
    // rclcpp::sleep_for(std::chrono::seconds(5));
    // 看向台面 - 使用robot_act_j服务
    auto L_act_j_request = std::make_shared<robo_ctrl::srv::RobotActJ::Request>();
    L_act_j_request->command_type  = 0; // ServoMoveStart
    L_act_j_request->target_joints = std::vector<double>({ 
        156.1977,
        -54.6986,
        -0.0274,
        -38.7380,
        -88.2650,
        -13.9097 });
    L_act_j_request->point_count     = 100;     // 100个点
    L_act_j_request->message_time    = 0.02;    // 0.01s/点
    L_act_j_request->use_incremental = false; // 使用绝对位置
    auto L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::seconds(5));
    L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::seconds(5));
    // 开始检查
    node->setObjectUpdateEnabled(true);

    if (node->hasObject(1)) {
        RCLCPP_INFO(node->get_logger(), "Found valid object with class ID 1");
    } else {
        RCLCPP_WARN(node->get_logger(), "No valid object with class ID 1 found");
    }

    // 准备抓取id=1的物体
    auto gripper_request      = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    gripper_request->slave_id = GRIPPER_ID_L;
    gripper_request->command  = GRIPPER_SET;
    gripper_request->position = GRIPPER_OPEN;
    gripper_request->speed    = 255; // 设置速度为255
    gripper_request->torque   = 255; // 设置扭矩为255
    auto gripper_future       = node->gripper_command_client_->async_send_request(gripper_request);
    if (rclcpp::spin_until_future_complete(node, gripper_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_act for gripper command");
        return 1;
    }
    auto gripper_response = gripper_future.get();
    if (!gripper_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open gripper");
        return 1;
    }

    // 生成obj bbox 的mask，使用cv2
    // 获取位置和边界框信息
    std::vector<double> object_position;
    float bbox_x = 0.0f, bbox_y = 0.0f, bbox_width = 0.0f, bbox_height = 0.0f;
    bool found_target = false;
    
    {
        std::lock_guard<std::mutex> lock(node->detected_objects_mutex_);
        for (const auto& obj: node->detected_objects_) {
            if (obj.id == 1 && obj.is_valid()) {
                object_position = obj.position;
                bbox_x = obj.bbox_x;
                bbox_y = obj.bbox_y;
                bbox_width = obj.bbox_width;
                bbox_height = obj.bbox_height;
                found_target = true;
                break;
            }
        }
    }
    
    if (!found_target || object_position.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No valid object with class ID 1 found for grasping");
        return 1;
    }
    
    RCLCPP_INFO(
        node->get_logger(),
        "Preparing to grasp object ID 1 at position: [%.3f, %.3f, %.3f]",
        object_position[0],
        object_position[1],
        object_position[2]
    );

    // 创建掩码图像（假设相机分辨率为 1280x720）
    // TODO: 从相机参数或订阅的图像消息中获取实际分辨率
    int image_width = 1280;
    int image_height = 720;
    cv::Mat mask = cv::Mat::zeros(image_height, image_width, CV_8UC1);
    
    // 计算 bbox 的左上角和右下角坐标
    int x1 = std::max(0, static_cast<int>(bbox_x - bbox_width / 2.0f));
    int y1 = std::max(0, static_cast<int>(bbox_y - bbox_height / 2.0f));
    int x2 = std::min(image_width - 1, static_cast<int>(bbox_x + bbox_width / 2.0f));
    int y2 = std::min(image_height - 1, static_cast<int>(bbox_y + bbox_height / 2.0f));
    // 扩大一些区域以确保覆盖整个物体
    int expand_pixels = 30;
    x1 = std::max(0, x1 - expand_pixels);
    y1 = std::max(0, y1 - expand_pixels);
    x2 = std::min(image_width - 1, x2 + expand_pixels);
    y2 = std::min(image_height - 1, y2 + expand_pixels);
    
    // 在掩码上绘制白色矩形（255 表示目标物体区域）
    cv::rectangle(mask, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255), cv::FILLED);
    
    RCLCPP_INFO(
        node->get_logger(),
        "Generated mask for object ID 1: bbox center=(%.1f, %.1f), size=(%.1f x %.1f), rect=[%d, %d, %d, %d]",
        bbox_x, bbox_y, bbox_width, bbox_height,
        x1, y1, x2, y2
    );

    // 将 cv::Mat 掩码转换为 sensor_msgs::Image
    auto mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
    mask_msg->header.stamp = node->now();
    mask_msg->header.frame_id = "camera_link";
    
    RCLCPP_INFO(node->get_logger(), "Calling grasp detection service with mask...");
    
    // 调用抓取检测服务（带掩码）
    auto grasp_request = std::make_shared<grab_detect::srv::GraspDetectTriggerMask::Request>();
    grasp_request->mask_image = *mask_msg;
    
    auto grasp_future = node->L_grasp_detect_mask_client_->async_send_request(grasp_request);
    
    if (rclcpp::spin_until_future_complete(node, grasp_future, std::chrono::seconds(10)) 
        != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call grasp detection service");
        return 1;
    }
    
    auto grasp_response = grasp_future.get();
    
    if (!grasp_response->success || grasp_response->num_grasps == 0) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Grasp detection failed: %s (num_grasps=%d)",
            grasp_response->message.c_str(),
            grasp_response->num_grasps
        );
        return 1;
    }
    
    RCLCPP_INFO(
        node->get_logger(),
        "Grasp detection succeeded: %d grasps detected. Message: %s",
        grasp_response->num_grasps,
        grasp_response->message.c_str()
    );
    
    // 检查并打印最佳抓取在 fake_gripper_frame 下的坐标
    if (grasp_response->best_under_gripper_fake.size() == 6) {
        RCLCPP_INFO(
            node->get_logger(),
            "Best grasp in fake_gripper_frame: position=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
            grasp_response->best_under_gripper_fake[0],
            grasp_response->best_under_gripper_fake[1],
            grasp_response->best_under_gripper_fake[2],
            grasp_response->best_under_gripper_fake[3],
            grasp_response->best_under_gripper_fake[4],
            grasp_response->best_under_gripper_fake[5]
        );
    } else {
        RCLCPP_WARN(
            node->get_logger(),
            "best_under_gripper_fake is empty or invalid (size=%zu). TF transform may have failed.",
            grasp_response->best_under_gripper_fake.size()
        );
    }
    std::vector<double> target_pose = {
        grasp_response->best_under_gripper_fake[0],
        grasp_response->best_under_gripper_fake[1],
        grasp_response->best_under_gripper_fake[2],
        grasp_response->best_under_gripper_fake[3],
        grasp_response->best_under_gripper_fake[4],
        grasp_response->best_under_gripper_fake[5]
    };

    // 移动到目标位置上方40mm处
    double approach_offset = 55.0;
    auto pose = node->getCurrentTcpPose(); 
    RCLCPP_INFO(
        node->get_logger(),
        "Current TCP pose before move up: x=%.3f, y=%.3f, z=%.3f",
        pose.x,
        pose.y,
        pose.z
    );
    double delta_z = node->desk_height_ - pose.z + approach_offset;
    std::cout << "delta_z: " << delta_z << std::endl;
    node->setObjectUpdateEnabled(false); // 禁用物体位置更新，避免抓取过程中目标位置发生变化
    
    // 使用 ServiceCaller 向上移动到安全高度
    auto move_up_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    node->gen_move_request(move_up_request, { 0, 0, delta_z, 0, 0, 0 }, true); // 使用增量模式
    
    auto move_up_response = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_,
        move_up_request,
        node,
        std::chrono::seconds(10),
        "L/robot_move_cart"
    );
    
    if (!move_up_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move robot up: %s", move_up_response->message.c_str());
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 等待2秒稳定

    // 使用 ServiceCaller 移动到目标抓取位置
    auto move_to_grasp_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    node->gen_move_request(move_to_grasp_request, {1000*grasp_response->best_under_gripper_fake[0]-10,
                                                   1000*grasp_response->best_under_gripper_fake[1]-60,
                                                   0, 0, 0, 0}, true); // 使用增量模式
    RCLCPP_INFO(
        node->get_logger(),
        "Moving to grasp position increment: x=%.3f, y=%.3f, z=%.3f",
        1000*grasp_response->best_under_gripper_fake[0],
        1000*grasp_response->best_under_gripper_fake[1],
        0.0
    );
    
    auto move_to_grasp_response = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_,
        move_to_grasp_request,
        node,
        std::chrono::seconds(10),
        "L/robot_move_cart"
    );
    
    if (!move_to_grasp_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move robot to grasp position: %s", move_to_grasp_response->message.c_str());
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒稳定
    // 把抓取方向与TCP x轴在xy平面上垂直（仅调整Z轴旋转，限制在±90°内）
    // best_under_gripper_fake[5] 是相对fake_gripper_frame的Z轴增量旋转（弧度）
    
    if (grasp_response->best_under_gripper_fake.size() >= 6) {
    // 1. 提取原始 Yaw（弧度转度）
    double input_rz = grasp_response->best_under_gripper_fake[5] * 180.0 / M_PI;

    // 2. 施加垂直约束：直接加 90 度
    // 说明：对于二指夹爪，Target = Input + 90 和 Target = Input - 90 抓取姿态是一样的
    double target_rz = input_rz + 90.0;

    // 3. 标准归一化到 [-180, 180]
    // 这一步处理超过 360 或 -360 的情况
    while (target_rz > 180.0) target_rz -= 360.0;
    while (target_rz < -180.0) target_rz += 360.0;

    // 4. 对称夹爪优化：将角度限制在 [-90, 90] 范围内
    // 如果角度超过 90 度，说明转 -80 度也能达到同样的抓取效果（180度对称性）
    if (target_rz > 90.0) {
        target_rz -= 180.0;
    } else if (target_rz < -90.0) {
        target_rz += 180.0;
    }

    // 最终赋值
    double d_rz = target_rz;

    RCLCPP_INFO(node->get_logger(), 
        "Grasp Z-axis processing: Input=%.2f, Target_Perpendicular=%.2f (Normalized to +/-90)", 
        input_rz, d_rz);

        // return 0;
    //     if (std::fabs(d_rz) > 5.0) { // 超过5度才调整
    //         auto orient_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    //         node->gen_move_request(orient_request, {0, 0, 0, 0, 0, d_rz}, true);
    //         auto orient_resp = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
    //             node->L_robot_move_cart_client_, orient_request, node, std::chrono::seconds(10), "L/robot_move_cart_orient"
    //         );
    //         if (!orient_resp->success) {
    //             RCLCPP_WARN(node->get_logger(), "Z-axis orientation adjust failed: %s", orient_resp->message.c_str());
    //         } else {
    //             RCLCPP_INFO(node->get_logger(), "Z-axis orientation adjusted by %.2f degrees", d_rz);
    //             node->wait_until_done();
    //         }
    //     } else {
    //         RCLCPP_INFO(node->get_logger(), "Z-axis orientation delta small (%.2f); skip adjustment", d_rz);
    //     }
    // } else {
    //     RCLCPP_WARN(node->get_logger(), "best_under_gripper_fake size < 6; skip orientation adjustment");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒稳定
    // 下降到抓取高度（假设先前在安全高度），使用深度或固定偏移
    double descend_dist = -52.0; // mm，简单策略：下降53mm抓取
    auto move_down_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    node->gen_move_request(move_down_request, {0, 0, descend_dist, 0, 0, 0}, true);
    auto move_down_resp = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_, move_down_request, node, std::chrono::seconds(10), "L/robot_move_cart"
    );
    if (!move_down_resp->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to descend for grasp: %s", move_down_resp->message.c_str());
        return 1;
    }
    node->wait_until_done();

    // 关闭夹爪执行抓取
    auto gripper_close_request = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    gripper_close_request->slave_id = GRIPPER_ID_L; // 使用左夹爪
    gripper_close_request->command  = GRIPPER_SET;
    gripper_close_request->position = GRIPPER_CLOSE;
    gripper_close_request->speed    = 255;
    gripper_close_request->torque   = 255;
    auto gripper_close_resp = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
        node->gripper_command_client_, gripper_close_request, node, std::chrono::seconds(5), "gripper_close_L"
    );
    if (!gripper_close_resp->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to close gripper: %s", gripper_close_resp->message.c_str());
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Gripper closed; lifting object");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    RCLCPP_INFO(node->get_logger(), "Gripper closed; lifting object");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // 抓取后抬起
    auto lift_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    node->gen_move_request(lift_request, {0, 0, 80, 0, 0, 0}, true); // 抬起80mm
    auto lift_resp = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_, lift_request, node, std::chrono::seconds(10), "L/robot_move_cart"
    );
    if (!lift_resp->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to lift after grasp: %s", lift_resp->message.c_str());
        return 1;
    }
    node->wait_until_done();
    std::this_thread::sleep_for(std::chrono::seconds(1));
        L_act_j_request->target_joints = std::vector<double>({ 
        124.741,
        0.6219,
        -50.57909,
        -42.81143,
        -90.3744,
        -56.64594 });
    L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::seconds(5));
    gripper_close_request = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    gripper_close_request->slave_id = GRIPPER_ID_L; // 使用左夹爪
    gripper_close_request->command  = GRIPPER_SET;
    gripper_close_request->position = GRIPPER_OPEN;
    gripper_close_request->speed    = 255;
    gripper_close_request->torque   = 255;
    gripper_close_resp = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
        node->gripper_command_client_, gripper_close_request, node, std::chrono::seconds(5), "gripper_close_L"
    );
    if (!gripper_close_resp->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to close gripper: %s", gripper_close_resp->message.c_str());
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Gripper closed; lifting object");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // std::this_thread::sleep_for(std::chrono::seconds(1));
/*
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // ();
    ████████╗  ██╗   ██╗  █████╗  ██╗       █████╗  ██████╗  ███╗   ███╗
    ██╔══  ██║ ██║   ██║ ██╔══██╗ ██║      ██╔══██╗ ██╔══██╗ ████╗ ████║
    ██║    ██║ ██║   ██║ ███████║ ██║      ███████║ ██████╔╝ ██╔████╔██║
    ██║    ██║ ██║   ██║ ██╔══██║ ██║      ██╔══██║ ██╔══██╗ ██║╚██╔╝██║
    ████████║  ╚██████╔╝ ██║  ██║ ███████╗ ██║  ██║ ██║  ██║ ██║ ╚═╝ ██║
    ╚═══════╝   ╚═════╝  ╚═╝  ╚═╝ ╚══════╝ ╚═╝  ╚═╝ ╚═╝  ╚═╝ ╚═╝     ╚═╝
        // f. u. c. k.  a.l.l.  t.h.e.  v.t.a.b.l.e.s. //
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

    // 恢复物体更新
    L_act_j_request->target_joints = std::vector<double>({ 
        156.1977,
        -54.6986,
        -0.0274,
        -38.7380,
        -88.2650,
        -13.9097 });
    L_act_j_request->point_count     = 100;     // 100个点
    L_act_j_request->message_time    = 0.02;    // 0.01s/点
    L_act_j_request->use_incremental = false; // 使用绝对位置
    L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::seconds(5));
    node->setObjectUpdateEnabled(true);

        L_act_j_request->target_joints = std::vector<double>({ 
        156.1977,
        -54.6986,
        -0.0274,
        -38.7380,
        -88.2650,
        -13.9097 });
    L_act_j_request->point_count     = 100;     // 100个点
    L_act_j_request->message_time    = 0.02;    // 0.01s/点
    L_act_j_request->use_incremental = false; // 使用绝对位置
    L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::seconds(5));
    if (node->hasObject(0)) {
        RCLCPP_INFO(node->get_logger(), "Found valid object with class ID 0");
    } else {
        RCLCPP_WARN(node->get_logger(), "No valid object with class ID 0 found");
    }
    bbox_x = 0.0f, bbox_y = 0.0f, bbox_width = 0.0f, bbox_height = 0.0f;
    found_target = false;
    
    {
        std::lock_guard<std::mutex> lock(node->detected_objects_mutex_);
        for (const auto& obj: node->detected_objects_) {
            if (obj.id == 0 && obj.is_valid()) {
                object_position = obj.position;
                bbox_x = obj.bbox_x;
                bbox_y = obj.bbox_y;
                bbox_width = obj.bbox_width;
                bbox_height = obj.bbox_height;
                found_target = true;
                break;
            }
        }
    }
    
    if (!found_target || object_position.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No valid object with class ID 0 found for grasping");
        return 1;
    }
    
    RCLCPP_INFO(
        node->get_logger(),
        "Preparing to grasp object ID 0 at position: [%.3f, %.3f, %.3f]",
        object_position[0],
        object_position[1],
        object_position[2]
    );
    mask = cv::Mat::zeros(image_height, image_width, CV_8UC1);
    
    // 计算 bbox 的左上角和右下角坐标
    x1 = std::max(0, static_cast<int>(bbox_x - bbox_width / 2.0f));
    y1 = std::max(0, static_cast<int>(bbox_y - bbox_height / 2.0f));
    x2 = std::min(image_width - 1, static_cast<int>(bbox_x + bbox_width / 2.0f));
    y2 = std::min(image_height - 1, static_cast<int>(bbox_y + bbox_height / 2.0f));
    // 扩大一些区域以确保覆盖整个物体
    expand_pixels = 30;
    x1 = std::max(0, x1 - expand_pixels);
    y1 = std::max(0, y1 - expand_pixels);
    x2 = std::min(image_width - 1, x2 + expand_pixels);
    y2 = std::min(image_height - 1, y2 + expand_pixels);
    
    // 在掩码上绘制白色矩形（255 表示目标物体区域）
    cv::rectangle(mask, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255), cv::FILLED);
    
    RCLCPP_INFO(
        node->get_logger(),
        "Generated mask for object ID 1: bbox center=(%.1f, %.1f), size=(%.1f x %.1f), rect=[%d, %d, %d, %d]",
        bbox_x, bbox_y, bbox_width, bbox_height,
        x1, y1, x2, y2
    );

    // 将 cv::Mat 掩码转换为 sensor_msgs::Image
    mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
    mask_msg->header.stamp = node->now();
    mask_msg->header.frame_id = "camera_link";
    
    RCLCPP_INFO(node->get_logger(), "Calling grasp detection service with mask...");
    
    // 调用抓取检测服务（带掩码）
    grasp_request = std::make_shared<grab_detect::srv::GraspDetectTriggerMask::Request>();
    grasp_request->mask_image = *mask_msg;
    
    grasp_future = node->L_grasp_detect_mask_client_->async_send_request(grasp_request);
    
    if (rclcpp::spin_until_future_complete(node, grasp_future, std::chrono::seconds(10)) 
        != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call grasp detection service");
        return 1;
    }
    
    grasp_response = grasp_future.get();
    
    if (!grasp_response->success || grasp_response->num_grasps == 0) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Grasp detection failed: %s (num_grasps=%d)",
            grasp_response->message.c_str(),
            grasp_response->num_grasps
        );
        return 1;
    }
    
    RCLCPP_INFO(
        node->get_logger(),
        "Grasp detection succeeded: %d grasps detected. Message: %s",
        grasp_response->num_grasps,
        grasp_response->message.c_str()
    );
    
    // 检查并打印最佳抓取在 fake_gripper_frame 下的坐标
    if (grasp_response->best_under_gripper_fake.size() == 6) {
        RCLCPP_INFO(
            node->get_logger(),
            "Best grasp in fake_gripper_frame: position=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
            grasp_response->best_under_gripper_fake[0],
            grasp_response->best_under_gripper_fake[1],
            grasp_response->best_under_gripper_fake[2],
            grasp_response->best_under_gripper_fake[3],
            grasp_response->best_under_gripper_fake[4],
            grasp_response->best_under_gripper_fake[5]
        );
    } else {
        RCLCPP_WARN(
            node->get_logger(),
            "best_under_gripper_fake is empty or invalid (size=%zu). TF transform may have failed.",
            grasp_response->best_under_gripper_fake.size()
        );
    }
    target_pose = {
        grasp_response->best_under_gripper_fake[0],
        grasp_response->best_under_gripper_fake[1],
        grasp_response->best_under_gripper_fake[2],
        grasp_response->best_under_gripper_fake[3],
        grasp_response->best_under_gripper_fake[4],
        grasp_response->best_under_gripper_fake[5]
    };

    // 移动到目标位置上方40mm处
    pose = node->getCurrentTcpPose(); 
    RCLCPP_INFO(
        node->get_logger(),
        "Current TCP pose before move up: x=%.3f, y=%.3f, z=%.3f",
        pose.x,
        pose.y,
        pose.z
    );
    delta_z = node->desk_height_ - pose.z + approach_offset;
    std::cout << "delta_z: " << delta_z << std::endl;
    node->setObjectUpdateEnabled(false); // 禁用物体位置更新，避免抓取过程中目标位置发生变化
    
    // 使用 ServiceCaller 向上移动到安全高度
    node->gen_move_request(move_up_request, { 0, 0, delta_z, 0, 0, 0 }, true); // 使用增量模式
    
    move_up_response = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_,
        move_up_request,
        node,
        std::chrono::seconds(10),
        "L/robot_move_cart"
    );
    
    if (!move_up_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move robot up: %s", move_up_response->message.c_str());
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 等待2秒稳定

    // 使用 ServiceCaller 移动到目标抓取位置
    node->gen_move_request(move_to_grasp_request, {1000*grasp_response->best_under_gripper_fake[0]-10,
                                                   1000*grasp_response->best_under_gripper_fake[1]-60,
                                                   0, 0, 0, 0}, true); // 使用增量模式
    RCLCPP_INFO(
        node->get_logger(),
        "Moving to grasp position increment: x=%.3f, y=%.3f, z=%.3f",
        1000*grasp_response->best_under_gripper_fake[0],
        1000*grasp_response->best_under_gripper_fake[1],
        0.0
    );
    
    move_to_grasp_response = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_,
        move_to_grasp_request,
        node,
        std::chrono::seconds(10),
        "L/robot_move_cart"
    );
    
    if (!move_to_grasp_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move robot to grasp position: %s", move_to_grasp_response->message.c_str());
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒稳定
    // 把抓取方向与TCP x轴在xy平面上垂直（仅调整Z轴旋转，限制在±90°内）
    // best_under_gripper_fake[5] 是相对fake_gripper_frame的Z轴增量旋转（弧度）
    
    if (grasp_response->best_under_gripper_fake.size() >= 6) {
    // 1. 提取原始 Yaw（弧度转度）
    auto input_rz = grasp_response->best_under_gripper_fake[5] * 180.0 / M_PI;

    // 2. 施加垂直约束：直接加 90 度
    // 说明：对于二指夹爪，Target = Input + 90 和 Target = Input - 90 抓取姿态是一样的
    auto target_rz = input_rz + 90.0;

    // 3. 标准归一化到 [-180, 180]
    // 这一步处理超过 360 或 -360 的情况
    while (target_rz > 180.0) target_rz -= 360.0;
    while (target_rz < -180.0) target_rz += 360.0;

    // 4. 对称夹爪优化：将角度限制在 [-90, 90] 范围内
    // 如果角度超过 90 度，说明转 -80 度也能达到同样的抓取效果（180度对称性）
    if (target_rz > 90.0) {
        target_rz -= 180.0;
    } else if (target_rz < -90.0) {
        target_rz += 180.0;
    }

    // 最终赋值
    auto d_rz = target_rz;

    RCLCPP_INFO(node->get_logger(), 
        "Grasp Z-axis processing: Input=%.2f, Target_Perpendicular=%.2f (Normalized to +/-90)", 
        input_rz, d_rz);

        // return 0;
    //     if (std::fabs(d_rz) > 5.0) { // 超过5度才调整
    //         auto orient_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    //         node->gen_move_request(orient_request, {0, 0, 0, 0, 0, d_rz}, true);
    //         auto orient_resp = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
    //             node->L_robot_move_cart_client_, orient_request, node, std::chrono::seconds(10), "L/robot_move_cart_orient"
    //         );
    //         if (!orient_resp->success) {
    //             RCLCPP_WARN(node->get_logger(), "Z-axis orientation adjust failed: %s", orient_resp->message.c_str());
    //         } else {
    //             RCLCPP_INFO(node->get_logger(), "Z-axis orientation adjusted by %.2f degrees", d_rz);
    //             node->wait_until_done();
    //         }
    //     } else {
    //         RCLCPP_INFO(node->get_logger(), "Z-axis orientation delta small (%.2f); skip adjustment", d_rz);
    //     }
    // } else {
    //     RCLCPP_WARN(node->get_logger(), "best_under_gripper_fake size < 6; skip orientation adjustment");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待1秒稳定
    // 下降到抓取高度（假设先前在安全高度），使用深度或固定偏移
    descend_dist = -52.0; // mm，简单策略：下降53mm抓取
    move_down_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    node->gen_move_request(move_down_request, {0, 0, descend_dist, 0, 0, 0}, true);
    move_down_resp = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_, move_down_request, node, std::chrono::seconds(10), "L/robot_move_cart"
    );
    if (!move_down_resp->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to descend for grasp: %s", move_down_resp->message.c_str());
        return 1;
    }
    node->wait_until_done();

    // 关闭夹爪执行抓取
    gripper_close_request = std::make_shared<epg50_gripper_ros::srv::GripperCommand::Request>();
    gripper_close_request->slave_id = GRIPPER_ID_L; // 使用左夹爪
    gripper_close_request->command  = GRIPPER_SET;
    gripper_close_request->position = GRIPPER_CLOSE;
    gripper_close_request->speed    = 255;
    gripper_close_request->torque   = 255;
    gripper_close_resp = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
        node->gripper_command_client_, gripper_close_request, node, std::chrono::seconds(5), "gripper_close_L"
    );
    if (!gripper_close_resp->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to close gripper: %s", gripper_close_resp->message.c_str());
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Gripper closed; lifting object");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // 抓取后抬起
    lift_resp = ServiceCaller<robo_ctrl::srv::RobotMoveCart>::callServiceSync(
        node->L_robot_move_cart_client_, lift_request, node, std::chrono::seconds(10), "L/robot_move_cart"
    );
    if (!lift_resp->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to lift after grasp: %s", lift_resp->message.c_str());
        return 1;
    }
    node->wait_until_done();
    std::this_thread::sleep_for(std::chrono::seconds(1));
        L_act_j_request->target_joints = std::vector<double>({ 
        141.5165,
        -19.4858,
        -0.1194,
        -75.1131,
        -88.3089,
        -40.1285 });
    L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::seconds(5));
    gripper_close_request->position = GRIPPER_OPEN;
    gripper_close_request->speed    = 255;
    gripper_close_request->torque   = 255;
    gripper_close_resp = ServiceCaller<epg50_gripper_ros::srv::GripperCommand>::callServiceSync(
        node->gripper_command_client_, gripper_close_request, node, std::chrono::seconds(5), "gripper_close_L"
    );
    if (!gripper_close_resp->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to close gripper: %s", gripper_close_resp->message.c_str());
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Gripper closed; lifting object");
    std::this_thread::sleep_for(std::chrono::seconds(1));


    // 恢复物体更新
    L_act_j_request->target_joints = std::vector<double>({ 
        156.1977,
        -54.6986,
        -0.0274,
        -38.7380,
        -88.2650,
        -13.9097 });
    L_act_j_response = ServiceCaller<robo_ctrl::srv::RobotActJ>::callServiceSync(
        node->L_robot_act_j_client_,
        L_act_j_request,
        node,
        std::chrono::seconds(10),
        "L/robot_act_j"
    );
    std::this_thread::sleep_for(std::chrono::seconds(5));


    RCLCPP_INFO(node->get_logger(), "Grasp sequence completed successfully");

    rclcpp::shutdown();
    return 0;
}

void open_castbon() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Open castbon function called");
}

int L_fix(
    std::shared_ptr<RobotMain>& node,
    double& total_angle_diff,
    std::shared_ptr<robo_ctrl::srv::RobotMoveCart_Request>& fix_request,
    std::shared_ptr<robo_ctrl::srv::RobotMoveCart_Response>& fix_response,
    bool& retFlag
) {
    std::vector<double> orientation_increment;
    retFlag = true;
    {
        std::lock_guard<std::mutex> lock(node->L_robot_state_mutex_);
        if (node->L_robot_state_) {
             orientation_increment = { 0,
                                      0,
                                      0,
                                      180.0 - node->L_robot_state_->tcp_pose.rx,
                                      0.0 - node->L_robot_state_->tcp_pose.ry,
                                      -90.0 - node->L_robot_state_->tcp_pose.rz };
        }
        else {
            RCLCPP_ERROR(node->get_logger(), "L robot state is null, cannot fix orientation");
            return 1;
        }
    }

    // 计算总角度差值
    total_angle_diff =
        std::abs(orientation_increment[3]) + std::abs(orientation_increment[4]) + std::abs(orientation_increment[5]);

    RCLCPP_INFO(
        node->get_logger(),
        "Orientation correction: rx=%.3f, ry=%.3f, rz=%.3f, total_diff=%.3f",
        orientation_increment[3],
        orientation_increment[4],
        orientation_increment[5],
        total_angle_diff
    );

    fix_request = std::make_shared<robo_ctrl::srv::RobotMoveCart::Request>();
    node->gen_move_request(fix_request, orientation_increment, true); // 使用增量模式
    fix_request->velocity = 90;

    auto fix_future = node->L_robot_move_cart_client_->async_send_request(fix_request);
    if (rclcpp::spin_until_future_complete(node, fix_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service robot_move_cart for fixing orientation");
        return 1;
    }
    fix_response = fix_future.get();
    if (!fix_response->success) {
        RCLCPP_ERROR(node->get_logger(), "Failed to fix robot orientation: %s", fix_response->message.c_str());
        return 1;
    }
    retFlag = false;
    return {};
}

// f. u. c. k.  a. l. l.

// Q.E.D.