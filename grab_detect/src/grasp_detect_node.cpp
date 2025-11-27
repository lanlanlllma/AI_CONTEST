#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "grab_detect/msg/grasp_result.hpp"
#include "grab_detect/srv/grasp_detect.hpp"
#include "grab_detect/srv/grasp_detect_trigger.hpp"
#include "grab_detect/srv/grasp_detect_trigger_mask.hpp"

#include <arpa/inet.h>
#include <cstring>
#include <jsoncpp/json/json.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

class GraspDetectNode: public rclcpp::Node {
public:
    GraspDetectNode(): Node("grasp_detect_node") {
        // 声明参数
        this->declare_parameter("server_host", "localhost");
        this->declare_parameter("server_port", 5555);
        this->declare_parameter("socket_timeout", 30.0);
        this->declare_parameter("trigger_topic", "/grasp_detect/trigger");
        this->declare_parameter("color_image_topic", "/color/image_raw");
        this->declare_parameter("depth_image_topic", "/depth/image_raw");
        this->declare_parameter("mask_image_topic", "");
        this->declare_parameter("service_name", "grasp_detect");
        this->declare_parameter("visualization_frame", "camera_link");
        this->declare_parameter("publish_visualization", true);
        this->declare_parameter("target_robot_frame", "Lrobot_base");
        this->declare_parameter("publish_grasp_tf", true);
        this->declare_parameter("fake_gripper_frame", "fake_gripper_frame");
        // this->declare_parameter("visualization_frame", "camera_frame");

        // 获取参数
        server_host_    = this->get_parameter("server_host").as_string();
        server_port_    = this->get_parameter("server_port").as_int();
        socket_timeout_ = this->get_parameter("socket_timeout").as_double();

        auto trigger_topic = this->get_parameter("trigger_topic").as_string();
        auto color_topic   = this->get_parameter("color_image_topic").as_string();
        auto depth_topic   = this->get_parameter("depth_image_topic").as_string();
        auto mask_topic    = this->get_parameter("mask_image_topic").as_string();
        auto service_name  = this->get_parameter("service_name").as_string();
        visualization_frame_ = this->get_parameter("visualization_frame").as_string();
        publish_visualization_ = this->get_parameter("publish_visualization").as_bool();
        target_robot_frame_ = this->get_parameter("target_robot_frame").as_string();
        publish_grasp_tf_ = this->get_parameter("publish_grasp_tf").as_bool();
        fake_gripper_frame_ = this->get_parameter("fake_gripper_frame").as_string();

        // 初始化 TF 组件
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // 创建可视化发布者
        if (publish_visualization_) {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/grasp_detect/markers", 10);
            RCLCPP_INFO(this->get_logger(), "Publishing visualization markers on /grasp_detect/markers");
        }

        // 创建订阅者
        trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            trigger_topic,
            10,
            std::bind(&GraspDetectNode::trigger_callback, this, std::placeholders::_1)
        );

        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_topic,
            10,
            std::bind(&GraspDetectNode::color_callback, this, std::placeholders::_1)
        );

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic,
            10,
            std::bind(&GraspDetectNode::depth_callback, this, std::placeholders::_1)
        );

        if (!mask_topic.empty()) {
            mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                mask_topic,
                10,
                std::bind(&GraspDetectNode::mask_callback, this, std::placeholders::_1)
            );
        }

        // 创建服务
        service_ = this->create_service<grab_detect::srv::GraspDetect>(
            service_name,
            std::bind(&GraspDetectNode::service_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // 创建 trigger 服务
        trigger_service_ = this->create_service<grab_detect::srv::GraspDetectTrigger>(
            service_name + "_trigger",
            std::bind(&GraspDetectNode::trigger_service_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // 创建 trigger_mask 服务
        trigger_mask_service_ = this->create_service<grab_detect::srv::GraspDetectTriggerMask>(
            service_name + "_trigger_mask",
            std::bind(
                &GraspDetectNode::trigger_mask_service_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        RCLCPP_INFO(this->get_logger(), "GraspDetectNode initialized");
        RCLCPP_INFO(this->get_logger(), "Server: %s:%d", server_host_.c_str(), server_port_);
        RCLCPP_INFO(this->get_logger(), "Listening on trigger topic: %s", trigger_topic.c_str());
    }

private:
    void trigger_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        RCLCPP_INFO(this->get_logger(), "Received trigger, processing grasp detection...");

        if (!latest_color_ || !latest_depth_) {
            RCLCPP_WARN(this->get_logger(), "Missing color or depth image!");
            return;
        }

        // 执行检测
        auto request         = std::make_shared<grab_detect::srv::GraspDetect::Request>();
        request->color_image = *latest_color_;
        request->depth_image = *latest_depth_;
        if (latest_mask_) {
            request->mask_image = *latest_mask_;
        }

        auto response = std::make_shared<grab_detect::srv::GraspDetect::Response>();
        process_detection(request, response);

        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Detection successful! Found %d grasps", response->num_grasps);
            // 可视化结果
            if (publish_visualization_) {
                publish_grasp_markers(response->grasps);
            }
            // 发布最高分抓取点的 TF
            if (publish_grasp_tf_ && !response->grasps.empty()) {
                publish_best_grasp_tf(response->grasps);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Detection failed: %s", response->message.c_str());
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
        std::shared_ptr<grab_detect::srv::GraspDetect::Response> response
    ) {
        RCLCPP_INFO(this->get_logger(), "Service called");
        process_detection(request, response);
    }

    void trigger_service_callback(
        const std::shared_ptr<grab_detect::srv::GraspDetectTrigger::Request> /*request*/,
        std::shared_ptr<grab_detect::srv::GraspDetectTrigger::Response> response
    ) {
        RCLCPP_INFO(this->get_logger(), "Trigger service called");

        if (!latest_color_ || !latest_depth_) {
            response->success    = false;
            response->num_grasps = 0;
            response->message    = "Missing color or depth image! Please ensure camera topics are publishing.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // 构建请求
        auto detect_request         = std::make_shared<grab_detect::srv::GraspDetect::Request>();
        detect_request->color_image = *latest_color_;
        detect_request->depth_image = *latest_depth_;
        if (latest_mask_) {
            detect_request->mask_image = *latest_mask_;
        }

        // 执行检测
        auto detect_response = std::make_shared<grab_detect::srv::GraspDetect::Response>();
        process_detection(detect_request, detect_response);

        // 复制结果
        response->success    = detect_response->success;
        response->num_grasps = detect_response->num_grasps;
        response->message    = detect_response->message;
        response->grasps     = detect_response->grasps;

        // 可视化结果
        if (response->success && publish_visualization_) {
            publish_grasp_markers(response->grasps);
        }
        // 发布最高分抓取点的 TF
        if (response->success && publish_grasp_tf_ && !response->grasps.empty()) {
            publish_best_grasp_tf(response->grasps);
        }
    }

    void trigger_mask_service_callback(
        const std::shared_ptr<grab_detect::srv::GraspDetectTriggerMask::Request> request,
        std::shared_ptr<grab_detect::srv::GraspDetectTriggerMask::Response> response
    ) {
        RCLCPP_INFO(this->get_logger(), "Trigger mask service called");

        if (!latest_color_ || !latest_depth_) {
            response->success    = false;
            response->num_grasps = 0;
            response->message    = "Missing color or depth image! Please ensure camera topics are publishing.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // 构建请求，使用请求中的 mask 图像
        auto detect_request         = std::make_shared<grab_detect::srv::GraspDetect::Request>();
        detect_request->color_image = *latest_color_;
        detect_request->depth_image = *latest_depth_;
        detect_request->mask_image  = request->mask_image;

        // 执行检测
        auto detect_response = std::make_shared<grab_detect::srv::GraspDetect::Response>();
        process_detection(detect_request, detect_response);

        // 复制结果
        response->success    = detect_response->success;
        response->num_grasps = detect_response->num_grasps;
        response->message    = detect_response->message;
        response->grasps     = detect_response->grasps;

        // 计算最佳抓取在 fake_gripper_frame 下的坐标
        if (response->success && !response->grasps.empty()) {
            RCLCPP_INFO(this->get_logger(), 
                "Attempting to transform best grasp from %s to %s",
                visualization_frame_.c_str(), fake_gripper_frame_.c_str());
            
            try {
                // 找到分数最高的抓取点
                auto best_grasp_it = std::max_element(response->grasps.begin(), response->grasps.end(),
                    [](const grab_detect::msg::GraspResult& a, const grab_detect::msg::GraspResult& b) {
                        return a.score < b.score;
                    });
                const auto& best_grasp = *best_grasp_it;

                RCLCPP_INFO(this->get_logger(), 
                    "Best grasp in %s: pos=[%.3f, %.3f, %.3f]",
                    visualization_frame_.c_str(),
                    best_grasp.translation[0],
                    best_grasp.translation[1],
                    best_grasp.translation[2]);

                // 将旋转矩阵转换为四元数
                tf2::Matrix3x3 rot_matrix(
                    best_grasp.rotation_matrix[0], best_grasp.rotation_matrix[1], best_grasp.rotation_matrix[2],
                    best_grasp.rotation_matrix[3], best_grasp.rotation_matrix[4], best_grasp.rotation_matrix[5],
                    best_grasp.rotation_matrix[6], best_grasp.rotation_matrix[7], best_grasp.rotation_matrix[8]
                );
                tf2::Quaternion quat;
                rot_matrix.getRotation(quat);

                // 创建 grasp 在 camera 坐标系的 pose
                geometry_msgs::msg::PoseStamped grasp_pose_camera;
                grasp_pose_camera.header.frame_id = visualization_frame_;
                grasp_pose_camera.header.stamp = this->now();
                grasp_pose_camera.pose.position.x = best_grasp.translation[0];
                grasp_pose_camera.pose.position.y = best_grasp.translation[1];
                grasp_pose_camera.pose.position.z = best_grasp.translation[2];
                grasp_pose_camera.pose.orientation.x = quat.x();
                grasp_pose_camera.pose.orientation.y = quat.y();
                grasp_pose_camera.pose.orientation.z = quat.z();
                grasp_pose_camera.pose.orientation.w = quat.w();

                RCLCPP_INFO(this->get_logger(), "Looking up transform...");
                
                // 查询从 fake_gripper_frame 到 visualization_frame 的变换
                geometry_msgs::msg::TransformStamped transform_gripper_to_camera;
                transform_gripper_to_camera = tf_buffer_->lookupTransform(
                    fake_gripper_frame_, visualization_frame_,
                    tf2::TimePointZero, std::chrono::milliseconds(500));

                RCLCPP_INFO(this->get_logger(), "Transform found, applying...");
                
                // 转换到 fake_gripper_frame 坐标系
                geometry_msgs::msg::PoseStamped grasp_pose_gripper;
                tf2::doTransform(grasp_pose_camera, grasp_pose_gripper, transform_gripper_to_camera);

                // 将四元数转换为欧拉角 (rx, ry, rz)
                tf2::Quaternion q(
                    grasp_pose_gripper.pose.orientation.x,
                    grasp_pose_gripper.pose.orientation.y,
                    grasp_pose_gripper.pose.orientation.z,
                    grasp_pose_gripper.pose.orientation.w
                );
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                // 填充响应
                response->best_under_gripper_fake = {
                    static_cast<float>(grasp_pose_gripper.pose.position.x),
                    static_cast<float>(grasp_pose_gripper.pose.position.y),
                    static_cast<float>(grasp_pose_gripper.pose.position.z),
                    static_cast<float>(roll),
                    static_cast<float>(pitch),
                    static_cast<float>(yaw)
                };

                RCLCPP_INFO(this->get_logger(), 
                    "Best grasp in %s: pos=[%.3f, %.3f, %.3f], rpy=[%.3f, %.3f, %.3f]",
                    fake_gripper_frame_.c_str(),
                    grasp_pose_gripper.pose.position.x,
                    grasp_pose_gripper.pose.position.y,
                    grasp_pose_gripper.pose.position.z,
                    roll, pitch, yaw);

            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), 
                    "Failed to transform grasp to %s frame: %s. Setting best_under_gripper_fake to empty.",
                    fake_gripper_frame_.c_str(), ex.what());
                response->best_under_gripper_fake.clear();
            }
        } else {
            response->best_under_gripper_fake.clear();
        }

        // 可视化结果
        if (response->success && publish_visualization_) {
            publish_grasp_markers(response->grasps);
        }
        // 发布最高分抓取点的 TF
        if (response->success && publish_grasp_tf_ && !response->grasps.empty()) {
            publish_best_grasp_tf(response->grasps);
        }
    }

    void publish_best_grasp_tf(const std::vector<grab_detect::msg::GraspResult>& grasps) {
        if (grasps.empty()) return;

        // 找到分数最高的抓取点
        auto best_grasp_it = std::max_element(grasps.begin(), grasps.end(),
            [](const grab_detect::msg::GraspResult& a, const grab_detect::msg::GraspResult& b) {
                return a.score < b.score;
            });

        const auto& best_grasp = *best_grasp_it;
        RCLCPP_INFO(this->get_logger(), "Best grasp score: %.3f", best_grasp.score);

        // 将旋转矩阵转换为四元数
        tf2::Matrix3x3 rot_matrix(
            best_grasp.rotation_matrix[0], best_grasp.rotation_matrix[1], best_grasp.rotation_matrix[2],
            best_grasp.rotation_matrix[3], best_grasp.rotation_matrix[4], best_grasp.rotation_matrix[5],
            best_grasp.rotation_matrix[6], best_grasp.rotation_matrix[7], best_grasp.rotation_matrix[8]
        );
        tf2::Quaternion quat;
        rot_matrix.getRotation(quat);

        // 创建从 camera_link 到 grasp_target 的变换
        geometry_msgs::msg::TransformStamped transform_camera_to_grasp;
        transform_camera_to_grasp.header.stamp = this->now();
        transform_camera_to_grasp.header.frame_id = visualization_frame_;
        transform_camera_to_grasp.child_frame_id = "grasp_target_camera";
        transform_camera_to_grasp.transform.translation.x = best_grasp.translation[0];
        transform_camera_to_grasp.transform.translation.y = best_grasp.translation[1];
        transform_camera_to_grasp.transform.translation.z = best_grasp.translation[2];
        transform_camera_to_grasp.transform.rotation.x = quat.x();
        transform_camera_to_grasp.transform.rotation.y = quat.y();
        transform_camera_to_grasp.transform.rotation.z = quat.z();
        transform_camera_to_grasp.transform.rotation.w = quat.w();

        try {
            // 查询从 target_robot_frame 到 visualization_frame 的变换
            geometry_msgs::msg::TransformStamped transform_robot_to_camera;
            transform_robot_to_camera = tf_buffer_->lookupTransform(
                target_robot_frame_, visualization_frame_,
                tf2::TimePointZero, std::chrono::milliseconds(500));

            // 计算从 robot 到 grasp_target 的变换
            // 先创建 grasp 在 camera 坐标系的 pose
            geometry_msgs::msg::PoseStamped grasp_pose_camera;
            grasp_pose_camera.header.frame_id = visualization_frame_;
            grasp_pose_camera.header.stamp = this->now();
            grasp_pose_camera.pose.position.x = best_grasp.translation[0];
            grasp_pose_camera.pose.position.y = best_grasp.translation[1];
            grasp_pose_camera.pose.position.z = best_grasp.translation[2];
            grasp_pose_camera.pose.orientation.x = quat.x();
            grasp_pose_camera.pose.orientation.y = quat.y();
            grasp_pose_camera.pose.orientation.z = quat.z();
            grasp_pose_camera.pose.orientation.w = quat.w();

            // 转换到机器人坐标系
            geometry_msgs::msg::PoseStamped grasp_pose_robot;
            tf2::doTransform(grasp_pose_camera, grasp_pose_robot, transform_robot_to_camera);

            // 发布从 robot 到 grasp_target 的 TF
            geometry_msgs::msg::TransformStamped transform_robot_to_grasp;
            transform_robot_to_grasp.header.stamp = this->now();
            transform_robot_to_grasp.header.frame_id = target_robot_frame_;
            transform_robot_to_grasp.child_frame_id = "grasp_target";
            transform_robot_to_grasp.transform.translation.x = grasp_pose_robot.pose.position.x;
            transform_robot_to_grasp.transform.translation.y = grasp_pose_robot.pose.position.y;
            transform_robot_to_grasp.transform.translation.z = grasp_pose_robot.pose.position.z;
            transform_robot_to_grasp.transform.rotation = grasp_pose_robot.pose.orientation;

            tf_broadcaster_->sendTransform(transform_robot_to_grasp);

            RCLCPP_INFO(this->get_logger(), 
                "Published /grasp_target TF in %s frame: [%.3f, %.3f, %.3f]",
                target_robot_frame_.c_str(),
                grasp_pose_robot.pose.position.x,
                grasp_pose_robot.pose.position.y,
                grasp_pose_robot.pose.position.z);

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), 
                "Failed to transform grasp to %s frame: %s",
                target_robot_frame_.c_str(), ex.what());
        }
    }

    void publish_grasp_markers(const std::vector<grab_detect::msg::GraspResult>& grasps) {
        visualization_msgs::msg::MarkerArray marker_array;
        auto stamp = this->now();

        for (size_t i = 0; i < (grasps.size() > 0 ? 1 : 0 ); ++i) {
            const auto& grasp = grasps[i];

            // 将旋转矩阵转换为四元数
            tf2::Matrix3x3 rot_matrix(
                grasp.rotation_matrix[0], grasp.rotation_matrix[1], grasp.rotation_matrix[2],
                grasp.rotation_matrix[3], grasp.rotation_matrix[4], grasp.rotation_matrix[5],
                grasp.rotation_matrix[6], grasp.rotation_matrix[7], grasp.rotation_matrix[8]
            );
            tf2::Quaternion quat;
            rot_matrix.getRotation(quat);

            // 创建坐标轴标记（表示抓取姿态）
            // x轴 - 蓝色（抓取方向）
            visualization_msgs::msg::Marker x_axis;
            x_axis.header.frame_id = visualization_frame_;
            x_axis.header.stamp = stamp;
            x_axis.ns = "grasp_axes";
            x_axis.id = i * 10;
            x_axis.type = visualization_msgs::msg::Marker::ARROW;
            x_axis.action = visualization_msgs::msg::Marker::ADD;
            x_axis.pose.position.x = grasp.translation[0];
            x_axis.pose.position.y = grasp.translation[1];
            x_axis.pose.position.z = grasp.translation[2];
            x_axis.pose.orientation.x = quat.x();
            x_axis.pose.orientation.y = quat.y();
            x_axis.pose.orientation.z = quat.z();
            x_axis.pose.orientation.w = quat.w();
            x_axis.scale.x = 0.1;  // 轴长度
            x_axis.scale.y = 0.01; // 箭头宽度
            x_axis.scale.z = 0.01;
            x_axis.color.r = 0.0;
            x_axis.color.g = 0.0;
            x_axis.color.b = 1.0;
            x_axis.color.a = 0.8;
            x_axis.lifetime = rclcpp::Duration::from_seconds(3);
            marker_array.markers.push_back(x_axis);

            // Y轴 - 绿色
            tf2::Quaternion y_rot;
            y_rot.setRPY(0, 0, M_PI/2);
            tf2::Quaternion y_quat = quat * y_rot;
            visualization_msgs::msg::Marker y_axis = x_axis;
            y_axis.id = i * 10 + 1;
            y_axis.pose.orientation.x = y_quat.x();
            y_axis.pose.orientation.y = y_quat.y();
            y_axis.pose.orientation.z = y_quat.z();
            y_axis.pose.orientation.w = y_quat.w();
            y_axis.color.r = 1.0;
            y_axis.color.g = 0.0;
            y_axis.color.b = 0.0;
            marker_array.markers.push_back(y_axis);

            // Z轴 - 红色
            tf2::Quaternion z_rot;
            z_rot.setRPY(0, -M_PI/2, 0);
            tf2::Quaternion z_quat = quat * z_rot;
            visualization_msgs::msg::Marker z_axis = x_axis;
            z_axis.id = i * 10 + 2;
            z_axis.pose.orientation.x = z_quat.x();
            z_axis.pose.orientation.y = z_quat.y();
            z_axis.pose.orientation.z = z_quat.z();
            z_axis.pose.orientation.w = z_quat.w();
            z_axis.color.r = 0.0;
            z_axis.color.g = 1.0;
            z_axis.color.b = 0.0;
            marker_array.markers.push_back(z_axis);

            // 创建夹爪框架标记（立方体表示抓取区域）
            visualization_msgs::msg::Marker gripper_box;
            gripper_box.header.frame_id = visualization_frame_;
            gripper_box.header.stamp = stamp;
            gripper_box.ns = "grasp_boxes";
            gripper_box.id = i * 10 + 3;
            gripper_box.type = visualization_msgs::msg::Marker::CUBE;
            gripper_box.action = visualization_msgs::msg::Marker::ADD;
            gripper_box.pose.position.x = grasp.translation[0];
            gripper_box.pose.position.y = grasp.translation[1];
            gripper_box.pose.position.z = grasp.translation[2];
            gripper_box.pose.orientation.x = quat.x();
            gripper_box.pose.orientation.y = quat.y();
            gripper_box.pose.orientation.z = quat.z();
            gripper_box.pose.orientation.w = quat.w();
            gripper_box.scale.x = grasp.depth*4;
            gripper_box.scale.y = grasp.width*4;
            gripper_box.scale.z = grasp.height*4;
            gripper_box.color.r = 0.0;
            gripper_box.color.g = 1.0;
            gripper_box.color.b = 1.0;
            gripper_box.color.a = 0.3;
            gripper_box.lifetime = rclcpp::Duration::from_seconds(3);
            marker_array.markers.push_back(gripper_box);

            // 添加文本标签显示得分
            visualization_msgs::msg::Marker text;
            text.header.frame_id = visualization_frame_;
            text.header.stamp = stamp;
            text.ns = "grasp_scores";
            text.id = i * 10 + 4;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = grasp.translation[0];
            text.pose.position.y = grasp.translation[1];
            text.pose.position.z = grasp.translation[2] + 0.05; // 稍微上移
            text.scale.z = 0.02; // 文字大小
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;
            text.text = "Score: " + std::to_string(grasp.score).substr(0, 4);
            text.lifetime = rclcpp::Duration::from_seconds(3);
            // marker_array.markers.push_back(text);
        }

        // 发布标记
        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published %zu grasp visualization markers", marker_array.markers.size());
    }

    void process_detection(
        const std::shared_ptr<grab_detect::srv::GraspDetect::Request> request,
        std::shared_ptr<grab_detect::srv::GraspDetect::Response> response
    ) {
        int sock = -1;
        try {
            // 创建socket
            sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock < 0) {
                throw std::runtime_error("Failed to create socket");
            }

            // 设置超时
            struct timeval timeout;
            timeout.tv_sec  = static_cast<int>(socket_timeout_);
            timeout.tv_usec = static_cast<int>((socket_timeout_ - timeout.tv_sec) * 1e6);
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

            // 连接服务器
            struct sockaddr_in server_addr;
            server_addr.sin_family = AF_INET;
            server_addr.sin_port   = htons(server_port_);
            inet_pton(AF_INET, server_host_.c_str(), &server_addr.sin_addr);

            RCLCPP_INFO(this->get_logger(), "Connecting to %s:%d...", server_host_.c_str(), server_port_);

            if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
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

        } catch (const std::exception& e) {
            if (sock >= 0) {
                close(sock);
            }
            response->success    = false;
            response->num_grasps = 0;
            response->message    = std::string("Error: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void send_image(int sock, const sensor_msgs::msg::Image& img) {
        // 转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img, img.encoding);
        } catch (cv_bridge::Exception& e) {
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

    void receive_results(int sock, std::shared_ptr<grab_detect::srv::GraspDetect::Response> response) {
        // 接收结果大小
        uint32_t result_size;
        if (recv(sock, &result_size, sizeof(result_size), 0) != sizeof(result_size)) {
            throw std::runtime_error("Failed to receive result size");
        }
        result_size = ntohl(result_size);

        RCLCPP_INFO(this->get_logger(), "Receiving results: %u bytes", result_size);

        // 接收结果数据
        std::vector<char> buffer(result_size);
        size_t received = 0;
        while (received < result_size) {
            ssize_t n = recv(sock, buffer.data() + received, result_size - received, 0);
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
        response->success    = root["success"].asBool();
        response->num_grasps = root["num_grasps"].asInt();
        response->message    = root.get("message", "").asString();

        if (response->success && root.isMember("grasps")) {
            const Json::Value& grasps = root["grasps"];
            for (const auto& grasp_json: grasps) {
                grab_detect::msg::GraspResult grasp;

                grasp.score  = grasp_json["score"].asDouble();
                grasp.width  = grasp_json["width"].asDouble();
                grasp.height = grasp_json["height"].asDouble();
                grasp.depth  = grasp_json["depth"].asDouble();

                const Json::Value& trans = grasp_json["translation"];
                for (int i = 0; i < 3; i++) {
                    grasp.translation[i] = trans[i].asDouble();
                }

                const Json::Value& rot = grasp_json["rotation_matrix"];
                for (int i = 0; i < 9; i++) {
                    int row                  = i / 3;
                    int col                  = i % 3;
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
    rclcpp::Service<grab_detect::srv::GraspDetectTrigger>::SharedPtr trigger_service_;
    rclcpp::Service<grab_detect::srv::GraspDetectTriggerMask>::SharedPtr trigger_mask_service_;

    sensor_msgs::msg::Image::SharedPtr latest_color_;
    sensor_msgs::msg::Image::SharedPtr latest_depth_;
    sensor_msgs::msg::Image::SharedPtr latest_mask_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::string visualization_frame_;
    bool publish_visualization_;

    // TF 相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    std::string target_robot_frame_;
    std::string fake_gripper_frame_;
    bool publish_grasp_tf_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraspDetectNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
