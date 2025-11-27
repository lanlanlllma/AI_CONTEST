#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class FakeGripperTFPublisher(Node):
    def __init__(self):
        super().__init__('fake_gripper_tf_publisher')
        
        # 声明参数
        self.declare_parameter('gripper_frame', 'Lgripper')  # 夹爪的坐标系
        self.declare_parameter('base_frame', 'Lrobot_base')  # 基座坐标系
        self.declare_parameter('fake_frame', 'fake_gripper_frame')  # 要发布的虚假坐标系名称
        self.declare_parameter('parent_frame', 'Lrobot_base')  # 父坐标系
        
        # 获取参数
        self.gripper_frame = self.get_parameter('gripper_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.fake_frame = self.get_parameter('fake_frame').value
        self.parent_frame = self.get_parameter('parent_frame').value
        
        # 初始化 TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.get_logger().info(
            f'Publishing fake TF "{self.fake_frame}" with position from "{self.gripper_frame}" '
            f'and orientation from "{self.base_frame}" in parent frame "{self.parent_frame}"'
        )
        
        # 创建定时器，10 Hz
        self.timer = self.create_timer(0.1, self.publish_fake_tf)
    
    def publish_fake_tf(self):
        try:
            # 等待 TF 可用
            # 获取夹爪相对于基座的变换（因为我们要在基座坐标系下发布）
            gripper_trans = self.tf_buffer.lookup_transform(
                self.base_frame,  # 改为从 base_frame 查询
                self.gripper_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # 基座相对于自己的变换是单位变换（位置 [0,0,0]，姿态单位四元数）
            # 不需要查询，直接使用单位姿态
            
            # 创建新的变换
            trans = TransformStamped()
            trans.header.stamp = self.get_clock().now().to_msg()
            trans.header.frame_id = self.parent_frame
            trans.child_frame_id = self.fake_frame
            
            # 使用夹爪的位置（已经是相对于 base_frame 的）
            trans.transform.translation.x = gripper_trans.transform.translation.x
            trans.transform.translation.y = gripper_trans.transform.translation.y
            trans.transform.translation.z = gripper_trans.transform.translation.z
            
            # 使用基座的姿态（单位四元数，因为我们希望姿态与 base_frame 一致）
            trans.transform.rotation.x = 0.0
            trans.transform.rotation.y = 0.0
            trans.transform.rotation.z = 0.0
            trans.transform.rotation.w = 1.0
            
            # 发布变换
            self.tf_broadcaster.sendTransform(trans)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF Error: {e}', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = FakeGripperTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()