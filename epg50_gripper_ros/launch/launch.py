from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for EPG50 gripper'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug output'
    )
    
    default_slave_id_arg = DeclareLaunchArgument(
        'default_slave_id',
        default_value='10',
        description='Default Modbus slave ID (十进制 10 = 十六进制 0x0A)'
    )

    # 创建节点
    gripper_node = Node(
        package='epg50_gripper_ros',
        executable='epg50_gripper_node',
        name='epg50_gripper',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'debug': LaunchConfiguration('debug'),
            'default_slave_id': LaunchConfiguration('default_slave_id'),
        }],
        output='screen'
    )

    # 返回启动描述
    return LaunchDescription([
        port_arg,
        debug_arg,
        default_slave_id_arg,
        gripper_node
    ])