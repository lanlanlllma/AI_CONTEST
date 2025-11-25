from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('grab_detect')
    config_file = os.path.join(pkg_dir, 'config', 'grasp_detect.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=config_file,
            description='Path to config file'
        ),
        
        Node(
            package='grab_detect',
            executable='grasp_detect_node',
            name='grasp_detect_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            emulate_tty=True
        )
    ])
