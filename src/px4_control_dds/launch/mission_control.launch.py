"""micro-XRCE-DDS 圆筒侦察任务控制器 Launch 文件"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动 mission_controller_dds 节点, 加载 YAML 参数."""
    pkg_share = get_package_share_directory('px4_control_dds')
    param_file = os.path.join(pkg_share, 'config', 'mission_params.yaml')

    return LaunchDescription([
        Node(
            package='px4_control_dds',
            executable='mission_control_dds_node',
            name='mission_controller_dds',
            output='screen',
            emulate_tty=True,
            parameters=[param_file],
        )
    ])
