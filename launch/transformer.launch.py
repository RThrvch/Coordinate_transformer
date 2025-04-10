import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'coordinate_transformer'
    config_file = 'sample_config.yaml'

    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        config_file)

    transformer_node = Node(
        package=pkg_name,
        executable='test_transformer_node',
        name='transformer_node',
        output='screen',
        parameters=[config_path]
    )

    return LaunchDescription([
        transformer_node
    ]) 