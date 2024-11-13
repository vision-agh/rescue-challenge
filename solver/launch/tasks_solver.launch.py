from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    example_node = Node(
        package='solver',
        executable='example',
        name='example',
        output='screen'
    )

    return LaunchDescription([
        example_node
    ])
