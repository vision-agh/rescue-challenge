import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Command to launch PX4 SITL with the gz_x500 model
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500'],
        cwd=os.path.expanduser('~/PX4-Autopilot'),
        output='screen'
    )

    # Command to run the Micro-ROS agent
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888'],
        output='screen'
    )

    # Node to run the simple_flight package's circle_flight node
    square_flight = Node(
        package='simple_flight',
        executable='square_flight',
        name='square_flight',
        output='screen'
    )

    return LaunchDescription([
        px4_sitl,
        micro_ros_agent,
        square_flight,
    ])
