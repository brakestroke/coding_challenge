from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the hunter_se_empty_world.launch.py file
    hunter_launch_path = os.path.join(
        get_package_share_directory('hunter_se_gazebo'),
        'launch',
        'hunter_se_empty_world.launch.py',
        
    )

    # Include the Hunter SE Gazebo launch file
    hunter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hunter_launch_path),
        launch_arguments={'use_rviz': 'false'}.items()
    )

    # Node to run diff_cmd_publisher
    diff_cmd_publisher_node = Node(
        package='diff2ackermann',
        executable='diff_cmd_publisher',
        name='diff_cmd_publisher',
        output='screen'
    )

    # Node to run diff_to_ackermann
    diff_to_ackermann_node = Node(
        package='diff2ackermann',
        executable='diff_to_ackermann',
        name='diff_to_ackermann',
        output='screen'
    )

    return LaunchDescription([
        hunter_launch,
        diff_cmd_publisher_node,
        diff_to_ackermann_node
    ])