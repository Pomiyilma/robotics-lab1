from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    diffdrive_share = get_package_share_directory('diffdrive_bot')
    control_launch = os.path.join(
        diffdrive_share, 'launch', 'control.launch.py'
    )

    side_length = LaunchConfiguration('side_length')

    return LaunchDescription([
        DeclareLaunchArgument(
            'side_length',
            default_value='0.5',
            description='Length of each side of the square in metres'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch)
        ),

        Node(
            package='my_robot_nodes',
            executable='rotate_server',
            name='rotate_server',
            output='screen'
        ),

        Node(
            package='my_robot_nodes',
            executable='square_driver',
            name='square_driver',
            output='screen',
            parameters=[{'side_length': side_length}]
        ),
    ])
