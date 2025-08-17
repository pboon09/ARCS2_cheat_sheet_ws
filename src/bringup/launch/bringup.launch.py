from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Rate at which to publish messages'
    )

    rate = LaunchConfiguration('rate')

    bringup_pkg = get_package_share_directory('bringup')

    rviz_config_path = os.path.join(bringup_pkg, 'rviz', 'rviz_example.rviz')

    topics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch/topics.launch.py')),
        launch_arguments=[
            ('rate', rate),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        rate_arg,

        rviz_node,
        topics_launch,
    ])