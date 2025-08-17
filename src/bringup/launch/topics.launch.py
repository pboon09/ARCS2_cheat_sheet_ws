from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Rate at which to publish messages'
    )

    rate = LaunchConfiguration('rate')

    timer_publisher = Node(
        package='topics',
        executable='timer_publisher.py',
        name='timer_publisher',
        output='screen',
        parameters=[{'rate': rate}]
    )

    counter_subscriber = Node(
        package='topics',
        executable='counter_subscriber.py',
        name='counter_subscriber',
        output='screen',
    )

    return LaunchDescription([
        rate_arg,

        timer_publisher,
        counter_subscriber,
    ])