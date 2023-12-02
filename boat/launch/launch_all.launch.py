from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="environment", executable="environment",   name="env_node"),
            Node(package="boat", executable="estimator",            name="estimator_node"),
            Node(package="boat", executable="controller",           name="controller_node"),
            Node(package="high_level", executable="high_level",     name="high_level_node"),
        ]
    )
