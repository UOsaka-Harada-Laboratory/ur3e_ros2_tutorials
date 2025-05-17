from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ur3e_tutorials",
            executable="hello_moveit_action",
            output="screen",
            parameters=[{"use_sim_time": False}],
        )
    ])