from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": "ur3e",
            "robot_ip": "127.0.0.1",
            "use_fake_hardware": "true",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "launch_rviz": "true",
        }.items()
    )

    run_hello_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ur3e_tutorials",
                executable="hello_joint_trajectory_controller",
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
        ]
    )

    return LaunchDescription([
        ur_driver_launch,
        run_hello_node,
    ])