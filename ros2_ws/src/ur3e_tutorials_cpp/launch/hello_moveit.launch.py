from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
            "launch_rviz": "false",
        }.items()
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": "ur3e",
            "launch_rviz": "true",
        }.items()
    )

    run_hello_moveit = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ur3e_tutorials_cpp",
                executable="hello_moveit",
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
        ]
    )

    return LaunchDescription([
        ur_driver_launch,
        ur_moveit_launch,
        run_hello_moveit,
    ])