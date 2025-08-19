import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node

PKG = "hardwarebot"

def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)

    # robot_state_publisher (publishes /robot_description)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_share, "launch", "rsp.launch.py")]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "use_ros2_control": "true",
            "sim_mode": "false",   # keep real-hardware path in ros2_control.xacro
        }.items(),
    )

    # controller_manager with the same robot_description (xacro)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        PathJoinSubstitution([pkg_share, "description", "robot.urdf.xacro"]),
                        " use_ros2_control:=true",
                        " sim_mode:=false",
                    ]
                )
            },
            os.path.join(pkg_share, "config", "trajectory_controller.yaml"),
        ],
        output="screen",
    )

    # spawn controllers after controller_manager is up
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    bot_grip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bot_grip_controller"],
        output="screen",
    )
    top_grip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["top_grip_controller"],
        output="screen",
    )

    start_spawners_evt = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        bot_grip_controller_spawner,
                        top_grip_controller_spawner,
                    ],
                )
            ],
        )
    )

    # optional MoveIt + RViz
    start_moveit_arg = DeclareLaunchArgument(
        "start_moveit",
        default_value="true",
        description="Start MoveIt! and RViz (set false for headless).",
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_share, "launch", "moveit.launch.py")]
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
    )

    return LaunchDescription(
        [
            start_moveit_arg,
            rsp,
            ros2_control_node,
            start_spawners_evt,
            moveit,
        ]
    )