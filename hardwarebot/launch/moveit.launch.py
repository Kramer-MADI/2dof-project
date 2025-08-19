import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_desc_pkg = get_package_share_directory('hardwarebot')

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="hardwarebot")
        .robot_description(file_path="description/robot.urdf.xacro")
        .robot_description_semantic(file_path="description/robot.srdf")
        .trajectory_execution(file_path="moveit/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="moveit/kinematics.yaml")
        .joint_limits(file_path="moveit/joint_limits.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path="moveit/pilz_cartesian_limits.yaml")
        .sensors_3d(file_path="moveit/sensors_3d.yaml")
        .to_moveit_configs()
    )

    # MoveIt move_group node
    config_dict = moveit_config.to_dict()
    config_dict.update({'use_sim_time': use_sim_time})
    move_group_node = Node(
         package="moveit_ros_move_group",
         executable="move_group",
         output="screen",
         parameters=[config_dict],
         arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz node configured for MoveIt
    rviz_config_path = os.path.join(get_package_share_directory('hardwarebot'), "config", "gripperbot.rviz")
    rviz_node = Node(
          package="rviz2",
          executable="rviz2",
          name="rviz2",
          output="screen",
          arguments=["-d", rviz_config_path],
          parameters=[
               {'use_sim_time': use_sim_time},
               moveit_config.robot_description,
               moveit_config.robot_description_semantic,
               moveit_config.planning_pipelines,
               moveit_config.robot_description_kinematics,
          ],
     )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        move_group_node,
        rviz_node
    ])