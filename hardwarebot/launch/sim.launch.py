import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():

    package_name='hardwarebot'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    moveit = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','moveit.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        #'minimal.world',
        #'built.world',
        'one_less_box.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Launch arguments for spawn pose
    x_arg = DeclareLaunchArgument('x', default_value='1.0', description='X position of the robot')
    y_arg = DeclareLaunchArgument('y', default_value='-0.035', description='Y position of the robot')
    z_arg = DeclareLaunchArgument('z', default_value='0.15', description='Z position of the robot')

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-x', LaunchConfiguration('x'),
                                   '-y', LaunchConfiguration('y'),
                                   '-z', LaunchConfiguration('z')],
                        output='screen')                     

    #trajectory_controllers.yaml for gazebo
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    bot_grip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bot_grip_controller"],
    )

    top_grip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["top_grip_controller"],
    )

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    # A delay action, execute the spawn_entity after period = n sec.
    spawn_entity_w_delay = TimerAction(
        period=2.0,  
        actions=[spawn_entity]
    )

    # Launch
    return LaunchDescription([
        rsp,
        moveit,
        world_arg,
        gazebo,
        x_arg,
        y_arg,
        z_arg,
        spawn_entity_w_delay,
        joint_state_broadcaster_spawner,
        bot_grip_controller_spawner,
        top_grip_controller_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])