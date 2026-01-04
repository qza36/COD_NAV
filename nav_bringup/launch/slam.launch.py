import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bring_up_dir = get_package_share_directory('nav_bringup')
    rviz_config_file = os.path.join(bring_up_dir, 'rviz', 'cod_nav.rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=os.path.join(bring_up_dir,'params','mapper_params_async.yaml')
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',default_value=os.path.join(bring_up_dir,'params','nav2_params.yaml')
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    load_nodes = GroupAction(
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time}
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    "--x", "0.0", "--y", "0.0", "--z", "0.0",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "map", "--child-frame-id", "odom",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    "--x", "0.0", "--y", "0.0", "--z", "0.0",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "chassis", "--child-frame-id", "front_rplidar_a2",
                ],
            ),
            Node(
                package="fake_vel_transform",
                executable="fake_vel_transform_node",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bring_up_dir,'launch','navigation_launch.py')),
                launch_arguments={
                                  'use_sim_time': use_sim_time,
                                  'autostart': "true",
                                  'params_file': nav2_params_file,
                                  'use_composition': 'False',
                                  'use_respawn': 'False',
                                  'container_name': 'nav2_container'}.items()
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d',rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_nav2_params_file,
        load_nodes
    ])
