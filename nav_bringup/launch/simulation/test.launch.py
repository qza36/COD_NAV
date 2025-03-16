import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    fastlio_dir = get_package_share_directory('fast_lio')
    nav_bringup_dir = get_package_share_directory('nav_bringup')
    terrrain_analysis_dir = get_package_share_directory('terrain_analysis')
    fastlio_config_path = os.path.join(fastlio_dir, 'config')
    fast_lio_config_file = 'mid360.yaml'
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    use_sim_time = LaunchConfiguration('use_sim_time')
    load_nodes = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav_bringup_dir,'/launch','/simulation' ,'/simulation.launch.py'])
            ),
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[PathJoinSubstitution([fastlio_config_path, fast_lio_config_file]),
                            {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            Node(
                package='ign_sim_pointcloud_tool',
                executable='ign_sim_pointcloud_tool_node',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([terrrain_analysis_dir,'/launch/terrain_analysis_launch.py']),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )

        ]
    )
    return LaunchDescription([
        declare_use_sim_time,
        load_nodes
    ])