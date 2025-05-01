import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    fastlio_dir = get_package_share_directory('fast_lio')
    lidar_localization_dir = get_package_share_directory('lidar_localization_ros2')
    livox_driver_dir = get_package_share_directory('livox_ros_driver2')
    bring_up_dir = get_package_share_directory('nav_bringup')
    fastlio_config_path = os.path.join(fastlio_dir, 'config')
    fast_lio_config_file = 'mid360.yaml'
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 定义节点和包含的launch文件
    load_nodes = GroupAction(
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['--z', '0.6', '--frame-id', 'chassis', '--child-frame-id', 'livox_frame',]
            ),
            Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in',  '/livox/lidar/pointcloud'),
                            ('scan', '/scan')],
                parameters=[{
                    'target_frame': 'chassis',
                    'transform_tolerance': 0.01,
                    'min_height': 0.2,
                    'max_height': 1.00,
                    'angle_min': -3.1416,  # -M_PI/2
                    'angle_max': 3.1416,  # M_PI/2
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.3333,
                    'range_min': 0.05,
                    'range_max': 5.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0
                }],
                name='pointcloud_to_laserscan'
            ),
            Node(
                package="fake_vel_transform",
                executable="fake_vel_transform_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[PathJoinSubstitution([fastlio_config_path, fast_lio_config_file]),
                            {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            Node(
                package='clear_costmap_caller',
                executable='clear_costmap_caller',
                output='screen'
            ),
             Node(
                 package= 'cod_serial',
                 executable= 'cod_serial',
                 output= 'screen'
             ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([livox_driver_dir, '/launch/msg_MID360_launch.py']),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([lidar_localization_dir, '/launch/lidar_localization.launch.py']),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([bring_up_dir,'/launch/nav_bringup.launch.py']),
                launch_arguments={'use_sim_time':use_sim_time}.items()
            )

        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        load_nodes
    ])