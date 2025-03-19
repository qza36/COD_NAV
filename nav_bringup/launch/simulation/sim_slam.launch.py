import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    fastlio_dir = get_package_share_directory('fast_lio')
    # 配置文件路径
    fastlio_config_path = os.path.join(fastlio_dir, 'config')
    fast_lio_config_file = 'mid360.yaml'

    slam_params_file = LaunchConfiguration('slam_params_file')


# 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("nav_bringup"),
                                   'params', 'mapper_params_localization.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 定义节点和包含的launch文件
    load_nodes = GroupAction(
        actions=[
            Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in',  '/livox/lidar/pointcloud'),
                            ('scan', '/scan')],
                parameters=[{
                    'target_frame': 'chassis',
                    'transform_tolerance': 0.01,
                    'min_height': 0.0,
                    'max_height': 0.65,
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
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[PathJoinSubstitution([fastlio_config_path, fast_lio_config_file]),
                            {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([lidar_localization_dir, '/launch/sim_lidar_localization.launch.py']),
            #     launch_arguments={'use_sim_time': use_sim_time}.items()
            # ),
            Node(
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time}
                ],
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                output='screen')
        ]
    )
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(load_nodes)

    return ld