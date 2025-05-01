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
    livox_driver_dir = get_package_share_directory('livox_ros_driver2')
    bringup_dir = get_package_share_directory('nav_bringup')
    config_dir = os.path.join(bringup_dir, 'params')
    config_basename = 'carto.localization.lua'
    resolution = '0.05'
    publish_period_sec = '0.5'

    cartographer_map = os.path.join(bringup_dir,'map', 'cartographer_map.pbstream')

    # 配置文件路径
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('nav_bringup'), 'urdf', 'carto.xacro')])

    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 定义节点和包含的launch文件
    load_nodes = GroupAction(
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_description
                }],
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='tf_static_publisher',
                output='screen',
                arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link',]
            ),
            # Node(
            #     package='tf2_ros',
            #     executable='static_transform_publisher',
            #     name='tf_static_publisher1',
            #     output='screen',
            #     arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'base_link',]
            # ),
            # Node(
            #     package='cartographer_ros',
            #     executable='cartographer_node',
            #     name='cartographer_node',
            #     output='screen',
            #     parameters=[{'use_sim_time': use_sim_time}],
            #     arguments=[
            #         '-configuration_directory', config_dir,
            #         '-configuration_basename', config_basename,
            #     ]
            # ),
            # Node(
            #     package='cartographer_ros',
            #     executable='cartographer_occupancy_grid_node',
            #     name='cartographer_occupancy_grid_node',
            #     output='screen',
            #     parameters=[{'use_sim_time': use_sim_time}],
            #     arguments=[
            #         '-resolution', resolution,
            #         '-publish_period_sec', publish_period_sec
            #     ]
            # ),
            Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in',  '/livox/lidar/pointcloud'),
                            ('scan', '/scan')],
                parameters=[{
                    'target_frame': 'base_link',
                    'transform_tolerance': 0.01,
                    'min_height': 0.2,
                    'max_height': 1.00,
                    'angle_min': -3.1416,  # -M_PI/2
                    'angle_max': 3.1416,  # M_PI/2
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.3333,
                    'range_min': 0.05,
                    'range_max': 10.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0
                }],
                name='pointcloud_to_laserscan'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([livox_driver_dir, '/launch/msg_MID360_launch.py']),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        load_nodes
    ])