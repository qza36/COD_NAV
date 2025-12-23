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
    bring_up_dir = get_package_share_directory('nav_bringup')


    fastlio_config_path = os.path.join(fastlio_dir, 'config')
    rviz_config_file = os.path.join(bring_up_dir, 'rviz', 'cod_nav.rviz')

    fast_lio_config_file = 'mid360.yaml'
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
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

    # 定义节点和包含的launch文件
    load_nodes = GroupAction(
        actions=[
            Node(
                package='cpp_lidar_filter',
                executable='lidar_filter_node',
                name='my_lidar_filter',
                output='screen',
                parameters=[{
                    'input_topic': '/livox/lidar',
                    'output_topic': '/livox/lidar_filtered',
                    'min_x': -0.4, 'max_x': 0.4,
                    'min_y': -0.3, 'max_y': 0.3,
                    'min_z': -0.1, 'max_z': 0.6,
                    'negative': True,   # 挖掉车身
                    'leaf_size': 0.05   # 降采样
                }]
            ),
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[PathJoinSubstitution([fastlio_config_path, fast_lio_config_file]),
                            {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in',  '/livox/lidar'),
                            ('scan', '/scan')],
                parameters=[{
                    'target_frame': 'base_link',
                    'transform_tolerance': 0.5,
                    'min_height': 0.01,
                    'max_height': 1.00,
                    'angle_min': -3.1416,  # -M_PI/2
                    'angle_max': 3.1416,  # M_PI/2
                    'angle_increment': 0.0087,  # M_PI/360.0
                    'scan_time': 0.3333,
                    'range_min': 0.5,
                    'range_max': 20.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0
                }],
                name='pointcloud_to_laserscan'
            ),
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
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.05",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
            ),
            Node(
                package="loam_interface",
                executable="loam_interface_node",
                name="loam_interface",
                output="screen",
                parameters=[
                    {
                        "state_estimation_topic": "Odometry",
                        "registered_scan_topic": "cloud_registered",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "lidar_frame": "livox_frame",
                    }
                ],
            ),
            Node(
                package="sensor_scan_generation",
                executable="sensor_scan_generation_node",
                output="screen",
                parameters=[
                    {"lidar_frame": "livox_frame"},
                    {"base_frame": "base_link"},
                    {"robot_base_frame": "base_link"},
                ],
            ),
            Node(
                package="fake_vel_transform",
                executable="fake_vel_transform_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="cod_serial_ul26",
                executable="cod_serial",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('realsense2_camera'),'launch','rs_launch.py')
                ),
                launch_arguments={
                    'depth_module.depth_profile': '1280x720x30',
                    'pointcloud.enable': 'true'
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bring_up_dir,'launch','navigation_launch.py')),
                launch_arguments={
                                  'use_sim_time': "false",
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
                output='screen',
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([bring_up_dir,'/launch/nav_bring_up.launch.py']),
            #     launch_arguments={'use_sim_time': use_sim_time,'map': '/home/cod-sentry/qza_ws/cod_nav/src/sim_test.yaml'}.items()
            # )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_nav2_params_file,
        load_nodes
    ])