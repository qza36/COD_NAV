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

    # 配置文件路径
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('nav_bringup'), 'urdf', 'simulation_waking_robot.xacro')])

    fastlio_config_path = os.path.join(fastlio_dir, 'config')
    fast_lio_config_file = 'mid360.yaml'
    localization_param_dir = os.path.join(lidar_localization_dir, 'param', 'localization.yaml')

    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 定义节点和包含的launch文件
    load_nodes = GroupAction(
        actions=[

            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[PathJoinSubstitution([fastlio_config_path, fast_lio_config_file]),
                            {'use_sim_time': use_sim_time}],
                output='screen'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([livox_driver_dir, '/launch/msg_MID360_launch.py']),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([bring_up_dir,'/launch/nav_bring_up.launch.py']),
            #     launch_arguments={'use_sim_time': use_sim_time,'map': '/home/cod-sentry/qza_ws/cod_nav/src/sim_test.yaml'}.items()
            # )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        load_nodes
    ])