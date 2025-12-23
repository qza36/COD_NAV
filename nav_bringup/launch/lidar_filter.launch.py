from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        )
    ])