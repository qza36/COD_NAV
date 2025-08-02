# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        namespace="",
        output="screen",
        remappings=remappings,
        parameters=[
            {
                "num_threads": 8,
                "num_neighbors": 10,
                "global_leaf_size": 0.25,
                "registered_leaf_size": 0.05,
                "max_dist_sq": 2.5,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "chassis",
                "lidar_frame": "chassis",
                #"prior_pcd_file": "/home/cod-sentry/qza_ws/cod_nav/nav_bringup/map/sim_test.pcd", #blue
                "prior_pcd_file": "/home/cod-sentry/qza_ws/cod_nav/PCD/sim_test.pcd" #red
            }
        ],
    )

    return LaunchDescription([node])
