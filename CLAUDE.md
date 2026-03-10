# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RoboMaster 2025 COD Team sentinel robot navigation system. ROS 2 Humble on Ubuntu 22.04, built on the Nav2 framework with Livox Mid-360 LiDAR. The robot is omnidirectional and uses MPPI for local path planning.

## Build & Run

```bash
# Workspace root: ~/qza_ws/cod_nav_rmul2026/
# Build (from workspace root)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja

# Build a single package
colcon build --packages-select <package_name> --cmake-args -DCMAKE_BUILD_TYPE=Release -G Ninja

# Source the workspace
source install/setup.bash

# Launch the full system (simultaneous SLAM + navigation)
ros2 launch nav_bringup slam.launch.py

# Teleop for testing
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Prerequisites: Livox SDK2 installed, then `rosdep install --from-paths src --ignore-src -r -y` from workspace root.

There is no test infrastructure — no unit or integration tests exist in these packages.

## Architecture

### Packages

| Package | Purpose |
|---------|---------|
| **nav_bringup** | Launch files, Nav2 params, behavior trees, maps, waypoints, URDF, RViz configs |
| **small_point_lio** | LiDAR-Inertial Odometry (ESKF-based, optimized with OpenMP). Outputs `/Odometry` and `/cloud_registered` |
| **cpp_lidar_filter** | PCL-based crop box + voxel grid filter. Removes robot body from point cloud |
| **pointcloud_to_laserscan** | Converts 3D PointCloud2 to 2D LaserScan for slam_toolbox |
| **fake_vel_transform** | Creates stable `base_link_fake` frame to decouple gimbal rotation from Nav2 velocity commands |

### Data Flow

```
/livox/lidar (PointCloud2)
  ├─> cpp_lidar_filter ──> /livox/lidar_filtered (for costmap spatio_temporal_voxel_layer)
  ├─> pointcloud_to_laserscan ──> /scan (for slam_toolbox)
  └─> small_point_lio (+/livox/imu) ──> /Odometry, /cloud_registered

slam_toolbox: /scan ──> map

Nav2 stack: global planner (SmacPlanner2D/Dubin) + local planner (MPPI) + SavitzkyGolaySmoother
  └─> /cmd_vel ──> velocity_smoother ──> fake_vel_transform ──> /aft_cmd_vel ──> cod_serial_ul26 (hardware)
```

### TF Frame Tree

```
map ──(static z=0.05)──> odom ──(small_point_lio)──> base_link
                                                       └──(fake_vel_transform)──> base_link_fake
```

Nav2's `robot_base_frame` is set to `base_link_fake` so that gimbal spinning does not confuse the local planner.

### Key Configuration Files

- **`nav_bringup/params/nav2_params.yaml`** — All Nav2 parameters (MPPI controller, costmaps, planners, BT navigator). Chinese comments explain each tuning decision.
- **`nav_bringup/params/mapper_params_async.yaml`** — slam_toolbox async SLAM (lifelong mapping mode)
- **`small_point_lio/config/mid360.yaml`** — LiDAR-IMU odometry parameters
- **`nav_bringup/launch/slam.launch.py`** — Main entry point, launches all nodes. Note: contains dead references to `fast_lio` package (legacy; actual LIO is `small_point_lio`).
- **`nav_bringup/launch/navigation_launch.py`** — Nav2 lifecycle node bringup
- **`nav_bringup/behavior_trees/`** — BT XMLs for navigate-to-pose and navigate-through-poses

### MPPI Controller Key Parameters

The MPPI controller runs omnidirectional motion (`motion_model: "Omni"`) with high max velocities (vx/vy_max: 7.5 m/s) tuned for the competition sentinel. Key tuning areas:
- **Oscillation suppression**: `temperature: 0.25` (low, forces decisive trajectory selection), `gamma: 0.008` (low regularization)
- **Goal convergence**: `GoalCritic.cost_weight: 15.0` with `threshold_to_consider: 2.5` for early activation; `PathFollowCritic` and `PathAlignCritic` disable within 1.5m of goal to avoid tangential forces
- **Costmap**: `cost_scaling_factor: 5.0` (fast decay for clear gradient), `SpatioTemporalVoxelLayer` with 0.5s linear voxel decay
- Critics, costmap resolution (0.05m), and voxel layer parameters are actively tuned — check recent git history for parameter evolution

A commented-out `OmniPidPursuitController` configuration is preserved in `nav2_params.yaml` as an alternative controller.

### Known Issues in Config

- `bt_navigator.odom_topic` is set to `"odomety"` (typo for `"odometry"`) in `nav2_params.yaml`

### External Dependencies (not in this repo)

- **cod_serial_ul26** — Hardware serial communication node (launched from slam.launch.py)
- **livox_ros_driver2** — Livox LiDAR driver
- **slam_toolbox** — 2D async SLAM
- **Nav2 stack** — Full navigation framework (controller, planner, smoother, BT navigator, costmap, behaviors)
- **spatio_temporal_voxel_layer** — 3D voxel costmap layer used for obstacle detection in both local and global costmaps

## Code Conventions

- C++20 for small_point_lio, C++14 for fake_vel_transform, system default for other packages
- Comments and commit messages are in Chinese
- small_point_lio uses precompiled headers and aggressive optimization flags (`-march=native`, `-ffast-math`, OpenMP)
- Launch files are Python-based (ROS 2 style)
- Parameter tuning is the primary development activity — most recent commits adjust MPPI and costmap parameters
