![cod_logo](resource/cod-1.png)
# RoboMaster2025辽宁科技大学COD战队哨兵机器人上位机导航系统
## 项目简介

- **运行环境**
  - Ubuntu 22.04
  - ROS 2 Humble
  - Livox Mid-360
- 基于 Nav2 框架开发的导航功能包
- 采用 fastlio-v2 作为 LIO（激光惯导里程计），并根据需求配置 NDT、ICP 点云配准算法
- 局部路径规划器选用 MPPI

### 目录结构
```
.
├── clear_costmap_caller    #定时清除成本地图 //没有启用
├── fake_vel_transform      #对云台自旋作解算，发布chassis_fake作为速度参考系
├── FAST_LIO                #fastlio-v2,魔改tf，发布odom->chassis
├── LICENSE                 #开源协议
├── lidar_localization_ros2 #定位包 //没有启用
├── small_gicp_relocalization #small_gicp冲定位
├── pb_omni_pid_pursuit_controller  #局部控制器
├── pb_nav2_plugins         #自定义nav2插件
├── nav_bringup             #启动包
├── ndt_omp_ros2            #ndt依赖
├── patchwork-plusplus      #点云分割，有效分割地面与非地面
├── pointcloud_to_laserscan #pcl2laserscan，全局成本地图更新源
├── README.md
└── rm_simulation           #仿真模块

```
## 使用说明
### 前置工作
- 安装[Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)

- 安装 `rosdep`  
   参考官方文档或使用如下命令进行安装：

   ```shell
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   ```
- 安装依赖
  ```shell
  mkdir ~/cod_ws/src
  git clone --recurse https://github.com/qza36/COD_NAV.git ~/cod_ws/src
  cd ~/cod_ws
  rosdep install --from-paths src --ignore-src -r -y
  ```
- 地图保存
  - 重定位所需的PCD地图，可由fastlio生成
  - 使用北理莫斯科北极熊战队的[pcd2pgm](https://github.com/LihanChen2004/pcd2pgm)项目将pcd转换成pgm珊格地图
- 雷达位置修改
  - FAST_LIO/livox_ros_driver2/src/config/MID360_config.json
  - ``bringup.launch.py``提供的静态转换 
### 编译

```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```

### 运行

```shell
lidar_mapping.sh  #建图
pcd2pgm.sh        #pcd2pgm
bringup.sh        #启动导航
```

### 实用工具

- 保存PCD地图

  ```shell
  ros2 service call /map_save std_srvs/srv/Trigger
  ```

- 小键盘控制机器人

  ```shell
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
### 配置说明
- 参数模板：`nav_bringup/params/nav2_params.yaml` 
- 可调项示例：  
  - LIO 与地图匹配（NDT/ICP）参数  
  - MPPI 局部规划器超参数  
  - 全局/局部成本地图分辨率  
### SLAM 相关（dev）

- 若需使用 SLAM 进行导航，先启动 `test.launch.py`（包含 mapserver），随后启动 `sim_slam.launch.py`（集成 Nav2 导航组件）
- 当前在仿真过程中，频繁出现如下警告：
  
  ```
  Message Filter dropping message: frame 'odom' at time 310.388 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
  ```
- 初步怀疑为 fast_lio 仿真问题，实际机器人尚待测试。
- 地图保存方式如下（参数配置可见 [nav2_params.yaml](nav_bringup/params/nav2_params.yaml)）：

  ```shell
  ros2 run nav2_map_server map_saver_cli -f MAP_NAME
  ```
