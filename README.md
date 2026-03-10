![cod_logo](resource/cod-1.png)
# RoboMaster2025辽宁科技大学COD战队哨兵机器人上位机导航系统
## 项目简介

- **运行环境**
  - Ubuntu 22.04
  - ROS 2 Humble
  - Livox Mid-360
- 基于 Nav2 框架开发的导航功能包
- 局部路径规划器选用 MPPI

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
### 编译

```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```

### 运行
目前只支持边建图边导航
```shell
ros2 launch nav_bringup slam.launch.py
```

### 实用工具

- 小键盘控制机器人

  ```shell
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
### 配置说明
- 参数模板：`nav_bringup/params/nav2_params.yaml` 
