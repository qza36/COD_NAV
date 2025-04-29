# cod_nav
## 介绍
- 环境
  - ubuntu 22.04
  - ros2 humble
- 基于nav2框架的导航功能包
- 选用fastlio-v2作为LIO，根据需求配置NDT、ICP点云配准
- 局部控制器选取MPPI
## 使用
  - build
    ```shell
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
    ```
  - run
    ```shell
        
    ros2 launch nav_bringup nav_bringup #启动定位、串口等模块
    ros2 launch nav_bringup nav_bringup.launch.py map:=/home/cod-sentry/qza_ws/cod_nav/slop.yaml
      ```
  - 实用工具
    - 保存地图
      ```shell
ros2 service call /map_save std_srvs/srv/Trigger 
      ```
    - 小键盘控制
      ```shell
      ros2 run teleop_twist_keyboard teleop_twist_keyboard
      ```
    - 保存地图
    ```shell
    ros2 run nav2_map_server map_saver_cli -f MAP_NAME
    ```
### SLAM

## TODO

- [ ] 实现SLAM时导航
- [X] 雷达坐标系自旋时，依旧能正常导航
- [X] 导航上斜坡
 