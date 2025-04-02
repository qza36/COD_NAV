#!/bin/bash

# 目标点坐标
goal_x=4.55
goal_y=-2.0
goal_z=0.0
goal_w=1.0

# 原点坐标
home_x=0.0
home_y=0.0
home_z=0.0
home_w=1.0

while true; do
# 读取 game_type 值，限制 2 秒内返回结果
game_type=$(timeout 2 ros2 topic echo /serial/receive --once | grep "game_type" | awk '{print $2}')

if [[ -z "$game_type" ]]; then
echo "无法获取 game_type 数据，等待 1 秒..."
sleep 1
continue
fi

echo "当前 game_type: $game_type"

if [[ "$game_type" == "3" ]]; then
echo "game_type = 3，返回..."
timeout 10 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $home_x, y: $home_y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $home_z, w: $home_w}}}}"
elif [[ "$game_type" == "2" ]]; then
echo "game_type = 2，前往goal..."
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $goal_x, y: $goal_y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $goal_z, w: $goal_w}}}}"
else
echo "game_type 未知，等待下一次检测..."
fi

sleep 1 # 每 5 秒检查一次 game_type 状态
done