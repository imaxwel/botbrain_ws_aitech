# 使用手臂控制g1_manipulation_pkg
---
## 使用指南

### 构建

```bash
# 1. 构建 Docker 镜像 (首次或 Dockerfile 变更时)
docker compose build manipulation

# 2. 编译 ROS 包 (代码变更时)
docker compose run --rm builder_base

# 或单独编译 manipulation 包:
docker compose run --rm manipulation bash -lc \
  "cd /botbrain_ws && colcon build --packages-select g1_manipulation_pkg"
```

### 启动

```bash
# 只启动 manipulation (最小依赖)
docker compose up -d manipulation

# 查看日志
docker compose logs -f manipulation

# 预期输出:
# [manipulation] DDS interface: eth1, config: /tmp/manipulation_cyclonedds.xml
# [arm_controller-1]: process started with pid [XX]
# [dx3_controller-2]: process started with pid [XX]
# [interactive_marker-3]: process started with pid [XX]
# [arm_controller] === BotBrain Arm Controller starting ===
# [dx3_controller] DX3Controller started — interface=eth1, arms=both
```

### Mode-B 联锁测试

**前提**: manipulation 容器运行，机器人不需要上电。

```bash
# D1. 确认 manipulation_vel 无数据
ros2 topic hz /g1_robot/manipulation_vel
# → 应显示无消息

# D2. 启用手臂
ros2 topic pub --once /g1_robot/manipulation/enabled std_msgs/msg/Bool "{data: true}"
# → 日志: "Arm ENABLED (Mode-B: base stopped)."

# D3. 确认零速发布
ros2 topic hz /g1_robot/manipulation_vel
# → 应显示 ~10 Hz

# D4. 禁用手臂
ros2 topic pub --once /g1_robot/manipulation/enabled std_msgs/msg/Bool "{data: false}"
# → 日志: "[Release] Ramping arm weight down over 2s..."

# D5. 确认停发
ros2 topic hz /g1_robot/manipulation_vel
# → 应显示无消息
```

### 手臂控制测试

**前提**: 机器人上电并站立，arm_controller 不再显示 "Waiting for LowState data..."。

```bash
# E1. 启用手臂
ros2 topic pub --once /g1_robot/manipulation/enabled std_msgs/msg/Bool "{data: true}"

# E2. 归位 (安全起始姿态)
ros2 topic pub --once /g1_robot/manipulation/home std_msgs/msg/Bool "{data: true}"
# → 等待日志 "Home position reached."

# E3. 发送目标 (右臂前伸，workspace 安全区)
ros2 topic pub --once /g1_robot/manipulation/hand_goal/right \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'pelvis'}, pose: {position: {x: 0.35, y: -0.20, z: 0.15}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}"

# E4. DX3 灵巧手
ros2 topic pub --once /g1_robot/manipulation/dx3/hand_action/right std_msgs/msg/String "{data: 'close'}"
ros2 topic pub --once /g1_robot/manipulation/dx3/hand_action/right std_msgs/msg/String "{data: 'open'}"

# E5. 安全禁用 (先归位再禁用)
ros2 topic pub --once /g1_robot/manipulation/home std_msgs/msg/Bool "{data: true}"
# 等 "Home position reached." 后:
ros2 topic pub --once /g1_robot/manipulation/enabled std_msgs/msg/Bool "{data: false}"
```

### 键盘遥控

```bash
docker compose exec manipulation bash
source /botbrain_ws/install/setup.bash
ros2 run g1_manipulation_pkg arm_teleop_keyboard --ros-args -r __ns:=/g1_robot
```

操作步骤: 按 `[` 启用 → 按 `h` 归位 → 用 `wasdqe` 调整 → 按 `]` 禁用。

### 安全操作规范

1. **始终先归位再操作** — Home 位姿是已验证的安全姿态
2. **每次移动不超过 5cm** — 用键盘遥控 (步长 2cm) 或小增量 topic pub
3. **避免 y 值过小** — 右臂 |y| < 0.10 可能打到躯干
4. **避免 x 值过小** — x < 0.10 手臂后收可能碰到身体
5. **紧急停止**: `ros2 topic pub --once /g1_robot/manipulation/enabled std_msgs/msg/Bool "{data: false}"`
   - 默认 (release_on_disable=true): 手臂 2 秒平滑归还运控
   - 设为 false 时: 手臂保持当前位置
6. **容器级停止**: `docker compose stop manipulation`
7. **右臂安全工作空间 (pelvis 坐标系)**:
   - x: 0.07 ~ 0.45 (前)
   - y: -0.07 ~ -0.47 (右侧)
   - z: 0.02 ~ 0.20 (上)

---

## 附录: ROS 话题列表

```
/g1_robot/manipulation/enabled                      # Bool: 启用/禁用手臂
/g1_robot/manipulation/home                         # Bool: 归位
/g1_robot/manipulation/hand_goal/left               # PoseStamped: 左手目标
/g1_robot/manipulation/hand_goal/right              # PoseStamped: 右手目标
/g1_robot/manipulation/workspace/left               # Marker: 左臂工作空间可视化
/g1_robot/manipulation/workspace/right              # Marker: 右臂工作空间可视化
/g1_robot/manipulation/dx3/hand_action/left         # String: 左手动作 (open/close)
/g1_robot/manipulation/dx3/hand_action/right        # String: 右手动作 (open/close)
/g1_robot/manipulation/dx3/left/motor_state         # JointState: 左手电机状态
/g1_robot/manipulation/dx3/right/motor_state        # JointState: 右手电机状态
/g1_robot/manipulation_vel                          # Twist: Mode-B 零速 (twist_mux)
/g1_robot/manipulation_ee_goal_markers/update       # InteractiveMarker
/g1_robot/manipulation_ee_goal_markers/feedback     # InteractiveMarkerFeedback
```

## 附录: ROS 参数

```yaml
# manipulation_config.yaml
use_robot: true                  # false=仿真模式
rate_hz: 250.0                   # 主循环频率
arm_velocity_limit: 5.0          # rad/s 关节速度限制
ik_world_frame: "pelvis"         # IK 参考坐标系
release_on_disable: true         # 禁用时是否交还控制权
stop_nav_topic: "manipulation_vel"  # Mode-B 零速话题
arm_controlled: "both"           # DX3 手控制: both/left/right
```
