# G1机器人导航与AprilTag位置修正分析

## 导航栈架构分析

### 1. 运动控制接口
- **话题**: `/cmd_vel` (geometry_msgs/Twist)
- **节点**: `g1_write_node` 接收速度命令并发送给G1机器人
- **控制**: 线速度(linear.x)和角速度(angular.z)

### 2. 导航接口
- **Action**: `/navigate_to_pose` (nav2_msgs/action/NavigateToPose)
- **服务**: `/cancel_nav2_goal` (取消导航目标)
- **架构**: Nav2导航栈(controller_server, planner_server, bt_navigator等)

### 3. 模式控制
- **服务**: `/mode` (bot_custom_interfaces/srv/Mode)
- **模式**: damp, preparation, run, squat, zero_torque等
- **用途**: 控制机器人FSM状态

## 创建的脚本

### apriltag_nav_correction.py
**功能**: 导航到目标点后使用AprilTag进行精确位置修正

**工作流程**:
1. 发送Nav2导航目标到指定位置
2. 等待导航完成
3. 切换到AprilTag视觉伺服模式
4. 检测AprilTag并计算位置误差
5. 发布修正速度命令直到达到精度要求

**关键参数**:
- `target_tag_id`: 目标AprilTag ID
- `correction_linear_gain`: 线速度增益(默认0.3)
- `correction_angular_gain`: 角速度增益(默认0.5)
- `position_tolerance`: 位置容差(默认0.05米)
- `max_correction_time`: 最大修正时间(默认30秒)

## 使用方法

### 1. 启动AprilTag检测节点
```bash
ros2 launch apriltag_ros tag_detection.launch.py
```

### 2. 启动导航修正节点
```bash
ros2 run bot_navigation apriltag_nav_correction.py --ros-args \
  -p target_tag_id:=0 \
  -p position_tolerance:=0.05
```

### 3. 发送导航目标
通过代码调用`navigate_to_goal(x, y, yaw)`方法，或使用RViz发送目标点。

## 技术要点

1. **状态机**: IDLE → NAVIGATING → CORRECTING → COMPLETED
2. **视觉伺服**: 基于AprilTag位置计算速度命令
3. **容错**: 超时保护、标签丢失处理
4. **速度限制**: 限制最大线速度和角速度确保安全
