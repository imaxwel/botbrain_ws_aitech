# AprilTag位置修正独立测试脚本

## 脚本说明

`apriltag_correction_standalone.py` - 独立的AprilTag位置修正节点，不依赖导航功能，可直接测试。

## 功能

- 检测指定ID的AprilTag
- 通过视觉伺服控制机器人移动到AprilTag前方
- 可通过服务启动/停止修正
- 达到精度要求后自动停止

## 使用步骤

### 1. 启动AprilTag检测
```bash
ros2 launch apriltag_ros tag_detection.launch.py
```

### 2. 启动修正节点
```bash
ros2 run bot_navigation apriltag_correction_standalone.py --ros-args \
  -p target_tag_id:=0 \
  -p position_tolerance:=0.05
```

### 3. 启动修正
```bash
ros2 service call /enable_correction std_srvs/srv/SetBool "{data: true}"
```

### 4. 停止修正
```bash
ros2 service call /enable_correction std_srvs/srv/SetBool "{data: false}"
```

## 参数配置

- `target_tag_id`: AprilTag ID (默认: 0)
- `kp_linear`: 线速度增益 (默认: 0.3)
- `kp_angular`: 角速度增益 (默认: 0.5)
- `position_tolerance`: 位置容差/米 (默认: 0.05)
- `max_linear_vel`: 最大线速度 (默认: 0.2)
- `max_angular_vel`: 最大角速度 (默认: 0.3)

## 测试建议

1. 先在安全区域测试
2. 调整增益参数观察响应速度
3. 确认精度满足要求后再集成到导航流程
