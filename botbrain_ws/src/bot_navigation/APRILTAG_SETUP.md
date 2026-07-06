# AprilTag检测配置说明

## 安装apriltag_ros

```bash
sudo apt install ros-humble-apriltag-ros
```

## 启动文件

已创建: `botbrain_ws/src/bot_navigation/launch/apriltag_detection.launch.py`

### 配置说明

- **相机话题**: `/front_camera/color/image_raw`
- **AprilTag家族**: 36h11
- **标签尺寸**: 0.162米 (需根据实际标签调整)

## 使用方法

### 1. 启动相机
```bash
ros2 launch bot_localization realsense.launch.py
```

### 2. 启动AprilTag检测
```bash
ros2 launch bot_navigation apriltag_detection.launch.py
```

### 3. 查看检测结果
```bash
ros2 topic echo /apriltag/detections
```

## 参数调整

编辑启动文件中的参数：
- `size`: AprilTag实际尺寸(米)
- `family`: 标签家族(36h11, 25h9等)
