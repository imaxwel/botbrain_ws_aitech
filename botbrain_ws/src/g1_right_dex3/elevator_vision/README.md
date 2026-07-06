# 电梯按键视觉定位 - 新方案

**核心策略：一次性标定 + 纯视觉运行**

## 方案概述

- **标定阶段：** 使用 AprilTag 建立相机→机器人坐标变换，保存静态矩阵
- **运行阶段：** yolonas_ocr + 深度 + 静态矩阵变换
- **精度：** ±7mm
- **优势：** 无需 AprilTag 辅助板，适用任何电梯

## 使用流程

### 阶段 1：查看相机内参

```bash
docker exec -it unitree-dex3-dev bash
source /opt/ros/humble/setup.bash
source /workspaces/unitree_dex3/install_container/setup.bash
cd /workspaces/unitree_dex3/elevator_vision/scripts
python3 check_camera_intrinsics.py
```

### 阶段 2：采集标定数据（5-10个位姿）

```bash
cd /workspaces/unitree_dex3/elevator_vision/scripts
python3 calibration_collector_node.py --ros-args \
  -p save_path:=/workspaces/unitree_dex3/elevator_vision/scripts/calibration/calibration_data.json
```

**操作：** 按空格记录，按q退出

#### 计算变换矩阵

```bash
cd /workspaces/unitree_dex3/elevator_vision/scripts/calibration
python3 compute_transform.py
```

### 阶段 3：运行检测

```bash
cd /workspaces/unitree_dex3/elevator_vision/scripts
python3 button_detector_node.py --ros-args \
  -p input_backend:=v4l2 \
  -p frozen_model_dir:=/workspaces/yolonas_ocr/frozen_model \
  -p target_floor:=3 \
  -p det_threshold:=0.3
```

## 与旧方案对比

| 项目 | 旧方案 | 新方案 |
|-----|-------|-------|
| 视觉 | AprilTag | yolonas_ocr |
| 变换 | TF2动态 | 静态矩阵 |
| 依赖 | 需辅助板 | 无需 |
| 控制 | apriltag_button_press_node | **复用** |
