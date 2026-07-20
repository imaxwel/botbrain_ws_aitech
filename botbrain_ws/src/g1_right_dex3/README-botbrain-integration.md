# unitree_g1_dex3_stack — BotBrain 集成文档

原项目（`unitree-dex3:humble` + `run.sh`）已并入 BotBrain 框架，由 `docker-compose.yaml` 统一编排管理。

- **机器人主机**：`192.168.100.30`
- **BotBrain 项目路径**：`/data/botbrain_ws/botbrain_project-main/`
- **代码位置**：`botbrain_ws/src/g1_right_dex3/`

---

## 1. 代码文件结构

```
botbrain_ws/src/
├── fcl/                          # FCL 碰撞库（从原项目修复完整版）
├── g1_right_dex3/
│   ├── unitree_g1_dex3_stack/    # 主包：ROS2 C++ 规划器 + Python 节点
│   ├── trac_ik/                  # TRAC-IK 求解器（含 trac_ik_lib 等子包）
│   ├── unitree_dex3_cpp/         # Dex-3 灵巧手 Python 绑定（pybind11）
│   ├── elevator_vision/          # 电梯视觉检测脚本
│   ├── yolonas_ocr/              # YOLO-NAS OCR 推理模型
│   ├── data/                     # 标定数据
│   └── right_arm_mode.py         # 右臂卸力/锁定工具
└── ...（其他 BotBrain 包）
```

---

## 2. Docker 服务说明

`docker-compose.yaml` 新增三个服务：

| 服务名 | 用途 | restart 策略 |
|--------|------|--------------|
| `g1_robot_camera` | 专用相机容器：运行 realsense + `cam_frame_writer.py`，将帧写入 `/run/latest_cam.bin` | `always`（开机自动启动） |
| `dev_dex3` | 常驻开发/运行容器，手动进入后执行 launch | `unless-stopped` |
| `builder_dex3` | 一次性编译容器（`--rm` 执行后自动销毁） | `no` |

### 相机架构（文件桥接）

```
g1_robot_camera（restart:always）
  └─ realsense2_camera（640x480, 6Hz, RELIABLE）
  └─ cam_frame_writer.py → /run/latest_cam.bin（原子写入）
             ↓ /run:/run 共享卷
g1_robot_dev_dex3
  └─ v4l2_apriltag_trigger（每150ms读文件）
```

- `g1_robot_camera` 完全独立，不受 localization/导航重启影响
- `cam_lifecycle_activate.py`：守护脚本，自动监控 realsense lifecycle 节点并发送 configure+activate
- `cam_frame_writer.py`：内置 watchdog，15秒无帧自动重启进程（防 Zenoh session 超时）
- `image_source_file: /run/latest_cam.bin` 已写入 `v4l2_apriltag_trigger.yaml`，无需手动指定

---

## 3. 快速启动（完整流程）

### 开机后自动完成的事（无需操作）

1. `botbrain.service` 自动启动所有容器（含 `g1_robot_camera`）
2. `g1_robot_camera` 内：realsense 启动 → lifecycle 守护脚本自动激活 → `cam_frame_writer` 开始写帧
3. 约 **30~60 秒**后 `/run/latest_cam.bin` 开始持续更新

### Step 1：确认相机已就绪（可选）

```bash
ssh unitree@192.168.100.30 "docker exec g1_robot_camera tail -2 /tmp/cam_frame_writer.log"
# 看到 frame #xxx 640x480 enc=rgb8 即就绪
```

### Step 2：运行 AprilTag 检测

```bash
docker exec -it g1_robot_dev_dex3 bash -c \
  "source /botbrain_ws/install/setup.bash && \
   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
   ros2 launch unitree_g1_dex3_stack apriltag.launch.py detect_only:=true"
```

启动后按 **G 键** 触发检测。

### Step 3（可选）：全流程按压按钮

```bash
docker exec -it g1_robot_dev_dex3 bash -c \
  "source /botbrain_ws/install/setup.bash && \
   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
   ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py dry_run:=false"

看到下面这四行后证明启动完成，可以按g进行按压：
[v4l2_apriltag_trigger.py-6] [INFO] [1784512591.731864356] [v4l2_apriltag_trigger]: [v4l2_apriltag_trigger] buffer ready from file (35 frames, 5.1s)
---
```

## 4. 编译详解

### 全量编译（首次 / 修改了 CMakeLists.txt / 修改了 C++ 代码）

```bash
cd /data/botbrain_ws/botbrain_project-main
docker compose run --rm builder_dex3
```

或手动进入容器编译：

```bash
docker exec -it g1_robot_dev_dex3 bash
cd /botbrain_ws
colcon build --packages-select fcl trac_ik_lib unitree_g1_dex3_stack \
  --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON -DPython3_EXECUTABLE=/usr/bin/python3
pip3 install --no-build-isolation --no-deps -e src/g1_right_dex3/unitree_dex3_cpp
```

### 增量编译（只改了 Python 脚本 / launch 文件 / config yaml）

```bash
docker exec -it g1_robot_dev_dex3 bash
cd /botbrain_ws
colcon build --packages-select unitree_g1_dex3_stack \
  --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON -DPython3_EXECUTABLE=/usr/bin/python3
```

编译成功标志：
```
Summary: 3 packages finished
Successfully installed unitree_cpp-1.0.3
```

> 修改后需要把新文件同步到机器人 install 目录，或用 `docker cp` 直接覆盖。

---

## 5. 容器管理

```bash
cd /data/botbrain_ws/botbrain_project-main

# 查看所有容器状态
docker compose ps

# 查看相机容器日志
docker compose logs -f camera

# 相机容器重启（会自动重新激活 lifecycle）
docker compose stop camera && sleep 6 && docker compose start camera

# 重启 dev_dex3
docker compose restart dev_dex3
```

---

## 6. 进入容器

```bash
# 进入 dev_dex3（运行 launch 的容器）
docker exec -it g1_robot_dev_dex3 bash

# 容器内 source 环境（每次进入后执行）
source /botbrain_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 7. Launch 入口

| Launch 文件 | 用途 |
|---|---|
| `apriltag.launch.py` | 仅 AprilTag 检测调试（`detect_only:=true`） |
| `apriltag_button_press.launch.py` | **全流程**：AprilTag 检测 → OMPL 规划 → 手臂执行 → 灵巧手按压 |
| `apriltag_button_open.launch.py` | 打开按钮 |
| `apriltag_button_up.launch.py` | 向上按钮 |
| `apriltag_button_down.launch.py` | 向下按钮 |
| `apriltag_reach.launch.py` | 端到端到达（不含灵巧手） |
| `elevator_button_press.launch.py` | **电梯视觉**：yolonas_ocr 检测楼层按钮 → 手臂按压 |

所有 launch 均在 `g1_robot_dev_dex3` 容器内执行，使用 `rmw_cyclonedds_cpp`。

---

## 8. 常用命令

### AprilTag 检测（安全测试，不动手臂）

```bash
docker exec -it g1_robot_dev_dex3 bash -c \
  "source /botbrain_ws/install/setup.bash && \
   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
   ros2 launch unitree_g1_dex3_stack apriltag.launch.py detect_only:=true"
```

### 按压按钮系列

```bash
# 进入容器后
source /botbrain_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 按压
ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py dry_run:=false

# 打开
ros2 launch unitree_g1_dex3_stack apriltag_button_open.launch.py dry_run:=false

# 向上
ros2 launch unitree_g1_dex3_stack apriltag_button_up.launch.py dry_run:=false
```

| 位置 | Launch 文件 |
|------|-------------|
| 按下（press） | `apriltag_button_press.launch.py` |
| 向上（up） | `apriltag_button_up.launch.py` |
| 向下（down） | `apriltag_button_down.launch.py` |
| 打开（open） | `apriltag_button_open.launch.py` |
| 关闭（close） | `apriltag_button_close.launch.py` |

启动后按 **G 键** 触发。

### 电梯视觉按压

**终端 1：启动 RealSense 深度相机**

```bash
docker exec -it g1_robot_dev_dex3 bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 launch realsense2_camera rs_launch.py \
   enable_depth:=true align_depth.enable:=true \
   rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30"
```

**终端 2：启动电梯视觉按压**

```bash
docker exec -it g1_robot_dev_dex3 bash -c \
  "source /botbrain_ws/install/setup.bash && \
   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
   ros2 launch unitree_g1_dex3_stack elevator_button_press.launch.py \
   target_floor:=5 det_threshold:=0.3"
```

> button_detector_node 需约 12 秒加载模型，出现 `button_detector_node ready` 后再按 G。

### 手臂回站立姿态（紧急恢复）

```bash
docker exec -it g1_robot_dev_dex3 bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 topic pub /executor/return_to_standing std_msgs/msg/Empty '{}' --once"
```

### 灵巧手控制

```bash
# 伸出中指
docker exec g1_robot_dev_dex3 python3 \
  /botbrain_ws/src/g1_right_dex3/unitree_dex3_cpp/example/control_dex3_right_setpoint.py \
  enP8p1s0 0 -1.05 -1.7 1.7 1.8 0 0

# 合拢
docker exec g1_robot_dev_dex3 python3 \
  /botbrain_ws/src/g1_right_dex3/unitree_dex3_cpp/example/control_dex3_right_setpoint.py \
  enP8p1s0 0 -1.05 -1.7 1.7 1.8 1.7 1.8
```

### 右臂卸力 / 锁定模式

```bash
docker exec -it g1_robot_dev_dex3 bash -c \
  "source /botbrain_ws/install/setup.bash && \
   python3 /botbrain_ws/src/g1_right_dex3/right_arm_mode.py"
```

| 命令 | 功能 |
|------|------|
| `free` | 卸力，可自由拖动右臂 |
| `lock` | 锁定，保持当前姿态 |
| `status` | 查看当前 7 个关节角度 |

---

## 9. 路径对照（原项目 → BotBrain）

| 原路径（`/workspaces/`） | 新路径（`/botbrain_ws/`） |
|---|---|
| `/workspaces/unitree_dex3/detect_img` | `/botbrain_ws/detect_img` |
| `/workspaces/unitree_dex3_cpp/example/control_dex3_right_setpoint.py` | `/botbrain_ws/src/g1_right_dex3/unitree_dex3_cpp/example/control_dex3_right_setpoint.py` |
| `/workspaces/yolonas_ocr/frozen_model` | `/botbrain_ws/src/g1_right_dex3/yolonas_ocr/frozen_model` |
| `/workspaces/unitree_dex3/elevator_vision/scripts/button_detector_node.py` | `/botbrain_ws/src/g1_right_dex3/elevator_vision/scripts/button_detector_node.py` |
| `install_container/setup.bash` | `/botbrain_ws/install/setup.bash` |

---

## 10. 注意事项

### 话题冲突风险

| 话题 / 资源 | 状态 |
|------|------|
| `/tf` / `/tf_static` | ✅ 已用命名空间隔离 |
| `/lf/lowstate` | ✅ 只读订阅，无冲突 |
| `rt/arm_sdk` | ⚠️ `g1_manipulation_pkg` 与本程序不能同时控制右臂 |
| `/dev/video*`（V4L2）| ✅ 不再使用 V4L2，改为文件桥接，无冲突 |
| `/run/latest_cam.bin` | ✅ 由 `g1_robot_camera` 写入，dev_dex3 只读 |

### 代码修改后必须重新编译

Python 脚本修改后也需要 `colcon build`，节点执行的是 `install/` 目录下的副本。

### 不要用 conda python

编译时明确指定 `-DPython3_EXECUTABLE=/usr/bin/python3`。

---

## 11. 问题排查

### 相机文件不更新（`camera file stale` 警告）

```bash
# 检查帧是否在写入
ssh unitree@192.168.100.30 "docker exec g1_robot_camera tail -3 /tmp/cam_frame_writer.log"
# 正常：frame #xxx 640x480 enc=rgb8

# 检查 lifecycle 状态
docker exec g1_robot_camera bash -c \
  'source /botbrain_ws/install/setup.bash && ros2 lifecycle get /g1_robot/front_camera'
# 正常：active [3]

# 重启相机容器（lifecycle守护脚本会自动重新激活）
ssh unitree@192.168.100.30 \
  "cd /data/botbrain_ws/botbrain_project-main && \
   docker compose stop camera && sleep 6 && docker compose start camera"
# 等待30~60秒后相机恢复
```

### 容器没起来 / 异常退出

```bash
docker compose logs --tail=50 camera
docker compose logs --tail=50 dev_dex3
```

### 手臂没有正常回到站立 / Dex-3 超时

> 不要直接关掉报错终端，否则下次启动手臂会瞬间冲击回原点。

```bash
docker exec -it g1_robot_dev_dex3 bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 topic pub /executor/return_to_standing std_msgs/msg/Empty '{}' --once"
```

### 手臂震颤

立即检查是否有多个 `joint_trajectory_executor` 在运行：

```bash
docker exec g1_robot_dev_dex3 bash -c \
  "source /opt/ros/humble/setup.bash && ros2 node list | grep executor"
```

如有多个，停止 `g1_manipulation`：

```bash
docker compose stop g1_manipulation
```

### TRAC-IK 无解循环

planner 反复 `No solution found`，检查 `reach_max_distance` 参数或手动发布可达目标位姿。

---

## 12. 修改 docker-compose.yaml 后的操作流程

修改 compose 文件后，**不要直接 `docker compose down`**，这会影响所有 BotBrain 服务。

```bash
# 只更新单个服务（--force-recreate 使新 command 生效）
cd /data/botbrain_ws/botbrain_project-main
docker compose up -d --force-recreate dev_dex3
# 或
docker compose up -d --force-recreate camera
```

---

## 13. 集成踩坑记录

### A. fcl 残缺版导致编译失败

**原因**：BotBrain 仓库中的 `botbrain_ws/src/fcl/` 是残缺版，缺少 `CMakeModules/` 目录。
**修复**：用原项目完整 fcl 覆盖。

### B. 重复包名冲突

**原因**：`g1_right_dex3/fcl/` 与顶层 `src/fcl/` 重名。
**修复**：删除 `g1_right_dex3/fcl/`。

### C. Zenoh late joiner problem（相机订阅无法建立）

**原因**：`g1_robot_dev_dex3` 使用 `rmw_cyclonedds_cpp`，无法订阅 Zenoh 相机话题；即使都用 Zenoh，新进程也因 peer mode late joiner 无法加入已有 peer group。
**修复**：文件桥接架构——`g1_robot_camera` 容器将帧写入 `/run/latest_cam.bin`，dev_dex3 轮询读取。

### D. QoS 不匹配导致收不到相机帧

**原因**：RealSense 默认发布 `BEST_EFFORT`，`cam_frame_writer.py` 订阅用 `RELIABLE` → ROS2 认为不兼容，静默丢弃所有消息。
**修复**：`cam_frame_writer.py` 订阅改为 `BEST_EFFORT`。

### E. `detect_scale: 0.5` 导致 AprilTag 检测失败

**原因**：图像缩小到 320x240 后 tag 无法被检测到。
**修复**：`v4l2_apriltag_trigger.yaml` 中 `detect_scale: 1.0`。

### F. realsense lifecycle 节点重启后卡在 Unconfigured

**原因**：BotBrain lifecycle manager 只在整个栈首次启动时发一次 configure/activate，`g1_robot_camera` 单独重启后没有节点再触发。
**修复**：新增 `cam_lifecycle_activate.py`，容器启动后自动监控并发 configure+activate，直到成功。

### G. 电机震颤（双节点同时写 rt/arm_sdk）

**原因**：两个 `joint_trajectory_executor` 节点同时向 `rt/arm_sdk` 写入指令。
**修复**：确保只有一个 launch 在运行，停止 `g1_manipulation` 服务。

### H. iptables raw table error

**原因**：`web_server` base service 缺少 `network_mode: host`。
**修复**：在 docker-compose.yaml 的 base service 加 `network_mode: host`。
