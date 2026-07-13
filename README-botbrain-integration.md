# unitree_g1_dex3_stack — BotBrain 集成文档

原项目（`unitree-dex3:humble` + `run.sh`）已并入 BotBrain 框架，由 `docker-compose.yaml` 统一编排管理。

- **机器人主机**：`192.168.100.30`
- **BotBrain 项目路径**：`/home/unitree/botbrain_ws/botbrain_project-main/`
- **代码位置**：`botbrain_ws/src/g1_right_dex3/`

---

## 1. 代码文件结构

```
botbrain_ws/src/ss
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

`docker-compose.yaml` 新增两个服务，均使用 `unitree-dex3:humble` 镜像：

| 服务名 | 用途 | restart 策略 |
|--------|------|--------------|
| `dev_dex3` | 常驻开发/运行容器，手动进入后执行 launch | `unless-stopped` |
| `builder_dex3` | 一次性编译容器（`--rm` 执行后自动销毁） | `no` |

> ⚠️ **不存在自动跑节点的服务**。功能节点需手动进入 `dev_dex3` 执行 launch 命令。

---

## 3. 快速启动（完整流程）

> **重要**：相机图像通过 **Unix socket bridge** 从 bringup 传入 dev_dex3（`/front_camera/image_raw_local`），dev_dex3 内部使用 **CycloneDDS**。两个 bridge 服务（`cam_bridge_sender` 在 bringup 侧、`cam_bridge_receiver` 在 dev_dex3 内）已配置为随 compose 自动启动，无需手动干预。

### Step 1：启动容器

```bash
# 宿主机（G1 机器人）
cd /data/botbrain_ws/botbrain_project-main
docker compose up -d dev_dex3 cam_bridge_sender
docker exec -it g1_robot_dev_dex3 bash
```

### Step 2：容器内 source 环境

```bash
# 容器内（每次进入后必须执行）
source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

> **注意**：launch 节点使用 CycloneDDS（`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`），不使用 Zenoh。相机图像已由 bridge 自动转发到本地话题 `/front_camera/image_raw_local`，约 5~6 Hz。

### Step 3：编译（修改代码后）

```bash
# 容器内
cd /botbrain_ws
colcon build --packages-select unitree_g1_dex3_stack
source install/setup.bash
```

### Step 4：启动节点

```bash
# 容器内（source 并 export RMW 后）

# 仅 AprilTag 检测（不动手臂）
ros2 launch unitree_g1_dex3_stack apriltag.launch.py detect_only:=true \
  apriltag_image_topic:=/front_camera/image_raw_local

# 按压按钮（dry-run 模式，不控制灵巧手）
ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py dry_run:=true \
  apriltag_image_topic:=/front_camera/image_raw_local

# 按压按钮（真机执行）
ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py dry_run:=false \
  apriltag_image_topic:=/front_camera/image_raw_local

# 打开按钮
ros2 launch unitree_g1_dex3_stack apriltag_button_open.launch.py dry_run:=false \
  apriltag_image_topic:=/front_camera/image_raw_local

# 关闭按钮
ros2 launch unitree_g1_dex3_stack apriltag_button_close.launch.py dry_run:=false \
  apriltag_image_topic:=/front_camera/image_raw_local

# 向上按钮
ros2 launch unitree_g1_dex3_stack apriltag_button_up.launch.py dry_run:=false \
  apriltag_image_topic:=/front_camera/image_raw_local

# 向下按钮
ros2 launch unitree_g1_dex3_stack apriltag_button_down.launch.py dry_run:=false \
  apriltag_image_topic:=/front_camera/image_raw_local
```

启动后触发执行（**键盘 G** 或 **话题触发**，二选一）：

```bash
# 方式1：键盘 G（需要本地终端，容器内 /dev/tty 不可用时请用方式2）

# 方式2：另开一个终端，发布触发话题
docker exec g1_robot_dev_dex3 bash -c \
  'source /opt/ros/humble/setup.bash && source /botbrain_ws/install/setup.bash && \
   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
   ros2 topic pub --once /apriltag/capture_trigger std_msgs/msg/Empty "{}"'
```

### 指定不同相机话题（可选）

```bash
ros2 launch unitree_g1_dex3_stack apriltag.launch.py \
  apriltag_image_topic:=/g1_robot/compressed_camera
```

### NoMachine 远程桌面（如需可视化调试）

```bash
# 宿主机
sudo /usr/NX/bin/nxserver --startup
```

---

## 4. 编译详解


### 全量编译（首次 / 修改了 CMakeLists.txt / 修改了 C++ 代码）

```bash
cd /botbrain_ws
colcon build --packages-select fcl trac_ik_lib unitree_g1_dex3_stack \
  --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON -DPython3_EXECUTABLE=/usr/bin/python3
pip3 install --no-build-isolation --no-deps -e src/g1_right_dex3/unitree_dex3_cpp
```

> 全量编译时机：首次部署、修改 C++ 源码、修改 CMakeLists.txt、fcl/trac_ik 有变化。

### 增量编译（只改了 Python 脚本 / launch 文件 / config yaml）

```bash
cd /botbrain_ws
colcon build --packages-select unitree_g1_dex3_stack \
  --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON -DPython3_EXECUTABLE=/usr/bin/python3
```

> 增量编译时机：只改了 `scripts/*.py`、`launch/*.py`、`config/*.yaml`，不涉及 C++ 或 CMake 变更。

编译成功标志：
```
Summary: 3 packages finished
Successfully installed unitree_cpp-1.0.3
```

> ⚠️ stderr 中出现 `warning` 是正常的，只要没有 `failed` 就成功。

---

## 5. 管理 dev_dex3 容器

```bash
cd /home/unitree/botbrain_ws/botbrain_project-main

# 首次启动（或机器重启后）
docker compose up -d dev_dex3

# 查看容器状态
docker compose ps dev_dex3

# 查看容器日志
docker compose logs -f dev_dex3

# 停止
docker compose stop dev_dex3

# 重启容器（更新代码后需重新 source，重启容器可确保环境干净）
docker compose restart dev_dex3
```

> `restart: unless-stopped` 已配置，机器重启后容器自动恢复（但不会自动执行 launch，需手动进入）。

---

## 6. 进入容器

```bash
# 宿主机
cd /home/unitree/botbrain_ws/botbrain_project-main
docker compose up -d dev_dex3
```

```bash
# 进入容器
docker exec -it g1_robot_dev_dex3 bash
```

```bash
# 容器内 source 环境（每次进入后执行）
source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
```

---

## 7. Launch 入口

| Launch 文件 | 用途 |
|---|---|
| `apriltag_button_press.launch.py` | **全流程**：AprilTag 检测 → OMPL 规划 → 手臂执行 → 灵巧手按压 |
| `elevator_button_press.launch.py` | **电梯视觉**：yolonas_ocr 检测楼层按钮 → 手臂按压 |
| `apriltag_reach.launch.py` | 端到端到达（不含灵巧手） |
| `apriltag.launch.py` | 仅相机 + AprilTag 调试 |
| `reach.launch.py` | 仅 planner 手动测试 |

---

## 8. 常用命令

> 以下命令均在**容器内**执行，先进入容器：
> ```bash
> docker exec -it g1_robot_dev_dex3 bash
> ```

### 验证节点是否正常运行

```bash
# 容器内
source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
ros2 node list
```

预期节点（至少包含）：
```
/ik_fcl_ompl_planner
/joint_state_publisher
/joint_trajectory_executor
/robot_state_publisher
/v4l2_apriltag_trigger
```

### 只启动相机识别（安全测试，不动手臂）

```bash
# 容器内
source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
ros2 launch unitree_g1_dex3_stack apriltag.launch.py detect_only:=true
```

### AprilTag 按钮按压（dry-run，不控制灵巧手）

```bash
# 容器内
source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py dry_run:=true
```

### AprilTag 按钮按压（真机执行）

> 使用 **ROS2 话题订阅**相机图像（`/g1_robot/front_camera/color/image_raw`），与 BotBrain 相机节点共存。
> 每个按钮位置对应不同的 launch 文件（yaml 里的 `offset_xyz` 不同）。

```bash
# 容器内
source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash

# 按压按钮
ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py dry_run:=false

# 打开按钮
ros2 launch unitree_g1_dex3_stack apriltag_button_open.launch.py dry_run:=false

# 关闭按钮
ros2 launch unitree_g1_dex3_stack apriltag_button_close.launch.py dry_run:=false

# 向上按钮
ros2 launch unitree_g1_dex3_stack apriltag_button_up.launch.py dry_run:=false

# 向下按钮
ros2 launch unitree_g1_dex3_stack apriltag_button_down.launch.py dry_run:=false
```

各按钮位置对应的 launch 文件：

| 位置 | Launch 文件 |
|------|-------------|
| 按下（press） | `apriltag_button_press.launch.py` |
| 向上（up） | `apriltag_button_up.launch.py` |
| 向下（down） | `apriltag_button_down.launch.py` |
| 打开（open） | `apriltag_button_open.launch.py` |
| 关闭（close） | `apriltag_button_close.launch.py` |

启动后按 **G 键** 触发按压序列。

### 电梯视觉按压

> 使用 **RealSense ROS 驱动**获取 RGB + 深度图像，需先在独立终端启动相机。
> **与 AprilTag 按压不同**：AprilTag 用 V4L2 直读，电梯视觉用 RealSense 深度图像。

**终端 1：启动 RealSense 深度相机**

```bash
docker exec -it g1_robot_dev_dex3 bash
```

```bash
# 容器内
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30
```

**终端 2：启动电梯视觉按压**

```bash
docker exec -it g1_robot_dev_dex3 bash
```

```bash
# 容器内
source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
ros2 launch unitree_g1_dex3_stack elevator_button_press.launch.py \
  target_floor:=5 det_threshold:=0.3
```

> ⚠️ `button_detector_node` 需约 **12 秒**加载 TensorFlow 模型，出现以下日志后再按 G：
> ```
> [button_detector_node] button_detector_node ready  backend=ros ...
> ```
> 系统最长等待 **60 秒**获取检测结果，出现 Ready 后可直接按 G。

### 手臂回站立姿态（紧急恢复）

```bash
# 容器内
source /opt/ros/humble/setup.bash
ros2 topic pub /executor/return_to_standing std_msgs/msg/Empty '{}' --once
```

### 手动触发相机拍照

```bash
# 容器内
source /opt/ros/humble/setup.bash
ros2 topic pub /apriltag/capture_trigger std_msgs/msg/Empty '{}' --once
```

### 手动发布目标位姿

```bash
# 容器内
source /opt/ros/humble/setup.bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "torso_link"}, pose: {position: {x: 0.4, y: -0.2, z: 0.18}, orientation: {w: 1.0}}}' --once
```

### 灵巧手控制

```bash
# 容器内
# 伸出中指
python3 /botbrain_ws/src/g1_right_dex3/unitree_dex3_cpp/example/control_dex3_right_setpoint.py \
  enP8p1s0 0 -1.05 -1.7 1.7 1.8 0 0

# 合拢
python3 /botbrain_ws/src/g1_right_dex3/unitree_dex3_cpp/example/control_dex3_right_setpoint.py \
  enP8p1s0 0 -1.05 -1.7 1.7 1.8 1.7 1.8
```

### 右臂卸力 / 锁定模式

```bash
# 容器内
source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
python3 /botbrain_ws/src/g1_right_dex3/right_arm_mode.py
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

### 旧容器冲突

原来的 `unitree-dex3-dev` 容器（通过 `run.sh` 启动）**不能与 `dev_dex3` 同时运行**，会产生 DDS 话题冲突：

```bash
docker stop unitree-dex3-dev
```

### 话题冲突风险

| 话题 / 资源 | 状态 |
|------|------|
| `/tf` / `/tf_static` | ✅ 已用 `/unitree_g1_dex3/tf` 命名空间隔离 |
| `/lf/lowstate` | ✅ 只读订阅，无冲突 |
| `/joint_states` | ⚠️ 与 BotBrain 可能重复发布，需监控 |
| `rt/arm_sdk` | ⚠️ `g1_manipulation_pkg` 与本程序不能同时控制右臂（见踩坑记录 §10-E） |
| `/dev/video4`（V4L2）| ✅ 不再使用 V4L2，改为订阅 ROS2 话题，与 BotBrain 相机节点共存无冲突 |

### 代码修改后必须重新编译

Python 脚本（install 安装的）修改后也需要 `colcon build`，因为 colcon 将文件安装到 `install/` 目录下，节点执行的是安装后的副本，不是 `src/` 下的原文件。

### 不要用 conda python

编译时明确指定 `-DPython3_EXECUTABLE=/usr/bin/python3`，防止 cmake 误用 conda 环境的 python 导致绑定失败。

---

## 11. 集成踩坑记录

并入 BotBrain 过程中遇到的问题与修复方案。

### A. fcl 残缺版导致编译失败

**现象**：
```
include could not find load file: CMakeModules/CompilerSettings
include could not find load file: CMakeModules/FCLVersion
src/fcl/src does not contain CMakeLists.txt
```

**原因**：BotBrain 仓库中的 `botbrain_ws/src/fcl/` 是残缺版，缺少 `CMakeModules/` 目录和子目录的 `CMakeLists.txt`。

**修复**：用原项目 Desktop 的完整 fcl 覆盖：
```bash
rsync -av /home/unitree/Desktop/unitree_dex3/src/fcl/ \
  /home/unitree/botbrain_ws/botbrain_project-main/botbrain_ws/src/fcl/
```

---

### B. 重复包名冲突

**现象**：
```
colcon ERR: found multiple packages with same name 'fcl'
```

**原因**：复制代码时将原项目的 `fcl` 一并放入 `g1_right_dex3/`，与顶层 `src/fcl/` 重名。

**修复**：删除 `g1_right_dex3/fcl/`，统一使用顶层 `src/fcl/`：
```bash
rm -rf botbrain_ws/src/g1_right_dex3/fcl
```

---

### C. trac_ik_python 未知包

**现象**：
```
colcon WARN: ignoring unknown package 'trac_ik_python'
```

**原因**：`trac_ik_python` 已预装在 `unitree-dex3:humble` 镜像中，无需从源码编译，但 `--packages-select` 里仍包含了它。

**修复**：从 `--packages-select` 列表中移除 `trac_ik_python`，只编译 `fcl trac_ik_lib unitree_g1_dex3_stack`。

---

### D. button_detector_node.py 找不到可执行文件

**现象**：
```
ros2 launch: executable 'button_detector_node' not found on libexec directory
```

**原因**：`unitree_g1_dex3_stack/CMakeLists.txt` 的 `install(PROGRAMS ...)` 段遗漏了 `scripts/button_detector_node.py`。

**修复**：在 `CMakeLists.txt` 的 `install(PROGRAMS)` 段补充该文件：
```cmake
install(PROGRAMS
  scripts/button_detector_node.py
  ...
  DESTINATION lib/${PROJECT_NAME}
)
```

---

### E. 电机发生震颤（双节点同时写 rt/arm_sdk）

**现象**：启动 launch 后手臂电机发生明显震颤，关节高频抖动。

**原因**：**两个 `joint_trajectory_executor` 节点同时向 `rt/arm_sdk` 写入指令**。

具体过程：
1. `docker-compose.yaml` 中配置了 `dex3_stack` 服务，`command` 为自动运行 `ros2 launch apriltag_button_press.launch.py`
2. 用户在 `dev_dex3` 容器里又手动运行了一次 launch
3. 两个实例同时发送相互冲突的关节指令，产生震颤

**修复**：删除 `dex3_stack` 服务，只保留 `dev_dex3`（`command: sleep infinity`），所有 launch 均手动执行。

> ⚠️ 同理，BotBrain 中的 `g1_manipulation_pkg`（`arm_controller`）也写入 `rt/arm_sdk`，运行本程序时必须停止该服务，否则两者会互相抢占右臂控制权。

---

### F. camera_only 模式反复报错 "no floor detected"

**现象**：`camera_only:=true` 启动后终端疯狂滚动 `[button_detector_node] no floor detected`，不是相机预览行为。

**原因**：`apriltag_button_press.launch.py` 的 `camera_only_actions` 列表错误引用了 `button_detector_node`（电梯视觉节点），而非相机触发节点 `v4l2_trigger_node`。

**修复**：将 `camera_only_actions` 改为只启动 `v4l2_trigger_node`：
```python
camera_only_actions = [v4l2_trigger_node]   # 不是 button_detector_node
```

---

### G. 磁盘空间不足导致 rsync 失败

**现象**：rsync 复制大文件时中途失败：
```
rsync: write failed on "...": No space left on device (28)
```

**原因**：机器人主机磁盘剩余空间不足，`yolonas_ocr` 模型文件较大。

**处理方式**：分批次复制，优先复制代码文件，模型文件单独处理；或先清理磁盘空间。

---

### H. 路径未迁移导致运行时找不到文件

**现象**：launch 启动后节点报 `FileNotFoundError` 或图片保存失败。

**原因**：config yaml 文件中仍残留原项目路径（`/workspaces/`），未更新为 BotBrain 路径（`/botbrain_ws/`）。

**涉及字段**：
- `debug_image_dir`：`/workspaces/unitree_dex3/detect_img` → `/botbrain_ws/detect_img`
- `dex3_setpoint_script`：`/workspaces/unitree_dex3_cpp/example/...` → `/botbrain_ws/src/g1_right_dex3/unitree_dex3_cpp/example/...`

**修复**：批量替换所有 config yaml 中的路径（共 10 个文件）。

---

### I. pydantic v2 不兼容警告

**现象**：运行 `control_dex3_right_setpoint.py` 时出现：
```
PydanticUserError: `model_fields` is not supported in Pydantic v1
```

**原因**：`unitree_dex3_cpp/example/config.py` 使用了 pydantic v1 语法，镜像内为 v2。

**当前状态**：不影响实际功能，暂未修复，后续可将 `config.py` 升级为 v2 语法。

---

## 12. 问题排查

### 容器没起来 / 异常退出

```bash
# 宿主机
docker compose logs dev_dex3 | tail -50
```

### 手臂没有正常回到站立 / Dex-3 超时

> ⚠️ 不要直接关掉报错终端，否则下次启动手臂会瞬间冲击回原点。

```bash
# 容器内
source /opt/ros/humble/setup.bash
ros2 topic pub /executor/return_to_standing std_msgs/msg/Empty '{}' --once
```

### 相机话题不存在

检查相机节点是否正常运行：

```bash
# 容器内
ros2 topic list | grep front_camera
```

如果没有 `/g1_robot/front_camera/color/image_raw`，需要检查 BotBrain 相机服务状态。

### TRAC-IK 无解循环

planner 日志出现反复 `No solution found`，目标超出可达范围，检查 `reach_max_distance` 参数或手动发布可达目标位姿。

### 手臂震颤

立即检查是否有多个 `joint_trajectory_executor` 在运行：

```bash
# 容器内
source /opt/ros/humble/setup.bash
ros2 node list | grep executor
```

如有多个，停止其他服务（特别是 `g1_manipulation`）：

```bash
# 宿主机
docker compose stop g1_manipulation
```

---

## 13. 修改 docker-compose.yaml 后的操作流程

修改 compose 文件后，**不要直接 `docker compose down` + `docker compose up`**，这会影响所有其他 BotBrain 服务。

### 只改了 compose 配置（未改代码）

```bash
docker compose stop dev_dex3
docker compose up -d dev_dex3
```

### 改了代码（Python / launch / config yaml）

```bash
# 先编译
docker compose run --rm builder_dex3
# 再重启
docker compose stop dev_dex3
docker compose up -d dev_dex3
```

### 改了 C++ 代码或 CMakeLists.txt

同上，`builder_dex3` 内部会全量编译 fcl + trac_ik_lib + unitree_g1_dex3_stack。

> ⚠️ `docker compose down` 会停止并删除**所有**服务容器（包括 BotBrain 的 bringup、navigation 等），非必要不要用。

---

## 14. 复制到另一台机器人

### 需要复制的内容

| 内容 | 路径 |
|------|------|
| 功能代码 | `botbrain_ws/src/g1_right_dex3/` |
| 修复后的 fcl | `botbrain_ws/src/fcl/` |
| compose 服务定义 | `docker-compose.yaml`（含 dev_dex3 + builder_dex3） |
| Docker 镜像 | `unitree-dex3:humble`（约 10.7GB，压缩后约 3-4GB） |

编译产物（`botbrain_ws/install/`）不需要复制，在目标机重新编译即可。

### 第一步：传输代码（在源机器人上执行）

```bash
rsync -av --progress \
  botbrain_ws/src/g1_right_dex3/ \
  unitree@<目标机IP>:/home/unitree/botbrain_ws/botbrain_project-main/botbrain_ws/src/g1_right_dex3/

rsync -av --progress \
  botbrain_ws/src/fcl/ \
  unitree@<目标机IP>:/home/unitree/botbrain_ws/botbrain_project-main/botbrain_ws/src/fcl/

scp docker-compose.yaml \
  unitree@<目标机IP>:/home/unitree/botbrain_ws/botbrain_project-main/docker-compose.yaml
```

### 第二步：传输镜像（在源机器人上执行）

```bash
# 导出压缩（约 3-4GB）
docker save unitree-dex3:humble | gzip > /tmp/unitree-dex3.tar.gz

# 传输（会提示输入目标机密码）
scp /tmp/unitree-dex3.tar.gz unitree@<目标机IP>:/tmp/

rm /tmp/unitree-dex3.tar.gz
```

### 第三步：目标机导入并编译

```bash
ssh unitree@<目标机IP>
```

```bash
# 导入镜像
docker load < /tmp/unitree-dex3.tar.gz
rm /tmp/unitree-dex3.tar.gz

cd /home/unitree/botbrain_ws/botbrain_project-main

# 编译
docker compose run --rm builder_dex3

# 启动
docker compose up -d dev_dex3
```
