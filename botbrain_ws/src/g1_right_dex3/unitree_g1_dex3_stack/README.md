# unitree_g1_dex3_stack

Unitree G1 右臂 + Dex-3 灵巧手 ROS 2 全栈：AprilTag V4L2 检测 → OMPL 规划 → 右臂执行 → 灵巧手按压。

**运行环境**：所有 ROS 2 节点均在 Docker 容器内运行（`unitree-dex3:humble`），宿主机通过 `run.sh` 启动。

---

## 1. Docker 环境

```bash
# 启动容器（交互式 shell，自动 source ROS 和 install_container）
cd /home/unitree/Desktop/unitree_container
./run.sh

# 容器内 shell 已自动 source：
#   /opt/ros/humble/setup.bash
#   /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
#   /workspaces/unitree_dex3/install_container/setup.bash

# 直接在容器内执行单条命令（不进入 shell）
./run.sh <command>
```

> **注意**：首次 `./run.sh` 会创建后台容器（`unitree-dex3-dev`），后续调用复用已有容器。

### 1.1 新机器从零部署

新机器建议按 **Docker 部署**，不要走旧的原生 ROS 编译流程。

#### 需要拷贝的目录

默认放在新机器的 `/home/unitree/Desktop/` 下：

| 目录 | 是否必须 | 说明 |
|---|---|---|
| `unitree_container/` | 必须 | Dockerfile、镜像构建脚本、容器启动脚本、DDS/环境变量配置 |
| `unitree_dex3/` | 必须 | ROS 2 工作区；至少需要 `src/fcl`、`src/trac_ik`、`src/unitree_g1_dex3_stack-main` |
| `unitree_dex3_cpp/` | Button Press 必须 | Dex-3 灵巧手 setpoint 脚本和 `unitree_cpp` Python 绑定 |
| `xr_teleoperate/` | 可选 | 仅 `right-arm-mode` 需要其 URDF/资源 |
| `unitree_sdk2_python/` | 可选 | 镜像内默认已安装；本地挂载只是备用 |

不需要拷贝 `build/`、`install/`、`log/`、`build_container/`、`install_container/`、`log_container/`，这些都是编译产物，到新机器后重新生成。

#### Step 1：构建 Docker 镜像

```bash
cd /home/unitree/Desktop/unitree_container
bash build.sh
```

如果需要代理：

```bash
USE_PROXY=1 PROXY_URL=http://192.168.100.20:7890 bash build.sh
```

#### Step 2：首次全量编译 ROS 2 工作区

首次编译必须让 `colcon` 从 `src/` 里按依赖顺序构建 `fcl`、`trac_ik_lib` 和主包：

```bash
cd /home/unitree/Desktop/unitree_container
./run.sh bash -lc 'colcon --log-base /workspaces/unitree_dex3/log_container build \
  --base-paths /workspaces/unitree_dex3/src \
  --build-base /workspaces/unitree_dex3/build_container \
  --install-base /workspaces/unitree_dex3/install_container \
  --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON -DPython3_EXECUTABLE=/usr/bin/python3'
```

> **坑**：首次编译不要加 `--packages-select unitree_g1_dex3_stack`。主包依赖源码工作区里的 `fcl` 和 `trac_ik_lib`，必须先把依赖编出来。后续只改主包时才用 `--packages-select unitree_g1_dex3_stack` 增量编译。

#### Step 3：编译 Dex-3 Python 绑定

Button Press 的灵巧手控制脚本依赖 `unitree_dex3_cpp`：

```bash
cd /home/unitree/Desktop/unitree_container
./run.sh bash -lc 'cd /workspaces/unitree_dex3_cpp && \
  python3 -m pip install -e . --no-build-isolation --no-deps && \
  python3 -c "import unitree_cpp; print(\"unitree_cpp OK\")"'
```

#### Step 4：基础验证

```bash
cd /home/unitree/Desktop/unitree_container

# 主包是否可被 ROS 找到
./run.sh bash -lc 'ros2 pkg prefix unitree_g1_dex3_stack'

# 安全验证：只启动相机识别，不启动 planner / executor / button press 节点
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py camera_only:=true
```

#### 新机器需要检查的硬件参数

| 参数 | 位置 | 说明 |
|---|---|---|
| `UNITREE_NET_IF` | `unitree_container/run.sh` 或启动前环境变量 | G1 通信网卡，当前默认 `enP8p1s0` |
| `expected_serial` | `config/apriltag_button_press.yaml` / `config/v4l2_apriltag_trigger.yaml` | D435i 序列号，换相机后需要改 |
| `camera_matrix` | 同上 | D435i RGB 相机内参，严格场景应使用新相机标定值 |

---

## 2. 编译

所有编译必须在容器内执行，使用 `build_container` / `install_container` / `log_container` 三套目录。

### 首次全量编译（从宿主机）

```bash
cd /home/unitree/Desktop/unitree_container
./run.sh bash -lc 'colcon --log-base /workspaces/unitree_dex3/log_container build \
  --base-paths /workspaces/unitree_dex3/src \
  --build-base /workspaces/unitree_dex3/build_container \
  --install-base /workspaces/unitree_dex3/install_container \
  --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON -DPython3_EXECUTABLE=/usr/bin/python3'
```

### 后续只重编主包

```bash
cd /home/unitree/Desktop/unitree_container
./run.sh bash -lc 'colcon --log-base /workspaces/unitree_dex3/log_container build \
  --base-paths /workspaces/unitree_dex3/src \
  --build-base /workspaces/unitree_dex3/build_container \
  --install-base /workspaces/unitree_dex3/install_container \
  --packages-select unitree_g1_dex3_stack \
  --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON -DPython3_EXECUTABLE=/usr/bin/python3'
```

编译后容器 shell 会自动 source `install_container/setup.bash`，无需手动 source。

> **坑**：修改源码后必须重新编译，`install_container/` 是编译产物的拷贝，直接改源码不会生效。

---

## 3. Launch 入口总览

| Launch 文件 | 用途 |
|---|---|
| `apriltag_button_press.launch.py` | **按钮按压全流程**：AprilTag 检测 → planner → executor → Dex-3 灵巧手伸指/按压/收回 |
| `apriltag_reach.launch.py` | **端到端到达**：AprilTag 检测 → G 键触发 → planner → executor（不含灵巧手） |
| `reach.launch.py` | **Planner 手动测试**：仅 robot + planner + executor，手动 pub `/goal_pose` |
| `apriltag.launch.py` | **相机独立调试**：仅 robot + V4L2 AprilTag 检测，不启动 planner/executor |

---

## 4. Button Press（按钮按压）

### 4.1 快速启动

```bash
cd /home/unitree/Desktop/unitree_container

# Dry-run（不控制灵巧手，安全测试规划和手臂运动）
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py dry_run:=true

# 真机执行（含灵巧手伸指和按压）
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py dry_run:=false

# 只启动相机识别（不启动 planner / executor / button press 节点）
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py camera_only:=true
```

启动后在终端**按 G 键**触发一次按压序列。

### 4.2 按压执行流程

按 G 后，`apriltag_button_press_node` 依次执行：

1. **拍照 & 检测**：发布 `/apriltag/capture_trigger` → V4L2 trigger 拍照 → AprilTag 检测 → 发布 `/apriltag/target_pose`
2. **pre-contact**：手臂先运动到目标前方 `pre_contact_offset_x`（默认 5cm）处
3. **伸出中指**：调用 Dex-3 setpoint 脚本设置 `pre_extend_pose`
4. **press-target**：手臂前进到 tag 目标位置（按压）
5. **retreat**：手臂退回 pre-contact 位置
6. **合上手指**：调用 Dex-3 setpoint 脚本设置 `close_pose`
7. **return-to-standing**：发布 `/executor/return_to_standing`，手臂回到站立姿态

### 4.3 Launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `dry_run` | `false` | 跳过 Dex-3 灵巧手 subprocess 调用 |
| `camera_only` | `false` | 只启动相机和检测，不启动规划/控制 |
| `planning_timeout` | `1.0` | OMPL 规划超时（秒） |
| `v4l2_video_device` | `auto` | V4L2 设备路径，`auto` 自动扫描 D435i |
| `debug_image_dir` | `/workspaces/unitree_dex3/detect_img` | AprilTag 调试图像保存目录 |

### 4.4 关键配置文件

**`config/apriltag_button_press.yaml`** — 三个节点的参数集中配置：

#### V4L2 相机 & AprilTag 检测

| 参数 | 默认值 | 说明 |
|---|---|---|
| `expected_serial` | `253243060636` | D435i 序列号，用于 auto 模式设备匹配 |
| `video_device` | `auto` | V4L2 设备路径，`auto` 自动扫描 |
| `image_width` / `image_height` | `640` / `480` | 采集分辨率 |
| `warmup_frames` | `12` | 按 G 后先丢弃的暖机帧数 |
| `warmup_min_s` | `2.0` | 暖机最低时间（秒） |
| `sample_count` | `4` | 每次触发采样帧数 |
| `tag_size` | `0.08` | AprilTag 物理尺寸（米） |
| `target_tag_id` | `0` | 目标 tag ID |
| `offset_xyz` | `[0.0, 0.0, -0.1]` | tag 局部坐标系下的目标偏移 |
| `reach_max_distance` | `0.55` | 肩到目标最大距离（米） |
| `camera_matrix` | `[602.02, 0, 330.96, ...]` | 相机内参（640×480，出厂标定值） |

#### Button Press 节点

| 参数 | 默认值 | 说明 |
|---|---|---|
| `trigger_key` | `g` | 触发键 |
| `base_rpy` | `[0.0117, -0.0823, 0.0013]` | 默认末端朝向（RPY 弧度） |
| `alt_rpy_y_threshold` | `-0.0954` | 目标 y > 此值时切换备选朝向 |
| `alt_rpy` | `[-0.0340, -0.1024, 0.5480]` | 备选末端朝向 |
| `pre_contact_offset_x` | `0.05` | pre-contact 在 x 方向回退距离（米） |
| `pre_extend_pose` | `[0,-1.05,-1.7,1.7,1.8,0,0]` | 灵巧手伸中指关节角 |
| `close_pose` | `[0,-1.05,-1.7,1.7,1.8,1.7,1.8]` | 灵巧手合拢关节角 |
| `dex3_setpoint_script` | `/workspaces/unitree_dex3_cpp/example/control_dex3_right_setpoint.py` | 灵巧手控制脚本路径 |
| `dex3_net_if` | `enP8p1s0` | 灵巧手通信网卡 |
| `capture_wait_timeout_s` | `6.0` | 等待 AprilTag 检测结果超时 |
| `traj_wait_timeout_s` | `5.0` | 等待轨迹规划结果超时 |
| `traj_completion_buffer_s` | `0.8` | 轨迹执行完后额外等待时间 |
| `ramp_wait_s` | `3.5` | return-to-standing 后等待 ramp 完成 |

#### Planner

| 参数 | 默认值 | 说明 |
|---|---|---|
| `tcp_offset_x` | `0.12` | TCP 相对末端 link 的 x 偏移 |
| `adaptive_orientation_enabled` | `false` | button press 模式下关闭自适应朝向 |
| `velocity_scale` | `0.01` | 轨迹速度缩放 |
| `fallback_enabled` | `true` | IK/OMPL 失败时自动回退搜索 |
| `ik_timeout` | `0.1` | 单次 TRAC-IK 超时（秒） |

---

## 5. AprilTag Reach（到达）

```bash
cd /home/unitree/Desktop/unitree_container

# 端到端（检测 + 规划 + 执行）
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py

# 只检测不执行
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py detect_only:=true

# 只启动相机
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py camera_only:=true
```

启动后按 **G 键**触发检测和手臂运动。配置文件为 `config/v4l2_apriltag_trigger.yaml` + `config/apriltag.yaml`。

---

## 6. 其他常用操作

### 手动发布目标位姿

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "torso_link"}, pose: {position: {x: 0.4, y: -0.2, z: 0.18}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' --once
```

### 读取右臂 TCP 位姿

```bash
# 先确保 robot.launch 在运行
ros2 run unitree_g1_dex3_stack tcp_torso_pose.py
```

### 手动触发相机拍照

```bash
ros2 topic pub /apriltag/capture_trigger std_msgs/msg/Empty '{}' --once
```

### 手动发送 return-to-standing

```bash
ros2 topic pub /executor/return_to_standing std_msgs/msg/Empty '{}' --once
```

### 右臂拖拽模式（卸力 / 锁定）

```bash
/home/unitree/Desktop/unitree_container/run.sh right-arm-mode
# 交互命令：free = 卸力拖拽, lock = 锁定, status = 查看关节角
```

### 灵巧手控制（容器内）

```bash
# 伸出中指
python3 /workspaces/unitree_dex3_cpp/example/control_dex3_right_setpoint.py enP8p1s0 0 -1.05 -1.7 1.7 1.8 0 0

# 合上
python3 /workspaces/unitree_dex3_cpp/example/control_dex3_right_setpoint.py enP8p1s0 0 -1.05 -1.7 1.7 1.8 1.7 1.8
```

## 7. 问题排查

### 相机打不开 / V4L2 设备占用

```bash
# 检查 /dev/video4 是否被占用
fuser -v /dev/video4

# 常见占用者：teleimager.image_server
# 杀掉占用进程后重试
kill <PID>
```

> V4L2 auto 模式扫描 `/dev/v4l/by-path/` 和 `/dev/v4l/by-id/`，匹配 D435i 序列号 `253243060636` 和 USB interface `03`。
> 当前机器上通常是 `/dev/video4`。

### 图像偏暗

冷启动时前几帧可能偏暗。已通过 `warmup_frames: 12` 和 `warmup_min_s: 2.0` 缓解。
如果仍然偏暗，可尝试增大 `warmup_frames` 或启用 `continuous_capture: true`（但会增加 CPU 占用）。

### TRAC-IK 无解（"No solution found" 循环）

planner 日志中出现反复的 `TRAC-IK random-seed result: No solution found`。
当前已调优：`ik_timeout: 0.1`，`ik_random_seed_tries: 3`，失败后只输出一条 WARN 汇总。
如果目标确实不可达，会触发 fallback 回退搜索（沿肩→目标方向收缩，步长 `fallback_step: 0.005`）。

### Dex-3 setpoint 脚本找不到

```
Dex-3 setpoint script not found: /home/unitree/Desktop/unitree_dex3_cpp/example/control_dex3_right_setpoint.py
```

路径必须是**容器内路径** `/workspaces/unitree_dex3_cpp/example/control_dex3_right_setpoint.py`，不是宿主机路径 `/home/unitree/Desktop/...`。已在配置中修正。

### rclpy import 失败

宿主机有 Miniforge/Conda Python 3.13，会覆盖 `/usr/bin/env python3`。
所有 ROS 2 Python 节点 shebang 已改为 `#!/usr/bin/python3`（硬编码系统 Python 3.10）。

### launch 退出时节点超时被 SIGKILL

`apriltag_button_press_node` 已处理 SIGINT/SIGTERM：信号回调中调用 `rclpy.try_shutdown()`，避免被 launch 系统 10s 超时后强杀。

### DDS 断言崩溃

```
dds_writecdr_impl_common: Assertion ... failed
```

Python cyclonedds 意外加载了 ROS Humble 的 `/opt/ros/humble/.../libddsc.so`。
容器内通过 `LD_LIBRARY_PATH=/usr/local/lib:...` 确保优先加载 `/usr/local/lib/libddsc.so.0`。

---

## 8. 已知的坑

1. **源码改完必须重编译** — `install_container/` 是编译拷贝，直接改 `src/` 下的源码不会生效
2. **路径是容器路径** — 配置文件中所有路径（`dex3_setpoint_script`、`debug_image_dir` 等）都是 `/workspaces/...`，不是 `/home/unitree/Desktop/...`
3. **V4L2 设备独占** — 同一时间只能一个进程打开 `/dev/video4`。启动 AprilTag 前必须确认 teleimager 等没有占用
4. **不要用 conda python 跑 ROS 节点** — conda 环境下的 python3 无法 import rclpy
5. **button press 中 `detect_only: true`** — V4L2 trigger 在 button press launch 中固定为 detect_only 模式，它只检测和发布 pose，不直接发布 `/goal_pose`。按压流程由 `apriltag_button_press_node` 编排
6. **`trigger_key: ""` vs `trigger_key: "g"`** — button press 的 V4L2 trigger 不监听键盘（`trigger_key` 为空），而是由 button press 节点通过 `/apriltag/capture_trigger` topic 触发拍照
7. **`auto_return_to_standing: false`** — button press launch 关闭了 executor 的自动回站立，由 button press 节点在序列完成后主动发送 `/executor/return_to_standing`
8. **base_rpy / alt_rpy** — 按压朝向不用 `adaptive_orientation`，而是根据 tag y 坐标选择 `base_rpy` 或 `alt_rpy`。调姿态时改这两个值
9. **warmup 不能太短** — D435i 冷启动需要 2 秒以上才能稳定曝光，`warmup_min_s` 不要低于 2.0
10. **TF 命名空间隔离** — 所有节点使用 `/unitree_g1_dex3/tf` 和 `/unitree_g1_dex3/tf_static`，不是默认的 `/tf`。其他工具监听时注意 remap

---

## 9. 文件结构

```
src/unitree_g1_dex3_stack-main/
├── config/
│   ├── apriltag.yaml                  # apriltag_reach 配置（planner 参数）
│   ├── apriltag_button_press.yaml     # button press 全部参数
│   └── v4l2_apriltag_trigger.yaml     # apriltag_reach 的 V4L2 相机参数
├── launch/
│   ├── apriltag_button_press.launch.py
│   ├── apriltag_reach.launch.py
│   ├── apriltag.launch.py
│   ├── reach.launch.py
│   ├── planner.launch.py
│   ├── control.launch.py
│   └── robot.launch.py
├── scripts/
│   ├── v4l2_apriltag_trigger.py       # V4L2 AprilTag 检测节点
│   ├── apriltag_button_press_node.py  # 按压序列编排节点
│   ├── tcp_torso_pose.py              # TCP 位姿读取工具
│   └── apriltag_reach_uat.py          # 到达 UAT 测试
└── src/
    ├── ik_fcl_ompl_planner.cpp        # C++ OMPL 规划器
    └── joint_trajectory_executor.cpp  # C++ 轨迹执行器
```

---

## 10. ROS 2 Topic 速查

| Topic | 类型 | 说明 |
|---|---|---|
| `/apriltag/capture_trigger` | `std_msgs/Empty` | 触发 V4L2 拍照 |
| `/apriltag/tag_pose` | `geometry_msgs/PoseStamped` | tag 原始位姿（torso_link 系） |
| `/apriltag/target_pose` | `geometry_msgs/PoseStamped` | tag + offset 目标位姿 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 规划目标（planner 输入） |
| `/joint_trajectory_targets` | `trajectory_msgs/JointTrajectory` | 规划输出轨迹 |
| `/executor/return_to_standing` | `std_msgs/Empty` | 触发手臂回站立 |
