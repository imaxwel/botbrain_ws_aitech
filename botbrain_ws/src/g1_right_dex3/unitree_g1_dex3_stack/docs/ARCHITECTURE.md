# 技术架构

## 1. 系统总览

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Docker 容器 (unitree-dex3:humble)                │
│                                                                         │
│  ┌──────────┐   ┌──────────────────┐   ┌───────────┐   ┌────────────┐  │
│  │ robot    │   │ v4l2_apriltag    │   │ ik_fcl    │   │ joint      │  │
│  │ _state   │   │ _trigger (Py)    │   │ _ompl     │   │ _trajectory│  │
│  │ _publisher│   │                  │   │ _planner  │   │ _executor  │  │
│  │ (ROS2)   │   │ AprilTag 检测     │   │ (C++)     │   │ (C++)      │  │
│  └────┬─────┘   └────────┬─────────┘   └─────┬─────┘   └──────┬─────┘  │
│       │                  │                    │                │         │
│       │   /joint_states  │ /apriltag/         │ /joint_        │ /arm   │
│       │                  │  target_pose       │  trajectory    │  _sdk  │
│       │                  │                    │  _targets      │        │
│  ┌────┴─────┐            │              ┌─────┴─────┐          │        │
│  │ joint    │            │              │           │          │        │
│  │ _state   │            │              │ /goal_pose│          │        │
│  │ _publisher│            │              │           │          │        │
│  │ (C++)    │            │              └───────────┘          │        │
│  └──────────┘            │                                     │        │
│                          │   ┌──────────────────────┐          │        │
│                          │   │ apriltag_button      │          │        │
│                          └──▶│ _press_node (Py)     ├──────────┘        │
│                              │ 序列编排器            │                   │
│                              └──────────────────────┘                   │
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ Unitree SDK (DDS)                                                │   │
│  │   /lf/lowstate ← G1 机体      /arm_sdk → G1 右臂 7-DOF          │   │
│  │   Dex-3 setpoint → 灵巧手 7-DOF (subprocess, unitree_sdk2py)    │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
        │                                                   │
    ┌───┴───┐                                         ┌─────┴─────┐
    │D435i  │ V4L2 /dev/video4                        │ G1 机体    │
    │RGB    │ 640×480 YUYV                            │ DDS Topic │
    └───────┘                                         └───────────┘
```

---

## 2. 节点职责

### 2.1 基础层（robot.launch.py）

| 节点 | 语言 | 职责 |
|---|---|---|
| `robot_state_publisher` | C++ (ROS2 内置) | 加载 URDF，发布 `/unitree_g1_dex3/tf_static`，提供 `robot_description` 参数 |
| `joint_state_publisher` | C++ | 订阅 `/lf/lowstate` (Unitree SDK DDS)，解析 29-DOF 关节角度 + Dex-3 手指角度，发布 `/joint_states` |

URDF 文件：`robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf`

### 2.2 感知层（V4L2 AprilTag Trigger）

| 节点 | 语言 | 职责 |
|---|---|---|
| `v4l2_apriltag_trigger` | Python | V4L2 采集 D435i RGB → pupil-apriltags 检测 → TF 变换到 torso_link → 发布 tag/target pose |

**工作模式**：
- **on-demand**（默认）：收到触发后才打开 V4L2 设备，暖机 → 采样 → 检测 → 关闭设备
- **continuous**：后台线程持续读帧，触发时从缓存取最新帧

**触发方式**：
- 键盘 G 键（`trigger_key: "g"`）— 用于 `apriltag_reach`
- Topic `/apriltag/capture_trigger`（`trigger_key: ""`）— 用于 `button_press`，由编排节点发布

**输出**：
- `/apriltag/tag_pose` — tag 原始位姿（torso_link 系）
- `/apriltag/target_pose` — tag + `offset_xyz` 偏移后的目标位姿

### 2.3 规划层（ik_fcl_ompl_planner）

| 节点 | 语言 | 职责 |
|---|---|---|
| `ik_fcl_ompl_planner` | C++ | 接收 `/goal_pose` → TRAC-IK 求解 → OMPL RRTConnect 规划 → 发布关节轨迹 |

**核心流程**：

```
/goal_pose 到达
    │
    ▼
[自适应朝向] ← 可选：根据 shoulder→target 方向计算末端朝向
    │
    ▼
[TRAC-IK] ← 目标笛卡尔位姿 → 7-DOF 关节角
    │
    ├── 成功 → [OMPL RRTConnect] 当前关节角 → 目标关节角
    │               │
    │               ├── 成功 → 轨迹简化 → 时间参数化(velocity_scale) → 发布 /joint_trajectory_targets
    │               └── 失败 → fallback
    │
    └── 失败 → fallback: 沿 shoulder→target 方向逐步回退，每步重试 IK+OMPL
```

**Fallback 策略**：
- 从原目标向 `right_shoulder_pitch_link` 方向回退
- 步长 `fallback_step`（0.005m），最大回退 `fallback_max_retraction`（0.05m）

**碰撞检测**（可选）：
- FCL 库，基于 URDF 碰撞几何
- 默认关闭（`collision_detection_enabled: false`），因为右臂操作空间碰撞风险低

### 2.4 执行层（joint_trajectory_executor）

| 节点 | 语言 | 职责 |
|---|---|---|
| `joint_trajectory_executor` | C++ | 接收关节轨迹 → 250Hz 线性插值 → KDL 重力补偿 → 发布 `/arm_sdk` (LowCmd) |

**核心特性**：
- 订阅 `/joint_trajectory_targets`，沿 waypoint 做 250Hz (4ms) 线性插值
- 每个控制周期叠加 KDL RNEA 计算的重力力矩补偿
- 轨迹执行完毕后进入 **hold** 模式（维持最后关节角 + 重力补偿）
- 收到 `/executor/return_to_standing` 后，**ramp** 回站立姿态（kp/kd 渐变到 0）
- `auto_return_to_standing`：轨迹执行完自动回站立（reach 模式开启，button_press 关闭）

**控制参数**：
- 右臂 7-DOF：shoulder×3 + elbow×1 + wrist×3
- kp/kd 分关节配置，wrist kd = 5.0
- 输出到 Unitree `/arm_sdk` DDS topic（`unitree_hg/LowCmd`）

### 2.5 编排层（apriltag_button_press_node）

| 节点 | 语言 | 职责 |
|---|---|---|
| `apriltag_button_press_node` | Python | 键盘触发 → 拍照 → 规划 → 手臂运动 → 灵巧手控制 → 回位 |

**不同于 reach 的关键设计**：
- V4L2 trigger 以 `detect_only=true` 运行，不自主发布 `/goal_pose`
- 由 button_press_node 通过 topic 触发拍照、收集 target_pose、构造 goal_pose
- 末端朝向不用 adaptive_orientation，而是 RPY 查表（`base_rpy` / `alt_rpy`）
- 灵巧手通过 subprocess 调用 `control_dex3_right_setpoint.py`（独立于 ROS）

---

## 3. 数据流

### 3.1 AprilTag Reach 数据流

```
[键盘 G] → v4l2_apriltag_trigger
               │
               ├── /apriltag/target_pose ──▶ (内部) ──▶ /goal_pose
               │
               ▼
         ik_fcl_ompl_planner
               │
               ├── /joint_trajectory_targets
               │
               ▼
         joint_trajectory_executor
               │
               ├── /arm_sdk (LowCmd)
               │
               ▼
           G1 右臂运动
               │
               ▼
         [auto_return_to_standing] → ramp 回站立
```

### 3.2 Button Press 数据流

```
[键盘 G] → apriltag_button_press_node
               │
               ├── /apriltag/capture_trigger ──▶ v4l2_apriltag_trigger
               │                                      │
               │◀── /apriltag/target_pose ◀────────────┘
               │
               ├── (根据计算出april tag的偏移点在torso link下的y轴坐标选择两种姿态其一，避免右臂撞到机身)/goal_pose (pre-contact) ──▶ planner ──▶ executor ──▶ 手臂到 pre
               │
               ├── [subprocess] Dex-3 伸中指
               │
               ├── /goal_pose (press) ──▶ planner ──▶ executor ──▶ 手臂前进按压
               │
               ├── /goal_pose (retreat) ──▶ planner ──▶ executor ──▶ 手臂退回
               │
               ├── [subprocess] Dex-3 合拢
               │
               └── /executor/return_to_standing ──▶ executor ──▶ ramp 回站立
```

---

## 4. 坐标系

```
                    torso_link (base_link)
                         │
              ┌──────────┴──────────┐
              │                     │
     right_shoulder_pitch_link    d435_link ──▶ camera_link
              │                                     │
     right_shoulder_roll_link              camera_color_frame
              │                                     │
     right_shoulder_yaw_link           camera_color_optical_frame
              │                         (AprilTag 检测坐标系)
         right_elbow_link
              │
     right_wrist_roll_link
              │
     right_wrist_pitch_link
              │
     right_wrist_yaw_link
              │
       right_tcp_link  ← IK 求解末端 (含 tcp_offset_x = 0.12m)
```

**TF 命名空间**：`/unitree_g1_dex3/tf` 和 `/unitree_g1_dex3/tf_static`（与系统默认 `/tf` 隔离）

**相机 TF 链**：`d435_link` → `camera_link` → `camera_color_frame` → `camera_color_optical_frame`（静态 TF，launch 中发布）

AprilTag 检测在 `camera_color_optical_frame` 系下得到 tag 位姿，通过 TF 变换到 `torso_link` 系后发布。

---

## 5. 硬件接口

### 5.1 DDS Topic（Unitree SDK）

| Topic | 方向 | 说明 |
|---|---|---|
| `/lf/lowstate` | G1 → 容器 | 全身 35-DOF 关节状态 + IMU + 手指状态 |
| `/arm_sdk` | 容器 → G1 | 右臂 7-DOF 关节指令（位置+kp+kd+力矩） |

- 使用 CycloneDDS（`rmw_cyclonedds_cpp`）
- 网卡：`enP8p1s0`
- 容器通过 `--network host --privileged` 直连 DDS 网络

### 5.2 V4L2 相机

| 属性 | 值 |
|---|---|
| 设备 | Intel RealSense D435i（仅使用 RGB） |
| V4L2 节点 | `/dev/video4`（auto 扫描匹配序列号 `253243060636`） |
| 分辨率 | 640×480 |
| 格式 | YUYV |
| USB Interface | `03` |

不使用 RealSense SDK / `realsense2_camera`，直接 V4L2 采集 RGB。

### 5.3 Dex-3 灵巧手

| 属性 | 值 |
|---|---|
| 控制方式 | Unitree SDK2 Python，subprocess 调用 |
| 脚本 | `/workspaces/unitree_dex3_cpp/example/control_dex3_right_setpoint.py` |
| 网卡 | `enP8p1s0` |
| 自由度 | 7-DOF (拇指×2 + 食指×2 + 中指×2 + 全合/全开×1) |

---

## 6. 构建系统

### 6.1 CMake 选项

| 选项 | 默认 | 说明 |
|---|---|---|
| `BUILD_IK_FCL_OMPL_PLANNER` | `OFF` | 编译 C++ 规划器（依赖 TRAC-IK, OMPL, FCL） |
| `Python3_EXECUTABLE` | 系统默认 | 容器内必须指定 `/usr/bin/python3` |

### 6.2 C++ 可执行文件

| 二进制 | 源文件 | 说明 |
|---|---|---|
| `joint_state_publisher` | `joint_state_publisher.cpp` | 关节状态发布 |
| `joint_trajectory_executor` | `joint_trajectory_executor.cpp` | 轨迹执行 + 重力补偿 |
| `ik_fcl_ompl_planner` | `ik_fcl_ompl_planner.cpp` | IK + 碰撞检测 + 运动规划 |
| `dex3_controller` | `dex3_controller.cpp` | Dex-3 ROS 控制器（未在当前 launch 中使用） |
| `right_hand_pressure_monitor` | `right_hand_pressure_monitor.cpp` | 手指压力监控 |
| `visual_detection_tester` | `visual_detection_tester.cpp` | 视觉检测调试工具 |

### 6.3 Python 脚本

| 脚本 | 说明 |
|---|---|
| `v4l2_apriltag_trigger.py` | V4L2 + AprilTag 检测主节点 |
| `apriltag_button_press_node.py` | 按压序列编排 |
| `tcp_torso_pose.py` | TCP 位姿读取工具 |
| `apriltag_reach_uat.py` | 到达验收测试 |
| `adaptive_orientation_ab.py` | 自适应朝向 A/B 测试 |
| `apriltag_goal_bridge.py` | AprilTag→goal_pose 桥接（reach 模式内置到 v4l2_trigger） |
| `gravity_torque_publisher.py` | 重力力矩发布（调试用） |

---

## 7. 配置文件对照

| 配置文件 | 使用场景 | 包含的节点参数 |
|---|---|---|
| `apriltag_button_press.yaml` | button_press launch | v4l2_trigger + button_press_node + planner |
| `v4l2_apriltag_trigger.yaml` | reach / apriltag launch | v4l2_trigger（检测参数不同，如 offset_xyz、trigger_key） |
| `apriltag.yaml` | reach launch | planner + tcp_torso_pose |

**button_press 和 reach 的配置差异**：

| 差异项 | button_press | reach |
|---|---|---|
| `offset_xyz` | `[0.0, 0.0, -0.1]` | `[0.12, -0.01, -0.13]` |
| `trigger_key` | `""` (topic 触发) | `"g"` (键盘触发) |
| `detect_only` | `true` (固定) | `false` (默认) |
| `adaptive_orientation` | `false` | `true` |
| `velocity_scale` | `0.01` | `0.02` |
| `auto_return_to_standing` | `false` | `true` |

---

## 8. Launch 组合关系

```
apriltag_button_press.launch.py
├── robot.launch.py
│   ├── robot_state_publisher (URDF)
│   └── joint_state_publisher (DDS → /joint_states)
├── static TF: d435_link → camera_link → camera_color_frame → camera_color_optical_frame
├── [TimerAction 3s]
│   ├── v4l2_apriltag_trigger (detect_only=true, trigger_key="")
│   ├── planner.launch.py
│   │   └── ik_fcl_ompl_planner
│   ├── control.launch.py
│   │   └── joint_trajectory_executor (auto_return_to_standing=false)
│   └── apriltag_button_press_node

apriltag_reach.launch.py
├── robot.launch.py
├── static TF (同上)
├── [TimerAction 3s]
│   ├── v4l2_apriltag_trigger (detect_only=false, trigger_key="g")
│   ├── planner.launch.py
│   └── control.launch.py

reach.launch.py (手动测试)
├── robot.launch.py
├── planner.launch.py
└── control.launch.py

apriltag.launch.py (相机调试)
├── robot.launch.py
├── static TF
└── v4l2_apriltag_trigger
```

---

## 9. 运行时依赖

| 层级 | 依赖 |
|---|---|
| **系统** | ROS 2 Humble, CycloneDDS, Python 3.10 |
| **C++ 库** | KDL, TRAC-IK, OMPL, FCL, geometric_shapes |
| **Python 库** | pupil-apriltags, scipy, numpy, opencv-python, tf2_ros |
| **Unitree** | unitree_hg (ROS msg), unitree_sdk2_python (Dex-3 控制) |
| **硬件** | G1 机体 (DDS), D435i (V4L2), Dex-3 灵巧手 (DDS) |
