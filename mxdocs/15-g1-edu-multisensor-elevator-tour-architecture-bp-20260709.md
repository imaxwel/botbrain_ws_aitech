# 11 G1 EDU 多传感器迎宾导览、电梯、闸机与 Dex3 按钮操作软件架构最佳实践

生成时间: 2026-07-09  
需求来源: `/Users/fausto/mdev/aitech/4g1edu/2026-6-17  总测试用例 075624.md`  
目标工程: `/Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech`  
复用参考: `/Users/fausto/mdev/aitech/4g1edu/BotBrainmx2`、`BotBrainmx2/botbrain_ws`、`BotBrainmx2/frontend`  
联网检索范围: ROS 2、RealSense D435i、Livox MID360、AprilTag、Nav2、RTAB-Map/FAST-LIO、Jetson/JetPack、Ultralytics TensorRT、Unitree G1 开发文档。

> 说明: 本文是架构和实施步骤建议，基于当前本地源码静态阅读和联网官方资料检索。没有连接真实 G1 EDU、D435i、MID360、电梯、闸机、门禁和 Dex3 真机执行验证。

---

## 1. 一句话结论

这个需求不能做成“YOLO 看到按钮后直接让手指去按”或“AprilTag 消失就让机器人进出电梯”。推荐做成 **传感器驱动层 + 感知 evidence 层 + Mission Supervisor 任务闭环层 + BotBrain 前端观测层**。

核心边界:

```text
传感器驱动层
  RealSense D435i / MID360 / G1 SDK / Dex3 / 气压计 / 音频
  只负责稳定发布 ROS topic、TF、diagnostics、lifecycle health

感知 evidence 层
  AprilTag 位姿、YOLO 检测、深度投影、LiDAR/Costmap、楼层估计、门/按钮/闸机状态
  只负责输出带 timestamp、frame_id、confidence、freshness 的证据

Mission Supervisor / task BT 层
  36 项用例、迎宾导览、电梯、闸机、门禁、低电量、异常恢复的唯一 owner
  用 action + evidence 判断完成/重试/人工接管

BotBrain 前端
  Mission Control、摄像头、地图、YOLO 历史、diagnostics、人工确认和接管
  不直接判断任务完成，也不直接写硬件
```

一句工程建议:

```text
LiDAR/FAST-LIO/Open3D/Nav2 负责“安全走到大概位置”。
D435i + AprilTag 负责“在关键设施前精确定位姿态”。
D435i + YOLO/深度负责“识别电梯门、按钮、灯、半开半闭和近场可操作物体”。
Dex3/arm_controller 负责“停稳后按压/刷卡”，并必须有视觉/深度/灯光/门态 evidence 确认。
Mission Supervisor 负责“什么时候允许按、失败怎么重试、什么时候停住找人”。
```

---

## 2. 联网检索依据

### 2.1 ROS 2 架构基线

| 主题 | 资料 | 对本项目的含义 |
|---|---|---|
| QoS | ROS 2 Humble QoS 文档: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html | 高频图像、点云、LiDAR、实时状态应优先 `best_effort`、小队列；控制命令和任务结果应 `reliable`。 |
| DDS 调优 | ROS 2 DDS tuning: https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html | 机器人网络不稳定时，高频传感器不应全用 Reliable，否则会堆积旧帧。 |
| Lifecycle | ROS 2 managed nodes: https://design.ros2.org/articles/node_lifecycle.html 和 https://docs.ros.org/en/humble/Tutorials/Demos/Managed-Nodes.html | RealSense、YOLO、压缩相机、状态机等长生命周期节点应明确 configure/activate/deactivate，不靠 shell sleep。 |
| TF | REP-105: https://www.ros.org/reps/rep-0105.html | 必须维护 `map -> odom -> base_link/base_footprint` 主链，AprilTag、D435i、Dex3 都应接到同一 TF 树。 |
| Humanoid frames | REP-120: https://www.ros.org/reps/rep-0120.html | G1 是人形机器人，手臂/躯干/头部 frame 命名应尽量贴合 humanoid frame 语义。 |
| rosbag | ROS 2 bag: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html | 电梯/按钮/门态模型必须先用 rosbag 回放验证，再上真机闭环。 |

### 2.2 摄像头、深度、LiDAR、定位资料

| 主题 | 资料 | 对本项目的含义 |
|---|---|---|
| D435i | Intel RealSense D435i: https://realsenseai.com/products/depth-camera-d435i/ | D435i 是带 IMU 的深度相机，适合近场按钮/门/AprilTag，但深度理想工作距离主要是近场，不应替代 MID360 做全局导航。 |
| RealSense ROS2 | RealSense ROS wrapper: https://github.com/realsenseai/realsense-ros 和 https://dev.realsenseai.com/docs/ros2-wrapper/ | `align_depth.enable`、`pointcloud.enable`、filters、camera_name、serial、IMU 参数要配置化；深度对齐彩色图是按钮 3D 反投影的基础。 |
| depth -> laser | depthimage_to_laserscan: https://github.com/ros-perception/depthimage_to_laserscan | D435i 深度可转换为近场 LaserScan，适合补盲和电梯入口安全，不应作为主导航雷达。 |
| pointcloud -> laser | pointcloud_to_laserscan: https://github.com/ros-perception/pointcloud_to_laserscan | MID360 点云投影成 LaserScan 可以复用 2D Nav2/costmap 生态。 |
| Livox ROS Driver 2 | https://github.com/Livox-SDK/livox_ros_driver2 | MID360 应由官方 ROS2 驱动或项目内固定版本驱动发布 PointCloud2，并纳入 lifecycle/diagnostics。 |
| FAST-LIO | https://github.com/hku-mars/FAST_LIO | LiDAR-IMU odometry 适合作为 G1 移动定位主线；当前 `botbrain_ws_aitech` 已引入 FAST-LIO/Open3D 方案。 |
| RTAB-Map | https://github.com/introlab/rtabmap_ros 和 https://introlab.github.io/rtabmap/ | 旧 BotBrainmx2 的 RTAB-Map 链路可作为历史参考；新工程应优先沿用 FAST-LIO/Open3D + Nav2 的现状。 |

### 2.3 AprilTag、Nav2、YOLO、Jetson、Unitree

| 主题 | 资料 | 对本项目的含义 |
|---|---|---|
| AprilTag ROS2 | https://github.com/christianrauch/apriltag_ros | AprilTag 节点发布 tag id、pose 和元数据，适合电梯厅、电梯内、门口、展点精定位。 |
| Nav2 BT/recovery | https://docs.nav2.org/behavior_trees/index.html 和 https://docs.nav2.org/behavior_trees/trees/nav_to_pose_recovery.html | Nav2 本身就是 BT + recovery 思路；本项目电梯/闸机/门禁也应按 action class 做有界 retry。 |
| Nav2 BT Navigator | https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html | 可按任务选择/扩展 BT，但业务层仍应由 Mission Supervisor 管控，不把所有设施交互塞进 Nav2。 |
| Jetson Orin NX | https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/ 和 https://docs.nvidia.com/jetson/jetpack/index.html | Jetson Orin NX 可跑多路 AI，但算力/功耗仍有限；应按任务阶段启停模型，避免所有模型常开。 |
| YOLO TensorRT | https://docs.ultralytics.com/integrations/tensorrt 和 https://docs.ultralytics.com/modes/export | 当前 `bot_yolo` 的 TensorRT engine 路线正确；需要加任务专用模型、结构化输出和深度融合。 |
| Unitree G1 | https://support.unitree.com/home/en/G1_developer 和 https://www.unitree.com/g1 | G1/Dex3/Unitree SDK 是底层执行入口；ROS 层应封装成安全 action，不让业务逻辑直接写 DDS。 |

---

## 3. 当前本地代码事实

### 3.1 `botbrain_ws_aitech` 已有能力

| 能力 | 当前入口 | 现状 |
|---|---|---|
| Docker 服务编排 | `botbrain_ws_aitech/docker-compose.yaml` | 已有 `fast_lio`、`localization`、`navigation`、`manipulation`、`yolo`、`foxglove`、`jetson_stats`、`web_server_prod` 等服务。 |
| 3D 定位 + 2D 导航 | `botbrain_ws/src/g1_pkg/launch/fast_lio.launch.py`、`localization_3d.launch.py`、`bot_navigation/navigation.launch.py` | 新工程已是 FAST-LIO/Open3D/Nav2 方向，优先沿用。 |
| D435i | `botbrain_ws/src/bot_localization/bot_localization/launch/realsense.launch.py` | 已按 `camera_config.yaml` 启前置 D435i、静态 TF、depthimage_to_laserscan、压缩图像节点。 |
| D435i 配置 | `botbrain_ws/src/g1_pkg/config/camera_config.yaml` | 前置 `d435i`、serial `419522072874`、parent `torso_link`、child `front_camera_link`、pitch 约 0.83 rad。 |
| MID360 | `botbrain_ws/src/g1_pkg/launch/livox_MID360.launch.py`、`MID360_config.json` | 已接 Livox driver，发布 `pointcloud`，配置 host/lidar IP 和 UDP 端口。 |
| 点云转 LaserScan | `botbrain_ws/src/g1_pkg/launch/pc2ls.launch.py`、`pointcloud_to_laserscan_params.yaml` | 已将 `pointcloud` 转为 `scan`，供 2D 导航/避障复用。 |
| AprilTag | `botbrain_ws/src/bot_navigation/launch/apriltag_detection.launch.py`、`APRILTAG_SETUP.md` | 已有 `apriltag_ros` 启动文件，当前 remap 到绝对 `/front_camera/...`。 |
| AprilTag 精定位 | `botbrain_ws/src/bot_navigation/scripts/apriltag_nav_correction.py` | 已有 Nav2 到点后视觉伺服校正原型，基于 `/apriltag/detections`。 |
| YOLO | `botbrain_ws/src/bot_yolo/bot_yolo/yolo_node.py` | 已是 ROS2 LifecycleNode，支持 TensorRT engine、tracking、压缩图像和 JSON 检测发布。 |
| Dex3/手臂 | `botbrain_ws/src/g1_manipulation_pkg` | 已有 `arm_controller`、`dx3_hand_controller`、interactive marker；手臂启用会通过 `manipulation_vel` 停住底盘。 |
| 前端 | `frontend/src/components/*`、`frontend/src/hooks/ros/*` | 已有 ROS 摄像头、地图、LaserScan、Nav2、diagnostics、YOLO 历史/Supabase、Mission Control 类结构。 |

### 3.2 `BotBrainmx2` 可复用能力

| 能力 | 当前入口 | 复用建议 |
|---|---|---|
| Mission Control 类型 | `BotBrainmx2/frontend/src/types/mission-control.ts` | 已有 `active_action`、`retry_summary`、`active_exception`、`pending_decisions`，适合承载本需求的 evidence 投影。 |
| Supervisor 前端上下文 | `BotBrainmx2/frontend/src/contexts/MissionSupervisorContext.tsx` | 已通过 `/snapshot`、`/preflight`、`/events`、`/stream` 等接口拉取任务状态，可扩展 sensor evidence。 |
| Mission Supervisor API 代理 | `BotBrainmx2/frontend/src/services/mission-supervisor.ts` | 建议沿用 `/api/mission-supervisor/*` 代理方式，不让浏览器直连机器人 ROS。 |
| 36 项任务和重试思想 | `BotBrainmx2/docss/110-g55-bt-retry-policy-turnstile-elevator-door.md` | 对电梯、闸机、门禁已有 action class/retry/failure_class 思路，应作为本方案的任务控制基线。 |
| 楼层气压证据 | `BotBrainmx2/docss/129-floor_estimate-mission-control-36-feedback-real-g1-bp-v3.md` | 楼层判断应作为 evidence 子系统，而不是前端或气压计直接完成 TC11/19/27。 |

### 3.3 目前需要先修的明显问题

这些不是大架构，而是上真机前应先修的工程坑:

| 问题 | 当前位置 | 风险 | 建议 |
|---|---|---|---|
| `realsense.launch.py` 即使 back camera 为空也创建 `depthimage_to_laserscan_back` | `bot_localization/.../realsense.launch.py` | front-only G1 会多一个订阅不存在 topic 的节点，diagnostics 混乱。 | 只在 `back_camera` 非空时创建 back scan node。 |
| `compressed_realsense.py` 订阅绝对 `/back_camera/color/image_raw` | `bot_localization/.../compressed_realsense.py` | namespace 下失效，且 back camera 当前为空。 | 订阅相对 `back_camera/color/image_raw`，并按配置开关创建。 |
| D435i IMU 当前被禁用 | `make_camera_params()` 中 `enable_gyro/accel/motion=False` | 用户需求提到 IMU；D435i IMU 对近场视觉时序/诊断有价值。 | 如果要用 D435i IMU，改为配置化启用，不要硬编码关闭。G1 主姿态仍以机身 IMU/LiDAR-IMU 为准。 |
| AprilTag launch 使用绝对 `/front_camera/...` | `bot_navigation/launch/apriltag_detection.launch.py` | 在 `robot_name=g1_robot` 时可能订阅不到 `/g1_robot/front_camera/...`。 | 用 namespace/LaunchConfiguration，默认 `/{robot_name}/front_camera/...` 或相对 topic。 |
| YOLO 输出只有 object/id/conf，不含 bbox | `bot_yolo/yolo_node.py` | 无法做按钮 3D 反投影、门态 ROI、前端 overlay、evidence 审计。 | 发布 `bbox_xyxy`、`header`、`frame_id`、`model_id`、`roi`、`latency_ms`；最好新增自定义 msg 或 `vision_msgs/Detection2DArray`。 |
| YOLO 输入 topic 相对 `front_camera/color/image_raw` | `bot_yolo/config/yolo.yaml` | 如果节点 namespace 配置不一致，会订阅错。 | 统一为相对 topic + namespace，或明确 `/g1_robot/front_camera/color/image_raw`。 |
| 前端 `G1-R1` profile 太通用 | `frontend/src/config/robot-profiles/profiles/g1-r1.ts` | 缺少 D435i 深度、AprilTag、YOLO、elevator evidence、Dex3 topic。 | 增加 `G1-EDU-DC-Tour` profile 或扩展现有 G1 profile。 |

---

## 4. 推荐总体架构

### 4.1 进程和容器视图

```text
Unitree G1 EDU / Jetson Orin NX
│
├─ base / bringup
│    ├─ robot_read_node / robot_write_node
│    ├─ twist_mux / emergency stop / diagnostics
│    └─ G1 SDK DDS bridge
│
├─ fast_lio
│    ├─ Livox MID360 driver
│    ├─ IMU correction
│    └─ LiDAR-IMU odometry / registered pointcloud
│
├─ localization
│    ├─ Open3D ICP localization
│    ├─ map -> odom TF
│    ├─ RealSense D435i driver
│    ├─ compressed camera
│    └─ depthimage_to_laserscan near-field scan
│
├─ navigation
│    ├─ Nav2 map_server / planner / controller / BT navigator
│    ├─ waypoint navigator
│    └─ AprilTag correction action client
│
├─ perception
│    ├─ apriltag_ros
│    ├─ yolo_node: general/person/elevator/button models
│    ├─ elevator_state_estimator
│    ├─ button_panel_estimator
│    ├─ turnstile_state_estimator
│    └─ evidence publisher
│
├─ manipulation
│    ├─ arm_controller
│    ├─ dx3_hand_controller
│    └─ press_button / card_tap action wrapper
│
├─ floor_estimate sidecar
│    ├─ barometer mobile + base
│    └─ floor evidence
│
└─ botbrain_ws_gateway
     ├─ HTTP/action boundary for Mission Supervisor
     ├─ health/preflight/evidence aggregation
     └─ no mission state ownership
```

```text
Operator / DellNB / Browser
│
├─ BotBrain frontend
│    ├─ Mission Control
│    ├─ live camera and map observe
│    ├─ perception evidence panel
│    ├─ pending decision / retry / takeover
│    └─ playback / rosbag/HIL test controls
│
└─ Mission Supervisor
     ├─ 36 test case state owner
     ├─ task BT / retry / risk guard
     ├─ instruction/auth/voice context
     └─ event/audit/snapshot stream
```

### 4.2 数据流

```text
MID360 PointCloud2 + G1/FAST-LIO IMU
  -> FAST-LIO / Open3D localization
  -> map->odom TF, /localization_3d, /cloud_registered_body_1
  -> Nav2 costmap and Mission evidence

D435i color/depth/camera_info
  -> compressed camera for front-end
  -> AprilTag detection for local pose anchor
  -> YOLO elevator/button/person/door detection
  -> depth projection: bbox center/ROI -> 3D point/plane
  -> elevator/button/turnstile state estimators
  -> Mission evidence

Mission Supervisor
  -> task action: navigate_to_waypoint / align_to_tag / call_elevator / press_floor / wait_floor / exit_elevator
  -> botbrain_ws_gateway
  -> ROS action/service/topic
  -> action result + evidence
  -> update snapshot / events / pending_decision

BotBrain frontend
  -> reads Mission Supervisor snapshot/events/evidence
  -> reads ROS camera/map for observe only
  -> operator decision/control goes back to Mission Supervisor
```

---

## 5. 传感器职责分工

### 5.1 MID360 LiDAR

主责:

- 大厅、走廊、电梯厅、楼层参观路径的全局定位和避障。
- 3D 点云地图、FAST-LIO 里程计、Open3D ICP 重定位。
- Nav2 costmap 障碍观测源，尤其是人流、墙体、立柱、门口障碍。

不建议主责:

- 按电梯按钮的毫米级定位。
- 判断按钮灯亮。
- 单独判断电梯门半开半闭。

工程要求:

- `frame_id` 固定为 `mid360_link` 或带 namespace 的等效 frame。
- 点云发布频率 10-20Hz 起步，先不要盲目拉到 50Hz。
- LiDAR 驱动、FAST-LIO、Open3D ICP 和 Nav2 costmap 的 TF 时间必须一致。
- 对前端和 rosbag 记录暴露 `/pointcloud`、`/scan`、`/cloud_registered_body_1`、`/localization_3d_confidence`、`/tf`。

### 5.2 RealSense D435i

主责:

- AprilTag 检测和 tag pose。
- 电梯门、门缝、按钮面板、按钮灯、楼层显示、闸机刷卡区、自动门状态的近场视觉。
- 利用 aligned depth 把 YOLO bbox/关键点反投影到 3D。
- 近场 LaserScan 补盲，尤其是电梯入口、门前、按钮操作区。

不建议主责:

- 大范围走廊全局定位。
- 在电梯内单独判断楼层。
- 在强反光/玻璃/金属门上只依赖深度值做安全判断。

工程要求:

- 彩色图、深度图、camera_info 必须同步或近似同步。
- 按钮 3D 反投影必须使用 `aligned_depth_to_color` 或明确的 depth/color 外参。
- D435i 相机外参需要做现场标定，不只依赖当前 YAML 的估计值。
- 对电梯按钮和门态，推荐使用 ROI 裁剪模型，减少 Jetson GPU 负载。

### 5.3 AprilTag

主责:

- 在电梯厅/电梯内/门禁点/展板点给机器人提供设施相对位姿。
- 给 Nav2 到点后的精调提供目标坐标。
- 给门状态推断提供证据之一。

关键原则:

- **AprilTag 可见/不可见不能单独等价于门关/门开。**
- Tag 检测必须多帧 debounce，例如 10-20 帧窗口。
- Tag pose 必须带 `frame_id`、`stamp`、`tag_size_version`、`tag_map_version`。
- 电梯内外 tag 应分组管理，不要用一个 `target_tag_id=0` 贯穿全场。

建议 tag 编号:

| 场景 | tag id 段 | 用途 |
|---|---:|---|
| UG 大厅/服务前台/展板 | 100-199 | 位置锚点和导览触发校验 |
| UG 闸机/刷卡区 | 200-219 | 闸机入口、刷卡面板、出口锚点 |
| UG 电梯外部 | 300-329 | 外门、外呼按钮、等待点 |
| 电梯轿厢内部 | 400-429 | 内门、按钮面板、操作位 |
| 11/14/15 楼电梯厅 | 500-599 | 出梯定位、楼层锚点 |
| 11/14/15 楼门禁/参观点 | 600-699 | 门禁、展点、讲解点 |

### 5.4 YOLO / 视觉模型

主责:

- `person/visitor/staff` 跟随和安全区域监测。
- `elevator_door`、`door_gap`、`door_leaf_left/right`、`elevator_panel`、`button`、`button_light`、`floor_digit`、`turnstile_gate`、`card_reader`。
- 给 task evidence 提供 bbox、confidence、tracking id、ROI 状态。

不建议主责:

- 直接输出“可以进电梯/可以按按钮”这种任务决策。
- 单模型常驻检测所有类别。

推荐模型拆分:

| 模型 | 启用阶段 | 类别 | 目标 |
|---|---|---|---|
| `yolo_general_nav` | 大厅/走廊导航 | person、obstacle、staff | 跟随、安全、人流 |
| `yolo_elevator_outer` | 到达电梯外 | elevator_door、door_gap、call_button、panel、indicator | 呼梯、门态 |
| `yolo_elevator_inner` | 进入电梯后 | floor_button、close_button、open_button、button_light、floor_digit、door_gap | 按楼层、确认灯、出梯 |
| `yolo_turnstile` | 闸机前后 | gate_leaf、card_reader、lane_open、person_in_lane | 刷卡/通行 |
| `yolo_access_door` | 11/14/15 楼门禁 | card_reader、door_gap、door_handle、door_open | 门禁开门确认 |

### 5.5 Dex3 / 手臂

主责:

- 停稳后刷卡/靠卡。
- 停稳后按电梯外呼按钮。
- 电梯内按目标楼层按钮。
- 必要时按关门按钮，但应优先遵守现场安全和工作人员指令。

硬约束:

- 手臂动作期间必须由 `manipulation_vel` 或更高优先级输入让底盘停止。
- 操作前必须确认机器人姿态稳定、足底状态安全、目标按钮 3D pose 新鲜。
- 操作后必须有独立确认: 按钮灯亮、状态变化、门状态变化、或人工确认。
- 不允许 `dx3_hand_controller` 直接作为业务 action；需要 `press_button_action` 包装，包含 precondition、motion、contact/position result、visual confirmation、timeout。

---

## 6. 推荐 ROS topic、service、action 和 evidence 合同

### 6.1 命名空间原则

统一使用 `robot_name=g1_robot` 后，topic 建议如下:

```text
/g1_robot/front_camera/color/image_raw
/g1_robot/front_camera/color/camera_info
/g1_robot/front_camera/depth/image_rect_raw
/g1_robot/front_camera/aligned_depth_to_color/image_raw
/g1_robot/front_camera/imu
/g1_robot/compressed_camera

/g1_robot/pointcloud
/g1_robot/scan
/g1_robot/front_camera/scan

/g1_robot/perception/apriltag/detections
/g1_robot/perception/yolo/detections
/g1_robot/perception/yolo/image_compressed
/g1_robot/perception/elevator/door_state
/g1_robot/perception/elevator/button_panel
/g1_robot/perception/turnstile/state
/g1_robot/perception/access_door/state

/g1_robot/floor_estimate
/g1_robot/diagnostic_stats
/g1_robot/mission/evidence
```

原则:

- 驱动原始 topic 保留设备语义。
- 感知结果集中到 `/perception/...`。
- 任务 evidence 集中到 `/mission/evidence` 或 Gateway HTTP。
- 前端和 Supervisor 不依赖模型内部 topic 名称，依赖 stable contract。

### 6.2 YOLO 结构化输出建议

当前 `bot_yolo` 输出:

```json
{
  "detections_num": "2",
  "detected_objects": [
    {"object_id":"0","object":"person","confidence":"0.932","track_id":12}
  ]
}
```

建议升级为:

```json
{
  "schema_version": "g1.perception.detections.v1",
  "stamp": "2026-07-09T10:22:33.123Z",
  "frame_id": "g1_robot/front_camera_color_optical_frame",
  "source_topic": "/g1_robot/front_camera/color/image_raw",
  "model_id": "elevator_inner_buttons_yolo11s_trt_20260709",
  "latency_ms": 31.4,
  "image_size": {"width": 640, "height": 480},
  "detections": [
    {
      "class_id": 11,
      "label": "floor_button_14",
      "confidence": 0.91,
      "track_id": 32,
      "bbox_xyxy": [318, 202, 356, 240],
      "center_px": [337, 221],
      "depth_m": 0.63,
      "point_camera_m": [0.04, -0.02, 0.63],
      "point_target_frame": "g1_robot/pelvis",
      "point_target_m": [0.42, -0.31, 0.88],
      "fresh": true
    }
  ]
}
```

如果后续引入 `vision_msgs/Detection2DArray` 更好，但 JSON 也可以先作为过渡，只要字段完整且版本化。

### 6.3 电梯门状态 evidence

电梯门状态不要由单个模型决定。建议 `elevator_state_estimator` 聚合:

- AprilTag 外门/内门可见性和 pose。
- YOLO `door_leaf`、`door_gap`、`elevator_door` 检测。
- D435i depth ROI: 门缝/入口自由空间。
- LiDAR/costmap: 入口是否被人或物体占用。
- 时间窗口: 例如最近 1.0-2.0 秒内连续帧。

输出:

```json
{
  "schema_version": "g1.elevator.door_state.v1",
  "location_id": "ug_elevator_a_outer",
  "stamp": "2026-07-09T10:23:01.122Z",
  "frame_id": "g1_robot/map",
  "state": "OPEN",
  "state_candidates": {
    "CLOSED": 0.03,
    "OPENING": 0.12,
    "OPEN": 0.81,
    "HALF_OPEN": 0.04,
    "UNKNOWN": 0.00
  },
  "confidence": 0.81,
  "stable_for_s": 1.2,
  "fresh": true,
  "evidence": {
    "apriltag_visible": false,
    "door_gap_px": 86,
    "depth_free_space_m": 1.4,
    "entry_costmap_clear": true,
    "sample_count": 18
  },
  "safe_to_enter": true,
  "safe_to_exit": false
}
```

状态集合建议固定:

```text
CLOSED
OPENING
OPEN
CLOSING
HALF_OPEN
BLOCKED
UNKNOWN
```

### 6.4 按钮面板 evidence

按钮面板应该输出面板坐标系和目标按钮 3D pose:

```json
{
  "schema_version": "g1.elevator.button_panel.v1",
  "panel_id": "elevator_a_inner_panel",
  "stamp": "2026-07-09T10:24:10.501Z",
  "frame_id": "g1_robot/front_camera_color_optical_frame",
  "panel_pose_frame": "g1_robot/pelvis",
  "panel_pose": {
    "position": [0.46, -0.34, 0.91],
    "orientation_xyzw": [0.0, 0.0, 0.707, 0.707]
  },
  "buttons": [
    {
      "label": "14",
      "bbox_xyxy": [318, 202, 356, 240],
      "press_point_frame": "g1_robot/pelvis",
      "press_point_m": [0.48, -0.36, 0.96],
      "normal_frame": "g1_robot/pelvis",
      "normal": [-0.98, 0.03, 0.02],
      "lit": false,
      "confidence": 0.89,
      "fresh": true
    }
  ]
}
```

### 6.5 Manipulation action 合同

建议新增 ROS action 或 Gateway task action:

```text
action_class: elevator_press_button
input:
  target_label: "14"
  panel_id: "elevator_a_inner_panel"
  expected_confirmation: "button_light_on"
  max_press_attempts: 2
  timeout_ms: 15000

feedback:
  phase: ALIGNING | REACHING | PRESSING | RETRACTING | VERIFYING
  current_attempt: 1
  evidence: {...}

result:
  status: SUCCEEDED | FAILED | TIMEOUT | CANCELLED
  error_code: BUTTON_NOT_FOUND | DEPTH_STALE | IK_FAILED | PRESS_TIMEOUT | LIGHT_NOT_CONFIRMED | SAFETY_STOP
  evidence: {...}
```

按压成功标准:

1. 目标按钮检测新鲜，`fresh=true`。
2. 3D press point 在手臂 workspace 内。
3. 底盘停止并姿态稳定。
4. 机械臂到位、执行按压、撤回。
5. 视觉确认按钮灯或面板状态变化。
6. Mission Supervisor 收到 action result 后才更新 TC10/18/26 的相关步骤。

---

## 7. 36 项需求到多传感器模块映射

| 阶段 | 用例 | 主传感器/模块 | evidence | 任务判断 owner |
|---|---:|---|---|---|
| 待机闲聊 | TC1 | 音频/LLM/voice context | voice event、language、latency | Mission Supervisor |
| 低电量前台换电 | TC2/35 | G1 battery、Nav2 | battery state、nav result | Mission Supervisor |
| 接收任务和授权 | TC3 | 语音、auth code | staff instruction、8 位密码 | Mission Supervisor |
| 大厅展板导览 | TC4-7 | Nav2、LiDAR、AprilTag/map waypoint | waypoint reached、tag optional、speech event | Mission Supervisor |
| 闸机 | TC8 | LiDAR、D435i、AprilTag、YOLO、Dex3/card | card_reader pose、gate open、lane clear | Mission Supervisor |
| 电梯外呼 | TC9/17/25 | Nav2、AprilTag、YOLO、D435i depth、Dex3 | call button pose、press result、button light/door state | Mission Supervisor |
| 进电梯/按楼层 | TC10/18/26 | door_state、person/entry clear、button_panel、Dex3 | safe_to_enter、target button lit | Mission Supervisor |
| 电梯上升楼层 | TC11/19/27 | floor_estimate、barometer、可选 OCR/楼层屏 | floor_label、confidence、stable window | Mission Supervisor |
| 电梯内对话 | TC12/20/28 | 音频/LLM | same-language response、forbidden action rejection | Mission Supervisor |
| 出电梯 | TC13/21/29 | floor evidence、door_state、LiDAR/costmap、staff instruction | target floor confirmed、safe_to_exit | Mission Supervisor |
| 楼层导览 | TC14/22/30 | Nav2、LiDAR、AprilTag/map waypoint、person tracking | route progress、guest follow evidence | Mission Supervisor |
| 门禁/自动门 | TC15/23/31 | D435i、YOLO、card_reader、door_state、Dex3/card | unlock/open evidence | Mission Supervisor |
| 下一楼层/返回 | TC16/24/32/33/34 | Mission route、Nav2、电梯流程复用 | route state、arrival evidence | Mission Supervisor |
| 复位 | TC36 | Supervisor state | IDLE snapshot | Mission Supervisor |

---

## 8. 详细实施步骤

### Step 0: 固化 mission profile 和 topic manifest

新增或扩展:

```text
frontend/src/config/robot-profiles/profiles/g1-edu-dc-tour.ts
botbrain_ws/src/g1_pkg/config/sensor_manifest.yaml
botbrain_ws/src/g1_pkg/config/facility_map.yaml
botbrain_ws/src/g1_pkg/config/apriltag_map.yaml
botbrain_ws/src/g1_pkg/config/perception_pipeline.yaml
```

`sensor_manifest.yaml` 示例:

```yaml
robot_name: g1_robot
sensors:
  d435i_front:
    type: realsense_d435i
    serial: "419522072874"
    namespace: front_camera
    parent_frame: torso_link
    child_frame: front_camera_link
    enable_color: true
    enable_depth: true
    enable_aligned_depth: true
    enable_imu: false   # 先保持 false；若要用 D435i IMU，单独验证后打开
    compressed_topic: compressed_camera
  mid360:
    type: livox_mid360
    frame_id: mid360_link
    pointcloud_topic: pointcloud
    scan_topic: scan
perception:
  apriltag:
    detections_topic: perception/apriltag/detections
  yolo:
    detections_topic: perception/yolo/detections
  elevator:
    door_state_topic: perception/elevator/door_state
    button_panel_topic: perception/elevator/button_panel
```

验收:

```bash
ros2 topic list | sort
ros2 run tf2_tools view_frames
ros2 topic hz /g1_robot/front_camera/color/image_raw
ros2 topic hz /g1_robot/pointcloud
ros2 topic hz /g1_robot/scan
```

### Step 1: 修正 D435i bringup

修改目标:

```text
botbrain_ws/src/bot_localization/bot_localization/launch/realsense.launch.py
botbrain_ws/src/bot_localization/bot_localization/scripts/compressed_realsense.py
botbrain_ws/src/g1_pkg/config/camera_config.yaml
```

操作:

1. `depthimage_to_laserscan_back` 只在 `back_camera` 非空时创建。
2. `compressed_realsense.py` 根据参数 `enable_front`、`enable_back` 创建 publisher/subscriber。
3. 后摄订阅使用相对 topic，不用绝对 `/back_camera/...`。
4. 压缩图像发布保留 `header.stamp` 和 `header.frame_id`。
5. 对前端观测压缩流可用 `best_effort` + depth 1；对任务 evidence 不用压缩图像。
6. 如启用 D435i IMU，单独发布 `/g1_robot/front_camera/imu`，不要直接混入 G1 主 odom，先只做 diagnostics 和时间同步验证。

验收:

```bash
ros2 lifecycle get /g1_robot/front_camera
ros2 lifecycle get /g1_robot/realsense_compressed_node
ros2 topic echo --once /g1_robot/front_camera/color/camera_info
ros2 topic echo --once /g1_robot/compressed_camera
ros2 topic hz /g1_robot/front_camera/depth/image_rect_raw
```

### Step 2: 标定 TF 和相机内外参

必须产出:

```text
calibration/g1_edu_dc_tour/front_d435i_intrinsics_YYYYMMDD.yaml
calibration/g1_edu_dc_tour/front_d435i_to_torso_tf_YYYYMMDD.yaml
calibration/g1_edu_dc_tour/mid360_to_base_tf_YYYYMMDD.yaml
calibration/g1_edu_dc_tour/tag_size_measurement_YYYYMMDD.md
```

步骤:

1. 用 RealSense 工具确认 serial、固件、彩色/深度帧率。
2. 现场测量 D435i 到 `torso_link` 或稳定躯干 frame 的外参。
3. 对 AprilTag 标定板测量 tag 实际边长，写入 `apriltag_map.yaml`。
4. 用同一组 tag 在不同距离/角度采样，统计 pose 抖动。
5. 将 TF 固化到 launch 参数，不要在多个 launch 重复发布同一 child frame。

验收阈值建议:

```text
静止 tag 1m 内，translation std < 2cm
静止 tag yaw std < 2deg
camera_info stamp 与 image stamp 差值 < 50ms
TF lookup camera -> pelvis 成功率 > 99%
```

### Step 3: AprilTag 地图和关键设施锚点

新增:

```text
botbrain_ws/src/g1_pkg/config/apriltag_map.yaml
botbrain_ws/src/bot_navigation/launch/apriltag_detection.launch.py
botbrain_ws/src/bot_navigation/scripts/tag_pose_evidence_node.py
```

`apriltag_map.yaml` 示例:

```yaml
tag_family: "36h11"
default_size_m: 0.162
tags:
  - id: 300
    name: "ug_elevator_a_outer_door_left"
    size_m: 0.162
    facility: "elevator_a"
    role: "outer_door_anchor"
    floor: "UG"
  - id: 401
    name: "elevator_a_inner_panel_anchor"
    size_m: 0.080
    facility: "elevator_a"
    role: "inner_button_panel_anchor"
  - id: 501
    name: "floor_11_elevator_hall_anchor"
    size_m: 0.162
    floor: "11"
    role: "hall_anchor"
```

操作:

1. 将 launch remap 改为 namespace-aware。
2. `tag_pose_evidence_node` 订阅 `/g1_robot/perception/apriltag/detections`。
3. 只发布最近窗口稳定后的 tag evidence。
4. tag 丢失时发布 `visible=false`，保留 `last_seen_age_ms`，不要直接删除状态。

验收:

```bash
ros2 topic echo /g1_robot/perception/apriltag/detections
ros2 topic echo /g1_robot/mission/evidence --filter "m.kind == 'apriltag_pose'"
```

### Step 4: YOLO 模块改造成任务感知服务

保留:

- LifecycleNode。
- TensorRT engine。
- 单帧队列和独立推理线程。
- `yolo/image_compressed` 观测流。

必须改:

1. 输出加 bbox、header、latency、model_id。
2. 支持不同模型和不同输入 ROI。
3. 支持按 mission phase 启停或切换模型。
4. 支持 `classes_allowlist`，避免大模型全场景常开。
5. 对按钮/灯状态需要输出小目标 bbox，必要时提高输入分辨率或 ROI 二阶段检测。

建议新增配置:

```yaml
yolo_node:
  ros__parameters:
    camera_topic: "front_camera/color/image_raw"
    camera_info_topic: "front_camera/color/camera_info"
    depth_topic: "front_camera/aligned_depth_to_color/image_raw"
    output_topic: "perception/yolo/detections"
    model_profiles:
      general_nav:
        engine_path: "/root/.cache/ultralytics/general_nav.engine"
        imgsz: 640
        conf: 0.35
        classes: ["person", "staff", "obstacle"]
      elevator_inner:
        engine_path: "/root/.cache/ultralytics/elevator_inner_buttons.engine"
        imgsz: 960
        conf: 0.25
        classes: ["floor_button_11", "floor_button_14", "floor_button_15", "button_light", "door_gap"]
    active_profile_topic: "perception/yolo/active_profile"
```

验收:

```bash
ros2 lifecycle set /g1_robot/yolo_node configure
ros2 lifecycle set /g1_robot/yolo_node activate
ros2 topic echo /g1_robot/perception/yolo/detections
ros2 topic hz /g1_robot/perception/yolo/detections
```

### Step 5: 训练和部署电梯/闸机模型

数据采集:

1. 用 D435i 在真实大厅、闸机、电梯外、电梯内、11/14/15 楼门禁采集 rosbag。
2. 每个点采集白天/夜间/强反光/人遮挡/门开/门关/半开半闭。
3. 采集 `color image + depth + camera_info + /tf + /apriltag/detections + /scan`。
4. 标注 bbox: 门、门缝、按钮、按钮灯、楼层数字、读卡器、闸机叶片、人员。

训练:

1. 从通用 YOLO11n/s 起步，按钮小目标建议至少 YOLO11s 或 ROI 二阶段。
2. 对电梯内按钮，优先用 panel ROI 后二次检测，而不是整图找小按钮。
3. 训练集按设施分层，不能只用一个电梯的样本。
4. 验证集必须包含半开半闭和遮挡样本。

部署:

```bash
python3 - <<'PY'
from ultralytics import YOLO
model = YOLO("elevator_inner_buttons.pt")
model.export(format="engine", device=0, half=True, imgsz=960, dynamic=False, workspace=2048)
PY
```

验收指标建议:

| 模型 | 指标 |
|---|---|
| 电梯门 open/closed/half/unknown | 真实场景 F1 > 0.90，half/unknown 召回优先于精度 |
| 按钮检测 | 目标楼层按钮召回 > 0.95，误按风险类别 precision > 0.98 |
| 按钮灯 | 灯亮确认 F1 > 0.90，不确定时必须输出 unknown |
| 闸机 lane_open | 安全通过状态召回 > 0.95，blocked/unknown 宁可保守 |

### Step 6: 电梯门状态估计器

新增:

```text
botbrain_ws/src/bot_perception/elevator_state_estimator.py
```

输入:

```text
/g1_robot/perception/apriltag/detections
/g1_robot/perception/yolo/detections
/g1_robot/front_camera/aligned_depth_to_color/image_raw
/g1_robot/front_camera/color/camera_info
/g1_robot/scan
/tf
```

输出:

```text
/g1_robot/perception/elevator/door_state
/g1_robot/mission/evidence
```

逻辑:

1. 根据 mission phase 选择 `outer` 或 `inner` 电梯门 profile。
2. 从 tag_map 确定门区域 ROI。
3. 用 AprilTag 可见性判断门面/相机相对位姿是否可信。
4. 用 YOLO 判断门缝和门叶位置。
5. 用 depth ROI 判断入口是否有自由空间。
6. 用 LiDAR/costmap 判断入口是否被人/物体挡住。
7. 计算状态概率和 `safe_to_enter/safe_to_exit`。
8. 多帧稳定后才发布可执行状态。

保守策略:

```text
UNKNOWN -> 不移动
HALF_OPEN -> 不进入/不出梯
OPEN but entry_costmap_clear=false -> 等待
OPEN and wrong_floor -> 不出梯
CLOSED and waiting_call -> 允许重新呼梯或等待
```

### Step 7: 按钮面板 3D 定位与 Dex3 按压

新增:

```text
botbrain_ws/src/bot_perception/button_panel_estimator.py
botbrain_ws/src/g1_manipulation_pkg/g1_manipulation_pkg/actions/press_button_action.py
```

步骤:

1. 到达电梯内操作位前，用 Nav2 到粗 waypoint。
2. 用 AprilTag 内部 panel anchor 精调姿态。
3. YOLO 检测按钮 bbox。
4. 读取 aligned depth，在按钮中心和周边 ROI 拟合平面。
5. 将 press point 从 camera optical frame 转到 `pelvis` 或 arm IK frame。
6. 检查 press point 是否在右手/左手 workspace。
7. 发布 `manipulation/enabled=true`，底盘进入 Mode-B 停止。
8. arm_controller 接收 `manipulation/hand_goal/right` 或 action wrapper 生成轨迹。
9. Dex3 手指保持合适姿态，末端按压到位后撤回。
10. YOLO/视觉确认按钮灯亮。
11. action result 返回 Supervisor。

失败处理:

| 错误 | 自动恢复 |
|---|---|
| `BUTTON_NOT_FOUND` | 微转头/微调身体，重新采样，最多 2 次 |
| `DEPTH_STALE` | 等待新深度帧，重启 RealSense lifecycle 只允许人工确认 |
| `PANEL_POSE_UNSTABLE` | 重新对准 AprilTag |
| `IK_FAILED` | 换手或调整站位，不直接乱按 |
| `LIGHT_NOT_CONFIRMED` | 可重按 1 次；仍失败进入人工 |
| `SAFETY_STOP` | 立即停止，禁止自动重试 |

### Step 8: 闸机刷卡/通行

推荐 task sequence:

```text
NavigateTo(turnstile_pre_wait)
AlignToTag(turnstile_card_reader_anchor)
DetectTurnstile(card_reader, lane_state)
TapCardOrWaitStaffOpen
WaitGateOpen
VerifyLaneClear
NavigateThroughTurnstileSlow
VerifyGuestsFollow
```

evidence:

```json
{
  "kind": "turnstile_state",
  "gate_open": true,
  "lane_clear": true,
  "card_reader_visible": true,
  "card_tap_attempt": 1,
  "staff_override": false,
  "confidence": 0.86
}
```

速度:

- 需求写了“不超过 2m/s”，但 G1 人形机器人迎宾导览实际建议远低于这个值。
- 闸机、电梯、门禁附近建议 `0.15-0.35 m/s`。
- 大厅直线导览可按现场安全调到 `0.4-0.6 m/s` 起步。

### Step 9: 电梯完整 task subtree

建议显式拆分，不要折叠成一个 waypoint:

```text
CallElevatorAndEnter(target_floor)
  1. NavigateTo(elevator_outer_wait)
  2. AlignToTag(outer_door_anchor)
  3. DetectCallButton
  4. PressCallButton
  5. WaitOuterDoorOpen
  6. VerifyEntryClear
  7. EnterElevator
  8. AlignToTag(inner_panel_anchor)
  9. DetectTargetFloorButton
  10. WaitStaffCloseInstruction
  11. PressTargetFloorButton
  12. VerifyButtonLight
  13. WaitElevatorTravelStart
  14. WaitTargetFloorByBarometer
  15. WaitInnerDoorOpen
  16. VerifyTargetFloorStillTrue
  17. WaitGuestsExitOrStaffInstruction
  18. ExitElevator
```

每一步都要有:

```text
per_attempt_timeout_ms
max_attempts
overall_deadline_ms
required_evidence
recovery_action
on_exhausted
```

### Step 10: Mission Supervisor / Gateway 接入

建议 `botbrain_ws_gateway` 暴露:

```text
GET  /g1/healthz
GET  /g1/preflight
GET  /g1/evidence
GET  /g1/evidence/{kind}
POST /g1/tasks/run
POST /g1/tasks/{task_id}/cancel
GET  /g1/tasks/{task_id}
GET  /g1/ros/topics
```

`POST /g1/tasks/run` 示例:

```json
{
  "schema_version": "g1.task.run.v1",
  "mission_run_id": "run-20260709-001",
  "action_class": "elevator_press_button",
  "target": {
    "facility_id": "elevator_a",
    "panel_id": "elevator_a_inner_panel",
    "button_label": "14"
  },
  "required_evidence": [
    "button_panel.fresh",
    "target_button.detected",
    "robot_stable",
    "manipulation.workspace_ok"
  ],
  "timeout_ms": 15000
}
```

Mission Supervisor 只看 action result 和 evidence，不 import ROS/rclpy，也不直接调 Unitree SDK。

### Step 11: BotBrain 前端扩展

复用现有:

```text
frontend/src/components/ros-camera-img.tsx
frontend/src/components/ros-rgbd-camera.tsx
frontend/src/components/yolo-detections.tsx
frontend/src/components/mission-control/*
frontend/src/hooks/ros/*
frontend/src/types/mission-control.ts
```

建议增加:

| 前端能力 | 数据来源 | 作用 |
|---|---|---|
| Perception Evidence Panel | Supervisor `/snapshot` 或 `/evidence` | 展示门态、按钮、tag、楼层、闸机证据 |
| Live Overlay | `/compressed_camera` + YOLO/AprilTag evidence | 给操作者看 bbox/tag/door state |
| Active Action Panel | `snapshot.blackboard_summary.active_action` | 展示正在按按钮/等门/等楼层 |
| Retry Panel | `retry_summary` | 展示 attempt、failure_class、recovery_action |
| Preflight Sensor Panel | `/preflight` | D435i、MID360、YOLO、AprilTag、Dex3、floor_estimate 是否 ready |
| Manual Confirm | `pending_decisions` | 高风险步骤人工批准/接管 |

不要做:

- 不要让浏览器直接调用 `/cmd_vel` 或 Unitree DDS。
- 不要让前端根据 `/barometer`、`/apriltag/detections`、`/yolo/detections` 自己判断 TC 完成。
- 不要把 Supabase `yolo_data` 当实时控制输入。它适合作审计/历史检索。

---

## 9. 推荐文件改造清单

### 9.1 先修补丁

```text
botbrain_ws/src/bot_localization/bot_localization/launch/realsense.launch.py
  - back scan node 按配置创建
  - D435i IMU 参数配置化
  - topic namespace 明确化

botbrain_ws/src/bot_localization/bot_localization/scripts/compressed_realsense.py
  - front/back 按参数创建
  - 移除绝对 /back_camera
  - QoS 参数化

botbrain_ws/src/bot_navigation/launch/apriltag_detection.launch.py
  - robot_name/namespace LaunchArgument
  - camera topic/camera_info topic 参数化
  - tag family/size 参数化或读取 apriltag_map.yaml

botbrain_ws/src/bot_yolo/bot_yolo/yolo_node.py
  - 输出 bbox、header、latency、model_id
  - 支持 profile/model switch
  - 可选 depth/camera_info 反投影
```

### 9.2 新增 perception 包

```text
botbrain_ws/src/bot_perception/package.xml
botbrain_ws/src/bot_perception/setup.py
botbrain_ws/src/bot_perception/config/elevator_state.yaml
botbrain_ws/src/bot_perception/config/button_panel.yaml
botbrain_ws/src/bot_perception/bot_perception/elevator_state_estimator.py
botbrain_ws/src/bot_perception/bot_perception/button_panel_estimator.py
botbrain_ws/src/bot_perception/bot_perception/turnstile_state_estimator.py
botbrain_ws/src/bot_perception/bot_perception/evidence_mux.py
botbrain_ws/src/bot_perception/launch/perception.launch.py
```

### 9.3 新增 Gateway / Mission 接口

如果当前 `botbrain_ws_gateway` 还未在 `botbrain_ws_aitech` 落地，建议新增:

```text
botbrain_ws/src/botbrain_ws_gateway/
  gateway_api.py
  ros_action_client.py
  evidence_store.py
  task_runtime.py
  schemas/
```

最低限度 endpoint:

```text
GET /g1/healthz
GET /g1/preflight
GET /g1/evidence
POST /g1/tasks/run
GET /g1/tasks/{task_id}
```

### 9.4 前端

```text
frontend/src/config/robot-profiles/profiles/g1-edu-dc-tour.ts
frontend/src/types/perception-evidence.ts
frontend/src/components/mission-control/PerceptionEvidencePanel.tsx
frontend/src/components/mission-control/ActiveActionEvidencePanel.tsx
frontend/src/components/camera-evidence-overlay.tsx
frontend/src/services/mission-supervisor.ts
```

---

## 10. QoS、频率和资源建议

### 10.1 QoS

| 数据 | QoS |
|---|---|
| 原始彩色/深度图像 | `best_effort`, `volatile`, `keep_last=1` |
| 压缩预览图 | `best_effort` 或前端链路 `cbor`, `keep_last=1` |
| 点云 / LaserScan | `best_effort`, `keep_last=1-5` |
| TF | 按 ROS 默认；确保 buffer 长度足够 |
| action goal/result | `reliable` |
| task evidence | `reliable`, `keep_last=10` 或 Gateway pull |
| emergency stop | `reliable`，必要时冗余通道 |
| diagnostics | `reliable`, `keep_last=10` |

### 10.2 频率

| 模块 | 建议起步 |
|---|---:|
| D435i color/depth | 640x480@15fps 或 848x480@15fps |
| D435i 压缩预览 | 640x360@10-15fps，JPEG 质量 20-40 |
| AprilTag | 10-15fps，关键阶段可提升 |
| YOLO general | 5-10fps |
| YOLO button ROI | 10-15fps，短时启用 |
| MID360 pointcloud | 10-20Hz |
| LaserScan/costmap | 10Hz 起步 |
| elevator_state_estimator | 10Hz |
| button_panel_estimator | 10Hz，只有电梯操作阶段启用 |
| Mission Supervisor snapshot | 1-5Hz |

### 10.3 Jetson 资源策略

- 默认只开导航主链、压缩相机、diagnostics。
- 到达电梯厅后启用 `elevator_outer` YOLO profile。
- 进电梯后切换 `elevator_inner` YOLO profile。
- 离开电梯后关闭电梯模型，恢复 general/person 模型。
- Dex3/arm 操作期间暂停非必要视觉模型，保留按钮确认模型。
- TensorRT engine 在部署阶段预生成，不在正式演示第一次 configure 时导出。

---

## 11. 测试与验收步骤

### 11.1 离线 rosbag 测试

每个关键点录制:

```bash
ros2 bag record \
  /tf /tf_static \
  /g1_robot/front_camera/color/image_raw \
  /g1_robot/front_camera/color/camera_info \
  /g1_robot/front_camera/depth/image_rect_raw \
  /g1_robot/front_camera/aligned_depth_to_color/image_raw \
  /g1_robot/pointcloud \
  /g1_robot/scan \
  /g1_robot/perception/apriltag/detections \
  /g1_robot/perception/yolo/detections \
  /g1_robot/floor_estimate
```

场景:

1. 大厅空场。
2. 大厅多人跟随。
3. 闸机开/关/半开/有人挡。
4. 电梯外门关、开、半开、有人进出。
5. 电梯内按钮各楼层、灯亮/不亮、反光。
6. 电梯中途停靠非目标楼层。
7. 11/14/15 楼出梯和门禁。

### 11.2 模块级验收

| 模块 | 验收命令/指标 |
|---|---|
| D435i | `topic hz` 稳定，`camera_info` 与 image 同步，压缩图不断流 |
| MID360 | 点云频率稳定，TF lookup 成功，Nav2 costmap 有障碍 |
| AprilTag | 固定距离 pose 抖动达标，tag 丢失状态可见 |
| YOLO | bbox/latency/model_id 输出完整，TensorRT engine 不临场导出 |
| Door State | 半开/unknown 保守，不误判 safe_to_enter |
| Button Panel | press point 在 pelvis/hand frame 内可视化正确 |
| Manipulation | 手臂启用时底盘停止，按压后能撤回，失败可 cancel |
| Gateway | action 有 accepted/running/result，evidence 可查询 |
| Mission Control | snapshot、active_action、retry、pending_decision 全部可见 |

### 11.3 端到端验收

按 36 项用例拆成 4 轮:

1. **纯回放**: Mission Supervisor playback，不接真机。
2. **感知回放**: rosbag + perception pipeline，验证 evidence。
3. **半实物**: 真机不动，D435i/MID360/Dex3 空动作，验证 Supervisor。
4. **真机低速**: 只走 UG 大厅到电梯外，不进电梯。
5. **真机电梯闭环**: 单楼层往返，先人工确认每个高风险步骤。
6. **多楼层导览**: 11/14/15 楼全流程，保留人工接管。

通过条件:

```text
高风险动作没有前端私自判定完成
所有电梯/闸机/门禁动作都有 evidence
门态 unknown/half_open 不触发进出电梯
错楼层不出梯
按钮未确认灯亮不进入 travel wait
重试次数可见，耗尽进入 WAITING_OPERATOR 或 SAFE_STOP
rosbag 可复现关键误判
```

---

## 12. 风险和取舍

### 12.1 最大风险

1. **电梯门状态误判**  
   AprilTag 被遮挡、YOLO 误检、深度反光都可能导致误判。必须多源融合，unknown 保守。

2. **按钮小目标定位误差**  
   bbox 中心不是按钮真实可按点，深度有空洞，按钮面板反光。必须用 ROI、平面拟合、视觉确认灯。

3. **G1 人形机器人近距离操作稳定性**  
   按按钮要求机器人脚下稳定、身体姿态稳定、手臂 workspace 合理。必须先停底盘再操作。

4. **Jetson 资源争抢**  
   FAST-LIO/Open3D/Nav2/YOLO/RealSense/前端/语音同时跑，容易抢 GPU/CPU/内存。必须按 mission phase 启停模型。

5. **多楼层地图和楼层证据混乱**  
   map frame、floor label、barometer、elevator state 必须由 Supervisor 统一解释，不能让各模块各自改变 floor。

### 12.2 关键取舍

| 方案 | 优点 | 缺点 | 建议 |
|---|---|---|---|
| 所有视觉模型常开 | 简单 | GPU 压力大，误报多 | 不采用 |
| 按任务阶段启停模型 | 资源可控、误报少 | 编排复杂 | 推荐 |
| AprilTag 判断门开 | 简单 | 遮挡误判风险高 | 只能作为证据之一 |
| YOLO 直接给按压点 | 快 | 3D 不可靠 | 必须结合 aligned depth/平面拟合 |
| 前端订阅所有 ROS topic 判断状态 | 开发快 | 任务 owner 混乱 | 不采用 |
| Gateway 聚合 evidence 给 Supervisor | 边界清晰、可审计 | 需要多写一层 | 推荐 |

---

## 13. 建议落地顺序

### 第一阶段: 先打通稳定观测

1. 修 D435i front-only bringup 和 namespace。
2. 修 AprilTag launch namespace。
3. 修 YOLO 输出 bbox/header/latency。
4. 建 `sensor_manifest.yaml` 和 `apriltag_map.yaml`。
5. 前端只显示 live camera + tag/yolo overlay + diagnostics。

交付物:

```text
ros2 topic list 清晰
TF 树正确
D435i/MID360/YOLO/AprilTag 都可生命周期管理
rosbag 能记录完整感知数据
```

### 第二阶段: 做电梯外部闭环

1. `elevator_state_estimator`。
2. `button_panel_estimator` 外呼按钮版本。
3. `press_call_button` action。
4. Supervisor 接入 `call_elevator` task。
5. Mission Control 显示 door_state/button evidence/retry。

交付物:

```text
机器人到电梯外等待点
识别外呼按钮
按外呼
等待门开
门态 unknown 时不移动
```

### 第三阶段: 做电梯内部按钮

1. 内部 panel tag 和 button map。
2. `press_floor_button` action。
3. 按钮灯确认。
4. floor_estimate 目标楼层确认。
5. 出梯前门态 + 楼层二次确认。

交付物:

```text
单楼层上行闭环
目标按钮灯确认
气压楼层确认
错楼层不出梯
```

### 第四阶段: 闸机、门禁、多楼层导览

1. 闸机 card_reader/gate_state。
2. 门禁 card_reader/door_state。
3. 11/14/15 楼 map/tag/evidence。
4. 36 项全流程 playback + HIL + 真机低速演示。

---

## 14. 最小可用版本定义

MVP 不要求机器人完全自主处理所有异常。MVP 应满足:

```text
1. G1 能稳定发布 D435i、MID360、TF、Nav2、YOLO、AprilTag、Dex3 diagnostics。
2. Mission Supervisor 能看到 action/evidence，而不是只看到 command_sent。
3. 电梯门状态至少有 OPEN/CLOSED/HALF_OPEN/UNKNOWN，UNKNOWN 不移动。
4. 电梯按钮按压必须确认灯亮，否则不进入电梯 travel。
5. 楼层确认必须来自 floor_estimate 或人工确认，不能只靠时间。
6. 前端能展示 active_action、retry、evidence、pending_decision。
7. 每次失败都能通过 rosbag 和 MissionEvent 复盘。
```

---

## 15. 本项目的建议结论

1. `botbrain_ws_aitech` 已经具备正确的大方向: FAST-LIO/Open3D/Nav2、D435i、MID360、YOLO、AprilTag 原型、Dex3/arm Mode-B、Next.js 前端。
2. 现在最大缺口不是缺某个模型，而是缺 **perception evidence contract** 和 **Mission Supervisor action/evidence 闭环**。
3. `bot_yolo` 可以复用，但要从“检测记录/可视化”升级为“任务感知证据发布者”。
4. `BotBrainmx2` 的 Mission Control、retry_summary、active_exception、pending_decision 类型值得直接迁移/对齐。
5. 电梯和闸机必须拆成显式 task action，不应继续折叠在 waypoint 里。
6. D435i 负责近场精细感知，MID360 负责全局导航和安全避障；不要让任何单一传感器独自决定高风险动作。
7. 前端应做观测、回放、人工确认和接管；任务完成判断必须留在 Mission Supervisor。

