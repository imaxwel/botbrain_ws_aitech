# Unitree G1 EDU 在松软地毯区域行走与 SLAM 建图最佳实践调研

日期：2026-07-07  
范围：Unitree G1 EDU、`botbrain_ws_aitech`、Fast-LIO2、Open3D ICP/localization、Nav2、数据中心展厅瓷砖/松软地毯混合地面。  
读者：项目经理、CTO、机器人系统负责人。

## 0. 真实问题与结论

现场现象不是简单的“地毯点云特征少”，而是：

```text
松软地毯改变足底接触动力学
  -> G1 原厂平衡/行走控制为了稳定性产生更多小步、调整步、姿态微修正
  -> 头部 MID360 与机体 IMU 经历更多低频摆动、yaw 微摆、启停瞬态
  -> Fast-LIO2 去畸变/短时运动先验和 ICP 匹配输入质量下降
  -> 地毯区地图墙体变厚、重影、局部弯曲、map->odom 跳变或定位置信度下降
```

截至本次联网检索，没有查到 Unitree 原厂公开发布的“G1 EDU 地毯/软地面专用步态参数、阻尼参数、足底材料建议或 SLAM 建图建议”。能确认的原厂公开信息是：

| 证据项 | 可确认内容 | 对本项目的含义 |
|---|---|---|
| Unitree G1 产品页 | G1 EDU 为 23-43 自由度，约 35kg+，单腿 6 自由度，膝关节最大扭矩 EDU 120 N.m，传感器含深度相机和 3D LiDAR，支持二次开发 | G1 EDU 硬件能力足够，但官方没有把地毯建图作为标准公开能力承诺 |
| Unitree G1 developer 文档 | 运控计算单元不向公众开放；开发者主要使用开发计算单元；G1 头部配 MID360 LiDAR 和 D435i；公开提示建议人形机器人动作开发时尽量让膝盖接近直立、降低步频、双脚略靠近，不建议带灵巧手做剧烈动作或平衡测试 | 低层步态/阻抗/接触控制大概率不能通过公开 SDK 做“地毯阻尼调参”；应通过速度、路径、FSM、质量门控和建图流程解决 |
| Unitree ROS2 / SDK2 文档 | 可读 `SportModeState`、`LowState`、IMU、足端力/估计力、速度、gait/mode 等；可通过官方 DDS/ROS2 链路发运动命令、FSM、速度档、连续步态 | `SportModeState` 应被纳入 SLAM 质量判定，而不仅仅作为前端遥测 |
| Unitree RL Gym real deployment | 物理部署示例强调吊装、zero torque、debug mode、逐步落地、稳定后再运动；并声明示例控制程序不是稳定控制程序，异常时及时退出 | 原厂对新控制策略上真机非常保守；地毯专项不应贸然替换原厂行走控制 |

本报告的核心建议：

1. **不要把“连续自主走过松软地毯时生成的 Fast-LIO2 地图”当成最终主地图。** 数据中心展厅这种固定场景，最佳实践是先用稳定平台/离线流程生成主地图，再让 G1 做定位和巡检验证。
2. **短期无需改 Fast-LIO2 核心，先做 carpet profile。** 地毯区限速、限角速度、限加速度、避免原地 yaw、放宽 Nav2 goal tolerance、减少控制器微修正，并把地毯区作为地图语义区域。
3. **中期要加 SLAM 质量门控。** 用 `/lf/odommodestate`、`/lf/lowstate`、`/livox/imu`、`/cmd_vel_out`、ICP fitness 等信号识别“碎步/调整步/接触不可靠窗口”，在这些窗口暂停地图插入或离线过滤 bag。
4. **长期建议升级为多会话/多因子建图。** 实时导航继续用 Fast-LIO2 + Open3D ICP，但最终地图构建用离线 pose graph、loop closure、R3LIVE/视觉约束、AprilTag/临时标志物或外部测量做约束。

一句话给 CTO：**这不是单纯调大 ICP 迭代次数能解决的问题，而是“柔顺接触导致运动输入质量下降”的系统问题；应该把地毯识别为特殊工况，做运动 profile + SLAM 质量门控 + 离线主地图。**

## 1. 本地系统现状

当前 `botbrain_ws_aitech` 与本问题直接相关的链路如下：

```text
Unitree G1 EDU
  -> MID360 /livox/lidar + /livox/imu
  -> g1_pkg imu_flip.py
  -> fast_lio fastlio_mapping
      /Odometry, /cloud_registered, /cloud_registered_body_1
  -> open3d_loc global_localization_node
      map->odom / ICP localization
  -> Nav2 MPPI
  -> g1_pkg robot_write_node
      /cmd_vel_out -> Unitree LocoClient.Move()
```

关键本地参数：

| 文件 | 现状 | 影响 |
|---|---|---|
| `g1_pkg/launch/fast_lio.launch.py` | 启动 `imu_flip.py`，再启动 `fastlio_mapping`；注释说明 MID360 roll-180 安装下 SDK 修正点云但 IMU 仍需 Y/Z 取反 | IMU/点云坐标一致性已经被显式处理；地毯问题不应优先怀疑“地图上下颠倒”类外参错误 |
| `fast_lio/config/mid360.yaml` | `feature_extract_enable: false`，`point_filter_num: 3`，`max_iteration: 3`，`filter_size_surf/map: 0.5`，`det_range: 15.0`，`time_sync_en: false`，`extrinsic_est_en: false` | 配置接近 Fast-LIO2 对 MID360 的常见直注册路线；不建议先改成在线估外参 |
| `g1_pkg/launch/localization_3d.launch.py` | Open3D ICP `loc_frequence: 2.5`，`threshold_fitness: 0.5`，`confidence_loc_th: 0.7`，`initialpose z=1.247`，并桥接 `body` 到 `g1_robot/base_footprint` | ICP 作为定位修正可用，但如果地毯段地图本身劣化，ICP 会把错误固化 |
| `g1_pkg/config/nav2_params.yaml` | MPPI `vx_max: 0.35`，`wz_max: 1.0`，`ax_max: 0.4`，`ay=0`，DiffDrive 模型，局部点云障碍层来自 `/cloud_registered_body_1` | 已经不是高速配置，但地毯段仍需更低角速度/加速度与更少微修正 |
| `g1_pkg/src/g1_write.cpp` | `cmd_vel_out` 内部限幅 `vx/vy <= 0.6 m/s`、`wz <= 1.0 rad/s` | Nav2 已更保守；地毯专项要改 Nav2 profile 或 speed limit，而不是只依赖 write node 饱和 |
| `g1_pkg/src/g1_driver/g1_driver.cpp` | 初始化 LocoClient，设置 `ContinuousGait(keep_move)`、`SetSpeedMode(speed_mode)`；提供 `Move()`、`StopMove()`、`SetFsmId()` | 可以做速度档和连续步态 A/B 测试，但不能公开调低层接触阻抗 |

当前配置已经做了不少真机修正。地毯区出问题，更像是**运动质量/建图采样策略**问题，不像是单个 YAML 参数错误。

## 2. Unitree 原厂公开建议：能确认什么，不能确认什么

### 2.1 未发现地毯专项建议

检索英文/中文关键词包括：

```text
Unitree G1 EDU carpet walking balance
Unitree G1 carpet floor manual
Unitree G1 developer balance floor
site:support.unitree.com G1 carpet
"Unitree G1" "carpet"
```

结果没有发现官方公开的“G1 EDU carpet mode / soft floor mode / carpet damping / carpet gait tuning”文档。

因此，下面这些说法不能作为原厂结论：

| 说法 | 证据状态 |
|---|---|
| G1 EDU 有公开的地毯模式 | 未查到 |
| Unitree 建议在地毯上调某个阻尼/迟滞参数 | 未查到 |
| Unitree 建议换某种足底材料用于地毯建图 | 未查到 |
| Unitree 对 G1 EDU 在松软地毯上的 SLAM 建图有公开最佳实践 | 未查到 |

### 2.2 可以采纳的原厂公开信息

Unitree G1 developer 文档有三条对本问题间接有用的提示：

1. “Developed leg movement procedures to bring the knee to upright or upright as far as possible。”
2. “Reduce the stride frequency as much as possible, and avoid standing still as far as possible。”
3. “Keep your feet slightly closer ... when walking。”

这段文字不是地毯专项，也更像是面向人形机器人动作开发/视频展示的运动风格提示。但它和现场现象一致：**G1 在软地面上如果步频变高、脚距变大、姿态来回调整，视觉上和 SLAM 上都会变差。**

Unitree G1 developer 文档还明确：

| 原厂信息 | 工程含义 |
|---|---|
| 运控计算单元 dedicated to Unitree motion control program and not open to the public | 不要假设可以通过公开 API 直接调足底阻抗、接触模型、地毯参数 |
| 开发计算单元 PC2 才是二次开发入口 | 我们能做的是上层速度/路径/感知/建图策略 |
| G1 头部有 MID360 LiDAR 和 D435i 深度相机 | 地毯问题可以通过多传感器/多约束缓解 |
| 灵巧手附着时不建议做 overly intense actions, such as running or balance tests | 地毯建图时应减少上身负载变化和手臂动作，避免上半身扰动进入 LiDAR 轨迹 |

Unitree ROS2 文档暴露的 `SportModeState` 很关键：

```text
mode
gait_type
foot_raise_height
position / velocity / yaw_speed
imu_state
foot_force
foot_position_body
foot_speed_body
```

这意味着现场不应只看 `/cloud_registered` 是否漂亮，而应把 G1 自身的 gait/contact telemetry 作为 SLAM 输入质量的一部分。

## 3. 为什么瓷砖可用，地毯变差

### 3.1 接触动力学变了

瓷砖地面近似刚性接触：

```text
足底落地 -> 高刚度支撑 -> 接触事件清晰 -> 控制器快速稳定 -> base motion 可预测
```

松软地毯是柔顺、耗能、带滞后的接触：

```text
足底落地 -> 地毯压缩/回弹/剪切 -> 接触高度与摩擦状态滞后
          -> 控制器感知到支撑不确定
          -> 小步补偿、姿态补偿、yaw/roll/pitch 微修正增加
```

用户描述的“瓷砖地面的高频冲击变成低通滤波/增加阻尼/迟滞系数的感觉”是准确的。对腿式机器人来说，这会破坏两个常用假设：

1. **足端接触是短时间稳定约束。**
2. **机体速度变化在 LiDAR 单帧内可由 IMU 充分解释且不出现过多控制瞬态。**

### 3.2 地毯不一定让 LiDAR 看不清，但会让 LiDAR 的运动轨迹变差

MID360 主要看墙、展台、机柜、玻璃、立柱和人，不靠地毯纹理建图。地毯真正的破坏路径通常是：

```text
更多碎步/调整步
  -> 头部 LiDAR 的 6DoF 轨迹更“抖”
  -> 单帧扫描去畸变更难
  -> 点到地图匹配残差增大
  -> 墙面增厚、边缘重影、走廊方向漂移
```

也就是说，**地毯区 SLAM 差，根因常常在机器人运动，不在地毯点云。**

### 3.3 Fast-LIO2 很强，但不是魔法

Fast-LIO2 的优点是直接把原始点注册到地图，不依赖手工特征；使用紧耦合 iterated Kalman filter 和 ikd-Tree，适合 MID360、ARM 平台和高实时性。论文宣称在多种 LiDAR、UAV/手持平台和 ARM 处理器上高效鲁棒。

但 Fast-LIO2 仍然需要：

| 条件 | 地毯区风险 |
|---|---|
| LiDAR-IMU 时间同步稳定 | 低频摆动和 yaw 微修正会放大时间偏差 |
| LiDAR-IMU 外参稳定 | G1 头部/机身柔性与安装件松动会在软地面更明显 |
| 场景几何可约束 | 数据中心展厅可能有长走廊、重复展柜、玻璃/反光、空旷区域 |
| 运动在单帧内可被 IMU 合理去畸变 | 碎步和启停瞬态让单帧内运动更复杂 |
| 地图更新窗口质量足够 | 错误帧被插入后，会污染后续匹配基准 |

因此，地毯区不能靠“把 ICP 迭代次数调大”解决。更大的迭代次数可能只是让错误匹配更自信。

## 4. 行业最佳实践：从论文和工程系统抽象出的共识

### 4.1 腿式机器人在滑/软/可变形地面上，应降低腿部里程计信任

Wisth、Camurri、Fallon 的 legged robot factor graph 系列工作明确指出：滑、软、泥地、松石等地形会让足端接触不确定，leg odometry 的误差难以建模；他们通过视觉、IMU、腿部里程计的紧耦合 factor graph 和 bias estimation 来处理接触非线性。在滑/软地面上，单纯依赖足端接触假设会漂。

对 G1 EDU 的直接含义：

```text
Unitree /lf/odommodestate 或腿部状态可以用于“质量评估”
但不应在地毯段被当作高可信主里程计硬约束
```

更好的做法是：

| 信号 | 用法 |
|---|---|
| 足端力/估计力变化 | 判断接触是否稳定 |
| gait_type 是否进入 adjust | 判断控制器是否在补偿 |
| foot_speed_body 在支撑期是否异常 | 判断脚底滑动或地毯沉陷 |
| IMU roll/pitch/yaw rate RMS | 判断上身扰动 |
| cmd_vel 与实际 velocity/yaw_speed 差值 | 判断运动执行质量 |

### 4.2 多传感器融合优于单一路线

截至 2026-07-07 的相关资料中，比较有决策价值的方向：

| 方向 | 代表资料 | 对本项目的启发 |
|---|---|---|
| LiDAR-IMU 直接法 | FAST-LIO2 | 继续作为实时 odom/mapping 主干，特别适合 MID360 |
| Factor graph + loop closure | LIO-SAM | 地毯区最终地图应通过 keyframe、loop closure、pose graph 修正，不只靠在线前端 |
| LiDAR-Inertial-Visual | R3LIVE | 展厅有墙面/展板/柜体，视觉可提供纹理和颜色约束，适合离线重建主地图 |
| Contact-aware legged estimation | Hartley 2018、Wisth 2019/2020、Lin 2021 等 | 腿部接触信息应被建模为有条件可信，而不是固定可信 |
| Adaptive / learned contact estimation | 2021-2026 多篇 contact/state estimation 工作 | 地毯区最应该做的是在线接触质量估计和 covariance/门控自适应 |
| Degenerate LiDAR benchmarking | GEODE dataset 2024 | 数据中心展厅可能有几何退化/重复结构，应显式做退化检测和外部约束 |

### 4.3 对固定展厅，最佳实践不是“让机器人自己边走边造主地图”

如果场地固定，且地图要给后续导览使用，最稳路线是：

```text
一次性高质量主地图
  -> 多会话/离线优化/人工验收
  -> G1 使用主地图做定位与导航
  -> 机器人运行中只做局部避障和定位修正，不随意改主地图
```

原因很现实：人形机器人在软地面上走路本来就是扰动源。让它在扰动最大的时候生产“系统真相地图”，相当于把最差的数据当作基准。

## 5. 推荐方案

### 5.1 立即可做：carpet operating profile

目标：不改核心算法，先降低地毯区的运动扰动和错误采样。

建议新增“地毯区 profile”，通过地图区域、waypoint 标签或任务脚本切换：

| 参数/行为 | 瓷砖区 | 地毯区建议 |
|---|---:|---:|
| `vx_max` | 0.35 m/s | 0.12-0.22 m/s |
| `wz_max` | 1.0 rad/s | 0.35-0.55 rad/s |
| `ax_max` | 0.4 m/s² | 0.15-0.25 m/s² |
| 原地旋转 | 可少量使用 | 尽量禁止，用弧线转弯替代 |
| 横向速度 `vy` | 当前 Nav2 已为 0 | 保持 0 |
| 倒退 | 不推荐 | 禁止 |
| goal xy tolerance | 0.05 m | 地毯内放宽到 0.10-0.18 m |
| goal yaw tolerance | 0.10 rad | 地毯内放宽到 0.18-0.30 rad |
| 路径形状 | 常规 | 长直线、少停顿、少急转、远离地毯边缘 |
| 上肢动作 | 可根据任务 | 建图/定位验证时锁定或使用稳定姿态 |

不要只“全局降速”。地毯真正怕的是启停和 yaw 微修正：

```text
低速 + 高频小角度纠偏 = 仍然会碎步
中低速 + 平滑弧线 + 少目标微调 = 更稳
```

对 `keep_move` / `ContinuousGait` 的建议：

| 模式 | 可能优点 | 可能风险 | 建议 |
|---|---|---|---|
| `keep_move=false` | 速度为 0 时不强制保持步态，减少站立踏步污染 | 频繁启停可能触发更多稳定调整 | 作为默认基线 |
| `keep_move=true` | 减少启停瞬态，可能让连续通过更平滑 | 若 0 速仍踏步，会在地毯上制造更多碎步 | 只做 A/B 测试，不能默认启用 |

测试方式：在同一路径上分别跑 `keep_move=false/true`，记录 `/lf/odommodestate`、`/livox/imu`、`/Odometry`、`/cloud_registered`，比较：

```text
IMU angular velocity RMS
yaw_speed RMS
cmd_vel -> actual velocity tracking error
gait_type=adjust 占比
ICP fitness / confidence
地图墙体厚度
```

### 5.2 立即可做：建图流程改为分段采样

地毯区建图不要“从瓷砖一路走进去一路写地图”。推荐流程：

1. 在瓷砖区建好基准地图，包含地毯入口两侧的墙/柱/展柜。
2. 地毯区只采稳定直线段，避免入口/出口/原地转身/目标点收敛阶段写入地图。
3. 进入地毯前后各留 1-2 m 稳定区，用于对齐。
4. 地毯段同一路径至少双向采集两次，离线比较墙体重合度。
5. 最终地图只合并质量通过的片段。

建议把地图更新窗口分为：

| 窗口 | 是否用于主地图 | 原因 |
|---|---|---|
| 起步 1-2 秒 | 否 | 启动步态瞬态 |
| 地毯入口第一步/第二步 | 否 | 接触模型突变最大 |
| 匀速直行 | 是 | 运动最可预测 |
| 原地 yaw 或小半径转弯 | 否 | 头部角速度和扫描畸变最大 |
| goal 收敛末端 | 否 | MPPI 小幅纠偏可能导致碎步 |
| 地毯出口第一步/第二步 | 否 | 接触模型再次突变 |

### 5.3 中期：增加 SLAM 质量门控节点

推荐新增一个轻量节点，先不改 Fast-LIO2 源码，做质量评分和 bag 标注：

```text
inputs:
  /lf/odommodestate
  /lf/lowstate
  /livox/imu
  /cmd_vel_out
  /Odometry
  /cloud_registered or ICP diagnostics

outputs:
  /slam_quality/state
  /slam_quality/score
  /slam_quality/bad_interval
  /mapping_gate/enabled
```

质量分可以先用规则，不必一开始上机器学习：

| 指标 | 异常含义 |
|---|---|
| `abs(yaw_speed)` 长时间高于期望，且 `cmd_vel.angular.z` 很小 | 被动姿态/平衡修正 |
| `imu gyro RMS` 或 jerk 超过瓷砖基线 2-3 倍 | 碎步/抖动窗口 |
| `gait_type=adjust` 占比上升 | 控制器正在调整 |
| 足端力方差异常或左右不对称 | 地毯接触不稳定 |
| `actual velocity - cmd_vel` 偏差大 | 速度执行不稳定 |
| ICP fitness 下降或 map->odom 更新被拒 | 外部定位质量下降 |

规则示例：

```text
GOOD:
  cmd_vel.x > 0.08
  abs(cmd_vel.angular.z) < 0.25
  gyro_rms_1s < tile_baseline * 1.8
  gait_adjust_ratio_2s < 0.15
  icp_confidence > 0.7

BAD:
  entrance/exit transition
  pure yaw
  goal convergence
  gait_adjust_ratio_2s > 0.30
  gyro_rms_1s > tile_baseline * 2.5
```

落地方式分两档：

| 档位 | 做法 | 风险 |
|---|---|---|
| 保守 | 只给 rosbag 打 bad interval 标签，离线过滤后重跑 Fast-LIO2/ICP | 不影响真机实时链路，推荐先做 |
| 激进 | 在线 gate `/livox/lidar` 或暂停地图插入 | 可能影响 Fast-LIO2 连续性，需要小心验证 |

不建议一开始在线丢 LiDAR 帧，因为 Fast-LIO2 前端需要连续 IMU/LiDAR 更新。更稳的是：**在线照常跑，离线用质量标签重建主地图。**

### 5.4 中期：改 Nav2 地毯区行为

在数据中心展厅这种场景，地毯区通常是固定区域。推荐在 2D map 上标注 carpet polygon，并让任务层或 Nav2 filter 切换 profile：

```text
tile_profile:
  vx_max 0.30-0.35
  wz_max 0.8-1.0
  xy_goal_tolerance 0.05

carpet_profile:
  vx_max 0.15-0.20
  wz_max 0.35-0.50
  ax_max 0.15-0.25
  no_pure_spin true
  xy_goal_tolerance 0.12
  yaw_goal_tolerance 0.20
```

路径层面：

| 做法 | 原因 |
|---|---|
| 地毯内 waypoint 不要太密 | 密 waypoint 会造成频繁目标收敛和碎步 |
| 不要在地毯边界放 goal | 边界是接触动力学突变点 |
| 先在瓷砖上完成朝向调整，再直行穿越地毯 | 避免地毯上原地旋转 |
| 地毯区入口前设置“稳定预备段” | 让速度进入平滑状态 |
| 出地毯后再做精确定位/朝向修正 | 把高精度动作放回刚性地面 |

### 5.5 中期：Open3D ICP 的使用策略

当前 `open3d_loc` 以 2.5 Hz 做全局/局部定位修正，`threshold_fitness=0.5`、`confidence_loc_th=0.7`。地毯区建议：

1. 不要在地毯异常窗口强行更新 `map->odom`。
2. ICP 置信度下降时，宁愿短时间依赖 Fast-LIO odom 连续性，也不要接受低质量跳变。
3. 地毯区定位目标应依赖墙、柱、展柜、门框，不依赖地面。
4. 如果展厅几何重复，增加 AprilTag/ArUco/视觉标志或固定反光特征，给 ICP 提供可区分锚点。

一个容易被忽略的点：**地毯区的 ICP 不是只看 fitness 数值，还要看接受更新后的轨迹是否平滑。** 低纹理/重复结构场景中，错误 ICP 也可能有看似不错的 fitness。

### 5.6 长期：离线主地图 + 在线定位分离

推荐最终生产流程：

```text
采集阶段:
  rosbag record raw lidar/imu/g1 state/tf/cmd_vel/icp/nav2
  多次通过地毯，标记 carpet polygon 和 bad intervals

离线阶段:
  过滤 bad intervals
  重跑 Fast-LIO2 或 FAST-LIO2 + pose graph
  加 loop closure
  ICP 多会话对齐
  人工检查墙厚、闭环误差、地毯入口/出口连续性

部署阶段:
  固定主地图
  G1 在线只做 localization + obstacle avoidance
  地毯区关闭主地图更新
```

如果预算允许，最稳的主地图采集方式不是 G1 自己走，而是：

| 采集方式 | 稳定性 | 备注 |
|---|---|---|
| 手持/背包 MID360 + IMU | 高 | 地毯不会影响足底控制；需要和 G1 坐标对齐 |
| 小推车/三脚架分站扫描 | 高 | 展厅固定场景最可靠；地毯区可慢速稳定采集 |
| G1 慢速多会话采集 | 中 | 可复用现有硬件，但必须做质量门控和离线过滤 |
| G1 单次自主在线建图 | 低 | 不推荐作为最终主地图 |

### 5.7 可选：视觉/标志物辅助

数据中心展厅通常允许布置少量临时工程标志，且不会长期影响观感。建议：

| 手段 | 使用场景 | 价值 |
|---|---|---|
| AprilTag / ArUco 临时贴在墙/展台侧面 | 建图/验收阶段 | 给地毯区入口、出口、长走廊提供强约束 |
| 天花板或墙面自然特征 | 运行阶段 | D435i/RGB 可辅助 R3LIVE 或视觉检查 |
| 反光柱/几何标志物 | LiDAR ICP 退化区域 | 增加几何可区分性 |
| UWB/外部定位 | 大面积开阔重复区域 | 给全局漂移兜底 |

注意：不要把标志物放地面地毯上，地毯会动、会压缩、会被人踩，且不在 MID360 最可靠视场内。

## 6. 参数建议与优先级

### 6.1 不建议优先改的参数

| 参数 | 不建议原因 |
|---|---|
| `extrinsic_est_en: true` | 地毯问题不是在线估外参能解决；在线估外参可能把运动误差吸收到外参里 |
| 大幅增大 `max_iteration` | 可能增加 CPU 和错误收敛，并不能修复输入轨迹抖动 |
| 大幅增大 IMU cov 试图“滤掉地毯” | 地毯导致的 IMU 变化是真实运动，不是纯噪声 |
| 盲目降低 voxel 到很小 | 会增加地图中的重影细节，CPU 压力也上升 |
| 让 ICP 低置信度也更新 | 地毯区最怕错误 map->odom 跳变 |

### 6.2 建议优先验证的改动

| 优先级 | 改动 | 预期收益 | 风险 |
|---:|---|---|---|
| P0 | 地毯区禁原地旋转，改弧线转向 | 减少 yaw 碎步和扫描畸变 | 路径需要更大空间 |
| P0 | 地毯区 `vx_max=0.15-0.20`、`wz_max=0.35-0.50`、`ax_max=0.15-0.25` | 降低启停和角速度扰动 | 通过时间增加 |
| P0 | 地毯入口/出口/goal 收敛窗口不进主地图 | 直接减少污染帧 | 需要离线流程或 gate |
| P1 | 记录 `/lf/odommodestate`、`/lf/lowstate`、`/livox/imu` 并打质量分 | 找到碎步和 SLAM 劣化的因果链 | 需要开发节点 |
| P1 | 地毯区放宽 goal tolerance | 减少末端微修正 | 最终停点精度下降 |
| P1 | 多会话离线合图 + loop closure | 主地图质量显著提升 | 工程流程复杂 |
| P2 | 引入 AprilTag/视觉/外部锚点 | 解决重复/退化几何 | 需要场地配合 |
| P2 | 尝试 R3LIVE/视觉纹理重建 | 更好的展厅可视地图 | 算力和工程复杂度增加 |

## 7. 验收指标

不要只凭 RViz 主观看“差不多”。建议设定以下量化指标：

| 指标 | 测量方式 | 建议目标 |
|---|---|---|
| 墙体厚度 | 同一墙面点云横向厚度，P95 | 瓷砖/地毯差异小于 1.5 倍 |
| 闭环误差 | 地毯环路回到入口的 pose 差 | 平面误差 < 0.15-0.25 m，yaw < 3-5° |
| ICP 置信度 | `confidence_loc_th` 通过率/fitness | 地毯稳定段通过率 > 90% |
| IMU 稳定性 | gyro RMS、acc jerk | carpet profile 比原 profile 下降 30%+ |
| gait adjust 占比 | `SportModeState.gait_type` 或等价状态 | profile 后显著下降 |
| Nav2 控制抖动 | cmd_vel 频繁变号/小幅震荡次数 | 地毯区接近 0 |
| 到点误差 | 人工量测或 AprilTag/全站仪 | 地毯区允许略放宽，但不影响导览体验 |

关键验收图：

```text
同一地毯路径，三组对比:
  A. 当前参数在线建图
  B. carpet profile 在线建图
  C. carpet profile + 离线 bad interval 过滤建图

每组输出:
  1. 地图俯视图
  2. 墙体截面厚度
  3. 轨迹 yaw/yaw_rate 曲线
  4. IMU gyro RMS 曲线
  5. gait/contact quality 曲线
```

如果 C 明显好于 B，而 B 又明显好于 A，就证明问题链路成立。

## 8. 推荐实施路线

### 第 1 周：不改核心，拿证据

1. 固定一条瓷砖路径和一条地毯路径。
2. 同时录：

```bash
/livox/lidar
/livox/imu
/Odometry
/cloud_registered
/cloud_registered_body_1
/tf
/tf_static
/lf/odommodestate
/lf/lowstate
/g1_robot/cmd_vel_out 或实际命名空间下 cmd_vel_out
/g1_robot/odom
/diagnostics
```

3. 跑当前 profile 与低速地毯 profile。
4. 输出墙厚、IMU、yaw、gait adjust、ICP confidence 对比。

### 第 2 周：地毯 profile 上线

1. 在 mission/waypoint 层标注 carpet zone。
2. 进入 carpet zone 前切低速/低角速度/低加速度 profile。
3. 禁止地毯区原地旋转和倒退。
4. 放宽地毯区目标容差。
5. 地毯入口/出口不设 waypoint，waypoint 放在刚性地面或地毯内部稳定直线段。

### 第 3-4 周：SLAM 质量门控

1. 开发 `slam_quality_monitor`。
2. 先只发布质量分和 bad interval，不影响实时控制。
3. 离线过滤 bad intervals 重跑建图。
4. 验收后再考虑在线暂停地图保存或在线 gate。

### 第 2 阶段：主地图生产流程升级

1. 用手持/推车/三脚架或 G1 多会话离线方式生产主地图。
2. 引入 loop closure 或 pose graph。
3. 地毯区只做 localization，不在线更新主地图。
4. 对重复/开阔/玻璃区域加视觉或临时标志物。

## 9. 风险与决策点

| 风险 | 后果 | 建议决策 |
|---|---|---|
| 继续用 G1 单次在线走图生产主地图 | 地毯区地图不可复现，后续导航调参被污染 | 停止作为主流程 |
| 把地毯问题误判为 ICP 参数问题 | 越调越复杂，定位跳变仍存在 | 先做运动质量和数据质量分析 |
| 贸然改低层控制或 RL policy | 安全风险高，且原厂公开支持有限 | 不建议作为近期路线 |
| 地毯区目标点过密 | 控制器持续微修正，碎步增多 | 重规划 waypoint |
| 忽略上肢/载荷姿态 | 头部 LiDAR 轨迹受上身扰动 | 建图时锁定上身稳定姿态 |
| 不记录原始 bag | 无法复盘，无法证明问题 | P0 强制录 bag |

## 10. 参考资料

Unitree 官方/原厂公开资料：

1. Unitree G1 产品页，G1/G1 EDU 参数、传感器、二次开发、安全提示：<https://www.unitree.com/g1/>
2. Unitree G1 中文产品页，G1/G1 EDU 参数与安全提示：<https://www.unitree-robot.com/cn/g1/>
3. Unitree G1 developer 文档，运控/开发计算单元、MID360/D435i、动作开发提示：<https://support.unitree.com/home/en/G1_developer>
4. Unitree SDK2 GitHub，官方 SDK2 与文档中心入口：<https://github.com/unitreerobotics/unitree_sdk2>
5. Unitree ROS2 GitHub，ROS2 接入、`SportModeState`、`LowState`、G1 示例：<https://github.com/unitreerobotics/unitree_ros2>
6. Unitree RL Gym，G1/H1/H1_2 RL、Sim2Real、物理部署示例和安全提示：<https://github.com/unitreerobotics/unitree_rl_gym>

SLAM / LiDAR-IMU：

1. Xu et al., “FAST-LIO2: Fast Direct LiDAR-inertial Odometry”, arXiv:2107.06829：<https://arxiv.org/abs/2107.06829>
2. Shan et al., “LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping”, arXiv:2007.00258：<https://arxiv.org/abs/2007.00258>
3. Lin and Zhang, “R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package”, arXiv:2109.07982：<https://arxiv.org/abs/2109.07982>
4. Qian et al., “RF-LIO: Removal-First Tightly-coupled Lidar Inertial Odometry in High Dynamic Environments”, arXiv:2206.09463：<https://arxiv.org/abs/2206.09463>
5. Chen et al., “Heterogeneous LiDAR Dataset for Benchmarking Robust Localization in Diverse Degenerate Scenarios”, arXiv:2409.04961：<https://arxiv.org/abs/2409.04961>

腿式机器人接触/状态估计：

1. Wisth, Camurri, Fallon, “Robust Legged Robot State Estimation Using Factor Graph Optimization”, arXiv:1904.03048：<https://arxiv.org/abs/1904.03048>
2. Wisth, Camurri, Fallon, “Preintegrated Velocity Bias Estimation to Overcome Contact Nonlinearities in Legged Robot Odometry”, arXiv:1910.09875：<https://arxiv.org/abs/1910.09875>
3. Hartley et al., “Legged Robot State-Estimation Through Combined Forward Kinematic and Preintegrated Contact Factors”, arXiv:1712.05873：<https://arxiv.org/abs/1712.05873>
4. Lin et al., “Legged Robot State Estimation using Invariant Kalman Filtering and Learned Contact Events”, arXiv:2106.15713：<https://arxiv.org/abs/2106.15713>
5. Maravgakis et al., “Probabilistic Contact State Estimation for Legged Robots using Inertial Information”, arXiv:2303.00538：<https://arxiv.org/abs/2303.00538>
6. Menner and Berntorp, “Simultaneous State Estimation and Contact Detection for Legged Robots by Multiple-Model Kalman Filtering”, arXiv:2404.03444：<https://arxiv.org/abs/2404.03444>
7. Nisticò et al., “Multi-Sensor Fusion for Quadruped Robot State Estimation using Invariant Filtering and Smoothing”, arXiv:2504.20615：<https://arxiv.org/abs/2504.20615>
8. Seo et al., “GAIT: Legged Robot Proprioceptive State Estimation with Attention over Inertial-Leg Tokens”, arXiv:2606.14160：<https://arxiv.org/abs/2606.14160>

可变形地面/软地面控制：

1. Lynch et al., “Efficient, Responsive, and Robust Hopping on Deformable Terrain”, arXiv:2311.18685：<https://arxiv.org/abs/2311.18685>

本地代码依据：

1. `botbrain_ws_aitech/botbrain_ws/src/g1_pkg/launch/fast_lio.launch.py`
2. `botbrain_ws_aitech/botbrain_ws/src/fast_lio/config/mid360.yaml`
3. `botbrain_ws_aitech/botbrain_ws/src/g1_pkg/launch/localization_3d.launch.py`
4. `botbrain_ws_aitech/botbrain_ws/src/open3d_loc/config/loc_param_g1.yaml`
5. `botbrain_ws_aitech/botbrain_ws/src/g1_pkg/config/nav2_params.yaml`
6. `botbrain_ws_aitech/botbrain_ws/src/g1_pkg/src/g1_write.cpp`
7. `botbrain_ws_aitech/botbrain_ws/src/g1_pkg/src/g1_driver/g1_driver.cpp`
