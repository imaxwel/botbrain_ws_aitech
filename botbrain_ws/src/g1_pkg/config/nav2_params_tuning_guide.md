# Nav2 参数调整指南 — G1 Robot

> 对应配置文件：`nav2_params.yaml`（同目录）  
> 调整前建议先备份：`cp nav2_params.yaml nav2_params.yaml.bak`

---

## 目录

1. [速度与运动限制](#1-速度与运动限制)
2. [目标到达精度](#2-目标到达精度)
3. [Costmap — 障碍物检测](#3-costmap--障碍物检测)
4. [Costmap — 膨胀层](#4-costmap--膨胀层)
5. [路径规划器](#5-路径规划器)
6. [MPPI Critics 行为权重](#6-mppi-critics-行为权重)
7. [机器人外形 Footprint](#7-机器人外形-footprint)
8. [局部地图尺寸](#8-局部地图尺寸)
9. [在线热调参数](#9-在线热调参数)
10. [按症状快速定位](#10-按症状快速定位)

---

## 1. 速度与运动限制

**位置**：`controller_server → FollowPath`（MPPI Controller）

```yaml
vx_max: 0.50      # 最大前进速度 (m/s)
vx_min: -0.30     # 最大后退速度 (m/s)
vy_max: 0.15      # Omni 横移速度，正负对称 (m/s)
wz_max: 0.80      # 最大角速度 (rad/s)
```

| 场景 | 建议调整 |
|------|---------|
| 空旷大厅，想走更快 | 当前上限为 `vx_max: 0.50`；继续提高前必须重新验收定位和制动 |
| 狭窄走廊，需要更稳 | `vx_max: 0.25 ~ 0.30`，`wz_max: 0.8` |
| 横移过多、路径摆动 | 降低 `vy_max`、`vy_std` |
| 转弯时身体倾斜明显 | 降低 `wz_max` |

> 当前 Humble MPPI 不读取 `ax_max/ax_min/ay_max/ay_min/az_max/vy_min`。速度链为：MPPI 输出 `/g1_robot/cmd_vel_nav_raw`，连续性节点用 EMA 抑制抖动并保持最长 `0.18s` 的消息缺口，再交给 20 Hz `velocity_smoother` 发布 `/g1_robot/cmd_vel_nav`。MPPI 明确发布零速、命令超过保持窗口或定位安全停止时都不会继续保持旧速度。

---

## 2. 目标到达精度

**位置**：`controller_server → general_goal_checker`

```yaml
xy_goal_tolerance: 0.25   # 位置容差 (m)
yaw_goal_tolerance: 0.50  # 朝向容差 (rad, ≈ 29°)
```

| 场景 | 建议调整 |
|------|---------|
| 精确对接（充电桩、货架） | `xy_goal_tolerance: 0.03 ~ 0.05` |
| 一般导航点（宽松） | `xy_goal_tolerance: 0.20 ~ 0.25` |
| 不需要特定朝向 | `yaw_goal_tolerance: 6.28`（关闭朝向检查） |

---

## 3. Costmap — 障碍物检测

**位置**：

- `global_costmap`：`static_layer + obstacle_layer + denoise_layer + inflation_layer`，动态层只接 `/scan`，不要接原始 PointCloud2。
- `local_costmap`：`obstacle_layer + denoise_layer + inflation_layer`，输入 `/scan`，窗口比全局动态层更适合实时避障。
- `/scan` 来源：`pointcloud_to_laserscan` 从 `/cloud_registered_body_1` 转到 `<robot_name>/base_footprint` 后过滤生成。

```yaml
obstacle_min_range: 0.45    # 近身回波不标记为障碍，避免腿部/机身/地面噪点
obstacle_max_range: 3.0     # 标记障碍物的最大距离 (m)
raytrace_min_range: 0.20    # 近距离开始允许清除
raytrace_max_range: 4.5     # 清除旧障碍的光线追踪距离 (m)
inf_is_valid: true          # /scan 的 Inf 空方向也作为清除射线
expected_update_rate: 1.0  # 允许短暂 TF/guard 恢复；持续断流仍会停车
observation_persistence: 0.0  # 不缓存旧观测；已标记格子仍依赖 raytrace 清除
```

| 场景 | 建议调整 |
|------|---------|
| 机器人脚边/附近大片发黑 | 增大 `/scan range_min` 或 `obstacle_min_range`，例如 `0.50` |
| 地面点干扰导航 | 提高 `pointcloud_to_laserscan.min_height`，例如 `0.25` |
| 检测不到低矮障碍物 | 降低 `pointcloud_to_laserscan.min_height`，例如 `0.10~0.15` |
| 动态障碍（行人）残影残留 | 保持 `observation_persistence: 0.0`，检查 `/scan` 是否有 `Inf` 空方向以及 TF 是否丢帧 |
| 地图出现幽灵障碍物 | 确认 `raytrace_max_range ≥ obstacle_max_range`，global/local 都使用 `/scan` 清除，不要接原始 PointCloud2 |

---

## 4. Costmap — 膨胀层

**位置**：`global_costmap` 和 `local_costmap` 的 `inflation_layer`

```yaml
global_costmap:
  footprint_padding: 0.0
  inflation_radius: 0.29
  cost_scaling_factor: 60.0
local_costmap:
  inflation_radius: 0.35
  cost_scaling_factor: 10.0
```

| 场景 | 建议调整 |
|------|---------|
| 走廊太窄，机器人频繁因路径代价过高而停止 | 先提高 `cost_scaling_factor`；不要把 `inflation_radius` 降到 footprint 外接半径以下 |
| 机器人经常靠近墙壁/障碍物 | `inflation_radius: 0.5 ~ 0.6` |
| 想让机器人优先走中间通道 | `cost_scaling_factor: 5.0`（衰减更慢，远离障碍更强） |
| 机器人绕路太多 | 提高 `cost_scaling_factor`（衰减更快，软代价带更窄） |

> global footprint 不额外 padding 时外接半径约 `0.285m`，因此 `inflation_radius=0.29m` 仍覆盖机器人几何足迹。较高的 `cost_scaling_factor=60.0` 只缩窄软代价带，不降低硬碰撞边界；local 仍使用 `0.02m/0.35m` 做实时动态保护。SmacPlanner2D 主要依赖膨胀栅格保护中心路径，local MPPI 再用多边形 footprint 做精确碰撞检查。

---

## 5. 路径规划器

**位置**：`planner_server → GridBased`（SmacPlanner2D）

```yaml
cost_travel_multiplier: 3.0   # 保留避障代价，但不允许它压倒实际路程
tolerance: 0.15               # 与实际到点能力协调的搜索容差 (m)
use_final_approach_orientation: true  # 终点朝向采用最后一段路径方向
allow_unknown: false          # 成品室内地图禁止穿越未知区
```

| 场景 | 建议调整 |
|------|---------|
| 路径绕远，明显走了不必要的大弯 | 保持 `cost_travel_multiplier: 2.0 ~ 5.0`，检查 global costmap |
| 机器人总是贴着障碍物走，想让它更保守 | 优先调整 inflation；不要把该权重直接放大到 `15+` |
| 地图边缘存在漏口 | 保持 `allow_unknown: false` 并在 PGM 中补虚拟墙 |
| 目标点在障碍物附近，规划失败 | 先修地图/移动点位；导航点不建议把 `tolerance` 放大到 `0.3` 以上 |

当前行为树以 2 Hz 检查全局路径，但不会无条件换路：新目标立即规划；已有路径第一次无效后继续执行局部控制，经过 `0.75s` 仍无效才重新规划。纯计时 `Delay` 不发布零速，因此瞬时 `/scan` 标记不会把一条正在执行的短路径立即改成地图级绕行，也不会为了确认障碍主动停车。

---

## 6. MPPI Critics 行为权重

**位置**：`controller_server → FollowPath → critics`

每个 Critic 有独立的 `cost_weight`，控制 MPPI 采样时对该因素的偏好强度。

```yaml
PathAlignCritic:
  cost_weight: 12.0      # 路径跟随紧密度，降低转弯处 stop-turn-go
CostCritic:
  cost_weight: 6.0       # 障碍物代价惩罚
GoalCritic:
  cost_weight: 5.0       # 趋向目标的驱动力
GoalAngleCritic:
  cost_weight: 2.0       # 到达目标时对准朝向
PreferForwardCritic:
  cost_weight: 1.0       # 正常控制不过度压制 Omni 转弯/横移
PathFollowCritic:
  cost_weight: 5.0       # 跟踪路径上最近点
PathAngleCritic:
  cost_weight: 2.0       # 对齐路径切线方向
```

| 现象 | 建议调整 |
|------|---------|
| 机器人频繁偏离规划路径 | 提高 `PathAlignCritic.cost_weight: 20.0` |
| 机器人走路抖动、蛇形前进 | 降低 `PathAngleCritic.cost_weight: 1.0` |
| 接近目标时不对准朝向 | 提高 `GoalAngleCritic.cost_weight: 5.0` |
| 机器人经常原地旋转而不前进 | 提高 `PreferForwardCritic.cost_weight: 8.0` |
| 机器人过于靠近障碍物 | 提高 `CostCritic.cost_weight: 6.0` |

正常 MPPI 连续失败后，行为树会先调用 `FollowPathFallback` 最长 1 秒：预测视野约 `1.25s`、前进速度上限 `0.20m/s`、禁止后退并加强前进方向偏置。它仍失败后才清 local costmap 并等待，不自动 spin/backup。

> 💡 调整 Critic 权重时，建议每次只改一个，步长不超过原值的 50%，观察效果后再调下一个。

---

## 7. 机器人外形 Footprint

**位置**：`global_costmap` 和 `local_costmap`。两处 footprint 几何尺寸保持一致，padding 按职责分层。

```yaml
# 当前：35cm(x) × 45cm(y) 的矩形，两处 footprint 相同
footprint: "[[0.175, 0.225], [0.175, -0.225], [-0.175, -0.225], [-0.175, 0.225]]"
global footprint_padding: 0.0   # 不额外扩大全局规划占用
local footprint_padding: 0.02   # 保留实时控制安全边距
```

格式说明：`[[x1,y1], [x2,y2], ...]`，坐标以 `base_footprint` 为原点，单位 m。

| 场景 | 建议调整 |
|------|---------|
| 实测 G1 尺寸与默认不符 | 按实际尺寸重新测量并修改四个顶点坐标 |
| 定位精度差，经常碰到障碍物 | `footprint_padding: 0.05 ~ 0.08` |
| 通道太窄无法通过但实际能过 | `footprint_padding: 0.0` |
| 机器人手臂展开时宽度更大 | 增大 y 方向顶点值 |

---

## 8. 局部地图尺寸

**位置**：`local_costmap`

```yaml
width: 4            # 局部地图宽度 (m)
height: 4           # 局部地图高度 (m)
resolution: 0.05    # 栅格分辨率 (m/格)
update_frequency: 10.0   # 更新频率 (Hz)
publish_frequency: 5.0   # 发布频率 (Hz)
```

| 场景 | 建议调整 |
|------|---------|
| 速度较快（>0.5m/s），看不到前方障碍 | `width: 6`，`height: 6` |
| CPU/内存不足，计算延迟高 | `width: 3`，`height: 3`，`update_frequency: 5.0` |
| 局部路径规划不够精细 | `resolution: 0.03`（更精细，计算量增加） |

---

## 9. 在线热调参数

无需重启节点，直接用命令行实时修改，适合调参阶段：

```bash
# 查看所有可调参数
ros2 param list /g1_robot/controller_server

# 修改最大速度
ros2 param set /g1_robot/controller_server FollowPath.vx_max 0.50

# 修改膨胀半径（需要重新触发costmap更新）
ros2 param set /g1_robot/global_costmap/global_costmap inflation_layer.inflation_radius 0.29

# 修改 Critic 权重
ros2 param set /g1_robot/controller_server FollowPath.PathAlignCritic.cost_weight 18.0
```

> ⚠️ `ros2 param set` 的修改在节点重启后会恢复到 yaml 文件值。确认效果满意后，**同步更新 `nav2_params.yaml`**。

---

## 10. 按症状快速定位

| 症状 | 首先检查 | 调整方向 |
|------|---------|---------|
| 走廊通不过，规划失败 | `cost_scaling_factor`、静态地图噪点 | 先提高衰减系数并修图，保持 global `inflation_radius >= 0.29` |
| 路径绕了很大的弯 | `allow_unknown`、`cost_travel_multiplier`、PGM 边界 | 禁止未知区、权重保持 2~5，并补虚拟墙 |
| 速度太慢 | `vx_max` | 当前为 0.50；不要继续提速，先检查路径、控制周期和安全停止 |
| 走几步就短暂停止，但 safety stop 没触发 | `cmd_vel_nav_raw/filtered/nav`、fallback 日志 | 区分 MPPI 明确零速、短时断流、滤波输出和局部降级触发 |
| 启动/停止太猛，机器人不稳 | velocity smoother | 调整 smoother 的加减速度，不要添加 MPPI 不读取的参数 |
| 路径还在但机器人突然停止 | BT action timeout、`/scan`、TF | 检查 `default_server_timeout`、扫描新鲜度和 FAST-LIO guard，不要只调 goal tolerance |
| 到达目标后位置偏差大 | planner `tolerance` + `xy_goal_tolerance` | 两者都要降低，否则误差可叠加 |
| 地图上有幽灵障碍物 | `observation_persistence` | 设为 0.0 |
| 行人走过后障碍残留 | `/scan` 清除射线、TF、`observation_persistence` | 确保 `use_inf/inf_is_valid=true`、TF 不丢帧，并保持 0.0 |
| 路径跟随偏差大、蛇形 | `PathAlignCritic.cost_weight` | 提高到 18~22 |
| XY 已到但接近目标时频繁旋转 | 最终路径朝向、`yaw_goal_tolerance` | 保持 `use_final_approach_orientation=true`；一般导航点使用 `0.50rad`，精确对接另设专用点位/参数 |
| Recovery 行为频繁触发 | `failure_tolerance`、costmap/TF 延迟 | 当前为 1.0s；先查服务端错误，不要无限放宽 |

---

*最后更新：2026-08-07*
