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
vx_max: 0.35      # 最大前进速度 (m/s)
vx_min: 0.0       # 最小前进速度，0 = 允许停止
wz_max: 1.0       # 最大角速度 (rad/s)
ax_max: 0.4       # 最大加速度 (m/s²)
ax_min: -0.5      # 最大制动加速度（负值）
az_max: 2.5       # 最大角加速度 (rad/s²)
```

| 场景 | 建议调整 |
|------|---------|
| 空旷大厅，想走更快 | `vx_max: 0.5 ~ 0.8` |
| 狭窄走廊，需要更稳 | `vx_max: 0.20`，`wz_max: 0.8` |
| 启动/制动太猛，机器人不稳 | `ax_max: 0.25`，`ax_min: -0.3` |
| 转弯时身体倾斜明显 | `az_max: 1.5`，`wz_max: 0.8` |

> ⚠️ G1 人形机器人建议 `vx_max ≤ 0.5`，`ax_max ≤ 0.5`，防止步态不稳定

---

## 2. 目标到达精度

**位置**：`controller_server → general_goal_checker`

```yaml
xy_goal_tolerance: 0.15   # 位置容差 (m)
yaw_goal_tolerance: 0.25  # 朝向容差 (rad, ≈ 14.3°)
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
inflation_radius: 0.35    # 覆盖 G1 padding 后约 0.34m 的外接半径
cost_scaling_factor: 12.0 # 衰减更快，减少中低代价格形成的宽黑圈
```

| 场景 | 建议调整 |
|------|---------|
| 走廊太窄，机器人频繁因路径代价过高而停止 | 先提高 `cost_scaling_factor`；不要把 `inflation_radius` 降到 footprint 外接半径以下 |
| 机器人经常靠近墙壁/障碍物 | `inflation_radius: 0.5 ~ 0.6` |
| 想让机器人优先走中间通道 | `cost_scaling_factor: 5.0`（衰减更慢，远离障碍更强） |
| 机器人绕路太多 | `cost_scaling_factor: 15.0`（衰减更快，近障碍代价低） |

> 对非圆形 footprint，`inflation_radius` 应覆盖机器人外接半径。窄通道优先调整 `cost_scaling_factor`，避免 SmacPlanner 失去可靠的 footprint 碰撞检查范围。

---

## 5. 路径规划器

**位置**：`planner_server → GridBased`（SmacPlanner2D）

```yaml
cost_travel_multiplier: 30.0  # 路径代价权重（越高越避开高代价区域）
tolerance: 0.1                # 目标不可达时的搜索容差 (m)
allow_unknown: false          # 是否允许规划穿越未知区域
```

| 场景 | 建议调整 |
|------|---------|
| 路径绕远，明显走了不必要的大弯 | `cost_travel_multiplier: 15.0 ~ 20.0` |
| 机器人总是贴着障碍物走，想让它更保守 | `cost_travel_multiplier: 50.0` |
| 地图不完整（边缘有未知区域），目标不可达 | `allow_unknown: true` |
| 目标点在障碍物附近，规划失败 | 先修地图/移动点位；导航点不建议把 `tolerance` 放大到 `0.3` 以上 |

---

## 6. MPPI Critics 行为权重

**位置**：`controller_server → FollowPath → critics`

每个 Critic 有独立的 `cost_weight`，控制 MPPI 采样时对该因素的偏好强度。

```yaml
PathAlignCritic:
  cost_weight: 14.0      # 路径跟随紧密度
CostCritic:
  cost_weight: 3.81      # 障碍物代价惩罚
GoalCritic:
  cost_weight: 5.0       # 趋向目标的驱动力
GoalAngleCritic:
  cost_weight: 3.0       # 到达目标时对准朝向
PreferForwardCritic:
  cost_weight: 5.0       # 偏好前向运动（减少倒退/原地旋转）
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

> 💡 调整 Critic 权重时，建议每次只改一个，步长不超过原值的 50%，观察效果后再调下一个

---

## 7. 机器人外形 Footprint

**位置**：`global_costmap` 和 `local_costmap`（两处需同步修改）

```yaml
# 当前：35cm(x) × 45cm(y) 的矩形
footprint: "[[0.175, 0.225], [0.175, -0.225], [-0.175, -0.225], [-0.175, 0.225]]"
footprint_padding: 0.02   # 额外安全边距 (m)
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
ros2 param set /g1_robot/controller_server FollowPath.vx_max 0.5

# 修改膨胀半径（需要重新触发costmap更新）
ros2 param set /g1_robot/global_costmap/global_costmap inflation_layer.inflation_radius 0.35

# 修改 Critic 权重
ros2 param set /g1_robot/controller_server FollowPath.PathAlignCritic.cost_weight 18.0
```

> ⚠️ `ros2 param set` 的修改在节点重启后会恢复到 yaml 文件值。确认效果满意后，**同步更新 `nav2_params.yaml`**。

---

## 10. 按症状快速定位

| 症状 | 首先检查 | 调整方向 |
|------|---------|---------|
| 走廊通不过，规划失败 | `cost_scaling_factor`、静态地图噪点 | 先提高衰减系数并修图，保持 `inflation_radius >= 0.35` |
| 路径绕了很大的弯 | `cost_travel_multiplier` | 降低到 15~20 |
| 速度太慢 | `vx_max` | 提高到 0.5 |
| 启动/停止太猛，机器人不稳 | `ax_max`、`ax_min` | 降低绝对值 |
| 路径还在但机器人突然停止 | BT action timeout、`/scan`、TF | 检查 `default_server_timeout`、扫描新鲜度和 FAST-LIO guard，不要只调 goal tolerance |
| 到达目标后位置偏差大 | planner `tolerance` + `xy_goal_tolerance` | 两者都要降低，否则误差可叠加 |
| 地图上有幽灵障碍物 | `observation_persistence` | 设为 0.0 |
| 行人走过后障碍残留 | `/scan` 清除射线、TF、`observation_persistence` | 确保 `use_inf/inf_is_valid=true`、TF 不丢帧，并保持 0.0 |
| 路径跟随偏差大、蛇形 | `PathAlignCritic.cost_weight` | 提高到 18~22 |
| 接近目标时频繁旋转 | `GoalAngleCritic.threshold_to_consider` | 降低到 0.3 |
| Recovery 行为频繁触发 | `failure_tolerance` | 提高到 0.5 |

---

*最后更新：2026-06-23*
