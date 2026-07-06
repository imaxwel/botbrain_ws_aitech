# G1 AMCL / Nav2 联调排障报告

## 1. 问题背景

本次联调场景为 G1 机器人在静态地图导航模式下，使用以下服务组合运行：

- `bringup`
- `state_machine`
- `localization`
- `navigation`

定位方案已从原工程中的 RTAB-Map 改为 AMCL + 静态栅格地图。

测试过程中主要遇到以下几类问题：

1. 在 Foxglove 中向 `/g1_robot/initialpose` 发送初始位姿时，日志出现：
   - `Failed to transform initial pose in time`
   - `Message Filter dropping message ... queue is full`
2. Nav2 节点虽然启动了进程，但一直停留在 `unconfigured`，无法接收导航目标。
3. 需要在 Foxglove 中直接发送导航目标，但工程内没有现成的 “topic -> NavigateToPose action” 桥接接口。
4. Nav2 运行后，控制链路已经能输出 `/g1_robot/cmd_vel_nav`，但日志中仍出现恢复行为和局部控制器相关警告。

本报告记录本次排障的分析过程、结论、最终保留的改动，以及已经尝试后回退的方案。

---

## 2. 问题一：`initialpose` 警告与队列满告警

### 2.1 现象

在 `/g1_robot/initialpose` 设置初始位姿后，`log/log7.txt` 中出现：

- `Failed to transform initial pose in time`
- `Message Filter dropping message ... discarding message because the queue is full`

相关日志可见：

- `log/log7.txt`
- `log/log6.txt`

### 2.2 分析结论

这两类日志不属于同一个根因。

#### A. `Failed to transform initial pose in time`

该警告的核心含义是：

- AMCL 在处理 `initialpose` 时，需要查询 `g1_robot/base_link -> g1_robot/odom` 的 TF
- Foxglove 发出的 `initialpose.header.stamp` 比当前 TF 缓存中的最新 TF 时间略超前了几十毫秒
- 因此产生了 `future extrapolation` 警告

但从日志可以确认，这不是“初始位姿设置失败”，因为警告后紧接着仍然出现了：

- `Setting pose (...)`

结论：

- 这是一个 TF 时间边界偏紧的警告
- 不等价于 AMCL 初始化失败
- 在当前调试阶段不是阻塞导航主流程的根因

#### B. `queue is full`

该警告出现在 AMCL 的 `Message Filter`，也出现在 `pointcloud_to_laserscan` 一侧。

结论：

- 它反映的是激光/点云消息在 TF 或滤波链路中短时堆积
- 启动阶段偶发少量出现，可以先不作为主问题处理
- 如果后续真实导航中持续刷屏，再单独做传感器链路优化

### 2.3 已尝试但已回退的方案

曾尝试对 `pointcloud_to_laserscan` 做以下调整：

- 增大 `queue_size`
- 设置 `target_frame`
- 调整 `scan_time`
- 提高 `transform_tolerance`

由于实际效果没有改善，这部分改动已全部回退，当前仓库未保留这些修改。

### 2.4 当前结论

这两类警告目前不作为主阻塞项继续追查。

优先级判断：

- `initialpose` 警告：低优先级
- `queue is full`：低到中优先级，视后续运行稳定性再定

---

## 3. 问题二：Nav2 节点无法激活，始终处于 `unconfigured`

### 3.1 现象

在启动 `navigation` 后检查：

```bash
ros2 lifecycle get /g1_robot/controller_server
ros2 lifecycle get /g1_robot/planner_server
ros2 lifecycle get /g1_robot/behavior_server
ros2 lifecycle get /g1_robot/bt_navigator
ros2 lifecycle get /g1_robot/waypoint_follower
```

最初全部返回：

```text
unconfigured [1]
```

同时：

```bash
ros2 action list | grep navigate
```

也看不到 `/g1_robot/navigate_to_pose`。

### 3.2 根因分析

根因有两层。

#### A. `state_machine` 仍然依赖旧的 RTAB-Map 导航链

在以下文件中：

- `botbrain_ws/src/bot_state_machine/config/navigation.json`

原有导航配置仍包含：

- `rtab_manager`
- `map_odom_node`

这是旧版 RTAB-Map 定位/导航链路的依赖。

而当前工程已经切换为：

- 静态地图
- `map_server`
- `amcl`
- `nav2`

因此状态机在 bringup Nav2 时，实际上沿着一个已经失效的依赖图在工作，导致导航链不能被正确拉起。

#### B. `navigation` 启动文件中，Nav2 自带 lifecycle manager 被注释掉了

在：

- `botbrain_ws/src/bot_navigation/launch/nav2.launch.py`

里虽然定义了 `lifecycle_manager_navigation`，但原本并没有加入 `LaunchDescription` 返回列表。

这意味着：

- Nav2 节点被启动了进程
- 但没有自动进入 `configure -> activate`
- 只能完全依赖外部 state machine 接管

一旦外部状态机链路不匹配，Nav2 就会长期停留在 `unconfigured`

### 3.3 解决方法

#### 修复 1：更新 state machine 的导航依赖图

将 `botbrain_ws/src/bot_state_machine/config/navigation.json` 改为当前 AMCL 场景对应的真实导航节点顺序：

- `controller_server`
- `smoother_server`
- `planner_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `nav2_utils`

同时为了方便以后恢复旧 RTAB-Map 模式，没有直接删除旧配置，而是将其保存在：

- `_disabled_legacy_nodes`

字段中。

这样处理的原因：

- JSON 本身不支持注释
- 保留在禁用字段中，既不会被状态机解析，也便于后续恢复

#### 修复 2：启用 Nav2 自带 lifecycle manager

在：

- `botbrain_ws/src/bot_navigation/launch/nav2.launch.py`

中启用了 `lifecycle_manager_navigation`，让 Nav2 能自行 autostart。

同时将：

- `use_sim_time = True`

改为：

- `use_sim_time = False`

因为当前运行环境是真实机器人而非仿真。

### 3.4 修复结果

修复后再检查：

```bash
ros2 action list | grep navigate
```

可以看到：

- `/g1_robot/navigate_through_poses`
- `/g1_robot/navigate_to_pose`

再检查 lifecycle：

```bash
ros2 lifecycle get /g1_robot/controller_server
ros2 lifecycle get /g1_robot/planner_server
ros2 lifecycle get /g1_robot/behavior_server
ros2 lifecycle get /g1_robot/bt_navigator
ros2 lifecycle get /g1_robot/waypoint_follower
```

全部返回：

```text
active [3]
```

结论：

- Nav2 已成功进入可接单状态
- 规划、控制、行为树和 waypoint follower 都已激活

---

## 4. 问题三：Foxglove 无法直接方便地发送导航目标

### 4.1 现象

工程本身基于 Nav2 标准 action：

- `/g1_robot/navigate_to_pose`

但在 Foxglove 中直接发送 action goal 并不一定方便，且工程中原本没有现成的“topic 转 action”桥。

### 4.2 解决方法

新增了一个桥接节点：

- `goal_pose_bridge.py`

功能：

- 订阅 `geometry_msgs/PoseStamped` 话题 `/g1_robot/goal_pose`
- 收到目标位姿后，转发为 Nav2 的 `NavigateToPose` action goal

### 4.3 新增的文件和接入点

新增：

- `botbrain_ws/src/bot_navigation/scripts/goal_pose_bridge.py`

接入：

- `botbrain_ws/src/bot_navigation/launch/nav_utils.launch.py`

同时更新安装和依赖：

- `botbrain_ws/src/bot_navigation/CMakeLists.txt`
- `botbrain_ws/src/bot_navigation/package.xml`

### 4.4 使用方式

Foxglove 中发送：

- 话题：`/g1_robot/goal_pose`
- 类型：`geometry_msgs/PoseStamped`

典型消息：

```json
{
  "header": {
    "stamp": { "sec": 0, "nanosec": 0 },
    "frame_id": "g1_robot/map"
  },
  "pose": {
    "position": { "x": -2.0, "y": 3.6, "z": 0.0 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  }
}
```

### 4.5 验证结果

从 `log/log9.txt` 可以看到：

- `goal_pose_bridge` 正常启动
- 收到了 `goal_pose`
- 成功转发到 `NavigateToPose`

日志证据：

- `Goal pose bridge ready: topic 'goal_pose' -> action 'navigate_to_pose'`
- `Forwarding goal_pose to NavigateToPose: ...`

结论：

- Foxglove -> `/goal_pose` -> Nav2 action 的桥接链路已打通

---

## 5. 问题四：Nav2 已输出 `cmd_vel_nav`，但控制器多次报 `Failed to make progress`

### 5.1 现象

在本次测试中：

- `ros2 topic echo /g1_robot/cmd_vel_nav` 持续有速度输出
- 但 `controller_server` 周期性报：
  - `Failed to make progress`
  - `[follow_path] [ActionServer] Aborting handle.`

### 5.2 结论

在当前这次测试中，这个现象本身是符合预期的。

原因：

- 本次明确说明机器人没有切到平衡模式
- 因此机器人不会真正执行速度命令
- Nav2 虽然在持续输出速度控制量，但里程计/位姿不会按预期发生可感知前进
- `progress_checker` 最终判定“没有进展”，于是中止当前跟踪任务

这意味着：

- 规划链路正常
- 控制链路正常
- `cmd_vel_nav` 输出正常
- 只是机器人执行层故意未允许运动

所以这类 `Failed to make progress` 在当前阶段不视为导航算法故障。

---

## 6. 问题五：Behavior Server 的恢复动作失败

### 6.1 现象

在 `controller_server` 因为“无前进进展”而进入恢复行为后，日志出现：

- `Running spin`
- `Running backup`
- `No Transform available Error looking up target frame: "odom"...`
- `Current robot pose is not available.`
- `Initial robot pose is not available.`

### 6.2 根因分析

`behavior_server` 使用的默认 frame 是裸：

- `odom`
- `map`
- `base_link`

但当前工程是 namespaced TF：

- `g1_robot/odom`
- `g1_robot/map`
- `g1_robot/base_link`

因此一旦恢复行为尝试做 TF 查询，就会找不到正确的 frame。

### 6.3 解决方法

在：

- `botbrain_ws/src/g1_pkg/config/nav2_params.yaml`

中补充 `behavior_server` 配置，显式指定：

- `local_frame: <prefix>odom`
- `global_frame: <prefix>map`
- `robot_base_frame: <prefix>base_link`

并同步补上 costmap/footprint 相关输入话题。

### 6.4 当前状态

该修复已经写入仓库，但尚需你同步到机器人并重启 `navigation` 后再次验证。

验证重点：

- recovery 触发后是否仍然报裸 `odom` 找不到
- `spin / backup / wait` 是否能正常进入

---

## 7. 问题六：MPPI 碰撞配置不一致警告

### 7.1 现象

日志中出现：

```text
Inconsistent configuration in collision checking. Please verify the robot's shape settings in both the costmap and the cost critic.
```

### 7.2 根因分析

在 G1 的 `nav2_params.yaml` 中：

- costmap 使用的是 footprint 多边形
- 但 MPPI 的 `CostCritic.consider_footprint` 原本为 `false`

这样就会导致：

- costmap 和控制器使用的碰撞模型不一致

### 7.3 解决方法

将：

- `consider_footprint: false`

改为：

- `consider_footprint: true`

### 7.4 当前状态

该修复已写入仓库。

同步并重启 `navigation` 后，应能消除这条警告，或者至少使碰撞模型与 costmap 语义一致。

---

## 8. 对 `log/log9.txt` 的总体结论

从 `log/log9.txt` 可以得出以下重要结论：

1. `goal_pose_bridge` 正常启动并成功转发目标点
2. `bt_navigator` 能正常接收导航目标
3. `controller_server` 能持续输出控制努力
4. `/g1_robot/cmd_vel_nav` 持续有数据，说明控制主链已经跑通
5. 当前“机器人不动”主要是因为没有进入平衡模式，这是测试设定使然
6. 当前最值得继续修的真实问题是：
   - `behavior_server` 的 TF frame 配置
   - MPPI 的 footprint 配置一致性

因此当前系统状态可以描述为：

- “导航软件主链已通”
- “执行层尚未允许运动”
- “恢复动作层还有一处命名空间 TF 配置需要补完验证”

---

## 9. 最终保留的改动

### 9.1 状态机导航依赖图

文件：

- `botbrain_ws/src/bot_state_machine/config/navigation.json`

内容：

- 将旧 RTAB-Map 导航链切换为当前 Nav2 实际节点链
- 保留旧节点于 `_disabled_legacy_nodes`

### 9.2 启用 Nav2 自管理 lifecycle

文件：

- `botbrain_ws/src/bot_navigation/launch/nav2.launch.py`

内容：

- `use_sim_time` 改为 `False`
- 启用 `lifecycle_manager_navigation`

### 9.3 新增 Foxglove 目标点桥接节点

文件：

- `botbrain_ws/src/bot_navigation/scripts/goal_pose_bridge.py`

配套接入文件：

- `botbrain_ws/src/bot_navigation/launch/nav_utils.launch.py`
- `botbrain_ws/src/bot_navigation/CMakeLists.txt`
- `botbrain_ws/src/bot_navigation/package.xml`

### 9.4 修复 G1 Nav2 参数

文件：

- `botbrain_ws/src/g1_pkg/config/nav2_params.yaml`

内容：

- `CostCritic.consider_footprint = true`
- 新增 `behavior_server` 的 namespaced frame 配置

---

## 10. 已尝试但未保留的改动

以下改动曾尝试，但由于无实际改善已回退：

- `botbrain_ws/src/g1_pkg/config/pointcloud_to_laserscan_params.yaml`
- `botbrain_ws/src/g1_pkg/launch/pc2ls.launch.py`

尝试内容包括：

- 调整 `queue_size`
- 设置 `target_frame`
- 修改 `scan_time`
- 修改 `transform_tolerance`

最终结论：

- 这些改动对当前主问题没有实质帮助
- 已全部恢复原状

---

## 11. 当前建议的验证顺序

在同步最新修改到机器人后，建议按以下顺序验证：

### 11.1 先验证 Nav2 仍然正常启动

```bash
ros2 action list | grep navigate
ros2 lifecycle get /g1_robot/controller_server
ros2 lifecycle get /g1_robot/planner_server
ros2 lifecycle get /g1_robot/behavior_server
ros2 lifecycle get /g1_robot/bt_navigator
ros2 lifecycle get /g1_robot/waypoint_follower
```

### 11.2 验证 `goal_pose_bridge`

```bash
docker compose logs -f navigation | grep goal_pose_bridge
ros2 node list | grep goal_pose_bridge
```

### 11.3 验证导航主链

在 Foxglove 发送 `/g1_robot/goal_pose` 后检查：

```bash
ros2 topic echo /g1_robot/cmd_vel_nav
docker compose logs -f navigation | grep -E "goal_pose_bridge|bt_navigator|controller_server"
```

### 11.4 验证恢复动作 TF 是否修复

当机器人后续进入平衡模式并真实可动后，若仍触发 recovery，观察：

```bash
docker compose logs -f navigation | grep -E "behavior_server|transformPoseInTargetFrame|odom"
```

若不再出现裸 `odom` 查找失败，说明 namespaced frame 配置已经生效。

---

## 12. 最终结论



1. `initialpose` 的 TF 警告和少量 queue full 不是当前导航主阻塞项。
2. Nav2 最初无法激活，根因是：
   - state machine 仍依赖旧 RTAB-Map 导航图
   - `nav2.launch.py` 中未启用 Nav2 自带 lifecycle manager
3. 修复后，Nav2 节点已全部进入 `active`，并能提供：
   - `/g1_robot/navigate_to_pose`
   - `/g1_robot/navigate_through_poses`
4. 为便于 Foxglove 发送目标点，新增了：
   - `/g1_robot/goal_pose` -> `/g1_robot/navigate_to_pose`
   的桥接节点
5. 从日志和 `/g1_robot/cmd_vel_nav` 的持续输出可确认：
   - 规划正常
   - 控制正常
   - 目标接收正常
6. 当前机器人不运动，主要因为测试时未启用平衡模式，这符合预期。
7. 当前尚需继续验证的剩余配置问题主要是：
   - `behavior_server` 的 namespaced TF recovery 配置
   - MPPI footprint 一致性修复后的运行效果

整体上，本次调试已经把系统推进到了：

- “AMCL + 静态地图 + Foxglove 发目标 + Nav2 输出速度指令”的完整可运行阶段。

