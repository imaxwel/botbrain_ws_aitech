# G1Pilot 上身控制能力迁移到 BotBrain 的深度研究分析报告

## 1. 结论摘要

### 1.1 最终结论

**可以迁移，而且具备较高工程可行性，但不建议“整包照搬 g1pilot”到 BotBrain。**

最优路线不是把 g1pilot 直接并入 BotBrain 的导航与控制主链，而是：

- **保留 BotBrain 现有导航/状态机/速度仲裁/前端框架不动**；
- **抽取 g1pilot 的上身实时控制能力**（IK、目标位姿接口、低层 DDS 臂控制、夹爪控制）；
- 在 BotBrain 中新增一个 **G1 Manipulation 子系统**，以“**上身独立控制、下身继续走官方平衡控制**”的方式接入。

### 1.2 可行性判断

从代码结构与底层控制边界看：

- **BotBrain 下身控制**：本质是 `cmd_vel_out -> g1_pkg/robot_write_node -> Unitree LocoClient`，完全依赖官方 locomotion/balance controller；
- **g1pilot 下身控制**：也是把步行和平衡交给官方控制器；
- **g1pilot 上身控制**：通过 `rt/arm_sdk` 发布 `LowCmd`，对上身关节做连续低层控制；
- **BotBrain 当前上身控制**：已经具备低层 DDS 通道，但只实现了“姿态保存/读取/回放”，并未实现连续实时笛卡尔控制或 IK 闭环。

因此二者并不是路线冲突，而是：

> BotBrain 已经具备一部分低层基础设施，g1pilot 则补齐了“连续式上身控制算法与交互层”。

### 1.3 对“无缝衔接导航”的判断

**可以做到与 BotBrain 导航策略无缝衔接**，前提是明确控制边界：

- **导航仍只产生底盘速度命令**；
- **上身控制节点绝不接管底盘运动控制权**；
- **上身控制与导航之间只共享状态，不共享速度输出口**；
- **通过状态机/任务调度实现“移动中允许控臂”与“某些抓取动作要求停步”两种模式切换**。

换言之，迁移成功的关键不是 Nav2，而是**控制权仲裁与部署隔离**。

---

## 2. 本次研究范围与依据

本次分析基于两个工程的实际代码与启动结构：

### 2.1 BotBrain_version 侧重点

重点查看了：

- `botbrain_ws/src/g1_pkg`
- `botbrain_ws/src/bot_bringup`
- `botbrain_ws/src/bot_navigation`
- `botbrain_ws/src/bot_state_machine`
- `botbrain_ws/src/bot_custom_interfaces`

### 2.2 g1pilot 侧重点

重点查看了：

- `g1pilot/manipulation/arm_controller.py`
- `g1pilot/manipulation/interactive_marker.py`
- `g1pilot/manipulation/dx3_hand.py`
- `g1pilot/navigation/loco_client.py`
- `g1pilot/navigation/nav2point.py`
- `g1pilot/teleoperation/joy_mux.py`
- `g1pilot/state/robot_state.py`
- `launch/*.launch.py`
- `docker/Dockerfile`
- `config/config.yaml`

---

## 3. 两个工程的控制架构对比

## 3.1 BotBrain 的 G1 控制架构

### 3.1.1 底盘/导航主链

BotBrain 的 G1 运动主链是典型的分层架构：

1. `bot_navigation` 输出导航速度；
2. `bot_bringup` 中的 `twist_mux` 做速度优先级仲裁；
3. 最终输出 `cmd_vel_out`；
4. `g1_pkg` 的 `robot_write_node` 订阅 `cmd_vel_out`；
5. `G1Driver::move()` 调用 Unitree 官方 `LocoClient::Move()`。

这条链路说明：

- BotBrain 的导航栈与底盘控制之间耦合度低；
- G1 的步态稳定、平衡、动态跟随都交给 Unitree 官方控制器；
- BotBrain 自身只负责速度来源管理，不负责步态求解。

### 3.1.2 BotBrain 的速度控制优势

BotBrain 的底盘控制框架有几个明显优势：

- `twist_mux` 已经形成稳定的优先级体系；
- 导航、网页摇杆、实体手柄、AI 指令都有统一入口；
- `dead_man_switch` 和零速度基线已经内建；
- 状态机 `bot_state_machine` 能控制生命周期与自动回退。

这意味着：

**后续上身控制迁移时，没有必要碰 BotBrain 的导航主链。**

### 3.1.3 BotBrain 当前上身控制能力

BotBrain 的 `g1_pkg` 并不是完全没有手臂能力。当前它已经具备：

- 初始化低层 DDS 通道：`rt/arm_sdk` + `rt/lowstate`；
- 读取上身当前关节角；
- 通过 `arm_cmd` 服务完成：
  - 保存当前 pose；
  - 从文件读取 pose；
  - 执行 pose；
  - 释放手臂权重；
  - 删除 pose。

但它的能力边界也非常明确：

- 当前控制对象只有：
  - `waist_yaw`
  - 左臂 5 轴：`shoulder_pitch/roll/yaw + elbow + wrist_roll`
  - 右臂 5 轴：同上
- **没有实现连续笛卡尔空间目标跟踪**；
- **没有 IK 求解器**；
- **没有实时位姿流接口**；
- **没有抓取前的目标/轨迹管理**；
- 本质上是“姿态库回放器”，不是“操作臂控制器”。

### 3.1.4 BotBrain 当前 G1 模型的限制

从当前实现可以看出，BotBrain 的 G1 上身控制不是完整 29DoF 操作模型：

- 它没有把 `waist_roll / waist_pitch / wrist_pitch / wrist_yaw` 纳入当前手臂控制主逻辑；
- `robot_read_node` 文档中也以 23 joints 为主；
- 当前 `UpperBodyPose` 语义偏“预定义姿态”，而非完整操作空间控制。

这会直接影响迁移方式：

> 如果目标是给后续视觉抓取铺路，那么只保留 BotBrain 现在的 pose 回放能力是不够的，必须引入类似 g1pilot 的连续控制链。

---

## 3.2 g1pilot 的控制架构

### 3.2.1 控制边界

g1pilot 的核心思想非常明确：

- **下身：官方 locomotion controller**
- **上身：自定义低层控制**

这和你希望在 BotBrain 中达成的目标高度一致。

### 3.2.2 g1pilot 下身链路

g1pilot 的 `loco_client.py`：

- 使用 `unitree_sdk2py.g1.loco.g1_loco_client.LocoClient`；
- 负责 FSM 切换、平衡启动、移动命令、急停；
- 接收手柄或自主导航转换来的 Joy 风格控制输入；
- 实际仍调用官方步行/平衡接口。

也就是说，g1pilot 并未“自写腿部控制器”，而是与 BotBrain 一样站在官方 locomotion 之上。

### 3.2.3 g1pilot 上身链路

g1pilot 的 `arm_controller.py` 是关键价值所在：

- 通过 `rt/arm_sdk` 发送 `LowCmd`；
- 通过 `rt/lowstate` 读取当前关节状态；
- 内部集成 Pinocchio IK 求解器；
- 接受左右手的 `PoseStamped` 目标；
- 执行目标过滤、方向步进限制、自动标定、速度限制、平滑控制；
- 以高频循环输出连续关节目标；
- 可以在 `arms_enabled=True` 时持续控制上身，而不接管底盘。

这正是 BotBrain 所缺失的能力。

### 3.2.4 g1pilot 对导航与控臂的并行模式

g1pilot 中导航并不是直接调用 Nav2 输出 `cmd_vel`，而是采用：

- 路径规划/目标生成；
- 再转换为 Joy 风格控制或 locomotion 命令；
- `joy_mux` 在“自主”和“手动”之间切换；
- `loco_client` 输出给官方 locomotion controller。

这一套导航实现**不需要整体迁入 BotBrain**，因为 BotBrain 已经有更成熟的 Nav2 + `twist_mux` + 状态机体系。

所以 g1pilot 真正值得迁入的，不是它的导航，而是：

- `arm_controller` 的控制思想；
- IK/滤波/限幅逻辑；
- `interactive_marker` / UI 控臂接口语义；
- DX3 手爪控制节点。

---

## 4. 真正的差异点：BotBrain 缺的不是“能发低层命令”，而是“实时操作层”

这一点非常关键。

BotBrain 与 g1pilot 在低层上并不是完全不同：

- 两边都能接触 Unitree 官方 locomotion 接口；
- 两边都能读低层状态；
- 两边都能写 `rt/arm_sdk`；
- 两边都遵循“下身官方、上身自控”的潜在模式。

差别在于：

### BotBrain 当前有：

- 生命周期管理；
- 导航和速度仲裁；
- 预定义姿态回放；
- G1 基础接口抽象。

### g1pilot 当前有：

- 连续操作空间控制；
- 实时 IK；
- 目标姿态流输入；
- 手爪控制；
- 操作界面与交互 marker。

因此迁移本质不是“推翻 BotBrain 的 G1 实现”，而是：

> 把 g1pilot 的“Manipulation Runtime”移植到 BotBrain 的 `g1_pkg` 或独立 `g1_manipulation_pkg` 中。

---

## 5. 依赖与冲突的全方面分析

## 5.1 依赖冲突总评

总体判断：

- **可以避免绝大部分冲突**；
- 真正麻烦的冲突主要集中在：
  - DDS/SDK 栈差异；
  - Python/C++ 混合部署；
  - 关节定义与模型差异；
  - 运行时双写 `rt/arm_sdk` 的控制权冲突。

### 风险等级划分

- **低风险**：BotBrain 导航链路兼容性
- **中风险**：ROS topic / namespace / state machine 接入
- **中高风险**：IK 依赖、Pinocchio、Python 运行时部署
- **高风险**：两个节点同时写上身 DDS、关节模型不一致、实时性不足

---

## 5.2 SDK / DDS 依赖差异

### 5.2.1 BotBrain 使用的 SDK 形态

BotBrain 的 `g1_pkg` 是 **C++ 版 `unitree_sdk2`**：

- `unitree::robot::g1::LocoClient`
- `ChannelPublisher/ChannelSubscriber`
- 依赖本机 `UNITREE_SDK2_ROOT`、动态库、`LD_LIBRARY_PATH`、`LD_PRELOAD`

### 5.2.2 g1pilot 使用的 SDK 形态

g1pilot 的 `arm_controller.py` 与 `loco_client.py` 使用 **Python 版 `unitree_sdk2py`**：

- `unitree_sdk2py.g1.loco.g1_loco_client`
- `unitree_sdk2py.core.channel`
- `LowCmd_ / LowState_` Python IDL

### 5.2.3 是否会冲突

**不会天然冲突，但不建议在同一控制面混合使用两套 SDK 去做同一种职责。**

推荐做法：

- **底盘 locomotion 继续保留 BotBrain C++ `g1_pkg`**；
- **上身 manipulation 初期可以独立使用 g1pilot 的 Python `unitree_sdk2py`**；
- 后续若要极致稳定与发布一致性，再考虑把上身控制重写为 BotBrain C++ 版本。

### 5.2.4 关键风险

如果同一时刻：

- BotBrain `g1_pkg` 正在写 `rt/arm_sdk`；
- 新迁入的 manipulation node 也在写 `rt/arm_sdk`；

就会出现**命令覆盖、抖动、关节跳变、权重控制互相抢写**。

这是迁移中的**第一风险项**。

### 5.2.5 规避方式

必须保证：

- `rt/arm_sdk` 在运行时只有一个“主写者”；
- BotBrain 原来的 `arm_cmd` 回放逻辑在启用实时操控时要么禁用，要么退化为调用新控制器接口；
- 最终系统内必须有明确的“手臂控制主节点”。

---

## 5.3 机器人模型与关节维度差异

### 5.3.1 BotBrain 当前关节视图

BotBrain 当前上身控制只覆盖：

- 1 个腰部关节：`waist_yaw`
- 双臂各 5 个关节
- 总体是 11 维上身控制语义

### 5.3.2 g1pilot 当前关节视图

g1pilot 使用的是更完整的 29 DoF G1 关节映射，包含：

- 腿部 12 轴
- 腰部 3 轴
- 左臂 7 轴
- 右臂 7 轴

### 5.3.3 冲突判断

这不是简单的 topic remap 能解决的问题，而是：

- **控制模型抽象不同**；
- **URDF/IK/关节上限定义不同**；
- **BotBrain 当前接口不足以承接完整 7DoF + 3DoF 腰部操作语义**。

### 5.3.4 工程含义

如果你要为后续视觉抓取铺路，我建议不要只迁 5 轴 arm pose 逻辑，而是直接把目标定为：

- 双臂 7DoF
- 至少保留 waist yaw
- 预留 waist roll/pitch 接口
- 夹爪独立控制

否则后续抓取、避障伸臂、手腕姿态对齐都会受限。

---

## 5.4 IK 及数学依赖冲突

### 5.4.1 g1pilot 的新增依赖

g1pilot 的操作层依赖显著多于 BotBrain 当前 `g1_pkg`：

- `pinocchio`
- `meshcat`（可选）
- `interactive_markers`
- `tf2_geometry_msgs`
- 可能还依赖 PyQt / RViz 交互组件
- 额外的 joint limit / IK helper / solver 模块

### 5.4.2 是否必须全搬

**不需要。**

真正必须迁的核心依赖是：

- `pinocchio`
- 与目标位姿输入相关的 ROS 消息依赖
- `interactive_markers`（仅当你要保留 RViz 控制）
- `tf2_ros` / `tf2_geometry_msgs`

而下面这些不必作为首批必须项：

- PyQt GUI
- meshcat
- 自定义桌面 UI
- g1pilot 导航器相关模块

### 5.4.3 建议

把迁移分成两层：

1. **运行时必需依赖**：用于机器人实体操控；
2. **开发/调试依赖**：用于 RViz、桌面调试、标定。

这样能降低 BotBrain 主系统的部署复杂度。

---

## 5.5 ROS 接口与命名空间冲突

### 5.5.1 BotBrain 的命名方式

BotBrain 全面 namespace 化，例如：

- `/{robot_name}/cmd_vel_out`
- `/{robot_name}/mode`
- `/{robot_name}/arm_cmd`

### 5.5.2 g1pilot 的命名方式

g1pilot 大量使用固定命名：

- `/g1pilot/hand_goal/right`
- `/g1pilot/arms/enabled`
- `/g1pilot/arms/home`
- `/g1pilot/dx3/hand_action/right`

### 5.5.3 冲突判断

这类冲突**非常容易修复**，本质属于接口重命名问题，不是架构问题。

建议迁移时统一改为 BotBrain 风格：

- `/{robot_name}/manipulation/hand_goal/right`
- `/{robot_name}/manipulation/hand_goal/left`
- `/{robot_name}/manipulation/enabled`
- `/{robot_name}/manipulation/home`
- `/{robot_name}/gripper/right/command`
- `/{robot_name}/gripper/left/command`

这样能避免：

- 多机器人场景冲突；
- 前端订阅混乱；
- 状态机管理节点歧义。

---

## 5.6 生命周期与状态机冲突

### 5.6.1 BotBrain 的优势

BotBrain 已经有一套很成熟的生命周期系统：

- 核心节点由 `bot_state_machine` 管理；
- 节点分类明确：`core / navigation / accessories / payload / camera / ia_stack`；
- 导航失败可自动回退到 TELEOP。

### 5.6.2 g1pilot 的现状

g1pilot 大多数节点是普通 ROS2 Node，不是 lifecycle node。

### 5.6.3 冲突判断

这意味着如果直接把 g1pilot 节点原封不动接入 BotBrain：

- 状态机看不到生命周期；
- 无法统一 configure / activate / deactivate；
- 无法纳入现有故障恢复机制。

### 5.6.4 推荐处理

迁移时不要直接把原节点当成“黑盒子”挂进去，而要：

- 让新的 manipulation 节点支持 lifecycle；
- 或先以普通 node 方式接入，但由 `payload/accessories` 类别做弱托管；
- 最终正式版再升级到 lifecycle 完整接入。

**建议优先采用：首期普通节点 + 次期生命周期化。**

---

## 5.7 导航衔接冲突

### 5.7.1 会不会破坏 BotBrain 现有导航

如果采用正确接入方式，**不会破坏**。

原因是 BotBrain 当前导航主链只关心：

- 地图/定位是否正常；
- Nav2 是否产出速度；
- `twist_mux` 是否仲裁速度；
- G1 `robot_write_node` 是否执行底盘速度。

而上身控制本身：

- 不需要写 `cmd_vel_out`；
- 不需要改 Nav2 参数；
- 不需要改变 FSM 切换服务语义；
- 不需要替换 locomotion client。

### 5.7.2 真正需要注意的点

需要定义以下策略：

#### 模式 A：导航中允许摆臂/抬臂

- 底盘继续接收导航速度；
- 上身独立接收位姿目标；
- 适合视觉对准、探索、非接触操作。

#### 模式 B：抓取时强制停步

- 进入抓取任务状态；
- 暂停导航 goal 或置零速度；
- 等待 base settle；
- 执行手臂与夹爪控制；
- 完成后恢复导航。

#### 模式 C：半动态抓取

- 底盘低速微动；
- 上身同步调整；
- 仅在验证后开放。

### 5.7.3 建议

对当前阶段来说，最稳妥的是：

- **先支持模式 A + 模式 B**；
- **暂时不要一开始就追求移动中精细抓取**。

---

## 6. 无缝接入 BotBrain 的最佳集成方案

## 6.1 方案候选

### 方案 A：直接把 g1pilot 全工程并入 BotBrain

**不推荐。**

原因：

- 导航、teleop、状态、配置都重复；
- 会引入两套控制哲学；
- 后续维护成本很高；
- 很容易出现 topic/行为重复。

### 方案 B：只把 `arm_controller` 相关逻辑移植进 `g1_pkg`

**中期可行，但首期不一定是最省风险。**

优点：

- 最终统一到一个 G1 包中；
- 前端与服务接口集中；
- 部署统一。

缺点：

- 需要把 Python IK / DDS / TF / lifecycle 等多逻辑塞进 `g1_pkg`；
- 改动较深，调试风险高；
- 容易在首轮迁移时动到已有稳定链路。

### 方案 C：在 BotBrain 新增独立 `g1_manipulation_pkg`

**最推荐。**

优点：

- 与 BotBrain 导航主链解耦；
- 迁移范围清晰；
- 控制权边界明确；
- 出问题时可以整体禁用，不影响底盘导航；
- 后续可逐步生命周期化并合并进 `g1_pkg`。

缺点：

- 需要额外维护一个包；
- 需要定义与前端、状态机、任务层的接口。

### 结论

**首选方案：方案 C。**

---

## 6.2 推荐目标架构

建议新增一条 manipulation 子链：

### 6.2.1 BotBrain 保留不变的部分

- `bot_navigation`
- `bot_bringup/twist_mux`
- `bot_state_machine`
- `g1_pkg` 的 `robot_read_node`
- `g1_pkg` 的 `robot_write_node` 的 locomotion/FSM/急停部分

### 6.2.2 新增部分

新增 `g1_manipulation_pkg`（名称可自定义），包含：

1. `arm_rt_controller_node`
   - 来源于 g1pilot `arm_controller`
   - 负责 IK、目标平滑、低层 DDS 上身控制

2. `gripper_controller_node`
   - 来源于 g1pilot `dx3_hand`
   - 负责左右手爪控制

3. `manipulation_bridge_node`
   - 负责把 BotBrain 前端命令转成左右手位姿/抓手命令
   - 负责 home / enable / stop / release 等高层接口

4. `interactive_marker_node`（可选）
   - 仅用于 RViz 调试

### 6.2.3 与 BotBrain 的接口形式

建议对外统一暴露：

- topics
  - `/{robot_name}/manipulation/hand_goal/right`
  - `/{robot_name}/manipulation/hand_goal/left`
  - `/{robot_name}/manipulation/enabled`
  - `/{robot_name}/manipulation/home`
  - `/{robot_name}/gripper/right/command`
  - `/{robot_name}/gripper/left/command`

- services
  - `/{robot_name}/manipulation/set_mode`
  - `/{robot_name}/manipulation/reset`
  - `/{robot_name}/manipulation/home`
  - `/{robot_name}/manipulation/stop`

- status
  - `/{robot_name}/manipulation/state`
  - `/{robot_name}/manipulation/goal_reached`
  - `/{robot_name}/manipulation/enabled_state`

这样前端、任务规划、视觉抓取模块都能稳定复用。

---

## 6.3 与导航策略的衔接方式

### 6.3.1 底盘控制边界

必须坚持：

- 底盘速度只走 `cmd_vel_* -> twist_mux -> cmd_vel_out -> robot_write_node`；
- manipulation 节点不能写 `cmd_vel_out`；
- manipulation 节点最多只能请求“暂停导航”或“恢复导航”。

### 6.3.2 任务层交互建议

建议未来视觉抓取任务采用如下状态流：

1. 视觉模块发现目标；
2. 任务层生成 `grasp_candidate`；
3. 判断是否需要移动底盘粗对齐；
4. 若需要，BotBrain 导航先执行 base 对齐；
5. base 到位后，调用 manipulation enable + hand goal；
6. 操作完成后，执行 gripper close/open；
7. 再决定是否恢复导航。

### 6.3.3 为什么这套方式最稳

因为它天然符合你当前两个工程的既有事实：

- 步行由官方控制器负责；
- 手臂由自定义控制器负责；
- 两者之间只通过任务协调，而不是互相接管。

---

## 7. 推荐实施路线图

## 7.1 第一阶段：最小可用迁移（建议优先）

目标：**先让 BotBrain 拥有“运行中可控手臂”的能力。**

### 交付内容

- 在 BotBrain 新增独立 manipulation 包；
- 迁移 `arm_controller` 核心逻辑；
- 改造为 BotBrain namespace 规范；
- 保留 RViz marker 调试；
- 底盘仍完全走 BotBrain 原链路；
- 支持 enable/home/左右手目标/手爪开合。

### 成功标准

- BotBrain 导航正常；
- 导航过程中可启用手臂控制；
- 手臂目标跟踪稳定；
- 急停后上身与下身都可安全退出；
- 不出现 `rt/arm_sdk` 双写。

### 风险控制

- 先禁用 BotBrain 原 `arm_cmd` 的实时使用，只保留兼容入口；
- 不碰 Nav2 参数；
- 不重写 `robot_write_node` 的底盘逻辑。

---

## 7.2 第二阶段：BotBrain 原生化接口整合

目标：**让 manipulation 成为 BotBrain 正式子系统。**

### 内容

- 定义 BotBrain 风格自定义 service/action；
- 把前端按钮、任务、可视化面板打通；
- 接入 `bot_state_machine`，至少做弱托管；
- 增加状态反馈与错误码。

### 产出

- 前端可直接发上身目标；
- 任务模块可以统一调用；
- 与 `arm_cmd` 兼容或替代关系明确。

---

## 7.3 第三阶段：为视觉抓取做准备

目标：**将“可控手臂”升级为“可执行抓取任务”。**

### 需要补充的能力

- 视觉目标到手坐标系的变换；
- 抓取点筛选；
- base-arm 联合粗对齐策略；
- grasp state machine；
- 失败恢复动作；
- 动静态模式切换。

### 这阶段才需要重点考虑

- 移动中抓取；
- 上身与底盘协同优化；
- 更细粒度碰撞检测；
- 操作空间约束与工作空间裁剪。

---

## 8. 关键风险清单与修复建议

## 8.1 风险一：`rt/arm_sdk` 双写

### 风险描述

两个节点同时写同一个上身低层 DDS topic，会造成：

- 手臂跳变；
- 重复权重切换；
- 姿态抖动；
- 抓取失败甚至硬件风险。

### 修复建议

- 系统里只允许一个 active arm writer；
- BotBrain 原 `arm_cmd` 改为调用 manipulation node，而不是再自己写 DDS；
- 通过状态机或参数锁定当前 arm master。

---

## 8.2 风险二：23DoF/29DoF 模型不一致

### 风险描述

BotBrain 当前语义模型不足以承载完整操作臂能力。

### 修复建议

- 以 g1pilot 的 29DoF joint map 为标准建立 BotBrain G1 manipulation model；
- 保持 `robot_read_node` / `joint_states` / URDF 定义一致；
- 先允许 manipulation 使用自己的 joint map，后续再做统一。

---

## 8.3 风险三：Python 实时性和部署复杂度

### 风险描述

首期迁移若直接使用 Python `arm_controller`：

- 部署依赖增多；
- Jetson/机器人本体环境一致性要仔细处理；
- 高负载时可能比 C++ 更脆弱。

### 修复建议

- 第一阶段接受 Python 方案，快速验证价值；
- 第二阶段视稳定性考虑 C++ 重写关键运行环；
- 把调试依赖和运行时依赖分离。

---

## 8.4 风险四：状态机未接管 manipulation 节点

### 风险描述

如果 manipulation 节点不纳入 BotBrain 状态机：

- 系统启动顺序不透明；
- 异常恢复不统一；
- 前端显示状态不完整。

### 修复建议

- 首期以 accessories/payload 类弱托管；
- 二期改造成 lifecycle node；
- 增加健康状态 topic。

---

## 8.5 风险五：导航中控臂造成姿态扰动

### 风险描述

即便底盘与上身控制链独立，机器人本体动力学仍会耦合：

- 大幅摆臂时会影响稳定性；
- 腰部参与时耦合更明显；
- 低速步行时末端抖动会放大。

### 修复建议

- 首期限制上身速度、加速度、工作空间；
- 抓取状态默认停步；
- 只开放保守的 walking + arm pose 模式；
- 逐步验证动态抓取。

---

## 9. 对后续视觉抓取的价值判断

迁移 g1pilot 的上身控制能力到 BotBrain，对后续视觉抓取是**必要且高价值**的。

原因很直接：

- 视觉抓取需要连续末端目标控制；
- 需要手爪控制；
- 需要从感知结果转成操作空间目标；
- 需要在导航与操作之间切换状态。

BotBrain 当前的 pose 回放式 `arm_cmd` 更适合：

- 演示动作；
- 固定姿态切换；
- 手动预设动作库。

它并不足以支撑：

- 视觉伺服；
- 在线抓取；
- 动态调整手腕姿态；
- 双臂连续操作。

所以从路线规划上看，这次迁移不是“锦上添花”，而是**抓取任务的基础设施建设**。

---

## 10. 最终建议

## 10.1 核心建议

**建议立即启动迁移，但采取“BotBrain 主框架不动、增量引入 manipulation 子系统”的策略。**

## 10.2 不建议的做法

不建议：

- 直接把 g1pilot 导航、joy_mux、loco_client 整套迁进 BotBrain；
- 一开始就把 `arm_controller` 深度塞进 `g1_pkg` 主链；
- 让多个节点同时写 `rt/arm_sdk`；
- 还没做安全策略就追求边走边抓。

## 10.3 最佳实践建议

建议优先级如下：

### P0（必须）

- 新建独立 manipulation 包
- 迁移 `arm_controller` 核心逻辑
- 统一 namespace
- 明确 arm writer 唯一控制权
- 保持 BotBrain 导航主链不变

### P1（强烈建议）

- 接入手爪控制
- 增加 manipulation 状态接口
- 接入 BotBrain 前端
- 增加 home / enable / release / stop 服务

### P2（后续增强）

- lifecycle 化
- 与任务层/视觉抓取模块联动
- 更完整 29DoF 模型统一
- 动态抓取与 base-arm 联动

---

## 11. 一句话结论

**这项迁移是可行的，且应当做；但正确方式不是把 g1pilot“整体并入”BotBrain，而是把它的“上身实时控制能力”抽取为 BotBrain 的 manipulation 子系统，与现有导航链并行协作。**
