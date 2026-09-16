# G1 本体语音到导航完整实施方案

> 状态：代码已实施并完成软件 mock；G1 本体麦克风/原生 ASR 的现场连通性仍需在机器人网络恢复后验收。本文冻结导航侧边界，不把未验证的音频 payload 写死。

## 1. 目标

使用宇树 G1 自带四麦克风阵列和原生 ASR，把用户语音转换为当前项目的命名点导航任务。云端只负责 LLM 意图理解和 TTS；不接入现有通用语音问答业务，不使用 B1 外接麦克风，不把语音接入 `cmd_vel`。

```text
G1 本体麦克风 → 宇树 voice/ASR → /audio_msg 文本
→ 云端 LLM → 本地导航网关 → waypoint/Nav2
→ 现有速度与 Unitree 驱动 → G1 到达
→ 云端 TTS PCM → Unitree AudioClient → 原厂扬声器
```

## 2. 当前导航事实

人工入口：

```bash
ros2 run bot_navigation waypoint_navigator.py <waypoint> --scene <scene>
```

[waypoint_navigator.py](../../botbrain_ws/src/bot_navigation/scripts/waypoint_navigator.py) 已完成：

- 场景和 waypoint YAML 读取；
- `/map_server/get_parameters` 场景一致性检查；
- 新鲜 `/scan`、`/map` 检查；
- 地图范围和目标占用检查；
- `/g1_robot/navigate_to_pose`（`nav2_msgs/action/NavigateToPose`）；
- Nav2 feedback、成功、取消、失败；
- 最终 TF 距离和朝向复核，默认距离阈值 0.22m、朝向阈值 8°。

真实运动链路：

```text
NavigateToPose
→ Nav2 BT/ComputePathToPose/FollowPath
→ MPPI
→ /g1_robot/cmd_vel_nav_raw
→ cmd_vel_continuity
→ /g1_robot/cmd_vel_nav_filtered
→ velocity_smoother
→ /g1_robot/cmd_vel_nav
→ twist_mux
→ /g1_robot/cmd_vel_out
→ robot_write_node
→ G1Driver::move()
→ Unitree LocoClient::Move(vx, -vy, wz)
```

以上链路、参数、行为树、定位、地图、Costmap 和驱动保持不变。语音模块只负责提交命名点任务。

## 3. G1 原生音频边界

根据 WK 中的 Unitree SDK2 源码确认的公开接口：

- G1 具有四麦克风阵列和原厂扬声器；
- Unitree SDK2 提供 `voice` 服务；
- API 1001：`TtsMaker`；
- API 1002：ASR 注册接口；
- API 1003：`PlayStream`；
- API 1004：`PlayStop`；
- 官方资料说明 `rt/audio_msg` 用于 ASR 消息；
- ROS 图中对应 `/audio_msg`，类型为 `std_msgs/msg/String`；
- `/audiosender` 为播放方向的 `unitree_go/msg/AudioData`，不作为麦克风输入；
- 公开 SDK 没有读取 G1 原始麦克风 PCM 的接口。

因此第一版采用：

```text
G1 本体麦克风 → G1 原生 ASR → /audio_msg 文本 → 云端 LLM
```

不采用 WK 的 `sounddevice.InputStream`、Opus 麦克风采集和云端 ASR。

公开 Python SDK 的 `AudioClient.Init()` 只把 API 1002 登记到客户端白名单，并未
调用该 API，因此不能把它当作 ASR 启动操作。实现层复用机器人系统已有的原生 ASR
发布者，同时订阅 `/audio_msg` 和 `/rt/audio_msg`；两个 ROS 话题如果是同一条消息
的不同映射，会由最终文本去重器合并。软件测试直接 mock 发布
`std_msgs/msg/String`。

## 4. 总体架构

```text
┌──────────────────────────────┐
│ G1 voice/ASR                  │
│ 四麦克风 → rt/audio_msg       │
└──────────────┬───────────────┘
               │ std_msgs/msg/String
               ▼
┌──────────────────────────────┐
│ g1_voice_input                │
│ 文本解析、final 过滤、去重     │
└──────────────┬───────────────┘
               │ VoiceTranscript
               ▼
┌──────────────────────────────┐
│ CloudLLMAdapter               │
│ 云端结构化意图解析             │
└──────────────┬───────────────┘
               │ NavigationIntent
               ▼
┌──────────────────────────────┐
│ voice_navigation_gateway      │
│ 白名单、场景、任务锁、Action    │
└──────────────┬───────────────┘
               │ NavigateToWaypoint
               ▼
┌──────────────────────────────┐
│ 现有 Waypoint/Nav2 核心        │
└──────────────┬───────────────┘
               │ result/feedback
               ▼
┌──────────────────────────────┐
│ SpeechOutput                  │
│ TTS PCM → Unitree AudioClient │
└──────────────────────────────┘
```

语音服务与 ROS2 服务同机运行，使用现有 Zenoh：

```text
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'
```

Unitree 底层运动仍走 DDS 网卡 `enP8p1s0`，语音层不接触底盘控制 DDS。

## 5. 新增模块

### 5.1 g1_voice_input

职责：订阅 G1 ASR 文本，转换为内部事件。

```python
@dataclass(frozen=True)
class VoiceTranscript:
    text: str
    final: bool
    timestamp: float
    source: str
    raw: str
```

解析器兼容：

```text
带我去电梯
{"text":"带我去电梯"}
{"state":"final","text":"带我去电梯"}
{"asr":{"text":"带我去电梯"}}
```

只处理最终文本；partial 文本不得重复触发导航。相同文本在去重窗口内只处理一次。

实机阶段仍需要运行时确认：

- 机器人系统中哪个进程负责调用 API 1002，以及当前固件是否持续发布最终 ASR 文本；
- `/audio_msg` 和 `rt/audio_msg` 的真实关系；
- 文本字段和 final 状态格式；
- 唤醒前后的状态变化。

未验证前，不在启动脚本中猜测 API 1002 的额外 payload 或唤醒参数。

### 5.2 CloudLLMAdapter

```python
@dataclass(frozen=True)
class NavigationContext:
    scene: str
    available_destinations: list[str]
    active_task_id: str | None
    navigation_state: str

@dataclass(frozen=True)
class NavigationIntent:
    intent: str  # navigate/cancel_navigation/navigation_status/unknown
    destination: str
    sequence: list[str]
    confidence: float
    need_confirmation: bool
```

云端 LLM 只能返回结构化 JSON：

```json
{
  "intent": "navigate",
  "destination": "电梯",
  "sequence": [],
  "confidence": 0.96,
  "need_confirmation": false
}
```

约束：

- 目的地只能来自本地提供的名称；
- 不得返回坐标、shell、ROS topic、Action、service 或 `cmd_vel`；
- “停止/取消/别走了”返回 `cancel_navigation`；
- JSON 或字段非法时返回 `unknown`；
- 目的地不明确时返回 `need_confirmation=true`。

不接入当前 `bot_rosa`，避免把文本服务、WK 会话和 ROSA 工具体系混在一起。

### 5.3 场景化目的地白名单

新增 `bot_voice_navigation/config/voice_destinations.yaml`（只配置当前 waypoint 文件中真实存在的点位）：

```yaml
scenes:
  ug:
    aliases:
      家: home
      回家: home
      办公室一: office1
      办公室1: office1
      办公室二: office2
      办公室2: office2
      办公室三: office3
      办公室3: office3
      办公室四: office4
      办公室4: office4
      办公室五: office5
      办公室5: office5
      转弯点: turn
```

解析顺序：

```text
云端 destination
→ 文本规范化
→ 当前 scene aliases
→ waypoint_store.scene_waypoints()
→ 目标存在性、地图场景和占用检查
```

云端不能指定场景。当前场景没有目标或目标没有录入 waypoint 时拒绝发送。

### 5.4 NavigateToWaypoint Action

新增 `bot_custom_interfaces/action/NavigateToWaypoint.action`：

```text
# Goal
string destination
string[] sequence
string scene
bool loop
---
# Result
bool success
string status
string message
string task_id
string[] completed_waypoints
---
# Feedback
string task_id
string current_waypoint
string state
float32 distance_remaining
float32 distance_to_goal
float32 yaw_error
uint32 recoveries
```

服务名：

```text
/g1_robot/navigate_to_waypoint
```

网关职责：

1. 校验请求；
2. 获取当前 scene；
3. 解析别名；
4. 加载 waypoint 数据库；
5. 核对 map_server 实际场景；
6. 检查定位、地图、扫描和 Nav2 就绪；
7. 调用导航控制器；
8. 转发 feedback/result/cancel；
9. 维护单任务锁；
10. 使用 request ID 幂等。

### 5.5 WaypointNavigationController

当前实现采用进程边界复用，而不是复制导航核心：`WaypointNavigationController` 以参数数组启动现有 `waypoint_navigator.py`，消费其标准输出形成反馈，并通过终止同一进程实现取消。命令行入口和现场已验收的导航逻辑保持为唯一执行后端：

```python
class WaypointNavigationController:
    def build_command(self, names: Sequence[str], scene: str, *, loop: bool = False) -> list[str]: ...
    def start(self, names: Sequence[str], scene: str, *, loop: bool = False, task_id: str = "", feedback_callback=None) -> NavigationResult: ...
    def request_cancel(self) -> bool: ...
```

必须保留现有检查和行为：

- 新鲜 `/scan`；
- `/map`；
- `live_map_scene()`；
- `goal_grid_occupancy()`；
- Nav2 Action；
- Action cancel；
- 最终 TF 复核；
- 命令行退出码；
- 现有人工命令行为。

不得复制第二套导航实现。

### 5.6 SpeechOutput

云端 TTS 统一转换为：

```text
16 kHz / mono / signed 16-bit little-endian PCM
```

通过 Unitree SDK2：

```python
ChannelFactoryInitialize(0, "enP8p1s0")
audio = AudioClient()
audio.SetTimeout(10.0)
audio.Init()
audio.PlayStream("botbrain_voice_navigation", stream_id, pcm_data)
```

取消：

```python
audio.PlayStop("botbrain_voice_navigation")
```

AudioClient 由单一 worker 持有，不能阻塞 ROS2 executor。第一版采用半双工：播报期间暂停普通 ASR 触发，播放 drain 后等待 100~300ms 再恢复。

## 6. 状态机

```text
IDLE → LISTENING → PARSING
     → CONFIRMING
     → VALIDATING
     → ACCEPTED
     → NAVIGATING
     → SUCCEEDED/CANCELED/ABORTED
     → SPEAKING → IDLE
```

异常状态：

```text
ASR_UNAVAILABLE
LLM_ERROR
INVALID_DESTINATION
NAVIGATION_UNREADY
SAFETY_STOP
NETWORK_ERROR
```

导航中再次收到地点请求时，不启动第二个任务；停止请求走 cancel。相同 request ID 必须幂等。

## 7. 安全边界

唯一允许的运动路径：

```text
本体 ASR → 云端意图 → 本地白名单 waypoint
→ NavigateToWaypoint → 现有 Nav2 → twist_mux → robot_write_node
```

禁止：

- 云端发送坐标；
- 云端发送 shell/ROS 命令；
- 云端发送 `cmd_vel`；
- 语音直接调用 Unitree `Move()`；
- 语音直接切换地图；
- 定位未就绪时发送目标；
- 并发导航目标。

以下任一条件成立时 fail-closed：

- `/localization_ready=false`；
- ICP 置信度低或过期；
- `/scan` 过期；
- `/g1_robot/nav_odom` 过期或协方差非法；
- `/map` 不可用；
- map_server 场景不一致；
- waypoint 不存在、越界或目标非法；
- Nav2 Action 不可用；
- 急停或高优先级安全停止有效。

现有 `localization_monitor` 继续负责安全零速度和必要的导航取消。

## 8. 部署

新增独立 Compose 服务 `voice_navigation`，不替换 `rosa`，不修改 `navigation`：

```yaml
voice_navigation:
  extends: base
  container_name: g1_robot_voice_navigation
  volumes:
    - ./botbrain_ws/:/botbrain_ws
    - /usr/local/include:/opt/robot_sdk/include:ro
    - /usr/local/lib:/opt/robot_sdk/lib:ro
    - /run:/run
  environment:
    RMW_IMPLEMENTATION: rmw_zenoh_cpp
    ZENOH_CONFIG_OVERRIDE: 'mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'
    UNITREE_SDK2_ROOT: /opt/robot_sdk
    G1_DDS_INTERFACE: enP8p1s0
    G1_ROBOT_NAMESPACE: g1_robot
  command:
    - bash
    - -lc
    - source install/setup.bash && ros2 launch bot_voice_navigation voice_navigation.launch.py
  restart: "no"
```

服务等待现有导航 Action，不负责启动或重启 Nav2；不需要访问 `/dev/snd`，不需要 B1 音频设备。

## 9. 实施阶段

### 阶段 0：ASR 适配框架

- 创建文本解析器、去重和 health 状态；
- 同时支持 `/audio_msg` 与可配置 `rt/audio_msg`；
- 用 mock 消息完成单元测试；
- 现场验证 API 1002 和实际文本格式，验证完成后再启用生产输入。

### 阶段 1：导航核心模块化

- 提取 `WaypointNavigationController`；
- 保持命令行入口和行为不变；
- 增加反馈回调与取消；
- 运行现有 waypoint、定位和导航边界测试。

### 阶段 2：ROS2 Action 网关

- 新增 Action 接口并更新 `bot_custom_interfaces`；
- 实现白名单、场景校验、任务锁、幂等和 cancel；
- 使用固定文本验证“名称 → waypoint → Nav2”。

### 阶段 3：云端 LLM

- 实现 `CloudLLMAdapter`；
- 使用 JSON schema、超时和有限重试；
- 先支持当前场景已登记的办公室、回家、转弯点、停止、状态；新增点位后再补别名。

### 阶段 4：原厂扬声器

- 实现 AudioClient 单例 worker；
- 接入云端 TTS PCM；
- 实现分块、drain、取消和半双工抑制。

### 阶段 5：整机编排

- 加入 `voice_navigation` Compose 服务；
- 配置启动顺序和健康检查；
- 完成实机闭环验收。

## 10. 测试与验收

### 单元测试

- ASR plain/JSON/final/partial 解析；
- 文本去重；
- LLM schema 校验；
- 别名和场景隔离；
- 并发任务锁；
- Action 状态转换和取消；
- 安全拒绝；
- TTS PCM 格式与分块。

### ROS2 集成测试

- mock `/audio_msg` → LLM → Action goal；
- mock Nav2 accepted/feedback/succeeded/aborted/canceled；
- 场景不一致时不发送 goal；
- 定位未就绪时不发送 goal；
- 重复请求不创建第二个任务；
- cancel 可取消当前目标。

### 实机验收

```text
1. 启动 bringup、fast_lio、localization、navigation
2. 确认 navigation preflight passed
3. 启动 voice_navigation
4. 对 G1 说“带我去办公室一”（当前仓库实际已登记点位）
5. 确认 G1 原生 ASR 产生 /audio_msg
6. 确认 LLM 输出 destination=办公室一
7. 确认本地解析为 `office1`
8. 确认 /g1_robot/navigate_to_pose accepted
9. 确认机器人沿现有速度链行走
10. 确认 feedback 持续更新
11. 确认最终距离 ≤ 0.22m、朝向误差 ≤ 8°
12. 确认原厂扬声器播报“已到达办公室一”
```

失败用例必须验证：

- ASR 不可用；
- LLM 超时/非法 JSON；
- 目的地不存在；
- 定位未就绪；
- 导航中重复请求；
- 说“停止”；
- 定位丢失；
- TTS 失败。

## 11. 第一版不做

- 原始本体麦克风 PCM 导出；
- 云端 ASR 替换 G1 原生 ASR；
- `bot_rosa` 接入；
- 任意坐标导航；
- 自动楼层识别和跨楼层连续导航；
- 语音直接控制底盘；
- 语音切换地图；
- 未标定的 barge-in；
- 未标定的多扬声器同播。

## 12. 完成定义

```text
用户通过 G1 本体麦克风说出白名单目的地
→ G1 原生 ASR 产生最终文本
→ 云端 LLM 返回合法意图
→ 本地映射到当前场景 waypoint
→ 现有 Nav2 Action 被接受
→ 机器人沿当前已验收链路移动
→ 现有到点复核通过
→ G1 原厂扬声器播报结果
```

任何一层未就绪，都不得向 Nav2 发送未经验证的目标。
