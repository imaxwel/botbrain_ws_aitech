# G1 本体语音导航 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:test-driven-development and superpowers:verification-before-completion. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 G1 本体 ASR 文本、云端 LLM 和当前 Nav2 waypoint 导航之间建立可测试的本机 ROS2 语音导航闭环。

**Architecture:** 新增独立 `bot_voice_navigation` 包，负责 ASR 文本解析、云端结构化意图、地点白名单、导航 Action 网关和原厂扬声器适配。现有 `bot_navigation` waypoint 数据库和 Nav2 行为保持不变，只提取可复用的导航控制器接口。

**Tech Stack:** ROS 2 Humble, Python 3, rclpy, rosidl action interfaces, Nav2 NavigateToPose, Unitree SDK2 AudioClient, urllib, pytest.

**Spec:** `docs/superpowers/specs/2026-09-04-g1-native-voice-navigation-design.md`

## Global Constraints

- 使用 G1 本体原生 ASR 文本输入，默认订阅 `/audio_msg`，不实现外接麦克风采集。
- 复用机器人系统已有的 G1 原生 ASR 发布者，同时兼容 `/audio_msg` 和 `/rt/audio_msg`；不把 `AudioClient.Init()` 误当作 API 1002 启动调用。
- 云端 LLM 只能返回结构化意图，不能返回坐标、shell、ROS 命令或 `cmd_vel`。
- 目的地必须经过本地场景白名单和 waypoint 数据库校验。
- 运动唯一入口是 `/g1_robot/navigate_to_pose`，不得直接调用 Unitree `Move()`。
- 现有 Nav2、FAST-LIO、定位、Costmap、速度平滑、twist_mux 和 G1 驱动行为保持不变。
- 现场测试只允许只读检查和软件 mock，不自动发送真实导航目标。

---

### Task 1: 语音文本与地点核心

**Files:**
- Create: `botbrain_ws/src/bot_voice_navigation/bot_voice_navigation/asr_parser.py`
- Create: `botbrain_ws/src/bot_voice_navigation/bot_voice_navigation/destinations.py`
- Create: `botbrain_ws/src/bot_voice_navigation/bot_voice_navigation/intent.py`
- Test: `botbrain_ws/src/bot_voice_navigation/test/test_core.py`

**Interfaces:** `parse_asr_message`, `TranscriptDeduper`, `DestinationResolver`, `parse_intent_payload`.

- [x] 写失败测试并确认缺少模块时收集失败。
- [x] 实现 plain/JSON ASR 解析、final 过滤、去重、场景别名和严格意图校验。
- [x] 运行 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest botbrain_ws/src/bot_voice_navigation/test/test_core.py -q`。

### Task 2: 云端 LLM/TTS 与原厂音频输出

**Files:**
- Create: `botbrain_ws/src/bot_voice_navigation/bot_voice_navigation/cloud.py`
- Create: `botbrain_ws/src/bot_voice_navigation/bot_voice_navigation/speech_output.py`
- Test: `botbrain_ws/src/bot_voice_navigation/test/test_cloud_and_speech.py`

**Interfaces:** `OpenAICompatibleLLM`, `CloudTTSClient`, `SpeechOutput`, `UnitreeAudioSink`.

- [x] 写失败测试并确认缺少模块时收集失败。
- [x] 实现可注入 HTTP transport、JSON schema 校验、PCM 校验和 Unitree `PlayStream` sink。
- [x] 运行核心与适配器测试。

### Task 3: 可复用 waypoint 控制器

**Files:**
- Create: `botbrain_ws/src/bot_navigation/bot_navigation/waypoint_controller.py`
- Modify: `botbrain_ws/src/bot_navigation/scripts/waypoint_navigator.py`
- Test: `botbrain_ws/src/g1_pkg/test/test_waypoint_controller_contract.py`

**Interfaces:** `NavigationResult`, `NavigationFeedback`, `WaypointNavigationController.start`, `request_cancel`.

- [x] 先写 controller 契约测试并确认失败。
- [x] 通过可取消进程控制器复用现有场景、地图、扫描、Nav2 Action 和最终 TF 复核。
- [x] 保持现有 CLI 参数、退出码和人工命令行为；原导航脚本仍是唯一执行后端。

### Task 4: NavigateToWaypoint Action 网关

**Files:**
- Create: `botbrain_ws/src/bot_custom_interfaces/action/NavigateToWaypoint.action`
- Modify: `botbrain_ws/src/bot_custom_interfaces/CMakeLists.txt`
- Modify: `botbrain_ws/src/bot_custom_interfaces/package.xml`
- Create: `botbrain_ws/src/bot_voice_navigation/bot_voice_navigation/navigation_gateway.py`
- Test: `botbrain_ws/src/bot_voice_navigation/test/test_gateway_policy.py`

**Interfaces:** `/g1_robot/navigate_to_waypoint`, single-task lock, alias resolution, cancel and fail-closed policy.

- [x] 写场景不一致、未知地点、重复任务和安全接受测试。
- [x] 添加 Action 定义并实现网关。
- [x] 构建 `bot_custom_interfaces` 并运行策略测试。

### Task 5: ASR 节点、云端链路和部署

**Files:**
- Create: `botbrain_ws/src/bot_voice_navigation/bot_voice_navigation/voice_navigation_node.py`
- Create: `botbrain_ws/src/bot_voice_navigation/launch/voice_navigation.launch.py`
- Create: `botbrain_ws/src/bot_voice_navigation/CMakeLists.txt`
- Create: `botbrain_ws/src/bot_voice_navigation/package.xml`
- Modify: `docker-compose.yaml`

- [x] 写 partial 抑制、Action 请求和结果播报测试。
- [x] 实现 ROS2 `/audio_msg` 订阅、LLM 调用、Action client 和 TTS。
- [x] 增加双话题监听，并明确公开 SDK 不足以主动启动 API 1002。
- [x] 添加 Compose 服务但不改变现有导航服务。

### Task 6: 验证

- [x] 运行所有新测试和现有导航边界测试。
- [x] 运行 `python3 -m compileall`。
- [x] 在 ROS2 Humble 系统环境构建 `bot_custom_interfaces bot_navigation bot_voice_navigation`。
- [x] 运行 `git diff --check`、YAML/XML/Action 语法检查。
- [x] 明确现场 ASR 接口验证仍需单独执行，不在软件 mock 中冒充实机通过。
