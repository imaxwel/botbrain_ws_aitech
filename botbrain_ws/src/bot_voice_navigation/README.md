# bot_voice_navigation

G1 本体原生 ASR 到当前 BotBrain waypoint/Nav2 的本机桥接。

## 数据流

```text
/audio_msg 或 /rt/audio_msg (std_msgs/msg/String)
  -> voice_navigation_node
  -> 云端结构化意图
  -> /g1_robot/navigate_to_waypoint
  -> 现有 waypoint_navigator.py
  -> /g1_robot/navigate_to_pose
```

该包不采集外接麦克风、不发送 `cmd_vel`、不调用 Unitree `Move()`。导航仍由现有 `bot_navigation`、Nav2、twist_mux 和 `g1_pkg` 完成。

## 配置

LLM/TTS 配置位于项目的 `botbrain_ws/robot_config.yaml`：

```yaml
robot_configuration:
  voice_navigation:
    llm_endpoint: "https://api.example/v1/chat/completions"
    llm_api_key: "..."
    llm_model: "..."
    tts_endpoint: ""
    tts_api_key: ""
    tts_model: ""
    tts_voice: "zh-CN"
```

`llm_endpoint`、`llm_api_key`、`llm_model` 为必填项。TTS 配置可留空；不配置 TTS 只会关闭语音播报，不影响 ASR、意图提取和导航。环境变量 `G1_LLM_*`、`G1_TTS_*` 仅作为兼容覆盖，不是正常部署的主配置方式。

地点别名位于 `config/voice_destinations.yaml`，只会解析到当前 `nav_waypoints.yaml` 中已经实际登记的点位。当前场景由 `/botbrain_ws/.runtime/map_scene` 决定，与 `建图导航指令.md` 的运行流程一致。

当前仓库 `ug` 场景的实际点位和语音匹配如下（以现场文件为准，新增点位后再补别名）：

| 语音说法 | 规范 waypoint | 实际导航命令 |
| --- | --- | --- |
| 回家、家 | `home` | `waypoint_navigator.py home --scene ug` |
| 办公室一/1 | `office1` | `waypoint_navigator.py office1 --scene ug` |
| 办公室二/2 | `office2` | `waypoint_navigator.py office2 --scene ug` |
| 办公室三/3 | `office3` | `waypoint_navigator.py office3 --scene ug` |
| 办公室四/4 | `office4` | `waypoint_navigator.py office4 --scene ug` |
| 办公室五/5 | `office5` | `waypoint_navigator.py office5 --scene ug` |
| 转弯点、转弯处 | `turn` | `waypoint_navigator.py turn --scene ug` |

当前 `nav_waypoints.yaml` 未登记 `ug_face_evelator`，因此“电梯”不会被发送；现场记录该点位后再在别名文件中启用。

## 启动

```bash
source /botbrain_ws/install/setup.bash
ros2 launch bot_voice_navigation voice_navigation.launch.py
```

先按 `建图导航指令.md` 完成 FAST-LIO、localization、navigation 和 `Navigation preflight passed`，再启动本包。节点同时监听 `/audio_msg` 与 `/rt/audio_msg`；两个话题若重复发布，会由文本去重器合并。节点不读取原始 PCM，也不采集外接麦克风。ASR 的最终文本格式支持 plain text 和常见 JSON 包装。

公开 Unitree Python SDK 的 `AudioClient.Init()` 只登记客户端可调用 API，并不会调用 API 1002 启动识别。本包因此复用机器人系统已经运行的原生 ASR 发布者，不构造未经实机确认的 ASR 请求。软件测试可直接用 `std_msgs/msg/String` mock 发布 `/audio_msg`。

## ROS2 Action

```text
/g1_robot/navigate_to_waypoint
bot_custom_interfaces/action/NavigateToWaypoint
```

目标使用地点名称和当前场景，网关执行白名单、场景、任务锁和取消检查，然后调用现有 waypoint 导航器。
