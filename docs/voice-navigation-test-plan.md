# G1 语音导航分阶段测试流程

本文用于逐步验证 G1 语音导航链路。必须按阶段执行并记录结果；前一阶段未通过时，不得进入下一阶段。

## 0. 当前状态与总待办

当前代码、干跑脚本和本地自动化测试已经准备好，但机器人现场验证尚未完成。按以下顺序逐项打勾，禁止跳过阶段：

- [ ] 将本项目最新语音导航代码同步到机器人项目。
- [ ] 在机器人项目的 `botbrain_ws/robot_config.yaml` 填写 LLM 三项必填配置。
- [ ] 重新编译 `bot_voice_navigation`，确保机器人使用最新配置读取代码。
- [ ] 执行阶段零，记录机器人容器状态和项目版本。
- [ ] 执行阶段一，只验证麦头和 Unitree 原生 ASR final 文本。
- [ ] 执行阶段二离线 mock，确认合法点位通过、未知点位被拒绝。
- [ ] 执行阶段二真实 LLM 干跑，确认只打印目标命令，机器人不移动。
- [ ] 执行阶段三，人工核对别名、场景、点位和最终导航参数。
- [ ] 完成阶段四全部安全检查。
- [ ] 最后才执行阶段五单点真实导航。

> **当前阻塞项：** `robot_config.yaml` 中的 `llm_endpoint`、`llm_api_key`、`llm_model` 默认留空。在这三项全部填写并通过真实 LLM 干跑前，禁止启动真实语音导航。

## 1. 测试目标与安全边界

当前链路如下：

```text
G1 麦头
  -> Unitree 原生 ASR（语音转文字）
  -> rt/audio_msg 或 ROS /audio_msg、/rt/audio_msg
  -> OpenAI-compatible LLM（从文字提取结构化意图）
  -> 当前场景的点位/别名白名单
  -> /g1_robot/navigate_to_waypoint
  -> waypoint_navigator.py
  -> Nav2
```

注意：LLM 不负责收音，也不负责语音转文字。阶段一先验证 Unitree 原生 ASR；阶段二才验证 LLM 对文字的意图提取。

安全规则：

- 阶段零至阶段三禁止机器人移动。
- 禁止发送 `/g1_robot/goal_pose`，禁止发送 `NavigateToWaypoint` Action goal，禁止直接执行 `waypoint_navigator.py`。
- `tools/test_voice_navigation_dry_run.py` 只解析并打印“将执行的命令”，不会调用 ROS Action、导航脚本或 `subprocess`。
- 只有阶段四的人工安全检查全部通过后，才进入阶段五真实导航。
- LLM/TTS 配置统一写在项目的 `botbrain_ws/robot_config.yaml`；示例占位符不能直接用于测试。

## 2. 当前代码基线

本项目已经包含：

- Compose 服务：`voice_navigation`；
- ASR 输入：ROS `/audio_msg`、`/rt/audio_msg`，以及 Unitree DDS `rt/audio_msg`；
- LLM/TTS 配置：`botbrain_ws/robot_config.yaml` 中的 `robot_configuration.voice_navigation`；
- 点位别名：`botbrain_ws/src/bot_voice_navigation/config/voice_destinations.yaml`；
- 点位数据：`botbrain_ws/src/bot_navigation/nav_waypoints.yaml`；
- Action 网关：`/g1_robot/navigate_to_waypoint`；
- 干跑脚本：`tools/test_voice_navigation_dry_run.py`；
- 原生 ASR 监听脚本：`botbrain_ws/src/bot_voice_navigation/scripts/test_g1_asr_monitor.py`。

当前仓库 `ug` 场景映射：

| 语音说法 | 实际点位 |
| --- | --- |
| 家、回家 | `home` |
| 办公室一、办公室1 | `office1` |
| 办公室二、办公室2 | `office2` |
| 办公室三、办公室3 | `office3` |
| 办公室四、办公室4 | `office4` |
| 办公室五、办公室5 | `office5` |
| 转弯点、转弯处 | `turn` |

当前没有“电梯”点位和别名，因此说“去电梯”必须被拒绝，不能猜测或发送导航。

## 3. 阶段零：检查容器和项目状态

### 3.1 本项目工作站

```bash
cd /home/aitech/Workspace/botbrain_project
docker compose ps -a
docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
git status --short
```

2026-09-17 本轮检查结果：本项目没有已创建或正在运行的 Compose 容器；系统里存在其他项目容器，但与本次语音导航无关。本机用于代码、单元测试和 mock/dry-run，真实 ASR/ROS2 在机器人端测试。

### 3.2 设置机器人连接变量

在工作站终端执行：

```bash
export ROBOT_IP=192.168.100.41
export ROBOT_USER=unitree
export ROBOT_PASSWORD='123'
export ROBOT_PROJECT=/data/botbrain_ws/botbrain_project-main
```

### 3.3 机器人端只读检查

```bash
SSHPASS="$ROBOT_PASSWORD" sshpass -e ssh -o StrictHostKeyChecking=no \
  "$ROBOT_USER@$ROBOT_IP" \
  "cd '$ROBOT_PROJECT' && git rev-parse --short HEAD && git status --short && docker compose ps -a"
```

再检查相关容器：

```bash
SSHPASS="$ROBOT_PASSWORD" sshpass -e ssh -o StrictHostKeyChecking=no \
  "$ROBOT_USER@$ROBOT_IP" \
  "docker ps -a --format '{{.Names}}|{{.Image}}|{{.Status}}' | grep '^g1_robot_' || true"
```

记录以下容器是否存在及状态：

```text
g1_robot_zenoh:
g1_robot_bringup:
g1_robot_fast_lio:
g1_robot_localization:
g1_robot_navigation:
g1_robot_voice_navigation:
```

这一步只检查，不启动或停止任何服务。基础定位/导航栈未就绪时，后续真实导航必须暂停；阶段一至阶段三仍可分别进行安全测试。

## 4. 阶段一：验证麦头和原生 ASR

目标：证明机器人说话输入能够变成 final 文本。不调用 LLM，不连接导航 Action。

### 4.1 检查音频设备和语音进程

```bash
SSHPASS="$ROBOT_PASSWORD" sshpass -e ssh "$ROBOT_USER@$ROBOT_IP" '
  ls -l /dev/snd 2>/dev/null || true
  arecord -l 2>/dev/null || true
  ps -eo pid,comm,args | grep -Ei "audio|asr|speech|voice" | grep -v grep || true
'
```

`/dev/snd` 或 `arecord` 只作为辅助信息。当前代码不直接采集 ALSA PCM，最终验收依据是 Unitree 原生 ASR 是否产生文本。

### 4.2 优先检查已有 ROS 话题

如果 `g1_robot_bringup` 正在运行：

```bash
SSHPASS="$ROBOT_PASSWORD" sshpass -e ssh "$ROBOT_USER@$ROBOT_IP" \
  "docker exec g1_robot_bringup bash -lc 'source /opt/ros/humble/setup.bash && source /botbrain_ws/install/setup.bash && ros2 topic list | grep -E \"^/audio_msg$|^/rt/audio_msg$\" || true'"
```

发现 `/audio_msg` 后监听一次：

```bash
SSHPASS="$ROBOT_PASSWORD" sshpass -e ssh "$ROBOT_USER@$ROBOT_IP" \
  "docker exec g1_robot_bringup bash -lc 'source /opt/ros/humble/setup.bash && source /botbrain_ws/install/setup.bash && timeout 30 ros2 topic echo /audio_msg --once'"
```

如果只有 `/rt/audio_msg`，将命令中的话题名替换为 `/rt/audio_msg`。

### 4.3 ROS 话题不可见时，直接监听 Unitree DDS

此命令创建一次性测试容器，只运行 ASR 监听脚本，不启动正式语音导航服务：

```bash
SSHPASS="$ROBOT_PASSWORD" sshpass -e ssh "$ROBOT_USER@$ROBOT_IP" \
  "cd '$ROBOT_PROJECT' && docker compose --profile voice_navigation run --rm --no-deps \
    -e G1_DDS_INTERFACE=enP8p1s0 \
    --entrypoint bash voice_navigation -lc \
    'python3 /botbrain_ws/src/bot_voice_navigation/scripts/test_g1_asr_monitor.py --timeout 30'"
```

命令出现 `Listening on Unitree rt/audio_msg` 后，对着麦头清楚说：

```text
带我去办公室一
```

通过标准：

- 输出 `ASR final text: ...`；
- 文本非空，含义与口述命令一致；
- 只输出 final 文本，不把 partial 文本送入下一阶段；
- 全程未调用 LLM 和导航。

若 30 秒超时，记录 ROS 话题、语音进程和容器日志后停止，不进入阶段二。不要自行调用未经实机确认的 Unitree API 1002。

## 5. 阶段二：配置并验证 LLM 意图提取

### 5.1 配置位置

配置统一位于当前项目：

```text
配置文件：项目根目录下的 botbrain_ws/robot_config.yaml
读取配置的服务：voice_navigation
运行容器：g1_robot_voice_navigation
```

本项目参考了 WK 中“项目内配置 + 大模型意图触发本地导航能力”的组织思路，但没有照搬 WK 的小智服务端、MCP 注册方式或硬编码脚本路径。当前链路仍是 G1 原生 ASR、独立 OpenAI-compatible LLM、点位白名单和现有 Nav2。

进入机器人项目并编辑配置：

```bash
cd /data/botbrain_ws/botbrain_project-main
nano botbrain_ws/robot_config.yaml
```

在现有 `robot_configuration` 下找到并填写以下配置段：

```yaml
robot_configuration:
  # 其他现有机器人配置保持不变
  voice_navigation:
    llm_endpoint: "https://实际供应商的完整接口/v1/chat/completions"
    llm_api_key: "实际 API Key"
    llm_model: "实际模型名称"
    tts_endpoint: ""
    tts_api_key: ""
    tts_model: ""
    tts_voice: "zh-CN"
```

> **明确提示：** `https://实际供应商...`、`实际 API Key`、`实际模型名称` 都是说明文字，不能原样保留。没有拿到真实值时保持空字符串，并将阶段二标记为未完成。

| 字段 | 是否必填 | 填写内容 |
| --- | --- | --- |
| `llm_endpoint` | 必填 | OpenAI-compatible Chat Completions 完整 URL，不是控制台首页或只有域名 |
| `llm_api_key` | 必填 | 对应接口的真实 API Key |
| `llm_model` | 必填 | 供应商实际开放的模型标识 |
| `tts_endpoint` | 可选 | 当前代码支持的 TTS 接口；第一轮可以留空 |
| `tts_api_key` | 可选 | TTS API Key；留空时会使用 `llm_api_key` |
| `tts_model` | 可选 | TTS 模型标识 |
| `tts_voice` | 可选 | 音色，空值自动使用 `zh-CN` |

不配置 TTS 时，只影响机器人语音播报，不影响“ASR 文本 -> LLM 意图提取 -> 导航命令”的主链路。

LLM 接口必须接受当前代码发送的 OpenAI-compatible Chat Completions 请求，并支持 JSON 对象输出。配置保存后先检查 YAML 能正常读取：

```bash
cd /data/botbrain_ws/botbrain_project-main
python3 -c 'import yaml; yaml.safe_load(open("botbrain_ws/robot_config.yaml", encoding="utf-8")); print("robot_config.yaml: OK")'
```

再检查三项 LLM 配置是否为非空，不打印 API Key：

```bash
python3 - <<'PY'
import yaml

data = yaml.safe_load(open("botbrain_ws/robot_config.yaml", encoding="utf-8"))
voice = data["robot_configuration"]["voice_navigation"]
for name in ("llm_endpoint", "llm_api_key", "llm_model"):
    print(f"{name}={'<configured>' if str(voice.get(name, '')).strip() else '<empty>'}")
PY
```

通过标准：`llm_endpoint`、`llm_api_key`、`llm_model` 三项都显示 `<configured>`。任一项为 `<empty>` 都不能执行真实 LLM 干跑。

配置修改不需要重新编译；语音导航代码修改后需要重新编译。首次部署本次改动时执行：

```bash
cd /data/botbrain_ws/botbrain_project-main
docker compose run --rm builder_base
```

如果机器人上已有旧的 `g1_robot_voice_navigation` 容器，正式启动时使用 `--force-recreate`，确保加载最新代码和配置。

### 5.2 先在本项目做离线 mock

```bash
cd /home/aitech/Workspace/botbrain_project
python3 tools/test_voice_navigation_dry_run.py \
  --transcript '带我去办公室一' \
  --scene ug
```

预期关键输出：

```text
DRY-RUN ONLY: 不启动 ROS Action，不调用导航脚本，不会让机器人运动
resolved waypoints: ["office1"]
would execute (NOT executed):
PASS: ASR -> intent -> scene/alias mapping -> navigator command dry-run
```

验证未知点位会被拒绝：

```bash
python3 tools/test_voice_navigation_dry_run.py \
  --transcript '带我去电梯' \
  --scene ug \
  --intent-json '{"intent":"navigate","destination":"电梯","sequence":[],"confidence":0.99,"need_confirmation":false}'
```

此命令预期非零退出，错误包含 `destination is not allowed`，且不能出现 `would execute`。

### 5.3 在机器人上调用真实 LLM，但仍然干跑

干跑脚本直接读取项目中的 `botbrain_ws/robot_config.yaml`。额外挂载本项目 `tools` 目录，只运行干跑脚本，不启动正式语音节点：

```bash
SSHPASS="$ROBOT_PASSWORD" sshpass -e ssh "$ROBOT_USER@$ROBOT_IP" \
  "cd '$ROBOT_PROJECT' && docker compose --profile voice_navigation run --rm --no-deps \
    -v \"\$PWD/tools:/test-tools:ro\" \
    --entrypoint bash voice_navigation -lc \
    'python3 /test-tools/test_voice_navigation_dry_run.py --llm --transcript \"带我去办公室一\" --scene ug'"
```

LLM 返回必须满足：

```json
{
  "intent": "navigate",
  "destination": "办公室一",
  "sequence": [],
  "confidence": 0.95,
  "need_confirmation": false
}
```

具体 confidence 可以不同，但正式节点默认要求不低于 `0.55`。模型不得返回坐标、shell 命令、ROS 命令或 `cmd_vel`。未知地点必须返回 `unknown` 或 `need_confirmation=true`，不能自行猜测。

通过标准：输出 `intent source: live LLM response`、合法白名单目标和 `would execute (NOT executed)`；机器人没有移动。

## 6. 阶段三：核对“将调用的导航脚本”

阶段二通过后，人工核对干跑输出中的命令。`办公室一` 应解析为：

```text
.../waypoint_navigator.py office1 --scene ug --robot g1_robot
```

其等价 ROS 命令是：

```bash
ros2 run bot_navigation waypoint_navigator.py office1 --scene ug
```

本阶段只核对字符串和参数，禁止执行上面的等价 ROS 命令。确认以下内容：

- `resolved waypoints` 是 `office1`，不是模型自由生成的名称；
- `--scene ug` 与当前定位场景一致；
- `nav_waypoints.yaml` 的 `ug` 场景确实存在 `office1`；
- 输出明确包含 `NOT executed`；
- 测试期间 `/g1_robot/navigate_to_waypoint` 没有收到新 goal，机器人没有移动。

## 7. 阶段四：真实导航前人工安全闸门

现场操作者逐项确认：

- [ ] 机器人已停稳，周围无人员和障碍，急停可立即使用。
- [ ] 当前地图、localization 和点位文件使用同一个场景。
- [ ] localization 日志出现 `Localization ready`。
- [ ] `/localization_ready=true`，实时点云与静态地图重合。
- [ ] Navigation 日志出现 `Navigation preflight passed`。
- [ ] Nav2 lifecycle 节点全部为 active。
- [ ] 阶段一已获得正确 final ASR 文本。
- [ ] 阶段二真实 LLM 已返回白名单目标。
- [ ] 阶段三干跑命令已人工核对，确认没有执行。
- [ ] 首次真实测试只使用一个已登记的近距离点位，不使用 `--loop`，不跨楼层。

任何一项未通过，都不得进入阶段五。

## 8. 阶段五：首次真实语音导航

只有阶段四全部通过后，才允许启动正式服务：

```bash
SSHPASS="$ROBOT_PASSWORD" sshpass -e ssh "$ROBOT_USER@$ROBOT_IP" \
  "cd '$ROBOT_PROJECT' && MAP_SCENE=ug docker compose --profile voice_navigation up -d --force-recreate voice_navigation && docker compose logs --tail=100 voice_navigation"
```

首次只说一个现场确认安全的已登记点位，例如：

```text
带我去办公室一
```

不要以“电梯”等未登记点位开始测试。观察日志中的 ASR、LLM 意图、Action 接受状态和导航结果。

出现以下任一情况立即停止语音导航服务，不重试运动：

- 日志提示 `robot_config.yaml missing required fields`；
- 目标不在当前场景白名单；
- 请求场景与 active scene 不一致；
- localization 未 ready；
- Navigation Action server 不可用；
- 点云与地图明显错位；
- 机器人运动方向异常。

导航中说“停止导航”或“取消导航”应走本地取消逻辑，不依赖云端 LLM。确认目标已取消、机器人停稳后，才能继续测试。

## 9. 测试记录模板

```text
日期/操作者：
机器人 IP：
机器人项目 HEAD：
当前 scene：

[ ] 阶段零：容器
zenoh：
bringup：
fast_lio：
localization：
navigation：
voice_navigation：

[ ] 阶段一：麦头/ASR
输入语句：
消息来源（DDS/ROS topic）：
final 文本：
结果/日志：

[ ] 阶段二：LLM
endpoint（不含 key）：
model：
transcript：
intent：
destination：
sequence：
confidence：
need_confirmation：
耗时/结果：

[ ] 阶段三：dry-run
resolved waypoints：
would execute 命令：
确认未执行：

[ ] 阶段四：安全闸门
未通过项/处理：

[ ] 阶段五：真实导航
实际点位：
Action 结果：
Nav2 结果：
是否取消或急停：
```
