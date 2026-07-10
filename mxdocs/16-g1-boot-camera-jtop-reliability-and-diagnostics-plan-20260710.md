# 16 G1 开机后 Cockpit 摄像头与 Health/jtop 稳定性优化及诊断脚本规划

日期: 2026-07-10  
分析对象: `botbrain_ws_aitech`，当前提交 `873e434`  
目标机器: `g1edu`、`g1hk` 及后续采用同一代码基线的 G1 机器人  
分析方式: 当前仓库源码、运行产物、Compose/systemd 模板及 `mxdocs/08`、`11`、`13`、`14` 的静态交叉核对

> 边界说明: 本文是实施规划，不代表已经在真实 G1 上完成冷启动验收。现场记录来自已有文档；本文新增的判断来自当前工作区静态检查。真正关闭问题前，必须在至少 `g1edu`、`g1hk` 各执行冷启动测试。

## 1. 最终目标和非目标

### 1.1 最终目标

每次 G1 完整关机再开机后，不依赖 SSH 手工命令，自动满足以下结果:

1. D435i 只被一个服务拥有，`/g1_robot/front_camera` 自动进入 `active`。
2. `/g1_robot/compressed_camera` 持续有新帧，Cockpit 能显示实时画面。
3. Jetson host 的 `jetson-stats/jtop` 后台能力正常，`/g1_robot/jtop_publisher` 自动进入 `active`。
4. `/g1_robot/diagnostic_stats` 持续有新数据，Health 页面能显示真实 Jetson 指标。
5. rosbridge 和 frontend 晚启动、重连或单独重启后，能自动恢复订阅。
6. 任一环节异常时，运维人员先运行只读脚本保存现场，再根据分型运行单一职责恢复脚本。
7. 同一套代码适用于不同机器；serial、namespace、网卡、ROS bridge 地址等机器差异只放在机器本地配置中。

### 1.2 非目标

- 不让摄像头是否工作依赖 FAST-LIO、Open3D localization、Nav2、YOLO 或 AprilTag 是否正常。
- 不用脚本自动猜测并写入 D435i serial。
- 不在诊断脚本中 patch Git 文件、清理整个 workspace、重启整机或修改 USB 内核参数。
- 不把 `docker compose restart`、固定 `sleep 30` 或 Docker restart policy 当成 readiness 编排。
- 不用 frontend 的默认值或假数据掩盖后端无数据。

## 2. 结论摘要

“摄像头和 jtop 时有时无”不是单个 frontend bug，而是以下问题叠加后的非确定性结果:

1. **当前源码、`install/` 和已创建容器不是同一个运行版本。** 容器统一执行 `source install/setup.bash`，但关键 installed 文件明显落后于 `src/`。
2. **systemd 没有声明完整目标服务图。** `botbrain.service` 不启动当前拥有相机的 `localization`，也不启动 `web_server_prod`；`web_server.service` 启停服务名还不一致。
3. **Compose 配置存在版本兼容风险。** 当前环境执行 `docker compose config` 已失败，错误为 `services.nav3d.env_file.0 must be a string`，说明长格式 `env_file` 与本机 Compose 版本不兼容。
4. **摄像头错误地由重型 localization 服务拥有。** localization 晚 30 秒启动、退出或被强杀都会让 Cockpit 同时丢图。
5. **lifecycle 编排只尝试一次。** state machine 遇到晚出现的 camera/jtop lifecycle service 会 `continue`，进入 TELEOP/AUTONOMOUS 后不会持续收敛。
6. **jtop host daemon、容器节点和 lifecycle 三个条件没有联合 readiness。** 安装脚本只确认 `jtop` 命令存在，没有固化 daemon enabled/active、socket 可访问、ROS 节点 active 和 topic fresh 的完整验收。
7. **frontend 把“WebSocket 在线”近似成“数据源正常”。** camera 没有首帧超时/陈旧帧检测，Health 也没有数据 freshness watchdog；解析代码还存在固定 jtop 版本回退，可能显示看似合理但并非现场真实的数据。
8. **相机 launch 仍有 front-only 残留。** 无后摄时仍启动 back scan，压缩节点仍创建 back publisher 并订阅错误 namespace 的绝对 topic。这不是前摄消失的首要根因，但会制造噪声。

推荐方案是: **一个可重复构建的 release + 一个明确创建完整基础栈的 systemd unit + 独立 camera service + camera/jtop 各自唯一 lifecycle owner + readiness/healthcheck + 每机本地配置 + 分层诊断脚本 + 多轮冷启动验收。**

## 3. 两条真实数据链路

### 3.1 Cockpit 摄像头

```text
D435i USB/UVC
  -> host USB 与 /dev/video* 枚举
  -> 唯一 camera owner container
  -> realsense2_camera_node
  -> /g1_robot/front_camera lifecycle active
  -> /g1_robot/front_camera/color/image_raw 有新帧
  -> realsense_compressed_node lifecycle active
  -> /g1_robot/compressed_camera 有新帧
  -> bringup 中 rosbridge_websocket :9090
  -> browser ROSLIB subscription
  -> /cockpit 显示实时图像
```

### 3.2 Health/jtop

```text
Jetson kernel/sysfs/NVML/tegrastats
  -> host jetson-stats/jtop daemon 或其 IPC
  -> /run 映射和容器访问权限
  -> g1_robot_jetson_stats container
  -> /g1_robot/jtop_publisher lifecycle active
  -> /g1_robot/diagnostic_stats 有新消息
  -> bringup 中 rosbridge_websocket :9090
  -> browser ROSLIB subscription + parser
  -> /health 显示真实且新鲜的 Jetson 指标
```

`http://<robot-ip>` 可访问或 `9090` 可连接，只证明链路的一部分正常，不能证明 camera/jtop 数据源正常。

## 4. 当前证据与根因分级

### 4.1 P0: `src/` 与实际 installed 运行产物不一致，已证实

当前静态检查:

| 文件 | `src` 时间 | `install` 时间 | SHA-256 是否相同 |
|---|---:|---:|---|
| `realsense.launch.py` | 2026-07-09 19:05 | 2026-07-06 22:15 | 否 |
| `compressed_realsense.py` | 2026-07-06 22:16 | 2026-07-06 22:15 | 是 |
| `ros2_jtop_node.py` | 2026-07-09 19:05 | 2026-07-06 22:15 | 否 |
| `camera_config.yaml` | 2026-07-09 19:05 | 2026-07-06 22:15 | 否 |

其中:

- 当前 source `localization_3d.launch.py` include RealSense，installed 版本未检出该 include。
- source jtop 已有 lifecycle/publisher 竞态防护，installed 版本仍是旧哈希。
- source camera serial 已改为空并准备由环境覆盖，installed 配置仍不同。

影响是“Git 已修复”不等于“机器人正在运行修复后的代码”。不同机器只要 build 时间、容器创建时间或 bind mount 内容不同，就会产生不同现象。

优化要求:

- `src/` 是唯一事实来源，`build/install/log` 是可删除、可重建产物。
- CI 或部署脚本必须执行干净构建，并输出 Git SHA、镜像 digest、关键 installed 文件哈希。
- 生产容器启动前运行 release manifest 校验；不允许靠 `docker exec cp` 长期修补。
- 开发环境可用 `colcon build --symlink-install`；生产 release 更适合在固定镜像/制品中构建并验证，不在每次开机时构建。

### 4.2 P0: systemd 没有创建完整基础栈，已证实

当前 `botbrain.service` 启动:

```text
dev bringup rosa jetson_stats state_machine
```

它没有启动当前相机 owner `localization`，也没有启动 `web_server_prod`。当前 `web_server.service` 启动 `web_server_prod`，但停止的却是 `web_server`。

另外，`restart: always` 只能重启已经创建过的容器，不能在新机器、执行过 `compose down` 或服务新增后自动创建容器。因此 g1hk “能自动起来”可能只是旧容器仍存在，不能证明 systemd 模板完整。

优化要求:

- 用一个明确的 `botbrain-stack.service` 创建最低可用基础栈: `zenoh bringup camera jetson_stats state_machine web_server_prod`。
- navigation/localization/yolo/manipulation 作为第二阶段或独立 unit，不得阻塞 camera/health。
- `ExecStartPre` 必须执行 `docker compose config -q`、本地配置校验和 release manifest 校验。
- `ExecStop` 与 `ExecStart` 服务集合一致，使用足够的 camera stop grace period。
- 不在生产开机栈启动交互式 `dev` 容器。

### 4.3 P0: Compose 文件未建立版本兼容门槛，当前环境已复现

当前 `docker-compose.yaml` 使用:

```yaml
env_file:
  - path: ${BOTBRAIN_ROBOT_ENV_FILE:-/etc/botbrain/robot.env}
    required: false
```

当前环境执行 `docker compose config` 返回:

```text
services.nav3d.env_file.0 must be a string
```

这说明仓库语法和本机 Compose 插件版本不匹配。不同 G1 的 Compose 版本不一致时，systemd 会在配置解析阶段失败；已经存在的旧容器可能继续运行，进一步造成“有的服务有、有的没有”的错觉。

二选一实施，不要保持隐含兼容:

1. 推荐: 在 `install.sh`/部署 preflight 中强制最低 Docker Compose 版本，并记录版本。
2. 兼容旧现场: 将 `env_file` 改为各版本都支持的字符串形式，同时由部署脚本保证 `/etc/botbrain/robot.env` 存在且权限正确。

无论选择哪种方案，`docker compose config -q` 都必须是部署和开机前置门禁。

### 4.4 P1: camera 与 localization 耦合，已证实

当前 `localization`:

- 继承 `nav3d` 重镜像。
- 先固定 `sleep 30`。
- 运行 `g1_pkg localization_3d.launch.py`，再间接创建 RealSense。
- 与 Open3D、map/localization 等更重、更容易重启的能力共享故障域。

已有现场文档记录 `localization Exited (137)`、RealSense `Device or resource busy`，以及 Docker stop 后被 SIGKILL 的过程。相机跟随 localization 退出是结构性结果，不是浏览器刷新可以修复的问题。

优化要求:

- 新增独立 `camera` Compose service，只启动 `realsense.launch.py` 和必要的压缩链路。
- 从 `localization_3d.launch.py` 去掉 RealSense include，保证 D435i 全局只有一个 owner。
- localization、AprilTag、YOLO、navigation 只订阅 camera topic。
- camera 容器设置 `stop_grace_period: 30s`；恢复脚本显式 `docker compose stop -t 30 camera`。

### 4.5 P1: lifecycle 对冷启动晚到节点不能持续收敛，已证实

当前 `StateController::bring_up()` 对非 core/navigation 节点:

```cpp
auto cur = get_node_state(n.name);
if (!cur) continue;
```

state machine 启动后很快开始 bring-up，而 localization 至少晚 30 秒。若 camera 或 jtop lifecycle service 当时不存在，它们被跳过；状态机随后进入 TELEOP/AUTONOMOUS，不会因为节点稍后出现而重新走完整 bring-up。

优化要求:

- camera/jtop 每个 lifecycle 节点只有一个 owner。
- owner 对 required 节点使用 deadline + 指数退避/有上限重试 + 明确失败状态。
- 节点晚出现、容器重启或从 `active` 掉线后，manager 能再次收敛到 `active`。
- 不同时保留 state machine、launch event handler、shell lifecycle guard 三个 owner。
- `front_camera` active 后且 raw topic fresh，才激活 compressed 节点；不要只按固定秒数排序。

可选择两种实现:

| 方案 | 优点 | 约束 |
|---|---|---|
| 修复现有 state machine，作为 camera/jtop 唯一 owner | 保留现有统一状态展示与命令入口 | 必须增加持续 reconcile、required/optional、deadline、restart 后重收敛 |
| camera/jtop 各自使用专用 lifecycle manager | 边界简单，基础感知不依赖全局状态机 | 必须从 `camera.json`、`accessories.json` 移除对应节点，全局 state machine 只观察 |

推荐短期修现有 state machine，降低迁移范围；中期如果全局状态机继续被导航故障扰动，再拆专用 manager。

### 4.6 P1: jtop 缺少 host-to-topic 联合 readiness，部分已证实

当前事实:

- `install.sh` 只安装或检测 `jtop` 命令，没有验证对应 host daemon 是否 enabled/active。
- Compose `base` 映射了 `/run:/run` 并使用 privileged，给 jtop IPC 提供了条件，但没有 readiness 检查。
- `jetson_stats` 节点是 lifecycle node，仍依赖 state machine 激活。
- source `ros2_jtop_node.py` 已增强 cleanup/publisher 防护，但 installed 版本不同。

因此 Health 缺数据至少有四种不同失败:

1. host `jetson-stats` 安装/daemon/IPC 异常。
2. container 无权或无法访问 host IPC。
3. ROS jtop node configure 失败或进程重启。
4. lifecycle 未 active，或 topic 已停止更新。

优化要求:

- 安装阶段验证 `jtop --version`、daemon unit、enabled/active 和一次非交互 API 读取。
- 开机 unit 对 host jtop daemon 使用真实 unit 名的 `Requires=`/`After=`；部署脚本先探测并固化 unit 名，不能运行时模糊猜测。
- `jetson_stats` healthcheck 同时检查 node active 与 `/g1_robot/diagnostic_stats` freshness。
- jtop configure 异常要保留具体 exception，不应无限快速重启。
- 恢复 jtop 不得顺手重启 camera/localization/state machine，除非证据表明 lifecycle owner 自身故障。

### 4.7 P2: front-only camera launch 仍有残留，已证实

当前代码在 `back.type: ""` 时仍:

- 无条件创建 `depthimage_to_laserscan_back`。
- 无条件创建 `compressed_back_camera` publisher。
- 订阅绝对 `/back_camera/color/image_raw`，与 `g1_robot` namespace 不一致。
- `realsense.launch.py` 读取 `child`，而 YAML 字段名是 `child_frame`。

优化要求:

- back camera 的 driver、scan、subscription 和 publisher 全部条件创建。
- 全部 ROS topic 使用“相对 topic + node namespace”或统一的显式绝对 topic，不混用。
- `child_frame` 字段读写一致，并对 YAML schema 做启动前校验。
- 压缩链路优先评估 ROS `image_transport` 标准插件；如保留 OpenCV 节点，要补 QoS、CPU、异常和 freshness 测试。

### 4.8 P2: frontend 只表达连接，不表达数据健康，已证实

当前 camera hook 在 rosbridge 在线时订阅；没有首帧 deadline，也没有“最后一帧时间”。若 topic 无 publisher，页面可能一直 loading。Health hook 只有收到并成功解析消息后才把 `isConnected` 设为 true，但数据停止后没有超时回落。

另有两个需要清理的误导点:

- `useRosJetsonDiagnostics.ts` 中 jtop version 有固定 `'4.3.2'` fallback；Health 应显示 `unknown/stale`，不能把默认版本当真实遥测。
- robot profile 内有带 `g1_robot/` 的 topic，但通用 topic factory 又统一加 namespace；当前主 hook 走通用常量，后续若切 profile resolver，可能形成双 namespace。topic 名称应只有一个解析入口。

frontend 应区分以下状态:

| 状态 | Camera 文案/行为 | Health 文案/行为 |
|---|---|---|
| rosbridge offline | ROS connection unavailable | ROS connection unavailable |
| bridge online, no publisher | Camera source unavailable | Diagnostics source unavailable |
| publisher present, no first message | Waiting for first frame | Waiting for first sample |
| 曾有数据但超时 | Camera stream stale | Diagnostics stale |
| parse/decode error | Frame decode failed | Diagnostics format incompatible |
| fresh | 显示画面与 last update | 显示真实数据与 last update |

建议 camera 以 5 秒无首帧、3 秒无新帧作为初始阈值；jtop 以 `max(3 * publish_interval, 10s)` 作为 stale 阈值。阈值应配置化，并在低带宽现场验证。

## 5. 推荐目标架构

```text
systemd
  botbrain-stack.service
    preflight: compose config + robot.env + release manifest + host jtop
    docker compose up -d --remove-orphans
      zenoh
      bringup          -> rosbridge :9090
      camera           -> D435i + raw + compressed
      jetson_stats     -> Jetson diagnostics
      state_machine    -> one lifecycle owner/reconciler
      web_server_prod  -> frontend :80

  botbrain-navigation.service (可选、后启动)
      fast_lio localization navigation yolo ...
```

基础栈的健康不依赖导航栈。推荐依赖关系:

```text
docker ready
  -> zenoh ready
  -> bringup/rosbridge ready
  -> camera process + lifecycle + topic ready
  -> jetson_stats process + lifecycle + topic ready
  -> frontend process ready

navigation stack
  -> 可依赖 camera topic ready
  -> 失败不反向停止基础栈
```

Compose `depends_on` 只能辅助表达启动顺序，不能替代业务 readiness。camera/jtop 的真正 ready 条件必须落在 lifecycle state 和新鲜 topic 上。

## 6. 每机配置标准

建议每台 G1 都存在 root-owned 配置:

```text
/etc/botbrain/robot.env
```

最少包含:

```bash
BOTBRAIN_ROBOT_ID=g1edu
BOTBRAIN_ROBOT_MODEL=g1
BOTBRAIN_ROS_NAMESPACE=g1_robot
BOTBRAIN_FRONT_D435I_SERIAL=243722074823
BOTBRAIN_ROSBRIDGE_PORT=9090
```

规则:

- `g1edu`、`g1hk` 的代码和镜像相同，只允许 `robot.env` 不同。
- serial 必须由人工对照机身标签、`/dev/v4l/by-id` 和停掉 owner 后的 RealSense 枚举结果确认。
- 环境变量 serial 必须按字符串传给 ROS，不能让纯数字被推断为整数。
- `robot.env` 权限建议 `root:root 0640`，诊断输出要对潜在 secret 做脱敏。
- repository 的 `camera_config.yaml` 不写任何真实机器 serial，只保留空默认值与通用参数。
- frontend namespace 不应同时硬编码在 profile topic 和 namespace helper 中。

部署 preflight 必须比较三处值:

1. host 实际 D435i serial。
2. `/etc/botbrain/robot.env` 目标 serial。
3. `docker compose config` 渲染后 camera 容器实际环境。

三者不一致时只报告并退出，不自动 patch。

## 7. systemd 与 Compose 实施规划

### 7.1 systemd

建议替换现有两个松散 unit，或明确让 web unit `PartOf=` 基础栈。核心语义示例:

```ini
[Unit]
Description=BotBrain G1 base runtime
Requires=docker.service
After=docker.service network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/data/botbrain_ws/botbrain_ws_aitech
Environment=BOTBRAIN_ROBOT_ENV_FILE=/etc/botbrain/robot.env
ExecStartPre=/usr/bin/docker compose config -q
ExecStartPre=/data/botbrain_ws/botbrain_ws_aitech/mxscripts/check_boot_preflight.sh
ExecStart=/usr/bin/docker compose up -d --remove-orphans zenoh bringup camera jetson_stats state_machine web_server_prod
ExecStartPost=/data/botbrain_ws/botbrain_ws_aitech/mxscripts/wait_base_runtime_ready.sh
ExecStop=/usr/bin/docker compose stop -t 30 web_server_prod state_machine jetson_stats camera bringup zenoh
TimeoutStartSec=180
TimeoutStopSec=90

[Install]
WantedBy=multi-user.target
```

注意:

- 示例路径要由安装器根据现场唯一安装目录渲染，不能保留 `BOTBRAIN_WORKSPACE_PATH` 占位符。
- `wait_base_runtime_ready.sh` 只等待并返回状态，不在其中 patch 或无限重启。
- 如果 readiness 失败，unit 明确失败并保留日志；容器级 restart policy 处理进程崩溃，外部 watchdog 处理持续不健康。
- 网络默认路由不应是 camera/jtop 启动的硬依赖。即使外网不可用，本地 USB、ROS、Cockpit/Health 仍应启动；只等待需要的本地接口或 Docker。

### 7.2 Compose

计划修改:

1. 新增独立 `camera` service。
2. 将 `apt-get update/install` 从所有启动 command 移到镜像构建期。
3. camera 增加 `stop_grace_period: 30s`、明确的 `/dev` 访问、robot env 和 healthcheck。
4. jetson_stats 增加 host jtop IPC/权限说明及 healthcheck。
5. 给服务增加 release labels，如 Git SHA、build timestamp、image digest。
6. 固定 Compose 最低版本，或改为兼容语法。
7. `web_server_prod` healthcheck 验证 HTTP；bringup healthcheck 验证 9090 listening，但二者不代替 topic readiness。

禁止在开机 command 中运行 `apt-get update` 的理由:

- 引入公网、DNS、APT lock 和镜像仓库状态依赖。
- 每个容器并发安装相同包，放大 Jetson CPU、内存、磁盘和网络压力。
- 让启动时间不可预测，加剧 lifecycle 时序竞态。
- 同一镜像在不同日期得到不同运行依赖，不可复现。

## 8. lifecycle 收敛策略

camera/jtop manager 对每个 required node 使用如下状态机:

```text
wait get_state service (deadline)
  -> UNCONFIGURED: configure
  -> INACTIVE: activate
  -> ACTIVE: verify topic freshness
  -> FINALIZED/ERROR: report, do not loop transitions blindly
  -> service disappears: mark unavailable, retry with bounded backoff
```

建议参数:

```text
service discovery deadline: 60s
configure timeout:           30s (RealSense 可适当放宽)
activate timeout:            20s
retry backoff:               1s, 2s, 4s, 8s, max 10s
total base readiness:        120s
steady reconcile interval:   5s
```

每次 transition 后必须重新 `get_state` 验证，不能只相信 service call 返回成功。topic freshness 是 active 之后的第二层验证。

依赖建议:

```text
front_camera
  -> raw image has publisher and receives frame
  -> realsense_compressed_node
  -> compressed image receives frame

host jtop ready
  -> jtop_publisher
  -> diagnostic_stats receives sample
```

## 9. `mxscripts/` 诊断与恢复脚本规划

当前仓库没有 `mxscripts/`。实施阶段新建该目录，并将脚本分为只读检查、有限恢复、深度诊断三类。

### 9.1 通用脚本规范

所有脚本必须:

- 使用 `#!/usr/bin/env bash` 和 `set -uo pipefail`；对预期可能失败的探测单独处理，不因一个工具缺失丢掉全部证据。
- 默认从仓库根目录自动定位 Compose 文件，也允许 `BOTBRAIN_ROOT` 显式覆盖。
- 读取 `BOTBRAIN_ROBOT_ENV_FILE`，默认 `/etc/botbrain/robot.env`。
- namespace 默认从配置解析，最后才回退 `g1_robot`；不在多处复制固定 topic。
- 每一段输出带 UTC/本地时间、hostname、robot id、Git SHA、Compose version。
- 输出人可读摘要，同时支持 `--json` 供远程运维收集。
- 退出码统一: `0=健康`、`1=功能异常`、`2=配置/参数错误`、`3=依赖工具或权限不足`、`4=恢复执行但验证失败`。
- 有总超时，不允许无限等待。
- 不输出 Wi-Fi 密码、API key、token 等 secret；环境打印使用 allowlist。
- 不用 `sudo` 静默提权。需要 root 时先检查 EUID 并明确退出。
- 日志建议保存到 `/var/log/botbrain/diagnostics/<timestamp>/`，同时在终端给出 evidence directory。

### 9.2 Camera 三脚本

#### `check_cockpit_camera.sh`

只读、日常首选，不重启、不改配置。

检查顺序:

1. `docker compose config -q` 和 camera service 是否存在。
2. `camera` 容器状态、restart count、health、最近日志关键错误。
3. 9090 是否监听，rosbridge 节点是否存在。
4. `/g1_robot/front_camera` 与 compressed node 是否存在及 lifecycle state。
5. raw/compressed topic 的 type、publisher count、subscriber count。
6. 在限定时间内各接收一条消息，并计算 header age；可选测 5 秒 hz。
7. host `lsusb` 和 `/dev/v4l/by-id` 是否有 RealSense，只读查看，不调用可能抢设备的流式工具。
8. serial env 与 by-id serial 是否一致。

输出最终分型码，例如:

```text
CAM-BRIDGE-DOWN
CAM-CONTAINER-ABSENT
CAM-USB-ABSENT
CAM-NODE-ABSENT
CAM-LIFECYCLE-INACTIVE
CAM-RAW-NO-FRAME
CAM-COMPRESSED-NO-FRAME
CAM-OK
```

#### `recover_camera_runtime.sh`

有副作用，只恢复 camera runtime，不 patch 配置、不碰 jtop/localization/navigation。

执行流程:

```text
precheck and evidence
  -> refuse if serial mismatch or multiple owners detected
  -> docker compose stop -t 30 camera
  -> verify no owner still holds related /dev/video* nodes
  -> docker compose up -d --force-recreate camera
  -> wait lifecycle services
  -> configure/activate idempotently through the single owner policy
  -> verify raw then compressed fresh messages
  -> write before/after summary
```

若 state machine 是唯一 owner，脚本应请求/等待 state machine 收敛，不能直接与它并发发 lifecycle transition。只有在明确的 `--manual-lifecycle` break-glass 模式下才直接 transition，并记录该操作。

脚本不得自动 USB reset。USB reset、重新插拔、整机重启属于人工升级处置。

#### `diagnose_realsense_usb.sh`

深度只读诊断，优先在 camera owner 停止前保存现场；默认不停止 owner。

收集:

- `lsusb -t`、RealSense VID:PID、USB speed/topology。
- `/dev/v4l/by-id`、symlink target、owner/group/mode。
- 只针对 RealSense 对应 video nodes 执行 `fuser -v`/`lsof`，不要泛杀 `/dev/video*` 使用者。
- `udevadm info` 与 device serial。
- `dmesg --since` 中 USB/UVC/xHCI/reset/disconnect/error；无权限时明确标记。
- Docker 容器 pid、设备映射和可能的重复 RealSense 进程。
- allowlist serial env、source/install config 哈希。
- camera owner 未运行时才允许可选 `--enumerate` 调用 `rs-enumerate-devices -s`；owner 正在采流时跳过，避免把 `RS2_USB_STATUS_BUSY` 误判为硬件消失。

### 9.3 jtop 三脚本

#### `check_health_jtop.sh`

只读、日常首选。

检查:

1. host `jtop`/`jetson-stats` 版本。
2. 探测到的 host daemon unit 是否 enabled、active，最近是否反复失败。
3. jtop IPC/socket 是否存在及 container 是否可见。
4. `jetson_stats` 容器状态、restart count、health 和最近异常。
5. `/g1_robot/jtop_publisher` lifecycle state。
6. `/g1_robot/diagnostic_stats` publisher count、一次消息、消息 age 和解析基本结构。
7. 9090/rosbridge 是否正常。

分型码:

```text
JTOP-HOST-NOT-INSTALLED
JTOP-HOST-DAEMON-DOWN
JTOP-IPC-UNAVAILABLE
JTOP-CONTAINER-DOWN
JTOP-CONFIGURE-FAILED
JTOP-LIFECYCLE-INACTIVE
JTOP-TOPIC-STALE
JTOP-BRIDGE-DOWN
JTOP-OK
```

#### `recover_jtop_runtime.sh`

有副作用，但只处理 jtop 链路:

```text
precheck and evidence
  -> start/restart known host jtop daemon only when explicitly requested
  -> docker compose up -d --force-recreate jetson_stats
  -> request/wait lifecycle owner convergence
  -> verify diagnostic_stats fresh sample
```

默认不重启 state machine；若唯一 owner 自身不健康，脚本应报告 `OWNER-UNHEALTHY` 并让运维单独处置，避免一个 jtop 问题 bring-down 全部 lifecycle 节点。

#### `diagnose_jtop_host.sh`

深度只读诊断:

- JetPack/L4T/kernel、Python 和 jetson-stats/jtop 版本矩阵。
- host daemon unit file、status、最近 journal。
- IPC path、mode、uid/gid、容器内可见性。
- 容器中 import `jtop` 的 Python 路径和包版本。
- `ros2_jtop_node.py` source/install 哈希。
- configure 失败 exception、container restart history。
- CPU/memory/disk pressure，帮助识别系统资源问题，但不自动清缓存或 kill 进程。

### 9.4 跨链路只读脚本

#### `check_boot_preflight.sh`

systemd `ExecStartPre` 使用，快速失败:

- workspace/Compose 文件存在。
- `docker compose version` 满足最低版本。
- `docker compose config -q` 成功。
- `/etc/botbrain/robot.env` 存在、权限合规、必需键非空。
- release manifest 与 installed 文件匹配。
- host D435i 枚举与配置 serial 一致。
- host jtop daemon ready。
- 必需端口没有被非预期进程占用。

#### `wait_base_runtime_ready.sh`

只等待、验证并退出，不修复:

- rosbridge listening。
- camera/jtop lifecycle active。
- compressed image 和 diagnostic topic fresh。
- frontend HTTP ready。

#### `collect_boot_evidence.sh`

统一调用所有只读检查并收集:

- `systemctl status/show`、本次 boot journal。
- `docker compose ps`、inspect、events、限定行数日志。
- ROS node/lifecycle/topic graph。
- USB/jtop host 证据。
- release/config 哈希和已脱敏环境。

该脚本不调用任何 `recover_*`。

## 10. 快速故障分型

| 现象 | 定位层 | 首选脚本 | 后续动作 |
|---|---|---|---|
| HTTP 80 不通 | frontend/container | `collect_boot_evidence.sh` | 单独恢复 web service |
| 80 通、9090 不通 | bringup/rosbridge | 两个 `check_*` | 检查 bringup，不重启 camera/jtop |
| 9090 通，camera node absent | compose/launch/install | `check_cockpit_camera.sh` | 查 camera container 和 release |
| camera node unconfigured/inactive | lifecycle owner | `check_cockpit_camera.sh` | 查 owner，再有限恢复 |
| front active，raw 无帧 | RealSense/USB/serial | `diagnose_realsense_usb.sh` | 处理 USB、占用或 serial |
| raw 有帧，compressed 无帧 | compressed node/QoS | `check_cockpit_camera.sh` | 只恢复 camera pipeline |
| ROS compressed 有帧，Cockpit 无图 | rosbridge/topic/frontend decode | browser console + camera check | 修 frontend/bridge，不碰 USB |
| jtop node absent | container/install | `check_health_jtop.sh` | 查 jetson_stats container |
| jtop configure 失败 | host daemon/IPC/version | `diagnose_jtop_host.sh` | 修 host/IPC 后恢复 jtop |
| jtop active，topic stale | callback/process/resource | `check_health_jtop.sh` | 保存日志，单独恢复 jtop |
| topic fresh，Health 无数据 | namespace/parser/frontend | browser console + jtop check | 修 frontend，不重启 daemon |
| 两者同时消失，9090 也不通 | bringup/ROS transport | `collect_boot_evidence.sh` | 优先修 zenoh/bringup |
| 两者同时消失，9090 正常 | lifecycle owner/install/base stack | evidence + lifecycle states | 查 state machine/release |

## 11. 分阶段实施清单

### Phase 0: 基线和可重复部署，P0

涉及文件:

```text
install.sh
docker-compose.yaml
botbrain.service / 新 botbrain-stack.service
web_server.service
新增 release manifest/deploy preflight
```

任务:

1. 决定并强制 Compose 最低版本，先让 `docker compose config -q` 在两台机器通过。
2. 清理并重建 `build/install/log`，验证关键 source/install 哈希或改为镜像内固定制品。
3. 停止在容器启动时 `apt-get update/install`，依赖进入镜像。
4. systemd 显式创建完整基础栈，修复 web stop 服务名。
5. 每台机器建立 `/etc/botbrain/robot.env`，记录但不提交真实 serial。
6. 记录 g1edu/g1hk 的 Docker、Compose、JetPack、jtop、D435i firmware 基线。

完成标准: 两台机器执行相同部署命令得到相同 Git SHA/镜像 digest，只有 robot.env 内容不同。

### Phase 1: 拆 camera 与可靠 lifecycle，P0/P1

涉及文件:

```text
docker-compose.yaml
botbrain_ws/src/g1_pkg/launch/localization_3d.launch.py
botbrain_ws/src/bot_localization/bot_localization/launch/realsense.launch.py
botbrain_ws/src/bot_localization/bot_localization/scripts/compressed_realsense.py
botbrain_ws/src/bot_state_machine/config/camera.json
botbrain_ws/src/bot_state_machine/src/state_controller.cpp
```

任务:

1. 新增 `camera` service。
2. localization 删除 RealSense include，验证进程列表中只有一个 RealSense owner。
3. lifecycle owner 增加 late join、bounded retry、steady reconcile 和 topic readiness。
4. front-only 条件创建、namespace、`child_frame` 和 QoS 修复。
5. 增加 camera 单元/launch 测试以及无 back camera 测试。

完成标准: 单独重启 localization/navigation 不影响 `/g1_robot/compressed_camera` 连续发布。

### Phase 2: jtop host-to-topic 闭环，P1

涉及文件:

```text
install.sh
docker-compose.yaml
botbrain_ws/src/bot_jetson_stats/...
botbrain_ws/src/bot_state_machine/config/accessories.json
```

任务:

1. 固化 host daemon 安装、enable 和 readiness。
2. 验证 IPC 映射和权限，固定 host/container 包版本兼容矩阵。
3. jtop lifecycle 加入持续收敛，但保持唯一 owner。
4. healthcheck 使用 topic freshness，不只检查进程存在。
5. 测试 host daemon 晚启动、container 重启、cleanup/activate 循环。

完成标准: 单独重启 camera/localization 不影响 diagnostics；单独重启 jetson_stats 能在 deadline 内恢复 fresh topic。

### Phase 3: frontend 可观测性，P1/P2

涉及文件:

```text
frontend/src/hooks/ros/useCameraStream.tsx
frontend/src/hooks/ros/useRobotCamera.tsx
frontend/src/hooks/ros/useRosJetsonDiagnostics.ts
frontend/src/utils/ros/namespace.ts
frontend/src/config/robot-profiles/...
Cockpit/Health 对应组件
```

任务:

1. camera/jtop 保存 `lastMessageAt`，实现 first-message timeout 和 stale timeout。
2. WebSocket reconnect 后明确 unsubscribe/resubscribe，并清除上一连接的陈旧数据。
3. 页面区分 bridge、publisher、message、decode/parse 状态。
4. 去掉固定 jtop 版本等伪真实 fallback。
5. topic/name namespace 统一从一个 resolver 获取。
6. frontend 显示 last update 和可复制的短错误码，错误码对应本文章节和脚本。

完成标准: topic 停止后页面在阈值内明确显示 stale；topic 恢复后无需刷新页面自动恢复。

### Phase 4: 脚本与自动验收，P1

涉及文件:

```text
mxscripts/check_cockpit_camera.sh
mxscripts/recover_camera_runtime.sh
mxscripts/diagnose_realsense_usb.sh
mxscripts/check_health_jtop.sh
mxscripts/recover_jtop_runtime.sh
mxscripts/diagnose_jtop_host.sh
mxscripts/check_boot_preflight.sh
mxscripts/wait_base_runtime_ready.sh
mxscripts/collect_boot_evidence.sh
```

任务:

1. 先落地所有只读脚本和 shell 静态检查。
2. 用正常、USB 拔出、serial 错误、daemon 停止、lifecycle inactive 等故障注入验证分型。
3. 再落地两个有限恢复脚本，测试幂等性与拒绝危险状态。
4. CI 使用 ShellCheck/Bats；G1 实机测试输出作为发布附件。

## 12. 冷启动与故障注入验收

### 12.1 单次冷启动门槛

从断电状态开机计时，建议初始 SLO:

| 指标 | 门槛 |
|---|---:|
| Docker/Compose 基础栈已创建 | 60s 内 |
| rosbridge 接受连接 | 90s 内 |
| front camera lifecycle active | 120s 内 |
| compressed camera 第一帧 | 130s 内 |
| jtop lifecycle active | 120s 内 |
| diagnostic_stats 第一条 | 130s 内 |
| frontend HTTP ready | 90s 内 |
| Cockpit/Health 均显示 fresh data | 150s 内 |

SLO 必须根据两台实机的 P95 数据调整，不能用更长固定 sleep 掩盖异常。

验收命令的最小语义:

```bash
docker compose ps
ros2 lifecycle get /g1_robot/front_camera
ros2 lifecycle get /g1_robot/realsense_compressed_node
ros2 lifecycle get /g1_robot/jtop_publisher
timeout 10 ros2 topic hz /g1_robot/compressed_camera
timeout 10 ros2 topic echo --once /g1_robot/diagnostic_stats
curl -fsS http://127.0.0.1/
```

### 12.2 重复冷启动

至少执行:

- g1edu: 10 次完整断电冷启动。
- g1hk: 10 次完整断电冷启动。
- 每次保存 `collect_boot_evidence.sh` 结果和 ready 时间。
- 20 次必须全部成功；任何一次靠人工 lifecycle set、container restart 或浏览器刷新才恢复都算失败。

### 12.3 故障注入

逐项验证:

1. D435i 未连接: camera 明确 degraded，jtop/Health 仍正常。
2. serial 故意配置错误: preflight/diagnose 明确 `SERIAL-MISMATCH`，不自动改值。
3. camera container kill: restart + lifecycle owner 在 deadline 内恢复。
4. localization container kill/restart: camera 流不中断。
5. host jtop daemon 晚启动: Health 先显示 source unavailable，daemon 恢复后自动变 fresh。
6. jetson_stats container kill: camera 不受影响，diagnostics 自动恢复。
7. rosbridge restart: 两个 frontend hook 自动重订阅，无需刷新。
8. browser 在服务启动前打开: 服务 ready 后页面自动恢复。
9. 无外网/DNS: 本地 camera、jtop、frontend 仍能启动。
10. 磁盘接近满、内存压力: healthcheck 失败要可解释，不允许无界 restart storm。

## 13. 发布、回滚与现场安全

发布顺序:

1. 备份每机 `robot.env`、systemd unit、Compose 渲染结果和当前 release manifest。
2. 在非关键机器先部署 Phase 0，只解决可重复运行和完整自启动。
3. 再部署独立 camera，部署窗口内确认只有一个 RealSense owner。
4. 通过 10 次冷启动后推广到第二台机器。
5. frontend freshness 改造最后发布，但不能用 UI 改造替代后端 readiness。

回滚单位应是完整 release，而不是单个 installed 文件:

```text
Git SHA + image digest + generated install artifact + compose + systemd + schema version
```

现场安全规则:

- 任何恢复前先保存证据。
- `Device or resource busy` 时先查 owner，不先 kill 全部 video 进程。
- camera stop 使用 30 秒，SIGKILL 后必须确认设备已释放再启动。
- 不自动 `usbreset`、unbind xHCI、修改 udev 或重启整机。
- 不让 camera 恢复脚本重启 jtop、localization、navigation 或整个 Compose project。
- 不让 jtop 恢复脚本触发 G1 reboot service。

## 14. Definition of Done

只有同时满足以下条件，才能认为“开机后 camera/jtop 时有时无”已经解决:

1. `docker compose config -q` 在 g1edu、g1hk 和部署 CI 都通过，Compose 版本受控。
2. 两台机器运行同一 release manifest，source/install/容器代码不存在隐式分叉。
3. systemd 从空容器状态也能创建完整基础栈，不依赖历史容器。
4. D435i 只有独立 camera service 一个 owner，localization 重启不影响画面。
5. camera 和 jtop lifecycle 有唯一 owner，支持晚到和进程重启后的持续收敛。
6. camera raw/compressed 与 jtop diagnostics 均有 freshness healthcheck。
7. Cockpit/Health 能区分 bridge offline、source absent、inactive、stale 和 parse/decode error，并自动恢复。
8. `mxscripts/` 的只读检查、深度诊断和有限恢复脚本完成实机故障注入验证。
9. g1edu 与 g1hk 各 10 次断电冷启动全部在 SLO 内成功，无人工干预。
10. 运维人员只根据页面短错误码和对应脚本，就能在一次采证中确定故障层级。

## 15. 建议实施优先级

```text
P0  Compose 可解析 + release/install 一致 + systemd 完整创建基础栈
P0  独立 camera service，消除双 owner 和 localization 耦合
P1  camera/jtop lifecycle 持续收敛 + topic freshness
P1  六个分层 camera/jtop 脚本 + preflight/evidence
P1  frontend stale/错误分型与自动重订阅
P2  front-only launch、namespace、image transport 和资源优化
```

不要先写一个“全自动修复所有问题”的大脚本。先把部署、所有权、readiness 和证据链做确定，再让恢复脚本只处理已经准确分型的单一故障。

## 16. 行业实践参考

- ROS 2 Managed Nodes lifecycle design: <https://design.ros2.org/articles/node_lifecycle.html>
- ROS 2 managed node demo: <https://docs.ros.org/en/humble/Tutorials/Demos/Managed-Nodes.html>
- Intel RealSense ROS wrapper: <https://github.com/realsenseai/realsense-ros>
- Docker Compose startup order and health conditions: <https://docs.docker.com/compose/how-tos/startup-order/>
- Docker Compose `env_file` reference: <https://docs.docker.com/reference/compose-file/services/#env_file>
- systemd service dependencies and ordering: <https://www.freedesktop.org/software/systemd/man/latest/systemd.unit.html>
- systemd service restart/start-limit semantics: <https://www.freedesktop.org/software/systemd/man/latest/systemd.service.html>
- jetson-stats/jtop: <https://github.com/rbonghi/jetson_stats>
- ShellCheck: <https://www.shellcheck.net/>
- Bats Core: <https://github.com/bats-core/bats-core>

仓库内关联文档:

- `mxdocs/08-boot-jtop-d435i-root-cause-and-best-practice-20260708.md`
- `mxdocs/11-g1edu-g1hk-shared-code-per-robot-config-best-practice-20260709.md`
- `mxdocs/13-cockpit-d435i-camera-troubleshooting-best-practices-20260709.md`
- `mxdocs/14-g1edu-g1hk-d435i-boot-diff-analysis-20260709.md`
