# 将 BotBrainmx2 的 Health 与 Mission Control 整合进 botbrain_ws_aitech 的最佳实践方案

日期：2026-07-06  
范围：`BotBrainmx2/frontend`、`botbrain_ws_aitech/frontend`、`botbrain_ws_aitech/botbrain_ws`、Unitree G1 EDU 真机、Mission Supervisor / gateway 链路。

## 0. 结论

可以整合，而且推荐整合到 `botbrain_ws_aitech`，但必须按“前端能力迁移、任务控制契约复用、真机 ROS runtime 冻结”的方式做。

目标不是把 `BotBrainmx2` 的整套工程混入 `botbrain_ws_aitech`，也不是让前端直接接管建图、导航或灵巧手。推荐目标是：

```text
Browser
  -> botbrain_ws_aitech/frontend /health
  -> botbrain_ws_aitech/frontend /mission-control
  -> Next.js BFF proxy /api/mission-supervisor/*
  -> Mission Supervisor :8787
  -> botbrain_ws_gateway :8899, 如已有则复用
  -> 已经在 Unitree G1 EDU 真机上稳定的 botbrain_ws_aitech ROS 2 / Docker runtime
  -> Unitree G1 EDU
```

这样做之后，`botbrain_ws_aitech` 页面可以完全取代 `BotBrainmx2` 的 Mission Control 页面，但有两个前提：

1. `botbrain_ws_aitech/frontend` 必须补齐 `BotBrainmx2/frontend` 里 Mission Control 依赖的页面、context、service、types、API proxy 和菜单入口。
2. Mission Control 的所有任务写操作仍然只走 Mission Supervisor 的 `instruction`、`operator-decision`、`control`、`tick` 等契约，不直接发布 ROS topic，不直接调用 Unitree SDK2，不直接写 `/cmd_vel_out`、Nav2 action 或 Dex3 控制接口。

当前代码盘点显示：

| 项 | 现状 | 建议 |
|---|---|---|
| `health` 页面 | `BotBrainmx2/frontend/src/app/health` 与 `botbrain_ws_aitech/frontend/src/app/health` 已基本同源，依赖的 health hooks/components/types 也已存在 | 不需要大迁移，只做差异校验；不要为了 Health 改 ROS runtime |
| `mission-control` 页面 | `BotBrainmx2` 已有，`botbrain_ws_aitech` 缺失 | 迁移前端文件和 BFF proxy |
| `MissionSupervisorContext` | `BotBrainmx2` 已有，`botbrain_ws_aitech` 缺失 | 迁移 |
| `mission-supervisor` service/types | `BotBrainmx2` 已有，`botbrain_ws_aitech` 缺失 | 迁移 |
| `components/mission-control` | `BotBrainmx2` 已有 Playback/TestCase 面板，`botbrain_ws_aitech` 缺失 | 迁移 |
| 菜单入口 | `BotBrainmx2` 有 `missionControl` action 和 `ClipboardList` 入口，`botbrain_ws_aitech` 缺失 | 补齐 |
| `mockg1` | `BotBrainmx2/mockg1` 提供 rosbridge-compatible G1 mock 与 Mission Supervisor mirror | 迁移为本地开发/回归工具，不接入真机控制 |
| 真机建图/导航/灵巧手 | `botbrain_ws_aitech` 已有 `fast_lio`、`open3d_loc`、`bot_navigation`、`g1_pkg`、`g1_manipulation_pkg`、`g1_right_dex3` 等 | 冻结，不作为本次迁移改动对象 |

## 1. 必须坚持的架构原则

### 1.1 Mission Supervisor 是任务状态唯一来源

Mission Control 只显示和提交操作，不拥有任务真相：

```text
Mission Supervisor owns:
  mission_state
  phase
  current_waypoint
  completed_test_cases
  pending_decisions
  active_exception
  events
  preflight
  adapter-status

Frontend owns:
  rendering
  operator confirmation UX
  stale-state guard
  BFF proxy
  local base URL override
```

任何能改变任务状态的动作必须进入 Mission Supervisor 的契约入口：

```text
POST /instruction
POST /operator-decision
POST /control
POST /tick
```

不允许在 `/mission-control` 页面里直接做这些事情：

```text
publish /cmd_vel_out
publish /cmd_vel_nav
send /navigate_to_pose action directly
call /g1_robot/arm_cmd directly
call Dex3 or Unitree SDK2 directly
derive "mission step completed" from raw ROS topic in browser
```

### 1.2 botbrain_ws_aitech 的真机 runtime 必须保持稳定边界

本次整合不应修改下面这些已经在 G1 EDU 上稳定调试过的能力目录，除非另起专项变更并完成真机回归：

```text
botbrain_ws_aitech/botbrain_ws/src/g1_pkg/
botbrain_ws_aitech/botbrain_ws/src/fast_lio/
botbrain_ws_aitech/botbrain_ws/src/open3d_loc/
botbrain_ws_aitech/botbrain_ws/src/bot_navigation/
botbrain_ws_aitech/botbrain_ws/src/bot_bringup/
botbrain_ws_aitech/botbrain_ws/src/bot_state_machine/
botbrain_ws_aitech/botbrain_ws/src/g1_manipulation_pkg/
botbrain_ws_aitech/botbrain_ws/src/g1_right_dex3/
botbrain_ws_aitech/docker-compose.yaml
botbrain_ws_aitech/tools/gotop/
botbrain_ws_aitech/tools/nav/
```

允许新增的内容：

```text
botbrain_ws_aitech/frontend/src/app/mission-control/
botbrain_ws_aitech/frontend/src/app/api/mission-supervisor/
botbrain_ws_aitech/frontend/src/contexts/MissionSupervisorContext.tsx
botbrain_ws_aitech/frontend/src/services/mission-supervisor.ts
botbrain_ws_aitech/frontend/src/types/mission-control.ts
botbrain_ws_aitech/frontend/src/components/mission-control/
botbrain_ws_aitech/mockg1/
botbrain_ws_aitech/scripts/run-mockg1-bt-stack.sh
botbrain_ws_aitech/scripts/stop-mockg1-bt-stack.sh
botbrain_ws_aitech/scripts/run-36-flow-mock-validation.sh
botbrain_ws_aitech/mxdocs/*
```

如果尚未有 `botbrain_ws_gateway`，也可以新增一个 sidecar，但它应作为独立新增包或独立服务，不改已有导航、建图和灵巧手节点：

```text
botbrain_ws_aitech/botbrain_ws/src/botbrain_ws_gateway/
```

### 1.3 rosbridge / Foxglove 是遥测和调试，不是安全关键任务控制入口

`botbrain_ws_aitech` 已经有 ROS/rosbridge/Foxglove 相关前端能力，这些可以继续用于：

- 地图显示
- 摄像头显示
- 雷达/里程计/电池/diagnostics 显示
- 调试和开发

但 Mission Control 替代 `BotBrainmx2` 时，任务控制必须走：

```text
Mission Control
  -> /api/mission-supervisor/*
  -> Mission Supervisor
  -> adapter / gateway
  -> ROS action / service
```

这样才能保持审计、幂等、preflight、异常恢复、人工决策和回放一致。

### 1.4 与官方最佳实践的对应关系

本方案采用的工程边界与以下官方/主流实践一致：

- Next.js App Router 用 `route.ts` Route Handler 实现 BFF/proxy，避免浏览器直连任意内部服务。
- ROS 2 managed lifecycle 强调节点具有已知生命周期和由外部监督进程管理状态。
- Nav2 Lifecycle Manager 负责按确定顺序配置、激活和降级 Nav2 节点，并通过 bond 检测服务异常。
- Unitree ROS2 依赖 CycloneDDS、网卡配置和厂商消息，应该留在机器人 runtime 容器/ROS 环境中，不应进入 Next.js 或浏览器。

参考：

- Next.js Route Handlers: https://nextjs.org/docs/app/api-reference/file-conventions/route
- ROS 2 Managed Nodes: https://design.ros2.org/articles/node_lifecycle.html
- Nav2 Lifecycle Manager: https://docs.nav2.org/configuration/packages/configuring-lifecycle.html
- Unitree ROS2: https://github.com/unitreerobotics/unitree_ros2

## 2. 最终目标架构

### 2.1 运行拓扑

```text
botbrain_ws_aitech/frontend
  /health
    - Jetson/system health
    - ROS diagnostics
    - state_machine status
    - network / wifi status
    - 不拥有任务控制

  /mission-control
    - Mission Supervisor snapshot
    - preflight gate
    - adapter-status
    - pending decisions
    - exception and retry state
    - 36 test cases projection
    - playback / live voice observe, 如果后端支持
    - operator commands

  /api/mission-supervisor/[...path]
    - allowlist proxy
    - localhost/private IPv4 target only
    - SSE /stream passthrough
    - request timeout

Mission Supervisor :8787
  - fake/test/g1_botbrain_ws profiles
  - mission truth
  - audit/events
  - preflight
  - adapters

botbrain_ws_gateway :8899, 推荐或复用已有
  - FastAPI HTTP API
  - rclpy action/service/topic clients
  - action registry
  - health aggregation
  - evidence aggregation

botbrain_ws_aitech runtime
  - state_machine
  - bringup
  - fast_lio
  - localization/open3d_loc
  - navigation/Nav2
  - manipulation/Dex3
  - jetson_stats
  - foxglove/rosbridge telemetry
```

### 2.2 Mission Control 能完全替代 BotBrainmx2 的判定标准

`botbrain_ws_aitech` 可以关掉 `BotBrainmx2/frontend` 的前提：

| 能力 | 验收标准 |
|---|---|
| 页面入口 | `http://<robot-host>:3000/mission-control` 可打开，菜单可进入 |
| Supervisor 连接 | `/api/mission-supervisor/healthz`、`snapshot`、`stream` 正常 |
| 状态投影 | mission_state、phase、current_waypoint、active_exception、pending_decisions 与 BotBrainmx2 一致 |
| 36 case | `TestCaseMonitorPanel` 能基于 `/visualization/model` 和 snapshot 显示完成进度 |
| 决策 | operator decision 提交后 Supervisor 状态推进，前端不绕过 allowed_actions |
| Preflight | NO-GO 时不能批准启动真实任务 |
| Stop/Pause | `PAUSE`、`ABORT`、`TAKEOVER`、`RESYNC` 行为与 BotBrainmx2 一致 |
| Health | `/health` 原有 Jetson/ROS/system 监控不退化 |
| 真机功能 | 原建图、定位、导航、灵巧手脚本和容器启动方式不变，真机回归通过 |

## 3. 文件级迁移清单

### 3.1 Health 页面

当前 `health` 页面在两个工程中已经基本存在：

```text
BotBrainmx2/frontend/src/app/health/page.tsx
BotBrainmx2/frontend/src/app/health/layout.tsx
BotBrainmx2/frontend/src/components/health/*

botbrain_ws_aitech/frontend/src/app/health/page.tsx
botbrain_ws_aitech/frontend/src/app/health/layout.tsx
botbrain_ws_aitech/frontend/src/components/health/*
```

建议先做差异校验：

```bash
cd /Users/fausto/mdev/aitech/4g1edu

diff -ru \
  BotBrainmx2/frontend/src/app/health \
  botbrain_ws_aitech/frontend/src/app/health

diff -ru \
  BotBrainmx2/frontend/src/components/health \
  botbrain_ws_aitech/frontend/src/components/health
```

如果只有空白、格式、构建产物无关差异，则不要改。Health 已经是 `botbrain_ws_aitech` 的一部分。

如果发现 `BotBrainmx2` 有新 health 功能，只迁移对应的前端页面/hook/type，不修改 `botbrain_ws_aitech/botbrain_ws` 的 ROS 节点。

### 3.2 Mission Control 必须迁移的文件

从 `BotBrainmx2/frontend` 迁移到 `botbrain_ws_aitech/frontend`：

```text
src/app/mission-control/layout.tsx
src/app/mission-control/page.tsx
src/app/api/mission-supervisor/[...path]/route.ts
src/contexts/MissionSupervisorContext.tsx
src/services/mission-supervisor.ts
src/types/mission-control.ts
src/components/mission-control/PlaybackControlPanel.tsx
src/components/mission-control/TestCaseMonitorPanel.tsx
```

推荐命令：

```bash
cd /Users/fausto/mdev/aitech/4g1edu

mkdir -p botbrain_ws_aitech/frontend/src/app/mission-control
mkdir -p 'botbrain_ws_aitech/frontend/src/app/api/mission-supervisor/[...path]'
mkdir -p botbrain_ws_aitech/frontend/src/components/mission-control

cp BotBrainmx2/frontend/src/app/mission-control/layout.tsx \
   botbrain_ws_aitech/frontend/src/app/mission-control/layout.tsx

cp BotBrainmx2/frontend/src/app/mission-control/page.tsx \
   botbrain_ws_aitech/frontend/src/app/mission-control/page.tsx

cp 'BotBrainmx2/frontend/src/app/api/mission-supervisor/[...path]/route.ts' \
   'botbrain_ws_aitech/frontend/src/app/api/mission-supervisor/[...path]/route.ts'

cp BotBrainmx2/frontend/src/contexts/MissionSupervisorContext.tsx \
   botbrain_ws_aitech/frontend/src/contexts/MissionSupervisorContext.tsx

cp BotBrainmx2/frontend/src/services/mission-supervisor.ts \
   botbrain_ws_aitech/frontend/src/services/mission-supervisor.ts

cp BotBrainmx2/frontend/src/types/mission-control.ts \
   botbrain_ws_aitech/frontend/src/types/mission-control.ts

cp BotBrainmx2/frontend/src/components/mission-control/*.tsx \
   botbrain_ws_aitech/frontend/src/components/mission-control/
```

### 3.3 菜单入口必须补齐

`botbrain_ws_aitech/frontend/src/types/RobotActionTypes.ts` 需要给 `MenuActionTypeName` 增加：

```ts
| 'missionControl'
```

`botbrain_ws_aitech/frontend/src/hooks/useMenuActions.tsx` 需要在 `return` 中增加：

```ts
missionControl: {
  label: 'Mission Control',
  icon: `${getDarkModeFolder()}list`,
  action: () => navigate('/mission-control'),
},
```

`botbrain_ws_aitech/frontend/src/components/nav-menu.tsx` 需要：

1. 从 `lucide-react` 引入 `ClipboardList`。
2. 在 `iconMap` 增加：

```ts
'Mission Control': ClipboardList,
```

3. 在 `ossNavButtons` 中插入：

```ts
menuActions.missionControl,
```

推荐插在 `menuActions.home` 后面：

```ts
const ossNavButtons: MenuActionType[] = [
  menuActions.dashboard,
  menuActions.fleet,
  menuActions.home,
  menuActions.missionControl,
  menuActions.myUi,
  menuActions.labs,
  menuActions.maps,
  menuActions.health,
  menuActions.user,
  menuActions.settings,
  menuActions.extras,
];
```

4. 在 desktop/mobile 的 `isActive` 判断中增加：

```ts
(btn.label === 'Mission Control' && pathname === '/mission-control') ||
```

注意：`botbrain_ws_aitech` 当前 `home` label 已是 `Cockpit` 且导航到 `/cockpit`，不要把它改回旧的 `/robot-home`。只加 Mission Control，避免无关路由回退。

### 3.4 Provider 迁移策略

`BotBrainmx2` 的 `layout.tsx` 已经拆成：

```text
src/app/providers.tsx
src/app/robot-runtime-providers.tsx
```

`botbrain_ws_aitech` 目前是在 root layout 中直接挂：

```text
RobotConnectionProvider
RobotProfileProvider
RobotCustomModeProvider
ActiveMissionProvider
NavigationTargetsProvider
```

推荐分两步：

#### Phase A：最小可用迁移

先不改 root provider 结构。因为 `mission-control/layout.tsx` 自己会包：

```tsx
<MissionSupervisorProvider>
  ...
</MissionSupervisorProvider>
```

`MissionSupervisorProvider` 只依赖已有的 `NotificationsProvider`，而 `botbrain_ws_aitech` root layout 已经提供它。

这样最小改动最安全，对现有建图、导航、灵巧手页面没有影响。

#### Phase B：可选优化

Mission Control 验收通过后，再考虑把 `BotBrainmx2` 的 `providers.tsx` 和 `robot-runtime-providers.tsx` 迁入 `botbrain_ws_aitech`，用于减少 `/` 首页不必要的 ROS runtime 连接。

这个优化不影响 Mission Control 替代目标，不应与第一轮迁移混做。

### 3.5 环境变量

在 `botbrain_ws_aitech/.env.example` 和生产 `frontend/.env` 中补充：

```bash
# Mission Supervisor API, server side default target for Next.js proxy.
MISSION_SUPERVISOR_URL=http://127.0.0.1:8787

# Browser visible default shown in the Mission Control connection panel.
NEXT_PUBLIC_MISSION_SUPERVISOR_URL=http://127.0.0.1:8787

# Optional. Default in BotBrainmx2 is 8000ms.
MISSION_SUPERVISOR_PROXY_TIMEOUT_MS=8000

# Optional. Set true only when Mission Control should require logged-in user.
NEXT_PUBLIC_REQUIRE_AUTH_FOR_MISSION_CONTROL=false
```

如果 `web_server` 容器用 `env_file: ./frontend/.env`，确认生产机器上实际有该文件。不要把真实 token、Supabase secret、Wi-Fi 密码提交进仓库。

### 3.6 mockg1 本地仿真栈

`BotBrainmx2/mockg1` 已迁移到：

```text
botbrain_ws_aitech/mockg1/
```

它是独立 Node ESM 工程，提供 rosbridge-compatible WebSocket mock，默认端点：

```text
ws://127.0.0.1:9090
```

支持的 scenario：

```text
default
low_battery
emergency
unstable_network
```

迁移边界：

- `mockg1` 只模拟前端使用的 rosbridge topic/service 契约。
- `mockg1` 不连接 Unitree SDK2、CycloneDDS、ROS 2 graph、Dex3 或 Nav2 action。
- 任何 mock 指令都没有真机副作用。
- Mission Control 任务写操作仍然走 Mission Supervisor；mockg1 只用于前端遥测、Health 和本地回归。

配套脚本已迁移到：

```text
scripts/run-mockg1-bt-stack.sh
scripts/stop-mockg1-bt-stack.sh
scripts/run-36-flow-mock-validation.sh
```

脚本设计原则：

- 默认 `MISSION_SUPERVISOR_PROFILE=fake`。
- 默认 `MISSION_SUPERVISOR_AUTO_APPROVE=1`，只用于 fake/test 验证。
- 运行日志和 pid 文件写入 `.run/mockg1-bt/`，该目录已加入 `.gitignore`。
- 不依赖 Linux-only 的 `setsid`、`timeout`、`ss`、`/proc`，在 macOS/Linux 上用 Node TCP probe、pid 文件和 `lsof` 做启动/清理。
- 前端 dev server 由脚本注入 `NEXT_PUBLIC_ROS_IP=127.0.0.1` 和 `NEXT_PUBLIC_ROS_PORT=9090`，因此 mock 模式不会修改生产 `.env` 或真机默认连接。

首次安装：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech/mockg1
source ~/.nvm/nvm.sh
nvm use v24.12.0
npm ci
npm test

cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech/frontend
source ~/.nvm/nvm.sh
nvm use v24.12.0
npm ci
```

启动完整本地 mock Mission Control 栈：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech
source ~/.nvm/nvm.sh
nvm use v24.12.0
./scripts/run-mockg1-bt-stack.sh --scenario default
```

启动后访问：

```text
http://localhost:3000/mission-control
http://localhost:3000/health
```

注意：`botbrain_ws_aitech` 当前页面仍按 BotBrainmx2 的行为要求登录。未登录访问 `/mission-control` 或 `/health` 会被全局 middleware 重定向到 `/`。

停止本地 mock 栈：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech
./scripts/stop-mockg1-bt-stack.sh --ports 3000,8787,9090
```

36 case fake 回归：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech
source ~/.nvm/nvm.sh
nvm use v24.12.0
./scripts/run-36-flow-mock-validation.sh --pace fast --keep-stack 0
```

如果需要把 Mission Supervisor snapshot 同步到 mock rosbridge topic：

```bash
./scripts/run-mockg1-bt-stack.sh --mirror 1
```

这会发布 `/mock/mission_snapshot`，便于前端或调试工具观察 Mission Supervisor 状态投影。

## 4. API 契约

### 4.1 Next.js BFF proxy

`/api/mission-supervisor/[...path]/route.ts` 的职责：

- 只允许明确 allowlist 的 GET/POST path。
- `target` 只能指向 localhost 或私网 IPv4。
- 默认走 `MISSION_SUPERVISOR_URL`。
- 透传 `/stream` SSE。
- 普通 JSON 请求有超时保护。

允许的 GET path 应包括：

```text
healthz
snapshot
events
stream
visualization/model
exceptions/catalog
preflight
voice-context
voice/sessions
voice/transcript
auth/demo-code
pending-decisions
adapter-status
testing/playback/scenarios
voice/sessions/{conversation_id}/transcript
testing/playback/sessions/{session_id}
testing/playback/sessions/{session_id}/report
```

允许的 POST path 应包括：

```text
instruction
intent
operator-decision
control
tick
testing/playback/sessions
testing/playback/sessions/{session_id}/control
```

不建议暴露：

```text
voice/events
raw ROS write APIs
gateway direct control APIs
arbitrary target URL
```

### 4.2 Mission Supervisor 前端 service

`mission-supervisor.ts` 应保持所有前端请求都先走：

```text
/api/mission-supervisor/<path>
```

不要在浏览器中直接请求：

```text
http://127.0.0.1:8787
http://<robot-ip>:8787
http://127.0.0.1:8899
ws://<robot-ip>:9090
```

Mission Control 的写请求都应带：

- `schema_version`
- `command_id` 或 `event_id`
- `idempotency_key`
- `mission_run_id`
- `expected_state_version`
- `operator_id`
- `reason/comment`

### 4.3 Stale 状态和危险操作

保留 `MissionSupervisorContext` 的策略：

```text
POLL_INTERVAL_MS = 1000
STALE_AFTER_MS = 2500
MAX_EVENTS = 200
```

页面行为：

- snapshot stale 时，普通 `PAUSE`、`RESUME`、`RESET_IDLE` 禁用。
- stale 时只保留 `ABORT`、`TAKEOVER`、`RESYNC` 等安全恢复动作。
- HIGH/CRITICAL decision 要求 operator comment。
- `preflight.go === false` 时不能批准 `START_MISSION`。

## 5. botbrain_ws_aitech 真机能力保护方案

### 5.1 不动已有 ROS topic/action/service contract

Mission Control 替代不应改变这些现有链路：

```text
建图:
  fast_lio
  g1_pkg/fast_lio.launch.py
  grid_accumulator / pcd_to_grid / map tools

定位:
  open3d_loc
  g1_pkg/localization_3d.launch.py
  existing map files and loc params

导航:
  bot_navigation/navigation.launch.py
  Nav2 action servers
  cmd_vel_nav -> twist_mux -> cmd_vel_out
  cancel_nav2_goal / Nav2 cancel

灵巧手/右臂:
  g1_manipulation_pkg
  g1_right_dex3
  Dex3 control scripts and calibration data
```

### 5.2 禁止绕过速度仲裁

生产任务代码禁止直接发布：

```text
/g1_robot/cmd_vel_out
/cmd_vel_out
```

应保持：

```text
Nav2 controller -> cmd_vel_nav -> twist_mux -> cmd_vel_out -> g1_write
```

Stop/abort 应优先：

1. cancel 当前 Nav2 goal。
2. 让 mux/zero velocity 生效。
3. 高风险时调用已有 emergency stop service。
4. 记录 evidence 回到 Mission Supervisor。

### 5.3 灵巧手控制必须 action 化

如果 Mission Control 后续要展示或触发电梯按钮/灵巧手任务，不要让页面直接调用 Dex3 topic/service。推荐由 gateway 暴露任务级动作：

```text
POST /g1/actions/press-button
GET  /g1/actions/{action_id}
POST /g1/actions/{action_id}/cancel
```

动作结果必须包含：

```json
{
  "status": "SUCCEEDED",
  "evidence": {
    "arm_controller": "done",
    "hand_controller": "done",
    "button_light": "confirmed",
    "camera_observation": "optional",
    "duration_ms": 3200
  }
}
```

Mission Supervisor 根据 evidence 推进任务。前端只显示 evidence，不自己判断“按钮按下完成”。

## 6. 推荐实施步骤

### Phase 0：建立保护基线

在任何代码迁移前记录当前真机稳定状态：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech

git status --short
git rev-parse HEAD
docker compose ps
```

记录 ROS graph：

```bash
docker compose exec dev bash -lc '
source install/setup.bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
'
```

记录关键 topic/action：

```bash
docker compose exec dev bash -lc '
source install/setup.bash
ros2 action list | grep -E "navigate_to_pose|follow_waypoints" || true
ros2 topic hz /g1_robot/battery --window 5
ros2 topic hz /g1_robot/odom --window 5
ros2 topic hz /g1_robot/imu/data --window 5
ros2 topic hz /g1_robot/joint_states --window 5
'
```

如果当前 namespace 不是 `/g1_robot`，按实际 topic 改命令，并把 namespace 写入 Mission Supervisor/gateway 配置。不要为了迁移前端强行改 namespace。

### Phase 1：迁移 Mission Control 前端静态依赖

执行第 3 章文件迁移。

随后检查 import：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech/frontend

npm run lint -- \
  src/app/mission-control \
  src/app/api/mission-supervisor \
  src/components/mission-control \
  src/contexts/MissionSupervisorContext.tsx \
  src/services/mission-supervisor.ts \
  src/types/mission-control.ts
```

如果项目的 lint script 不支持追加路径，则运行：

```bash
npm run lint
```

再运行构建：

```bash
npm run build
```

### Phase 2：补齐菜单入口

修改：

```text
frontend/src/types/RobotActionTypes.ts
frontend/src/hooks/useMenuActions.tsx
frontend/src/components/nav-menu.tsx
```

验证：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech/frontend
npm run lint
npm run build
```

手工验证：

```bash
npm run dev
```

打开：

```text
http://127.0.0.1:3000/mission-control
```

预期：

- 页面可以打开。
- 菜单有 Mission Control。
- 如果 Mission Supervisor 未启动，页面显示 connection error，而不是白屏。
- `/health` 页面仍可打开。

### Phase 3：接入同一个 Mission Supervisor

启动或复用现有 Mission Supervisor：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/tour-guide-robot/Mission-Supervisor-BT

MISSION_SUPERVISOR_PROFILE=fake \
MISSION_SUPERVISOR_LOG_DIR=/tmp/mission-supervisor-fake \
python -m mission_supervisor.api
```

先用 fake/test profile 验证，不连真机控制：

```bash
curl http://127.0.0.1:8787/healthz
curl http://127.0.0.1:8787/snapshot
curl http://127.0.0.1:8787/preflight
curl http://127.0.0.1:8787/adapter-status
curl http://127.0.0.1:8787/visualization/model
```

再通过 `botbrain_ws_aitech` proxy 验证：

```bash
curl http://127.0.0.1:3000/api/mission-supervisor/healthz
curl http://127.0.0.1:3000/api/mission-supervisor/preflight
curl http://127.0.0.1:3000/api/mission-supervisor/adapter-status
```

SSE 验证：

```bash
curl -N http://127.0.0.1:3000/api/mission-supervisor/stream
```

如果浏览器和 Mission Supervisor 不在同一台机器，可在页面 Connection 面板设置：

```text
http://<robot-private-ip>:8787
```

proxy 会通过 `target` 参数转发，但仍只允许 localhost/private IPv4。

### Phase 3.5：用 mockg1 做本地端到端回归

在不连接 Unitree G1 EDU 真机的情况下，可以启动 `mockg1 + Mission Supervisor fake + botbrain_ws_aitech frontend`：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech
source ~/.nvm/nvm.sh
nvm use v24.12.0

cd mockg1
npm ci
npm test

cd ../frontend
npm ci

cd ..
./scripts/run-mockg1-bt-stack.sh --scenario default
```

打开：

```text
http://localhost:3000/mission-control
http://localhost:3000/health
```

预期：

- 未登录时仍重定向到 `/`。
- 登录后 Mission Control 能通过 `/api/mission-supervisor/*` 看到 fake Mission Supervisor。
- Health/ROS 前端连接地址可指向 `ws://127.0.0.1:9090`。
- `low_battery`、`emergency`、`unstable_network` scenario 能在页面上体现相应状态。
- 所有日志在 `.run/mockg1-bt/<run-id>/`。

停止：

```bash
./scripts/stop-mockg1-bt-stack.sh --ports 3000,8787,9090
```

36 case fake 验证：

```bash
./scripts/run-36-flow-mock-validation.sh --pace fast --keep-stack 0
```

这个阶段不应启动 `botbrain_ws_aitech/botbrain_ws`、不应运行 Docker compose、不应调用 Unitree G1 EDU 真机 bringup。

### Phase 4：对比 BotBrainmx2 与 botbrain_ws_aitech 页面输出

并行启动两个前端，但只连同一个 Mission Supervisor：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/BotBrainmx2/frontend
npm run dev -- -p 3001
```

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech/frontend
npm run dev -- -p 3000
```

打开：

```text
http://127.0.0.1:3001/mission-control
http://127.0.0.1:3000/mission-control
```

逐项对比：

| 面板 | 预期 |
|---|---|
| Connection | base URL、状态一致 |
| Supervisor Snapshot | mission_state、phase、waypoint 一致 |
| Preflight | blockers/checks 一致 |
| Adapter Status | profile、adapter、details 一致 |
| Decision Queue | pending decisions 一致 |
| Active Exception | exception instance 一致 |
| 36 Test Cases | completed/running/waiting 状态一致 |
| Playback | fake/test profile 下可用；real profile 下按后端策略禁用或保护 |
| Event Log | sequence/state_version 单调一致 |

当两个页面输出一致后，`botbrain_ws_aitech` 已具备替代 `BotBrainmx2` Mission Control 的前端条件。

### Phase 5：只读连接 G1 真机 runtime

启动 `botbrain_ws_aitech` 已稳定的真机容器组合：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech

docker compose up -d \
  dev \
  bringup \
  state_machine \
  jetson_stats \
  fast_lio \
  localization \
  navigation \
  foxglove
```

需要灵巧手时再启动：

```bash
docker compose up -d manipulation
```

只读检查：

```bash
docker compose ps

docker compose exec dev bash -lc '
source install/setup.bash
ros2 node list | grep -E "g1|nav|state|lio|loc" || true
ros2 action list | grep -E "navigate_to_pose|follow_waypoints" || true
ros2 topic list | grep -E "battery|odom|imu|joint|scan|diagnostics" || true
'
```

此阶段不通过 Mission Control 触发任何真机动作，只确认 health/preflight 能看到真机栈状态。

### Phase 6：接入或新增 botbrain_ws_gateway

如果已有 gateway，直接配置 Mission Supervisor：

```bash
MISSION_SUPERVISOR_PROFILE=g1_botbrain_ws
BOTBRAIN_WS_GATEWAY_URL=http://127.0.0.1:8899
```

如果没有 gateway，新增一个薄 sidecar，先只读：

```text
GET /g1/healthz
GET /g1/state
GET /g1/sensors
GET /g1/pois
```

只读 gateway preflight 应检查：

```text
gateway process alive
ROS graph reachable
state_machine/status fresh
Nav2 action server visible
Nav2 lifecycle active or managed by bot_state_machine
battery fresh
odom fresh
imu fresh
joint_states fresh
TF fresh
map/localization health
twist_mux alive
emergency stop state known
manipulation state known if route needs dexterous hand
route waypoint mapping complete
map_id / route_id compatible
```

`docker-compose.yaml` 中新增 gateway service 时，不要改已有服务命令：

```yaml
  botbrain_ws_gateway:
    extends: base
    container_name: g1_robot_botbrain_ws_gateway
    command: ["bash", "-lc", "source install/setup.bash && python3 -m botbrain_ws_gateway --host 0.0.0.0 --port 8899"]
    restart: always
```

### Phase 7：低风险动作验证

顺序必须是：

1. fake/test profile 全通过。
2. 真机只读 preflight 通过。
3. 真机 stop/cancel endpoint 验证，但不导航。
4. 空场、低速、单点 Nav2 goal。
5. 多 waypoint 导航。
6. 灵巧手 dry-run。
7. 灵巧手真实单动作。
8. 任务级组合动作。

单点导航只允许通过 Mission Supervisor 触发：

```text
Mission Control
  -> POST /api/mission-supervisor/instruction
  -> Mission Supervisor
  -> botbrain_ws_gateway
  -> /g1_robot/navigate_to_pose
```

不要从页面直接调用 `/navigate_to_pose`。

### Phase 8：切换入口，停用 BotBrainmx2 Mission Control

在确认 `botbrain_ws_aitech` 完成替代后：

1. systemd/nginx/入口文档只保留 `botbrain_ws_aitech/frontend`。
2. `BotBrainmx2/frontend` 不再作为 operator 页面启动。
3. Mission Supervisor 仍然可以保持原位置或后续独立部署，不要求放入 `botbrain_ws_aitech` 仓库。
4. `BotBrainmx2` 可保留为历史参考，直到稳定运行一个完整测试周期。

## 7. 验证矩阵

### 7.1 前端构建验证

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech/frontend

npm run lint
npm run build
```

验收：

- 无 TypeScript 编译错误。
- 无 missing import。
- `mission-control/page.tsx` 不因缺 `MissionSupervisorContext`、`mission-control` components 或 types 报错。
- `/health` 不退化。

### 7.2 API proxy 验证

```bash
curl -i http://127.0.0.1:3000/api/mission-supervisor/healthz
curl -i http://127.0.0.1:3000/api/mission-supervisor/snapshot
curl -i http://127.0.0.1:3000/api/mission-supervisor/preflight
curl -i http://127.0.0.1:3000/api/mission-supervisor/adapter-status
curl -N http://127.0.0.1:3000/api/mission-supervisor/stream
```

安全验证：

```bash
curl -i 'http://127.0.0.1:3000/api/mission-supervisor/unknown-path'
curl -i 'http://127.0.0.1:3000/api/mission-supervisor/healthz?target=https://example.com'
```

预期：

- unknown path 返回 404。
- public internet target 被拒绝。
- localhost/private IPv4 target 可用。

### 7.3 Mission Control 行为验证

| 场景 | 预期 |
|---|---|
| Supervisor 未启动 | 页面显示 connection error，不白屏 |
| Supervisor fake profile | snapshot/preflight/events 可见 |
| SSE 断开 | polling fallback 仍有状态，stale banner 正确 |
| snapshot stale | 普通操作禁用，只保留安全恢复动作 |
| pending decision | 新 decision 出现在队列，通知出现 |
| HIGH/CRITICAL decision | 要求 comment |
| preflight NO-GO | START_MISSION approve 禁用 |
| submit control | 返回 result 后 snapshot 刷新 |
| playback fake/test | 可加载 scenario 和 step |
| playback real | 未显式允许时禁用或由后端拒绝 |

### 7.4 ROS runtime 回归

建图：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech
docker compose logs --tail=200 fast_lio
```

定位：

```bash
docker compose logs --tail=200 localization
```

导航：

```bash
docker compose exec dev bash -lc '
source install/setup.bash
ros2 action list | grep navigate_to_pose
ros2 lifecycle get /g1_robot/controller_server || true
ros2 lifecycle get /g1_robot/planner_server || true
ros2 lifecycle get /g1_robot/bt_navigator || true
'
```

灵巧手：

```bash
docker compose ps manipulation
docker compose logs --tail=200 manipulation
```

验收：

- 本次前端迁移前后，ROS graph 关键 topic/action/service 名称不变。
- 已有 `tools/gotop`、`tools/nav` 手动验证流程仍可运行。
- Dex3 calibration 文件和启动方式未被修改。

### 7.5 真机分阶段验证

| 阶段 | 允许动作 | 不允许动作 |
|---|---|---|
| A | 打开 `/mission-control` 和 `/health` | 任何移动/手臂动作 |
| B | 查看 preflight/adapter-status | 导航、按按钮 |
| C | 调用 stop/cancel | 新建导航 goal |
| D | 单点低速导航 | 多点任务、电梯任务 |
| E | 多点导航 | 灵巧手真实按压 |
| F | 灵巧手 dry-run | 自动电梯完整闭环 |
| G | 灵巧手单动作 | 无人值守组合任务 |
| H | 完整任务 | 绕过 Mission Supervisor 的任何控制 |

每一阶段必须记录：

- Mission Supervisor event log。
- gateway action evidence。
- ROS logs。
- 操作员观察结果。
- 失败时的 stop/abort 结果。

## 8. 回滚方案

### 8.1 前端快速回滚

如果 `/mission-control` 出现问题，但 `/health` 和现有机器人控制页面要继续使用：

1. 从 `nav-menu.tsx` 暂时移除 `menuActions.missionControl`。
2. 保留文件但不暴露入口。
3. 或删除以下新增文件：

```text
frontend/src/app/mission-control/
frontend/src/app/api/mission-supervisor/
frontend/src/contexts/MissionSupervisorContext.tsx
frontend/src/services/mission-supervisor.ts
frontend/src/types/mission-control.ts
frontend/src/components/mission-control/
```

4. 运行：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech/frontend
npm run build
```

### 8.2 真机 runtime 回滚

因为本方案不改已有 ROS runtime，正常情况下不需要回滚建图、导航和灵巧手代码。

如果新增了 gateway service，可单独停掉：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech
docker compose stop botbrain_ws_gateway
```

不应停止：

```text
bringup
state_machine
fast_lio
localization
navigation
manipulation
```

除非这是原本的真机维护流程。

### 8.3 Operator 入口回滚

如果 `botbrain_ws_aitech` Mission Control 未完全稳定：

```text
临时 operator 入口回退到 BotBrainmx2/frontend:3001
生产 ROS runtime 继续使用 botbrain_ws_aitech
Mission Supervisor 不变
```

这样可以只回退页面，不回退真机 runtime。

## 9. 风险清单与规避

| 风险 | 规避 |
|---|---|
| 迁移页面时改动 root provider，影响现有 ROS 连接 | 第一阶段不改 root provider，只新增 mission layout 自己的 provider |
| 前端直接连 Supervisor，绕过 BFF 安全限制 | 所有请求走 `/api/mission-supervisor/*` |
| proxy 允许任意 target，形成 SSRF | 保留 localhost/private IPv4 校验和 allowlist |
| Mission Control 误用 rosbridge 控制真机 | 页面只走 Mission Supervisor 写 API |
| Mission Supervisor 直接 import rclpy/Unitree SDK2 | 用 gateway sidecar 做隔离 |
| 修改 Nav2/twist_mux 导致已验证导航退化 | 本次冻结 `bot_navigation`、`bot_bringup`、`g1_pkg` |
| 直接发布 `/cmd_vel_out` 绕过 mux | 代码审查和 gateway preflight 禁止 |
| Dex3 与旧 arm_cmd 双写 | 任务动作统一走一种 gateway action，保留 manipulation interlock |
| Preflight 只看进程不看 topic freshness | healthz 必须检查 action/service/topic freshness |
| fake/test 回归被 real profile 破坏 | 先 fake/test，后 real，只新增 profile，不改默认 fake/test |
| 36 case 被前端本地计算推进 | 只读 Supervisor `completed_test_cases` 和 events |

## 10. 推荐提交拆分

不要一次提交所有改动。推荐拆成五个 commit：

### Commit 1：Mission Control 静态迁移

```text
frontend/src/app/mission-control/
frontend/src/app/api/mission-supervisor/
frontend/src/contexts/MissionSupervisorContext.tsx
frontend/src/services/mission-supervisor.ts
frontend/src/types/mission-control.ts
frontend/src/components/mission-control/
```

验证：

```bash
npm run build
```

### Commit 2：菜单入口

```text
frontend/src/types/RobotActionTypes.ts
frontend/src/hooks/useMenuActions.tsx
frontend/src/components/nav-menu.tsx
```

验证：

```bash
npm run build
```

### Commit 3：env/example 与部署文档

```text
.env.example
mxdocs/101-g55-merge-BotBrainmx2-to-botbrain_ws_aitech.md
```

### Commit 4：mockg1 本地仿真与回归脚本

```text
mockg1/
scripts/run-mockg1-bt-stack.sh
scripts/stop-mockg1-bt-stack.sh
scripts/run-36-flow-mock-validation.sh
.gitignore
```

验证：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech/mockg1
source ~/.nvm/nvm.sh
nvm use v24.12.0
npm ci
npm test

cd ..
bash -n scripts/run-mockg1-bt-stack.sh \
  scripts/stop-mockg1-bt-stack.sh \
  scripts/run-36-flow-mock-validation.sh
./scripts/run-mockg1-bt-stack.sh --help
./scripts/run-36-flow-mock-validation.sh --help
```

### Commit 5：可选 gateway

只有当 Mission Supervisor 当前没有能连接 `botbrain_ws_aitech` 真机 runtime 的 gateway 时才做：

```text
botbrain_ws/src/botbrain_ws_gateway/
docker-compose.yaml, 仅新增 service
```

这个 commit 必须单独真机回归。

## 11. 最小落地命令清单

以下是最短可执行顺序：

```bash
cd /Users/fausto/mdev/aitech/4g1edu

# 0. 使用指定 Node.js
source ~/.nvm/nvm.sh
nvm use v24.12.0

# 1. 复制 Mission Control 前端文件
mkdir -p botbrain_ws_aitech/frontend/src/app/mission-control
mkdir -p 'botbrain_ws_aitech/frontend/src/app/api/mission-supervisor/[...path]'
mkdir -p botbrain_ws_aitech/frontend/src/components/mission-control

cp BotBrainmx2/frontend/src/app/mission-control/layout.tsx \
   botbrain_ws_aitech/frontend/src/app/mission-control/layout.tsx
cp BotBrainmx2/frontend/src/app/mission-control/page.tsx \
   botbrain_ws_aitech/frontend/src/app/mission-control/page.tsx
cp 'BotBrainmx2/frontend/src/app/api/mission-supervisor/[...path]/route.ts' \
   'botbrain_ws_aitech/frontend/src/app/api/mission-supervisor/[...path]/route.ts'
cp BotBrainmx2/frontend/src/contexts/MissionSupervisorContext.tsx \
   botbrain_ws_aitech/frontend/src/contexts/MissionSupervisorContext.tsx
cp BotBrainmx2/frontend/src/services/mission-supervisor.ts \
   botbrain_ws_aitech/frontend/src/services/mission-supervisor.ts
cp BotBrainmx2/frontend/src/types/mission-control.ts \
   botbrain_ws_aitech/frontend/src/types/mission-control.ts
cp BotBrainmx2/frontend/src/components/mission-control/*.tsx \
   botbrain_ws_aitech/frontend/src/components/mission-control/

# 2. 手工补菜单三处
# - src/types/RobotActionTypes.ts
# - src/hooks/useMenuActions.tsx
# - src/components/nav-menu.tsx

# 3. 构建验证
cd botbrain_ws_aitech/frontend
npm run build

# 4. 启动 Mission Supervisor fake/test
cd /Users/fausto/mdev/aitech/4g1edu/tour-guide-robot/Mission-Supervisor-BT
MISSION_SUPERVISOR_PROFILE=fake \
MISSION_SUPERVISOR_LOG_DIR=/tmp/mission-supervisor-fake \
python -m mission_supervisor.api

# 5. 启动 botbrain_ws_aitech 前端
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech/frontend
npm run dev
```

打开：

```text
http://127.0.0.1:3000/mission-control
http://127.0.0.1:3000/health
```

如需同时迁移并运行 `mockg1`：

```bash
cd /Users/fausto/mdev/aitech/4g1edu/botbrain_ws_aitech
source ~/.nvm/nvm.sh
nvm use v24.12.0

cd mockg1
npm ci
npm test

cd ../frontend
npm ci

cd ..
./scripts/run-mockg1-bt-stack.sh --scenario default
```

## 12. 最终验收清单

迁移完成后逐项打勾：

```text
[ ] /health 与迁移前功能一致。
[ ] /mission-control 在 botbrain_ws_aitech 可访问。
[ ] 菜单有 Mission Control，active 状态正确。
[ ] /api/mission-supervisor/healthz 可访问。
[ ] /api/mission-supervisor/stream 可持续输出 SSE。
[ ] MissionSupervisorContext 无前端异常。
[ ] fake/test profile 下 36 case、events、pending decisions 正常。
[ ] mockg1 `npm test` 通过。
[ ] `run-mockg1-bt-stack.sh` 能启动 mockg1、Mission Supervisor fake 和 botbrain_ws_aitech 前端。
[ ] `stop-mockg1-bt-stack.sh` 能清理 mock 栈进程。
[ ] `run-36-flow-mock-validation.sh --pace fast --keep-stack 0` 能输出 runner summary。
[ ] preflight NO-GO 时不能批准启动任务。
[ ] BotBrainmx2 与 botbrain_ws_aitech 同连一个 Supervisor 时显示一致。
[ ] 未修改 g1_pkg、fast_lio、open3d_loc、bot_navigation、g1_manipulation_pkg、g1_right_dex3。
[ ] docker compose 中原 bringup/navigation/localization/manipulation 启动方式不变。
[ ] 真机建图流程回归通过。
[ ] 真机定位/导航流程回归通过。
[ ] 真机灵巧手/Dex3 流程回归通过。
[ ] Stop/abort 在 Mission Supervisor -> gateway -> ROS 链路中可审计。
[ ] BotBrainmx2 frontend 可以停止，operator 使用 botbrain_ws_aitech。
```

## 13. 推荐结论

推荐执行。`botbrain_ws_aitech` 本身已经承载 G1 EDU 真机稳定 runtime，把 `BotBrainmx2` 的 Mission Control 前端整合进来，可以减少双前端维护成本，并让 operator 统一使用一个页面。

关键是不要把“页面迁移”扩大成“真机 runtime 重构”。第一阶段只做前端和 BFF proxy 迁移。第二阶段只读接入 Mission Supervisor/gateway。第三阶段才在低速、空场、可回滚条件下验证真实导航和灵巧手动作。只要这个边界守住，`botbrain_ws_aitech` 可以完全取代 `BotBrainmx2` 的 Mission Control，同时保持已经稳定的建图、导航和灵巧手能力不动。
