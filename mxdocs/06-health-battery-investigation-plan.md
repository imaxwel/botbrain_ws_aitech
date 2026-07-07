# 80 端口 Health 页面电池信息缺失排查与修正方案

生成时间: 2026-07-07  
当前工作区: `/data/unitree/botbrain_ws`  
对比旧工作区: `/data/unitree/botbrain-rel/BotBrain`

## 结论摘要

当前电池采集链路本身是正常的，底层 Unitree BMS topic `/lf/bmsstate` 有数据，`g1_read.py` 也已经稳定发布 `sensor_msgs/msg/BatteryState` 到 `/g1_robot/battery`，频率约 20 Hz。

前端没有显示电池信息的直接原因是 topic 路径不一致:

- 当前 ROS 实际发布: `/g1_robot/battery`
- 当前前端实际订阅: `/battery`
- 当前 `/battery` 没有任何 publisher，只有 rosbridge 代表浏览器建立的 subscriber

因此浏览器一直在监听空 topic，电池百分比保持默认值或显示缺失。

另外，直接访问 `http://192.168.37.204/health` 会收到 `307 Temporary Redirect` 到 `/`。这是 Next.js middleware 的鉴权行为，不是电池数据缺失的根因，但会影响用 curl 或外部探针检查 `/health`。

## 已验证现象

### HTTP 层

当前运行的 web 容器:

- 容器: `g1_robot_web_server_prod`
- 镜像: `node:22-bullseye`
- compose 文件: `/data/unitree/botbrain_ws/docker-compose.yaml`
- 工作目录: `/app`
- bind mount: `/data/unitree/botbrain_ws/frontend:/app`
- 网络: `host`
- 命令: `npm run start`
- 环境: `NODE_ENV=production`, `PORT=80`

`curl -i http://192.168.37.204/health` 返回:

```text
HTTP/1.1 307 Temporary Redirect
location: /

/
```

对应源码在 `frontend/src/middleware.ts`:

- `/` 是登录页
- `path !== '/'` 都被视为 protected route
- 未登录访问 `/health` 时会被重定向到 `/`

这说明裸 HTTP 请求看到的 `/` 是鉴权重定向结果。浏览器如果已有 Supabase session，则会进入 `/health` 页面。

### 当前 ROS 数据层

底层 BMS 有数据，示例:

```text
/lf/bmsstate:
  bmsvoltage: [52231, 52255, 0]
  current: -2156
  soc: 83
  soh: 90
```

转换后的电池 topic 有数据，示例:

```text
/g1_robot/battery:
  voltage: 52.29399871826172
  current: -2.1640000343322754
  percentage: 0.8500000238418579
  frame_id: g1_robot/base
```

`/g1_robot/battery` 发布频率约 20 Hz:

```text
average rate: 20.2 Hz
```

topic 端点情况:

```text
/battery:
  Type: sensor_msgs/msg/BatteryState
  Publisher count: 0
  Subscription count: 1
  Subscriber: /rosbridge_websocket

/g1_robot/battery:
  Type: sensor_msgs/msg/BatteryState
  Publisher count: 1
  Publisher: /g1_robot/robot_read_node
  Subscription count: 0
```

`docker logs g1_robot_bringup` 也记录了浏览器经 rosbridge 订阅的是根 topic:

```text
Subscribed to /battery
```

未看到对应的:

```text
Subscribed to /g1_robot/battery
```

## 代码链路定位

### ROS 发布路径

`botbrain_ws/robot_config.yaml` 当前配置:

```yaml
robot_configuration:
  robot_name: "g1_robot"
  robot_model: "g1"
```

`botbrain_ws/src/g1_pkg/launch/robot_interface.launch.py` 中:

```python
g1_read_node = LifecycleNode(
    package='g1_pkg',
    executable='g1_read.py',
    name='robot_read_node',
    namespace=robot_name,
)
```

`botbrain_ws/src/g1_pkg/scripts/g1_read.py` 中:

```python
self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
```

因为节点 namespace 是 `g1_robot`，相对 topic `battery` 被解析为:

```text
/g1_robot/battery
```

这与实际 `ros2 topic echo /g1_robot/battery --once` 的结果一致。

### 前端订阅路径

`frontend/src/app/health/layout.tsx` 会渲染:

```tsx
<RobotHeader />
```

所以 `/health` 上看到的电池信息来自全局 header，不是 `frontend/src/app/health/page.tsx` 主体里的 Jetson/diagnostics 面板。排查和修复应优先看 `RobotHeader` 的电池 hook。

`frontend/src/components/robot-header.tsx`:

```tsx
const batteryState = useThrottledBatteryState();
```

`frontend/src/hooks/ros/useThrottledBatteryState.tsx`:

```tsx
const rawBatteryState = useRobotBatteryState();
```

`frontend/src/hooks/ros/useRobotBatteryState.tsx`:

```tsx
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';

topicFactory.createAndSubscribeTopic<BatteryState>('battery', ...)
```

`frontend/src/utils/ros/topics-and-services.tsx`:

```tsx
export const topicsMessages = {
  battery: 'battery',
}

export function getRosTopic(...) {
  return `${isDummy ? '/dummy' : ''}/${topic}`;
}
```

所以前端固定订阅:

```text
/battery
```

项目里已经有 profile-aware 的 `frontend/src/utils/ros/topics-and-services-v2.tsx`，但当前电池 hook 仍然 import legacy factory，没有使用 robot profile。

同时，`frontend/src/config/robot-profiles/profiles/g1-r1.ts` 里的 G1 profile 仍配置为:

```tsx
battery: 'battery'
```

即使切到 v2 factory，如果不修改 G1 profile，也仍然会订阅 `/battery`。

## 旧目录对比

旧 web 容器:

- 容器: `web_server_prod`
- 容器 ID: `aa810c4625d2`
- compose 文件: `/data/unitree/botbrain-rel/BotBrain/docker-compose.yaml`
- bind mount: `/data/unitree/botbrain-rel/BotBrain/frontend:/app`
- 旧环境端口: `PORT=3000`
- 当前状态: exited

旧目录和当前目录的关键前端源码对比:

- `frontend/src/app/health/page.tsx`: 无差异
- `frontend/src/components/robot-header.tsx`: 无差异
- `frontend/src/utils/ros/topics-and-services.tsx`: 无差异

旧目录 ROS 配置也同样是:

```yaml
robot_name: "g1_robot"
robot_model: "g1"
```

旧 `g1_read.py` 也同样是相对发布 `battery`，并且 `g1_read_node` 同样在 `namespace=robot_name` 下启动。

因此，从可见源码和残留容器配置看，旧目录本身并没有一个明确的“前端订阅 `/g1_robot/battery`”实现。旧容器当时能看到电池信息，更可能是当时运行态存在以下情况之一:

1. 旧 ROS graph 中曾有额外 publisher 或手工 relay 在发布根 `/battery`
2. 当时浏览器连接的是另一个机器人/ROS graph，其中根 `/battery` 有数据
3. 当时使用的运行态与当前磁盘源码不完全一致，例如手工改动、未纳入文档的临时节点或残留进程

旧运行态已经退出，无法直接复现当时的 ROS graph；当前可验证事实足以定位当前问题: 前端订阅 `/battery`，实际数据在 `/g1_robot/battery`。

## 根因

根因是命名空间策略没有在前端和 ROS 侧保持一致。

ROS 侧 G1 节点遵循 namespace:

```text
robot_name = g1_robot
relative topic battery -> /g1_robot/battery
```

前端 legacy topic 工厂没有 namespace 概念:

```text
battery -> /battery
```

最终形成:

```text
/g1_robot/robot_read_node publishes /g1_robot/battery
rosbridge/frontend subscribes /battery
```

两端没有连接上。

## 建议修正路线

### 阶段 1: 立刻恢复显示，提供兼容根 topic

如果目标是最快恢复当前 80 端口前端显示，建议先在 ROS 侧提供 `/battery` 兼容 topic。当前镜像没有安装 `topic_tools`，不能直接用现成 relay 命令，因此需要以下二选一。

方案 A: 在 `g1_read.py` 中同时发布 `/battery`

- 保留现有 `/g1_robot/battery`
- 新增一个兼容 publisher: `/battery`
- 在 `low_bms_state_callback` 中将同一份 `BatteryState` 同时 publish 到两个 topic

优点:

- 对现有前端零改动
- 恢复最快
- `/g1_robot/battery` 仍保留，其他 namespaced 消费者不受影响

风险:

- 根 `/battery` 在多机器人 ROS graph 中可能冲突
- 这是兼容方案，不是长期架构修正

方案 B: 新增一个小 relay 节点

- subscribe: `/g1_robot/battery`
- publish: `/battery`
- 在 bringup 中随 G1 启动

优点:

- 不改 `g1_read.py` 的职责
- relay 可以后续删除

风险:

- 多一个节点和 launch 管理项
- 仍然存在根 topic 冲突风险

不推荐的短期方案:

- 仅在 launch 中把 `('battery', '/battery')` 做 remap。这样会把原本的 `/g1_robot/battery` 移走，只剩 `/battery`，可能破坏依赖 namespaced topic 的消费者。

阶段 1 验证:

```bash
docker exec g1_robot_dev bash -lc 'source /botbrain_ws/install/setup.bash && ros2 topic info -v /battery'
docker exec g1_robot_dev bash -lc 'source /botbrain_ws/install/setup.bash && timeout 5 ros2 topic echo /battery --once'
```

期望:

- `/battery` 有 1 个 publisher
- `/battery` 能 echo 到非零 `percentage`
- 当前前端不改代码即可显示电池百分比

### 阶段 2: 正式修复前端 topic 映射

长期建议让前端按机器人 profile 或 namespace 配置订阅真实 ROS topic，而不是依赖根 topic 兼容。

建议改动:

1. 修改 `frontend/src/config/robot-profiles/profiles/g1-r1.ts`

```tsx
topics: {
  battery: 'g1_robot/battery',
}
```

注意: v2 `getRosTopic()` 会自动补前导 `/`，所以 profile 中不要写成 `'/g1_robot/battery'`，否则要确认不会生成双斜杠。

2. 修改 `frontend/src/hooks/ros/useRobotBatteryState.tsx`

将 legacy factory:

```tsx
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
```

替换为 profile-aware factory:

```tsx
import { ROSTopicFactory } from '@/utils/ros/topics-and-services-v2';
import { useRobotProfile } from '@/contexts/RobotProfileContext';
```

并把 `currentProfile` 传给 factory。`useEffect` 的依赖要包含 profile，否则 `connectToRobotWithInfo()` 先连 ROS、后设置 `connectedRobot` 时，电池 hook 可能已经用空 profile 订阅了 `/battery`，不会自动切换到 `/g1_robot/battery`。

3. 处理没有 `connectedRobot` 的场景

当前前端支持只输入 IP 连接机器人；这种情况下 `connection.connectedRobot` 可能为 `null`，profile 也会是 `null`。建议增加一个明确 fallback:

- 优先用 `currentProfile.topics.battery`
- 若无 profile，则读取 `NEXT_PUBLIC_ROS_NAMESPACE`
- 若 namespace 也没有，再 fallback 到 legacy `/battery`

建议在 `frontend/.env.example` 增加:

```text
NEXT_PUBLIC_ROS_NAMESPACE=g1_robot
```

并在当前部署的 `frontend/.env` 设置同名值。不要把真实 Supabase key 等敏感内容写入文档或提交。

4. 前端构建与重启

当前生产容器使用 bind mount 的 `frontend/.next`。修改前端后需要重新 build 并重启生产服务:

```bash
docker compose run --rm web_server_builder
docker compose up -d web_server_prod
```

阶段 2 验证:

```bash
docker logs g1_robot_bringup | tail -n 200
docker exec g1_robot_dev bash -lc 'source /botbrain_ws/install/setup.bash && ros2 topic info -v /g1_robot/battery'
```

期望:

- rosbridge 日志出现 `Subscribed to /g1_robot/battery`
- `/g1_robot/battery` 的 subscriber count 增加
- 浏览器 header 或健康页相关区域显示非零电池百分比

### 阶段 3: 明确 `/health` 的 HTTP 语义

当前 `/health` 是需要登录的前端页面，不是公开健康检查 API。直接访问会被 middleware 重定向到 `/`。

如果 `/health` 要继续作为页面使用:

- 保持现状即可
- 排查时使用已登录浏览器查看
- 外部服务探针不要用 `/health`

如果 `/health` 要作为 HTTP 探针使用:

建议新增公开 API，例如:

```text
/api/health
```

返回最小 JSON:

```json
{
  "ok": true,
  "service": "botbrain-web",
  "time": "..."
}
```

并保持 middleware 已经排除 `api` route。不要让服务端 `/api/health` 直接声称电池正常，除非后端也能可靠访问 ROS graph；当前电池数据是浏览器经 rosbridge 订阅得到的。

如果希望 `/health` 页面未登录可访问:

- 在 `frontend/src/middleware.ts` 里将 `/health` 加入 public paths
- 需要同时评估页面内是否暴露机器人状态、网络信息、控制按钮等敏感能力

## 修正优先级

推荐执行顺序:

1. 先做阶段 1 的 ROS 兼容根 topic，快速恢复当前生产 UI 电池显示
2. 再做阶段 2 的前端 profile-aware 修正，把 G1 电池 topic 正式映射到 `/g1_robot/battery`
3. 最后决定 `/health` 是页面还是探针；如果是探针，新增 `/api/health`

如果只允许做一个正式修复，优先做阶段 2。阶段 1 适合作为临时兼容或回滚保护。

## 回归验证清单

ROS 数据:

```bash
docker exec g1_robot_dev bash -lc 'source /botbrain_ws/install/setup.bash && timeout 5 ros2 topic echo /lf/bmsstate --once'
docker exec g1_robot_dev bash -lc 'source /botbrain_ws/install/setup.bash && timeout 5 ros2 topic echo /g1_robot/battery --once'
docker exec g1_robot_dev bash -lc 'source /botbrain_ws/install/setup.bash && timeout 5 ros2 topic hz /g1_robot/battery'
```

前端订阅:

```bash
docker logs g1_robot_bringup | tail -n 250
```

检查点:

- 正式方案下应看到 `Subscribed to /g1_robot/battery`
- 兼容方案下 `/battery` 应有 publisher，且浏览器订阅 `/battery` 能收到数据

HTTP 行为:

```bash
curl -i http://192.168.37.204/health
```

检查点:

- 如果 `/health` 仍是受保护页面，未登录返回 307 是预期行为
- 如果新增 `/api/health`，应验证它返回 200 和 JSON

浏览器 UI:

- 登录后打开 `http://192.168.37.204/health`
- 确认顶部 header 电池百分比不是 0
- 确认电压/电流估算信息能随数据刷新
- 刷新页面后仍然显示
- 断开 ROS 连接后能回到默认值，不残留旧电池值

## 附带发现

当前 bringup 日志还有一些与电池无关的问题:

- rosbridge 订阅 `/robot_status` 时无法 import `custom_interfaces.msg`
- 前端周期性调用 `/available_networks`、`/saved_networks`、`/check_4g`，但当前 ROS graph 中服务不存在

这些不影响本次电池根因，但会影响 Health 页面其他模块或网络控制面板，建议后续单独排查:

- 确认前端使用的 custom message package 名称是否应为 `bot_custom_interfaces`
- 确认 WiFi/network service 节点是否已启动，或前端是否应按 namespace 调用服务

## 最小可执行修复建议

为了尽快让当前生产页面恢复电池显示，建议先实施:

1. 在 ROS 侧提供 `/battery` 兼容发布，同时保留 `/g1_robot/battery`
2. 重启 `g1_robot_bringup`
3. 验证 `/battery` 有 publisher 且浏览器显示电池

随后实施正式修复:

1. 将 G1 profile 的 `battery` topic 改为 `g1_robot/battery`
2. 将 `useRobotBatteryState` 接入 `topics-and-services-v2` 和 `RobotProfileContext`
3. 增加 `NEXT_PUBLIC_ROS_NAMESPACE` fallback，覆盖直接 IP 连接场景
4. 重新构建并重启 `web_server_prod`
5. 验证 rosbridge 订阅 `/g1_robot/battery`
