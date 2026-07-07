# 08 开机自启动 jtop 与 D435i 浏览器画面根因分析及最佳实践方案

生成时间: 2026-07-08  
分析对象: `botbrain_ws_aitech` 仓库 `main` 分支  
分析范围: 2026-07-07 的 `main` 分支提交，以及当前工作区源码、`docker-compose.yaml`、systemd 模板、`botbrain_ws/install` 运行时安装空间  
目标: G1 开机后自动启动服务，同时在浏览器中稳定看到 jtop/Jetson 信息和 Intel RealSense D435i 摄像头画面。

> 说明: 本文是静态代码与配置梳理，没有连接真实 G1 主机执行冷启动验证。下面的结论基于当前本地仓库状态、提交历史和 `botbrain_ws/install` 与 `src` 的差异。

---

## 1. 结论摘要

这次没有完全修好的核心原因不是某一个小 bug，而是运行链路被拆成了多层临时补丁:

1. **源码已改，但运行时 `botbrain_ws/install` 没有同步**  
   当前 `botbrain_ws/install` 是 Git ignore 的 colcon 安装产物，时间戳停在 2026-07-06 22:15/22:16，而 2026-07-07 的 D435i、jtop、自启动修复都发生在之后。实际容器启动时 `source install/setup.bash`，多数 launch/config/script 来自 `install`，不是直接读 `src`。因此“提交看起来修了”，但运行时仍可能用旧版。

2. **开机自启动没有拉起完整目标栈**  
   `botbrain.service` 只 `docker compose up dev bringup rosa jetson_stats state_machine -d`，没有显式创建/拉起 `localization`、`fast_lio`、`foxglove`、`web_server_prod`。`restart: always` 只能重启已经存在的容器，不能保证新部署或容器被删除后自动创建。

3. **D435i 被挂在 3D localization 里，架构耦合过重**  
   浏览器看 D435i 画面本质只需要 `front_camera -> compressed_camera -> rosbridge/frontend`，不应依赖 `open3d_loc`、`map_server`、`fast_lio` 或 3D 定位是否成功。当前源码把 RealSense include 到 `g1_pkg/localization_3d.launch.py`，导致摄像头画面受 localization 容器状态影响。

4. **ROS2 lifecycle 控制存在启动时序竞态**  
   `state_machine` 的 `bring_up()` 在辅助类节点 `get_state()` 失败时直接 `continue`，不会等待节点稍后出现。开机时 D435i lifecycle 节点可能比 state_machine 晚 30-60 秒出现，于是被跳过，长期停在 `unconfigured`，没有图像。

5. **最后一次“相机自动激活守卫”是症状补丁，不是可靠编排**  
   2026-07-07 18:18 提交把 `ros2 lifecycle set` 放到 `start_localization.sh` 后台进程中等待相机节点。这在工程上不可靠: 它依赖另一个 launch 文件先把相机节点拉起来，依赖 shell 后台进程存活，且没有被 systemd/compose 单独观测。

6. **jtop 修复只覆盖了一个崩溃竞态，未解决生命周期编排问题**  
   17:24 提交给 `ros2_jtop_node.py` 加了 publisher `None` 守卫，能避免一次 deactivate/cleanup 与 timer callback 竞争导致的 AttributeError。但 `jtop_publisher` 仍依赖 state_machine 激活；如果 state_machine 启动时序、install 滞后或节点未被激活，Health 页仍没有数据。

7. **前端话题命名空间已修一部分，但连接配置仍需固化**  
   前端已把 `/g1_robot/battery`、`/g1_robot/diagnostic_stats`、`/g1_robot/compressed_camera` 作为目标方向修正。但浏览器端还依赖 `NEXT_PUBLIC_ROS_IP`/`NEXT_PUBLIC_ROS_PORT` 或数据库中的机器人地址。若默认值仍指向 `192.168.1.95:9090`，而实际机器人是 `192.168.37.204`，页面会连接错 ROS bridge。

---

## 2. 2026-07-07 main 分支提交时间线

| 时间 | 提交 | 主要内容 | 判断 |
|---|---|---|---|
| 11:14 | `6a72903` | 3D 导航、FAST-LIO、Nav2、localization 参数调整 | 与 D435i/jtop 间接相关，增加了运行链路复杂度 |
| 14:13 | `92a2153` | 新增 Health/Battery 调查文档 | 分析文档 |
| 14:36 | `3eeb06a` | 前端统一 `/g1_robot` 命名空间，修电池/健康信息 topic | 方向正确，但需要重建前端生产包 |
| 15:26 | `db20e04` | 新增 `mxdocs/07...`，记录 REL 与当前工作区差异 | 已指出 install 与运行容器问题，但不是完整闭环 |
| 16:48 | `098ac8b` | 修 D435i: serial 字符串、真实序列号、删 `back_camera`、改前端 camera topic、新增 `start_localization.sh` | 源码层方向正确，但当前 `install` 未同步 |
| 17:24 | `031c1a5` | 修 jtop timer callback 与 lifecycle cleanup 竞态 | 源码层方向正确，但当前 `install` 未同步 |
| 18:18 | `0694efe` | 在 `start_localization.sh` 加相机 lifecycle 后台激活守卫 | 临时补丁，未解决编排与 install 滞后 |

---

## 3. 证据链

### 3.1 当前 systemd 模板没有拉起完整目标服务

`botbrain.service` 当前模板:

```ini
ExecStart=/bin/bash -c "source ./botbrain_ws/robot_select.sh && /usr/bin/docker compose up dev bringup rosa jetson_stats state_machine -d"
ExecStop=/usr/bin/docker compose stop dev bringup jetson_stats state_machine
```

问题:

- 没有显式启动 `localization`。如果 D435i 仍由 localization launch 拉起，开机后摄像头不会被保证启动。
- 没有显式启动 `foxglove`。如果要用 Foxglove WebSocket 看 ROS 数据，8765 服务不会被保证创建。
- 没有显式启动 `web_server_prod`。Web 前端靠另一个 `web_server.service`，但两个 systemd unit 之间没有依赖关系。
- `dev` 被作为生产自启动服务启动，不是目标功能所必需。
- `ExecStop` 没有停止 `rosa`，也没有停止 `localization`、`fast_lio`、`foxglove`。

`web_server.service` 当前模板:

```ini
ExecStart=/bin/bash -c "/usr/bin/docker compose up web_server_prod -d"
ExecStop=/usr/bin/docker compose stop web_server
```

问题:

- 启动的是 `web_server_prod`，停止的是 `web_server`，服务名不一致。`systemctl stop web_server.service` 不会按预期停止生产前端容器。

### 3.2 `docker-compose.yaml` 里虽然有 restart，但不能代替首次创建

相关服务:

```yaml
foxglove:
  restart: always

fast_lio:
  restart: always

localization:
  command: ["bash", "-lc", "/botbrain_ws/start_localization.sh"]
  restart: always

jetson_stats:
  restart: always

web_server_prod:
  restart: unless-stopped
```

`restart: always` 或 `unless-stopped` 只在容器已经被创建后生效。如果容器从未 `docker compose up` 过、被 `docker compose down` 删除、换机部署、重建 workspace，Docker daemon 不会凭空创建它们。因此 systemd 必须显式 `docker compose up -d` 完整服务列表。

### 3.3 `botbrain_ws/install` 明确落后于 `src`

当前 `botbrain_ws/install` 是 ignore 的构建产物:

```gitignore
botbrain_ws/build/
botbrain_ws/install/
botbrain_ws/log/
```

关键文件差异:

| 功能 | 源码 `src` 当前状态 | 运行时 `botbrain_ws/install` 当前状态 | 影响 |
|---|---|---|---|
| `g1_pkg/localization_3d.launch.py` | 已 include `bot_localization/realsense.launch.py` | 没有 include RealSense launch | localization 启动后不会创建 D435i lifecycle 节点 |
| `g1_pkg/config/camera_config.yaml` | D435i serial 为 `419522072874` | serial 为 `243722074823` | 可能打开错误设备或找不到目标相机 |
| `bot_localization/realsense.launch.py` | `serial_no` 用 `ParameterValue(..., value_type=str)` | `serial_no: serial` | 纯数字 serial 可能被 ROS launch 当成整数，RealSense 参数类型错误 |
| `bot_state_machine/config/camera.json` | 只含 `front_camera`、`realsense_compressed_node` | 仍含 `back_camera` | state_machine 会尝试管理不存在的后置相机 |
| `bot_jetson_stats/ros2_jtop_node.py` | `jetson_callback()` 有 publisher `None` 守卫 | 没有守卫 | deactivate/cleanup 与 timer 竞态仍可导致 jtop 进程崩溃 |

这条证据是本次最关键的根因之一。只要运行时仍来自旧 `install`，后续讨论的源码修复都不会稳定生效。

### 3.4 `start_localization.sh` 的后台 lifecycle 守卫无法保证相机启动

当前脚本核心逻辑:

```bash
sleep 30
source install/setup.bash
export LD_LIBRARY_PATH=/opt/open3d/lib:$LD_LIBRARY_PATH

(
  NS=/g1_robot
  FRONT=$NS/front_camera
  COMP=$NS/realsense_compressed_node
  # wait and ros2 lifecycle set ...
) &

exec ros2 launch g1_pkg localization_3d.launch.py
```

问题:

- 它先启动后台守卫，再 `exec ros2 launch ...`。如果 installed 的 `localization_3d.launch.py` 没 include RealSense，后台守卫永远等不到 `/g1_robot/front_camera`。
- 它使用相对路径 `source install/setup.bash`，依赖容器工作目录恰好是 `/botbrain_ws`。
- lifecycle 操作被放在 shell 后台进程里，compose 和 systemd 看不到它的状态。
- 它与 state_machine 同时管理相同 lifecycle 节点，存在“双 owner”风险。
- 一旦 localization 进程退出，后台守卫也跟着容器结束；没有独立的重试和健康观测。

### 3.5 state_machine 启动时序设计会跳过晚出现的节点

`StateController::bring_up()` 对非 core/navigation 的辅助节点逻辑是:

```cpp
auto cur = get_node_state(n.name);
if (!cur) continue;
...
if(!node_activate(n)) continue;
```

这意味着:

- 如果 state_machine 启动时 `front_camera`、`realsense_compressed_node`、`jtop_publisher` 的 lifecycle service 还没 ready，就直接跳过。
- 之后即使节点出现，也不会自动重试，除非手动重启 state_machine 或发命令。
- 开机冷启动最容易触发这个问题，因为 USB 相机枚举、RealSense reset、Docker 容器、ROS discovery 都需要时间。

这解释了“手动调试能好，重启后不稳定”的现象。

### 3.6 D435i launch 仍有 front-only 场景残留问题

当前 `realsense.launch.py` 源码虽已修正主要 D435i 问题，但仍有几个工程质量问题:

1. `depthimage_to_laserscan_back` 无条件创建，即使 `back.type: ""`。
2. `compressed_realsense.py` 无条件创建 `compressed_back_camera` publisher，并订阅绝对话题 `/back_camera/color/image_raw`，不带 `/g1_robot` namespace。
3. `child_frame = front_cfg.get('child', ...)` 与配置里的 `child_frame` 字段不一致，当前靠默认值绕过。
4. 图像 topic 使用自定义 OpenCV 压缩节点，能工作，但行业上更推荐 ROS `image_transport` 的标准 compressed pipeline，或明确维护当前自定义节点的 QoS、CPU 占用和异常处理。

这些不是当前最大根因，但会继续制造误导日志和维护成本。

### 3.7 前端连接和话题链路

浏览器看到 D435i 画面的实际链路应是:

```text
D435i USB
  -> realsense2_camera_node lifecycle active
  -> /g1_robot/front_camera/color/image_raw
  -> realsense_compressed_node lifecycle active
  -> /g1_robot/compressed_camera
  -> rosbridge websocket :9090
  -> frontend http :80
```

浏览器看到 jtop 信息的实际链路应是:

```text
Jetson host jtop/jetson-stats service
  -> g1_robot_jetson_stats container
  -> /g1_robot/jtop_publisher lifecycle active
  -> /g1_robot/diagnostic_stats
  -> rosbridge websocket :9090
  -> frontend /health
```

当前前端默认 ROS 地址:

```ts
const DEFAULT_IP = process.env.NEXT_PUBLIC_ROS_IP ?? '192.168.1.95';
const DEFAULT_PORT = process.env.NEXT_PUBLIC_ROS_PORT ?? '9090';
```

如果生产环境没有设置 `NEXT_PUBLIC_ROS_IP`，且数据库/本地保存的 robot address 也不是当前机器人地址，浏览器会连错 ROS bridge。Next.js 的 `NEXT_PUBLIC_*` 变量在构建时固化，所以修改 `frontend/.env` 后必须重新 build 生产前端。

---

## 4. 根因分层

### P0 根因: 构建与运行时不一致

`src` 与 `botbrain_ws/install` 分叉，是当前最直接的失败原因。典型症状:

- Git diff 看起来修好了 D435i/jtop。
- 容器日志和运行行为仍像旧代码。
- 重启后恢复到旧问题。
- 不同 AI/人工调试可能通过 `docker exec cp` 临时让某个容器好起来，但这个状态没有进入可重复构建流程。

### P0 根因: 自启动服务列表不完整

systemd 没有明确描述“要实现 jtop + D435i 浏览器画面”所需的完整服务图。生产目标至少需要:

- `bringup`: ROS bridge/机器人基础 bringup
- `jetson_stats`: jtop 数据发布
- `camera` 或当前临时的 `localization`: D435i driver + compressed image
- `state_machine`: lifecycle 管理，除非改成独立 lifecycle manager
- `web_server_prod`: 浏览器前端
- `foxglove`: 如果要用 Foxglove 观察 ROS 数据

当前没有一个 systemd unit 原子地拉起这套目标栈。

### P1 根因: lifecycle 编排不可靠

state_machine 的“节点不存在就跳过”适合运行中命令，不适合冷启动编排。冷启动需要:

- 等待 lifecycle service 出现；
- 有 deadline 和重试；
- 能记录哪个节点没 ready；
- 对可选节点降级，对必需节点失败；
- 保证每个 lifecycle 节点只有一个 owner。

当前用后台 shell 守卫补洞，会导致双 owner 和不可观测。

### P1 根因: 摄像头与定位耦合

浏览器看 D435i 画面不应依赖 3D localization 容器。localization 引入 Open3D、map server、ICP、FAST-LIO 依赖，任何一个失败都会影响相机画面。行业最佳实践是把 camera pipeline 拆成独立服务，让定位/导航订阅相机或深度数据，而不是由定位服务拥有相机启动。

### P2 根因: 前端配置未作为部署资产管理

前端已修命名空间，但生产构建需要固定:

- ROS bridge IP/hostname；
- ROS bridge port；
- ROS namespace；
- 是否使用数据库中的 robot address；
- 修改后重新 build。

这些现在没有和 systemd/compose 部署步骤形成闭环。

---

## 5. 推荐目标架构

推荐把目标拆成 5 个稳定边界:

```text
systemd: botbrain-stack.service
  |
  +-- docker compose up -d --remove-orphans
      |
      +-- web_server_prod      : Next.js production, port 80
      +-- bringup              : robot interface + rosbridge :9090
      +-- jetson_stats         : jtop publisher lifecycle nodes
      +-- camera               : D435i + compressed image, independent from localization
      +-- state_machine        : lifecycle owner with wait/retry, or observer only
      +-- foxglove             : optional ROS inspection bridge :8765
      +-- fast_lio/localization/navigation : optional nav stack, not required for camera page
```

设计原则:

1. **源码是唯一事实来源，install 是可再生产物**  
   不提交 `install`，不手工长期维护 `install`。每次部署都 rebuild，调试时用 `--symlink-install`。

2. **开机自启动由 systemd 创建完整 compose 栈**  
   不依赖 Docker restart policy 创建容器。restart policy 只用于容器崩溃后的自动恢复。

3. **camera 独立于 localization**  
   `camera` 服务只负责 D435i driver 和压缩图像。localization/fast_lio 可以失败，但不影响浏览器画面。

4. **lifecycle 节点只能有一个 owner**  
   不要 state_machine、shell guard、nav2 lifecycle_manager 同时管理同一节点。推荐修 state_machine，使它成为唯一 owner；或把 camera/jtop 从 state_machine 配置中移除，交给专用 lifecycle manager。

5. **每个目标都有验收命令**  
   冷启动后必须验证容器、lifecycle、topic publisher、topic hz、前端连接。

---

## 6. 详细操作步骤: 当前工程先修到可重复工作

以下步骤按“先恢复可用，再做架构清理”的顺序执行。

### Step 0: 在目标 G1 主机确认路径与分支

```bash
cd /data/unitree/botbrain_ws_aitech
git status --short --branch
git branch --show-current
git log --oneline -7
```

预期:

- 分支是 `main`。
- 包含 2026-07-07 的 `0694efe`、`031c1a5`、`098ac8b` 等提交。
- 工作区没有未确认的本地临时改动；如果有，先备份或记录。

### Step 1: 停止旧容器，清理旧安装产物

```bash
cd /data/unitree/botbrain_ws_aitech

docker compose ps
docker compose stop web_server_prod bringup jetson_stats state_machine localization fast_lio foxglove 2>/dev/null || true

# 清理旧 colcon 产物。它们是 ignore 的可再生产物。
rm -rf botbrain_ws/build botbrain_ws/install botbrain_ws/log
```

如果担心现场产物需要回滚，可先打包:

```bash
tar czf /tmp/botbrain_ws_install_backup_$(date +%Y%m%d-%H%M%S).tgz \
  botbrain_ws/install botbrain_ws/build botbrain_ws/log 2>/dev/null || true
```

### Step 2: 用 symlink-install 重建 ROS workspace

调试和现场迭代建议使用 `--symlink-install`，让 Python launch/config/script 的 `install` 指向 `src`，避免再次出现“源码已改但运行时没改”。

```bash
cd /data/unitree/botbrain_ws_aitech

docker compose run --rm builder_base bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  cd /botbrain_ws
  colcon build --symlink-install \
    --packages-select \
      go2_pkg \
      tita_pkg \
      bot_bringup \
      bot_description \
      bot_custom_interfaces \
      joystick_bot \
      bot_localization \
      bot_navigation \
      bot_localization_interfaces \
      bot_state_machine \
      bot_jetson_stats \
      bot_jetson_stats_interfaces \
      bot_rosa \
      g1_pkg \
      go2w_pkg \
      g1_manipulation_pkg \
      fast_lio \
      open3d_loc \
    --cmake-args \
      -DOpen3D_DIR=/opt/open3d/lib/cmake/Open3D \
      -DUNITREE_SDK2_ROOT=/opt/robot_sdk
'
```

构建后检查关键文件是否已经跟源码一致:

```bash
diff -u \
  botbrain_ws/src/g1_pkg/config/camera_config.yaml \
  botbrain_ws/install/g1_pkg/share/g1_pkg/config/camera_config.yaml

diff -u \
  botbrain_ws/src/bot_state_machine/config/camera.json \
  botbrain_ws/install/bot_state_machine/share/bot_state_machine/config/camera.json

diff -u \
  botbrain_ws/src/bot_localization/bot_localization/launch/realsense.launch.py \
  botbrain_ws/install/bot_localization/share/bot_localization/launch/realsense.launch.py

diff -u \
  botbrain_ws/src/bot_jetson_stats/bot_jetson_stats/scripts/ros2_jtop_node.py \
  botbrain_ws/install/bot_jetson_stats/lib/bot_jetson_stats/ros2_jtop_node.py
```

预期:

- `--symlink-install` 下很多文件可能是 symlink，`diff` 应无输出或只剩非功能性差异。
- `localization_3d.launch.py` 的 installed 版本必须 include `realsense.launch.py`。
- jtop installed 版本必须包含 `if self.diag_pub is None or self.human_pub is None: return`。

### Step 3: 临时按当前架构恢复相机服务

在未拆分 `camera` 服务前，当前源码中 D435i 是由 `localization` 间接拉起的。因此先确保当前链路能运行:

```bash
cd /data/unitree/botbrain_ws_aitech

docker compose up -d bringup jetson_stats state_machine localization web_server_prod
```

等待 60-90 秒后检查:

```bash
docker compose ps

docker compose exec localization bash -lc '
  set -e
  cd /botbrain_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 node list | sort | grep -E "front_camera|realsense_compressed_node|global_localization|map_server" || true
  echo "--- lifecycle ---"
  ros2 lifecycle get /g1_robot/front_camera || true
  ros2 lifecycle get /g1_robot/realsense_compressed_node || true
  echo "--- topics ---"
  ros2 topic info /g1_robot/front_camera/color/image_raw || true
  ros2 topic info /g1_robot/compressed_camera || true
'
```

如果 lifecycle 不是 `active`，先用当前临时方案手动激活确认硬件链路:

```bash
docker compose exec localization bash -lc '
  cd /botbrain_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  for n in /g1_robot/front_camera /g1_robot/realsense_compressed_node; do
    st=$(ros2 lifecycle get "$n" 2>/dev/null | awk "{print \$1}" || true)
    echo "$n state=$st"
    if [ "$st" = "unconfigured" ]; then
      ros2 lifecycle set "$n" configure
      sleep 8
      ros2 lifecycle set "$n" activate
    elif [ "$st" = "inactive" ]; then
      ros2 lifecycle set "$n" activate
    fi
  done
'
```

验证图像:

```bash
docker compose exec localization bash -lc '
  cd /botbrain_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  timeout 10 ros2 topic hz /g1_robot/front_camera/color/image_raw
'

docker compose exec localization bash -lc '
  cd /botbrain_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  timeout 10 ros2 topic hz /g1_robot/compressed_camera
'
```

预期:

- `/g1_robot/front_camera` 为 `active`。
- `/g1_robot/realsense_compressed_node` 为 `active`。
- `/g1_robot/compressed_camera` 有 publisher 且 hz 大于 0。

### Step 4: 验证 jtop/Jetson 数据

先确认 host 侧 jetson-stats 服务存在:

```bash
systemctl list-units --type=service | grep -Ei 'jtop|jetson'
systemctl status jtop.service 2>/dev/null || systemctl status jetson_stats.service 2>/dev/null || true
```

如果没有安装:

```bash
sudo -H pip3 install -U jetson-stats
sudo reboot
```

容器侧验证:

```bash
cd /data/unitree/botbrain_ws_aitech
docker compose up -d jetson_stats state_machine bringup

sleep 30

docker compose exec jetson_stats bash -lc '
  cd /botbrain_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 lifecycle get /g1_robot/jtop_publisher || true
  ros2 topic info /g1_robot/diagnostic_stats || true
  timeout 8 ros2 topic echo /g1_robot/diagnostic_stats --once
'
```

如果 `jtop_publisher` 未 active:

```bash
docker compose exec jetson_stats bash -lc '
  cd /botbrain_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  st=$(ros2 lifecycle get /g1_robot/jtop_publisher 2>/dev/null | awk "{print \$1}" || true)
  echo "jtop state=$st"
  if [ "$st" = "unconfigured" ]; then
    ros2 lifecycle set /g1_robot/jtop_publisher configure
    sleep 2
    ros2 lifecycle set /g1_robot/jtop_publisher activate
  elif [ "$st" = "inactive" ]; then
    ros2 lifecycle set /g1_robot/jtop_publisher activate
  fi
'
```

预期:

- `/g1_robot/jtop_publisher` 为 `active`。
- `/g1_robot/diagnostic_stats` 收到 `std_msgs/String`。
- `/g1_robot/diagnostics` 收到 `diagnostic_msgs/DiagnosticArray`。

### Step 5: 固化前端生产配置并重建

在目标机器人上设置前端公开配置。示例:

```bash
cd /data/unitree/botbrain_ws_aitech

grep -q '^NEXT_PUBLIC_ROS_IP=' frontend/.env || echo 'NEXT_PUBLIC_ROS_IP=192.168.37.204' >> frontend/.env
grep -q '^NEXT_PUBLIC_ROS_PORT=' frontend/.env || echo 'NEXT_PUBLIC_ROS_PORT=9090' >> frontend/.env
grep -q '^NEXT_PUBLIC_ROS_NAMESPACE=' frontend/.env || echo 'NEXT_PUBLIC_ROS_NAMESPACE=g1_robot' >> frontend/.env
```

如果现场 IP 不是 `192.168.37.204`，改成真实 IP 或稳定 hostname。然后重建生产前端:

```bash
docker compose run --rm web_server_builder
docker compose up -d web_server_prod
```

浏览器访问:

```text
http://<机器人IP>/
http://<机器人IP>/health
```

浏览器开发者工具里应看到 ROS WebSocket 连接到:

```text
ws://<机器人IP>:9090
```

### Step 6: 替换 systemd 自启动为完整栈

推荐合并为一个 `botbrain-stack.service`，统一创建/停止目标服务。示例:

```ini
[Unit]
Description=BotBrain G1 runtime stack
Requires=docker.service
After=docker.service network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
User=root
WorkingDirectory=/data/unitree/botbrain_ws_aitech
ExecStartPre=/bin/bash -lc 'for i in $(seq 1 60); do ip route | grep -q "default via" && exit 0; sleep 1; done; exit 1'
ExecStart=/usr/bin/docker compose up -d --remove-orphans bringup jetson_stats state_machine localization foxglove web_server_prod
ExecStop=/usr/bin/docker compose stop web_server_prod foxglove localization state_machine jetson_stats bringup
TimeoutStartSec=180
TimeoutStopSec=60

[Install]
WantedBy=multi-user.target
```

安装:

```bash
sudo tee /etc/systemd/system/botbrain-stack.service >/dev/null <<'EOF'
[Unit]
Description=BotBrain G1 runtime stack
Requires=docker.service
After=docker.service network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
User=root
WorkingDirectory=/data/unitree/botbrain_ws_aitech
ExecStartPre=/bin/bash -lc 'for i in $(seq 1 60); do ip route | grep -q "default via" && exit 0; sleep 1; done; exit 1'
ExecStart=/usr/bin/docker compose up -d --remove-orphans bringup jetson_stats state_machine localization foxglove web_server_prod
ExecStop=/usr/bin/docker compose stop web_server_prod foxglove localization state_machine jetson_stats bringup
TimeoutStartSec=180
TimeoutStopSec=60

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl disable botbrain.service web_server.service 2>/dev/null || true
sudo systemctl enable --now botbrain-stack.service
```

> 当前临时架构仍用 `localization` 负责 D435i。完成下面 Step 7 后，应把 `localization` 替换为 `camera`，并按需单独启动 `fast_lio/localization/navigation`。

### Step 7: 拆分 camera 服务，解除与 localization 的耦合

这是推荐的正式架构改造。新增 compose 服务:

```yaml
  camera:
    extends: base
    container_name: g1_robot_camera
    command: ["bash", "-lc", "cd /botbrain_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch bot_localization realsense.launch.py"]
    restart: unless-stopped
```

如果 `base` 镜像没有 `realsense2_camera`，改为 `extends: nav3d`，并先验证:

```bash
docker compose run --rm base bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg prefix realsense2_camera'
docker compose run --rm nav3d bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg prefix realsense2_camera'
```

然后把 systemd 服务列表改为:

```ini
ExecStart=/usr/bin/docker compose up -d --remove-orphans bringup jetson_stats state_machine camera foxglove web_server_prod
ExecStop=/usr/bin/docker compose stop web_server_prod foxglove camera state_machine jetson_stats bringup
```

这样浏览器 D435i 画面不再依赖 `fast_lio`、`open3d_loc`、map server 或 localization 是否成功。

### Step 8: 修 state_machine lifecycle 编排

推荐原则: camera/jtop 如果仍在 `bot_state_machine/config/*.json` 中，就让 state_machine 成为唯一 lifecycle owner；删除 `start_localization.sh` 里的后台 lifecycle guard。

需要实现的行为:

1. 对每个必需 lifecycle 节点，等待 `get_state` service 出现，最多等待 120 秒。
2. 出现后按 `unconfigured -> configure -> inactive -> activate` 激活。
3. 失败要记录节点名、当前 state、失败 transition。
4. 可选节点允许降级，但必须在状态发布中显示 failed/absent。
5. 不要 `if (!cur) continue` 静默跳过。

伪代码:

```cpp
std::optional<State> wait_node_state(name, timeout) {
  auto deadline = now + timeout;
  while (now < deadline) {
    auto cur = get_node_state(name);
    if (cur) return cur;
    sleep(500ms);
  }
  return std::nullopt;
}

for (const auto& n : nodes_) {
  auto cur = wait_node_state(n.name, node_timeout(n));
  if (!cur) {
    mark_absent(n.name);
    if (required(n)) state_ = State::ERROR;
    continue;
  }
  node_activate(n);
}
```

如果选择改用 `nav2_lifecycle_manager` 或自定义 Python lifecycle manager 来管理 camera/jtop，则必须:

- 从 `bot_state_machine/config/camera.json` 和 `accessories.json` 中移除这些节点，避免双 owner。
- state_machine 只订阅状态，不再发 lifecycle transition。

不推荐继续使用 `start_localization.sh` 中的后台 `ros2 lifecycle set` 守卫作为正式方案。

### Step 9: 修 D435i front-only launch 残留

建议修正:

1. `realsense.launch.py` 只在 `back.type` 非空时创建 `depthimage_to_laserscan_back`。
2. `compressed_realsense.py` 只在存在后置相机时创建后置 subscription/publisher，且使用相对 topic 或带 namespace。
3. `front_cfg.get('child', ...)` 改为 `front_cfg.get('child_frame', ...)`。
4. 图像 QoS 改成适合 sensor stream 的 profile，避免 reliable 阻塞。
5. 长期方案使用标准 `image_transport` compressed publisher，减少自维护 OpenCV 压缩节点。

最小行为目标:

```text
/g1_robot/front_camera/color/image_raw        有 publisher，有 hz
/g1_robot/compressed_camera                   有 publisher，有 hz
/g1_robot/back_camera/*                       没有配置 back 时不启动、不报错
```

### Step 10: 冷启动验收

执行:

```bash
sudo reboot
```

重启后 SSH 登录，等待 2-3 分钟:

```bash
cd /data/unitree/botbrain_ws_aitech

systemctl status botbrain-stack.service --no-pager
docker compose ps
```

ROS 验收:

```bash
docker compose exec bringup bash -lc '
  cd /botbrain_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  echo "--- nodes ---"
  ros2 node list | sort | grep -E "front_camera|realsense_compressed_node|jtop_publisher|rosbridge|state_machine" || true
  echo "--- lifecycle ---"
  ros2 lifecycle get /g1_robot/front_camera || true
  ros2 lifecycle get /g1_robot/realsense_compressed_node || true
  ros2 lifecycle get /g1_robot/jtop_publisher || true
  echo "--- topics ---"
  ros2 topic info /g1_robot/compressed_camera || true
  ros2 topic info /g1_robot/diagnostic_stats || true
'
```

摄像头验收:

```bash
docker compose exec bringup bash -lc '
  cd /botbrain_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  timeout 10 ros2 topic hz /g1_robot/compressed_camera
'
```

jtop 验收:

```bash
docker compose exec bringup bash -lc '
  cd /botbrain_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  timeout 10 ros2 topic echo /g1_robot/diagnostic_stats --once
'
```

前端验收:

1. 打开 `http://<机器人IP>/health`。
2. 确认 System Information、CPU、GPU、内存、温度、电源、风扇、存储信息刷新。
3. 打开摄像头所在页面，确认画面连续刷新。
4. 浏览器开发者工具确认 WebSocket 为 `ws://<机器人IP>:9090`，没有连到旧默认 IP。

通过标准:

- `docker compose ps` 中目标服务都是 `Up`。
- `/g1_robot/front_camera`、`/g1_robot/realsense_compressed_node`、`/g1_robot/jtop_publisher` 都是 `active`。
- `/g1_robot/compressed_camera` 和 `/g1_robot/diagnostic_stats` 有 publisher 且能收到数据。
- 重启 3 次均通过，不需要人工 `docker exec` 或手动 lifecycle set。

---

## 7. 推荐后续代码改动清单

### P0 必做

1. 把 `docker-compose.yaml` 的 builder 命令改成 `colcon build --symlink-install ...`，至少现场调试版本必须如此。
2. 新增 `camera` compose service，把 D435i 从 `localization` 中拆出来。
3. 替换 systemd 为一个完整的 `botbrain-stack.service`，或修正现有两个 unit 的服务列表与 stop 列表。
4. 修 state_machine 的 lifecycle wait/retry，不再静默跳过晚出现节点。
5. 删除或禁用 `start_localization.sh` 的后台 lifecycle guard。
6. 重新 build ROS workspace 和前端生产包。

### P1 应做

1. 修 `realsense.launch.py` 的 back camera 条件创建和 `child_frame` 字段。
2. 修 `compressed_realsense.py` 的 namespace、back camera 条件创建、QoS。
3. 明确 `frontend/.env` 的 `NEXT_PUBLIC_ROS_IP`、`NEXT_PUBLIC_ROS_PORT`、`NEXT_PUBLIC_ROS_NAMESPACE`，并写入部署文档。
4. 给 `jetson_stats` 和 `camera` 加健康检查脚本，例如检查 topic publisher 与 lifecycle active。
5. 把 `foxglove_bridge.launch.py` 中默认配置文件名修正为真实存在的 `foxglove_bridge_params.yaml`，或统一 compose 和 launch 使用同一个配置。

### P2 优化

1. 用 `image_transport` 标准 compressed pipeline 替代自定义 OpenCV JPEG 压缩节点。
2. 把相机 serial 等每台机器不同的参数从源码配置迁移到 `robot_config.yaml` 或 per-host env/config。
3. 给冷启动验收做成脚本，例如 `tools/verify_boot_stack.sh`。
4. 在 CI 或部署脚本中加入 `src/install diff` 检查，防止 installed space 再次滞后。

---

## 8. 不建议继续采用的做法

1. 不建议长期手工 `cp src/... install/...`。这是救火手段，不是部署流程。
2. 不建议让 Docker restart policy 代替 systemd 创建目标服务。
3. 不建议把 D435i 浏览器画面绑定到 3D localization 是否成功。
4. 不建议同时用 state_machine、shell guard、nav2 lifecycle_manager 管同一个 lifecycle 节点。
5. 不建议只看 Git 提交判断是否修好，必须看 `install`、容器日志、ROS topic/lifecycle、浏览器 WebSocket。

---

## 9. 最小可用验收命令清单

冷启动后只要跑下面 6 条，基本就能判断目标是否达成:

```bash
cd /data/unitree/botbrain_ws_aitech
systemctl is-active botbrain-stack.service
docker compose ps
```

```bash
docker compose exec bringup bash -lc 'cd /botbrain_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 lifecycle get /g1_robot/front_camera'
```

```bash
docker compose exec bringup bash -lc 'cd /botbrain_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 lifecycle get /g1_robot/realsense_compressed_node'
```

```bash
docker compose exec bringup bash -lc 'cd /botbrain_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 10 ros2 topic hz /g1_robot/compressed_camera'
```

```bash
docker compose exec bringup bash -lc 'cd /botbrain_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 lifecycle get /g1_robot/jtop_publisher'
```

```bash
docker compose exec bringup bash -lc 'cd /botbrain_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 10 ros2 topic echo /g1_robot/diagnostic_stats --once'
```

若这些命令通过，再看浏览器。如果命令不通过，先修 ROS/container，不要先改前端。

---

## 10. 当前最可能的实际失败路径

结合当前仓库状态，最可能发生的是:

```text
开机
  -> systemd botbrain.service 只拉起 bringup/jetson_stats/state_machine 等
  -> localization/camera 不一定被创建
  -> 即使 localization 被 Docker restart policy 拉起，也使用旧 botbrain_ws/install
  -> 旧 install 的 localization_3d.launch.py 没 include realsense.launch.py
  -> /g1_robot/front_camera 根本不出现
  -> start_localization.sh 后台 guard 等不到节点
  -> state_machine 也因节点未出现而跳过
  -> /g1_robot/compressed_camera 无 publisher
  -> 浏览器没有 D435i 画面
```

jtop 的可能失败路径:

```text
开机
  -> jetson_stats 容器启动
  -> state_machine 负责激活 jtop_publisher
  -> 如果时序正常，可能 active
  -> 如果 state_machine 被摄像头/其他 lifecycle 重启扰动，旧 install 里的 jtop callback 无 None 守卫
  -> timer callback 与 cleanup 竞争，AttributeError 崩溃
  -> /g1_robot/diagnostic_stats 无 publisher
  -> /health 没有 Jetson 信息
```

因此，正确修复顺序必须是:

1. 重建/同步 `install`；
2. 完整自启动服务列表；
3. lifecycle wait/retry；
4. camera 与 localization 解耦；
5. 前端生产配置和 build；
6. 冷启动验收。

