# g1edu 与 g1hk D435i 开机行为差异分析

日期: 2026-07-09

目标: 解释为什么 `g1hk` 开机后浏览器访问 `/cockpit` 通常能直接看到 D435i 画面，而 `g1edu` 开机后经常看不到画面；确认差异来自配置、自启动、运行产物还是代码；给出后续统一成同一套代码和稳定启动的行业最佳实践。

本文只做分析和操作建议，不改变当前运行配置。

## 结论摘要

这不是一个单纯的前端 `/cockpit` 问题。两台机器的 Git HEAD 一样，但运行时并不一样。

核心结论:

1. `g1edu` 与 `g1hk` 的源码仓库 HEAD 都是 `0b0d7bd`，`docker-compose.yaml` 和 `start_localization.sh` checksum 一致。
2. 两台机器的 D435i 机器本地配置不同，这是正确的:
   - `g1edu`: `/etc/botbrain/robot.env` 中 `BOTBRAIN_FRONT_D435I_SERIAL=243722074823`
   - `g1hk`: `/etc/botbrain/robot.env` 中 `BOTBRAIN_FRONT_D435I_SERIAL=419522072874`
3. 关键差异不在源码，而在 `install/` 运行产物:
   - `g1edu` 的 `src/` 与 `install/` 一致，已经走环境变量覆盖序列号。
   - `g1hk` 的 `src/` 与 `install/` 不一致，当前实际运行的 `install/` 仍然把 `419522072874` 硬编码在 `camera_config.yaml` 中，且 `realsense.launch.py` 不读取 `BOTBRAIN_FRONT_D435I_SERIAL`。
   - 因此 `g1hk` 看起来稳定，并不代表它已经完全采用新的“同代码、每机 env 配置”方案。
4. 自启动策略也不同:
   - `g1edu` 的 `botbrain.service` 开机直接启动 `localization navigation yolo` 等重服务。
   - `g1hk` 的 `botbrain.service` 只启动核心服务，`localization/navigation` 是通过 Docker `restart: always` 从既有容器状态恢复。
   - 这会导致两台机器开机时相机初始化时机、资源压力、容器重启顺序不同。
5. `g1edu` 还有一个本地修改:
   - `botbrain_ws/cyclonedds_config.xml` 增加了 `wlx94ba06f26399` 网络接口。
   - 这通常不是 D435i USB 采集失败的直接原因，但可能影响 ROS2 话题发现和跨容器通信稳定性。
6. `g1edu` 运维状态比 `g1hk` 更脆弱:
   - `g1edu` 根分区 `/` 使用率约 93%，swap 使用约 4.7Gi。
   - `g1hk` 根分区 `/` 使用率约 33%，swap 使用约 2.3Gi。
   - `g1edu` 的 `navigation` 容器出现 `Exited (137)`，说明当前启动链路有被杀或强制停止的历史。
7. 当前排查时两边 D435i 都能被 USB 看到，且 ROS 压缩图像话题均能达到约 6Hz。因此故障更偏向“开机链路和运行时差异”，不是固定代码路径必然失败。

## 现场对比

### 仓库与路径

| 项目 | g1edu | g1hk |
|---|---|---|
| 仓库路径 | `/data/botbrain_ws/botbrain_project-main` | `/data/unitree/botbrain_ws` |
| Git HEAD | `0b0d7bd` | `0b0d7bd` |
| 当前工作区 | `botbrain_ws/cyclonedds_config.xml` 有本地修改 | 干净 |
| Docker | `29.1.3` | `29.1.3` |
| Docker Compose | `v2.30.3` | `v5.1.4` |

`g1edu` 本地修改:

```diff
diff --git a/botbrain_ws/cyclonedds_config.xml b/botbrain_ws/cyclonedds_config.xml
index 7e506a1..6b7c53f 100644
--- a/botbrain_ws/cyclonedds_config.xml
+++ b/botbrain_ws/cyclonedds_config.xml
@@ -5,6 +5,7 @@
       <Interfaces>
         <NetworkInterface name="lo" priority="100" multicast="true" />
         <NetworkInterface name="enP8p1s0" priority="10" multicast="true" />
+        <NetworkInterface name="wlx94ba06f26399" priority="50" multicast="true" />
       </Interfaces>
```

判断:

- 如果这个无线网卡接口确实是 `g1edu` 必须使用的 ROS2 通信接口，应改成机器本地配置，不建议直接改仓库里的通用 `cyclonedds_config.xml`。
- 如果不是必须，应恢复为仓库标准版本，避免 DDS 发现路径不一致。
- 暂时不要改 `ROS_DOMAIN_ID` 和 namespace `g1_robot`。

### D435i 序列号配置

| 项目 | g1edu | g1hk |
|---|---|---|
| `/etc/botbrain/robot.env` | `BOTBRAIN_FRONT_D435I_SERIAL=243722074823` | `BOTBRAIN_FRONT_D435I_SERIAL=419522072874` |
| `docker compose config localization` | 能看到 `243722074823` | 能看到 `419522072874` |
| 当前运行容器 env | 能看到 `BOTBRAIN_FRONT_D435I_SERIAL=243722074823` | 当前运行容器未显示该 env |
| 近期日志确认物理序列号 | 需要在停止占用后再次确认 | 日志确认 `Device Serial No: 419522072874` |

`g1hk` 当前运行容器未显示 `BOTBRAIN_FRONT_D435I_SERIAL`，但 `docker compose config` 已经能看到 `419522072874`。这说明 `g1hk` 的当前容器很可能是在 env_file 改造前创建的，后续没有被 force recreate。它现在能工作，是因为 `install/g1_pkg/.../camera_config.yaml` 里硬编码了 `419522072874`。

这是一个典型的“配置看起来已更新，但运行容器和 install 产物仍是旧状态”的问题。

### 源码与 install 运行产物

两台机器的源码文件 checksum 一致:

| 文件 | checksum |
|---|---|
| `docker-compose.yaml` | `b0b5c0809f953d9a154133912788d52b45e283aac2e64cb9f7f7637341ee1413` |
| `botbrain_ws/start_localization.sh` | `960c0c2b155d58fad20812a637f3738aee558cfe9690503b7423edac41e63d0b` |
| `botbrain_ws/src/bot_localization/bot_localization/launch/realsense.launch.py` | `946b299f73a2fcf9ff454f641196bca002d61c05a2b187166ecb0f632cb26f87` |
| `botbrain_ws/src/g1_pkg/config/camera_config.yaml` | `0f045832d0d39dc46556496dddb19e1d5c8e6fbb7870925e95ec540a6414ec84` |

`g1edu` 的 `install/` 与源码一致:

| 文件 | g1edu checksum |
|---|---|
| `install/bot_localization/.../realsense.launch.py` | `946b299f73a2fcf9ff454f641196bca002d61c05a2b187166ecb0f632cb26f87` |
| `install/g1_pkg/.../camera_config.yaml` | `0f045832d0d39dc46556496dddb19e1d5c8e6fbb7870925e95ec540a6414ec84` |

`g1hk` 的 `install/` 与源码不一致:

| 文件 | g1hk install checksum | 含义 |
|---|---|---|
| `install/bot_localization/.../realsense.launch.py` | `e76c90d42a56ae471b3e398d69bbb98cbd2f3407d0f4ac6417e70811f6ff6871` | 旧版 launch，不读取 env |
| `install/g1_pkg/.../camera_config.yaml` | `a83155b9c5d14d3f086a4c795c8a8341e435450f50b16ec55b2d1d6133e9866e` | 旧版配置，硬编码 `419522072874` |

`g1hk` 的 install diff 关键点:

```diff
-def env_override(name: str, fallback: str = "") -> str:
-    value = os.environ.get(name, "").strip()
-    return value if value else (fallback or "")
-
 ...
-    front_serial = env_override(
-        "BOTBRAIN_FRONT_D435I_SERIAL",
-        (_raw_cam.get('front') or {}).get('serial_number', '')
-    )
+    front_serial = (_raw_cam.get('front') or {}).get('serial_number', '')
```

```diff
 camera_configuration:
   front:
     type: "d435i"
-    serial_number: ""
+    serial_number: "419522072874"
```

判断:

- `g1edu` 现在代表“新方案”: 每台机器的 D435i serial 从 `/etc/botbrain/robot.env` 注入。
- `g1hk` 现在代表“旧运行态”: 源码虽然新，但运行的 install 产物仍旧硬编码 serial。
- 如果只看 Git HEAD，会误判两台机器完全一致；实际必须同时检查 `install/` 和正在运行的容器环境。

### systemd 自启动差异

`g1edu`:

```ini
WorkingDirectory=/data/botbrain_ws/botbrain_project-main
ExecStart=/bin/bash -c "source ./botbrain_ws/robot_select.sh && /usr/bin/docker compose up dev bringup localization navigation rosa yolo jetson_stats state_machine -d"
ExecStop=/usr/bin/docker compose stop dev bringup localization navigation rosa yolo jetson_stats state_machine
```

`g1hk`:

```ini
WorkingDirectory=/data/unitree/botbrain_ws
ExecStart=/bin/bash -c "source ./botbrain_ws/robot_select.sh && /usr/bin/docker compose up dev bringup rosa jetson_stats state_machine -d"
ExecStop=/usr/bin/docker compose stop dev bringup jetson_stats state_machine
```

差异:

- `g1edu` 开机服务明确启动 `localization`、`navigation`、`yolo`。
- `g1hk` 开机服务不明确启动 `localization` 和 `navigation`。
- 但两台机器 `localization/navigation` 容器都有 `restart=always`，所以 `g1hk` 开机后这些容器仍可被 Docker 自动恢复。

这会导致:

1. `g1edu` 开机同时拉起更多重服务，相机初始化更容易和 map、TF、导航、YOLO、前端服务竞争 CPU、内存、USB、DDS 发现。
2. `g1hk` 由 Docker 恢复已存在的 `localization` 容器，启动路径更短，实际行为更接近“恢复上次状态”。
3. 如果 `g1edu` 开机时 `localization` 被重启、被 SIGKILL、或内部 launch 某个节点阻塞，浏览器连上 9090 时就可能看到没有相机画面的状态。

### 容器状态差异

排查时观察到:

| 项目 | g1edu | g1hk |
|---|---|---|
| `g1_robot_localization` | running，但曾刚重启 | running，约开机后持续运行 |
| `g1_robot_navigation` | `Exited (137)` | running |
| `g1_robot_yolo` | running | running |
| `g1_robot_web_server_prod` | running | running |

`g1edu` inspect:

```text
/g1_robot_localization restart=always ... oom=false exit=0
/g1_robot_navigation restart=always ... oom=false exit=137
```

`g1hk` inspect:

```text
/g1_robot_localization restart=always ... oom=false exit=0
/g1_robot_navigation restart=always ... oom=false exit=0
```

`Exited (137)` 一般表示进程收到 SIGKILL 或被强制终止。这里 Docker inspect 显示 `OOMKilled=false`，所以不一定是 Docker 判定的 OOM，但仍然说明启动链路存在强制停止或资源压力。

### 当前相机状态

排查时两台机器当前状态都能看到 D435i USB 设备:

```text
Bus 002 Device 003: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i
```

两台机器 ROS lifecycle 当前均可到 active:

```text
/g1_robot/front_camera active [3]
/g1_robot/realsense_compressed_node active [3]
```

两台机器 `/g1_robot/compressed_camera` 当前均可采样到约 6Hz:

```text
average rate: 5.7 - 6.0 Hz
```

注意:

- 当 RealSense 已被 ROS 节点占用时，直接运行 `rs-enumerate-devices -s` 可能报 `RS2_USB_STATUS_BUSY` 和 `No device detected`。
- 这不等于相机未插入，也不等于序列号错误。
- 真正确认物理 serial 时，应先停止占用相机的容器，再运行枚举命令。

### 资源状态差异

| 项目 | g1edu | g1hk |
|---|---|---|
| `/` 使用率 | 93% | 33% |
| `/data` 使用率 | 13% | 7% |
| 内存 used | 约 10Gi/15Gi | 约 11Gi/15Gi |
| swap used | 约 4.7Gi/7.6Gi | 约 2.3Gi/7.6Gi |

判断:

- `g1edu` 根分区 93% 是明确的运维风险。
- Docker、apt、日志、临时文件、ROS 构建缓存都可能受根分区空间影响。
- 高 swap 使用会放大开机阶段的抖动，尤其是 `localization/navigation/yolo` 同时启动时。

## 根因判断

### 根因 1: “同一套代码”不等于“同一套运行态”

两台机器源码一致，但 `install/` 不一致，且当前运行容器 env 也不一致。

`g1hk` 能工作，实际依赖的是旧 install 产物中硬编码的 `419522072874`。这使它绕过了新的 env 注入路径。

`g1edu` 使用的是新 install 产物，依赖 `/etc/botbrain/robot.env` 注入 `243722074823`。如果 env、容器 recreate、物理 serial、USB 初始化任一环节不稳定，就更容易暴露问题。

### 根因 2: 自启动链路不一致

`g1edu` 开机一次性启动更多重服务，`g1hk` 只启动核心服务并依赖 Docker 自动恢复其它容器。

这会影响:

- 相机节点创建时间。
- lifecycle 自动 configure/activate 的时机。
- 9090 桥接服务连接到 ROS 图时能否发现话题。
- 前端打开 `/cockpit` 时是否已经有 `/g1_robot/compressed_camera` publisher。

### 根因 3: g1edu 的 ROS2 通信配置存在机器本地修改

`g1edu` 增加了无线网卡 `wlx94ba06f26399` 到 CycloneDDS interface 列表。若该网卡状态不稳定，可能影响 ROS2 discovery。

这类配置应作为机器本地配置管理，而不是混在通用仓库文件里。

### 根因 4: g1edu 资源压力更大

根分区 93%、swap 使用较高、`navigation Exited (137)`，说明 `g1edu` 启动阶段更容易受资源压力和强制终止影响。即使相机配置正确，也可能在开机窗口期表现为:

- lifecycle node 不出现。
- raw/compressed topic publisher 为 0。
- 前端 9090 已连接，但图像 topic 尚未建立。
- 修复脚本等待 `/g1_robot/front_camera` 或 `/g1_robot/compressed_camera` 超时。

## 推荐的标准化目标

建议收敛为以下架构:

1. Git 仓库完全相同，`src/` 是唯一源码真相。
2. `install/` 是构建产物，不允许长期手工漂移。
3. 每台机器的差异只放在机器本地配置:
   - `/etc/botbrain/robot.env`
   - 如确实需要，`/etc/botbrain/cyclonedds_config.xml`
   - 如确实需要，`/etc/botbrain/robot.local.env`
4. Compose 只负责注入机器本地配置到容器。
5. `realsense.launch.py` 从环境变量读取 D435i serial，环境变量为空时才回退 YAML 默认值。
6. `camera_config.yaml` 中不硬编码具体机器的 D435i serial，默认保持空字符串。
7. `systemd` 自启动采用分层:
   - core: `dev bringup rosa jetson_stats state_machine web`
   - camera: D435i RealSense 节点和 compressed bridge
   - localization/navigation/yolo: 等 camera 和基础 ROS 图健康后再启动
8. 9090/前端不应承担“启动相机”的职责，它只消费 ROS topic。

## 建议操作步骤

### 第 1 步: 固化每台机器的本地配置

`g1edu`:

```bash
sudo mkdir -p /etc/botbrain
sudo tee /etc/botbrain/robot.env >/dev/null <<'EOF'
BOTBRAIN_FRONT_D435I_SERIAL=243722074823
EOF
sudo chmod 0644 /etc/botbrain/robot.env
```

`g1hk`:

```bash
sudo mkdir -p /etc/botbrain
sudo tee /etc/botbrain/robot.env >/dev/null <<'EOF'
BOTBRAIN_FRONT_D435I_SERIAL=419522072874
EOF
sudo chmod 0644 /etc/botbrain/robot.env
```

注意:

- 目前 `g1edu` 已配置为 `243722074823`。
- 目前 `g1hk` 已配置为 `419522072874`。
- 后续不要再把具体 serial 写回仓库 YAML。

### 第 2 步: 停止相机占用后确认 g1edu 物理 serial

只有当需要最终确认 USB 物理序列号时才执行，执行期间会中断相机画面:

```bash
cd /data/botbrain_ws/botbrain_project-main
docker compose stop -t 30 localization

lsusb | grep -Ei 'Intel|RealSense|8086'
rs-enumerate-devices -s
ls -l /dev/v4l/by-id 2>/dev/null || true

docker compose up -d localization
```

预期:

- `rs-enumerate-devices -s` 应显示 `243722074823`。
- 如果显示其它 serial，优先相信 `rs-enumerate-devices`，然后更新 `/etc/botbrain/robot.env`。
- 如果停止 `localization` 后仍然 `RS2_USB_STATUS_BUSY`，说明还有其它进程独占相机，需要用下面命令查占用:

```bash
sudo fuser -v /dev/video* /dev/media* 2>/dev/null || true
ps aux | grep -Ei 'realsense|rs-enumerate|camera' | grep -v grep
```

### 第 3 步: 让 g1hk 也切到新方案，但要有回滚点

`g1hk` 目前能正常工作，但它的正常依赖旧 install 产物。建议先记录当前状态，再切换。

在 `g1hk`:

```bash
cd /data/unitree/botbrain_ws

git rev-parse HEAD
git status --short
docker compose ps -a
docker compose logs --tail=200 localization > /tmp/g1hk-localization-before-env-migration.log
```

然后重新构建 install 产物，使 `install/` 与 `src/` 一致:

```bash
cd /data/unitree/botbrain_ws

docker compose exec -T dev bash -lc '
  set -e
  cd /botbrain_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-select bot_localization g1_pkg
'
```

如果项目已有标准 build 脚本，应优先使用项目脚本，目标是让这些文件一致:

```bash
sha256sum \
  botbrain_ws/src/bot_localization/bot_localization/launch/realsense.launch.py \
  botbrain_ws/install/bot_localization/share/bot_localization/launch/realsense.launch.py \
  botbrain_ws/src/g1_pkg/config/camera_config.yaml \
  botbrain_ws/install/g1_pkg/share/g1_pkg/config/camera_config.yaml
```

然后强制重建正在运行的容器，让 env_file 真正进入容器:

```bash
cd /data/unitree/botbrain_ws
docker compose up -d --force-recreate --no-deps localization
```

验证:

```bash
docker compose exec -T localization bash -lc '
  env | grep BOTBRAIN_FRONT_D435I_SERIAL
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 lifecycle get /g1_robot/front_camera
  ros2 lifecycle get /g1_robot/realsense_compressed_node
  timeout 10 ros2 topic hz /g1_robot/compressed_camera
'
```

预期:

```text
BOTBRAIN_FRONT_D435I_SERIAL=419522072874
active [3]
active [3]
average rate: about 6 Hz
```

### 第 4 步: 统一 systemd 自启动策略

短期可以先把 `g1edu` 调整得更接近 `g1hk`，减少开机瞬间的重服务并发启动。

当前 `g1edu`:

```bash
docker compose up dev bringup localization navigation rosa yolo jetson_stats state_machine -d
```

当前 `g1hk`:

```bash
docker compose up dev bringup rosa jetson_stats state_machine -d
```

建议的行业最佳实践不是简单照抄 `g1hk`，而是拆成分层启动:

1. `botbrain-core.service`
   - 启动 core 服务: `dev bringup rosa jetson_stats state_machine web_server`
2. `botbrain-camera.service`
   - 启动 RealSense camera 相关容器或 camera 子服务。
   - 等 `/g1_robot/front_camera` lifecycle active 和 `/g1_robot/compressed_camera` 有 publisher。
3. `botbrain-apps.service`
   - 启动 `navigation/yolo` 等依赖 camera 或较重的服务。

如果暂时不拆 Compose 服务，建议至少在 `g1edu` 上不要把 `navigation` 和 `yolo` 与 camera 同时作为 systemd 第一批启动项。可以先用 runbook 手动验证:

```bash
cd /data/botbrain_ws/botbrain_project-main

docker compose up -d dev bringup rosa jetson_stats state_machine
sleep 15
docker compose up -d localization
sleep 30
docker compose up -d navigation yolo
```

确认稳定后再改 systemd。

### 第 5 步: 把 CycloneDDS 机器差异移出通用仓库

如果 `g1edu` 需要无线网卡 `wlx94ba06f26399`，建议使用机器本地 DDS 配置:

```bash
sudo mkdir -p /etc/botbrain
sudo cp /data/botbrain_ws/botbrain_project-main/botbrain_ws/cyclonedds_config.xml /etc/botbrain/cyclonedds_config.xml
```

然后 Compose 注入:

```yaml
environment:
  CYCLONEDDS_URI: file:///etc/botbrain/cyclonedds_config.xml
volumes:
  - /etc/botbrain/cyclonedds_config.xml:/etc/botbrain/cyclonedds_config.xml:ro
```

更稳妥的做法是:

- 仓库保留 `cyclonedds_config.xml.template`。
- 每台机器在 `/etc/botbrain/` 放自己的实际配置。
- 部署脚本检查接口存在:

```bash
ip link show enP8p1s0
ip link show wlx94ba06f26399
```

### 第 6 步: 清理 g1edu 根分区

`g1edu` 根分区 93% 是必须处理的运维风险。建议目标降到 80% 以下。

先只查看:

```bash
df -h /
sudo du -xh /var/lib/docker --max-depth=1 | sort -h | tail -n 20
sudo du -xh /var/log --max-depth=1 | sort -h | tail -n 20
journalctl --disk-usage
docker system df
```

谨慎清理:

```bash
sudo journalctl --vacuum-time=7d
docker image prune
docker builder prune
```

不要在不了解影响时执行:

```bash
docker system prune -a
```

因为它可能删除还需要的镜像，导致离线环境下无法恢复。

## 开机后标准验收命令

在 `g1edu`:

```bash
cd /data/botbrain_ws/botbrain_project-main

docker compose ps -a

docker compose exec -T localization bash -lc '
  echo "serial env=$BOTBRAIN_FRONT_D435I_SERIAL"
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 lifecycle get /g1_robot/front_camera || true
  ros2 lifecycle get /g1_robot/realsense_compressed_node || true
  ros2 topic info --verbose /g1_robot/front_camera/color/image_raw || true
  timeout 10 ros2 topic hz /g1_robot/compressed_camera || true
'
```

合格标准:

```text
serial env=243722074823
/g1_robot/front_camera active [3]
/g1_robot/realsense_compressed_node active [3]
/g1_robot/front_camera/color/image_raw publishers: >= 1
/g1_robot/compressed_camera about 6 Hz
```

检查 9090 桥接服务:

```bash
cd /data/botbrain_ws/botbrain_project-main
docker compose ps -a | grep -Ei 'web|foxglove|rosa|state|bridge'
ss -ltnp | grep ':9090' || true
docker compose logs --tail=150 web_server 2>/dev/null || true
docker compose logs --tail=150 foxglove 2>/dev/null || true
docker compose logs --tail=150 state_machine 2>/dev/null || true
```

如果 `/cockpit` 能连接 9090 但没有图像，优先检查 ROS topic，而不是前端:

```bash
docker compose exec -T localization bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 topic list | grep -E "front_camera|compressed_camera"
  ros2 topic info --verbose /g1_robot/compressed_camera
'
```

## 故障分型

### 类型 A: USB 层看不到 D435i

症状:

```bash
lsusb | grep -Ei 'Intel|RealSense|8086'
```

没有输出。

处理:

1. 检查 USB 线、供电、Hub。
2. 换 USB 3 口。
3. 查看内核日志:

```bash
dmesg -T | grep -Ei 'usb|uvc|realsense|8086'
```

### 类型 B: USB 能看到，但 ROS 没有 `/g1_robot/front_camera`

症状:

```text
/g1_robot/front_camera state: <absent>
raw publisher: 0
compressed publisher: 0
```

处理:

```bash
cd /data/botbrain_ws/botbrain_project-main
docker compose ps -a localization
docker compose logs --tail=300 localization | grep -Ei 'realsense|front_camera|serial|error|failed|exception|died|No device'
docker compose exec -T localization bash -lc 'env | grep BOTBRAIN_FRONT_D435I_SERIAL'
```

重点判断:

- env 是否是 `243722074823`。
- `install/` 是否与 `src/` 一致。
- `localization` 是否反复重启。
- 是否有其它进程占用 `/dev/video*`。

### 类型 C: `/g1_robot/front_camera` active，但 `/g1_robot/compressed_camera` 没有

症状:

```bash
ros2 lifecycle get /g1_robot/front_camera
ros2 lifecycle get /g1_robot/realsense_compressed_node
ros2 topic info /g1_robot/compressed_camera
```

处理:

```bash
docker compose logs --tail=200 localization | grep -Ei 'compressed|realsense_compressed|image_raw|error|failed'
```

重点判断:

- `realsense_compressed_node` 是否 active。
- raw image topic 是否存在 publisher。
- compressed 节点订阅的 topic 名是否仍是 `/g1_robot/front_camera/color/image_raw`。

### 类型 D: ROS 图像正常，但浏览器看不到

症状:

```text
/g1_robot/compressed_camera 有 6Hz
/cockpit 无图像
```

处理:

1. 查 9090 服务是否监听:

```bash
ss -ltnp | grep ':9090' || true
```

2. 查 bridge/web logs:

```bash
cd /data/botbrain_ws/botbrain_project-main
docker compose logs --tail=200 web_server 2>/dev/null || true
docker compose logs --tail=200 foxglove 2>/dev/null || true
docker compose logs --tail=200 state_machine 2>/dev/null || true
```

3. 浏览器刷新前先确认 topic:

```bash
docker compose exec -T localization bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  timeout 5 ros2 topic hz /g1_robot/compressed_camera
'
```

## 最小改造建议

短期建议:

1. 保持 `g1edu` `/etc/botbrain/robot.env` 为 `243722074823`。
2. 停止占用后确认一次 `g1edu` 物理 serial，排除相机被换过或线缆接错。
3. 清理 `g1edu` 根分区，使 `/` 使用率低于 80%。
4. 把 `g1edu` 的 `navigation/yolo` 从第一批 systemd 启动中移出，先观察开机 camera 是否稳定。
5. 给 `fix_d435i_camera-sz.sh` 增加一个模式: 只做诊断，不自动 restart；因为在开机抖动期盲目 restart 可能让问题更难复现。

中期建议:

1. 在两台机器都使用 env-based serial 方案。
2. 让 `g1hk` 重新构建 `install/`，去掉硬编码 serial。
3. 统一 systemd unit 模板，只保留路径和本地 env 差异。
4. 把 DDS interface 配置移到 `/etc/botbrain/`。
5. 拆分 camera service，不要让 AprilTag、导航、定位等节点直接独占 D435i 设备。它们应订阅 ROS image topic。

长期建议:

1. CI 或部署脚本检查 `src/` 与 `install/` 关键文件是否一致。
2. 开机后自动执行 camera health check:

```bash
ros2 lifecycle get /g1_robot/front_camera
ros2 lifecycle get /g1_robot/realsense_compressed_node
timeout 10 ros2 topic hz /g1_robot/compressed_camera
```

3. 将 health check 结果暴露到 `/health`，让 `/cockpit` 能显示“camera initializing / no publisher / bridge disconnected / ok”等明确状态。
4. 对 `localization` 拆分职责:
   - `realsense_camera`
   - `image_compressor`
   - `localization_algorithms`
   - `navigation`

这样 D435i 作为共享传感器只被一个 camera driver 进程打开，其它进程通过 ROS topic 消费图像，避免设备独占冲突。

## 最终判断

`g1hk` 当前开机正常，不是因为它和 `g1edu` 运行态完全一致，而是因为它保留了旧的 install 产物和较轻的 systemd 启动路径。

`g1edu` 当前开机不稳定，主要风险来自:

1. 已切到新的 env-based serial 路径，要求容器 env、install 产物、物理 serial 三者完全一致。
2. systemd 开机直接启动 `localization/navigation/yolo`，并发重服务较多。
3. 根分区和 swap 压力更大。
4. DDS 配置有机器本地修改，可能影响 ROS2 发现稳定性。

推荐方向不是回退到硬编码 serial，而是把 `g1hk` 也迁移到同一套 env-based 方案，同时把自启动和机器本地配置标准化。这样两台 G1 EDU 才是真正“共用同一套代码仓库，只在 `/etc/botbrain/robot.env` 保留机器差异”。
