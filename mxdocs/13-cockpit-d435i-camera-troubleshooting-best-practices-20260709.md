# 13 Cockpit D435i 画面不稳定的分层排障与修复最佳实践

日期: 2026-07-09  
适用范围: `g1edu:/data/botbrain_ws/botbrain_project-main`，`http://192.168.100.30/cockpit`，ROS 2 Humble，Docker Compose，Intel RealSense D435i。

## 1. 结论

Cockpit 打开后先连 `9090`，只能说明浏览器到 `rosbridge_websocket` 这段通了，不能说明 D435i 图像链路正常。

浏览器画面完整链路是:

```text
Browser /cockpit
  -> ws://192.168.100.30:9090
  -> rosbridge_websocket
  -> /g1_robot/compressed_camera
  -> /g1_robot/realsense_compressed_node
  -> /g1_robot/front_camera/color/image_raw
  -> /g1_robot/front_camera
  -> realsense2_camera_node
  -> librealsense / UVC
  -> D435i USB hardware
```

所以排障必须从前到后、从后到前都能定位。最重要的判断是:

- `9090` 通，但 `/g1_robot/compressed_camera` 没 publisher: 不是前端问题，是 ROS 图像链路问题。
- `/g1_robot/compressed_camera` 没 publisher，且 `/g1_robot/front_camera/color/image_raw` 没 publisher: 是 RealSense driver 或相机 owner 容器问题。
- `/g1_robot/front_camera` 节点 absent: `realsense.launch.py` 没跑起来、容器已退出、或 RealSense 节点启动失败。
- `rs-enumerate-devices` 在容器内找不到，但 host `lsusb` 能看到: 容器不在运行、容器设备映射/权限/环境有问题，或 RealSense 被其他进程占用。
- `Device or resource busy`: 多进程抢 D435i、旧进程未退出、USB/librealsense 状态卡住，或同一容器里多个节点/launch 重复打开。
- `Exited (137)`: 容器被 SIGKILL。常见原因是 `docker compose restart` 先发 SIGTERM 后超时强杀、内存压力、或系统/脚本外部 kill。不能把它当作 lifecycle 小问题处理。

## 2. 当前 g1edu 现场观察

这次脚本输出:

```text
detected D435i serial : <none>
/g1_robot/front_camera/color/image_raw publishers: 0
/g1_robot/compressed_camera publishers: 0
waiting: no /g1_robot/front_camera and no raw publisher
/g1_robot/front_camera state: <absent>
/g1_robot/realsense_compressed_node state: <absent>
```

这说明问题不在 cockpit 前端，也不是 compressed 节点单独卡住，而是更上游的 `front_camera` RealSense lifecycle 节点没有出现在 ROS graph。

进一步现场检查显示:

```text
g1_robot_localization  exited  Exited (137)
g1_robot_navigation    exited  Exited (137)
```

host 层能看到 D435i:

```text
Bus 002 Device 003: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i
/dev/v4l/by-id/...253243060636-video-index0 -> ../../video0
/dev/v4l/by-id/...253243060636-video-index1 -> ../../video1
/dev/v4l/by-id/...253243060636-video-index2 -> ../../video2
/dev/v4l/by-id/...253243060636-video-index3 -> ../../video3
```

但 `localization` 日志里有大量:

```text
control_transfer returned error ... Device or resource busy, number: 16
```

这个组合很关键:

1. 相机硬件并非完全断开，host 能枚举到。
2. 当前运行时没有 `/g1_robot/front_camera`，因为相机 owner 容器已经退出。
3. 退出前 RealSense driver 遇到 `Device or resource busy`。
4. `docker compose restart localization` 期间 Docker events 显示先 `signal=15`，约 10 秒后 `signal=9`，最后 `exitCode=137`。也就是说，脚本重启时可能没有给 RealSense/librealsense 足够时间干净释放设备。

因此当前应先按“容器是否活着”和“D435i 是否被干净释放”排查，而不是继续重复刷新 cockpit。

## 3. 标准排障顺序

每次 cockpit 没画面，不要先重启一堆服务。按下面顺序查，避免把原始问题覆盖掉。

### Step 1: 确认浏览器到 9090 是否通

在 host 上:

```bash
ss -ltnp | grep ':9090' || true
```

期望:

```text
LISTEN 0 128 0.0.0.0:9090
```

从本机或其他机器可以用 WebSocket 快速订阅:

```bash
node - <<'JS'
const ws = new WebSocket('ws://192.168.100.30:9090');
const timer = setTimeout(() => { console.log('timeout'); process.exit(1); }, 8000);
ws.onopen = () => {
  console.log('open');
  ws.send(JSON.stringify({
    op: 'subscribe',
    topic: '/g1_robot/compressed_camera',
    type: 'sensor_msgs/CompressedImage'
  }));
};
ws.onmessage = e => {
  const msg = JSON.parse(e.data);
  if (msg.op === 'publish') {
    console.log('got image bytes=' + (msg.msg?.data || '').length);
    clearTimeout(timer);
    process.exit(0);
  }
};
ws.onerror = e => console.error(e.message || e);
JS
```

判断:

- WebSocket 打不开: 查 `bringup` / rosbridge。
- WebSocket 能打开但收不到图: 继续查 ROS topic，不要先改前端。

### Step 2: 查容器状态

```bash
cd /data/botbrain_ws/botbrain_project-main

docker compose ps -a --format 'table {{.Name}}\t{{.Service}}\t{{.State}}\t{{.Status}}' |
  grep -E 'bringup|localization|jetson_stats|state_machine|web_server_prod|navigation|camera|NAME'
```

重点看:

```text
g1_robot_bringup       running
g1_robot_localization  running
g1_robot_state_machine running
g1_robot_web_server_prod running
```

如果 `localization` 是 `exited`，`/g1_robot/front_camera` 一定不会稳定出现。先查退出原因:

```bash
docker inspect g1_robot_localization \
  --format 'OOMKilled={{.State.OOMKilled}} ExitCode={{.State.ExitCode}} Error={{.State.Error}} FinishedAt={{.State.FinishedAt}}'

docker compose logs --tail=250 localization |
  grep -Ei 'realsense|front_camera|serial|device|busy|error|failed|exception|traceback|killed|died'
```

判断:

- `ExitCode=137` 且 `OOMKilled=false`: 多半是 Docker stop/restart 超时后 SIGKILL，或外部 kill。
- `OOMKilled=true`: 内存压力，先减服务或拆 camera service。
- 日志有 `Device or resource busy`: 查 V4L2/librealsense 占用和重复 owner。

### Step 3: 查 ROS graph

从 `dev` 容器查，避免在已经退出的 `localization` 容器里执行:

```bash
cd /data/botbrain_ws/botbrain_project-main

docker compose exec -T dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 daemon stop >/dev/null 2>&1 || true
  echo NODES
  ros2 node list --no-daemon | grep -E "front_camera|realsense_compressed|rosbridge" || true
  echo TOPICS
  ros2 topic list --no-daemon | grep -E "front_camera|compressed_camera" | sort || true
  echo RAW
  ros2 topic info --verbose --no-daemon /g1_robot/front_camera/color/image_raw || true
  echo COMPRESSED
  ros2 topic info --verbose --no-daemon /g1_robot/compressed_camera || true
'
```

期望正常状态:

```text
/g1_robot/front_camera
/g1_robot/realsense_compressed_node
/g1_robot/front_camera/color/image_raw Publisher count: 1
/g1_robot/compressed_camera Publisher count: 1
```

进一步确认帧率:

```bash
docker compose exec -T dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  timeout 10 ros2 topic hz /g1_robot/front_camera/color/image_raw
  timeout 10 ros2 topic hz /g1_robot/compressed_camera
'
```

判断:

- raw 有 hz，compressed 没 hz: 查 `realsense_compressed_node` lifecycle、订阅 topic、QoS。
- raw 没 publisher，front_camera active: 查 RealSense node 日志。
- front_camera absent: 查 launch 是否启动、容器是否退出、RealSense driver 是否崩溃。

### Step 4: 查 lifecycle

```bash
docker compose exec -T dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 lifecycle get --no-daemon --spin-time 2 /g1_robot/front_camera || true
  ros2 lifecycle get --no-daemon --spin-time 2 /g1_robot/realsense_compressed_node || true
'
```

修复:

```bash
docker compose exec -T dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  for n in /g1_robot/front_camera /g1_robot/realsense_compressed_node; do
    state=$(ros2 lifecycle get --no-daemon --spin-time 2 "$n" 2>/dev/null | awk "NR==1{print \$1}")
    echo "$n state=${state:-absent}"
    case "$state" in
      unconfigured)
        ros2 lifecycle set --no-daemon "$n" configure || true
        sleep 2
        ros2 lifecycle set --no-daemon "$n" activate || true
        ;;
      inactive)
        ros2 lifecycle set --no-daemon "$n" activate || true
        ;;
    esac
  done
'
```

注意: 如果 node 是 `absent`，不要反复 lifecycle set。必须回到容器/launch/USB 层。

### Step 5: 查 host 是否枚举到 D435i

```bash
lsusb | grep -Ei 'Intel|RealSense|8086' || true
ls -l /dev/v4l/by-id 2>/dev/null | grep -Ei 'Intel|RealSense|D435|253243060636|243722074823' || true
```

判断:

- host `lsusb` 都没有: USB 线、供电、Hub、相机固件/重置问题。
- host 有，容器没有: 容器没运行、`/dev` 映射/权限、privileged、或容器内工具环境问题。

当前 g1edu 这次 host 能看到:

```text
8086:0b3a RealSense D435i
253243060636-video-index0..3
```

这意味着这次不是“硬件完全没插上”，而是运行链路/占用/容器退出问题。

### Step 6: 查 D435i 是否被其他进程占用

```bash
sudo fuser -v /dev/video* 2>/dev/null || true
sudo lsof /dev/video* 2>/dev/null || true
pgrep -af 'realsense|rs-enumerate|realsense-viewer|v4l2|apriltag|VideoCapture' || true
```

如果看到以下进程在打开 D435i，先停掉:

- `v4l2_apriltag_trigger.py`
- `realsense-viewer`
- 手工运行的第二个 `realsense2_camera_node`
- 旧的 `rs-enumerate-devices` 卡住进程
- 任何 `cv2.VideoCapture(/dev/video*)` 程序

行业最佳实践是 D435i 只能有一个硬件 owner:

```text
realsense2_camera_node owns USB device
其他 AprilTag / Web / localization / navigation 只订阅 ROS topics
```

### Step 7: 查 RealSense serial 配置

当前已采用 per-robot env 的方向，推荐以 `/etc/botbrain/robot.env` 为准。

检查:

```bash
sudo cat /etc/botbrain/robot.env
```

期望类似:

```text
BOTBRAIN_FRONT_D435I_SERIAL=243722074823
```

但注意: 本次 host `/dev/v4l/by-id` 显示的是 `253243060636`。如果这是当前实际插在 g1edu 上的 D435i，就必须核对“实际安装的相机”与“预期 serial”是否发生过更换。不要盲目把配置改来改去。正确流程:

1. 物理确认这台机器上应使用哪只 D435i。
2. 用 `lsusb`、`rs-enumerate-devices -s`、`/dev/v4l/by-id` 三者交叉确认 serial。
3. 如果确认换了相机，再更新 `/etc/botbrain/robot.env`。
4. 不要再把真实 serial 写回 Git 跟踪的 YAML。

容器内检查 env:

```bash
docker compose run --rm --no-deps localization bash -lc '
  printenv BOTBRAIN_FRONT_D435I_SERIAL
  rs-enumerate-devices -s || true
'
```

如果 `docker compose run` 能枚举，但常驻 `localization` 不能，重点看常驻容器是否退出或设备被占用。

## 4. 对 `fix_d435i_camera-sz.sh` 的改进建议

现在脚本已经比旧版本好，但还需要避免几个误判。

### 4.1 不要只在 `localization` 容器里枚举相机

当前脚本 Step 2:

```text
detected D435i serial : <none>
WARN: rs-enumerate-devices did not find a RealSense camera in localization
```

这可能有三种含义:

1. host 真的没有枚举到 D435i。
2. `localization` 容器不在运行。
3. `localization` 容器里正在/曾经有 RealSense owner 卡住，导致 `rs-enumerate-devices` 看不到或超时。

脚本应同时检查 host:

```bash
lsusb | grep -Ei 'Intel|RealSense|8086'
ls -l /dev/v4l/by-id | grep -Ei 'RealSense|D435'
```

输出应分开写:

```text
host lsusb D435i          : present / absent
host v4l serial           : 253243060636
container rs-enumerate    : present / absent / container-not-running
configured serial         : ...
```

### 4.2 重启前先判断容器是否健康

如果 `localization` 已经 `exited`，不要直接 `docker compose restart localization` 后等 ROS node。应先:

```bash
docker compose ps -a localization
docker inspect g1_robot_localization --format 'OOMKilled={{.State.OOMKilled}} ExitCode={{.State.ExitCode}}'
docker compose logs --tail=200 localization
```

脚本输出应明确:

```text
ERROR: localization is exited(137), front_camera cannot exist.
```

### 4.3 对 `docker compose restart localization` 增加 timeout

当前 Docker events 显示 `restart localization` 时:

```text
signal=15
10s later signal=9
exitCode=137
```

RealSense/librealsense 退出需要释放 USB 设备。10 秒可能不够，强杀后更容易出现设备 busy。

建议改成:

```bash
docker compose stop -t 30 localization
sleep 3
docker compose up -d --no-deps localization
```

不要用默认短 timeout 直接 restart。

### 4.4 如果日志出现 `Device or resource busy`，先做占用检查

脚本遇到以下日志:

```text
Device or resource busy, number: 16
```

应自动提示:

```bash
sudo fuser -v /dev/video* 2>/dev/null || true
sudo lsof /dev/video* 2>/dev/null || true
pgrep -af 'realsense|v4l2|apriltag|VideoCapture|realsense-viewer'
```

如果存在 V4L2 AprilTag，必须先停它。参考 `12-d435i-apriltag-shared-camera-best-practice-20260709.md`。

### 4.5 不要自动 patch Git 里的 serial YAML

当前最佳实践已经改成:

```text
/etc/botbrain/robot.env
  BOTBRAIN_FRONT_D435I_SERIAL=...
```

脚本不应再默认 patch:

```text
botbrain_ws/src/g1_pkg/config/camera_config.yaml
botbrain_ws/install/g1_pkg/share/g1_pkg/config/camera_config.yaml
```

建议:

- 默认 `PATCH_SERIAL=0`。
- 只读取并提示差异。
- 真要更新，更新 `/etc/botbrain/robot.env`，并要求人工确认。

### 4.6 jtop 不应混在 D435i 修复脚本主路径里

`jtop_publisher` 属于 health/system info，不是 D435i 图像链路。D435i 修复脚本可以附带检查，但不应因为 jtop absent 干扰相机排障。

建议分开:

- `fix_d435i_camera-sz.sh`: 只修 camera pipeline。
- `fix_health_jtop.sh`: 修 `/g1_robot/diagnostic_stats`。

## 5. 推荐的一次性恢复流程

当 cockpit 没画面、且确认 `localization` 已退出或 RealSense busy 时，使用这个流程。

### 5.1 记录现场

```bash
cd /data/botbrain_ws/botbrain_project-main

date
docker compose ps -a | grep -E 'localization|bringup|state_machine|web_server_prod|jetson_stats|navigation'
docker inspect g1_robot_localization --format 'OOMKilled={{.State.OOMKilled}} ExitCode={{.State.ExitCode}} FinishedAt={{.State.FinishedAt}}' || true
docker compose logs --tail=160 localization > /tmp/localization_camera_fail.log 2>&1 || true
```

### 5.2 停掉可能抢相机的进程

```bash
pgrep -af 'v4l2_apriltag|realsense-viewer|rs-enumerate|VideoCapture' || true

sudo fuser -v /dev/video* 2>/dev/null || true
sudo lsof /dev/video* 2>/dev/null || true
```

如果确认某个调试进程占用相机，停止它。不要杀 `dockerd`，不要重启整机作为第一选择。

### 5.3 干净停止 camera owner

当前 camera owner 临时在 `localization` 里:

```bash
docker compose stop -t 30 localization
sleep 3
```

如果仍有 video device 占用，再查:

```bash
sudo fuser -v /dev/video* 2>/dev/null || true
```

### 5.4 确认 host 仍能看到 D435i

```bash
lsusb | grep -Ei 'Intel|RealSense|8086'
ls -l /dev/v4l/by-id 2>/dev/null | grep -Ei 'RealSense|D435'
```

如果 host 看不到，执行硬件层处理:

- 重新插拔 D435i。
- 检查 USB-C 线和 Hub 供电。
- 优先使用稳定的 USB3 口。
- 必要时重启 Jetson，但这是最后手段。

### 5.5 启动 localization

```bash
docker compose up -d --no-deps localization
sleep 10
docker compose ps -a localization
docker compose logs --tail=120 localization |
  grep -Ei 'realsense|front_camera|serial|device|busy|error|failed|exception'
```

如果马上 `Exited (137)`，不要继续等 lifecycle，直接查容器退出原因。

### 5.6 激活 lifecycle

```bash
docker compose exec -T dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  for n in /g1_robot/front_camera /g1_robot/realsense_compressed_node; do
    echo "== $n =="
    ros2 lifecycle get --no-daemon --spin-time 3 "$n" || true
    ros2 lifecycle set --no-daemon "$n" configure || true
    sleep 2
    ros2 lifecycle set --no-daemon "$n" activate || true
    sleep 2
    ros2 lifecycle get --no-daemon --spin-time 3 "$n" || true
  done
'
```

### 5.7 验证 raw 和 compressed

```bash
docker compose exec -T dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 topic info --verbose --no-daemon /g1_robot/front_camera/color/image_raw
  ros2 topic info --verbose --no-daemon /g1_robot/compressed_camera
  timeout 10 ros2 topic hz /g1_robot/front_camera/color/image_raw
  timeout 10 ros2 topic hz /g1_robot/compressed_camera
'
```

期望:

```text
/g1_robot/front_camera/color/image_raw Publisher count: 1
/g1_robot/compressed_camera Publisher count: 1
average rate: > 0
```

## 6. 长期修复方向

当前最大架构问题是: D435i browser 画面依赖 `localization` 容器，而 `localization` 同时运行 RealSense、Open3D localization、map/submap、可能还有其他重负载节点。任何一个重节点导致容器退出，cockpit 画面也会消失。

行业最佳实践是拆出独立 camera service。

### 6.1 目标架构

```text
camera service
  -> owns D435i USB
  -> runs realsense2_camera_node
  -> runs realsense_compressed_node
  -> publishes /g1_robot/front_camera/*
  -> publishes /g1_robot/compressed_camera

localization service
  -> subscribes camera/depth/scan topics
  -> can fail/restart without killing camera

apriltag/manipulation/navigation/web
  -> subscribe ROS topics only
```

### 6.2 Compose 服务边界

新增 `camera` 服务，短期可以继承 `nav3d`:

```yaml
camera:
  extends: nav3d
  container_name: g1_robot_camera
  command: ["bash", "-lc", "source install/setup.bash && ros2 launch bot_localization realsense.launch.py"]
  restart: always
```

然后从 `localization_3d.launch.py` 或 `start_localization.sh` 里去掉 RealSense include，避免两个容器同时启动 `front_camera`。

注意: 拆分时必须保证 D435i 只有一个 owner。如果 `camera` 和 `localization` 同时 launch `realsense.launch.py`，就会出现 `Device or resource busy`。

### 6.3 state_machine 管理

如果继续让 `state_machine` 管理 lifecycle，需要把 camera 节点从“localization 附属节点”变成“camera service 节点”。`state_machine` 应支持:

- 等待 lifecycle service 出现，而不是启动时 absent 就永久跳过。
- 对 `front_camera`、`realsense_compressed_node` 的 configure/activate 做幂等重试。
- 不因为 localization 重启就 bring_down camera。

### 6.4 脚本策略

最终建议拆成三个脚本:

```text
check_cockpit_camera.sh
  只读检查，不重启、不改配置。

recover_camera_runtime.sh
  做 stop -t 30 / up -d / lifecycle activate / verify。

diagnose_realsense_usb.sh
  做 host lsusb、v4l by-id、fuser/lsof、dmesg、serial env 检查。
```

不要让一个脚本同时 patch 配置、重启 localization、修 jtop、改 lifecycle、验证 web。脚本越大，越容易掩盖真正故障。

## 7. 快速判断表

| 现象 | 最可能层级 | 关键命令 | 修复方向 |
|---|---|---|---|
| 9090 不通 | bringup/rosbridge | `ss -ltnp | grep 9090` | `docker compose up -d bringup` |
| 9090 通但 cockpit 无图 | ROS image pipeline | `ros2 topic info /g1_robot/compressed_camera` | 查 compressed/raw publisher |
| compressed 无 publisher，raw 有 publisher | compressed node | lifecycle get compressed | activate/restart compressed node |
| raw 无 publisher，front_camera active | RealSense driver | localization logs | 查 RealSense error/USB |
| front_camera absent | launch/container | `docker compose ps -a localization` | 启动/修复 localization 或 camera service |
| localization Exited 137 | 容器被杀 | `docker inspect` / docker events | stop timeout、内存、外部 kill |
| host lsusb 无 D435i | 硬件/USB | `lsusb` | 线缆、供电、Hub、重插 |
| host 有，container rs 无 | 容器/占用 | `fuser/lsof /dev/video*` | 停抢占进程、重启 camera owner |
| Device busy | 多 owner/未释放 | logs + fuser/lsof | 停 V4L2/AprilTag/realsense-viewer，干净 stop |
| serial 不一致 | 配置/换相机 | `/etc/botbrain/robot.env` + by-id | 人工确认后改 robot.env |

## 8. 当前最可能的问题点

结合本次输出，当前最可能不是前端问题，而是:

1. `localization` 退出为 `137`，所以 `/g1_robot/front_camera` 不存在。
2. 重启过程中 RealSense/librealsense 没有干净释放，日志出现 `Device or resource busy`。
3. host 能看到 D435i serial `253243060636`，但之前配置讨论过 `243722074823`，需要人工确认当前机器实际安装的是哪只 D435i。
4. 内存压力较高，`15Gi` 内存已用约 `13Gi`，swap 已用约 `5.5Gi`。虽然 Docker inspect 显示 `OOMKilled=false`，但这种状态下重服务更容易卡死或被强杀。

短期建议:

```bash
cd /data/botbrain_ws/botbrain_project-main

docker compose stop -t 30 localization
sleep 3
sudo fuser -v /dev/video* 2>/dev/null || true
lsusb | grep -Ei 'Intel|RealSense|8086'
ls -l /dev/v4l/by-id | grep -Ei 'RealSense|D435'
docker compose up -d --no-deps localization
sleep 15
docker compose ps -a localization
```

如果再次 `Exited (137)`，先不要继续运行修复脚本，改查:

```bash
docker compose logs --tail=300 localization
docker events --since 10m --until 0s | grep g1_robot_localization
free -h
```

中期建议是尽快把 D435i 拆到独立 `camera` service，避免 cockpit 画面被 3D localization、navigation、Open3D 内存压力拖垮。

