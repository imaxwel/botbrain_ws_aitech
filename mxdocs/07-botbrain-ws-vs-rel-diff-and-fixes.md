# 07 botbrain_ws 与 botbrain-rel/BotBrain 差异分析及修复清单

生成时间: 2026-07-07
执行机器: unitree-g1-nx（192.168.37.204）
参考基线: `/data/unitree/botbrain-rel/BotBrain`（摄像头画面+3D雷达点云已调通）
当前目标: `/data/unitree/botbrain_ws`（/health、/cockpit 已可正常访问，但部分功能未迁移）

---

## TL;DR — 当前状态

| 功能 | botbrain-rel/BotBrain | botbrain_ws | 状态 |
|------|----------------------|-------------|------|
| Web 服务（80端口） | ✅（历史参考） | ✅ 正常 | 已完成 |
| /health 电池/Jetson 数据 | ✅ | ✅（已修复，见06文档） | 已修复 |
| 摄像头画面（/cockpit） | ✅ | ❌ localization 容器崩溃 | **需修复** |
| 3D 雷达点云（/cockpit） | ✅ | ❌ fast_lio/localization 未正常运行 | **需修复** |
| 机械臂操控 | ❌（REL无此包） | ⚠️ 容器在跑但未深度验证 | 新增功能 |
| foxglove_bridge | ❌（REL无） | ✅ 容器在跑 | 新增 |

---

## 1. 两个工作区结构对比

### 1.1 docker-compose.yaml 主要差异

**botbrain-rel/BotBrain（基线，调好的版本）**

```yaml
# base 镜像：botbotrobotics/botbrain:base
# 无 nav3d 镜像，localization 用 base
# builder_base 直接 extends: base，不含 fast_lio/open3d_loc
# localization: ros2 launch bot_localization localization.launch.py （2D定位）
# 无 fast_lio、无 foxglove、无 manipulation
# 容器命名：dev/bringup/localization 等（无 g1_robot_ 前缀）
# web_server 的 env_file: .env（根目录）
```

**botbrain_ws（当前工作区）**

```yaml
# 新增 nav3d 基础镜像：botbotrobotics/botbrain:nav3d（含 Open3D 支持）
# base + nav3d 均挂载 ./deps/open3d141:/opt/open3d:ro
# base + nav3d 均挂载 /usr/local/include|lib:/opt/robot_sdk/...
# builder_base 改为 extends: nav3d，额外编译 fast_lio/open3d_loc/g1_manipulation_pkg
# localization: extends: nav3d，用 localization_3d.launch.py（3D ICP定位）
# 新增 fast_lio 容器（extends: nav3d，sleep 15 后启动）
# 新增 foxglove 容器（ros2 run foxglove_bridge）
# 新增 manipulation 容器（Dockerfile.manipulation）
# 所有容器 container_name 加 g1_robot_ 前缀
# web_server 的 env_file 改为 ./frontend/.env（而非根目录 .env）
# web_server_prod 增加 PORT=80 环境变量
```

### 1.2 新增 ROS2 包

| 包名 | 用途 |
|------|------|
| `fast_lio` | 基于 IMU+LiDAR 的里程计，为 3D 定位提供 odom |
| `fcl` | 碰撞检测库（供 g1_manipulation_pkg 使用，.git 空壳已删除） |
| `open3d_loc` | 基于 Open3D ICP 的 3D 全局定位（替代原 2D localization） |
| `g1_manipulation_pkg` | G1 右手 DEX3 机械臂操控栈 |
| `g1_right_dex3` | DEX3 驱动层（含 yolonas_ocr，整体 gitignore） |

### 1.3 新增 frontend 文件

botbrain-rel/BotBrain 有但 botbrain_ws 没有的（在 REL 中存在，WS 中已删）：
- `app/mission-control/` — 任务控制页面
- `app/providers.tsx`、`app/robot-runtime-providers.tsx`
- `components/mission-control/` 系列组件
- `contexts/MissionSupervisorContext.tsx`
- `services/mission-supervisor.ts`
- `types/mission-control.ts`

botbrain_ws 新增（REL 中没有）：
- `utils/ros/namespace.ts` — ROS namespace 统一管理（见06文档）

> **结论**：botbrain_ws/frontend 是从 BotBrain 仓库派生后裁剪了 mission-control 功能的版本，同时修复了 namespace 问题。

---

## 2. 摄像头画面不可用的根因

### 2.1 三处配置未从 botbrain-rel 同步

| 文件 | botbrain-rel（正确值） | botbrain_ws（当前错误值） |
|------|----------------------|-------------------------|
| `src/g1_pkg/config/camera_config.yaml` front.serial_number | `"419522072874"` | `"test"` |
| `src/bot_localization/bot_localization/launch/realsense.launch.py` serial_no 传参 | `ParameterValue("" if serial is None else str(serial), value_type=str)` | `serial`（原始值，纯数字会被推断为整数） |
| `src/bot_state_machine/config/camera.json` | 只含 front_camera + realsense_compressed_node（无 back_camera） | 多出 back_camera 节点块，导致状态机等待不存在的服务而卡住 |

**install 目录（已编译版本）的状态**：
- `install/g1_pkg/.../camera_config.yaml` — serial_number = "test"（**错误，需修复**）
- `install/bot_localization/.../realsense.launch.py` — 用原始 `serial`（**错误，需修复**）
- `install/bot_state_machine/.../camera.json` — 含 back_camera（**错误，需修复**）

### 2.2 localization 容器崩溃

localization 容器（g1_robot_localization）当前状态：`Exited (1) ~22分钟前`

崩溃日志关键行：
```
ImportError: librcl_action.so: cannot open shared object file: No such file or directory
```

**原因**：localization 容器使用 `nav3d` 镜像，其启动命令在 `source install/setup.bash` 前先 `export LD_LIBRARY_PATH=/opt/open3d/lib:`，但缺少冒号后的原始 `$LD_LIBRARY_PATH`（compose 配置中 `LD_LIBRARY_PATH` 环境变量没有 `$LD_LIBRARY_PATH` 展开），导致原 ROS2 路径（含 `librcl_action.so`）被覆盖丢失。

实际 compose command：
```yaml
command: ["bash", "-lc", "sleep 30 && source install/setup.bash && export LD_LIBRARY_PATH=/opt/open3d/lib:$LD_LIBRARY_PATH && ros2 launch g1_pkg localization_3d.launch.py"]
```

而 compose 环境变量块里：
```yaml
environment:
  LD_LIBRARY_PATH: /opt/open3d/lib   # 只有 open3d，没有 ROS2 的路径
```

容器启动时 `$LD_LIBRARY_PATH` 已经只有 `/opt/open3d/lib`，command 里的 `export` 再 `export LD_LIBRARY_PATH=/opt/open3d/lib:$LD_LIBRARY_PATH` 结果仍然只有 `/opt/open3d/lib`。`source install/setup.bash` 本应追加 ROS2 路径，但这一步在 export 之后，所以 ROS2 的 lib 路径没有问题——

> **重新分析**：崩溃发生在 `ros2` 命令调用时，而 `source install/setup.bash` 在前。实际错误是 `install/setup.bash` 能否找到 `librcl_action.so`。该 .so 位于 ROS2 humble 系统路径。
> 
> 真正原因：`install/setup.bash` 被 source 后，会把 install 目录下的 lib 追加到 `LD_LIBRARY_PATH`，但 nav3d 镜像的基础 `$LD_LIBRARY_PATH` 与 base 镜像不同，可能缺少系统 ROS lib 路径。或者 builder_base（`extends: nav3d`）在 nav3d 镜像里 `colcon build` 出的 install，其 setup.bash 里 AMENT_PREFIX_PATH 路径不含 `/opt/ros/humble`。

**需进一步验证**：在 nav3d 容器内手动复现该错误，检查 `LD_LIBRARY_PATH` 和 `AMENT_PREFIX_PATH`。

---

## 3. 3D 雷达点云不可用的根因

3D 点云依赖链：
```
LiDAR（MID360）→ fast_lio（g1_robot_fast_lio 容器）→ /Odometry → open3d_loc（g1_robot_localization）→ /tf(map→odom→body)
```

当前 `g1_robot_fast_lio` 状态：`Exited (137)` — 被 OOM 或外部信号 kill 掉，不是正常退出。

`g1_robot_localization` 状态：`Exited (1)` — 见上节崩溃分析。

两个容器都不在运行，cockpit 的 3D 点云视图因此无数据。

---

## 4. 修复步骤

### 4.1 修正摄像头配置（源码 + install 双修）

```bash
cd /data/unitree/botbrain_ws

# Step A: 修正 camera_config.yaml 序列号
sed -i 's/serial_number: "test"/serial_number: "419522072874"/' \
    botbrain_ws/src/g1_pkg/config/camera_config.yaml

# Step B: 修正 realsense.launch.py 序列号类型锁定
# 需要手动编辑：在文件顶部 import 区加一行，并修改 serial_no 赋值
# 文件：botbrain_ws/src/bot_localization/bot_localization/launch/realsense.launch.py

# Step C: 修正 camera.json，删除 back_camera 节点块
# 文件：botbrain_ws/src/bot_state_machine/config/camera.json

# Step D: 同步到 install 目录（容器内 root 复制）
docker exec g1_robot_bringup bash -lc \
  'cp /botbrain_ws/src/g1_pkg/config/camera_config.yaml \
   $(find /botbrain_ws/install -path "*g1_pkg*camera_config.yaml" | head -1)'

docker exec g1_robot_state_machine bash -lc \
  'cp /botbrain_ws/src/bot_state_machine/config/camera.json \
   /botbrain_ws/install/bot_state_machine/share/bot_state_machine/config/camera.json'
```

`realsense.launch.py` 改动如下（需在容器内 cp）：

```python
# 文件头部加：
from launch_ros.descriptions import ParameterValue

# make_camera_params 函数内，serial_no 那行改为：
"serial_no": ParameterValue("" if serial is None else str(serial), value_type=str),
```

同步到 install：
```bash
docker exec g1_robot_localization bash -lc \
  'cp /botbrain_ws/src/bot_localization/bot_localization/launch/realsense.launch.py \
   $(find /botbrain_ws/install -name "realsense.launch.py" | head -1)' 2>/dev/null || true
# 注意：localization 容器目前已退出，需先 docker compose start localization 或用 bringup 容器
```

修改完成后重启相机所属容器：
```bash
cd /data/unitree/botbrain_ws
docker compose restart localization
sleep 35
docker compose restart state_machine
sleep 50
```

### 4.2 排查并修复 localization 容器崩溃

```bash
# 1. 先在 nav3d 基础镜像里验证 librcl_action.so 是否存在
docker run --rm -it botbotrobotics/botbrain:nav3d bash -lc \
  "find /opt/ros -name 'librcl_action.so' 2>/dev/null; echo LD=\$LD_LIBRARY_PATH"

# 2. 复现崩溃
docker run --rm -it \
  -v /data/unitree/botbrain_ws/botbrain_ws:/botbrain_ws \
  -v /data/unitree/botbrain_ws/deps/open3d141:/opt/open3d:ro \
  -e LD_LIBRARY_PATH=/opt/open3d/lib \
  botbotrobotics/botbrain:nav3d bash -lc \
  "source /botbrain_ws/install/setup.bash && ros2 --version"

# 3. 如果 ros2 --version 失败，原因是 setup.bash 没有把 ROS humble lib 加进来
#    临时解法：在 localization command 里显式加 source /opt/ros/humble/setup.bash
```

**快速临时修复**（修改 docker-compose.yaml localization 的 command）：

```yaml
# 原：
command: ["bash", "-lc", "sleep 30 && source install/setup.bash && export LD_LIBRARY_PATH=/opt/open3d/lib:$LD_LIBRARY_PATH && ros2 launch g1_pkg localization_3d.launch.py"]

# 改为（在 source install/setup.bash 前先 source ROS humble）：
command: ["bash", "-lc", "sleep 30 && source /opt/ros/humble/setup.bash && source install/setup.bash && export LD_LIBRARY_PATH=/opt/open3d/lib:$LD_LIBRARY_PATH && ros2 launch g1_pkg localization_3d.launch.py"]
```

### 4.3 修复 fast_lio 容器 OOM 崩溃（ExitCode 137）

```bash
# 查看 fast_lio 日志
docker logs g1_robot_fast_lio --tail 50 2>&1

# 如果是 OOM kill，检查内存占用
free -h

# 临时：给 fast_lio 容器加 memory limit 或调整 sleep 时间
# 永久：需分析 fast_lio launch 参数，降低点云处理频率
```

### 4.4 验证修复效果

```bash
cd /data/unitree/botbrain_ws

# 摄像头验证
docker exec g1_robot_localization bash -lc \
  "source install/setup.bash && for n in front_camera realsense_compressed_node; do
     echo -n \"\$n: \"; ros2 lifecycle get /\$n; done"

# 验证 /compressed_camera 有 publisher
docker exec g1_robot_bringup bash -lc \
  "source install/setup.bash && ros2 topic info /compressed_camera | grep Publisher"

# fast_lio + 点云验证
docker exec g1_robot_fast_lio bash -lc \
  "source install/setup.bash && timeout 5 ros2 topic hz /cloud_registered" 2>/dev/null || \
docker exec g1_robot_bringup bash -lc \
  "source install/setup.bash && ros2 topic list | grep -E 'cloud|scan'"
```

---

## 5. botbrain.service 差异说明

`botbrain-rel/BotBrain/botbrain.service` 是一个**残缺的模板**（`[Service]` 段缺失所有实际配置），没有 WorkingDirectory/ExecStart 等字段，不可直接使用。

`botbrain_ws/botbrain.service` 同样是模板（WorkingDirectory=BOTBRAIN_WORKSPACE_PATH 占位符）。

**实际运行版本**（`/etc/systemd/system/botbrain.service`）已手动修改为真实路径：
- `WorkingDirectory=/data/unitree/botbrain_ws`
- `ExecStart` 启动的容器：`dev bringup rosa jetson_stats state_machine`（**注意：不含 localization、fast_lio、foxglove**）
- localization/fast_lio/foxglove 有各自的 `restart: always`，由 docker 守护进程自动重启

**结论**：systemd service 只负责初始 `docker compose up` 那些没有 `restart: always` 的容器（dev/bringup/rosa/jetson_stats/state_machine），其他容器靠 docker restart policy 自管理。

---

## 6. 前端 env_file 路径变化

| | botbrain-rel（旧） | botbrain_ws（新） |
|--|--|--|
| web_server env_file | `- .env`（根目录） | `- ./frontend/.env`（前端目录） |
| web_server_prod env_file | `- .env` | `- ./frontend/.env` |
| PORT | 未在 compose 设置 | `PORT=80` 显式设置 |

旧版本根目录 `.env` 的 `PORT=3000`，新版本前端自己的 `.env` 不含 PORT，改为 compose 层直接注入 `PORT=80`。这个改动是正确的，保持当前设置即可。

---

## 7. 现有正常运行的容器清单

```
g1_robot_web_server_prod   ← 80端口前端，正常
g1_robot_bringup           ← ROS bringup，正常
g1_robot_state_machine     ← 状态机，正常
g1_robot_yolo              ← YOLO 视觉，正常
g1_robot_jetson_stats      ← Jetson 监控，正常
g1_robot_foxglove          ← Foxglove bridge，正常
g1_robot_rosa              ← ROS 语音，刚重启（Up 几秒）
g1_robot_dev               ← 开发容器，正常
g1_robot_dev_dex3          ← DEX3 开发容器，正常
g1_robot_manipulation      ← 机械臂，正常
```

已退出：
```
g1_robot_fast_lio          ← ExitCode 137（OOM/kill），需修复
g1_robot_localization      ← ExitCode 1（librcl_action.so ImportError），需修复
g1_robot_navigation        ← ExitCode 137，需调查
```

---

## 8. 优先级建议

1. **P0 — 摄像头修复（3处配置）**：camera_config.yaml 序列号、realsense.launch.py 类型锁定、camera.json 删 back_camera。方案已清楚（见 botbrain-rel/BotBrain/docss/03-fix-camera-not-ready.md）。
2. **P1 — localization 容器崩溃**：先排查 nav3d 镜像内 librcl_action.so 可见性，临时方案是在 command 里前置 `source /opt/ros/humble/setup.bash`。
3. **P2 — fast_lio OOM**：查日志确认是否真的 OOM，考虑限制内存或降低点云频率。
4. **P3 — navigation 退出**：查日志，可能依赖 localization 的 TF 未就绪导致超时退出。

> 以上 P0+P1 修复后，cockpit 的摄像头画面和 3D 雷达点云预计可以恢复到与 botbrain-rel/BotBrain 相同的可用状态。
