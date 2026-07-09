# 11 两台 G1 EDU 共用同一套代码与 D435i per-robot 配置最佳实践

日期: 2026-07-09

目标: `g1edu` 与 `g1hk` 两台 Unitree G1 EDU 使用同一个 Git 代码仓库、同一个提交历史和同一套 Docker/ROS 代码，同时保持每台机器自己的 D435i serial、ROS domain、前端环境变量和部署目录差异。

## 1. 当前实际情况

| 项目 | g1edu | g1hk |
| --- | --- | --- |
| SSH alias | `g1edu` | `g1hk` |
| hostname | `unitree-g1-nx` | `unitree-g1-nx` |
| 主要 IP | `192.168.100.30` | `192.168.37.204` |
| 部署目录 | `/data/botbrain_ws/botbrain_project-main` | `/data/unitree/botbrain_ws` |
| Git commit | `587433e` | `587433e` |
| Git 状态 | 有 3 个本地改动 | 干净 |
| D435i serial 证据 | `rs-enumerate-devices -s` 显示 `243722074823` | `localization` 日志显示 `419522072874` |
| `/etc/botbrain/robot.env` | 不存在 | 不存在 |
| `camera_config.yaml` | `serial_number: "419522072874"` | `serial_number: "419522072874"` |
| `realsense.launch.py` | 只读 YAML serial，无 `BOTBRAIN_*` 环境变量覆盖 | 同左 |
| `robot_config.yaml` | `robot_name: "g1_robot"` | 同左 |
| Docker Compose | ROS 容器未加载 per-robot env；`env_file` 只用于 frontend | 同左 |

结论:

1. 两台机器已经是同一个提交 `587433e`，这对合并成同一套代码是好事。
2. 当前 D435i serial 被硬编码在仓库内的 `botbrain_ws/src/g1_pkg/config/camera_config.yaml` 和运行产物 `botbrain_ws/install/.../camera_config.yaml`。
3. `g1edu` 的真实 D435i serial 与仓库配置不一致，所以同一套代码直接部署到两台机器时必然有一台会错。
4. 机器身份配置尚未从代码仓库中分离。
5. 两台机器都使用 `robot_name: "g1_robot"`。如果它们在同一个 ROS domain 同时在线，会互相发现同名节点和 topic。

## 2. 行业最佳实践

采用三层边界:

1. **代码层 Git 管理**  
   放通用源码、launch、compose、默认模板、示例配置。不放真实 D435i serial、不放真实 Supabase key、不放机器 IP。

2. **机器身份层 host-local 管理**  
   每台机器人各自保存 `/etc/botbrain/robot.env`，内容包括 D435i serial、ROS_DOMAIN_ID、机器人编号、可选 IP。这个文件不进 Git。

3. **运行产物层 build 生成**  
   `botbrain_ws/install` 是 colcon build 产物。不要手改 `install` 当作长期方案。代码合并后必须 rebuild 或由 CI/部署脚本生成。

不要用下面这些方式长期维护:

- 为每台机器人维护一个长期 Git 分支，只为了不同 serial。
- 在部署前手动 `sed` 修改 `camera_config.yaml`。
- 只改 `install/.../camera_config.yaml`，不改源码。
- 把 `/etc/botbrain/robot.env`、`frontend/.env`、真实 Supabase key 提交到 Git。

## 3. 目标目录和配置形态

部署目录可以不同，代码仍然可以相同:

```bash
# g1edu
/data/botbrain_ws/botbrain_project-main

# g1hk
/data/unitree/botbrain_ws
```

每台机器都建立同名本机配置:

```bash
/etc/botbrain/robot.env
```

推荐内容:

```bash
# g1edu: /etc/botbrain/robot.env
BOTBRAIN_ROBOT_ID=g1edu
BOTBRAIN_FRONT_D435I_SERIAL=243722074823
ROS_DOMAIN_ID=31
```

```bash
# g1hk: /etc/botbrain/robot.env
BOTBRAIN_ROBOT_ID=g1hk
BOTBRAIN_FRONT_D435I_SERIAL=419522072874
ROS_DOMAIN_ID=32
```

说明:

- `BOTBRAIN_FRONT_D435I_SERIAL` 是 RealSense driver 使用的设备选择参数。
- `ROS_DOMAIN_ID` 用来隔离两台机器人 ROS 2 graph。两台机器人如果在同一网络上，必须不同。
- 暂时不建议立刻修改 `robot_name: "g1_robot"`，因为前端、topic profile、已有脚本都依赖 `/g1_robot/...`。先用不同 `ROS_DOMAIN_ID` 做隔离。
- 如果以后要让一个中央控制台同时管理两台机器人，再规划 namespace 改造，例如 `/g1edu`、`/g1hk`。

## 4. 一次性代码改造步骤

以下改造只做一次，提交到同一个 Git 分支。之后两台机器人只维护自己的 `/etc/botbrain/robot.env`。

### Step 1: 先处理 g1edu 的未提交改动

`g1edu` 当前有 3 个本地改动。不要直接覆盖。

```bash
ssh g1edu
cd /data/botbrain_ws/botbrain_project-main

git status --short
git diff -- botbrain_ws/src/g1_pkg/launch/fast_lio.launch.py
git diff -- botbrain_ws/src/g1_pkg/scripts/grid_accumulator.py
git diff -- botbrain_ws/src/g1_right_dex3/unitree_g1_dex3_stack/config/apriltag_button_number15.yaml
```

如果这些改动是有效修复，先提交:

```bash
git switch -c chore/fleet-per-robot-config
git add botbrain_ws/src/g1_pkg/launch/fast_lio.launch.py \
        botbrain_ws/src/g1_pkg/scripts/grid_accumulator.py \
        botbrain_ws/src/g1_right_dex3/unitree_g1_dex3_stack/config/apriltag_button_number15.yaml
git commit -m "Preserve local G1 navigation and Dex3 changes"
```

如果只是临时实验，先 stash:

```bash
git stash push -m "local g1edu experiments before fleet config migration"
git switch -c chore/fleet-per-robot-config
```

### Step 2: 给 Docker Compose 注入 per-robot env

修改 `docker-compose.yaml`。核心原则: 所有 ROS 容器都能读到 `/etc/botbrain/robot.env`。

推荐在文件顶部增加 anchor:

```yaml
x-robot-env: &robot-env
  env_file:
    - ${BOTBRAIN_ROBOT_ENV_FILE:-/etc/botbrain/robot.env}

services:
  base:
    <<: *robot-env
    image: botbotrobotics/botbrain:base
    ...

  nav3d:
    <<: *robot-env
    image: botbotrobotics/botbrain:nav3d
    ...
```

同时给这些服务或模板也加上同一个 env:

- `base`
- `nav3d`
- `yolo_base`
- `manipulation`
- `dev_dex3`
- `builder_dex3`

原因:

- `localization` 继承 `nav3d`，D435i 在这里启动。
- `bringup`、`state_machine`、`jetson_stats`、`navigation` 等继承 `base`，需要同一个 `ROS_DOMAIN_ID`。
- `yolo`、`manipulation`、`dex3` 也可能参与 ROS graph，不能留在默认 domain。

如果某台机器没有 sudo 权限创建 `/etc/botbrain/robot.env`，可以用项目本地文件作为临时替代:

```bash
mkdir -p .botbrain
cat > .botbrain/robot.env <<'EOF'
BOTBRAIN_ROBOT_ID=g1edu
BOTBRAIN_FRONT_D435I_SERIAL=243722074823
ROS_DOMAIN_ID=31
EOF

cat >> .env <<'EOF'
BOTBRAIN_ROBOT_ENV_FILE=./.botbrain/robot.env
EOF
```

但生产建议仍然使用 `/etc/botbrain/robot.env`，因为它与代码仓库无关。

### Step 3: 修改 RealSense launch，使 env 覆盖 YAML

修改:

```bash
botbrain_ws/src/bot_localization/bot_localization/launch/realsense.launch.py
```

在 import 区域已有 `import os`，增加一个小函数:

```python
def env_override(name: str, fallback: str = "") -> str:
    value = os.environ.get(name, "").strip()
    if value:
        return value
    return fallback or ""
```

把现有 serial 读取逻辑:

```python
front_serial = (_raw_cam.get('front') or {}).get('serial_number', '')
back_serial  = (_raw_cam.get('back') or {}).get('serial_number', '')
```

改成:

```python
front_serial = env_override(
    "BOTBRAIN_FRONT_D435I_SERIAL",
    (_raw_cam.get('front') or {}).get('serial_number', '')
)
back_serial = env_override(
    "BOTBRAIN_BACK_D435I_SERIAL",
    (_raw_cam.get('back') or {}).get('serial_number', '')
)
```

保留当前已经正确的字符串类型传参:

```python
"serial_no": ParameterValue("" if serial is None else str(serial), value_type=str),
```

原因: RealSense serial 是纯数字。如果 ROS launch 把它当整数处理，会导致参数类型错误或驱动匹配失败。

### Step 4: 仓库内默认 camera_config 不再写真实 serial

修改:

```bash
botbrain_ws/src/g1_pkg/config/camera_config.yaml
```

把:

```yaml
serial_number: "419522072874"
```

改成:

```yaml
serial_number: ""
```

这样仓库默认配置不绑定任何一台机器。生产环境由 `/etc/botbrain/robot.env` 显式指定 serial。

注意: 不要手工长期维护 `botbrain_ws/install/g1_pkg/share/g1_pkg/config/camera_config.yaml`。它应该由 build 生成。紧急修复可以临时改 install，但合并方案必须改 src 并 rebuild。

### Step 5: 增加模板文件

建议提交一个模板，帮助后续新增机器人:

```bash
deploy/robot.env.example
```

内容:

```bash
BOTBRAIN_ROBOT_ID=g1edu-example
BOTBRAIN_FRONT_D435I_SERIAL=
BOTBRAIN_BACK_D435I_SERIAL=
ROS_DOMAIN_ID=31
```

同时确认 `.gitignore` 覆盖真实本机配置:

```gitignore
.botbrain/
robot.env
*.local.env
frontend/.env.local
```

真实 `frontend/.env` 也属于部署环境配置。当前两台机器的 Supabase 配置不同，建议长期也迁移到 host-local 或部署系统管理，不应作为业务源码差异。

## 5. 两台机器上的本机配置步骤

### g1edu

```bash
ssh g1edu

sudo install -d -m 0755 /etc/botbrain
sudo tee /etc/botbrain/robot.env >/dev/null <<'EOF'
BOTBRAIN_ROBOT_ID=g1edu
BOTBRAIN_FRONT_D435I_SERIAL=243722074823
ROS_DOMAIN_ID=31
EOF
sudo chmod 0644 /etc/botbrain/robot.env

cat /etc/botbrain/robot.env
```

校验 serial:

```bash
rs-enumerate-devices -s
```

如果 host 上 `rs-enumerate-devices` 与 `/dev/v4l/by-id` 不一致，以 librealsense 的 `rs-enumerate-devices -s` 和 `realsense2_camera_node` 启动日志为准。当前 `g1edu` 曾出现 `/dev/v4l/by-id` 显示旧 serial 的情况，不建议只看 udev symlink 下结论。

### g1hk

```bash
ssh g1hk

sudo install -d -m 0755 /etc/botbrain
sudo tee /etc/botbrain/robot.env >/dev/null <<'EOF'
BOTBRAIN_ROBOT_ID=g1hk
BOTBRAIN_FRONT_D435I_SERIAL=419522072874
ROS_DOMAIN_ID=32
EOF
sudo chmod 0644 /etc/botbrain/robot.env

cat /etc/botbrain/robot.env
```

当前 `g1hk` host 环境没有 `rs-enumerate-devices`，并且容器内枚举可能被运行中的 driver 占用，出现 `RS2_USB_STATUS_BUSY`。可用运行日志确认:

```bash
cd /data/unitree/botbrain_ws
docker compose logs --tail=1500 localization | grep -E "Device (with serial number|Serial No)"
```

当前日志确认 `g1hk` 使用:

```text
Device Serial No: 419522072874
```

## 6. 构建与部署步骤

### 推荐 Git 流程

建议使用一个中央 Git remote，所有机器人只 checkout 同一分支:

```bash
# 在 g1edu 完成代码改造后
cd /data/botbrain_ws/botbrain_project-main
git status --short
git add docker-compose.yaml \
        botbrain_ws/src/bot_localization/bot_localization/launch/realsense.launch.py \
        botbrain_ws/src/g1_pkg/config/camera_config.yaml \
        deploy/robot.env.example \
        .gitignore
git commit -m "Move robot-specific camera serial to host env"
git push origin chore/fleet-per-robot-config
```

在 `g1hk` 拉取同一分支:

```bash
ssh g1hk
cd /data/unitree/botbrain_ws
git fetch origin
git switch chore/fleet-per-robot-config
git pull --ff-only
```

如果暂时没有中央 Git remote，可以用 patch 过渡，但不建议长期这样:

```bash
# g1edu
git format-patch -1 HEAD --stdout > /tmp/fleet-per-robot-config.patch
scp /tmp/fleet-per-robot-config.patch g1hk:/tmp/

# g1hk
cd /data/unitree/botbrain_ws
git am /tmp/fleet-per-robot-config.patch
```

### 每台机器 rebuild

因为当前运行时使用 `source install/setup.bash`，改了 `src` 后必须更新 `install`。

`g1edu`:

```bash
ssh g1edu
cd /data/botbrain_ws/botbrain_project-main

docker compose run --rm builder_base
docker compose restart localization state_machine bringup jetson_stats navigation yolo
```

`g1hk`:

```bash
ssh g1hk
cd /data/unitree/botbrain_ws

docker compose run --rm builder_base
docker compose restart localization state_machine bringup jetson_stats navigation yolo
```

如果只想缩短 build 时间，可以先尝试:

```bash
docker compose run --rm builder_base bash -lc '
  cd /botbrain_ws &&
  source /opt/ros/humble/setup.bash &&
  colcon build --packages-select bot_localization g1_pkg bot_state_machine
'
```

但长期建议让 `builder_base` 的完整构建保持可用。

## 7. 验证清单

### g1edu

```bash
ssh g1edu
cd /data/botbrain_ws/botbrain_project-main

docker compose exec -T localization bash -lc '
  printenv BOTBRAIN_ROBOT_ID
  printenv BOTBRAIN_FRONT_D435I_SERIAL
  printenv ROS_DOMAIN_ID
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 lifecycle get --no-daemon /g1_robot/front_camera
  ros2 lifecycle get --no-daemon /g1_robot/realsense_compressed_node
  ros2 topic info --verbose --no-daemon /g1_robot/front_camera/color/image_raw
  ros2 topic info --verbose --no-daemon /g1_robot/compressed_camera
  timeout 10 ros2 topic hz /g1_robot/compressed_camera
'
```

期望:

- `BOTBRAIN_FRONT_D435I_SERIAL=243722074823`
- `/g1_robot/front_camera` 是 `active`
- `/g1_robot/front_camera/color/image_raw` 有 publisher
- `/g1_robot/compressed_camera` 有 publisher 且 `topic hz` 大于 0
- cockpit: `http://192.168.100.30/cockpit`

### g1hk

```bash
ssh g1hk
cd /data/unitree/botbrain_ws

docker compose exec -T localization bash -lc '
  printenv BOTBRAIN_ROBOT_ID
  printenv BOTBRAIN_FRONT_D435I_SERIAL
  printenv ROS_DOMAIN_ID
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 lifecycle get --no-daemon /g1_robot/front_camera
  ros2 lifecycle get --no-daemon /g1_robot/realsense_compressed_node
  ros2 topic info --verbose --no-daemon /g1_robot/front_camera/color/image_raw
  ros2 topic info --verbose --no-daemon /g1_robot/compressed_camera
  timeout 10 ros2 topic hz /g1_robot/compressed_camera
'
```

期望:

- `BOTBRAIN_FRONT_D435I_SERIAL=419522072874`
- `/g1_robot/front_camera` 是 `active`
- `/g1_robot/compressed_camera` 有帧
- cockpit: `http://192.168.37.204/cockpit`

## 8. 回滚方案

如果改造后某台机器相机无法启动:

1. 确认 env 已进入容器:

   ```bash
   docker compose exec -T localization bash -lc 'env | grep -E "BOTBRAIN|ROS_DOMAIN"'
   ```

2. 确认 launch 使用的 install 产物已经更新:

   ```bash
   docker compose exec -T localization bash -lc '
     grep -n "BOTBRAIN_FRONT_D435I_SERIAL\\|env_override\\|serial_no" \
       /botbrain_ws/install/bot_localization/share/bot_localization/launch/realsense.launch.py
   '
   ```

3. 临时回退到 YAML serial:

   ```bash
   # 只作为紧急恢复，不作为长期方案
   sed -i 's/serial_number: ""/serial_number: "419522072874"/' \
     botbrain_ws/src/g1_pkg/config/camera_config.yaml
   docker compose run --rm builder_base
   docker compose restart localization
   ```

4. Git 回退代码提交:

   ```bash
   git revert <commit-that-added-per-robot-env>
   docker compose run --rm builder_base
   docker compose restart localization
   ```

## 9. 后续架构建议

优先级从高到低:

1. **把 D435i 从 localization 拆到独立 `camera` service**  
   cockpit 看图只需要 RealSense driver 和 compressed node，不应依赖 Open3D、map server、FAST-LIO 或 3D localization。

2. **统一 systemd 启动入口，但保留每台机器不同 root path**  
   可以用 `/opt/botbrain/current` symlink 指向实际部署目录:

   ```bash
   # g1edu
   sudo ln -sfn /data/botbrain_ws/botbrain_project-main /opt/botbrain/current

   # g1hk
   sudo ln -sfn /data/unitree/botbrain_ws /opt/botbrain/current
   ```

   systemd 只写:

   ```text
   WorkingDirectory=/opt/botbrain/current
   ExecStart=/usr/bin/docker compose up -d --remove-orphans bringup jetson_stats state_machine localization web_server_prod
   ```

3. **把前端运行环境也 host-local 化**  
   当前两台机器 `frontend/.env` 内容不同。长期建议让前端也读取本机 env 文件或由部署系统注入，不把真实 Supabase key 作为源码差异。

4. **中央控制台场景再做 namespace 改造**  
   只要每台机器人各自访问自己的 cockpit，`/g1_robot` 可以暂时保留。若同一个 ROS bridge 或同一个控制台要同时看两台机器人，应进一步把 `robot_name`、topic profile、frontend robot profile 都参数化。

## 10. 最终判断

当前最小正确方案不是“把两台机器的 `camera_config.yaml` 改成不同值”，而是:

```text
同一 Git commit
  + 每台机器自己的 /etc/botbrain/robot.env
  + Docker Compose 把 env 注入 ROS 容器
  + realsense.launch.py 用 env 覆盖 YAML serial
  + 每台机器 rebuild install
```

这样新增第三台 G1 EDU 时，只需要:

1. checkout 同一套代码；
2. 写一个新的 `/etc/botbrain/robot.env`；
3. rebuild/restart；
4. 按验证清单确认 camera topic。

代码仓库不再因为 D435i serial 分裂。
