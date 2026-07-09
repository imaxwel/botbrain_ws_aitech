# G1 机器人 Zenoh 跨机器通信部署指南

> 适用场景：Workstation 需要通过 RViz2 直连 G1 查看建图/导航效果。
>
> 原理：将 G1 的 ROS 2 通信中间件从 CycloneDDS（依赖多播，跨机器不稳定）切换到 Zenoh（TCP 直连，参考项目 `g1_3d_nav_ros2` 同款方案）。

---

## 前置条件

- G1 上已有 `botbrain_project` 部署，docker compose 正常运行
- G1 宿主机已安装 `ros-humble-rmw-zenoh-cpp`（`dpkg -l | grep rmw-zenoh` 确认）
- G1 上有 `g1_nav_final:latest` 镜像（`docker images | grep g1_nav_final` 确认）
- Workstation 已安装 `ros-humble-rmw-zenoh-cpp` 和 `ros-humble-rviz2`

---

## 步骤一：安装 Zenoh 到 botbrain 容器并 commit 镜像

> ⚠️ 以下假设 docker compose 项目目录为 `/data/botbrain_ws/botbrain_project-main`，
> 如实际路径不同请替换。

```bash
ssh unitree@<G1_IP>

# ========== 1.1 给 nav3d 镜像装 Zenoh（fast_lio / localization 用）==========

# 复制 vendor 库到容器
docker cp /opt/ros/humble/opt/zenoh_cpp_vendor g1_robot_fast_lio:/opt/ros/humble/opt/

# 安装 deb 包
docker cp /var/cache/apt/archives/ros-humble-rmw-zenoh-cpp_*.deb g1_robot_fast_lio:/tmp/
docker exec g1_robot_fast_lio dpkg -i /tmp/ros-humble-rmw-zenoh-cpp_*.deb

# 配置动态库搜索路径
docker exec g1_robot_fast_lio bash -c \
    "echo /opt/ros/humble/opt/zenoh_cpp_vendor/lib > /etc/ld.so.conf.d/zenoh.conf && ldconfig"

# 停容器 → commit 覆盖原镜像
docker stop g1_robot_fast_lio
docker commit g1_robot_fast_lio botbotrobotics/botbrain:nav3d
echo "✅ nav3d 已更新（含 Zenoh）"

# ========== 1.2 给 base 镜像装 Zenoh（bringup / state_machine / foxglove 等用）==========

docker cp /opt/ros/humble/opt/zenoh_cpp_vendor g1_robot_bringup:/opt/ros/humble/opt/
docker cp /var/cache/apt/archives/ros-humble-rmw-zenoh-cpp_*.deb g1_robot_bringup:/tmp/
docker exec g1_robot_bringup dpkg -i /tmp/ros-humble-rmw-zenoh-cpp_*.deb
docker exec g1_robot_bringup bash -c \
    "echo /opt/ros/humble/opt/zenoh_cpp_vendor/lib > /etc/ld.so.conf.d/zenoh.conf && ldconfig"

docker stop g1_robot_bringup
docker commit g1_robot_bringup botbotrobotics/botbrain:base
echo "✅ base 已更新（含 Zenoh）"

# ========== 1.3 确认镜像 tag 指向新版本 ==========
docker images botbotrobotics/botbrain:nav3d --format "nav3d → {{.ID}}"
docker images botbotrobotics/botbrain:base   --format "base  → {{.ID}}"
```

---

## 步骤二：修改 docker-compose.yaml

```bash
cd <项目目录>   # 例如 /data/botbrain_ws/botbrain_project-main

# 备份原文件
cp docker-compose.yaml docker-compose.yaml.bak.zenoh

# 2.1 切换 RMW：cyclonedds → zenoh
sed -i 's|RMW_IMPLEMENTATION: rmw_cyclonedds_cpp|RMW_IMPLEMENTATION: rmw_zenoh_cpp|g' docker-compose.yaml

# 2.2 替换通信配置
sed -i 's|CYCLONEDDS_URI: "file:///botbrain_ws/cyclonedds_config.xml"|ZENOH_CONFIG_OVERRIDE: '"'"'mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'"'"'|g' docker-compose.yaml

# 2.3 在 state_machine 服务前插入 zenoh 路由器服务
sed -i '/^  state_machine:/i\  zenoh:\n    image: g1_nav_final:latest\n    network_mode: host\n    ipc: host\n    container_name: g1_robot_zenoh\n    command: ["bash", "-lc", "source /opt/ros/humble/setup.bash && ZENOH_CONFIG_OVERRIDE='\''listen/endpoints=[\\"tcp/0.0.0.0:7448\\"]'\'' ros2 run rmw_zenoh_cpp rmw_zenohd"]\n    restart: always\n' docker-compose.yaml

# 2.4 验证
echo "=== RMW 检查 ==="
grep "RMW_IMPLEMENTATION" docker-compose.yaml | head -5
echo "=== Zenoh 服务检查 ==="
grep -A4 "^  zenoh:" docker-compose.yaml
```

---

## 步骤三：重启服务（zenoh 路由器必须先启动）

```bash
cd <项目目录>

# 停止所有服务
docker compose stop

# 清理残留的 zenohd 进程
pkill -f rmw_zenohd 2>/dev/null; sleep 2

# 先启动 zenoh 路由器
docker compose up -d zenoh
sleep 3
ss -tlnp | grep 7448   # 确认 7448 端口在监听

# 再启动其他服务
docker compose up -d bringup state_machine foxglove fast_lio localization
```

---

## 步骤四：Workstation 端使用

```bash
# 每次打开 RViz2 前执行
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp:<G1_IP>:7448"]'

# 验证连接（Publisher count 必须 > 0）
ros2 daemon stop  >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
sleep 3
ros2 topic info /Odometry_loc | grep "Publisher count"
# → Publisher count: 1  ✅

# 建图可视化
rviz2 -d <项目路径>/configs/g1_mapping_rviz2.rviz

# 导航可视化
rviz2 -d <项目路径>/configs/g1_nav_loc_rviz2.rviz
```

> `<G1_IP>` 默认 `192.168.100.30`。Fixed Frame 选 `camera_init`。

---

## 回滚方法

如果 Zenoh 方案有问题需要退回 CycloneDDS：

```bash
cd <项目目录>

# 恢复备份
cp docker-compose.yaml.bak.zenoh docker-compose.yaml

# 重启
docker compose stop
docker compose up -d bringup state_machine foxglove fast_lio localization
```

> 回滚后镜像里的 Zenoh 包仍然保留，不影响使用。

---

## 常见问题

| 问题 | 排查 |
|------|------|
| `Publisher count: 0` | G1 服务未启动、zenoh 路由器未监听 7448、G1 IP 不对 |
| `Unknown topic` | `ros2 daemon stop && ros2 daemon start` 刷新 |
| RViz2 打开后无数据 | Fixed Frame 手动选 `camera_init` |
| 镜像 commit 后丢失 | 确认没执行过 `docker compose down`（容器被删除会丢 commit） |
