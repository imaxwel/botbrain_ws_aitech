# G1 手臂控制迁移报告 — g1pilot → BotBrain

> 日期: 2026-04-07  
> 平台: Unitree G1 (29-DOF), Jetson (aarch64), ROS 2 Humble  
> 目标: 将 g1pilot 独立项目的手臂控制 (arm_controller / dx3_hand / interactive_marker) 迁移至 BotBrain Docker 框架

---

## 目录

1. [迁移概述](#1-迁移概述)
2. [Docker 镜像构建](#2-docker-镜像构建)
   - 2.1 [Dockerfile.manipulation](#21-dockerfilemanipulation)
   - 2.2 [cyclonedds Python 绑定安装 (aarch64)](#22-cyclonedds-python-绑定安装-aarch64)
   - 2.3 [unitree_sdk2py 安装](#23-unitree_sdk2py-安装)
3. [DDS Domain 冲突与解决](#3-dds-domain-冲突与解决)
   - 3.1 [问题本质](#31-问题本质)
   - 3.2 [失败尝试](#32-失败尝试)
   - 3.3 [最终方案: Monkey-patch](#33-最终方案-monkey-patch)
   - 3.4 [Monkey-patch 本地绑定陷阱](#34-monkey-patch-本地绑定陷阱)
4. [URDF 与 IK Solver 路径修复](#4-urdf-与-ik-solver-路径修复)
5. [网络接口配置](#5-网络接口配置)
6. [手臂控制权交还机制](#6-手臂控制权交还机制)
   - 6.1 [问题: 直接失去力矩](#61-问题-直接失去力矩)
   - 6.2 [解决: Weight 渐变释放](#62-解决-weight-渐变释放)
   - 6.3 [release_on_disable 参数](#63-release_on_disable-参数)
7. [交互式控制方案](#7-交互式控制方案)
8. [文件清单与修改摘要](#8-文件清单与修改摘要)

---

## 1. 迁移概述

### 背景

g1pilot 是一个独立的 ROS 2 包，直接运行在主机上，通过 `unitree_sdk2py` (CycloneDDS) 控制 G1 手臂和灵巧手。BotBrain 是一个 Docker 化的机器人框架，所有服务以容器方式运行，使用 `rmw_cyclonedds_cpp` 作为 ROS 2 中间件。

### 核心挑战

| 挑战 | 难度 | 描述 |
|------|------|------|
| cyclonedds Python 绑定 aarch64 安装 | ★★★★ | 无预编译 wheel，multiarch 路径不匹配 |
| DDS Domain 冲突 | ★★★★★ | rmw_cyclonedds_cpp 与 unitree_sdk2py 共享 libddsc.so，双重 `dds_create_domain` |
| 控制权交还 | ★★★ | 需对齐 g1_driver.cpp 的 weight 渐变机制 |
| 包路径迁移 | ★★ | g1pilot → g1_manipulation_pkg 的 URDF/mesh 引用修改 |

### 迁移结果

- ✅ Docker 镜像构建成功 (`botbotrobotics/botbrain:manipulation`)
- ✅ 三个节点全部启动无报错 (arm_controller / dx3_controller / interactive_marker)
- ✅ ROS 话题正常注册 (13 个 manipulation 相关话题)
- ✅ 手臂 Home 归位 + IK 追踪正常
- ✅ 手臂控制权平滑释放 (weight 渐变 2 秒)
- ✅ Mode-B 联锁 (manipulation_vel 压制导航)

---

## 2. Docker 镜像构建

### 2.1 Dockerfile.manipulation

文件: `docker/Dockerfile.manipulation`

基于 `botbotrobotics/botbrain:base` 构建，新增以下依赖:

```dockerfile
FROM botbotrobotics/botbrain:base

# 1. pinocchio (IK solver)
RUN pip3 install --no-cache-dir pin

# 2. unitree_sdk2_python (HongTu 版)
RUN git clone https://github.com/yuanqizhiti/HongTu.git /tmp/HongTu && \
    mv /tmp/HongTu/unitree_sdk2_python /unitree_sdk2_python && \
    rm -rf /unitree_sdk2_python/cyclonedds && \
    rm -rf /tmp/HongTu

# 3. cyclonedds Python 绑定 (详见 2.2)
# 4. unitree_sdk2py install (详见 2.3)
# 5. interactive_markers + tf2_geometry_msgs
```

### 2.2 cyclonedds Python 绑定安装 (aarch64)

**问题**: `pip install cyclonedds==0.10.2` 在 aarch64 上无预编译 wheel，需从源码构建。`setup.py` 中的 `good_directory()` 函数检查:

```python
def good_directory(path):
    # 要求 $path/include/ 存在
    # 要求 $path/bin/ 存在
    # 要求 $path/lib/libddsc.so 存在
```

但 ROS 2 Humble aarch64 将文件放在 multiarch 子目录:
- `lib/aarch64-linux-gnu/libddsc.so` (不是 `lib/libddsc.so`)
- `include/cyclonedds/dds/` (不是 `include/dds/`)

**解决**: 创建 `/cyclonedds_home/` 符号链接目录:

```dockerfile
RUN set -ex && \
    mkdir -p /cyclonedds_home/lib /cyclonedds_home/bin /cyclonedds_home/include && \
    DDSC=$(find /opt/ros/humble -name "libddsc.so" -print -quit) && \
    ln -sf "$(dirname $DDSC)"/* /cyclonedds_home/lib/ && \
    INC=$(find /opt/ros/humble/include -maxdepth 2 -name "dds" -type d -print -quit) && \
    ln -sf "$(dirname $INC)"/* /cyclonedds_home/include/ && \
    ...

ENV CYCLONEDDS_HOME=/cyclonedds_home
ENV LD_LIBRARY_PATH="/cyclonedds_home/lib:${LD_LIBRARY_PATH}"

RUN pip3 install --no-cache-dir cyclonedds==0.10.2
```

**额外修复**: 运行时 `libiceoryx_binding_c.so` 找不到 → 添加 `LD_LIBRARY_PATH` 环境变量。

### 2.3 unitree_sdk2py 安装

HongTu 仓库的 `cyclonedds` 子目录是一个**空的 git submodule**（mode 160000，无 .gitmodules），直接 clone 后该目录存在但为空，导致 `pip install -e .` 尝试安装一个不存在的子包。

**解决**: 在 clone 后立即 `rm -rf cyclonedds`，让 setuptools 只发现 `unitree_sdk2py` 一个顶层包。

---

## 3. DDS Domain 冲突与解决

### 3.1 问题本质

在同一个进程中:

```
rmw_cyclonedds_cpp → rmw_create_node() → dds_create_domain(0, CYCLONEDDS_URI_config)
unitree_sdk2py     → ChannelFactoryInitialize() → Domain(0, xml) → dds_create_domain(0, xml)
```

**CycloneDDS C 库不允许同一进程对同一 domain-id 调用两次 `dds_create_domain`**，导致:

```
[ERROR] rmw_create_node: failed to create domain, error Precondition Not Met
```

关键发现: `rmw_cyclonedds_cpp` 和 pip `cyclonedds` **共享同一个 `libddsc.so`**（通过符号链接指向同一文件），所以它们的 domain 注册表是同一个全局状态。

### 3.2 失败尝试

| 尝试 | 方法 | 结果 |
|------|------|------|
| #1 | 在 rclpy.init() 前调 ChannelFactoryInitialize | Precondition Not Met (rmw 再次 create domain) |
| #2 | 生成匹配的 DDS XML，统一 CYCLONEDDS_URI | docker-compose heredoc 语法错误 |
| #3 | 外部 bash 脚本 start_manipulation.sh | DDS interface 正确但仍冲突 |
| #4 | 深入分析 rmw 源码 | 发现 rmw **总会** dds_create_domain |

### 3.3 最终方案: Monkey-patch

**原理**: 用空壳 `_NoOpDomain` 替换 `ChannelFactory.Init()` 中使用的 `Domain` 类，使 `ChannelFactoryInitialize` 标记 `__initialized=True` 但不调用 `dds_create_domain`。Domain 0 统一由 rmw 通过 `CYCLONEDDS_URI` 创建。

**初始化顺序**:

```
main() → monkey-patch Domain
       → rclpy.init()
       → Node.__init__()
           → super().__init__()
               → rmw dds_create_domain(0, config) ✅ (首次创建)
           → _init_robot_interface()
               → ChannelFactoryInitialize(0, iface)
                   → NoOp Domain ✅ (跳过)
                   → DomainParticipant(0) 加入已有 domain ✅
```

### 3.4 Monkey-patch 本地绑定陷阱

**第一次 monkey-patch 失败**: 修改了 `cyclonedds.domain.Domain`:

```python
import cyclonedds.domain as _cdd_mod
_cdd_mod.Domain = _NoOpDomain  # ← 无效！
```

**原因**: `unitree_sdk2py/core/channel.py` 文件顶部使用的是:

```python
from cyclonedds.domain import Domain, DomainParticipant
```

这在 `channel.py` 模块的**本地命名空间**中创建了独立的 `Domain` 绑定。之后修改 `cyclonedds.domain.Domain` 不影响 `channel.py` 已有的引用。

**正确做法**: 必须 patch `channel.py` 模块内的 `Domain` 引用:

```python
import unitree_sdk2py.core.channel as _ch_mod
_ch_mod.Domain = _NoOpDomain  # ← 正确！直接替换 channel.py 中的绑定
```

> **教训**: Python monkey-patch 时，必须 patch **使用方模块**中的引用，而非定义方模块。如果目标模块用了 `from X import Y`，则 `X.Y = new` 对目标模块无效。

---

## 4. URDF 与 IK Solver 路径修复

### 4.1 URDF mesh 路径

g1pilot 的 URDF 中 mesh 引用 `package://g1pilot/description_files/meshes/xxx.STL`，BotBrain 中无 `g1pilot` 包。

**修复**: sed 全局替换 (59 处):

```bash
sed -i 's|package://g1pilot/|package://g1_manipulation_pkg/|g' 29dof.urdf
```

### 4.2 IK Solver package_dirs

`ik_solver.py` 中 Pinocchio 的 `package_dirs` 指向了错误的目录层级。

**修复**: 从 `pkg_share` 改为 `os.path.dirname(pkg_share)`:

```python
# 修复前
mesh_dir = pkg_share  # → .../share/g1_manipulation_pkg/
# 修复后
mesh_dir = os.path.dirname(pkg_share)  # → .../share/
```

Pinocchio 解析 `package://g1_manipulation_pkg/description_files/meshes/xxx.STL` 时，会在 `mesh_dir` 下查找 `g1_manipulation_pkg/description_files/meshes/xxx.STL`。

---

## 5. 网络接口配置

### 问题

机器人使用 `eth1` 而非默认的 `eth0`，导致:

```
eth0: does not match an available interface
```

### 解决

1. `robot_config.yaml` 中 `network_interface: eth1`
2. `start_manipulation.sh` 从 `robot_config.yaml` 读取接口名，生成 DDS XML 配置并 `export CYCLONEDDS_URI`
3. 两个 Python 节点通过 `_read_botbrain_config()` / `_read_interface()` 从同一配置文件读取接口名

---

## 6. 手臂控制权交还机制

### 6.1 问题: 直接失去力矩

初版 `_release_arm_control()` 设置 `kp=0, kd=0` + `mode_pr=0`，手臂瞬间失去力矩导致坠落。

### 6.2 解决: Weight 渐变释放

对齐 BotBrain C++ `g1_driver.cpp` 的 `release_arm_joints()` 机制:

- 宇树 G1 的 `kNotUsedJoint0` (关节 index 29) 的 `.q` 值用于控制 **arm_sdk 混合权重** (0.0~1.0)
- Weight=1.0: arm_sdk 完全控制手臂
- Weight=0.0: 运控完全控制手臂
- 中间值: 按比例混合

**实现 (arm_controller.py `_release_arm_control`)**:

```
Phase 1 (2 秒, 50Hz, 100 步):
  mode_pr = 1
  weight: 1.0 → 0.0 (线性递减)
  手臂关节: 保持当前位置 + 正常增益 (有力矩)
  → 运控逐渐接管

Phase 2:
  mode_pr = 0 (连发 5 帧)
  → 彻底释放 arm_sdk 通道
```

### 6.3 release_on_disable 参数

新增 ROS 参数 `release_on_disable` (默认 `true`):

| 值 | 禁用时行为 |
|---|---|
| `true` (默认) | Weight 渐变 2 秒 → 运控接管 → 手臂回默认站姿 |
| `false` | 保持当前位置，arm_sdk 继续以低增益维持力矩 |

**运行时动态切换**:

```bash
ros2 param set /g1_robot/arm_controller release_on_disable false
ros2 param set /g1_robot/arm_controller release_on_disable true
```

---

## 7. 交互式控制方案

由于 Jetson 无桌面环境，无法使用 RViz interactive markers。提供两个替代方案:

### 方案 1: 键盘增量遥控 (SSH 终端)

新增 `arm_teleop_keyboard` 工具:

```bash
docker compose exec manipulation bash
source /botbrain_ws/install/setup.bash
ros2 run g1_manipulation_pkg arm_teleop_keyboard --ros-args -r __ns:=/g1_robot
```

| 按键 | 功能 |
|------|------|
| `w`/`s` | 前/后 (x) |
| `a`/`d` | 左/右 (y) |
| `q`/`e` | 上/下 (z) |
| `1`/`2` | 切换左臂/右臂 |
| `[` / `]` | 启用/禁用手臂 |
| `h` | 归位 (Home) |
| `+`/`-` | 增大/减小步长 (默认 2cm) |

### 方案 2: Foxglove Studio (网页 3D)

```bash
docker compose up -d foxglove
# 浏览器打开 https://app.foxglove.dev → 连接 ws://<机器人IP>:8765
```

---

## 8. 文件清单与修改摘要

### 新增文件

| 文件 | 用途 |
|------|------|
| `docker/Dockerfile.manipulation` | manipulation 镜像构建 |
| `botbrain_ws/src/g1_manipulation_pkg/` (整个包) | 从 g1pilot 迁移的手臂控制 ROS 2 包 |
| `g1_manipulation_pkg/scripts/start_manipulation.sh` | DDS XML 生成 + 启动脚本 |
| `g1_manipulation_pkg/scripts/arm_teleop_keyboard.py` | 键盘遥控工具 |

### 修改文件

| 文件 | 修改内容 |
|------|----------|
| `docker-compose.yaml` | 新增 manipulation service 定义 |
| `arm_controller.py` | monkey-patch DDS Domain、weight 渐变释放、release_on_disable 参数 |
| `dx3_hand.py` | monkey-patch DDS Domain |
| `ik_solver.py` | 修复 `package_dirs` 路径 |
| `29dof.urdf` | `package://g1pilot/` → `package://g1_manipulation_pkg/` (59 处) |
| `manipulation_config.yaml` | 新增 `release_on_disable` 参数 |
| `setup.py` | 新增 `arm_teleop_keyboard` entry_point |

### 关键代码段: Monkey-patch (arm_controller.py / dx3_hand.py)

```python
def main(args=None):
    # 必须 patch channel.py 模块本地的 Domain 绑定
    import unitree_sdk2py.core.channel as _ch_mod

    class _NoOpDomain:
        def __init__(self, *a, **kw): pass
        def close(self): pass
        def __del__(self): pass

    _ch_mod.Domain = _NoOpDomain

    rclpy.init(args=args)
    node = ArmController()  # __init__ 中 super().__init__ 让 rmw 先建 domain
    ...
```

### 关键代码段: Weight 渐变释放 (arm_controller.py)

```python
def _release_arm_control(self):
    # Phase 1: weight 从 1.0 渐降到 0.0（2 秒，50Hz，100 步）
    for i in range(steps + 1):
        weight = 1.0 - float(i) / float(steps)
        self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = weight
        # 手臂关节保持当前位置 + 正常增益
        ...
        self.lowcmd_publisher.Write(self.msg)
        time.sleep(0.02)

    # Phase 2: mode_pr=0 彻底释放
    self.msg.mode_pr = 0
    ...
```

---

