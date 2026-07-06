# G1 FSM / SDK 排查报告

## 1. 问题背景

在同时运行以下服务时：

- `bringup`
- `state_machine`
- `localization`
- `navigation`

发现 `bringup` 服务持续输出读取机器人当前 locomotion 模式失败的日志：

```text
[g1_controller_commands.py-6] Timeout in current_mode()
[g1_write_node-4] [G1Write] get_current_mode_callback(): failed to get FSM ID
```

最初的怀疑方向是：`botbotrobotics/botbrain:base` 镜像内的 Unitree SDK 版本与机器人主机版本不匹配。

## 2. 初始代码链路分析

报错链路只发生在 `bringup` 相关节点中：

1. `controller_commands_node` 调用 ROS2 服务 `current_mode`
2. `robot_write_node` 接收到服务请求
3. `robot_write_node` 调用 `G1Driver::get_fsm_id(...)`
4. `G1Driver` 再调用 `unitree::robot::g1::LocoClient::GetFsmId(...)`

相关文件：

- `botbrain_ws/src/g1_pkg/scripts/g1_controller_commands.py`
- `botbrain_ws/src/g1_pkg/src/g1_write.cpp`
- `botbrain_ws/src/g1_pkg/src/g1_driver/g1_driver.cpp`
- `botbrain_ws/src/g1_pkg/include/g1_driver/g1_driver.hpp`

重要结论：

- `g1_read.py` 并不走这条链路
- `g1_read.py` 订阅的是 Unitree ROS 话题，例如 `/lf/lowstate`、`/lf/odommodestate`
- 因此“遥测正常”并不能说明 `GetFsmId()` 正常

## 3. 已完成的验证

### 3.1 容器内 SDK 是否存在

在运行中的容器里执行：

```bash
ldconfig -p | grep -E 'unitree_sdk2|ddsc|ddscxx'
ls -l /usr/local/lib/libunitree_sdk2*
```

结果：

- 存在 `libddscxx.so.0`
- 存在 `libddsc.so.0`
- 存在 `libunitree_sdk2.a`

结论：

- 镜像中确实带有 Unitree C++ SDK 相关文件
- `unitree_sdk2` 以静态库形式存在，而不是动态库

### 3.2 ROS 接口类型验证

在容器中执行：

```bash
source install/setup.bash
ros2 interface show bot_custom_interfaces/srv/CurrentMode
```

结果：

- `CurrentMode` 服务接口存在且有效

结论：

- 问题不是因为 ROS2 服务类型定义缺失

### 3.3 Python SDK 替换方向验证

之前g1pilot成功的替换流程，使用的是：

- `unitree_sdk2_python`
- `pip install -e .`

后续验证发现该 HongTu SDK 只包含 Python 侧内容，例如：

- `unitree_sdk2py/g1/loco/g1_loco_client.py`
- `crc_amd64.so`
- `crc_aarch64.so`

结论：

- 该 Python SDK 不能直接替换当前工程使用的 C++ SDK
- 当前报错链路的核心是 C++，不是 Python

## 4. 为支持替换 SDK 所做的工程改造

为了在继续使用基础镜像的前提下切换 SDK，对工程做了以下增强：

- 编译期支持通过 `UNITREE_SDK2_ROOT` 指定自定义 SDK 根目录
- 运行期优先从指定 SDK 根目录查找 `ddsc/ddscxx`

这样可以实现：

1. 将机器人主机 SDK 挂载到容器，例如 `/opt/robot_sdk`
2. 编译 `g1_pkg` 时优先使用该挂载 SDK
3. 运行 `g1_write_node` 时优先使用同一套 DDS 动态库

## 5. 静态链接与动态库的重要发现

这是本次排查中的关键点：

- `libunitree_sdk2.a` 是静态库
- 仅仅挂载新的 `.a` 文件不会自动生效
- 必须重新编译 `g1_write_node`，新的静态库才会被真正打入可执行文件
- `ddsc` 和 `ddscxx` 是动态库，运行时路径也必须正确

通过以下内容确认了这一点：

- `build/g1_pkg/CMakeFiles/g1_write_node.dir/link.txt`
- `build/g1_pkg/CMakeCache.txt`
- `ldd install/g1_pkg/lib/g1_pkg/g1_write_node`

在某个阶段的结果是：

- `UNITREE_SDK2` 和 `DDSCXX` 已经来自 `/opt/robot_sdk`
- `DDSC` 仍然落到了 ROS Humble 的路径

后续通过调整 CMake 查找优先级，使其优先使用挂载的 SDK。

## 6. 机器人主机 SDK 验证

使用机器人主机自身 `/usr/local` 下的 SDK 文件结构，包含：

- `/usr/local/include/unitree`
- `/usr/local/lib/libunitree_sdk2.a`
- `/usr/local/lib/libddsc.so`
- `/usr/local/lib/libddscxx.so`

这证明：直接使用机器人主机 SDK 作为容器内 SDK 来源是可行的。

## 7. 并发与请求模式分析

在 SDK 路径问题基本排除后，排查重点转向运行时行为。

发现：

- `controller_commands_node` 每 `0.2s` 轮询一次 `current_mode`
- 客户端超时仅 `0.5s`
- `robot_write_node` 使用 `MultiThreadedExecutor`
- `G1Driver` 对同一个 `LocoClient` 没有互斥保护
- 因此服务端和客户端都可能出现重叠调用

这会导致：

- 请求堆积
- 多次重叠调用 `GetFsmId()`
- 日志中出现大量超时与失败

## 8. 加入保护后的关键诊断结果

在加入防重入和额外日志后，机器人端出现如下日志：

```text
get_current_mode_callback(): failed to get FSM ID
(rc=3104, get_fsm_mode rc=0 value=0, get_balance_mode rc=7301 value=-1)
```

这是整个会话中最重要的结果。

### 8.1 错误码含义

由 Unitree 头文件可以直接确认（机器人端位置/usr/local/include/unitree/robot）：

- `3104` 在 `unitree/internal/internal_error.hpp` 中定义为：
  - `UT_ROBOT_ERR_CLIENT_API_TIMEOUT`
  - 含义：API 调用超时

- `7301` 在 `unitree/robot/g1/loco/g1_loco_error.hpp` 中定义为：
  - `UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE`
  - 含义：LocoState 不可用

### 8.2 关键解释

`GetFsmId`、`GetFsmMode`、`GetBalanceMode` 虽然都属于同一个 `sport` 服务，但它们在这套 SDK 中是三个独立 API：

- `GetFsmId()` -> API ID `7001`
- `GetFsmMode()` -> API ID `7002`
- `GetBalanceMode()` -> API ID `7003`

因此，从 SDK 设计上说，完全有可能出现：

- `GetFsmMode()` 成功
- `GetBalanceMode()` 返回 `7301`
- `GetFsmId()` 超时返回 `3104`

这意味着：

- client 到 robot 的 RPC 通道是通的
- `sport` 服务本身是可达的
- 至少部分 loco API 是工作的
- 但 `GetFsmId()` 并没有在当前机器人固件 / 当前运行状态下正常返回

## 9. 影响评估

### 9.1 对导航和定位的影响

当前结论：

- 这个问题 **不会直接阻塞** `localization` 和 `navigation`
- 它主要影响 `controller_commands_node` 内的手柄模式判断与模式切换逻辑

所以：

- `map_server`
- `amcl`
- `nav2`
- TF
- 定位链路

都可以继续独立调试

但是：

- 如果机器人要先进入正确 locomotion 模式后才接受移动指令
- 那么该问题可能会 **间接影响** 机器人是否真正执行移动
- 这属于底盘模式管理问题，不是 Nav2 本身问题

### 9.2 对点云队列满日志的判断

日志中还出现了：

```text
pointcloud_to_laserscan_node: queue is full
```

结论：

- 这是独立的性能 / TF / 吞吐问题
- 它会增加系统负载
- 但它并不能解释 `3104` / `7301` 这一组 loco API 错误结果

## 10. 本次会话中实际修改过的文件

### 10.1 SDK 选择与链接支持

- `botbrain_ws/src/g1_pkg/CMakeLists.txt`
- `botbrain_ws/src/g1_pkg/launch/robot_interface.launch.py`

修改内容：

- 增加 `UNITREE_SDK2_ROOT` 支持
- 增加挂载 SDK 的头文件和库文件优先查找逻辑
- 让运行时优先从指定 SDK 路径加载 `ddsc` / `ddscxx`
- 增加 RPATH，优先使用挂载 SDK 目录

### 10.2 LocoClient 并发保护

- `botbrain_ws/src/g1_pkg/include/g1_driver/g1_driver.hpp`
- `botbrain_ws/src/g1_pkg/src/g1_driver/g1_driver.cpp`

修改内容：

- 给 `LocoClient` 访问增加互斥锁
- 将以下调用串行化：
  - `GetFsmId`
  - `GetFsmMode`
  - `GetBalanceMode`
  - `SetFsmId`
  - `Move`
  - `StopMove`
  - `SetSpeedMode`
  - `ContinuousGait`
  - `Start`

### 10.3 服务端增强日志

- `botbrain_ws/src/g1_pkg/src/g1_write.cpp`

修改内容：

- 输出 `GetFsmId()` 的返回码
- 在 `GetFsmId()` 失败时，额外打印：
  - `GetFsmMode()` 返回码和值
  - `GetBalanceMode()` 返回码和值

### 10.4 客户端请求去重

- `botbrain_ws/src/g1_pkg/scripts/g1_controller_commands.py`

修改内容：

- 禁止对 `current_mode` 发出重叠请求
- 增加异步请求 in-flight 管理
- 增加 pending 请求提示日志
- 降低高频轮询对服务端的压力

## 11. 最终技术结论

截至本次会话结束，最可能的情况是：

1. 工程已经正确使用了挂载的机器人主机 SDK
2. 到机器人 `sport` 服务的 RPC 链路是通的
3. `GetFsmMode()` 可用
4. `GetBalanceMode()` 返回 `LocoState not available`
5. `GetFsmId()` 返回超时
6. 因此当前问题已经不适合继续用“本地构建/链接错误”来解释
7. 剩余问题更像是机器人端服务行为差异、固件版本差异，或者在当前运行状态下 `GetFsmId()` / `GetBalanceMode()` 不被完整支持

## 12. 当前阶段的实际决策

在当前项目阶段，可以将 FSM 读取问题暂时降级处理：

- 先继续推进定位和导航相关工作
- 后续如果出现机器人不动、手柄切模式失效，再回头重点处理该问题

## 13. 后续建议

如果后续必须解决该问题，最实用的方向是：

1. 对比一个在同一台机器人上正常工作的 C++ G1 工程
2. 确认对方是否真的使用了 `GetFsmId()`
3. 如果没有，那么说明当前工程不应该继续依赖 `GetFsmId()` 作为唯一模式来源
4. 可考虑将 `current_mode` 改为以下策略：
   - 优先 `GetFsmId()`
   - 失败时使用 `GetFsmMode()` 兜底
   - 再结合最近一次成功下发的模式命令做缓存判断

