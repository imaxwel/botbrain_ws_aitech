源工程`sudo ./install.sh` 构建完成
配置了命名空间

#### 一 状态机问题

**问题**:启动bringup和state_machine服务会出现管理节点时，每个节点都报错（log1.txt）

> ```bash
> g1_robot_state_machine  | [state_machine_node-1] [ERROR] [1774419637.319956155] [g1_robot.state_machine]: [LifecycleManager] bring_down: missing state for behavior_server (get_state f
> ailed?)                 
> ```

**解决**：状态机尝试管理的节点名称可能与实际节点的命名空间不匹配！

> 节点的完整名称是 g1_robot.robot_read_node（带命名空间），但状态机在查找时使用的是 robot_read_node

```cpp
// 在lifecycle_manager.cpp 的 create_comms() 函数中
// 添加命名空间
// 原：
auto change_cli = this->create_client<lifecycle_msgs::srv::ChangeState>
    ("/" + n.name + "/change_state",
// 改：
auto sub = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
          prefix + "/" + n.name + "/transition_event",
 
```

完整：`void LifecycleManager::create_comms()`
```cpp
void LifecycleManager::create_comms()
{
    // Fresh rebuild of per-node subscriptions/clients.
    transition_subs_.clear();
    change_state_srvs_.clear();
    get_state_srvs_.clear();

    // Get the namespace from the node
    std::string ns = this->get_namespace();
    std::string prefix = (ns == "/" || ns.empty()) ? "" : ns;

    for (const auto &n : nodes_) 
    {
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cbg_;
    
        // Watch lifecycle transition events for this node.
        auto sub = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
          prefix + "/" + n.name + "/transition_event",
          rclcpp::SystemDefaultsQoS(),
          [this, node_name = n.name](const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
            this->transition_callback(node_name, msg);
          },
          sub_opts
        );
        transition_subs_.emplace(n.name, std::move(sub));

        // Service clients used to push lifecycle commands and query status.
        auto change_cli = this->create_client<lifecycle_msgs::srv::ChangeState>
            (prefix + "/" + n.name + "/change_state",
            rmw_qos_profile_services_default, 
            cbg_);
        change_state_srvs_.emplace(n.name, change_cli);

        auto get_cli = this->create_client<lifecycle_msgs::srv::GetState>
            (prefix + "/" + n.name + "/get_state",
            rmw_qos_profile_services_default, 
            cbg_);
        get_state_srvs_.emplace(n.name, get_cli);
    }

    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        for (const auto &n : nodes_)
            state_cache_[n.name] = "unknown";
    }

    print_info("Watching " + std::to_string(transition_subs_.size()) + " transition_event topics.");
}
```

#### 二 建图问题

**问题**：topic 重映射规则中出现了双斜杠 // （log2.txt）

> ```bash 
> g1_robot_localization  | [rtabmap-6]   what():  failed to initialize rcl: Couldn't parse remap rule: '-r odom:=g1_robot//odom'. Error: error not set, at ./src/rcl/arguments.c:371
> g1_robot_localization  | [ERROR] [lidar_deskewing-7]: process has died [pid 141, exit code -6, cmd '/opt/rtab_ws/install/rtabmap_util/lib/rtabmap_util/lidar_deskewing --ros-args --params-file /tmp/launch_params_i36g0ggh -r input_cloud:=g1_robot//pointcloud -r output_cloud:=g1_robot//pointcloud/deskewed'].
> ```

**解决**：通常发生在字符串拼接时，当 robot_name 已经包含尾部斜杠，或者拼接逻辑有问题。

```py
# rtabmap_lidar.launch.py
# 原：
robot_name = config['robot_name']
prefix = robot_name + '/' if robot_name != '' else ''
# ...
odom_topic = f'{prefix}/odom'  # 这里会变成 'g1_robot//odom'
lidar_topic = f'{prefix}/pointcloud'  # 这里会变成 'g1_robot//pointcloud'
# 改：
odom_topic = f'{prefix}odom'
lidar_topic = f'{prefix}pointcloud'
```

#### 三 建图话题未接收到

**问题**：雷达正常启动，但ros2 topic echo /g1_robot/pointcloud 没有数据输出,rtabmap 正在等待数据(log3.txt)

> ```bash
> [rtabmap-6] [WARN]: rtabmap: Did not receive data since 5 seconds!
> rtabmap subscribed to (approx sync):
>    /g1_robot/odom \
>    /g1_robot/pointcloud/deskewed
> 
> ```

**解决**：Livox 雷达驱动发布的 topic 名称与 rtabmap 订阅的 topic 名称不匹配！`remappings=[('livox/lidar', 'pointcloud')]` 没有添加命名空间
修改 robot_interface.launch.py 和livox_MID360.launch.py来传递正确的 namespace

```py
# robot_interface.launch.py原：
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'livox_MID360.launch.py')),
        launch_arguments={'prefix': prefix}.items()
    )
# 改：
livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'livox_MID360.launch.py')),
        launch_arguments={
            'prefix': prefix,
            'namespace': robot_name
        }.items()
    )
```

```python
# livox_MID360.launch.py
# 原：
# Extract namespace from prefix (remove trailing slash)
    prefix_config = LaunchConfiguration('prefix')
# 改：
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='', description='Robot namespace')
# 在livox_driver = Node 添加：
namespace=LaunchConfiguration('namespace'),
# 在return 添加：
namespace_arg,
```

需要先启动了状态机再开启建图

#### 四 FSM查询超时（未解决）

log4.txt

> ```bash
> g1_robot_bringup  | [g1_controller_commands.py-6] [WARN] [1774426476.913984997] [g1_robot.controller_commands_node]: Timeout in current_mode()
> g1_robot_bringup  | [g1_write_node-4] [WARN] [1774426478.412041429] [g1_robot.robot_write_node]: [G1Write] get_current_mode_callback(): failed to get FSM ID
> 
> ```

` ros2 topic echo /lf/sportmodestate` 
输出：
`Cannot echo topic '/lf/sportmodestate', as it contains more than one type: 
[unitree_go/msg/SportModeState, unitree_hg/msg/SportModeState]`
同一个 topic 有两个不同的消息类型在发布

在`g1_read.py` 中
```python
from unitree_hg.msg import BmsState, LowState
from unitree_go.msg import SportModeState  # ❌ 错误！应该用 unitree_hg

```

`ros2 topic info /lf/sportmodestate -v` 查看发布者和订阅者是谁和消息类型

`ros2 topic list | grep /lf/` 查找相关话题



#### 原定位导航

需要让机器人提前进入平衡状态，定位导航代码没有对机器人的FSM进行设置和验证，直接发送速度指令

系统采用先建图后定位导航的模式，localization服务有建图和定位两种模式，通过调用服务参数进行更换。（默认是定位模式）

```bash
# ========== 第一次：建图 ==========
# 1. 启动系统（默认定位模式）
ros2 launch bot_localization localization.launch.py

# 2. 切换到建图模式
ros2 service call /g1_robot/rtab_manager/set_mapping \
  bot_localization_interfaces/srv/SetMapping \
  "{database_path: 'office_map.db', clear_db: true}"

# 3. 遥控机器人建图
# （手动控制机器人移动）

# 4. 保存地图
ros2 service call /g1_robot/rtab_manager/save_database \
  bot_localization_interfaces/srv/SaveDatabase
# 或者直接 Ctrl+C（也会自动保存）

# ========== 第二次：导航 ==========
# 1. 配置默认地图（可选）
# 修改 robot_config.yaml:
#   default_map: "office_map.db"

# 2. 启动定位服务（自动加载地图）
ros2 launch bot_localization localization.launch.py

# 3. 启动导航服务
ros2 launch bot_navigation navigation.launch.py

# 4. 发送导航目标
# （通过RViz或代码发送导航目标）

# 地图存放
# botbrain_ws/src/{robot_model}_pkg/maps
```

**地图**

```bash
# 1. 查看所有可用地图
ros2 service call /g1_robot/rtab_manager/list_db_files \
  bot_localization_interfaces/srv/ListDbFiles

# 输出示例：
# success: true
# db_files: ['office_map.db', 'warehouse_map.db', 'factory_map.db']
# message: "Found 3 database files"

# 2. 查看当前加载的地图
ros2 service call /g1_robot/rtab_manager/get_current_database \
  std_srvs/srv/Trigger

# 输出示例：
# success: true
# message: "office_map.db"

# 3. 切换到不同的地图
ros2 service call /g1_robot/rtab_manager/load_database \
  bot_localization_interfaces/srv/LoadDB \
  "{database_path: 'warehouse_map.db', clear_db: false}"

# 4. 设置默认地图（下次启动时自动加载）
ros2 service call /g1_robot/rtab_manager/set_default_map \
  bot_localization_interfaces/srv/SetDefaultMap \
  "{map_name: 'warehouse_map.db'}"

```

> clear_db: false - 保留内存中的工作记忆（推荐）
> 优点：切换更快，保留一些临时信息
> 适用：在已知地图之间切换
>
> clear_db: true - 完全清空内存后加载
> 优点：干净的状态，避免旧数据干扰
> 适用：地图差异很大，或者出现定位问题时

**原工程对于G1**

> 使用的功能：
> ✅ ICP点云配准（Iterative Closest Point）
> ✅ 图优化（Graph Optimization）
> ✅ 闭环检测（基于几何相似性，不是视觉）
> ✅ 3D点云地图
>
> 不使用的功能：
> ❌ 视觉特征匹配（ORB/SURF）
> ❌ RGB图像
> ❌ 词袋模型（Bag of Words）
> ❌ 视觉闭环检测

非真正的RTABMAP，属于纯雷达模式
```py
rtabmap_parameters = {
    'subscribe_depth': False,      # ❌ 不使用深度图
    'subscribe_rgb': False,        # ❌ 不使用RGB图像
    'subscribe_scan_cloud': True,  # ✅ 只使用激光点云
    'Reg/Strategy': '1',           # ICP配准（不是视觉特征）
    'Icp/PointToPlane': 'true',    # 点到面ICP
}
```

本质：

> 纯几何SLAM（Geometric SLAM）
> ├─ 使用ICP进行帧间配准
> ├─ 使用几何相似性检测闭环
> ├─ 图优化消除累积误差
> └─ 生成3D点云地图 + 2D占据栅格

#### 新定位导航

原工程: LiDAR → RTAB-Map (ICP) → .db → 2D栅格
新方案: LiDAR+相机+IMU → FAST-LIVO2 → .pcd → 2D栅格

> STATIC_MAP_README.md - 快速入门指南
> ✅ 明确说明使用FAST-LIVO2建图
> ✅ 简化了建图流程（在独立工程完成）
> ✅ 强调地图文件复制和配置
> ✅ 添加多地图切换说明
> ✅ 优化故障排查部分
> ✅ 添加FAST-LIVO2 vs 原工程对比
>
> STATIC_MAP_GUIDE.md - 详细使用指南
> ✅ 更新系统架构说明
> ✅ 详细的配置步骤
> ✅ 完整的AMCL参数配置
> ✅ 多地图管理方案
> ✅ 自动地图切换示例
> ✅ FAST-LIVO2地图质量优化建议

文件：
pc2ls.launch.py
添加了prefix到target_frame参数：

```python
parameters=[params_file, {
    'target_frame': f'{prefix}base_link'  # 现在是 "g1_robot/base_link"
}],
```

#### nav2服务没有启动

```bash
ros2 lifecycle get /g1_robot/planner_server
ros2 lifecycle get /g1_robot/behavior_server
ros2 lifecycle get /g1_robot/bt_navigator
ros2 lifecycle get /g1_robot/waypoint_follower
unconfigured [1]                             
unconfigured [1]                             
unconfigured [1]                             
unconfigured [1]                             

```
启动nav2.launch.py的生命周期管理节点 ,并且取消use_sim_time
```python
return LaunchDescription([
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        # velocity_smoother,
        lifecycle_manager,
    ])
```

正常启动现象：
```bash
$ ros2 action list | grep navigate
/g1_robot/navigate_through_poses
/g1_robot/navigate_to_pose

$ ros2 lifecycle get /g1_robot/controller_server
ros2 lifecycle get /g1_robot/planner_server
ros2 lifecycle get /g1_robot/behavior_server
ros2 lifecycle get /g1_robot/bt_navigator
ros2 lifecycle get /g1_robot/waypoint_follower
active [3]                                   
active [3]                                   
active [3]                                   
active [3]                                   
active [3]   
```

#### behavior_server recovery失败
log9.txt(268、306) 
其服务在查找odom，而实际系统配置了命名空间
需要在 nav2_params.yaml (line 223) 补齐namespaced frame 配置

#### controller_server 里 collision checking 警告
```bash
g1_robot_navigation  | [controller_server-3] [WARN] [1774843040.412986085] [g1_robot.controller_server]: Inconsistent configuration in collision checking. Please verify the robot's sh
ape settings in both the costmap and the cost critic.                                      
g1_robot_navigation  | [controller_server-3] [INFO] [1774843040.413113032] [g1_robot.controller_server]: InflationCostCritic instantiated with 1 power and 300.000000 / 0.015000 weight
s. Critic will collision check based on circular cost.   
```
controller_server 里 MPPI 的碰撞检查配置和 costmap 机器人形状不一致
原因是 costmap 用 footprint，多边形碰撞体；但 CostCritic.consider_footprint 原来是 false。这会让局部控制器的碰撞评估偏保守或不一致。
