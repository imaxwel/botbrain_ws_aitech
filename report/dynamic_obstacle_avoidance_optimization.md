# G1机器人动态避障优化方案

## 当前状态分析

### 已有功能
- ✅ Local Costmap 配置了 obstacle_layer（点云数据）
- ✅ Global Costmap 配置了 obstacle_layer
- ✅ MPPI控制器启用了 CostCritic（碰撞检测）
- ✅ 障碍物检测范围：3.0米
- ✅ 更新频率：Local 10Hz, Global 5Hz

### 存在问题
- ⚠️ 缺少降噪层（DenoiseLayer），可能导致误检
- ⚠️ Local Costmap 更新频率较低（10Hz）
- ⚠️ 膨胀半径较小（0.4m），安全裕度不足
- ⚠️ 控制器频率20Hz，对快速避障响应可能不够

## 优化方案

### 1. 添加降噪层（推荐）
在 Local Costmap 中添加降噪层，过滤点云噪点：

```yaml
/**/local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "denoise_layer", "inflation_layer"]
      
      denoise_layer:
        plugin: "nav2_costmap_2d::DenoiseLayer"
        enabled: true
        minimal_group_size: 3        # 至少3个相邻点才认为是障碍物
        group_connectivity_type: 8   # 8连通（包括对角线）
```

### 2. 提高更新频率
```yaml
/**/local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 15.0    # 从10.0提升到15.0
      publish_frequency: 10.0   # 从5.0提升到10.0
```

### 3. 优化膨胀参数
增加安全裕度，避免机器人过于接近障碍物：

```yaml
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.55      # 从0.4增加到0.55
        cost_scaling_factor: 8.0    # 从10.0降低到8.0（更平滑的代价衰减）
```

### 4. 优化MPPI控制器参数
提高避障响应速度和精度：

```yaml
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 60              # 从56增加到60（更长的预测时间）
      batch_size: 2500            # 从2000增加到2500（更多轨迹采样）
      vx_max: 0.30                # 从0.35降低到0.30（更保守的速度）
      
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0          # 从3.81增加到5.0（更重视避障）
        near_collision_cost: 253
        critical_cost: 300.0
        consider_footprint: true
        collision_cost: 1000000.0
        near_goal_distance: 1.0
        trajectory_point_step: 1  # 从2降低到1（更密集的轨迹检查）
```

### 5. 调整障碍物高度范围
根据实际环境调整：

```yaml
      obstacle_layer:
        cloud:
          min_obstacle_height: 0.05   # 从0.07降低，检测更低的障碍物
          max_obstacle_height: 1.50   # 从1.30增加，检测更高的障碍物
```

### 6. 添加Voxel Layer（高级方案）
如果需要更精确的3D障碍物检测：

```yaml
/**/local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]
      
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        footprint_clearing_enabled: true
        max_obstacle_height: 1.5
        z_resolution: 0.05
        z_voxels: 16
        origin_z: 0.0
        mark_threshold: 0
        observation_sources: cloud
        cloud:
          topic: pointcloud
          data_type: "PointCloud2"
          marking: true
          clearing: true
          obstacle_range: 3.0
          raytrace_range: 3.5
          min_obstacle_height: 0.05
          max_obstacle_height: 1.5
```

## 测试方案

### 1. 静态障碍物测试
- 在已知地图中放置新的静态障碍物
- 观察机器人是否能检测并绕行

### 2. 动态障碍物测试
- 在机器人导航路径上移动障碍物（人、物体）
- 观察机器人的避障反应时间和轨迹

### 3. 窄通道测试
- 测试机器人通过狭窄通道的能力
- 验证膨胀半径设置是否合理

### 4. 可视化调试
使用RViz2或Foxglove查看：
- `/g1_robot/local_costmap/costmap` - 局部代价地图
- `/g1_robot/local_costmap/published_footprint` - 机器人足迹
- `/g1_robot/received_global_plan` - 全局路径
- `/g1_robot/local_plan` - 局部路径
- `/pointcloud` - 点云数据

```bash
ros2 run rviz2 rviz2
# 添加以上话题进行可视化
```

## 诊断命令

```bash
# 查看costmap更新频率
ros2 topic hz /g1_robot/local_costmap/costmap

# 查看点云数据
ros2 topic echo /pointcloud --once

# 查看控制器状态
ros2 topic echo /g1_robot/controller_server/transition_event

# 查看避障轨迹
ros2 topic echo /g1_robot/received_global_plan
```

## 实施步骤

1. 备份当前配置文件
2. 逐步应用优化（建议先添加降噪层）
3. 重新编译和启动导航服务
4. 进行测试验证
5. 根据实际效果微调参数
