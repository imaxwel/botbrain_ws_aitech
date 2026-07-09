# ROS2 Bag 录制工具使用说明

## 概述

本工具用于录制 ROS2 系统中所有发布的话题数据（包括雷达、摄像头、深度图像、点云、IMU、关节状态等），并保存为 MCAP 格式文件。

## 文件位置

- **脚本路径**: `src/rosbag_recorder.py`
- **录制输出目录**: `bags/`

## 使用方法

### 方法一：使用 Python 脚本（推荐）

```bash
# 进入项目目录
cd /data/botbrain_ws/botbrain_project-main/botbrain_ws

# 激活 ROS2 环境
source install/setup.bash

# 录制所有话题，录制10秒，保存到 bags 目录
python3 src/rosbag_recorder.py -d 10 -o bags
```

### 方法二：使用 ros2 bag CLI

```bash
# 进入项目目录
cd /data/botbrain_ws/botbrain_project-main/botbrain_ws

# 激活 ROS2 环境
source install/setup.bash

# 录制所有话题，录制10秒，保存到 bags 目录
ros2 bag record --all -d 10 -o bags/recording_$(date +%Y%m%d_%H%M%S) --storage mcap
```

## 参数说明

| 参数 | 简写 | 类型 | 默认值 | 说明 |
|------|------|------|--------|------|
| `--output` | `-o` | string | `./bags` | 输出目录 |
| `--duration` | `-d` | int | None | 录制时长（秒） |
| `--topics` | `-t` | list | None | 要录制的话题列表 |
| `--exclude` | `-x` | list | None | 要排除的话题列表 |
| `--queue-size` | `-q` | int | 100 | 订阅队列大小 |
| `--list-topics` | - | flag | False | 列出所有可用话题 |

## 常用命令示例

### 1. 查看当前活跃话题

```bash
python3 src/rosbag_recorder.py --list-topics
```

### 2. 录制所有话题（无时间限制，按 Ctrl+C 停止）

```bash
python3 src/rosbag_recorder.py -o bags
```

### 3. 录制所有话题（录制30秒）

```bash
python3 src/rosbag_recorder.py -d 30 -o bags
```

### 4. 录制特定话题

```bash
python3 src/rosbag_recorder.py -t /camera/image_raw /points_raw /scan /imu/data -o bags
```

### 5. 录制所有话题但排除 TF 话题

```bash
python3 src/rosbag_recorder.py -x /tf /tf_static -o bags
```

### 6. 录制所有传感器数据（推荐配置）

```bash
python3 src/rosbag_recorder.py -t \
  /camera/image_raw \
  /camera/depth/image_raw \
  /camera/depth/points \
  /scan \
  /points_raw \
  /imu/data \
  /joint_states \
  /tf \
  /tf_static \
  -d 60 \
  -o bags
```

## 录制输出

录制完成后，MCAP 文件会保存在 `bags/` 目录下：

```
bags/
└── recording_20260709_174828/
    ├── recording_20260709_174828_0.mcap
    ├── recording_20260709_174828_1.mcap
    ├── recording_20260709_174828_2.mcap
    └── ...
```

## 回放录制的数据

### 回放所有话题

```bash
cd /data/botbrain_ws/botbrain_project-main/botbrain_ws
source install/setup.bash
ros2 bag play bags/recording_20260709_174828/
```

### 循环回放

```bash
ros2 bag play bags/recording_20260709_174828/ -l
```

### 调整播放速度（0.5倍速）

```bash
ros2 bag play bags/recording_20260709_174828/ -r 0.5
```

### 只播放特定话题

```bash
ros2 bag play bags/recording_20260709_174828/ --topics /camera/image_raw /points_raw
```

## 查看录制信息

### 使用 rosbags 库

```bash
pip install rosbags
rosbags info bags/recording_20260709_174828/
```

### 使用 mcaptui（可视化工具）

```bash
pip install mcaptui
mcaptui bags/recording_20260709_174828/
```

### 查看文件大小

```bash
ls -la bags/recording_20260709_174828/
du -sh bags/recording_20260709_174828/
```

## 注意事项

1. **环境激活**: 录制前必须先激活 ROS2 环境（`source install/setup.bash`）
2. **存储空间**: MCAP 文件可能较大，请确保磁盘空间充足
3. **网络带宽**: 录制大量高频话题（如点云）时，建议使用有线网络
4. **时间同步**: 确保系统时间准确，以便回放时时间戳正确
5. **权限**: 确保对输出目录有写入权限

## 故障排除

### 问题：录制的 MCAP 文件为空

**原因**: `rosbag2_py` 无法加载自定义消息类型

**解决方案**: 使用 `--use-cli` 参数或直接使用 `ros2 bag record` 命令

```bash
python3 src/rosbag_recorder.py --use-cli -d 10 -o bags
```

### 问题：无法找到话题

**原因**: 话题未发布或环境未正确激活

**解决方案**: 
1. 确保 ROS2 环境已激活
2. 确保相关节点正在运行
3. 使用 `ros2 topic list` 确认话题存在

### 问题：录制中断

**原因**: 磁盘空间不足或网络中断

**解决方案**: 
1. 检查磁盘空间：`df -h`
2. 清理旧的录制文件
3. 使用较短的录制时长

## 附录

### 常用话题列表

| 话题类型 | 示例话题名 | 说明 |
|----------|------------|------|
| 摄像头 | `/camera/image_raw` | RGB 图像 |
| 深度 | `/camera/depth/image_raw` | 深度图像 |
| 点云 | `/camera/depth/points` | 点云数据 |
| 雷达 | `/scan` | 激光雷达扫描 |
| IMU | `/imu/data` | 惯性测量单元 |
| 关节状态 | `/joint_states` | 机器人关节状态 |
| TF | `/tf`, `/tf_static` | 坐标变换 |

### MCAP 格式优势

- **现代格式**: 替代传统的 `.bag` 和 `.db3` 格式
- **跨平台**: 支持 ROS1/ROS2，可在不同系统间共享
- **紧凑高效**: 更好的压缩率和读写性能
- **工具支持**: 可使用 `mcaptui` 等工具可视化查看