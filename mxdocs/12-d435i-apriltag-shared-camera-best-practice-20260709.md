# 12 D435i 与 AprilTag 共用相机的最佳实践

日期: 2026-07-09  
适用范围: `g1edu:/data/botbrain_ws/botbrain_project-main`，Unitree G1 EDU，Intel RealSense D435i，ROS 2 Humble，Docker Compose。

## 1. 结论

不要让 AprilTag 节点再直接打开 D435i 的 `/dev/video*` 或 librealsense 设备。

推荐架构是:

```text
D435i USB
  -> 唯一相机 owner: realsense2_camera_node
  -> ROS topics: Image + CameraInfo + Depth + TF
  -> AprilTag / OCR / Web / navigation 等多个消费者订阅 ROS topics
```

也就是说，D435i 硬件只能由一个进程拥有。其他节点全部通过 ROS topic 消费图像。

当前看起来像“独占模式”，这是正常现象，不是异常。RealSense D435i 底层通过 UVC/librealsense 打开彩色、深度、IMU 等接口时，通常不能再被另一个 `cv2.VideoCapture(/dev/videoX)`、`realsense-viewer`、第二个 `realsense2_camera_node` 或自写 V4L2 节点同时打开。同一台相机应该只有一个 driver owner。

## 2. 当前 g1edu 实际相机 topic

当前 D435i 由 `/g1_robot/front_camera` 发布。已确认存在的关键 topic:

| 用途 | Topic | 类型 | 说明 |
|---|---|---|---|
| AprilTag 推荐 RGB 输入 | `/g1_robot/front_camera/color/image_raw` | `sensor_msgs/msg/Image` | 原始彩色图像，当前 640x480 |
| AprilTag 必需内参 | `/g1_robot/front_camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 内参 K/D/P，和 RGB 图像时间戳/坐标系配套 |
| 深度对齐到彩色 | `/g1_robot/front_camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 需要 RGB-D 或按钮三维定位时使用 |
| 对齐深度内参 | `/g1_robot/front_camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | 和 aligned depth 配套 |
| 原始深度 | `/g1_robot/front_camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 当前被 `depthimage_to_laserscan_front` 使用 |
| 浏览器画面 | `/g1_robot/compressed_camera` | `sensor_msgs/msg/CompressedImage` | 给 cockpit/web 看图，不推荐给 AprilTag 算法用 |

当前 CameraInfo 一帧显示:

```text
frame_id: g1_robotfront_camera_color_optical_frame
height: 480
width: 640
K: fx=602.0224609375, fy=601.4728393554688, cx=330.9566955566406, cy=256.2699279785156
D: [0, 0, 0, 0, 0]
```

AprilTag 节点应以 `CameraInfo.header.frame_id` 或图像 header frame 为相机源坐标系，不要继续硬编码旧的 `camera_color_optical_frame`，除非 TF 里确实存在这个 frame。

## 3. 为什么不能让 AprilTag 直接打开 D435i

现有仓库里有两类 AprilTag 实现:

1. `bot_navigation/launch/apriltag_detection.launch.py`  
   使用 `apriltag_ros`，订阅 ROS 图像 topic。这是推荐方向，但当前 remap 少了 `/g1_robot` namespace。

2. `g1_right_dex3/unitree_g1_dex3_stack/scripts/v4l2_apriltag_trigger.py`  
   使用 V4L2/OpenCV 直接打开相机设备。这会和当前 `realsense2_camera_node` 抢同一个 D435i，属于不推荐路径。只适合停掉 RealSense driver 后做单独相机调试。

直接 V4L2 的典型问题:

- `/dev/video*` 编号不稳定，重插 USB 后可能变化。
- 同一 D435i 被 `front_camera` 打开后，V4L2 节点可能打不开设备或卡住。
- 即使偶尔能打开，也可能抢占带宽、改变曝光/格式、影响 Web 画面和定位。
- 多个进程各自采集，会造成相机参数、时间戳、TF、内参来源不一致。

行业最佳实践是“单 producer，多 subscriber”。

## 4. 推荐接入方式 A: apriltag_ros 订阅当前 D435i topic

这是最直接、最标准的方式。

当前 `bot_navigation/launch/apriltag_detection.launch.py` 里写的是:

```python
remappings=[
    ('image_rect', '/front_camera/color/image_raw'),
    ('camera_info', '/front_camera/color/camera_info'),
]
```

在 g1edu 当前命名空间下，应改成:

```python
remappings=[
    ('image_rect', '/g1_robot/front_camera/color/image_raw'),
    ('camera_info', '/g1_robot/front_camera/color/camera_info'),
]
```

临时命令验证可以用:

```bash
cd /data/botbrain_ws/botbrain_project-main

docker compose exec -T dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 run apriltag_ros apriltag_node --ros-args     -r image_rect:=/g1_robot/front_camera/color/image_raw     -r camera_info:=/g1_robot/front_camera/color/camera_info     -p family:=36h11     -p size:=0.05     -p max_hamming:=0
'
```

另一个终端验证:

```bash
docker compose exec -T dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 topic echo --once /apriltag/detections
'
```

注意事项:

- `size` 必须填实际 tag 边长，单位米。
- AprilTag 检测应使用 raw RGB 或 rectified RGB，不要用 `/g1_robot/compressed_camera`。
- 如果检测节点在另一个容器中运行，该容器只需要 ROS/DDS 可见，不需要再打开 `/dev/video*`。

## 5. 推荐接入方式 B: 使用已有 ROS Image 版 detector

`g1_right_dex3/unitree_g1_dex3_stack/scripts/apriltag_detector_node.py` 已经是订阅 ROS topic 的模式，核心参数包括:

```text
rgb_topic
camera_info_topic
tag_pose_topic
target_pose_topic
output_frame
```

建议配置为:

```yaml
apriltag_detector:
  ros__parameters:
    tag_family: "tag36h11"
    tag_size: 0.05
    target_tag_id: 0
    rgb_topic: "/g1_robot/front_camera/color/image_raw"
    camera_info_topic: "/g1_robot/front_camera/color/camera_info"
    tag_pose_topic: "/apriltag/tag_pose"
    target_pose_topic: "/apriltag/target_pose"
    output_frame: "torso_link"
    imshow: false
```

如果 `output_frame:=torso_link`，必须确认 TF 里存在从相机 optical frame 到 `torso_link` 的变换。当前 CameraInfo 的 frame 是:

```text
g1_robotfront_camera_color_optical_frame
```

因此要验证:

```bash
docker compose exec -T dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 run tf2_ros tf2_echo torso_link g1_robotfront_camera_color_optical_frame
'
```

如果 TF 不通，AprilTag 可以先输出在相机 frame 下，或者补齐 `base_link/torso_link -> front_camera_link -> color_optical_frame` 的静态 TF。不要在 detector 里硬编码一个不存在的 frame 名。

## 6. Trigger 模式应该怎么做

如果业务需要“按 G 才拍一张图并检测”，也不要回到 V4L2 独占相机。

推荐做法:

```text
AprilTag ROS node 持续订阅 /g1_robot/front_camera/color/image_raw
  -> 内存里缓存 latest Image + latest CameraInfo
  -> /apriltag/capture_trigger 收到 Empty
  -> 处理最近 N 帧或最近一帧
  -> 发布 /apriltag/tag_pose 和 /apriltag/target_pose
```

也就是把 `v4l2_apriltag_trigger.py` 改造成 `ros_image_apriltag_trigger.py`，保留原来的 AprilTag 检测、offset、TF、debug image 保存逻辑，但把图像来源从 `cv2.VideoCapture` 改成 ROS subscriber。

建议参数:

```yaml
ros_image_apriltag_trigger:
  ros__parameters:
    image_topic: "/g1_robot/front_camera/color/image_raw"
    camera_info_topic: "/g1_robot/front_camera/color/camera_info"
    trigger_topic: "/apriltag/capture_trigger"
    sample_count: 4
    sample_interval_s: 0.05
    tag_family: "tag36h11"
    tag_size: 0.05
    target_tag_id: 0
    tag_pose_topic: "/apriltag/tag_pose"
    target_pose_topic: "/apriltag/target_pose"
```

这样既保留“触发检测”的业务语义，又不抢 D435i。

## 7. Docker Compose 边界建议

当前 Compose 里很多服务继承 `base/nav3d`，并且都有 `/dev:/dev`、`privileged: true`。这对调试方便，但不利于约束硬件 owner。

长期建议:

1. 新增独立 `camera` service 作为 D435i 唯一 owner。  
   它启动 `realsense.launch.py`，并从 `/etc/botbrain/robot.env` 读取 `BOTBRAIN_FRONT_D435I_SERIAL=243722074823`。

2. `localization`、`navigation`、`apriltag`、`manipulation` 只订阅 ROS topic。  
   不再直接启动 `realsense2_camera_node`，也不直接打开 `/dev/video*`。

3. AprilTag service 不需要 USB 访问权限。  
   理想状态下不挂载 `/dev`，不 `privileged`。当前如果为了复用镜像暂时继承了 `/dev`，也要在代码层禁止打开 `/dev/video*`。

4. 同一 ROS 图内保持一致的 DDS 配置。  
   当前保持 `network_mode: host`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、`CYCLONEDDS_URI=file:///botbrain_ws/cyclonedds_config.xml` 即可。`ROS_DOMAIN_ID` 和 `g1_robot` 暂时不改。

推荐服务关系:

```text
camera service
  owns D435i USB
  publishes /g1_robot/front_camera/*

apriltag service
  subscribes /g1_robot/front_camera/color/image_raw
  subscribes /g1_robot/front_camera/color/camera_info
  publishes /apriltag/detections or /apriltag/tag_pose

web/cockpit
  subscribes /g1_robot/compressed_camera

navigation/manipulation
  subscribes AprilTag result topics
```

## 8. QoS 和性能建议

- 同机容器内优先订阅 raw `sensor_msgs/Image`，不要订阅 JPEG 压缩图再解码。
- AprilTag 检测可以降频，不必处理每一帧。当前 `apriltag_detector_node.py` 已经做了约 3 Hz 的处理节流。
- 图像订阅 QoS 使用 `KEEP_LAST(1)`，避免算法处理慢时堆积旧帧。
- 如果 RealSense publisher 是 RELIABLE，订阅端 RELIABLE 或 sensor_data BEST_EFFORT 都可以测试，但生产上建议显式确认 topic info 兼容。
- 多个算法订阅同一 raw topic 是正常的，不会多次打开 USB，只会增加 CPU/GPU 处理负载。
- 如果未来追求低延迟和低拷贝，可以把 camera driver、image_proc、AprilTag detector 做成同一个 composable container 并开启 intra-process 通信。但第一步不需要为此重构。

## 9. 现有代码的具体修改建议

### 9.1 `bot_navigation` 的 apriltag launch

把 topic remap 改为带 namespace:

```python
remappings=[
    ('image_rect', '/g1_robot/front_camera/color/image_raw'),
    ('camera_info', '/g1_robot/front_camera/color/camera_info'),
]
```

更好的写法是改成 launch arguments:

```python
DeclareLaunchArgument('image_topic', default_value='/g1_robot/front_camera/color/image_raw')
DeclareLaunchArgument('camera_info_topic', default_value='/g1_robot/front_camera/color/camera_info')
```

然后 remap 到 `LaunchConfiguration`，方便 g1edu/g1hk 共用同一套代码。

### 9.2 `g1_right_dex3` 的 V4L2 trigger

不建议继续用 `v4l2_apriltag_trigger.py` 作为常规运行路径。

短期策略:

- 保留它用于离线/单独相机调试。
- 在 README 里明确: 运行它之前必须停掉 D435i owner。
- 常规业务改用 `apriltag_detector_node.py` 或新增 `ros_image_apriltag_trigger.py`。

迁移重点:

```text
删除: cv2.VideoCapture(video_device)
新增: create_subscription(Image, image_topic, image_cb, QoS depth 1)
新增: create_subscription(CameraInfo, camera_info_topic, info_cb, QoS depth 1)
保留: pupil_apriltags Detector、pose estimate、offset、TF transform、debug image
```

### 9.3 `apriltag_button_press.launch.py`

它已经有 `input_backend`、`image_topic`、`info_topic`、`depth_topic` 参数。当前默认值仍像旧系统:

```text
/camera/realsense2_camera/color/image_raw
/camera/realsense2_camera/color/camera_info
/camera/realsense2_camera/depth/image_rect_raw
```

在 g1edu 当前系统应改为:

```text
input_backend:=ros
image_topic:=/g1_robot/front_camera/color/image_raw
info_topic:=/g1_robot/front_camera/color/camera_info
depth_topic:=/g1_robot/front_camera/aligned_depth_to_color/image_raw
```

如果 button press 仍调用 V4L2 trigger，就仍然会抢相机，需要改成 ROS image backend。

## 10. 验证命令

确认 D435i owner 正常:

```bash
cd /data/botbrain_ws/botbrain_project-main

docker compose exec -T dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 lifecycle get --no-daemon /g1_robot/front_camera
  ros2 topic info --verbose --no-daemon /g1_robot/front_camera/color/image_raw
  ros2 topic info --verbose --no-daemon /g1_robot/front_camera/color/camera_info
  timeout 10 ros2 topic hz /g1_robot/front_camera/color/image_raw
'
```

期望:

```text
/g1_robot/front_camera active [3]
/g1_robot/front_camera/color/image_raw Publisher count: 1
/g1_robot/front_camera/color/camera_info Publisher count: 1
image_raw hz > 0
```

确认没有 V4L2 抢占者:

```bash
sudo fuser -v /dev/video* 2>/dev/null || true
sudo lsof /dev/video* 2>/dev/null || true
```

如果看到 `v4l2_apriltag_trigger.py`、`realsense-viewer`、另一个 `realsense2_camera_node` 正在打开同一 D435i，应该停止它。

验证 AprilTag 订阅链路:

```bash
docker compose exec -T dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /botbrain_ws/install/setup.bash
  ros2 topic info --verbose --no-daemon /apriltag/detections || true
  ros2 topic info --verbose --no-daemon /apriltag/tag_pose || true
  ros2 topic info --verbose --no-daemon /apriltag/target_pose || true
'
```

## 11. 故障判断表

| 现象 | 最可能原因 | 处理 |
|---|---|---|
| cockpit 有画面，AprilTag 没检测 | remap 仍指向 `/front_camera/...` 或 `/camera/realsense2_camera/...` | 改为 `/g1_robot/front_camera/color/...` |
| AprilTag 启动后 cockpit 黑屏 | AprilTag/V4L2 节点抢了 D435i | 停 V4L2 节点，改 ROS topic subscriber |
| `camera_info` 收不到 | 订阅 topic 错或 QoS 不兼容 | 用 `ros2 topic info --verbose` 查 publisher/subscriber |
| 有 detections 但 pose transform 失败 | `output_frame` 与相机 frame 没有 TF | 查 `CameraInfo.header.frame_id`，补 TF 或换 output_frame |
| 检测延迟高 | 每帧都跑 AprilTag 或使用 compressed 图像 | 降频、KEEP_LAST(1)、raw image、必要时 composable |
| 两台 G1 配置不同 | 把 serial/topic 写死在代码或 YAML | serial 放 `/etc/botbrain/robot.env`，topic 用 launch args |

## 12. 最终建议

对 g1edu 当前状态，建议按这个优先级执行:

1. 保持 `/g1_robot/front_camera` 作为 D435i 唯一 owner。
2. 所有 AprilTag 节点订阅 `/g1_robot/front_camera/color/image_raw` 和 `/g1_robot/front_camera/color/camera_info`。
3. 不再常规运行 `v4l2_apriltag_trigger.py`。
4. 把 `bot_navigation/launch/apriltag_detection.launch.py` 的 remap 修到 `/g1_robot/...`。
5. 对需要触发拍照的业务，新增或改造为 ROS image trigger 节点，而不是打开 `/dev/video*`。
6. 后续把 D435i 从 `localization` 拆到独立 `camera` compose service，让 Web、localization、AprilTag、manipulation 都成为消费者。
