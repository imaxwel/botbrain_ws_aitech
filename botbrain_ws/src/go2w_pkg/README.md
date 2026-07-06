# go2w_pkg

The Go2W robot-specific package for the BotBrain platform. This package provides all the necessary components, configurations, and interfaces specifically designed for the Unitree Go2W wheeled-quadruped robot.

## Description

The `go2w_pkg` package serves as the robot-specific implementation for the Unitree Go2W. It bridges the Unitree SDK's DDS topics to the ROS 2 ecosystem, handling sensor data reading, robot command writing, joystick/controller input mapping, and video streaming. All nodes follow the ROS 2 lifecycle pattern.

## Directory Structure

```
go2w_pkg/
├── config/                      # Configuration files
│   ├── camera_config.yaml      # RealSense camera TF and serial numbers
│   ├── nav2_params.yaml        # Nav2 navigation parameters
│   └── pointcloud_to_laserscan_params.yaml  # Point cloud to laser scan conversion
├── go2w_pkg/                    # Python package
│   └── __init__.py
├── go2w_setup.bash              # Go2W-specific environment setup script
├── include/go2w_pkg/            # C++ header directory (reserved)
├── launch/
│   ├── pc2ls.launch.py         # Point cloud to laser scan launcher
│   └── robot_interface.launch.py  # Main Go2W robot interface launcher
├── maps/                        # Stored navigation maps
├── meshes/                      # 3D mesh files for robot visualization
│   ├── base.dae                # Robot base mesh
│   ├── calf.stl / calf_mirror.stl
│   ├── foot.dae
│   ├── go2w_interface.stl      # Go2W interface mesh
│   ├── hip.dae
│   ├── left_wheel.dae / right_wheel.dae  # Wheel meshes (Go2W-specific)
│   └── thigh.dae / thigh_mirror.dae
├── scripts/                     # Executable Python nodes
│   ├── go2w_controller_commands.py  # Joystick/controller button handler
│   ├── go2w_read.py            # Sensor data reader (lifecycle node)
│   ├── go2w_video_stream.py    # Video stream publisher (lifecycle node)
│   └── go2w_write.py           # Robot command writer (lifecycle node)
├── urdf/
│   └── robot.urdf              # Go2W robot description
├── xacro/                       # XACRO files (reserved)
├── CMakeLists.txt
└── package.xml
```

## Nodes

### `go2w_read.py` — `robot_read_node`

Lifecycle node that bridges Unitree DDS data into standard ROS 2 topics.

**Subscribes:**
| Topic | Type | Description |
|---|---|---|
| `/lf/sportmodestate` | `unitree_go/SportModeState` | Sport mode state (IMU, position, velocity) |
| `/lf/lowstate` | `unitree_go/LowState` | Low-level state (motors, battery, BMS) |
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | Raw LiDAR point cloud |

**Publishes:**
| Topic | Type | Description |
|---|---|---|
| `odom` | `nav_msgs/Odometry` | Robot odometry |
| `imu/data` | `sensor_msgs/Imu` | IMU orientation, angular velocity, linear acceleration |
| `imu_temp` | `std_msgs/Float32` | IMU temperature |
| `pointcloud` | `sensor_msgs/PointCloud2` | Republished LiDAR cloud (frame: `radar`) |
| `battery` | `sensor_msgs/BatteryState` | Battery voltage, current, and state of charge |
| `joint_states` | `sensor_msgs/JointState` | All 16 joint positions and velocities |
| `motor_temp` | `bot_custom_interfaces/MotorTemperature` | Per-motor temperatures |

Also broadcasts the `{prefix}odom → {prefix}base_link` TF transform.

> All topics listed above are published under the robot namespace (`/{robot_name}/`) as configured in `robot_config.yaml`.

---

### `go2w_write.py` — `lifecycle_robot_write_node`

Lifecycle node that translates ROS 2 commands into Unitree API requests and publishes robot status.

**Subscribes:**
| Topic | Type | Description |
|---|---|---|
| `cmd_vel_out` | `geometry_msgs/Twist` | Velocity commands (forwarded when not in emergency stop) |
| `/lf/sportmodestate` | `unitree_go/SportModeState` | Current op mode and gait type |
| `/api/sport/response` | `unitree_api/Response` | Sport API response |
| `/api/robot_state/response` | `unitree_api/Response` | Robot state API response |
| `/api/vui/response` | `unitree_api/Response` | VUI (lights) API response |

**Publishes:**
| Topic | Type | Description |
|---|---|---|
| `/api/sport/request` | `unitree_api/Request` | Sport mode commands |
| `/api/robot_state/request` | `unitree_api/Request` | Robot state commands |
| `/api/vui/request` | `unitree_api/Request` | VUI (LED) commands |
| `robot_status` | `bot_custom_interfaces/RobotStatus` | Emergency stop state and light state (2 Hz) |

**Services:**
| Service | Type | Description |
|---|---|---|
| `mode` | `bot_custom_interfaces/Mode` | Switch mode: `damp`, `balance_stand`, `stand_up`, `stand_down` |
| `speed_level` | `bot_custom_interfaces/SpeedLevel` | Set speed level in range `[-1.0, 1.0]` |
| `current_mode` | `bot_custom_interfaces/CurrentMode` | Get current mode string |
| `emergency_stop` | `std_srvs/SetBool` | Toggle emergency stop (stops motion and damps the robot) |
| `light_control` | `bot_custom_interfaces/LightControl` | Set LED brightness `[1–10]` or turn off |

---

### `go2w_controller_commands.py` — `controller_commands_node`

Lifecycle node that maps physical controller button presses to robot service calls at 10 Hz.

**Button Mappings:**
| Button Combo | Action |
|---|---|
| `start` | `balance_stand` (or exit pose mode if in mode 2) |
| `L2 + A` | Toggle `stand_up` / `stand_down` |
| `L2 + B` | `damp` |
| `L2 + X` | `recovery_stand` |

---

### `go2w_video_stream.py` — `robot_video_stream`

Lifecycle node that captures the Go2W's onboard H.264 camera stream via GStreamer and publishes it as ROS 2 image topics at 24 fps.

- **Stream source:** UDP multicast `230.1.1.1:1720` on interface `eno1`
- **Resolution:** 1280×720

**Publishes:**
| Topic | Type | Description |
|---|---|---|
| `camera` | `sensor_msgs/Image` | Raw BGR image (1280×720) |
| `compressed_camera` | `sensor_msgs/CompressedImage` | JPEG compressed image (640×360, quality 20) |

> **Note:** The video stream node is commented out in `robot_interface.launch.py` and must be enabled manually if needed.

---

## Launch Files

### `robot_interface.launch.py`

Starts the full Go2W robot interface:
- `go2w_read_node`
- `go2w_write_node`
- `controller_commands_node`
- Point cloud to laser scan (`pc2ls.launch.py`)

The robot namespace and TF prefix are read from `robot_config.yaml`.

### `pc2ls.launch.py`

Converts the `pointcloud` topic to a `LaserScan` using the `pointcloud_to_laserscan` package, using parameters from `config/pointcloud_to_laserscan_params.yaml`.

---

## Configuration

### `go2w_setup.bash`

Environment setup script that:
1. Reads `network_interface` from `robot_config.yaml`
2. Sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI` pointing to `cyclonedds_config.xml`
3. Configures the Ethernet interface with a static IP (`192.168.123.170/16`) to communicate with the Go2W

Source this script before launching the robot interface:
```bash
source botbrain_ws/src/go2w_pkg/go2w_setup.bash
```

### `config/camera_config.yaml`

Defines TF frames and serial numbers for the RealSense depth cameras:
- **Front:** D435i (serial `420122071637`), mounted at `botbrain_base`
- **Back:** slot reserved, currently unconfigured

---

## Dependencies

- `rclpy`, `rclcpp`
- `unitree_go` — Unitree Go2 message types (`LowState`, `SportModeState`)
- `unitree_api` — Unitree API request/response messages
- `bot_custom_interfaces` — Custom BotBrain service and message types
- `joystick_bot` — Controller button state messages
- `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`
- `tf2_ros`
- `cv2` + `cv_bridge` (video stream node)
- `pointcloud_to_laserscan`

---

## Contributing

1. **Create Feature Branch**: Create a new branch from `dev` named `feature/[name_of_feature]`
2. **Development**: Make your changes following the Go2W-specific patterns
3. **Testing**: Test with actual Go2W robot hardware
4. **Documentation**: Update this README with any new features or changes
5. **Submit Pull Request**: Create a pull request to the `dev` branch

---

**Note:** This package is specifically designed for the Unitree Go2W robot. Ensure the robot is powered on, the Ethernet interface is correctly configured via `go2w_setup.bash`, and the Unitree SDK DDS bridge is running before launching.
