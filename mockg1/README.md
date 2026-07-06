# BotBrain Mock G1

Standalone rosbridge-compatible WebSocket mock for developing BotBrain G1 pages without a real Unitree G1, ROS2, Unitree SDK, or `rosbridge_server`.

The mock runs on the same protocol surface the frontend already uses. Point BotBrain at `localhost` and the UI can subscribe to status, sensors, maps, cameras, diagnostics, and send control/service commands.

## Quick Start

```bash
cd BotBrain/mockg1
npm install
npm start
```

Default endpoint:

```text
ws://localhost:9090
```

In BotBrain's frontend connection UI, use either:

```text
localhost
```

or:

```text
ws://localhost:9090
```

## Options

```bash
npm start -- --host 127.0.0.1 --port 9090 --scenario default
```

Environment variables:

| Variable | Default | Description |
| --- | --- | --- |
| `MOCKG1_HOST` | `0.0.0.0` | Listen host |
| `MOCKG1_PORT` | `9090` | Listen port |
| `MOCKG1_SCENARIO` | `default` | Scenario name |
| `MOCKG1_NAMESPACE` | empty | Also accept `/<namespace>/topic` and map it to `/topic` |
| `MOCKG1_CAMERA_FPS` | `5` | Camera topic publish rate |
| `MOCKG1_LOG_LEVEL` | `info` | `error`, `warn`, `info`, `debug`, or `silent` |

Available scenarios:

| Scenario | Behavior |
| --- | --- |
| `default` | Nominal battery, normal status |
| `low_battery` | Starts with low battery and warning diagnostics |
| `emergency` | Starts with emergency stop active |
| `unstable_network` | Periodically closes one client connection |

## rosbridge Support

Supported client operations:

| Operation | Behavior |
| --- | --- |
| `subscribe` / `unsubscribe` | Tracks per-client topic subscriptions and only pushes subscribed topics |
| `advertise` / `unadvertise` | Accepted for frontend publishers |
| `publish` | Accepts velocity, goal, cancel, initial pose, and audio topics |
| `call_service` | Returns rosbridge `service_response` with matching `id` |
| `set_level` | Accepted as a no-op |

Outgoing frames are normal rosbridge JSON, for example:

```json
{ "op": "publish", "topic": "/battery", "msg": { "percentage": 0.82 } }
```

ROSLIB 2.x accepts JSON even when a subscription requests `compression: "cbor"`.

## Topics

| Topic | Type | Rate / Behavior |
| --- | --- | --- |
| `/battery` | `sensor_msgs/BatteryState` | 2 Hz, slowly drains |
| `/imu_temp` | `std_msgs/Float32` | 2 Hz |
| `/odom` | `nav_msgs/Odometry` | 10 Hz, integrated from latest velocity command |
| `/joint_states` | `sensor_msgs/JointState` | 20 Hz, G1 23-joint names |
| `/scan` | `sensor_msgs/LaserScan` | 5 Hz, synthetic room and moving obstacle |
| `/map` | `nav_msgs/OccupancyGrid` | Immediate on subscribe, then 1 Hz |
| `/compressed_camera` | `sensor_msgs/CompressedImage` | Synthetic JPEG |
| `/compressed_back_camera` | `sensor_msgs/CompressedImage` | Synthetic JPEG |
| `/viz/camcam/rgb/compressed_image` | `sensor_msgs/CompressedImage` | Synthetic JPEG |
| `/viz/camcam/thermal/compressed_image` | `sensor_msgs/CompressedImage` | Synthetic JPEG |
| `/lf/sportmodestate` | `unitree_go/SportModeState` | 10 Hz, frontend mode codes |
| `/robot_status` | `bot_custom_interfaces/RobotStatus` | 2 Hz |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz |
| `/diagnostic_stats` | `std_msgs/String` | 1 Hz Jetson-like text stats |
| `/state_machine/status` | `bot_custom_interfaces/StatusArray` | 1 Hz |
| `/listener` | `std_msgs/String` | 1 Hz status log line |

Accepted publish topics:

| Topic | Behavior |
| --- | --- |
| `/cmd_vel_nipple` | Drives simulated odometry |
| `/cmd_vel_joy` | Drives simulated odometry |
| `/goal_pose` | Stores goal and switches to run mode if safe |
| `/cancel_goal` | Clears goal and stops motion |
| `/initialpose` | Updates simulated x/y pose |
| `/audio_streaming` | Counts audio packets, no audio processing |

## Services

| Service | Behavior |
| --- | --- |
| `/rosapi/get_time` | Returns current mock time for frontend keep-alive |
| `/mode` | Sets G1 mode, with frontend action aliases |
| `/current_mode` | Returns current G1 mode string |
| `/emergency_stop` | `data: true` activates emergency, `data: false` releases it |
| `/arm_cmd` | Simulates save/apply/release/delete pose commands |
| `/pose` | Toggles pose mode |
| `/light_control` | Toggles light state |
| `/obstacle_avoidance` | Toggles obstacle avoidance state |
| `/rosa_prompt` | Acknowledges prompt |
| `/talk` | Acknowledges TTS/talk request |
| `/delivery_control` | Acknowledges delivery request |
| `/state_machine/command` | Activates, deactivates, or restarts mock nodes |

Strict G1 modes accepted by the real G1 driver are:

```text
zero_torque, damp, preparation, run, squat, start
```

The mock also accepts UI aliases such as `damping`, `balance`, `manipulation`, `dance`, `dance1`, `hello`, `stop_move`, `sit`, and `stretch` so current BotBrain controls can be tested without frontend changes.

## Tests

```bash
npm test
```

The tests start the mock on a random local port and validate rosbridge publish/service behavior with a WebSocket client.

## Notes

- This mock is intentionally contract-level. It does not emulate Unitree DDS internals.
- No command has real robot side effects.
- By default, topics are un-namespaced to match the current frontend legacy ROS paths. Set `MOCKG1_NAMESPACE=g1_robot` to also accept `/g1_robot/battery`, `/g1_robot/odom`, and similar aliases.
