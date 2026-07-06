const JPEG_BASE64 =
  '/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAP//////////////////////////////////////////////////////////////////////////////////////2wBDAf//////////////////////////////////////////////////////////////////////////////////////wAARCAABAAEDASIAAhEBAxEB/8QAFQABAQAAAAAAAAAAAAAAAAAAAAX/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oADAMBAAIQAxAAAAH/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAEFAqf/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oACAEDAQE/ASP/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oACAECAQE/ASP/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAY/An//xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAE/ISf/2gAMAwEAAgADAAAAEP/EABQRAQAAAAAAAAAAAAAAAAAAABD/2gAIAQMBAT8QH//EABQRAQAAAAAAAAAAAAAAAAAAABD/2gAIAQIBAT8QH//EABQQAQAAAAAAAAAAAAAAAAAAABD/2gAIAQEAAT8QH//Z';

export const CAMERA_FRAME_BYTES = Array.from(Buffer.from(JPEG_BASE64, 'base64'));

export const G1_JOINT_NAMES = [
  'left_hip_pitch_joint',
  'left_hip_roll_joint',
  'left_hip_yaw_joint',
  'left_knee_joint',
  'left_ankle_pitch_joint',
  'left_ankle_roll_joint',
  'right_hip_pitch_joint',
  'right_hip_roll_joint',
  'right_hip_yaw_joint',
  'right_knee_joint',
  'right_ankle_pitch_joint',
  'right_ankle_roll_joint',
  'waist_yaw_joint',
  'left_shoulder_pitch_joint',
  'left_shoulder_roll_joint',
  'left_shoulder_yaw_joint',
  'left_elbow_joint',
  'left_wrist_roll_joint',
  'right_shoulder_pitch_joint',
  'right_shoulder_roll_joint',
  'right_shoulder_yaw_joint',
  'right_elbow_joint',
  'right_wrist_roll_joint',
];

export function makeStamp(nowMs = Date.now()) {
  const sec = Math.floor(nowMs / 1000);
  return {
    sec,
    nanosec: (nowMs - sec * 1000) * 1000000,
  };
}

export function makeHeader(seq, frameId, nowMs = Date.now()) {
  return {
    seq,
    stamp: makeStamp(nowMs),
    frame_id: frameId,
  };
}

export function quaternionFromYaw(yaw) {
  return {
    x: 0,
    y: 0,
    z: Math.sin(yaw / 2),
    w: Math.cos(yaw / 2),
  };
}

export function makeVector3(x = 0, y = 0, z = 0) {
  return { x, y, z };
}

export function makeBatteryState(seq, battery, nowMs = Date.now()) {
  const percentage = clamp(battery.percentage, 0, 1);
  const voltage = 48 + percentage * 9;
  const current = battery.isCharging ? 2.0 : -1.4;

  return {
    header: makeHeader(seq, 'base', nowMs),
    voltage,
    temperature: battery.temperatureC,
    current,
    charge: percentage * 10,
    capacity: 10,
    design_capacity: 10,
    percentage,
    power_supply_status: battery.isCharging ? 1 : 2,
    power_supply_health: percentage < 0.15 ? 3 : 1,
    power_supply_technology: 3,
    present: true,
    cell_voltage: [voltage / 4, voltage / 4, voltage / 4, voltage / 4],
    cell_temperature: [
      battery.temperatureC,
      battery.temperatureC + 0.2,
      battery.temperatureC - 0.1,
      battery.temperatureC,
    ],
    location: 'mock_g1_pack',
    serial_number: 'MOCK-G1-BATTERY',
  };
}

export function makeOdometry(seq, pose, velocity, nowMs = Date.now()) {
  return {
    header: makeHeader(seq, 'odom', nowMs),
    child_frame_id: 'base_link',
    pose: {
      pose: {
        position: makeVector3(pose.x, pose.y, pose.z),
        orientation: quaternionFromYaw(pose.yaw),
      },
      covariance: zeroCovariance(),
    },
    twist: {
      twist: {
        linear: makeVector3(velocity.linearX, velocity.linearY, 0),
        angular: makeVector3(0, 0, velocity.angularZ),
      },
      covariance: zeroCovariance(),
    },
  };
}

export function makeJointState(seq, positions, nowMs = Date.now()) {
  return {
    header: makeHeader(seq, '', nowMs),
    name: G1_JOINT_NAMES,
    position: positions,
    velocity: G1_JOINT_NAMES.map(() => 0),
    effort: G1_JOINT_NAMES.map(() => 0),
  };
}

export function makeImuTemperature(seq, temperatureC) {
  void seq;
  return { data: temperatureC };
}

export function makeLaserScan(seq, ranges, nowMs = Date.now()) {
  const angleMin = -Math.PI;
  const angleMax = Math.PI;

  return {
    header: makeHeader(seq, 'mid360_link', nowMs),
    angle_min: angleMin,
    angle_max: angleMax,
    angle_increment: (angleMax - angleMin) / Math.max(ranges.length - 1, 1),
    time_increment: 0,
    scan_time: 0.2,
    range_min: 0.12,
    range_max: 8.0,
    ranges,
    intensities: ranges.map((range) => (range < 7.9 ? 80 : 5)),
  };
}

export function makeOccupancyGrid(seq, grid, nowMs = Date.now()) {
  return {
    header: makeHeader(seq, 'map', nowMs),
    info: {
      map_load_time: makeStamp(nowMs),
      resolution: grid.resolution,
      width: grid.width,
      height: grid.height,
      origin: {
        position: makeVector3(grid.originX, grid.originY, 0),
        orientation: quaternionFromYaw(0),
      },
    },
    data: grid.data,
  };
}

export function makeCompressedImage(seq, frameId = 'camera_link', nowMs = Date.now()) {
  return {
    header: makeHeader(seq, frameId, nowMs),
    format: 'jpeg',
    data: CAMERA_FRAME_BYTES,
  };
}

export function makeSportModeState(seq, state, nowMs = Date.now()) {
  return {
    header: makeHeader(seq, 'base_link', nowMs),
    gait_type: state.velocity.linearX || state.velocity.linearY || state.velocity.angularZ ? 1 : 0,
    foot_raise_height: 0.04,
    position: [state.pose.x, state.pose.y, state.pose.z],
    body_height: state.pose.z,
    velocity: [state.velocity.linearX, state.velocity.linearY, 0],
    yaw_speed: state.velocity.angularZ,
    mode: state.sportMode,
    foot_position_body: [
      0.16,
      0.09,
      -0.78,
      0.16,
      -0.09,
      -0.78,
      -0.16,
      0.09,
      -0.78,
      -0.16,
      -0.09,
      -0.78,
    ],
    foot_speed_body: Array(12).fill(0),
  };
}

export function makeRobotStatus(state) {
  return {
    emergency_stop: state.emergencyStop,
    light_state: state.lightState,
    obstacle_avoidance_state: state.obstacleAvoidanceState,
  };
}

export function makeDiagnostics(seq, state, nowMs = Date.now()) {
  const level = state.diagnosticLevel;
  const message = state.emergencyStop ? 'Emergency stop active' : state.diagnosticMessage;
  const status = [
    {
      level,
      name: 'mock_g1/connection',
      message,
      hardware_id: 'mock-g1',
      values: [
        { key: 'scenario', value: state.scenarioName },
        { key: 'mode', value: state.mode },
        { key: 'emergency_stop', value: String(state.emergencyStop) },
      ],
    },
    {
      level: state.battery.percentage < 0.2 ? 1 : 0,
      name: 'mock_g1/battery',
      message: state.battery.percentage < 0.2 ? 'Battery low' : 'Battery nominal',
      hardware_id: 'mock-g1-battery',
      values: [{ key: 'percentage', value: `${Math.round(state.battery.percentage * 100)}%` }],
    },
  ];

  if (state.missionStatus) {
    status.push(makeMissionDiagnosticStatus(state.missionStatus));
  }

  return {
    header: makeHeader(seq, 'mock_g1', nowMs),
    status,
  };
}

export function makeDiagnosticStatsString(state) {
  const cpu = 24 + Math.round(Math.abs(Math.sin(state.elapsedSeconds / 4)) * 18);
  const gpu = 12 + Math.round(Math.abs(Math.cos(state.elapsedSeconds / 5)) * 24);
  const ramUsed = 3.1 + Math.abs(Math.sin(state.elapsedSeconds / 12)) * 0.8;
  const diskUsed = 28.2;
  const temp = state.battery.temperatureC + 8;

  return {
    data: [
      '[jetson_stats board status] NV Power[0] MODE_15W - JC active',
      ' - Up Time: 0:42:10',
      ' - jetson_clocks on boot: True',
      ' - jtop: mockg1',
      `[jetson_stats cpu 0] ${cpu}%`,
      ' - Freq: 1420800',
      ' - Governor: schedutil',
      `[jetson_stats cpu 1] ${Math.max(4, cpu - 7)}%`,
      ' - Freq: 1420800',
      ' - Governor: schedutil',
      `[jetson_stats gpu gpu] ${gpu}%`,
      `[jetson_stats mem ram] ${ramUsed.toFixed(1)}GB/8.0GB`,
      '[jetson_stats temp]',
      ` - CPU: ${temp.toFixed(1)}`,
      ` - GPU: ${(temp - 2).toFixed(1)}`,
      `[jetson_stats power] curr=${4200 + gpu * 30}mW avg=4500mW`,
      '[jetson_stats pwmfan fan] speed=[34]%',
      `[jetson_stats board disk] ${diskUsed.toFixed(1)}GB/64.0GB`,
      '[jetson_stats board config]',
      ' - Model: Mock Jetson Orin Nano',
      ' - Serial Number: MOCK-G1-JETSON',
      ' - Jetpack: 6.0',
    ].join('\n'),
  };
}

export function makeStateMachineStatus(seq, nodes, nowMs = Date.now()) {
  return {
    header: makeHeader(seq, 'state_machine', nowMs),
    containers: nodes.map((node) => ({
      name: node.name,
      display_name: node.displayName,
      status: node.status,
    })),
  };
}

export function makeListenerMessage(state) {
  return {
    data: `mockg1 mode=${state.mode} emergency=${state.emergencyStop} x=${state.pose.x.toFixed(2)} y=${state.pose.y.toFixed(2)}`,
  };
}

function zeroCovariance() {
  return Array(36).fill(0);
}

function makeMissionDiagnosticStatus(status) {
  const completedCount = status.completed_test_cases.length;
  const routeTotal = status.route_total || 36;
  const fault = status.last_fault;

  return {
    level: fault ? 2 : 0,
    name: 'mission: supervisor',
    message: fault ? fault.message || fault.error_code || 'Mission fault' : summarizeMissionStatus(status),
    hardware_id: 'mission-supervisor',
    values: [
      { key: 'mission_state', value: status.mission_state },
      { key: 'phase', value: status.phase },
      { key: 'current_floor', value: status.current_floor },
      { key: 'target_floor', value: status.target_floor ?? '' },
      { key: 'current_waypoint', value: status.current_waypoint ?? '' },
      { key: 'completed_test_cases', value: String(completedCount) },
      { key: 'route_index', value: String(status.route_index) },
      { key: 'route_total', value: String(routeTotal) },
      { key: 'state_version', value: String(status.state_version) },
    ],
  };
}

function summarizeMissionStatus(status) {
  const state = status.mission_state || 'UNKNOWN';
  const phase = status.phase || 'unknown';
  const waypoint = status.current_waypoint ? ` wp=${status.current_waypoint}` : '';
  const completedCount = status.completed_test_cases.length;
  const routeTotal = status.route_total || 36;

  return `${state}/${phase}${waypoint} tc=${completedCount}/${routeTotal}`;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}
