import {
  G1_JOINT_NAMES,
  makeBatteryState,
  makeCompressedImage,
  makeDiagnosticStatsString,
  makeDiagnostics,
  makeImuTemperature,
  makeJointState,
  makeLaserScan,
  makeListenerMessage,
  makeOccupancyGrid,
  makeOdometry,
  makeRobotStatus,
  makeSportModeState,
  makeStateMachineStatus,
} from './messages.js';

const MODE_ALIASES = {
  damping: 'damp',
  balance: 'run',
  balance_stand: 'run',
  stand_up: 'preparation',
  stand_down: 'squat',
  manipulation: 'preparation',
  dance: 'run',
  dance1: 'run',
  hello: 'run',
  stop_move: 'run',
  sit: 'squat',
  rise_sit: 'preparation',
  stretch: 'run',
  start: 'run',
};

const VALID_MODES = new Set(['zero_torque', 'damp', 'preparation', 'run', 'squat']);

const STATE_MACHINE_SUCCESS = 1;
const STATE_MACHINE_INVALID_NODE = 3;
const STATE_MACHINE_INVALID_COMMAND = 4;
const MISSION_SNAPSHOT_TOPIC = '/mock/mission_snapshot';
const MISSION_MIRROR_SCHEMA_VERSION = 'g1.mock.mission_mirror.v1';

export class G1Simulator {
  constructor({ scenario, namespace = '', logger = console } = {}) {
    this.scenario = scenario;
    this.namespace = namespace;
    this.logger = logger;
    this.seq = new Map();
    this.startedAt = Date.now();
    this.lastUpdateAt = Date.now();
    this.lastCommandAt = 0;
    this.lastGoal = null;
    this.lastInitialPose = null;
    this.audioPacketsReceived = 0;
    this.poseNames = new Set(['home', 'wave', 'carry']);

    this.pose = { x: 0, y: 0, z: 0.78, yaw: 0 };
    this.velocity = { linearX: 0, linearY: 0, angularZ: 0 };
    this.command = { linearX: 0, linearY: 0, angularZ: 0 };
    this.mode = scenario.initialMode;
    this.emergencyStop = Boolean(scenario.initialEmergencyStop);
    this.lightState = false;
    this.obstacleAvoidanceState = false;
    this.poseMode = false;
    this.battery = {
      percentage: scenario.batteryPercentage,
      temperatureC: scenario.imuTemperatureC,
      isCharging: false,
    };
    this.diagnosticLevel = scenario.diagnosticLevel;
    this.diagnosticMessage = scenario.diagnosticMessage;
    this.scenarioName = scenario.name;
    this.stateMachineNodes = createStateMachineNodes();
    this.mapGrid = createMapGrid();
    this.missionStatus = null;
    this.missionTargetPose = null;
  }

  get elapsedSeconds() {
    return (Date.now() - this.startedAt) / 1000;
  }

  get sportMode() {
    if (this.emergencyStop) return 99;
    if (this.poseMode || this.mode === 'squat') return 2;
    if (this.mode === 'run') return 3;
    if (this.mode === 'preparation') return 1;
    if (this.mode === 'damp') return 7;
    return 0;
  }

  update(nowMs = Date.now()) {
    const dt = Math.max(0, Math.min((nowMs - this.lastUpdateAt) / 1000, 0.25));
    this.lastUpdateAt = nowMs;

    if (this.emergencyStop || nowMs - this.lastCommandAt > 800) {
      this.command = { linearX: 0, linearY: 0, angularZ: 0 };
    }

    this.velocity = { ...this.command };

    const cos = Math.cos(this.pose.yaw);
    const sin = Math.sin(this.pose.yaw);
    this.pose.x += (this.velocity.linearX * cos - this.velocity.linearY * sin) * dt;
    this.pose.y += (this.velocity.linearX * sin + this.velocity.linearY * cos) * dt;
    this.pose.yaw = normalizeAngle(this.pose.yaw + this.velocity.angularZ * dt);

    this.updateMissionPose(dt);

    this.battery.percentage = Math.max(
      0,
      this.battery.percentage - this.scenario.batteryDrainPerSecond * dt,
    );
    this.battery.temperatureC =
      this.scenario.imuTemperatureC +
      Math.sin(this.elapsedSeconds / 5) * 1.5 +
      Math.abs(this.velocity.linearX) * 1.2;
  }

  handlePublish(topic, msg = {}) {
    if (topic === '/cmd_vel_nipple' || topic === '/cmd_vel_joy') {
      this.setVelocityCommand(msg);
      return { accepted: true, message: 'Velocity command accepted' };
    }

    if (topic === '/goal_pose') {
      this.lastGoal = msg;
      this.mode = this.emergencyStop ? this.mode : 'run';
      return { accepted: true, message: 'Navigation goal stored' };
    }

    if (topic === '/cancel_goal') {
      this.lastGoal = null;
      this.command = { linearX: 0, linearY: 0, angularZ: 0 };
      return { accepted: true, message: 'Navigation goal canceled' };
    }

    if (topic === '/initialpose') {
      this.lastInitialPose = msg;
      const pose = msg.pose?.pose ?? msg.pose;
      if (pose?.position) {
        this.pose.x = Number(pose.position.x) || 0;
        this.pose.y = Number(pose.position.y) || 0;
      }
      return { accepted: true, message: 'Initial pose stored' };
    }

    if (topic === '/audio_streaming') {
      this.audioPacketsReceived += 1;
      return { accepted: true, message: 'Audio packet accepted' };
    }

    if (topic === MISSION_SNAPSHOT_TOPIC) {
      return this.setMissionStatus(msg);
    }

    return { accepted: true, message: 'Topic accepted without side effects' };
  }

  callService(service, args = {}) {
    if (service === '/rosapi/get_time') {
      return {
        result: true,
        values: { time: this.makeRosTime() },
      };
    }

    if (service === '/rosapi/topics') {
      return {
        result: true,
        values: { topics: this.getTopicNames(), types: this.getTopicTypes() },
      };
    }

    if (service === '/rosapi/services') {
      return {
        result: true,
        values: { services: this.getServiceNames() },
      };
    }

    if (service === '/mode') return this.setMode(args.mode);
    if (service === '/current_mode') return { result: true, values: { mode: this.mode } };
    if (service === '/emergency_stop') return this.setEmergencyStop(args.data);
    if (service === '/arm_cmd') return this.handleArmCommand(args);
    if (service === '/pose') return this.setPoseMode(args);
    if (service === '/light_control') return this.setLight(args);
    if (service === '/obstacle_avoidance') return this.setObstacleAvoidance(args);
    if (service === '/rosa_prompt') return this.genericSuccess('ROSA prompt accepted');
    if (service === '/talk') return this.genericSuccess('Talk request accepted');
    if (service === '/delivery_control') return this.genericSuccess('Delivery command accepted');
    if (service === '/state_machine/command') return this.handleStateMachineCommand(args);

    return {
      result: false,
      values: { success: false, message: `Unknown service: ${service}` },
    };
  }

  getMessage(topic) {
    const seq = this.nextSeq(topic);

    if (topic === '/battery') return makeBatteryState(seq, this.battery);
    if (topic === '/imu_temp') return makeImuTemperature(seq, this.battery.temperatureC);
    if (topic === '/odom') return makeOdometry(seq, this.pose, this.velocity);
    if (topic === '/joint_states') return makeJointState(seq, this.makeJointPositions());
    if (topic === '/scan') return makeLaserScan(seq, this.makeLaserRanges());
    if (topic === '/map') return makeOccupancyGrid(seq, this.mapGrid);
    if (topic === '/compressed_camera') return makeCompressedImage(seq, 'front_camera_link');
    if (topic === '/compressed_back_camera') return makeCompressedImage(seq, 'back_camera_link');
    if (topic === '/viz/camcam/rgb/compressed_image') {
      return makeCompressedImage(seq, 'rgb_camera_link');
    }
    if (topic === '/viz/camcam/thermal/compressed_image') {
      return makeCompressedImage(seq, 'thermal_camera_link');
    }
    if (topic === '/lf/sportmodestate') return makeSportModeState(seq, this);
    if (topic === '/robot_status') return makeRobotStatus(this);
    if (topic === '/diagnostics') return makeDiagnostics(seq, this);
    if (topic === '/diagnostic_stats') return makeDiagnosticStatsString(this);
    if (topic === '/state_machine/status') {
      return makeStateMachineStatus(seq, this.getStateMachineNodes());
    }
    if (topic === '/listener') return makeListenerMessage(this);

    return null;
  }

  getTopicNames() {
    return [
      '/battery',
      '/imu_temp',
      '/odom',
      '/joint_states',
      '/scan',
      '/map',
      '/compressed_camera',
      '/compressed_back_camera',
      '/viz/camcam/rgb/compressed_image',
      '/viz/camcam/thermal/compressed_image',
      '/lf/sportmodestate',
      '/robot_status',
      '/diagnostics',
      '/diagnostic_stats',
      '/state_machine/status',
      '/listener',
      MISSION_SNAPSHOT_TOPIC,
    ];
  }

  getTopicTypes() {
    return [
      'sensor_msgs/BatteryState',
      'std_msgs/Float32',
      'nav_msgs/Odometry',
      'sensor_msgs/JointState',
      'sensor_msgs/LaserScan',
      'nav_msgs/OccupancyGrid',
      'sensor_msgs/CompressedImage',
      'sensor_msgs/CompressedImage',
      'sensor_msgs/CompressedImage',
      'sensor_msgs/CompressedImage',
      'unitree_go/SportModeState',
      'bot_custom_interfaces/RobotStatus',
      'diagnostic_msgs/DiagnosticArray',
      'std_msgs/String',
      'bot_custom_interfaces/StatusArray',
      'std_msgs/String',
      'g1_mock/MissionMirrorSnapshot',
    ];
  }

  setMissionStatus(payload) {
    const firstMirrorUpdate = !this.missionStatus;
    const missionStatus = normalizeMissionStatus(payload);

    this.missionStatus = missionStatus;
    this.missionTargetPose = waypointPose(missionStatus.route_index, missionStatus.route_total);

    if (firstMirrorUpdate && this.missionTargetPose) {
      this.pose = { ...this.missionTargetPose };
      this.velocity = { linearX: 0, linearY: 0, angularZ: 0 };
    }

    return {
      accepted: true,
      message: 'Mission snapshot mirrored',
      publishTopics: ['/state_machine/status', '/diagnostics', '/odom'],
    };
  }

  getServiceNames() {
    return [
      '/rosapi/get_time',
      '/rosapi/topics',
      '/rosapi/services',
      '/mode',
      '/current_mode',
      '/emergency_stop',
      '/arm_cmd',
      '/pose',
      '/light_control',
      '/obstacle_avoidance',
      '/rosa_prompt',
      '/talk',
      '/delivery_control',
      '/state_machine/command',
    ];
  }

  setVelocityCommand(msg) {
    if (this.emergencyStop) {
      this.command = { linearX: 0, linearY: 0, angularZ: 0 };
      return;
    }

    this.command = {
      linearX: clamp(Number(msg.linear?.x) || 0, -0.6, 0.6),
      linearY: clamp(Number(msg.linear?.y) || 0, -0.6, 0.6),
      angularZ: clamp(Number(msg.angular?.z) || 0, -1.0, 1.0),
    };
    this.lastCommandAt = Date.now();
  }

  setMode(inputMode) {
    const requestedMode = String(inputMode ?? '');
    const mode = MODE_ALIASES[requestedMode] ?? requestedMode;

    if (!VALID_MODES.has(mode)) {
      return {
        result: true,
        values: { success: false, message: `Invalid mode: ${requestedMode}` },
      };
    }

    if (this.emergencyStop) {
      return {
        result: true,
        values: { success: false, message: 'Emergency is active; mode change is blocked' },
      };
    }

    this.mode = mode;
    this.poseMode = mode === 'squat';

    return {
      result: true,
      values: { success: true, message: `Mode changed to ${mode}` },
    };
  }

  setEmergencyStop(data) {
    const next = typeof data === 'boolean' ? data : !this.emergencyStop;
    this.emergencyStop = next;
    this.command = { linearX: 0, linearY: 0, angularZ: 0 };
    this.velocity = { linearX: 0, linearY: 0, angularZ: 0 };
    if (!next && this.mode === 'zero_torque') this.mode = 'damp';

    return {
      result: true,
      values: {
        success: true,
        message: next ? 'Emergency stop active' : 'Emergency stop released',
      },
    };
  }

  handleArmCommand(args) {
    const command = Number(args.command);
    const name = String(args.name ?? '').trim() || 'mock_pose';

    if (command === 0 || command === 1) {
      this.poseNames.add(name);
      return this.genericSuccess(`Pose saved: ${name}`);
    }
    if (command === 2) return this.genericSuccess(`Pose applied: ${name}`);
    if (command === 3) return this.genericSuccess('Arm joint position released');
    if (command === 4) {
      this.poseNames.delete(name);
      return this.genericSuccess(`Pose deleted: ${name}`);
    }

    return {
      result: true,
      values: { success: false, message: `Unknown arm command: ${command}` },
    };
  }

  setPoseMode(args) {
    this.poseMode = Boolean(args.flag ?? args.control ?? true);
    return this.genericSuccess(this.poseMode ? 'Pose mode enabled' : 'Pose mode disabled');
  }

  setLight(args) {
    this.lightState = Boolean(args.control ?? args.data ?? false);
    return this.genericSuccess(this.lightState ? 'Light enabled' : 'Light disabled');
  }

  setObstacleAvoidance(args) {
    this.obstacleAvoidanceState = Boolean(args.control ?? args.data ?? false);
    return this.genericSuccess(
      this.obstacleAvoidanceState ? 'Obstacle avoidance enabled' : 'Obstacle avoidance disabled',
    );
  }

  handleStateMachineCommand(args) {
    const node = this.stateMachineNodes.find((item) => item.name === args.node);
    if (!node) {
      return { result: true, values: { result: STATE_MACHINE_INVALID_NODE, success: false } };
    }

    const command = Number(args.command);
    if (command === 1) node.status = 'active';
    else if (command === 2) node.status = 'inactive';
    else if (command === 3) node.status = 'active';
    else {
      return {
        result: true,
        values: { result: STATE_MACHINE_INVALID_COMMAND, success: false },
      };
    }

    return { result: true, values: { result: STATE_MACHINE_SUCCESS, success: true } };
  }

  genericSuccess(message) {
    return {
      result: true,
      values: { success: true, message },
    };
  }

  getStateMachineNodes() {
    if (!this.missionStatus) return this.stateMachineNodes;

    return [
      ...this.stateMachineNodes,
      {
        name: 'mission_supervisor',
        displayName: 'Mission Supervisor',
        status: summarizeMissionStatus(this.missionStatus),
      },
    ];
  }

  updateMissionPose(dt) {
    if (!this.missionTargetPose) return;
    if (dt <= 0) return;

    const previous = { ...this.pose };
    const alpha = Math.min(1, dt * 2.5);

    this.pose.x = lerp(this.pose.x, this.missionTargetPose.x, alpha);
    this.pose.y = lerp(this.pose.y, this.missionTargetPose.y, alpha);
    this.pose.z = lerp(this.pose.z, this.missionTargetPose.z, alpha);
    this.pose.yaw = normalizeAngle(
      this.pose.yaw + normalizeAngle(this.missionTargetPose.yaw - this.pose.yaw) * alpha,
    );

    const dx = this.pose.x - previous.x;
    const dy = this.pose.y - previous.y;
    const dyaw = normalizeAngle(this.pose.yaw - previous.yaw);

    if (Math.abs(dx) + Math.abs(dy) + Math.abs(dyaw) > 0.0001) {
      this.velocity = {
        linearX: dx / dt,
        linearY: dy / dt,
        angularZ: dyaw / dt,
      };
    }
  }

  makeJointPositions() {
    const t = this.elapsedSeconds;
    const moving = Math.abs(this.velocity.linearX) + Math.abs(this.velocity.linearY) > 0.01;
    const gaitAmp = moving ? 0.14 : 0.025;

    return G1_JOINT_NAMES.map((name, index) => {
      if (name.includes('knee')) return 0.18 + Math.sin(t * 4 + index) * gaitAmp;
      if (name.includes('ankle')) return Math.sin(t * 3 + index) * gaitAmp * 0.5;
      if (name.includes('shoulder')) return Math.sin(t * 1.2 + index) * 0.08;
      if (name.includes('elbow')) return 0.2 + Math.sin(t + index) * 0.05;
      if (name.includes('waist')) return this.pose.yaw * 0.15;
      return Math.sin(t * 2 + index) * gaitAmp;
    });
  }

  makeLaserRanges() {
    const count = 181;
    const ranges = [];
    const obstacleAngle = Math.sin(this.elapsedSeconds / 3) * 0.8;

    for (let i = 0; i < count; i += 1) {
      const angle = -Math.PI + (i / (count - 1)) * Math.PI * 2;
      const wall = 4.4 + Math.sin(angle * 3) * 0.35;
      const obstacle = Math.abs(angle - obstacleAngle) < 0.12 ? 1.4 : 8.0;
      ranges.push(round(Math.min(wall, obstacle), 3));
    }

    return ranges;
  }

  makeRosTime() {
    const nowMs = Date.now();
    const secs = Math.floor(nowMs / 1000);
    return { secs, nsecs: (nowMs - secs * 1000) * 1000000 };
  }

  nextSeq(topic) {
    const next = (this.seq.get(topic) ?? 0) + 1;
    this.seq.set(topic, next);
    return next;
  }
}

function createStateMachineNodes() {
  return [
    { name: 'robot_write_node', displayName: 'G1 Writer', status: 'active' },
    { name: 'robot_read_node', displayName: 'G1 Reader', status: 'active' },
    { name: 'controller_commands_node', displayName: 'Controller Commands', status: 'active' },
    { name: 'rosbridge_server', displayName: 'ROSBridge', status: 'active' },
    { name: 'localization', displayName: 'Localization', status: 'active' },
    { name: 'navigation', displayName: 'Navigation', status: 'active' },
    { name: 'yolo', displayName: 'YOLO', status: 'inactive' },
    { name: 'rosa', displayName: 'ROSA', status: 'active' },
  ];
}

function createMapGrid() {
  const width = 48;
  const height = 48;
  const data = Array(width * height).fill(0);

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const idx = y * width + x;
      if (x === 0 || y === 0 || x === width - 1 || y === height - 1) data[idx] = 100;
      if (x > 14 && x < 18 && y > 8 && y < 34) data[idx] = 100;
      if (x > 29 && x < 33 && y > 17 && y < 42) data[idx] = 100;
      if (x > 4 && x < 11 && y > 36 && y < 43) data[idx] = -1;
    }
  }

  return {
    width,
    height,
    resolution: 0.1,
    originX: -2.4,
    originY: -2.4,
    data,
  };
}

export function waypointPose(routeIndex = 0, routeTotal = 36) {
  const cols = [-2.0, -1.3, -0.25, 0.25, 1.3, 1.9];
  const rows = [-1.8, -1.2, -0.6, 0.0, 0.8, 1.6];
  const total = Math.max(1, Math.min(Number(routeTotal) || 36, cols.length * rows.length));
  const index = clamp(Math.trunc(Number(routeIndex) || 0), 0, total - 1);
  const row = Math.floor(index / cols.length);
  const rawCol = index % cols.length;
  const col = row % 2 === 0 ? rawCol : cols.length - 1 - rawCol;

  return {
    x: cols[col],
    y: rows[row],
    z: 0.78,
    yaw: row % 2 === 0 ? 0 : Math.PI,
  };
}

function normalizeMissionStatus(payload = {}) {
  const completedTestCases = Array.isArray(payload.completed_test_cases)
    ? payload.completed_test_cases
        .map((value) => Math.trunc(Number(value)))
        .filter((value) => Number.isInteger(value) && value > 0)
    : [];
  const routeTotal = positiveInteger(payload.route_total, 36);
  const fallbackRouteIndex = completedTestCases.length > 0 ? completedTestCases.length - 1 : 0;
  const routeIndex = clamp(integer(payload.route_index, fallbackRouteIndex), 0, routeTotal - 1);

  return {
    schema_version: String(payload.schema_version || MISSION_MIRROR_SCHEMA_VERSION),
    mission_state: stringValue(payload.mission_state, 'UNKNOWN'),
    phase: stringValue(payload.phase, 'unknown'),
    current_floor: stringValue(payload.current_floor, 'unknown'),
    target_floor: optionalString(payload.target_floor),
    current_waypoint: optionalString(payload.current_waypoint),
    completed_test_cases: completedTestCases,
    current_node: optionalString(payload.current_node),
    route_index: routeIndex,
    route_total: routeTotal,
    last_fault: normalizeFault(payload.last_fault),
    state_version: integer(payload.state_version, 0),
  };
}

function normalizeFault(value) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) return null;
  return {
    ...value,
    message: optionalString(value.message),
    error_code: optionalString(value.error_code),
  };
}

function summarizeMissionStatus(status) {
  const waypoint = status.current_waypoint ? ` wp=${status.current_waypoint}` : '';
  const completedCount = status.completed_test_cases.length;
  const routeTotal = status.route_total || 36;
  return `${status.mission_state}/${status.phase}${waypoint} tc=${completedCount}/${routeTotal}`;
}

function stringValue(value, fallback) {
  const text = optionalString(value);
  return text || fallback;
}

function optionalString(value) {
  if (value === null || value === undefined) return null;
  const text = String(value).trim();
  return text || null;
}

function integer(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? Math.trunc(parsed) : fallback;
}

function positiveInteger(value, fallback) {
  return Math.max(1, integer(value, fallback));
}

function normalizeAngle(angle) {
  let next = angle;
  while (next > Math.PI) next -= Math.PI * 2;
  while (next < -Math.PI) next += Math.PI * 2;
  return next;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function round(value, digits) {
  const scale = 10 ** digits;
  return Math.round(value * scale) / scale;
}

function lerp(start, end, alpha) {
  return start + (end - start) * alpha;
}
