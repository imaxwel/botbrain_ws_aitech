import assert from 'node:assert/strict';
import test from 'node:test';
import { WebSocket } from 'ws';
import { G1Simulator } from '../src/g1-simulator.js';
import { RosbridgeMockServer } from '../src/rosbridge.js';
import { getScenarioConfig } from '../src/scenarios.js';

const silentLogger = {
  error() {},
  warn() {},
  info() {},
  debug() {},
};

test('publishes core subscribed topics', async () => {
  const { server, ws } = await startClient();
  try {
    ws.send(JSON.stringify({ op: 'subscribe', id: 'sub-battery', topic: '/battery' }));
    ws.send(JSON.stringify({ op: 'subscribe', id: 'sub-odom', topic: '/odom' }));
    ws.send(JSON.stringify({ op: 'subscribe', id: 'sub-status', topic: '/robot_status' }));

    const battery = await waitForMessage(ws, (msg) => msg.topic === '/battery');
    const odom = await waitForMessage(ws, (msg) => msg.topic === '/odom');
    const status = await waitForMessage(ws, (msg) => msg.topic === '/robot_status');

    assert.equal(battery.op, 'publish');
    assert.equal(typeof battery.msg.percentage, 'number');
    assert.equal(odom.msg.child_frame_id, 'base_link');
    assert.equal(typeof odom.msg.pose.pose.position.x, 'number');
    assert.equal(typeof status.msg.emergency_stop, 'boolean');
  } finally {
    await stop(server, ws);
  }
});

test('responds to rosapi get_time service calls', async () => {
  const { server, ws } = await startClient();
  try {
    ws.send(JSON.stringify({
      op: 'call_service',
      id: 'time-1',
      service: '/rosapi/get_time',
      args: {},
    }));

    const response = await waitForMessage(ws, (msg) => msg.id === 'time-1');
    assert.equal(response.op, 'service_response');
    assert.equal(response.result, true);
    assert.equal(typeof response.values.time.secs, 'number');
    assert.equal(typeof response.values.time.nsecs, 'number');
  } finally {
    await stop(server, ws);
  }
});

test('sets and reads current mode and rejects invalid modes', async () => {
  const { server, ws } = await startClient();
  try {
    ws.send(JSON.stringify({
      op: 'call_service',
      id: 'mode-run',
      service: '/mode',
      args: { mode: 'run' },
    }));
    const modeResponse = await waitForMessage(ws, (msg) => msg.id === 'mode-run');
    assert.equal(modeResponse.result, true);
    assert.equal(modeResponse.values.success, true);

    ws.send(JSON.stringify({
      op: 'call_service',
      id: 'current-mode',
      service: '/current_mode',
      args: {},
    }));
    const currentResponse = await waitForMessage(ws, (msg) => msg.id === 'current-mode');
    assert.equal(currentResponse.values.mode, 'run');

    ws.send(JSON.stringify({
      op: 'call_service',
      id: 'bad-mode',
      service: '/mode',
      args: { mode: 'not_a_mode' },
    }));
    const badResponse = await waitForMessage(ws, (msg) => msg.id === 'bad-mode');
    assert.equal(badResponse.result, true);
    assert.equal(badResponse.values.success, false);
  } finally {
    await stop(server, ws);
  }
});

test('toggles emergency stop and publishes status changes', async () => {
  const { server, ws } = await startClient();
  try {
    ws.send(JSON.stringify({ op: 'subscribe', id: 'sub-status', topic: '/robot_status' }));
    ws.send(JSON.stringify({ op: 'subscribe', id: 'sub-sport', topic: '/lf/sportmodestate' }));

    ws.send(JSON.stringify({
      op: 'call_service',
      id: 'emergency-on',
      service: '/emergency_stop',
      args: { data: true },
    }));

    const onResponse = await waitForMessage(ws, (msg) => msg.id === 'emergency-on');
    assert.equal(onResponse.values.success, true);

    const emergencyStatus = await waitForMessage(
      ws,
      (msg) => msg.topic === '/robot_status' && msg.msg.emergency_stop === true,
    );
    const emergencySport = await waitForMessage(
      ws,
      (msg) => msg.topic === '/lf/sportmodestate' && msg.msg.mode === 99,
    );
    assert.equal(emergencyStatus.msg.emergency_stop, true);
    assert.equal(emergencySport.msg.mode, 99);

    ws.send(JSON.stringify({
      op: 'call_service',
      id: 'emergency-off',
      service: '/emergency_stop',
      args: { data: false },
    }));
    const offResponse = await waitForMessage(ws, (msg) => msg.id === 'emergency-off');
    assert.equal(offResponse.values.success, true);

    const normalStatus = await waitForMessage(
      ws,
      (msg) => msg.topic === '/robot_status' && msg.msg.emergency_stop === false,
    );
    assert.equal(normalStatus.msg.emergency_stop, false);
  } finally {
    await stop(server, ws);
  }
});

test('integrates velocity commands and blocks motion during emergency stop', async () => {
  const { server, ws } = await startClient();
  try {
    ws.send(JSON.stringify({ op: 'subscribe', id: 'sub-odom', topic: '/odom' }));
    const initialOdom = await waitForMessage(ws, (msg) => msg.topic === '/odom');
    const initialX = initialOdom.msg.pose.pose.position.x;

    ws.send(JSON.stringify({
      op: 'publish',
      topic: '/cmd_vel_nipple',
      msg: {
        linear: { x: 0.4, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      },
    }));

    const movedOdom = await waitForMessage(
      ws,
      (msg) => msg.topic === '/odom' && msg.msg.pose.pose.position.x > initialX + 0.01,
      1500,
    );
    const movedX = movedOdom.msg.pose.pose.position.x;

    ws.send(JSON.stringify({
      op: 'call_service',
      id: 'emergency-on-motion',
      service: '/emergency_stop',
      args: { data: true },
    }));
    await waitForMessage(ws, (msg) => msg.id === 'emergency-on-motion');

    ws.send(JSON.stringify({
      op: 'publish',
      topic: '/cmd_vel_nipple',
      msg: {
        linear: { x: 0.6, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      },
    }));

    await delay(250);
    const stoppedOdomA = await waitForMessage(ws, (msg) => msg.topic === '/odom');
    await delay(150);
    const stoppedOdomB = await waitForMessage(ws, (msg) => msg.topic === '/odom');
    const stoppedXA = stoppedOdomA.msg.pose.pose.position.x;
    const stoppedXB = stoppedOdomB.msg.pose.pose.position.x;
    assert.ok(stoppedXA >= movedX);
    assert.ok(Math.abs(stoppedXB - stoppedXA) < 0.01);
  } finally {
    await stop(server, ws);
  }
});

test('publishes map and compressed camera messages', async () => {
  const { server, ws } = await startClient();
  try {
    ws.send(JSON.stringify({ op: 'subscribe', id: 'sub-map', topic: '/map' }));
    ws.send(JSON.stringify({ op: 'subscribe', id: 'sub-camera', topic: '/compressed_camera' }));

    const map = await waitForMessage(ws, (msg) => msg.topic === '/map');
    const camera = await waitForMessage(ws, (msg) => msg.topic === '/compressed_camera');

    assert.equal(map.msg.info.width, 48);
    assert.equal(map.msg.info.height, 48);
    assert.equal(map.msg.data.length, 48 * 48);
    assert.equal(camera.msg.format, 'jpeg');
    assert.ok(Array.isArray(camera.msg.data));
    assert.ok(camera.msg.data.length > 20);
  } finally {
    await stop(server, ws);
  }
});

async function startClient(options = {}) {
  const scenario = getScenarioConfig(options.scenario ?? 'default');
  const simulator = new G1Simulator({ scenario, logger: silentLogger });
  const server = new RosbridgeMockServer({
    host: '127.0.0.1',
    port: 0,
    namespace: options.namespace ?? '',
    simulator,
    logger: silentLogger,
  });
  await server.start();

  const ws = new WebSocket(server.url);
  await new Promise((resolve, reject) => {
    ws.once('open', resolve);
    ws.once('error', reject);
  });

  return { server, ws };
}

function waitForMessage(ws, predicate, timeoutMs = 1000) {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      cleanup();
      reject(new Error(`Timed out waiting for WebSocket message after ${timeoutMs}ms`));
    }, timeoutMs);

    const onMessage = (data) => {
      let parsed;
      try {
        parsed = JSON.parse(data.toString());
      } catch {
        return;
      }

      if (predicate(parsed)) {
        cleanup();
        resolve(parsed);
      }
    };

    const onClose = () => {
      cleanup();
      reject(new Error('WebSocket closed while waiting for message'));
    };

    const cleanup = () => {
      clearTimeout(timer);
      ws.off('message', onMessage);
      ws.off('close', onClose);
    };

    ws.on('message', onMessage);
    ws.on('close', onClose);
  });
}

async function stop(server, ws) {
  if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) {
    ws.close();
  }
  await server.stop();
}

function delay(ms) {
  return new Promise((resolve) => {
    setTimeout(resolve, ms);
  });
}
