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

test('mission snapshot mirror updates state machine, diagnostics, and odom', async () => {
  const { server, ws } = await startClient();

  try {
    ws.send(JSON.stringify({ op: 'subscribe', id: 'sub-state-machine', topic: '/state_machine/status' }));
    ws.send(JSON.stringify({ op: 'subscribe', id: 'sub-diagnostics', topic: '/diagnostics' }));
    ws.send(JSON.stringify({ op: 'subscribe', id: 'sub-odom', topic: '/odom' }));

    const initialStateMachine = await waitForMessage(
      ws,
      (msg) => msg.topic === '/state_machine/status',
    );
    const initialDiagnostics = await waitForMessage(ws, (msg) => msg.topic === '/diagnostics');
    const initialOdom = await waitForMessage(ws, (msg) => msg.topic === '/odom');
    const initialPosition = initialOdom.msg.pose.pose.position;

    assert.equal(
      initialStateMachine.msg.containers.some((container) => container.name === 'mission_supervisor'),
      false,
    );
    assert.equal(
      initialDiagnostics.msg.status.some((status) => status.name === 'mission: supervisor'),
      false,
    );

    ws.send(JSON.stringify({
      op: 'publish',
      topic: '/mock/mission_snapshot',
      msg: {
        schema_version: 'g1.mock.mission_mirror.v1',
        mission_state: 'ACTIVE',
        phase: 'elevator',
        current_floor: 'UG',
        target_floor: '11',
        current_waypoint: 'elevator_11_travel',
        completed_test_cases: [1, 2, 3],
        current_node: 'ElevatorTravel',
        route_index: 5,
        route_total: 36,
        last_fault: null,
        state_version: 10,
      },
    }));

    const mirroredStateMachine = await waitForMessage(
      ws,
      (msg) =>
        msg.topic === '/state_machine/status' &&
        msg.msg.containers.some((container) => container.name === 'mission_supervisor'),
    );
    const missionContainer = mirroredStateMachine.msg.containers.find(
      (container) => container.name === 'mission_supervisor',
    );
    assert.equal(missionContainer.display_name, 'Mission Supervisor');
    assert.match(missionContainer.status, /^ACTIVE\/elevator/);
    assert.match(missionContainer.status, /wp=elevator_11_travel/);
    assert.match(missionContainer.status, /tc=3\/36/);

    const mirroredDiagnostics = await waitForMessage(
      ws,
      (msg) =>
        msg.topic === '/diagnostics' &&
        msg.msg.status.some((status) => status.name === 'mission: supervisor'),
    );
    const missionDiagnostic = mirroredDiagnostics.msg.status.find(
      (status) => status.name === 'mission: supervisor',
    );
    assert.equal(missionDiagnostic.level, 0);
    assert.equal(
      missionDiagnostic.values.find((value) => value.key === 'phase')?.value,
      'elevator',
    );
    assert.equal(
      missionDiagnostic.values.find((value) => value.key === 'completed_test_cases')?.value,
      '3',
    );

    const mirroredOdom = await waitForMessage(
      ws,
      (msg) => {
        if (msg.topic !== '/odom') return false;
        const position = msg.msg.pose.pose.position;
        return (
          Math.abs(position.x - initialPosition.x) > 0.1 ||
          Math.abs(position.y - initialPosition.y) > 0.1
        );
      },
      1500,
    );
    assert.equal(typeof mirroredOdom.msg.pose.pose.position.x, 'number');
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
