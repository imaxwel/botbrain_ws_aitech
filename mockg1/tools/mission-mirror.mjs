#!/usr/bin/env node
import { WebSocket } from 'ws';

const DEFAULTS = {
  supervisorUrl: 'http://127.0.0.1:8787',
  mockUrl: 'ws://127.0.0.1:9090',
  interval: 500,
};
const TOPIC = '/mock/mission_snapshot';
const SCHEMA_VERSION = 'g1.mock.mission_mirror.v1';

const options = parseArgs(process.argv.slice(2));
let stopped = false;
let activeSocket = null;

process.on('SIGINT', stop);
process.on('SIGTERM', stop);

run().catch((error) => {
  console.error(`mission-mirror fatal: ${error.message}`);
  process.exit(1);
});

async function run() {
  let backoffMs = 500;

  while (!stopped) {
    try {
      activeSocket = await connectWebSocket(options.mockUrl);
      backoffMs = 500;
      await mirrorUntilDisconnected(activeSocket);
    } catch (error) {
      if (!stopped) {
        console.error(`mission-mirror reconnecting after error: ${error.message}`);
        await sleep(backoffMs);
        backoffMs = Math.min(backoffMs * 2, 8000);
      }
    } finally {
      if (activeSocket) {
        activeSocket.close();
        activeSocket = null;
      }
    }
  }
}

async function mirrorUntilDisconnected(ws) {
  let lastKey = '';

  ws.send(JSON.stringify({ op: 'advertise', topic: TOPIC, type: 'g1_mock/MissionMirrorSnapshot' }));

  while (!stopped && ws.readyState === WebSocket.OPEN) {
    const snapshot = await fetchSnapshot(options.supervisorUrl);
    const payload = snapshotToMissionMirrorPayload(snapshot);
    const nextKey = `${snapshot.sequence ?? ''}:${payload.state_version}`;

    if (nextKey !== lastKey) {
      ws.send(JSON.stringify({ op: 'publish', topic: TOPIC, msg: payload }));
      lastKey = nextKey;
    }

    await sleep(options.interval);
  }

  if (!stopped) {
    throw new Error('mockg1 websocket disconnected');
  }
}

async function fetchSnapshot(supervisorUrl) {
  const url = new URL('/snapshot', normalizeHttpBase(supervisorUrl));
  const response = await fetch(url, {
    headers: { Accept: 'application/json' },
    cache: 'no-store',
  });

  if (!response.ok) {
    throw new Error(`snapshot request failed with HTTP ${response.status}`);
  }

  return response.json();
}

function snapshotToMissionMirrorPayload(snapshot) {
  const blackboard = snapshot.blackboard_summary ?? {};
  const health = snapshot.health ?? {};
  const route = Array.isArray(blackboard.route) ? blackboard.route : [];
  const completedTestCases = Array.isArray(blackboard.completed_test_cases)
    ? blackboard.completed_test_cases
        .map((value) => Number(value))
        .filter((value) => Number.isInteger(value) && value > 0)
    : [];

  return {
    schema_version: SCHEMA_VERSION,
    mission_state: snapshot.mission_state ?? 'UNKNOWN',
    phase: snapshot.phase ?? 'unknown',
    current_floor: snapshot.current_floor ?? 'unknown',
    target_floor: snapshot.target_floor ?? null,
    current_waypoint: snapshot.current_waypoint ?? null,
    completed_test_cases: completedTestCases,
    current_node: blackboard.current_node ?? null,
    route_index: Number.isInteger(Number(blackboard.route_index))
      ? Number(blackboard.route_index)
      : Math.max(0, completedTestCases.length - 1),
    route_total: route.length || 36,
    last_fault: health.last_fault ?? null,
    state_version: Number.isInteger(Number(snapshot.state_version))
      ? Number(snapshot.state_version)
      : 0,
  };
}

function parseArgs(argv) {
  const parsed = { ...DEFAULTS };

  for (let index = 0; index < argv.length; index += 1) {
    const raw = argv[index];
    const [key, inlineValue] = raw.split('=', 2);

    if (key === '-h' || key === '--help') {
      printUsage();
      process.exit(0);
    }

    const value = inlineValue ?? argv[index + 1];
    if (inlineValue === undefined) index += 1;

    if (value === undefined) {
      throw new Error(`missing value for ${key}`);
    }

    if (key === '--supervisor-url') {
      parsed.supervisorUrl = value;
    } else if (key === '--mock-url') {
      parsed.mockUrl = value;
    } else if (key === '--interval') {
      parsed.interval = Math.max(100, Number(value) || DEFAULTS.interval);
    } else {
      throw new Error(`unknown option: ${key}`);
    }
  }

  return parsed;
}

function printUsage() {
  console.log(`Usage:
  node tools/mission-mirror.mjs [options]

Options:
  --supervisor-url <url>  Mission Supervisor base URL. Default: ${DEFAULTS.supervisorUrl}
  --mock-url <url>        mockg1 rosbridge WS URL. Default: ${DEFAULTS.mockUrl}
  --interval <ms>         Snapshot poll interval. Default: ${DEFAULTS.interval}
`);
}

function normalizeHttpBase(value) {
  return value.endsWith('/') ? value : `${value}/`;
}

function connectWebSocket(url) {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(url);

    ws.once('open', () => resolve(ws));
    ws.once('error', reject);
  });
}

function stop() {
  stopped = true;
  if (activeSocket) activeSocket.close();
}

function sleep(ms) {
  return new Promise((resolve) => {
    setTimeout(resolve, ms);
  });
}
