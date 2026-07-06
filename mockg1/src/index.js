#!/usr/bin/env node
import { G1Simulator } from './g1-simulator.js';
import { RosbridgeMockServer } from './rosbridge.js';
import { getScenarioConfig, listScenarioNames } from './scenarios.js';

let args;
try {
  args = parseArgs(process.argv.slice(2));
} catch (error) {
  console.error(`[mockg1:error] ${error.message}`);
  console.error('Run with --help for usage.');
  process.exit(1);
}

if (args.help) {
  printHelp();
  process.exit(0);
}

const logLevel = args.logLevel ?? process.env.MOCKG1_LOG_LEVEL ?? 'info';
const logger = createLogger(logLevel);
const scenarioName = args.scenario ?? process.env.MOCKG1_SCENARIO ?? 'default';
const scenario = getScenarioConfig(scenarioName);

if (scenario.name !== scenarioName) {
  logger.warn(`unknown scenario "${scenarioName}", falling back to "${scenario.name}"`);
}

const host = args.host ?? process.env.MOCKG1_HOST ?? '0.0.0.0';
const port = Number(args.port ?? process.env.MOCKG1_PORT ?? 9090);
const namespace = args.namespace ?? process.env.MOCKG1_NAMESPACE ?? '';
const cameraFps = Number(args.cameraFps ?? process.env.MOCKG1_CAMERA_FPS ?? 5);

const simulator = new G1Simulator({ scenario, namespace, logger });
const server = new RosbridgeMockServer({ host, port, namespace, cameraFps, simulator, logger });

process.on('SIGINT', () => shutdown('SIGINT'));
process.on('SIGTERM', () => shutdown('SIGTERM'));

server.start().catch((error) => {
  logger.error(error.stack || error.message);
  process.exit(1);
});

async function shutdown(signal) {
  logger.info(`received ${signal}, shutting down mockg1`);
  await server.stop();
  process.exit(0);
}

function parseArgs(argv) {
  const parsed = {};

  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];
    if (arg === '--help' || arg === '-h') parsed.help = true;
    else if (arg === '--host') parsed.host = argv[++index];
    else if (arg === '--port') parsed.port = argv[++index];
    else if (arg === '--scenario') parsed.scenario = argv[++index];
    else if (arg === '--namespace') parsed.namespace = argv[++index];
    else if (arg === '--camera-fps') parsed.cameraFps = Number(argv[++index]);
    else if (arg === '--log-level') parsed.logLevel = argv[++index];
    else if (arg.startsWith('--host=')) parsed.host = arg.slice('--host='.length);
    else if (arg.startsWith('--port=')) parsed.port = arg.slice('--port='.length);
    else if (arg.startsWith('--scenario=')) parsed.scenario = arg.slice('--scenario='.length);
    else if (arg.startsWith('--namespace=')) parsed.namespace = arg.slice('--namespace='.length);
    else if (arg.startsWith('--camera-fps=')) parsed.cameraFps = Number(arg.slice('--camera-fps='.length));
    else if (arg.startsWith('--log-level=')) parsed.logLevel = arg.slice('--log-level='.length);
    else throw new Error(`Unknown argument: ${arg}`);
  }

  if (Number.isFinite(parsed.cameraFps) && parsed.cameraFps > 0) {
    process.env.MOCKG1_CAMERA_FPS = String(parsed.cameraFps);
  }

  return parsed;
}

function printHelp() {
  console.log(`BotBrain Mock G1

Usage:
  npm start -- [options]
  node src/index.js [options]

Options:
  --host <host>            Listen host. Default: MOCKG1_HOST or 0.0.0.0
  --port <port>            Listen port. Default: MOCKG1_PORT or 9090
  --scenario <name>        Scenario: ${listScenarioNames().join(', ')}
  --namespace <name>       Also accept /<namespace>/topic aliases
  --camera-fps <number>    Reserved for camera stream tuning. Default: 5
  --log-level <level>      error, warn, info, debug, silent
  --help                   Show this help
`);
}

function createLogger(level) {
  const levels = { silent: 0, error: 1, warn: 2, info: 3, debug: 4 };
  const active = levels[level] ?? levels.info;

  return {
    error: (...args) => active >= levels.error && console.error('[mockg1:error]', ...args),
    warn: (...args) => active >= levels.warn && console.warn('[mockg1:warn]', ...args),
    info: (...args) => active >= levels.info && console.log('[mockg1:info]', ...args),
    debug: (...args) => active >= levels.debug && console.log('[mockg1:debug]', ...args),
  };
}
