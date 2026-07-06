import { WebSocket, WebSocketServer } from 'ws';

const BASE_PERIODIC_TOPICS = [
  ['/battery', 500],
  ['/imu_temp', 500],
  ['/odom', 100],
  ['/joint_states', 50],
  ['/scan', 200],
  ['/map', 1000],
  ['/lf/sportmodestate', 100],
  ['/robot_status', 500],
  ['/diagnostics', 1000],
  ['/diagnostic_stats', 1000],
  ['/state_machine/status', 1000],
  ['/listener', 1000],
];

export class RosbridgeMockServer {
  constructor({
    host = '0.0.0.0',
    port = 9090,
    namespace = '',
    cameraFps = 5,
    simulator,
    logger = console,
  } = {}) {
    this.host = host;
    this.port = Number(port);
    this.namespace = normalizeNamespace(namespace);
    this.cameraFps = Math.max(1, Number(cameraFps) || 5);
    this.simulator = simulator;
    this.logger = logger;
    this.wss = null;
    this.clientState = new WeakMap();
    this.topicTimers = [];
    this.tickTimer = null;
    this.networkTimer = null;
  }

  async start() {
    if (this.wss) return this;

    this.wss = new WebSocketServer({ host: this.host, port: this.port });
    this.wss.on('connection', (ws, request) => this.handleConnection(ws, request));

    await new Promise((resolve, reject) => {
      this.wss.once('listening', resolve);
      this.wss.once('error', reject);
    });

    this.tickTimer = setInterval(() => this.simulator.update(), 50);
    for (const [topic, periodMs] of this.getPeriodicTopics()) {
      this.topicTimers.push(setInterval(() => this.publishTopic(topic), periodMs));
    }

    if (this.simulator.scenario.networkDropEveryMs > 0) {
      this.networkTimer = setInterval(
        () => this.dropOneClientForNetworkScenario(),
        this.simulator.scenario.networkDropEveryMs,
      );
    }

    this.logger.info?.(`mockg1 listening on ${this.url}`);
    return this;
  }

  async stop() {
    for (const timer of this.topicTimers) clearInterval(timer);
    this.topicTimers = [];
    if (this.tickTimer) clearInterval(this.tickTimer);
    if (this.networkTimer) clearInterval(this.networkTimer);
    this.tickTimer = null;
    this.networkTimer = null;

    if (!this.wss) return;

    for (const client of this.wss.clients) {
      client.close();
    }

    await new Promise((resolve) => this.wss.close(resolve));
    this.wss = null;
  }

  get address() {
    return this.wss?.address();
  }

  get url() {
    const address = this.address;
    const port = typeof address === 'object' && address ? address.port : this.port;
    const host = this.host === '0.0.0.0' ? 'localhost' : this.host;
    return `ws://${host}:${port}`;
  }

  handleConnection(ws, request) {
    this.clientState.set(ws, {
      subscriptions: new Map(),
      advertised: new Set(),
      remoteAddress: request.socket.remoteAddress,
    });

    ws.on('message', (data) => this.handleRawMessage(ws, data));
    ws.on('error', (error) => this.logger.warn?.(`websocket error: ${error.message}`));
  }

  getPeriodicTopics() {
    const cameraPeriodMs = Math.round(1000 / this.cameraFps);
    return [
      ...BASE_PERIODIC_TOPICS,
      ['/compressed_camera', cameraPeriodMs],
      ['/compressed_back_camera', cameraPeriodMs],
      ['/viz/camcam/rgb/compressed_image', cameraPeriodMs],
      ['/viz/camcam/thermal/compressed_image', cameraPeriodMs],
    ];
  }

  handleRawMessage(ws, data) {
    let message;
    try {
      message = JSON.parse(data.toString());
    } catch (error) {
      this.sendStatus(ws, 'error', undefined, `Invalid JSON frame: ${error.message}`);
      return;
    }

    this.handleMessage(ws, message);
  }

  handleMessage(ws, message) {
    const state = this.clientState.get(ws);
    if (!state || !message || typeof message.op !== 'string') return;

    switch (message.op) {
      case 'subscribe':
        this.handleSubscribe(ws, state, message);
        break;
      case 'unsubscribe':
        this.handleUnsubscribe(state, message);
        break;
      case 'advertise':
        state.advertised.add(canonicalizeTopic(message.topic, this.namespace));
        break;
      case 'unadvertise':
        state.advertised.delete(canonicalizeTopic(message.topic, this.namespace));
        break;
      case 'publish':
        this.handlePublish(message);
        break;
      case 'call_service':
        this.handleCallService(ws, message);
        break;
      case 'set_level':
        this.sendStatus(ws, 'info', message.id, 'Log level accepted');
        break;
      default:
        this.sendStatus(ws, 'warning', message.id, `Unsupported rosbridge op: ${message.op}`);
        break;
    }
  }

  handleSubscribe(ws, state, message) {
    if (!message.topic) {
      this.sendStatus(ws, 'error', message.id, 'subscribe requires a topic');
      return;
    }

    const requestedTopic = ensureLeadingSlash(message.topic);
    const canonicalTopic = canonicalizeTopic(requestedTopic, this.namespace);
    state.subscriptions.set(subscriptionKey(message), {
      requestedTopic,
      canonicalTopic,
      type: message.type,
      compression: message.compression,
    });

    this.publishTopicToClient(ws, canonicalTopic);
  }

  handleUnsubscribe(state, message) {
    if (message.id) state.subscriptions.delete(message.id);
    if (message.topic) {
      const requestedTopic = ensureLeadingSlash(message.topic);
      for (const [key, subscription] of state.subscriptions.entries()) {
        if (subscription.requestedTopic === requestedTopic) state.subscriptions.delete(key);
      }
    }
  }

  handlePublish(message) {
    if (!message.topic) return;
    const topic = canonicalizeTopic(message.topic, this.namespace);
    const result = this.simulator.handlePublish(topic, message.msg ?? {});
    this.logger.debug?.(`publish ${topic}: ${result.message}`);

    for (const publishTopic of result.publishTopics ?? []) {
      this.publishTopic(publishTopic);
    }
  }

  handleCallService(ws, message) {
    const service = canonicalizeTopic(message.service, this.namespace);
    const response = this.simulator.callService(service, message.args ?? {});

    this.send(ws, {
      op: 'service_response',
      id: message.id,
      service: message.service,
      values: response.values ?? {},
      result: response.result !== false,
    });

    if (service === '/emergency_stop' || service === '/mode') {
      this.publishTopic('/robot_status');
      this.publishTopic('/lf/sportmodestate');
    }
    if (service === '/state_machine/command') this.publishTopic('/state_machine/status');
  }

  publishTopic(canonicalTopic) {
    if (!this.wss) return;
    for (const client of this.wss.clients) {
      this.publishTopicToClient(client, canonicalTopic);
    }
  }

  publishTopicToClient(ws, canonicalTopic) {
    if (ws.readyState !== WebSocket.OPEN) return;
    const state = this.clientState.get(ws);
    if (!state) return;

    for (const subscription of state.subscriptions.values()) {
      if (subscription.canonicalTopic !== canonicalTopic) continue;
      const msg = this.simulator.getMessage(canonicalTopic);
      if (!msg) continue;
      this.send(ws, {
        op: 'publish',
        topic: subscription.requestedTopic,
        msg,
      });
    }
  }

  sendStatus(ws, level, id, msg) {
    this.send(ws, {
      op: 'status',
      id,
      level,
      msg,
    });
  }

  send(ws, message) {
    if (ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify(message));
  }

  dropOneClientForNetworkScenario() {
    if (!this.wss) return;
    const [client] = this.wss.clients;
    if (client) client.close(1011, 'mock unstable_network scenario');
  }
}

function subscriptionKey(message) {
  return message.id || message.topic;
}

export function canonicalizeTopic(topic, namespace = '') {
  const cleanTopic = ensureLeadingSlash(topic);
  const cleanNamespace = normalizeNamespace(namespace);

  if (!cleanNamespace) return cleanTopic;

  const namespacedPrefix = `/${cleanNamespace}`;
  if (cleanTopic === namespacedPrefix) return '/';
  if (cleanTopic.startsWith(`${namespacedPrefix}/`)) {
    return cleanTopic.slice(namespacedPrefix.length) || '/';
  }

  return cleanTopic;
}

function ensureLeadingSlash(value = '') {
  const text = String(value);
  return text.startsWith('/') ? text : `/${text}`;
}

function normalizeNamespace(value = '') {
  return String(value).replace(/^\/+|\/+$/g, '');
}
