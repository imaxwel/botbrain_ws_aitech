#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "${ROOT_DIR}/.." && pwd)"
MOCKG1_DIR="${ROOT_DIR}/mockg1"
FRONTEND_DIR="${ROOT_DIR}/frontend"
MISSION_DIR="${REPO_ROOT}/tour-guide-robot/Mission-Supervisor-BT"

SCENARIO="default"
MOCK_HOST="127.0.0.1"
MOCK_PORT="9090"
API_HOST="127.0.0.1"
API_PORT="8787"
FRONTEND_PORT="3000"
BROWSER_HOST=""
AUTO_APPROVE="1"
PROFILE="fake"
MIRROR="0"
LOG_ROOT="${ROOT_DIR}/.run/mockg1-bt"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/run-mockg1-bt-stack.sh [options]

Options:
  --scenario <name>       Mock G1 scenario: default, low_battery, emergency, unstable_network
  --mock-port <port>      Mock G1 rosbridge port. Default: 9090
  --api-port <port>       Mission Supervisor API port. Default: 8787
  --frontend-port <port>  BotBrain frontend port. Default: 3000
  --host <host>           Local service listen host for mock/API. Default: 127.0.0.1
  --browser-host <host>   Host advertised to the browser. Default: --host, or 127.0.0.1 for 0.0.0.0
  --profile <name>        Mission Supervisor profile. Default: fake
  --auto-approve <0|1>    Mission Supervisor auto approve. Default: 1
  --mirror <0|1>          Start mockg1 Mission Supervisor mirror process. Default: 0
  -h, --help              Show this help

Examples:
  ./scripts/run-mockg1-bt-stack.sh
  ./scripts/run-mockg1-bt-stack.sh --scenario low_battery
  ./scripts/run-mockg1-bt-stack.sh --scenario emergency --frontend-port 3001
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)
      SCENARIO="${2:?missing value for --scenario}"
      shift 2
      ;;
    --mock-port)
      MOCK_PORT="${2:?missing value for --mock-port}"
      shift 2
      ;;
    --api-port)
      API_PORT="${2:?missing value for --api-port}"
      shift 2
      ;;
    --frontend-port)
      FRONTEND_PORT="${2:?missing value for --frontend-port}"
      shift 2
      ;;
    --host)
      MOCK_HOST="${2:?missing value for --host}"
      API_HOST="$MOCK_HOST"
      shift 2
      ;;
    --browser-host)
      BROWSER_HOST="${2:?missing value for --browser-host}"
      shift 2
      ;;
    --profile)
      PROFILE="${2:?missing value for --profile}"
      shift 2
      ;;
    --auto-approve)
      AUTO_APPROVE="${2:?missing value for --auto-approve}"
      shift 2
      ;;
    --mirror)
      MIRROR="${2:?missing value for --mirror}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$SCENARIO" in
  default|low_battery|emergency|unstable_network) ;;
  *)
    echo "Unsupported scenario: ${SCENARIO}" >&2
    echo "Allowed: default, low_battery, emergency, unstable_network" >&2
    exit 2
    ;;
esac

case "$MIRROR" in
  0|1) ;;
  *)
    echo "Unsupported --mirror value: ${MIRROR}" >&2
    echo "Allowed: 0, 1" >&2
    exit 2
    ;;
esac

if [[ -z "$BROWSER_HOST" ]]; then
  if [[ "$MOCK_HOST" == "0.0.0.0" ]]; then
    BROWSER_HOST="127.0.0.1"
  else
    BROWSER_HOST="$MOCK_HOST"
  fi
fi

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_dir() {
  if [[ ! -d "$1" ]]; then
    echo "Missing directory: $1" >&2
    exit 1
  fi
}

require_node_modules() {
  local name="$1"
  local dir="$2"

  if [[ ! -d "${dir}/node_modules" ]]; then
    echo "Missing ${name} dependencies: ${dir}/node_modules" >&2
    echo "Install them first:" >&2
    echo "  cd ${dir} && npm ci" >&2
    exit 1
  fi
}

tcp_check() {
  local host="$1"
  local port="$2"

  node - "$host" "$port" <<'NODE'
const net = require('node:net');
const host = process.argv[2];
const port = Number(process.argv[3]);
const socket = net.createConnection({ host, port });
let settled = false;

function finish(code) {
  if (settled) return;
  settled = true;
  socket.destroy();
  process.exit(code);
}

socket.setTimeout(1000);
socket.once('connect', () => finish(0));
socket.once('timeout', () => finish(1));
socket.once('error', () => finish(1));
NODE
}

assert_tcp_free() {
  local name="$1"
  local host="$2"
  local port="$3"

  if tcp_check "$host" "$port" >/dev/null 2>&1; then
    echo "${name} port is already in use: ${host}:${port}" >&2
    echo "Stop the existing process first, or choose another port." >&2
    return 1
  fi
}

wait_tcp() {
  local name="$1"
  local host="$2"
  local port="$3"
  local attempts="${4:-60}"

  for _ in $(seq 1 "$attempts"); do
    if tcp_check "$host" "$port" >/dev/null 2>&1; then
      echo "${name} ready on ${host}:${port}"
      return 0
    fi
    sleep 1
  done

  echo "${name} did not become ready on ${host}:${port}" >&2
  return 1
}

configure_local_no_proxy() {
  local current="${no_proxy:-${NO_PROXY:-}}"
  local entry

  for entry in localhost 127.0.0.1 ::1 "$MOCK_HOST" "$API_HOST" "$BROWSER_HOST"; do
    if [[ -n "$entry" && ",${current}," != *",${entry},"* ]]; then
      current="${current:+${current},}${entry}"
    fi
  done

  export no_proxy="$current"
  export NO_PROXY="$current"
}

children_of() {
  local pid="$1"
  pgrep -P "$pid" 2>/dev/null || true
}

terminate_tree() {
  local pid="$1"
  local child

  for child in $(children_of "$pid"); do
    terminate_tree "$child"
  done

  kill "$pid" >/dev/null 2>&1 || true
}

kill_tree() {
  local pid="$1"
  local child

  for child in $(children_of "$pid"); do
    kill_tree "$child"
  done

  kill -KILL "$pid" >/dev/null 2>&1 || true
}

require_cmd npm
require_cmd node
require_cmd uv
require_cmd pgrep
require_dir "$MOCKG1_DIR"
require_dir "$FRONTEND_DIR"
require_dir "$MISSION_DIR"
require_node_modules "mockg1" "$MOCKG1_DIR"
require_node_modules "frontend" "$FRONTEND_DIR"
configure_local_no_proxy

RUN_ID="$(date +%Y%m%d-%H%M%S)-${SCENARIO}"
LOG_DIR="${STACK_RUN_DIR:-${LOG_ROOT}/${RUN_ID}}"
mkdir -p "$LOG_DIR"

PIDS=()
CLEANED_UP=0

cleanup() {
  local code=$?
  local pid

  if [[ "$CLEANED_UP" -eq 1 ]]; then
    exit "$code"
  fi
  CLEANED_UP=1

  if [[ ${#PIDS[@]} -gt 0 ]]; then
    echo
    echo "Stopping services..."
    for pid in "${PIDS[@]}"; do
      terminate_tree "$pid"
    done
    sleep 1
    for pid in "${PIDS[@]}"; do
      if kill -0 "$pid" >/dev/null 2>&1; then
        kill_tree "$pid"
      fi
    done
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
  echo "Logs: $LOG_DIR"
  exit "$code"
}

trap cleanup EXIT INT TERM

start_service() {
  local name="$1"
  local dir="$2"
  local log_file="$3"
  local pid_file="${LOG_DIR}/${name}.pid"
  local pid
  shift 3

  rm -f "$pid_file"
  (
    cd "$dir"
    exec "$@"
  ) >"$log_file" 2>&1 &
  pid=$!
  echo "$pid" >"$pid_file"
  PIDS+=("$pid")
  echo "Started ${name} pid=${pid} log=${log_file}"
}

monitor_services() {
  local pid

  while true; do
    for pid in "${PIDS[@]}"; do
      if ! kill -0 "$pid" >/dev/null 2>&1; then
        echo "A service exited; stopping the stack." >&2
        exit 1
      fi
    done
    sleep 1
  done
}

assert_tcp_free "mockg1" "$MOCK_HOST" "$MOCK_PORT"
assert_tcp_free "mission-supervisor" "$API_HOST" "$API_PORT"
assert_tcp_free "botbrain-frontend" "127.0.0.1" "$FRONTEND_PORT"

echo "Scenario: ${SCENARIO}"
echo "Mock G1: ws://${BROWSER_HOST}:${MOCK_PORT} (listen ${MOCK_HOST}:${MOCK_PORT})"
echo "Mission Supervisor: http://${BROWSER_HOST}:${API_PORT} (listen ${API_HOST}:${API_PORT})"
echo "BotBrain frontend: http://localhost:${FRONTEND_PORT}"
echo "Mission Supervisor profile: ${PROFILE}"
echo "Mission Supervisor auto approve: ${AUTO_APPROVE}"
echo "Mission mirror: ${MIRROR}"
echo "Logs: ${LOG_DIR}"
echo

start_service \
  "mockg1" \
  "$MOCKG1_DIR" \
  "${LOG_DIR}/mockg1.log" \
  env MOCKG1_SCENARIO="$SCENARIO" npm run dev -- --host "$MOCK_HOST" --port "$MOCK_PORT" --scenario "$SCENARIO"
wait_tcp "mockg1" "$MOCK_HOST" "$MOCK_PORT"

start_service \
  "mission-supervisor" \
  "$MISSION_DIR" \
  "${LOG_DIR}/mission-supervisor.log" \
  env MISSION_SUPERVISOR_PROFILE="$PROFILE" MISSION_SUPERVISOR_AUTO_APPROVE="$AUTO_APPROVE" \
    MISSION_SUPERVISOR_LOG_DIR="${LOG_DIR}/mission-supervisor" \
    uv run uvicorn mission_supervisor.api:app --host "$API_HOST" --port "$API_PORT"
wait_tcp "mission-supervisor" "$API_HOST" "$API_PORT"

if [[ "$MIRROR" == "1" ]]; then
  start_service \
    "mission-mirror" \
    "$MOCKG1_DIR" \
    "${LOG_DIR}/mission-mirror.log" \
    node tools/mission-mirror.mjs \
      --supervisor-url "http://${BROWSER_HOST}:${API_PORT}" \
      --mock-url "ws://${BROWSER_HOST}:${MOCK_PORT}" \
      --interval 500
fi

start_service \
  "botbrain-frontend" \
  "$FRONTEND_DIR" \
  "${LOG_DIR}/frontend.log" \
  env MISSION_SUPERVISOR_URL="http://${BROWSER_HOST}:${API_PORT}" \
    NEXT_PUBLIC_MISSION_SUPERVISOR_URL="http://${BROWSER_HOST}:${API_PORT}" \
    NEXT_PUBLIC_ROS_IP="$BROWSER_HOST" \
    NEXT_PUBLIC_ROS_PORT="$MOCK_PORT" \
    npm run dev -- -p "$FRONTEND_PORT"
wait_tcp "botbrain-frontend" "127.0.0.1" "$FRONTEND_PORT" 90

echo
echo "All services are running."
echo "Open after login: http://localhost:${FRONTEND_PORT}/mission-control"
echo "Health after login: http://localhost:${FRONTEND_PORT}/health"
echo "Connect BotBrain robot address to: ws://${BROWSER_HOST}:${MOCK_PORT}"
echo "Mission Supervisor UI: http://${BROWSER_HOST}:${API_PORT}/ui"
echo
echo "Tail logs with:"
echo "  tail -f ${LOG_DIR}/*.log"
echo
echo "Press Ctrl+C to stop all services."

monitor_services
