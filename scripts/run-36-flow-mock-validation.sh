#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "${ROOT_DIR}/.." && pwd)"
MISSION_DIR="${REPO_ROOT}/tour-guide-robot/Mission-Supervisor-BT"
STACK_SCRIPT="${ROOT_DIR}/scripts/run-mockg1-bt-stack.sh"
STOP_SCRIPT="${ROOT_DIR}/scripts/stop-mockg1-bt-stack.sh"

SCENARIO="tc36_full_fake_happy_path"
MOCK_SCENARIO="default"
MOCK_HOST="127.0.0.1"
API_HOST="127.0.0.1"
MOCK_PORT="9090"
API_PORT="8787"
FRONTEND_PORT="3000"
BROWSER_HOST=""
PROFILE="fake"
AUTO_APPROVE="1"
MIRROR="0"
PACE="auto"
STEP_DELAY="2.0"
KEEP_STACK="0"
FAIL_FAST="0"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/run-36-flow-mock-validation.sh [options]

Options:
  --scenario <name|path>   Scenario name under Mission-Supervisor-BT/tests/scenarios or a YAML path.
                           Default: tc36_full_fake_happy_path
  --mock-scenario <name>   Mock G1 scenario. Default: default
  --profile <name>         Mission Supervisor profile. Default: fake
  --auto-approve <0|1>     Mission Supervisor auto approve. Default: 1
  --mirror <0|1>           Start mockg1 Mission Supervisor mirror process. Default: 0
  --pace <auto|step|fast>  Runner pace. Default: auto
  --step-delay <seconds>   Delay between runner steps in auto pace. Default: 2.0
  --mock-port <port>       Mock G1 rosbridge port. Default: 9090
  --api-port <port>        Mission Supervisor API port. Default: 8787
  --frontend-port <port>   BotBrain frontend port. Default: 3000
  --host <host>            Local mock/API listen host. Default: 127.0.0.1
  --browser-host <host>    Host advertised to browser/clients. Default: --host, or 127.0.0.1 for 0.0.0.0
  --fail-fast              Stop runner on first failing scenario/step.
  --keep-stack <0|1>       Keep services running after validation. Default: 0
  -h, --help               Show this help

Examples:
  ./scripts/run-36-flow-mock-validation.sh
  ./scripts/run-36-flow-mock-validation.sh --pace fast --keep-stack 0
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)
      SCENARIO="${2:?missing value for --scenario}"
      shift 2
      ;;
    --mock-scenario)
      MOCK_SCENARIO="${2:?missing value for --mock-scenario}"
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
    --pace)
      PACE="${2:?missing value for --pace}"
      shift 2
      ;;
    --step-delay)
      STEP_DELAY="${2:?missing value for --step-delay}"
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
    --fail-fast)
      FAIL_FAST="1"
      shift
      ;;
    --keep-stack)
      KEEP_STACK="${2:?missing value for --keep-stack}"
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

case "$PACE" in
  auto|step|fast) ;;
  *)
    echo "Unsupported pace: ${PACE}" >&2
    exit 2
    ;;
esac

case "$MIRROR" in
  0|1) ;;
  *)
    echo "Unsupported --mirror value: ${MIRROR}" >&2
    exit 2
    ;;
esac

case "$KEEP_STACK" in
  0|1) ;;
  *)
    echo "Unsupported --keep-stack value: ${KEEP_STACK}" >&2
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
    echo "Stop the existing stack first, or choose another port." >&2
    return 1
  fi
}

wait_http() {
  local name="$1"
  local url="$2"
  local attempts="${3:-90}"

  for _ in $(seq 1 "$attempts"); do
    if [[ -n "$STACK_PID" ]] && ! kill -0 "$STACK_PID" >/dev/null 2>&1; then
      echo "Stack process exited while waiting for ${name}; see ${RUN_DIR}/stack.log" >&2
      tail -n 80 "${RUN_DIR}/stack.log" >&2 || true
      return 1
    fi

    if curl --noproxy "*" -fsS "$url" >/dev/null 2>&1; then
      echo "${name} ready: ${url}"
      return 0
    fi
    sleep 1
  done

  echo "${name} did not become ready: ${url}" >&2
  tail -n 80 "${RUN_DIR}/stack.log" >&2 || true
  return 1
}

resolve_scenario() {
  local value="$1"
  if [[ "$value" == *.yaml || "$value" == */* ]]; then
    printf '%s\n' "$value"
    return
  fi
  printf '%s/tests/scenarios/%s.yaml\n' "$MISSION_DIR" "$value"
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

require_cmd curl
require_cmd uv
require_cmd npm
require_cmd node
configure_local_no_proxy

if [[ ! -x "$STACK_SCRIPT" ]]; then
  echo "Missing executable stack script: ${STACK_SCRIPT}" >&2
  exit 1
fi

if [[ ! -d "$MISSION_DIR" ]]; then
  echo "Missing Mission Supervisor directory: ${MISSION_DIR}" >&2
  exit 1
fi

SCENARIO_PATH="$(resolve_scenario "$SCENARIO")"
if [[ ! -f "$SCENARIO_PATH" ]]; then
  echo "Missing scenario: ${SCENARIO_PATH}" >&2
  exit 2
fi

RUN_ID="run-$(date +%Y%m%d-%H%M%S)"
RUN_DIR="${ROOT_DIR}/.run/mockg1-bt/${RUN_ID}"
RUNNER_DIR="${RUN_DIR}/runner"
mkdir -p "$RUNNER_DIR"

assert_tcp_free "Mock G1" "$MOCK_HOST" "$MOCK_PORT"
assert_tcp_free "Mission Supervisor" "$API_HOST" "$API_PORT"
assert_tcp_free "BotBrain frontend" "127.0.0.1" "$FRONTEND_PORT"

STACK_PID=""
CLEANED_UP=0

cleanup() {
  local code=$?
  if [[ "$CLEANED_UP" -eq 1 ]]; then
    exit "$code"
  fi
  CLEANED_UP=1

  if [[ -n "$STACK_PID" && "$KEEP_STACK" != "1" ]]; then
    echo "Stopping validation stack pid=${STACK_PID}..."
    kill "$STACK_PID" >/dev/null 2>&1 || true
    wait "$STACK_PID" 2>/dev/null || true
    if [[ -x "$STOP_SCRIPT" ]]; then
      "$STOP_SCRIPT" --ports "${FRONTEND_PORT},${API_PORT},${MOCK_PORT}" >/dev/null 2>&1 || true
    fi
  elif [[ -n "$STACK_PID" && "$KEEP_STACK" == "1" ]]; then
    echo "Stack left running with wrapper pid=${STACK_PID}."
  fi

  echo "Run directory: ${RUN_DIR}"
  exit "$code"
}

trap cleanup EXIT INT TERM

echo "Run ID: ${RUN_ID}"
echo "Scenario: ${SCENARIO_PATH}"
echo "Mission mirror: ${MIRROR}"
echo "Run directory: ${RUN_DIR}"
echo

STACK_RUN_DIR="$RUN_DIR" "$STACK_SCRIPT" \
  --scenario "$MOCK_SCENARIO" \
  --mock-port "$MOCK_PORT" \
  --api-port "$API_PORT" \
  --frontend-port "$FRONTEND_PORT" \
  --host "$MOCK_HOST" \
  --browser-host "$BROWSER_HOST" \
  --profile "$PROFILE" \
  --auto-approve "$AUTO_APPROVE" \
  --mirror "$MIRROR" \
  >"${RUN_DIR}/stack.log" 2>&1 &
STACK_PID=$!

wait_http "Mission Supervisor" "http://${BROWSER_HOST}:${API_PORT}/healthz"
wait_http "Mission Supervisor preflight" "http://${BROWSER_HOST}:${API_PORT}/preflight"
wait_http "BotBrain frontend" "http://127.0.0.1:${FRONTEND_PORT}"

RUNNER_ARGS=(
  "$SCENARIO_PATH"
  --base-url "http://${BROWSER_HOST}:${API_PORT}"
  --pace "$PACE"
  --step-delay "$STEP_DELAY"
  --report-dir "$RUNNER_DIR"
  --run-id "$RUN_ID"
)
if [[ "$FAIL_FAST" == "1" ]]; then
  RUNNER_ARGS+=(--fail-fast)
fi

(
  cd "$MISSION_DIR"
  PYTHONUNBUFFERED=1 uv run --locked python tools/mission_scenario_runner.py "${RUNNER_ARGS[@]}"
)

echo
echo "Validation completed."
echo "Mission Control: http://localhost:${FRONTEND_PORT}/mission-control"
echo "Mission Supervisor UI: http://${BROWSER_HOST}:${API_PORT}/ui"
echo "Mock G1 rosbridge: ws://${BROWSER_HOST}:${MOCK_PORT}"
echo "Summary: ${RUNNER_DIR}/summary.md"
