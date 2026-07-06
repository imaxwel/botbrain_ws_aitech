#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PORTS="3000,13000,8787,18787,9090,19090"
GRACE_SECONDS="3"
DRY_RUN="0"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/stop-mockg1-bt-stack.sh [options]

Options:
  --ports <csv>        Listening ports to close. Default: 3000,13000,8787,18787,9090,19090
  --grace <seconds>    Seconds to wait after SIGTERM before SIGKILL. Default: 3
  --dry-run            Print matched processes without killing them.
  -h, --help           Show this help

Examples:
  ./scripts/stop-mockg1-bt-stack.sh
  ./scripts/stop-mockg1-bt-stack.sh --ports 3000,8787,9090
  ./scripts/stop-mockg1-bt-stack.sh --dry-run
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ports)
      PORTS="${2:?missing value for --ports}"
      shift 2
      ;;
    --grace)
      GRACE_SECONDS="${2:?missing value for --grace}"
      shift 2
      ;;
    --dry-run)
      DRY_RUN="1"
      shift
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

TARGET_PIDS=()
TARGET_REASONS=()
SELF_PID="$$"

require_optional_notice() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Optional command not found: $1" >&2
    return 1
  fi
}

is_number() {
  case "$1" in
    ''|*[!0-9]*) return 1 ;;
    *) return 0 ;;
  esac
}

pid_exists() {
  kill -0 "$1" >/dev/null 2>&1
}

add_pid() {
  local pid="$1"
  local reason="$2"
  local index

  is_number "$pid" || return 0
  [[ "$pid" != "$SELF_PID" ]] || return 0
  pid_exists "$pid" || return 0

  index=0
  while [[ "$index" -lt "${#TARGET_PIDS[@]}" ]]; do
    if [[ "${TARGET_PIDS[$index]}" == "$pid" ]]; then
      TARGET_REASONS[$index]="${TARGET_REASONS[$index]}; ${reason}"
      return 0
    fi
    index=$((index + 1))
  done

  TARGET_PIDS+=("$pid")
  TARGET_REASONS+=("$reason")
}

children_of() {
  local pid="$1"
  pgrep -P "$pid" 2>/dev/null || true
}

collect_pid_files() {
  local pid_file
  local pid

  while IFS= read -r pid_file; do
    [[ -s "$pid_file" ]] || continue
    pid="$(sed -n '1p' "$pid_file" | tr -d '[:space:]')"
    add_pid "$pid" "pid file ${pid_file#"$ROOT_DIR"/}"
  done < <(find "$ROOT_DIR/.run/mockg1-bt" -type f -name "*.pid" -print 2>/dev/null || true)
}

collect_matching_processes() {
  local line
  local pid
  local args

  while IFS= read -r line; do
    pid="${line%% *}"
    args="${line#* }"

    case "$args" in
      *"mission-mirror.mjs"*)
        add_pid "$pid" "mission mirror"
        ;;
      *"mockg1/src/index.js"*)
        add_pid "$pid" "mockg1 process"
        ;;
    esac
  done < <((ps -eo pid=,args= 2>/dev/null || ps -axo pid=,command=) | sed 's/^ *//')
}

collect_port_processes() {
  local port
  local pid
  local clean_port

  require_optional_notice lsof >/dev/null 2>&1 || return 0

  IFS=',' read -r -a port_list <<<"$PORTS"
  for port in "${port_list[@]}"; do
    clean_port="${port//[[:space:]]/}"
    [[ -n "$clean_port" ]] || continue
    while IFS= read -r pid; do
      add_pid "$pid" "listening port ${clean_port}"
    done < <(lsof -nP -iTCP:"$clean_port" -sTCP:LISTEN -t 2>/dev/null || true)
  done
}

print_process() {
  local pid="$1"
  ps -o pid,ppid,stat,etime,args -p "$pid" 2>/dev/null || true
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

remaining_listeners() {
  local port
  local clean_port
  local output

  if ! command -v lsof >/dev/null 2>&1; then
    echo "  lsof not available; cannot inspect remaining listeners"
    return
  fi

  IFS=',' read -r -a port_list <<<"$PORTS"
  for port in "${port_list[@]}"; do
    clean_port="${port//[[:space:]]/}"
    [[ -n "$clean_port" ]] || continue
    output="$(lsof -nP -iTCP:"$clean_port" -sTCP:LISTEN 2>/dev/null || true)"
    if [[ -n "$output" ]]; then
      echo "$output"
    fi
  done
}

if ! command -v pgrep >/dev/null 2>&1; then
  echo "Missing required command: pgrep" >&2
  exit 1
fi

collect_pid_files
collect_matching_processes
collect_port_processes

if [[ "${#TARGET_PIDS[@]}" -eq 0 ]]; then
  echo "No mockg1/BotBrain BT stack processes found."
  exit 0
fi

echo "Matched processes:"
index=0
while [[ "$index" -lt "${#TARGET_PIDS[@]}" ]]; do
  echo "  pid=${TARGET_PIDS[$index]} reason=${TARGET_REASONS[$index]}"
  print_process "${TARGET_PIDS[$index]}" | sed 's/^/    /'
  index=$((index + 1))
done

if [[ "$DRY_RUN" == "1" ]]; then
  echo "Dry run only; no processes were killed."
  exit 0
fi

echo
echo "Sending SIGTERM..."
for pid in "${TARGET_PIDS[@]}"; do
  terminate_tree "$pid"
done

sleep "$GRACE_SECONDS"

echo "Sending SIGKILL to remaining processes, if any..."
for pid in "${TARGET_PIDS[@]}"; do
  if pid_exists "$pid"; then
    kill_tree "$pid"
  fi
done

echo
echo "Remaining listeners on requested ports:"
listeners="$(remaining_listeners)"
if [[ -n "$listeners" ]]; then
  printf '%s\n' "$listeners"
else
  echo "  none"
fi
