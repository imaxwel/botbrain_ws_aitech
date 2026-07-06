#!/usr/bin/env bash

# Unified ROS environment loader for BotBrain containers.
# Source order (lower -> higher overlays):
# 1) /opt/ros/<distro>
# 2) Unitree overlay (/opt/unitree_overlay)
# 3) BotBrain workspace overlay (/botbrain_ws/install)

set -e

safe_source() {
  local setup_file="$1"
  [ -f "${setup_file}" ] || return 0

  local had_nounset=0
  case $- in
    *u*) had_nounset=1; set +u ;;
  esac

  # shellcheck disable=SC1090
  source "${setup_file}"

  if [ "${had_nounset}" -eq 1 ]; then
    set -u
  fi
}

source_ros_base() {
  local setup_file
  local setup_candidates=()

  if [ -n "${ROS_DISTRO:-}" ]; then
    setup_candidates+=("/opt/ros/${ROS_DISTRO}/setup.bash")
    setup_candidates+=("/opt/ros/${ROS_DISTRO}/install/setup.bash")
  fi

  setup_candidates+=("/opt/ros/humble/setup.bash")
  setup_candidates+=("/opt/ros/humble/install/setup.bash")
  setup_candidates+=("/opt/ros/foxy/setup.bash")
  setup_candidates+=("/opt/ros/foxy/install/setup.bash")

  for setup_file in "${setup_candidates[@]}"; do
    if [ -f "${setup_file}" ]; then
      safe_source "${setup_file}"
      return 0
    fi
  done

  echo "[ros_env] ERROR: ROS base setup not found under /opt/ros" >&2
  return 1
}

iface_exists() {
  local iface="$1"
  [ -n "${iface}" ] && [ -d "/sys/class/net/${iface}" ]
}

iface_is_virtual() {
  local iface="$1"
  case "${iface}" in
    lo|docker*|br-*|veth*|virbr*|cni*|flannel*|tailscale*|tun*|tap*|wg*|zt*)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

iface_is_wireless() {
  local iface="$1"
  [ -d "/sys/class/net/${iface}/wireless" ]
}

iface_is_up() {
  local iface="$1"
  local state
  state="$(cat "/sys/class/net/${iface}/operstate" 2>/dev/null || true)"
  [ "${state}" = "up" ]
}

iface_has_carrier() {
  local iface="$1"
  local carrier_file="/sys/class/net/${iface}/carrier"
  if [ ! -f "${carrier_file}" ]; then
    return 0
  fi
  [ "$(cat "${carrier_file}" 2>/dev/null || true)" = "1" ]
}

iface_has_global_ipv4() {
  local iface="$1"
  if command -v ip >/dev/null 2>&1; then
    ip -o -4 addr show dev "${iface}" scope global 2>/dev/null | grep -q .
    return $?
  fi
  return 0
}

iface_is_usable() {
  local iface="$1"
  iface_exists "${iface}" || return 1
  iface_is_virtual "${iface}" && return 1
  iface_is_up "${iface}" || return 1
  iface_has_carrier "${iface}" || return 1
  iface_has_global_ipv4 "${iface}" || return 1
  return 0
}

iface_is_auto_keyword() {
  local iface="$1"
  case "${iface}" in
    auto|AUTO|Auto)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

candidate_interfaces() {
  if command -v ip >/dev/null 2>&1; then
    ip -o -4 addr show scope global 2>/dev/null \
      | awk '{print $2}' \
      | sed 's/@.*//' \
      | awk '!seen[$0]++'
    return 0
  fi

  if [ -d "/sys/class/net" ]; then
    ls -1 /sys/class/net 2>/dev/null
  fi
}

read_robot_config_network_interface() {
  local config_file="/botbrain_ws/robot_config.yaml"
  local iface=""

  if [ ! -f "${config_file}" ] && [ -f "robot_config.yaml" ]; then
    config_file="robot_config.yaml"
  fi
  [ -f "${config_file}" ] || return 0

  if command -v python3 >/dev/null 2>&1; then
    iface="$(
      python3 - "${config_file}" <<'PY' 2>/dev/null || true
import sys
try:
    import yaml
except Exception:
    yaml = None

if yaml is None:
    raise SystemExit(0)

cfg_path = sys.argv[1]
try:
    with open(cfg_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    value = (data.get("robot_configuration") or {}).get("network_interface") or ""
    print(str(value).strip())
except Exception:
    pass
PY
    )"
  fi

  if [ -z "${iface}" ]; then
    iface="$(awk -F: '/^[[:space:]]*network_interface:[[:space:]]*/ {gsub(/[[:space:]"]/, "", $2); print $2; exit}' "${config_file}" 2>/dev/null || true)"
  fi

  printf '%s' "${iface}"
}

choose_auto_network_interface() {
  local iface

  # Prefer wired interfaces with carrier and IPv4.
  while IFS= read -r iface; do
    [ -n "${iface}" ] || continue
    iface_is_wireless "${iface}" && continue
    iface_is_usable "${iface}" || continue
    printf '%s' "${iface}"
    return 0
  done < <(candidate_interfaces)

  # Fall back to any usable non-virtual interface.
  while IFS= read -r iface; do
    [ -n "${iface}" ] || continue
    iface_is_usable "${iface}" || continue
    printf '%s' "${iface}"
    return 0
  done < <(candidate_interfaces)

  return 1
}

resolve_network_interface() {
  local explicit_iface="${BOTBRAIN_NETWORK_INTERFACE:-}"
  local config_iface=""
  local auto_iface=""

  if iface_is_auto_keyword "${explicit_iface}"; then
    explicit_iface=""
  fi

  if [ -n "${explicit_iface}" ]; then
    if iface_is_usable "${explicit_iface}"; then
      printf '%s' "${explicit_iface}"
      return 0
    fi
    echo "[ros_env] WARN: BOTBRAIN_NETWORK_INTERFACE=${explicit_iface} is not usable; falling back to auto detection." >&2
  fi

  config_iface="$(read_robot_config_network_interface)"
  if iface_is_auto_keyword "${config_iface}"; then
    config_iface=""
  fi
  if [ -n "${config_iface}" ] && iface_is_usable "${config_iface}"; then
    printf '%s' "${config_iface}"
    return 0
  fi

  auto_iface="$(choose_auto_network_interface || true)"
  if [ -n "${auto_iface}" ]; then
    printf '%s' "${auto_iface}"
    return 0
  fi

  return 1
}

render_cyclonedds_runtime_config() {
  local iface="$1"
  local base_config="/botbrain_ws/cyclonedds_config.xml"
  local runtime_config="/tmp/cyclonedds_config.runtime.xml"

  if [ ! -f "${base_config}" ] && [ -f "cyclonedds_config.xml" ]; then
    base_config="cyclonedds_config.xml"
  fi

  if [ -f "${base_config}" ]; then
    awk -v iface="${iface}" '
      /<NetworkInterface / && $0 !~ /name="lo"/ && !replaced {
        sub(/name="[^"]+"/, "name=\"" iface "\"")
        replaced=1
      }
      /<\/Interfaces>/ && !replaced {
        print "        <NetworkInterface name=\"" iface "\" priority=\"10\" multicast=\"true\" />"
        replaced=1
      }
      { print }
    ' "${base_config}" > "${runtime_config}"
  else
    cat > "${runtime_config}" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="lo" priority="100" multicast="true" />
        <NetworkInterface name="${iface}" priority="10" multicast="true" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
EOF
  fi

  printf '%s' "${runtime_config}"
}

configure_cyclonedds_interface() {
  local resolved_iface=""
  local runtime_config=""

  resolved_iface="$(resolve_network_interface || true)"
  if [ -z "${resolved_iface}" ]; then
    echo "[ros_env] WARN: unable to resolve a usable network interface; letting CycloneDDS auto-select." >&2
    unset CYCLONEDDS_URI
    return 0
  fi

  runtime_config="$(render_cyclonedds_runtime_config "${resolved_iface}")"
  if [ ! -f "${runtime_config}" ]; then
    echo "[ros_env] WARN: failed to render CycloneDDS runtime config; letting CycloneDDS auto-select." >&2
    unset CYCLONEDDS_URI
    return 0
  fi

  export BOTBRAIN_NETWORK_INTERFACE_RESOLVED="${resolved_iface}"
  export CYCLONEDDS_URI="file://${runtime_config}"
  echo "[ros_env] CycloneDDS interface resolved to: ${resolved_iface}" >&2
}

source_ros_base

if [ -f "/opt/unitree_overlay/setup.bash" ]; then
  # shellcheck disable=SC1091
  safe_source "/opt/unitree_overlay/setup.bash"
elif [ "${BOTBRAIN_SKIP_UNITREE_OVERLAY:-0}" = "1" ]; then
  echo "[ros_env] WARN: skipping Unitree overlay because BOTBRAIN_SKIP_UNITREE_OVERLAY=1." >&2
else
  echo "[ros_env] ERROR: /opt/unitree_overlay/setup.bash not found." >&2
  echo "[ros_env] Build base image with docker/unitree-overlay/Dockerfile first." >&2
  return 1 2>/dev/null || exit 1
fi

if [ -f "/botbrain_ws/install/setup.bash" ]; then
  safe_source "/botbrain_ws/install/setup.bash"
elif [ -f "install/setup.bash" ]; then
  safe_source "install/setup.bash"
fi

# Keep this last so later setup scripts cannot override CYCLONEDDS_URI.
configure_cyclonedds_interface
