#!/usr/bin/env bash

WORKSPACE="${HOME}/Quadcopter/nmpc_px4_ros2_ws"
SERIAL_DEVICE="${1:-/dev/ttyAMA0}"

echo "========================================"
echo "NMPC hardware preflight check"
echo "========================================"
echo "Workspace: ${WORKSPACE}"
echo "PX4 ROS2 serial device: ${SERIAL_DEVICE}"
echo ""
echo "This script is read-only. It does not arm, switch modes, or send motor commands."
echo ""

failures=0
warnings=0

check_file() {
  local path="$1"
  local label="$2"
  if [ -e "$path" ]; then
    echo "[OK] ${label}: ${path}"
  else
    echo "[FAIL] ${label} missing: ${path}"
    failures=$((failures + 1))
  fi
}

check_command() {
  local cmd="$1"
  local label="$2"
  if command -v "$cmd" >/dev/null 2>&1; then
    echo "[OK] ${label}: $(command -v "$cmd")"
  else
    echo "[FAIL] ${label} not found: ${cmd}"
    failures=$((failures + 1))
  fi
}

check_file "${WORKSPACE}/install/setup.bash" "ROS2 workspace setup"
check_file "${SERIAL_DEVICE}" "Pixhawk serial device"
check_command ros2 "ROS2 CLI"
check_command MicroXRCEAgent "Micro XRCE-DDS Agent"

if [ -e "${SERIAL_DEVICE}" ] && [ ! -r "${SERIAL_DEVICE}" ]; then
  echo "[WARN] Serial device is not readable by this user. Check dialout permissions."
  warnings=$((warnings + 1))
fi

if [ -e "${SERIAL_DEVICE}" ] && [ ! -w "${SERIAL_DEVICE}" ]; then
  echo "[WARN] Serial device is not writable by this user. Check dialout permissions."
  warnings=$((warnings + 1))
fi

if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  # shellcheck source=/dev/null
  source "${WORKSPACE}/install/setup.bash"

  if ros2 pkg executables nmpc_px4_ros2 2>/dev/null | grep -q "nmpc_flight_mode"; then
    echo "[OK] NMPC executable installed: nmpc_px4_ros2 nmpc_flight_mode"
  else
    echo "[FAIL] NMPC executable not found. Rebuild nmpc_px4_ros2_ws."
    failures=$((failures + 1))
  fi

  if ros2 pkg executables nmpc_px4_ros2_utils 2>/dev/null | grep -q "ref_traj_pub_node"; then
    echo "[OK] Reference trajectory publisher installed"
  else
    echo "[FAIL] Reference trajectory publisher not found. Rebuild nmpc_px4_ros2_ws."
    failures=$((failures + 1))
  fi

  echo ""
  echo "ROS2 topic check:"
  if timeout 3 ros2 topic list >/tmp/nmpc_preflight_topics.txt 2>/dev/null; then
    if grep -q "/fmu/out/vehicle_status" /tmp/nmpc_preflight_topics.txt; then
      echo "[OK] PX4 vehicle_status topic visible"
    else
      echo "[WARN] /fmu/out/vehicle_status not visible. Start MicroXRCEAgent and confirm PX4 DDS bridge."
      warnings=$((warnings + 1))
    fi

    if grep -q "/fmu/out/vehicle_odometry" /tmp/nmpc_preflight_topics.txt; then
      echo "[OK] PX4 vehicle_odometry topic visible"
    else
      echo "[WARN] /fmu/out/vehicle_odometry not visible. NMPC cannot run without odometry."
      warnings=$((warnings + 1))
    fi
  else
    echo "[WARN] Could not query ROS2 topics. Is ROS_DOMAIN_ID correct and is the agent running?"
    warnings=$((warnings + 1))
  fi
fi

echo ""
echo "Physical safety checklist:"
echo "[ ] Propellers removed for bench testing"
echo "[ ] Frame secured"
echo "[ ] RC transmitter powered, bound, and configured for fallback"
echo "[ ] Pixhawk safety switch connected"
echo "[ ] QGroundControl connected and able to disarm"
echo "[ ] Battery can be disconnected immediately"
echo ""
echo "Safety switch rule:"
echo "- Keep the safety switch locked while checking topics and launch files."
echo "- Press the safety switch only for the actuator-output phase."
echo "- In /fmu/out/vehicle_status, safety_off=true means outputs are enabled."
echo ""

if [ "$failures" -gt 0 ]; then
  echo "Preflight result: FAIL (${failures} failure(s), ${warnings} warning(s))"
  exit 1
fi

echo "Preflight result: PASS with ${warnings} warning(s)"
exit 0
