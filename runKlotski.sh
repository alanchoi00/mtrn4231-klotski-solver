#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_IP="${1:-192.168.0.100}"

# Camera TF transform values (x y z qx qy qz qw) with defaults
CAM_TX="${2:-1.30938}"
CAM_TY="${3:-0.0206053}"
CAM_TZ="${4:-0.670571}"
CAM_QX="${5:--0.398486}"
CAM_QY="${6:-0.00254305}"
CAM_QZ="${7:-0.917119}"
CAM_QW="${8:-0.00974536}"

# helper: return 0 if a window with exactly this title exists, 1 otherwise
window_exists() {
  local title="$1"

  # prefer xdotool if available
  if command -v xdotool >/dev/null 2>&1; then
    # search by name (title). xdotool returns one or more window ids on success.
    xdotool search --name --onlyvisible --limit 1 -- "$title" >/dev/null 2>&1 && return 0 || return 1
  fi

  # fallback to wmctrl if available
  if command -v wmctrl >/dev/null 2>&1; then
    # wmctrl -l output: <winid> <desktop> <host> <title...>
    # strip the first three columns and compare the rest to the exact title
    if wmctrl -l | sed 's/^[^ ]\+ \+[^ ]\+ \+[^ ]\+ \+//' | grep -Fx -- "$title" >/dev/null 2>&1; then
      return 0
    else
      return 1
    fi
  fi

  # no reliable window tool available — assume it doesn't exist
  return 1
}

TITLE1="UR5eDriverServer"
TITLE2="Dashboard"
TITLE3="CameraTF"

if window_exists "$TITLE1"; then
  echo "Terminal with title \"$TITLE1\" already exists; skipping launch."
else
  gnome-terminal --title="$TITLE1" -- bash -lc "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=$ROBOT_IP use_fake_hardware:=false launch_rviz:=false; exec bash" &
fi

sleep 5

if window_exists "$TITLE2"; then
  echo "Terminal with title \"$TITLE2\" already exists; skipping launch."
else
  gnome-terminal --title="$TITLE2" -- bash -lc "cd \"$SCRIPT_DIR/src/dashboard_app\" && npm install && npm run dev; exec bash" &
fi

sleep 5

if window_exists "$TITLE3"; then
  echo "Terminal with title \"$TITLE3\" already exists; skipping launch."
else
  gnome-terminal --title="$TITLE3" -- bash -lc "ros2 run tf2_ros static_transform_publisher $CAM_TX $CAM_TY $CAM_TZ $CAM_QX $CAM_QY $CAM_QZ $CAM_QW base_link camera_link; exec bash" &
fi

sleep 2

source "$SCRIPT_DIR/install/setup.bash"

ros2 launch src/launch/klotski.launch.py
