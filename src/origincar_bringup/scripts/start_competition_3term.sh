#!/usr/bin/env bash
set -euo pipefail

workspace="${ORIGINCAR_WS:-$HOME/dev_ws}"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
overlay_setup="$workspace/install/setup.bash"

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS setup not found: $ros_setup" >&2
  exit 1
fi
if [[ ! -f "$overlay_setup" ]]; then
  echo "Workspace is not built: $overlay_setup" >&2
  exit 1
fi

terminal_command() {
  local command="$1"
  printf 'source %q; source %q; %s' "$ros_setup" "$overlay_setup" "$command"
}

open_terminal() {
  local title="$1"
  local command="$2"
  local shell_command
  shell_command="$(terminal_command "$command")"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- bash -lc "$shell_command; exec bash"
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "$title" -e bash -lc "$shell_command; exec bash" &
  else
    echo "No supported terminal emulator found (gnome-terminal or xterm)." >&2
    echo "Run these commands in three terminals:" >&2
    echo "[Terminal 1] $(terminal_command 'ros2 launch origincar_bringup task.launch.py')" >&2
    echo "[Terminal 2] $(terminal_command 'ros2 launch origincar_bringup vehicle_stack.launch.py')" >&2
    echo "[Terminal 3] $(terminal_command 'ros2 launch connect_to_pc car_pc_bridge.launch.py')" >&2
    exit 1
  fi
}

# Start dependencies first. TaskMaster waits for a stable map TF, but opening
# the stack first makes startup logs easier to read during field operation.
open_terminal "OriginCar 2 - Vehicle Stack" \
  "ros2 launch origincar_bringup vehicle_stack.launch.py"
open_terminal "OriginCar 3 - Connect to PC" \
  "ros2 launch connect_to_pc car_pc_bridge.launch.py"
open_terminal "OriginCar 1 - TaskMaster" \
  "ros2 launch origincar_bringup task.launch.py"

echo "Opened three OriginCar terminals."
