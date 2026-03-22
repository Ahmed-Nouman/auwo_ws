#!/usr/bin/env bash
# Idempotent ros2_control spawner: skip if controller already active; unload stale instance otherwise.
# Avoids "Controller already loaded, skipping load_controller" + "Failed to configure controller" on relaunch.
set -euo pipefail
CTRL="$1"
shift

for _ in $(seq 1 120); do
  if ros2 service list 2>/dev/null | grep -q '/controller_manager/list_controllers'; then
    break
  fi
  sleep 0.25
done

line="$(ros2 control list_controllers 2>/dev/null | grep -E "^${CTRL}[[:space:]]" || true)"
if [[ -n "$line" ]] && echo "$line" | grep -q "active"; then
  echo "[spawn_controller_safe] ${CTRL} already active — skipping spawner."
  exit 0
fi
if [[ -n "$line" ]]; then
  echo "[spawn_controller_safe] ${CTRL} loaded but not active — unloading then spawning."
  ros2 control unload_controller --controller-manager /controller_manager "${CTRL}" 2>/dev/null || true
  sleep 0.4
fi
exec ros2 run controller_manager spawner "${CTRL}" "$@"
