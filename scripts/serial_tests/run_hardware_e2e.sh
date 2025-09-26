#!/usr/bin/env bash
set -euo pipefail

# run_hardware_e2e.sh
# WSL-friendly orchestrator for end-to-end serial probes.
# Requirements: node (optional bridge), docker, python3

COM_PORT="/dev/ttyS11"
HOST_PORT=5000
CONTAINER_NAME=microros_agent_probe_e2e
RESULTS_DIR="/tmp/hw_e2e_results"
BRIDGE_SCRIPT="/mnt/c/serial-bridge/serial-tcp-bridge.cjs"

mkdir -p "$RESULTS_DIR"
echo "E2E runner; results -> $RESULTS_DIR"

bridge_pid_file="$RESULTS_DIR/bridge_pid.txt"

start_bridge() {
  if command -v node >/dev/null 2>&1 && [ -f "$BRIDGE_SCRIPT" ]; then
    echo "Starting Node bridge..."
    node "$BRIDGE_SCRIPT" --port $HOST_PORT --com COM11 --baud 115200 &
    echo $! > "$bridge_pid_file"
    sleep 1
  else
    echo "Node bridge not available or bridge script missing; assume TCP bridge already present on port $HOST_PORT"
  fi
}

start_container() {
  echo "Starting docker container $CONTAINER_NAME"
  docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
  docker run -d --name "$CONTAINER_NAME" --privileged --entrypoint /bin/sh microros/micro-ros-agent:humble -c 'sleep infinity'
  docker exec -u root "$CONTAINER_NAME" bash -lc 'apt-get update && apt-get install -y socat >/tmp/socat_install.log 2>&1' || true
  docker exec -u root -d "$CONTAINER_NAME" bash -lc "socat -d -d PTY,link=$COM_PORT,raw,echo=0 TCP:host.docker.internal:$HOST_PORT 2>/tmp/socat.log"
  sleep 1
  docker exec -d "$CONTAINER_NAME" bash -lc "/micro-ros_entrypoint.sh serial --dev $COM_PORT -b 115200 > /tmp/agent.log 2>&1 || true"
  # start capture
  docker exec -d "$CONTAINER_NAME" bash -lc "stdbuf -o0 cat $COM_PORT > /tmp/pty_capture.bin & echo $! > /tmp/pty_capture.pid" || true
  sleep 1
}

stop_and_collect() {
  echo "Collecting logs into $RESULTS_DIR"
  docker cp "$CONTAINER_NAME:/tmp/pty_capture.bin" "$RESULTS_DIR/pty_capture.bin" 2>/dev/null || true
  docker cp "$CONTAINER_NAME:/tmp/agent.log" "$RESULTS_DIR/agent.log" 2>/dev/null || true
  docker cp "$CONTAINER_NAME:/tmp/socat.log" "$RESULTS_DIR/socat.log" 2>/dev/null || true
  docker cp "$CONTAINER_NAME:/tmp/socat_install.log" "$RESULTS_DIR/socat_install.log" 2>/dev/null || true
  docker exec -u root "$CONTAINER_NAME" bash -lc 'if [ -f /tmp/pty_capture.pid ]; then kill $(cat /tmp/pty_capture.pid) || true; fi' || true
  docker rm -f "$CONTAINER_NAME" || true
  if [ -f "$bridge_pid_file" ]; then
    echo "Stopping bridge PID $(cat $bridge_pid_file)"
    kill "$(cat $bridge_pid_file)" 2>/dev/null || true
  fi
}

start_bridge
start_container

echo "Running e2e python runner..."
PY=$(command -v python3 || command -v python)
"$PY" "$(dirname "$0")/e2e_probe_runner.py" --host 127.0.0.1 --port $HOST_PORT --results "$RESULTS_DIR" --inter-delay 0.6
RC=$?

stop_and_collect

echo "E2E runner finished with exit code $RC. Results at $RESULTS_DIR"
exit $RC
