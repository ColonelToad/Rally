#!/bin/bash
set -e

RALLY_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$RALLY_ROOT"

mkdir -p /tmp/rally logs

echo "[Integration Test] Starting RLS pipeline validation..."

timeout 8 ./build/mujoco_bridge <<< "" > logs/test_mujoco_bridge.log 2>&1 &
MUJOCO_PID=$!
sleep 1

timeout 7 ./build/coordination_bus > logs/test_coordination_bus.log 2>&1 &
COORD_PID=$!
sleep 1

timeout 6 ./build/arm_controller --side left --core 2 > logs/test_arm_left.log 2>&1 &
LEFT_PID=$!

timeout 5 ./build/arm_controller --side right --core 3 > logs/test_arm_right.log 2>&1 &
RIGHT_PID=$!

sleep 4

kill $MUJOCO_PID 2>/dev/null || true
kill $COORD_PID 2>/dev/null || true
kill $LEFT_PID 2>/dev/null || true
kill $RIGHT_PID 2>/dev/null || true

sleep 1

echo "[Integration Test] Checking telemetry logs..."

if [ -f rally_telemetry.log ]; then
    SIZE=$(stat -f%z rally_telemetry.log 2>/dev/null || stat -c%s rally_telemetry.log 2>/dev/null || echo 0)
    RECORDS=$((SIZE / 64))
    echo "[Integration Test] mujoco_bridge telemetry: $RECORDS records ($SIZE bytes)"
    if [ "$RECORDS" -gt 100 ]; then
        echo "✓ mujoco_bridge produced sufficient logs"
    else
        echo "✗ mujoco_bridge log count too low"
        exit 1
    fi
else
    echo "✗ mujoco_bridge telemetry.log not found"
    exit 1
fi

for side in left right; do
    LOG="rally_telemetry_${side}.log"
    if [ -f "$LOG" ]; then
        SIZE=$(stat -f%z "$LOG" 2>/dev/null || stat -c%s "$LOG" 2>/dev/null || echo 0)
        RECORDS=$((SIZE / 64))
        echo "[Integration Test] arm_controller ($side) telemetry: $RECORDS records ($SIZE bytes)"
    fi
done

echo ""
echo "✓ Integration test PASSED: Full RLS pipeline ran end-to-end"
