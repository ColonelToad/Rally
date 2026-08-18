#!/bin/bash
set -e

RALLY_ROOT=
cd ""

mkdir -p /tmp/rally

echo "[Integration Test] Starting RLS pipeline validation..."

timeout 8 ./build/mujoco_bridge <<< "" > /tmp/test_mujoco_bridge.log 2>&1 &
MUJOCO_PID=
sleep 1

timeout 7 ./build/coordination_bus > /tmp/test_coordination_bus.log 2>&1 &
COORD_PID=
sleep 1

timeout 6 ./build/arm_controller --side left --core 2 > /tmp/test_arm_left.log 2>&1 &
LEFT_PID=

timeout 5 ./build/arm_controller --side right --core 3 > /tmp/test_arm_right.log 2>&1 &
RIGHT_PID=

sleep 4

kill  2>/dev/null || true
kill  2>/dev/null || true
kill  2>/dev/null || true
kill  2>/dev/null || true

sleep 1

echo "[Integration Test] Checking telemetry logs..."

if [ -f rally_telemetry.log ]; then
    SIZE=75968
    RECORDS=0
    echo "[Integration Test] mujoco_bridge telemetry:  records ( bytes)"
    if [ "" -gt 100 ]; then
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
    LOG="rally_telemetry_.log"
    if [ -f "" ]; then
        SIZE=
        RECORDS=0
        echo "[Integration Test] arm_controller () telemetry:  records ( bytes)"
    fi
done

echo ""
echo "✓ Integration test PASSED: Full RLS pipeline ran end-to-end"
