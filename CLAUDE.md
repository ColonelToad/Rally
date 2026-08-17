# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## Project Overview

**Rally** is a high-performance robotic table tennis simulation system combining deterministic C++ real-time control loops with Python-based LLM orchestration. The system features dual-arm robotic interception via MuJoCo physics simulation, with inter-process communication via ZeroMQ (ZMQ).

### Architecture at a Glance

- **C++ Real-Time Engine** (1kHz physics arbiter, 500Hz control loops): Handles physics simulation, dual-arm ownership arbitration, inverse kinematics, trajectory prediction, and impedance control.
- **Python Orchestrator** (60 FPS viewer, AI coaching): MuJoCo passive viewer with GLFW input handling and an edge-optimized local LLM (Qwen2.5 via llama.cpp) for tactical strategy.
- **Inter-Process Boundary**: ZeroMQ carries all communication; no shared memory between processes.

Read `docs/PHILOSOPHY.md` first for the full design rationale—it documents the reasoning behind architectural decisions and is essential context for understanding why code is structured as it is.

---

## Build & Development

### Prerequisites
- CMake (3.14+)
- C++17 compiler (GCC/Clang)
- Eigen3
- ZeroMQ (libzmq)
- MuJoCo 3.10.0 (preinstalled in `mujoco/` directory)
- Python 3.10+ (for viewer and LLM orchestration)
- `pip install mujoco zmq llama-cpp-python`

### Build & Run

```bash
# Build the project
mkdir build && cd build
cmake ..
make

# Run the C++ engine
./rally_engine

# Run a specific test
ctest --output-on-failure -R <test_name>

# Run all tests
ctest --output-on-failure

# Run with AddressSanitizer enabled (default in Debug)
cmake .. -DCMAKE_BUILD_TYPE=Debug
make
ctest --output-on-failure
```

### Sanitizer & Embedded Discipline

- **AddressSanitizer & UndefinedBehaviorSanitizer** are enabled by default in Debug builds (`-fsanitize=address,undefined`).
- The `RALLY_SANITIZE` CMake option controls this; it defaults to `ON`.
- **Zero heap allocation in hot paths**: The codebase forbids `new`, `malloc`, and `std::vector` resize inside timing-critical loops. All control-path data structures use fixed-size arrays or pre-allocated pools.
- See `docs/TESTING.md` for how tests verify absence of dynamic allocation.

---

## Project Structure

### Core Directories

- **`include/`**: Public headers organized by subsystem.
  - `rally/`: Core data structures and messages (e.g., `BallState`, `StrategyCommand`).
  - `control/`, `hal/`, `ipc/`, `planner/`: Subsystem interfaces and declarations.
  - All boundary messages are C structs with size assertions (e.g., `static_assert(sizeof(BallState) == 48, ...)`).

- **`src/`**: Implementation files, mirroring include structure.
  - `core/`: Task executor, clock, and core timing utilities.
  - `hal/`: Hardware Abstraction Layer (MuJoCo emulator for simulation).
  - `ipc/`: ZeroMQ context management and IPC utilities.

- **`apps/`**: Standalone executables.
  - `mujoco_bridge_main.cpp`: Simulation physics engine (1kHz arbiter + HAL bridge).
  - `arm_controller_main.cpp`: Per-arm 500Hz control loop (IK, trajectory planning, impedance).
  - `coordination_bus_main.cpp`: Ownership arbitration and strategy routing.
  - `ekf_validator_main.cpp`: Extended Kalman Filter validation against reference data.
  - `benchmark_executor_main.cpp`: Performance profiling harness.

- **`tests/unit/`**: Catch2-based unit tests.
  - All tests use mocked clock (`rally::core::Clock` in `SIMULATED` mode).
  - Tests verify both correctness and execution time (jitter).

- **`docs/`**:
  - `PHILOSOPHY.md`: Design principles—**read this first**.
  - `TESTING.md`: Testing philosophy and validation strategy.
  - `EMBEDDED_NOTES.md`: Target hardware mapping (Zynq UltraScale+) and real deployment considerations.

---

## Key Architectural Boundaries

### C++ vs. Python Ownership

From **PHILOSOPHY.md Principle 1**: C++ owns the critical path (control loops, physics, real-time I/O); Python owns human/AI interfaces (viewer, LLM orchestration, dashboards). This boundary is enforced at the module level and does not move without a recorded reason.

### High-Level Controller (HLC) vs. Low-Level Controller (LLC)

From **PHILOSOPHY.md Principle 8**:
- **HLC** (100Hz): Ownership arbitration, strategic decision-making, per-arm assignment. Stateful.
- **LLC** (500Hz): Per-arm EKF, trajectory planning, torque control. Executes HLC commands without making strategic decisions.
- Data flow is strictly one-directional: HLC → LLC. No LLC → HLC calls. No LLC → LLC peer communication except through HLC.

### Message Contracts at Layer Boundaries

All inter-process and inter-component communication uses versioned C structs defined in `include/rally/messages/`:

| From | To | Message |
|------|----|---------|
| LLM orchestrator | Coordination bus | `StrategyCommand` |
| Coordination bus | Arm controller | `ArmAssignment` |
| Arm controller | HAL | `TorqueCommand` |
| HAL | MuJoCo | raw ZMQ frame |
| MuJoCo | HAL | `SensorPacket` |
| HAL | Arm controller | `JointState` |
| Arm controller | Coordination bus | `ArmStatus` |

Each message struct has a compile-time size assertion: violating a struct layout will fail the build.

---

## Dependency Philosophy

From **PHILOSOPHY.md Principle 2**: Dependencies must handle non-trivial complexity not central to the project.

### Included & Justified
- **Eigen3**: Header-only, battle-tested in aerospace/robotics, handles floating-point numerical stability in linear algebra.
- **MuJoCo / ZeroMQ**: Production-grade maturity, handle platform-specific edge cases and ABI stability (C core for both).

### Not Included
- General-purpose JSON parser: For fixed-schema messages, hand-written parsers are preferred.

When adding a dependency, document *why* in a CMakeLists.txt comment: what specific complexity justifies it.

---

## Testing Philosophy

From `docs/TESTING.md`:

1. **Timing is Correctness**: Tests verify execution time, not just output values. The task executor tests assert jitter bounds under synthetic load.
2. **The Clock is Always Mocked**: Tests use `rally::core::Clock` in `SIMULATED` mode for determinism across machines and CI systems.
3. **Zero-Allocation Assertions**: AddressSanitizer catches memory leaks; test design verifies memory usage remains flat during control loops.
4. **Framework**: Catch2—lightweight, CMake integration, clean macro-based assertions (`REQUIRE`, `CHECK`).
5. **Offline Validation**: EKF, kinematics, and trajectory planning are validated against pre-recorded DeepMind datasets.

---

## Embedded Deployment & Memory Constraints

From `docs/EMBEDDED_NOTES.md`:

Rally is designed for embedded deployment (Zynq UltraScale+ MPSoC) from day one, even though it runs on desktop simulation today:

- **No heap allocation in the hot path**: Enforced from the start, verified by sanitizers.
- **Fixed-size arrays & pre-allocated pools**: All control-path data structures are static.
- **Offline-capable by default**: LLM and Streamlit layers can be compiled out via `RALLY_ONLINE_FEATURES` flag (defaults to off).
- **Minimum memory footprint as a design constraint**: When equivalent in correctness, lower footprint is preferred.
- **Fixed-point arithmetic optional path**: EKF and trajectory planner have `#ifdef RALLY_FIXED_POINT` for fixed-point replace of `double`.

The target deployment splits execution across:
- **APU (Quad Cortex-A53)**: Linux with PREEMPT_RT, task supervision, trajectory generation, logging, network comms.
- **RPU (Dual Cortex-R5F)**: Bare-metal or FreeRTOS, deterministic 500Hz–1kHz control loop, IK, impedance control.
- **PL (FPGA)**: High-frequency I/O, fixed-point EKF acceleration, motor drive interfaces.

Code structure today must not require refactoring to support this mapping.

---

## Observability

From **PHILOSOPHY.md Principle 6**: Observability is first-class, not an afterthought.

Every component logs structured records from day one (not `printf`, not ad-hoc files). A lightweight ring-buffer logger writes binary timestamped records at fixed memory cost, replayable offline:

- **Task executor**: Period, actual execution time, deadline miss flag (every cycle).
- **EKF**: Predicted landing point, innovation (measurement residual), covariance trace (every fusion step).
- **Ownership arbiter**: Assignment decision, ball zone, confidence (every decision).
- **HAL**: Round-trip latency to MuJoCo (every frame).

Jitter histograms are derived from task executor logs. The Streamlit dashboard and offline replayer read the same log format—there is one schema, not separate "live" and "replay" formats.

---

## Code Style & Patterns

- **Explicit typed contracts**: Every boundary message is a versioned C struct with size assertions.
- **No PHILOSOPHY violations without recording**: If a principle is broken, annotate the violation site with a `// PHILOSOPHY: <number>` comment and the reason.
- **Data layout for cache efficiency**: Prefer cache-friendly layouts from the start, not after profiling.
- **Correctness before performance, but designed for performance later**: Use fixed-size arrays from day one even when `std::vector` would be easier, so optimizations never require structural changes.
- **Interface stability**: If making correct code fast would require changing interfaces, memory layout, or thread ownership, the original design was wrong.

---

## Reviewing Changes

When reviewing PRs or changes, verify:

1. **HLC/LLC boundary maintained**: No LLC → HLC calls. No LLC → LLC peer communication except through HLC.
2. **Message size assertions unchanged** (or deliberately updated with comment).
3. **No dynamic allocation in hot paths** (timing-critical loops, control threads).
4. **Observability preserved**: Logging statements updated or added for new behavior.
5. **Embedded constraints honored**: Code must still compile and run on resource-constrained targets after the change.
6. **PHILOSOPHY compliance**: If a principle is violated, a recorded reason must be present.

---

## Common Commands Cheat Sheet

```bash
# Configure and build
cd /path/to/rally
mkdir build && cd build
cmake ..
make

# Run all unit tests with output
ctest --output-on-failure

# Run a specific test by name
ctest --output-on-failure -R task_executor

# Run with verbose output
ctest --output-on-failure -V

# Rebuild a specific target
make arm_controller

# Clean and rebuild
cd build
rm -rf *
cmake ..
make

# Run the main simulation
./rally_engine

# Run an individual app
./arm_controller
./coordination_bus
./mujoco_bridge
```

---

## Contact & Philosophy Review

If a decision or design pattern is not immediately obvious, consult `docs/PHILOSOPHY.md`—the answer should be traceable to a principle recorded there. When in doubt about architectural trade-offs, ask: "What would the principle behind the principle say?"
