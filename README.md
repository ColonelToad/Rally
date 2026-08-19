# Rally: Real-Time Dual-Arm Robotic Table Tennis System

Rally is a high-performance robotic table tennis simulation combining deterministic C++ real-time control loops (1kHz physics, 500Hz per-arm) with Python-based LLM orchestration. The system demonstrates **ownership arbitration without learning**, **EKF-based ball prediction** (validated against real robot data), and **adaptive arm strategy via recursive least squares**.

## Quick Links

- **Architecture & Design**: See [`docs/TECHNICAL_WRITEUP.md`](docs/TECHNICAL_WRITEUP.md) (3.5k words)
- **Design Philosophy**: [`docs/PHILOSOPHY.md`](docs/PHILOSOPHY.md) — read first for context
- **Testing & Validation**: [`docs/TESTING.md`](docs/TESTING.md)
- **Embedded Deployment**: [`docs/EMBEDDED_NOTES.md`](docs/EMBEDDED_NOTES.md)

## System Architecture

Rally splits execution across **C++ (real-time) + Python (orchestration)**, communicating entirely via ZeroMQ to ensure process isolation and determinism.

### C++ Real-Time Engine (1kHz + 500Hz)

| Component | Frequency | Purpose |
|-----------|-----------|---------|
| **Physics Arbiter** | 1kHz | MuJoCo forward dynamics, ball EKF, ownership arbitration |
| **Left Arm Controller** | 500Hz | Trajectory planning, IK, impedance control |
| **Right Arm Controller** | 500Hz | Trajectory planning, IK, impedance control |
| **High-Level Controller** | 100Hz | Ownership routing, RLS strategy updates, rally outcome handling |

**Key Algorithms**:
- **Ownership Arbiter**: Hysteresis-based spatial partitioning (no learning)
- **EKF Ball Predictor**: Drag model with 4.1mm validation error on real robot data
- **AnalyticalIK**: 7-DOF Franka Panda with joint limits
- **RLS Adaptive Strategy**: Per-arm bias learning (θ = [target_offset_y, aggression_factor, reaction_margin])

### Python Orchestrator (60 FPS + Async)

- **MuJoCo Passive Viewer**: Renders simulation + keyboard input (GLFW)
- **LLM Brain** (optional): Qwen2.5 via llama.cpp for tactical evaluation
- **Streamlit Dashboard** (future): Telemetry visualization

## Technical Highlights

### No Dynamic Allocation in Hot Paths
```cpp
// All control-path data structures are fixed-size
class EkfBallPredictor {
    Eigen::Vector6d state_;
    Eigen::Matrix6d P_;
};
```

### Unified Binary Logging (One Schema, One Replayer)
- 64-byte fixed-size LogRecord written by every subsystem
- Lock-free per-thread SPSC buffers
- 5ms periodic flush to disk
- Supports live dashboards + offline analysis

### Real-World Validation
- **DeepMind Dataset**: EKF tested on 200 real ball trajectories → 4.1mm mean error
- **Fixed-Point Path**: Q32.32 math validated for embedded FPU-less targets
- **Integration Tests**: All 4 processes run concurrently, telemetry logged to disk

## Getting Started

### Prerequisites
```bash
# Ubuntu/Debian
sudo apt-get install cmake g++ libeigen3-dev libzmq3-dev

# Python
pip install mujoco llama-cpp-python
```

### Build & Run

```bash
mkdir build && cd build
cmake ..
make

# Terminal 1: Physics loop (1kHz)
./mujoco_bridge

# Terminal 2: Ownership arbiter + RLS (100Hz)
./coordination_bus

# Terminal 3 & 4: Per-arm control (500Hz each, with core affinity)
./arm_controller --side left --core 2
./arm_controller --side right --core 3

# View logs
ls logs/rally_telemetry*.log   # Binary telemetry from each process
```

### Build Options

```bash
# Fixed-point math for embedded targets (no FPU)
cmake .. -DRALLY_FIXED_POINT=ON

# Include LLM orchestrator + Streamlit (adds Python dependency)
cmake .. -DRALLY_ONLINE_FEATURES=ON
```

### Testing

```bash
# Unit tests
ctest --output-on-failure

# Integration test (all 4 processes + telemetry validation)
bash tests/integration/test_rls_pipeline.sh
```