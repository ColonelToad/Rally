# Rally: Real-time Autonomous Low-Latency control sYstem

Rally is a high-performance robotic table tennis simulation that bridges deterministic, high-frequency C++ control loops with a Python-based asynchronous LLM tactical coach. The system is designed to simulate dual-arm robotic interception using MuJoCo, featuring real-time physics calculations and edge-optimized AI strategy adjustments.

## System Architecture

The architecture is split into two primary domains, communicating entirely via ZeroMQ (ZMQ) to ensure process isolation and zero-blocking execution.

### 1. C++ Real-Time Engine (The Brawn)
* **Physics & Arbiter (1kHz):** Manages the core simulation step and a thread-safe dual-arm ownership arbiter that dynamically assigns the active robotic arm based on ball trajectory, preventing collisions.
* **Control Loops (500Hz):** Handles Inverse Kinematics (IK), trajectory prediction, and impedance control for real-time paddle manipulation.

### 2. Python Orchestrator & LLM (The Brain)
* **MuJoCo Passive Viewer (60 FPS):** Renders the simulation and handles human keyboard inputs via GLFW without interrupting the C++ physics threads.
* **Asynchronous Tactical Coach:** An edge-optimized local LLM (`Qwen2.5` via `llama.cpp`) running entirely on CPU. It utilizes a custom "Latest-State Lock" drop-queue to evaluate post-rally telemetry and issue macro-tactical strategy offsets (e.g., targeting the opponent's weak side) without causing CPU contention or dropping simulation frames.

## Technical Stack
* **C++17:** Core engine, multithreading, IK, and deterministic control.
* **Python 3:** Viewer orchestration and AI inference.
* **MuJoCo:** High-fidelity physics rendering and kinematics.
* **ZeroMQ (ZMQ):** High-speed inter-process communication (PUSH/PULL for commands, SUB for telemetry).
* **llama.cpp:** Local LLM inference optimized for CPU execution.

## Getting Started

### Prerequisites
* CMake & C++ Compiler (GCC/Clang)
* Python 3.10+
* `pip install mujoco zmq llama-cpp-python`

### Running the System
1. **Start the C++ Backend:**
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ./rally_engine