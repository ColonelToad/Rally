# Target Deployment Architecture: AMD/Xilinx Zynq UltraScale+ MPSoC

This document outlines the architectural mapping required to transition this C++ codebase from the desktop/MuJoCo simulation environment to an embedded hardware target (e.g., Zynq UltraScale+ ZCU102 or custom PCB).

---

## 1. Heterogeneous Processing Layout

The Zynq UltraScale+ architecture contains a Processing System (PS) and Programmable Logic (PL/FPGA). The application workload is divided across three execution domains:

+-----------------------------------------------------------------------+
|                       Zynq UltraScale+ MPSoC                          |
|                                                                       |
|  +---------------------------------+  +----------------------------+  |
|  | APU: Quad ARM Cortex-A53        |  | RPU: Dual ARM Cortex-R5F   |  |
|  | OS: PREEMPT_RT Linux            |  | OS: Bare-Metal / FreeRTOS  |  |
|  +---------------------------------+  +----------------------------+  |
|  | - ZeroMQ / WebSockets Telemetry |  | - 500Hz/1kHz Control Loop  |  |
|  | - 60/40 Seam Supervisor Logic   |  | - Analytical IK Solver     |  |
|  | - Trajectory Spline Generation  |  | - Impedance Controller     |  |
|  +---------------------------------+  +----------------------------+  |
|                  |                                   |                |
|                  +-----------------+-----------------+                |
|                                    | AXI4-Lite / DMA                  |
|                                    v                                  |
|  +-----------------------------------------------------------------+  |
|  | PL: Programmable Logic (FPGA)                                   |  |
|  | - Custom Fixed-Point EKF IP Core (Camera State Predictor)       |  |
|  | - EtherCAT / CAN FD Master IP Core (Motor Drives)                 |  |
|  | - SPI Encoder Interfaces                                        |  |
|  +-----------------------------------------------------------------+  |
+-----------------------------------------------------------------------+

### A. Application Processing Unit (APU) - Quad Cortex-A53
* **OS:** Linux with `PREEMPT_RT` patch set.
* **Role:** Task allocation supervisor, high-level trajectory generation, logging, external camera interface, and network comms (ZeroMQ/WebSockets).
* **Execution:** Runs non-real-time and soft real-time routines.

### B. Real-Time Processing Unit (RPU) - Dual Cortex-R5F (Lockstep)
* **OS:** Bare-metal or FreeRTOS.
* **Role:** Deterministic 500Hz to 1kHz control hot path. Executes `ImpedanceController`, `AnalyticalIK`, and HAL hardware reads/writes.
* **Memory:** Code and hot-path stack mapped directly into 128KB Tightly Coupled Memory (TCM) for zero-wait-state instruction execution.

### C. Programmable Logic (PL / FPGA)
* **Role:** Offloads high-frequency I/O and parallel matrix acceleration.
* **IP Cores:**
  * **CAN FD / EtherCAT Master:** Replaces `MuJoCoEmulator` to directly send torque commands to motor drives (e.g., Elmo Gold Twitter).
  * **Fixed-Point EKF Accelerator:** Runs 16.16 fixed-point matrix multiplies for 1kHz ball trajectory prediction in hardware.

---

## 2. Hardware Abstraction Layer (HAL) Mapping

In simulation, `hal::MuJoCoEmulator` implements `hal::Sensor` and `hal::Actuator`. On physical hardware:

1. **`hal::Sensor::readJoints()`**
   * Replaced by `hal::ZynqSpiSensor`, which reads optical joint encoder registers over SPI via AXI DMA transfers into RPU memory.
2. **`hal::Actuator::writeTorques()`**
   * Replaced by `hal::CanFdActuator`, which packages 16-bit target current commands and writes them to the CAN FD IP registers in the PL.

---

## 3. Real-Time Guarantees & Memory Discipline

To prevent Priority Inversion, Non-Deterministic Latency, and Out-Of-Memory (OOM) faults:

* **Zero Dynamic Allocation (`malloc`/`new`):** All stack and static memory allocations are verified via `-fsanitize=address` during CI/CD.
* **Cache Management:** RPU memory pages mapped to TCM bypass the L1 cache, eliminating cache-miss jitter during impedance torque calculations.
* **Fixed-Point Math Strategy:** High-speed EKF matrix operations in the PL use Q16.16 fixed-point arithmetic, while the RPU uses hardware Floating Point Units (FPUs) for single-precision IEEE 754 analytical IK calculations.