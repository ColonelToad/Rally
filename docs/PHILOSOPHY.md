# RALLY — Design Philosophy

> *Real-time Autonomous Low-Latency sYstem*

This document records the engineering principles that govern RALLY's design. It exists for two reasons: to force precision about decisions that are easy to hold loosely, and to let any reviewer understand the thinking behind the code without reverse-engineering it. When a decision in the codebase seems non-obvious, the answer should be traceable to something written here.

---

## 1. C++ owns the critical path. Python owns the human and AI interfaces.

The boundary between these two languages is a deliberate architectural decision, not a matter of convenience. Anything with a timing constraint or a hardware interface is written in C++. Anything that talks to a human or an AI model is written in Python. That line is documented, enforced at the module boundary level, and does not move without a recorded reason.

This is not "use C++ when possible." Trying to write the LLM orchestration layer or the Streamlit dashboard in C++ would be worse engineering. The principle is about ownership of the critical path, not maximizing C++ line count.

The language boundary itself is a design artifact worth reading. If a reviewer wants to understand what RALLY considers timing-critical, they look at what is written in C++.

---

## 2. Dependencies earn their place.

A dependency is justified when it handles non-trivial complexity that would take weeks to replicate correctly and that is not the point of the project. The bar is genuine: does this library handle numerical edge cases, ABI stability concerns, or platform-specific behavior that would consume significant engineering time to solve from scratch?

**Eigen** clears the bar. It is header-only, used in production aerospace and robotics systems, and handles floating-point numerical stability in linear algebra that is easy to get subtly wrong. Reimplementing it would be weeks of work that teaches nothing new about real-time systems.

**A JSON parser for a fixed schema** does not clear the bar. A small hand-written parser for a known, versioned message format is a better choice than importing a general-purpose library.

**MuJoCo and ZeroMQ** clear the bar on maturity grounds (see Principle 4).

When a dependency is added, a comment in the relevant `CMakeLists.txt` or `README` records *why* — what specific complexity it handles that justified inclusion.

---

## 3. Designed for embedded deployment from day one.

RALLY is written as if it will run on an ARM Cortex-A72 with 4GB RAM, no network guarantee, and a real-time OS. It will not run there on day one, but the code should never need structural changes to get there.

Concretely this means:

- **No heap allocation in the hot path.** No `new`, no `malloc`, no `std::vector` resize inside any loop with a timing constraint. Fixed-size arrays and pre-allocated pools from the start.
- **Static allocation for all control-path data structures.** Size assertions (`static_assert`) catch alignment surprises at compile time.
- **Offline-capable by default.** The LLM and Streamlit layers can be disabled at compile time without touching the control loop. The flag is `RALLY_ONLINE_FEATURES` and it defaults to off.
- **Minimum memory footprint as a design constraint, not an afterthought.** When two implementations are equivalent in correctness, the one with lower memory footprint is preferred.
- **Fixed-point arithmetic as an optional compile path.** The EKF and trajectory planner have a `#ifdef RALLY_FIXED_POINT` path that replaces `double` with a fixed-point type. This path is tested on every commit.

The target embedded profile is documented in `docs/EMBEDDED_NOTES.md`, including what would change for a real deployment and what would not.

---

## 4. Use the tool that handles the non-standard cases.

Library maturity matters most at the edges of the problem, not the center. At the center — the Kalman filter, the IK solver, the PID controller — the code is written from scratch because that is the point of the project. At the edges — MuJoCo integration, ZeroMQ messaging, LLM inference — maturity matters because non-standard behavior at the edges causes failures that are hard to diagnose and expensive to fix.

The guiding question when choosing a library is not "is this the most modern option" but "has this handled the non-standard cases that exist just off the happy path." A library that works for the common case and breaks silently on slightly unusual hardware or slightly malformed input is worse than no library at all.

Prefer libraries with a stable C ABI. C++ ABIs are not stable across compilers and versions in the way C ABIs are. MuJoCo is a C library. ZeroMQ has a C core. This is not a coincidence, and it is a property worth preserving when choosing future dependencies.

This principle has a corollary: do not choose a dependency because it is new or because it represents a skill worth learning. Choose it because it is the most battle-tested option for the specific integration problem at hand.

---

## 5. Explicit typed contracts at every layer boundary.

Every message that crosses a layer boundary has a versioned, typed schema defined in one place before any implementation begins. These live in `include/rally/messages/`. They are C structs, not C++ classes, so they are layout-stable and can cross the C/C++ boundary without surprises.

Every schema struct has a size assertion:

```cpp
static_assert(sizeof(BallState) == 48, "BallState layout changed — update all serializers");
```

This is not defensive programming. It is the boundary contract made machine-checkable.

The layers and their boundary messages are:

| From | To | Message |
|------|----|---------|
| LLM orchestrator | Coordination bus | `StrategyCommand` |
| Coordination bus | Arm controller (A or B) | `ArmAssignment` |
| Arm controller | HAL | `TorqueCommand` |
| HAL | MuJoCo | raw ZeroMQ frame |
| MuJoCo | HAL | `SensorPacket` |
| HAL | Arm controller | `JointState` |
| Arm controller | Coordination bus | `ArmStatus` |

No layer reaches across this boundary to touch the internals of another. If a reviewer sees a direct dependency between non-adjacent layers, it is a bug in the architecture, not a shortcut.

---

## 6. Observability is a first-class concern, not an afterthought.

Every component emits structured log records from day one. Not `printf`. Not ad-hoc file writes. A lightweight ring-buffer logger (`core/logger`) writes binary timestamped records at a fixed memory cost, replayable offline without re-running the simulation.

The specific things logged at all times:

- Task executor: period, actual execution time, deadline miss flag — every cycle
- EKF: predicted landing point, innovation (measurement residual), covariance trace — every fusion step
- Ownership arbiter: assignment decision, ball zone, confidence — every decision
- HAL: round-trip latency to MuJoCo — every frame

The jitter histogram is derived from the task executor logs. The Streamlit dashboard reads the same log format the offline replayer does — there is one log schema, not a "live" format and a "replay" format.

The test for observability: given only the log files from a session, you should be able to reconstruct exactly what the system was doing at any timestamp. If you cannot, the logging is incomplete.

---

## 7. Correctness before performance, but designed so performance work never requires structural changes.

Do not optimize prematurely. But do not write code that would require restructuring to optimize later.

In practice: use fixed-size arrays from the start even when `std::vector` would be easier to write. Choose data layouts that are cache-friendly from the start even before profiling. Put timing-critical code in its own translation unit from the start so it can be compiled with different flags without touching everything else.

The rule of thumb: if making this correct code fast would require changing its interfaces, its memory layout, or its thread ownership model, the original design was wrong. Performance work should be local — changing implementations, not architectures.

---

## 8. The HLC/LLC split is a hard boundary, not a guideline.

Following DeepMind's own architecture for the table tennis system, RALLY separates "does the system know what to do" from "can it physically do it."

The **High-Level Controller (HLC)** is the ownership arbiter and strategy layer. It decides which arm plays the ball, what the target return trajectory is, and what strategic mode is active. It operates at 100Hz and is allowed to be stateful in a way the LLC is not.

The **Low-Level Controller (LLC)** is the per-arm EKF, trajectory planner, and torque controller. It executes what the HLC assigns. It operates at 500Hz. It does not make strategic decisions. It does not communicate with the other arm — only the HLC does.

No LLC component calls into the HLC. No HLC component writes directly to a torque command. The data flow is one-directional across this boundary. If it is not, it is a bug.

---

## What this document is not

This is not a list of rules to follow mechanically. It is a record of the reasoning behind the design. When a situation arises that these principles do not clearly resolve, the right question is "what would the principle behind the principle say" — not "which rule applies."

When a principle is violated for a good reason, the reason is recorded in a comment at the violation site with a `// PHILOSOPHY:` prefix and a reference to the principle number. A violation with a recorded reason is better than a clean codebase with no record of the tradeoffs made.
