# RALLY — Testing Philosophy

This document defines how I prove RALLY works. Just as `PHILOSOPHY.md` governs the architecture, these principles govern the validation.

---

## 1. Timing is Correctness
In a real-time system, producing the right answer too late is exactly as wrong as producing the wrong answer. 
* Our tests do not just verify output values; they verify execution time. 
* The `task_executor` tests explicitly assert that the jitter histogram remains within acceptable bounds under synthetic load.

## 2. The Clock is Always Mocked
Tests never rely on the host OS scheduler's wall-clock time. Every test initializes `rally::core::Clock` in `SIMULATED` mode. This guarantees that tests are entirely deterministic, reproducible across different machines, and immune to CI pipeline CPU throttling.

## 3. Zero-Allocation Assertions
Because RALLY forbids heap allocation in the hot path, our tests must prove the absence of `malloc`/`new`. We rely on our `RALLY_SANITIZE` (AddressSanitizer) build flag to catch memory leaks, but we also design our test harnesses to verify that memory usage remains perfectly flat during continuous control loops.

## 4. Framework Choice: Catch2
We use Catch2 for our C++ testing framework. 
* **Why:** It is lightweight, integrates seamlessly with CMake, and its macro-based assertion system (`REQUIRE`, `CHECK`) keeps test code exceptionally clean. 
* **Boundary:** Catch2 itself allocates memory during test setup and assertion reporting. This is acceptable because the test framework operates *outside* the hot path. The code *under test* remains strictly statically allocated.

## 5. Offline-First Validation
The EKF, Kinematics, and Trajectory Planner are validated against the pre-recorded DeepMind dataset. A test passes if RALLY's predicted intercept points and generated joint torques match the known-good dataset distributions within our defined tolerance limits.