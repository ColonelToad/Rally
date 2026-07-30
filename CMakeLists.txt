cmake_minimum_required(VERSION 3.14)
# Updated project name to reflect the whole system, not just kinematics
project(Rally VERSION 0.1.0 LANGUAGES CXX)

# --- Embedded Constraints & Sanitizers ---
# Enable warnings and treat them as errors for embedded discipline
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")

# Enable AddressSanitizer and UndefinedBehaviorSanitizer for Debug builds
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address,undefined -fno-omit-frame-pointer")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fsanitize=address,undefined")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================================
# 1. BEST PRACTICES & COMPILER FLAGS (From PHILOSOPHY.md Principle 3)
# ==============================================================================
option(RALLY_SANITIZE "Build with Address and Undefined Behavior sanitizers" ON)

# Strict warnings to catch subtle C++ bugs early
add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)

if(RALLY_SANITIZE)
    message(STATUS "RALLY: Sanitizers ENABLED (-fsanitize=address,undefined)")
    add_compile_options(-fsanitize=address -fsanitize=undefined)
    add_link_options(-fsanitize=address -fsanitize=undefined)
endif()

# ==============================================================================
# 2. DEPENDENCIES (From PHILOSOPHY.md Principle 2 & 4)
# ==============================================================================

# -- MuJoCo --
set(MUJOCO_PATH "${CMAKE_SOURCE_DIR}/mujoco/mujoco-3.10.0" CACHE PATH "Path to MuJoCo installation")

# -- Eigen3 --
find_package(Eigen3 REQUIRED)

# -- ZeroMQ --
# We use pkg-config to reliably find the ZeroMQ C library
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# ==============================================================================
# 3. CORE LIBRARIES
# ==============================================================================

# Build the IPC wrapper as a static library
add_library(rally_ipc STATIC src/ipc/zmq_context.cpp)
# Give it access to our include/ folder
target_include_directories(rally_ipc PUBLIC ${CMAKE_SOURCE_DIR}/include)
# Link ZeroMQ specifically to this library
target_include_directories(rally_ipc PUBLIC ${ZMQ_INCLUDE_DIRS})
target_link_directories(rally_ipc PUBLIC ${ZMQ_LIBRARY_DIRS})
target_link_libraries(rally_ipc PUBLIC ${ZMQ_LIBRARIES})

# ==============================================================================
# 4. HARDWARE ABSTRACTION LAYER (HAL)
# ==============================================================================
add_library(rally_hal STATIC src/hal/mujoco_emulator.cpp)
target_include_directories(rally_hal PUBLIC ${CMAKE_SOURCE_DIR}/include)
# The HAL needs MuJoCo headers to compile
target_include_directories(rally_hal PUBLIC ${MUJOCO_PATH}/include)
target_link_directories(rally_hal PUBLIC ${MUJOCO_PATH}/lib)
target_link_libraries(rally_hal PUBLIC mujoco)

# ==============================================================================
# 5. WEEK 3: SYSTEM ARCHITECTURE EXECUTABLES (Production)
# ==============================================================================

# 1. MuJoCo Bridge
add_executable(mujoco_bridge apps/mujoco_bridge_main.cpp)
# Link the new rally_hal library alongside ipc and core
target_link_libraries(mujoco_bridge PRIVATE rally_ipc rally_core rally_hal)
target_include_directories(mujoco_bridge PRIVATE ${MUJOCO_PATH}/include)
target_link_directories(mujoco_bridge PRIVATE ${MUJOCO_PATH}/lib)
target_link_libraries(mujoco_bridge PRIVATE mujoco glfw)

# 2. Arm Controller
add_executable(arm_controller apps/arm_controller_main.cpp)
target_link_libraries(arm_controller PRIVATE rally_ipc)
# Controller will need Eigen for kinematics/planning later
target_link_libraries(arm_controller PRIVATE Eigen3::Eigen)

# 3. Coordination Bus
add_executable(coordination_bus apps/coordination_bus_main.cpp)
target_link_libraries(coordination_bus PRIVATE rally_ipc)

add_library(rally_core STATIC 
    src/core/clock.cpp
    src/core/task_executor.cpp
)
target_include_directories(rally_core PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rally_core PUBLIC Eigen3::Eigen)
target_link_libraries(mujoco_bridge PRIVATE rally_ipc rally_core)
target_link_libraries(arm_controller PRIVATE rally_ipc rally_core Eigen3::Eigen)
target_link_libraries(coordination_bus PRIVATE rally_ipc rally_core)

enable_testing()

add_executable(test_task_executor tests/unit/test_task_executor.cpp)
# Ensure it can find catch.hpp in the unit tests folder
target_include_directories(test_task_executor PRIVATE tests/unit)
target_link_libraries(test_task_executor PRIVATE rally_core)

# Register the test with CMake
add_test(NAME TaskExecutorTest COMMAND test_task_executor)

add_executable(benchmark_executor apps/benchmark_executor_main.cpp)
target_link_libraries(benchmark_executor PRIVATE rally_core Eigen3::Eigen)

add_executable(ekf_validator apps/ekf_validator_main.cpp)
# Link the basic 'mujoco' library and Eigen
target_link_libraries(ekf_validator PRIVATE rally_core mujoco Eigen3::Eigen)
# Point to both our local includes and MuJoCo's includes
target_include_directories(ekf_validator PRIVATE include ${MUJOCO_PATH}/include)
# Point to where the MuJoCo library file actually lives
target_link_directories(ekf_validator PRIVATE ${MUJOCO_PATH}/lib)
