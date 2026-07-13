cmake_minimum_required(VERSION 3.10)
project(RallyKinematics)

set(CMAKE_CXX_STANDARD 17)

# 1. Define where MuJoCo lives on your machine
set(MUJOCO_PATH "${CMAKE_SOURCE_DIR}/mujoco/mujoco-3.10.0" CACHE PATH "Path to MuJoCo installation")
# 2. Find Eigen3 (System installed via apt)
find_package(Eigen3 REQUIRED)

# 3. Include directories for our headers
include_directories(
    ${MUJOCO_PATH}/include
    ${EIGEN3_INCLUDE_DIRS}
)

# 4. Define the executable
add_executable(kinematics kinematics.cpp)

# 5. Link the MuJoCo compiled libraries to our executable
target_link_directories(kinematics PRIVATE ${MUJOCO_PATH}/lib)
target_link_libraries(kinematics PRIVATE mujoco)
add_executable(workspace workspace.cpp)
target_link_directories(workspace PRIVATE ${MUJOCO_PATH}/lib)
target_link_libraries(workspace PRIVATE mujoco)
add_executable(ik ik.cpp)
target_link_directories(ik PRIVATE ${MUJOCO_PATH}/lib)
target_link_libraries(ik PRIVATE mujoco)