#ifndef HAL_MUJOCO_EMULATOR_HPP
#define HAL_MUJOCO_EMULATOR_HPP

#include "sensor.hpp"
#include "actuator.hpp"
#include <mujoco/mujoco.h>
#include <iostream>
#include <array>

namespace hal {

class MuJoCoEmulator : public Sensor, public Actuator {
private:
    const mjModel* m;
    mjData* d;
    int ball_qpos_adr;
    int ball_qvel_adr;

public:
    MuJoCoEmulator(const mjModel* model, mjData* data);

    // --- Sensor Interface ---
    void readJoints(std::array<double, 7>& q_left, std::array<double, 7>& dq_left,
                    std::array<double, 7>& q_right, std::array<double, 7>& dq_right) override;
    void readBallState(std::array<double, 3>& pos, std::array<double, 3>& vel) override;
    double getTime() override;

    // --- Actuator Interface ---
    void writeTorques(const std::array<double, 7>& tau_left, const std::array<double, 7>& tau_right) override;
};

} // namespace hal

#endif // HAL_MUJOCO_EMULATOR_HPP