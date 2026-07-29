#ifndef HAL_MUJOCO_EMULATOR_HPP
#define HAL_MUJOCO_EMULATOR_HPP

#include "sensor.hpp"
#include "actuator.hpp"
#include <mujoco/mujoco.h>
#include <iostream>

namespace hal {

class MuJoCoEmulator : public Sensor, public Actuator {
private:
    const mjModel* m;
    mjData* d;
    int ball_qpos_adr;
    int ball_qvel_adr;

public:
    MuJoCoEmulator(const mjModel* model, mjData* data) : m(model), d(data) {
        int ball_id = mj_name2id(m, mjOBJ_JOINT, "ball_joint");
        if (ball_id >= 0) {
            ball_qpos_adr = m->jnt_qposadr[ball_id];
            ball_qvel_adr = m->jnt_dofadr[ball_id];
        } else {
            ball_qpos_adr = -1;
            ball_qvel_adr = -1;
            std::cerr << "[HAL] Warning: ball_joint not found for emulation!" << std::endl;
        }
    }

    // --- Sensor Interface ---
    void readJoints(std::array<double, 7>& q, std::array<double, 7>& dq) override {
        for (int i = 0; i < 7; ++i) {
            q[i] = d->qpos[i];
            dq[i] = d->qvel[i];
        }
    }

    void readBallState(std::array<double, 3>& pos, std::array<double, 3>& vel) override {
        if (ball_qpos_adr >= 0) {
            pos[0] = d->qpos[ball_qpos_adr];
            pos[1] = d->qpos[ball_qpos_adr + 1];
            pos[2] = d->qpos[ball_qpos_adr + 2];

            vel[0] = d->qvel[ball_qvel_adr];
            vel[1] = d->qvel[ball_qvel_adr + 1];
            vel[2] = d->qvel[ball_qvel_adr + 2];
        } else {
            pos.fill(0.0);
            vel.fill(0.0);
        }
    }

    double getTime() override {
        return d->time;
    }

    // --- Actuator Interface ---
    void writeTorques(const std::array<double, 7>& torques) override {
        for (int i = 0; i < 7; ++i) {
            // Apply torque commands plus bias forces (gravity/Coriolis compensation)
            d->ctrl[i] = torques[i] + d->qfrc_bias[i];
        }
    }
};

} // namespace hal

#endif // HAL_MUJOCO_EMULATOR_HPP