#include "hal/mujoco_emulator.hpp"

namespace hal {

MuJoCoEmulator::MuJoCoEmulator(const mjModel* model, mjData* data) : m(model), d(data) {
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
void MuJoCoEmulator::readJoints(std::array<double, 7>& q, std::array<double, 7>& dq) {
    // Note: Currently only reading the first 7 joints (Left Arm). 
    // If your impedance controller needs Right Arm states, this will need to be 
    // updated to accept an arm index or return both arrays.
    for (int i = 0; i < 7; ++i) {
        q[i] = d->qpos[i];
        dq[i] = d->qvel[i];
    }
}

void MuJoCoEmulator::readBallState(std::array<double, 3>& pos, std::array<double, 3>& vel) {
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

double MuJoCoEmulator::getTime() {
    return d->time;
}

// --- Actuator Interface ---
void MuJoCoEmulator::writeTorques(const std::array<double, 7>& tau_left, const std::array<double, 7>& tau_right) {
    // 1. Apply Left Arm Torques (Indices 0 to 6)
    for (int i = 0; i < 7; ++i) {
        d->ctrl[i] = tau_left[i] + d->qfrc_bias[i];
    }

    // 2. Apply Right Arm Torques (Indices 7 to 13)
    for (int i = 0; i < 7; ++i) {
        d->ctrl[i + 7] = tau_right[i] + d->qfrc_bias[i + 7];
    }
}

} // namespace hal