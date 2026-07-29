#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP

#include <mujoco/mujoco.h>
#include <array>

class ImpedanceController {
public:
    // Franka Panda joint stiffness and damping gains
    std::array<double, 7> kp = {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
    std::array<double, 7> kd = {50.0,  50.0,  50.0,  50.0,  20.0,  15.0,  5.0};

    void computeTorques(const mjModel* m, mjData* d, 
                        const std::array<double, 7>& q_ref, 
                        const std::array<double, 7>& v_ref) {
        for (int i = 0; i < 7; ++i) {
            double q_err = q_ref[i] - d->qpos[i];
            double v_err = v_ref[i] - d->qvel[i];

            // PD control law + MuJoCo bias force compensation (gravity + Coriolis)
            double tau_impedance = kp[i] * q_err + kd[i] * v_err;
            d->ctrl[i] = tau_impedance + d->qfrc_bias[i];
        }
    }
};

#endif // IMPEDANCE_CONTROLLER_HPP