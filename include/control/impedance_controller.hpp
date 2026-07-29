#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP

#include <array>

class ImpedanceController {
public:
    // Franka Panda joint stiffness and damping gains
    std::array<double, 7> kp = {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
    std::array<double, 7> kd = {50.0,  50.0,  50.0,  50.0,  20.0,  15.0,  5.0};

    std::array<double, 7> computeTorques(const std::array<double, 7>& q_curr, 
                                         const std::array<double, 7>& dq_curr,
                                         const std::array<double, 7>& q_ref, 
                                         const std::array<double, 7>& dq_ref) {
        std::array<double, 7> torques;
        for (int i = 0; i < 7; ++i) {
            double q_err = q_ref[i] - q_curr[i];
            double v_err = dq_ref[i] - dq_curr[i];

            // Pure PD control law (gravity compensation is now handled by the HAL/Actuator)
            torques[i] = kp[i] * q_err + kd[i] * v_err;
        }
        return torques;
    }
};

#endif // IMPEDANCE_CONTROLLER_HPP