#ifndef OWNERSHIP_ARBITER_HPP
#define OWNERSHIP_ARBITER_HPP

#include <iostream>
#include <string>
#include <array>

enum class ArmOwner {
    NONE,
    LEFT_ARM,
    RIGHT_ARM
};

class OwnershipArbiter {
private:
    ArmOwner current_owner;
    double hysteresis_margin; // Prevents rapid switching in the center zone

public:
    OwnershipArbiter(double margin = 0.05) : current_owner(ArmOwner::NONE), hysteresis_margin(margin) {}

    /**
     * @brief Determines which arm owns the ball using hysteresis and velocity tie-breakers.
     */
    ArmOwner arbitrate(const std::string& thread_name, const std::array<double, 3>& pos, const std::array<double, 3>& vel, double sim_time) {
        bool moving_left = (vel[0] < 0.0);
        bool moving_right = (vel[0] > 0.0);

        ArmOwner previous_owner = current_owner;

        // Asymmetric / Hysteresis boundary logic around X = 0.0
        if (pos[0] < -hysteresis_margin) {
            current_owner = ArmOwner::LEFT_ARM;
        } else if (pos[0] > hysteresis_margin) {
            current_owner = ArmOwner::RIGHT_ARM;
        } else {
            // Center Zone: Tie-breaker using velocity direction
            if (moving_left && pos[0] <= 0.02) {
                current_owner = ArmOwner::LEFT_ARM;
            } else if (moving_right && pos[0] > -0.02) {
                current_owner = ArmOwner::RIGHT_ARM;
            }
            // If ambiguous, it retains previous_owner implicitly
        }

        // Log only on ownership transition. We log the thread_name so we can 
        // verify both threads independently reached the same conclusion.
        if (current_owner != previous_owner) {
            std::cout << "[Arbiter][" << thread_name << "][t=" << sim_time << "s] Ownership assigned to: " 
                      << (current_owner == ArmOwner::LEFT_ARM ? "Arm A (Left)" : 
                          current_owner == ArmOwner::RIGHT_ARM ? "Arm B (Right)" : "NONE")
                      << " (Ball Pos X: " << pos[0] << "m)\n";
        }

        return current_owner;
    }
};

#endif // OWNERSHIP_ARBITER_HPP