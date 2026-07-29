#ifndef ANALYTICAL_IK_HPP
#define ANALYTICAL_IK_HPP

#include <array>
#include <cmath>
#include <algorithm>

class AnalyticalIK {
public:
    /**
     * @brief Computes analytical inverse kinematics for the Franka Emika Panda arm.
     * @param x Target Cartesian X position [m]
     * @param y Target Cartesian Y position [m]
     * @param z Target Cartesian Z position [m]
     * @param redundant_q3 Redundant joint angle for theta_3 to resolve redundancy [rad]
     * @param q_out Output array for the 7 joint angles [rad]
     * @return true if a valid analytical solution is found within workspace limits, false otherwise.
     */
    static bool computeIK(double x, double y, double z, double redundant_q3, std::array<double, 7>& q_out) {
        // 1. Franka Panda Link Dimensions (DH / Geometric Constants)
        const double d1 = 0.333;
        const double d3 = 0.316;
        const double d5 = 0.384;
        //const double d7 = 0.107; // Distance to end effector grasp point

        // 2. Approximate target wrist center position by backing off along the approach vector
        // For a ball interception hit, we assume an overhead or side reach pointing inwards.
        // Let's compute wrist position assuming a fixed orientation looking towards the origin/base.
        double r = std::sqrt(x * x + y * y);
        
        // Basic workspace boundary check
        double max_reach = 0.85;
        double min_reach = 0.10;
        if (r > max_reach || r < min_reach || z < 0.0 || z > 1.2) {
            return false;
        }

        // 3. Solve Theta 1 (Base rotation)
        double theta1 = std::atan2(y, x);

        // 4. Set Redundant Joint (Theta 3)
        double theta3 = redundant_q3;

        // 5. Solve Theta 2 and Theta 4 using planar triangle geometry from shoulder to wrist center
        // Distance from shoulder center to wrist center projected in the vertical plane
        double r_w = r - 0.0825; // accounting for shoulder offset if applicable
        double z_w = z - d1;
        double dist_sq = r_w * r_w + z_w * z_w;
        double dist = std::sqrt(dist_sq);

        // Check reachability of the triangle formed by upper arm (d3) and forearm (d5)
        if (dist > (d3 + d5) || dist < std::abs(d3 - d5)) {
            return false; // Out of arm span
        }

        double alpha = std::atan2(z_w, r_w);
        double cos_beta = (d3 * d3 + dist_sq - d5 * d5) / (2.0 * d3 * dist);
        cos_beta = std::clamp(cos_beta, -1.0, 1.0);
        double beta = std::acos(cos_beta);

        // Depending on configuration preference (elbow-up vs elbow-down)
        double theta2 = alpha - beta; 

        // Elbow distance / angle mapping for theta 4
        double cos_gamma = (d3 * d3 + d5 * d5 - dist_sq) / (2.0 * d3 * d5);
        cos_gamma = std::clamp(cos_gamma, -1.0, 1.0);
        double theta4 = M_PI - std::acos(cos_gamma);

        // 6. Wrist joints (Theta 5, Theta 6, Theta 7) set to maintain safe tool orientation for impact
        double theta5 = 0.0;
        double theta6 = M_PI_2;
        double theta7 = theta1; // Align end-effector orientation with base approach

        // Populate output array
        q_out[0] = theta1;
        q_out[1] = theta2;
        q_out[2] = theta3;
        q_out[3] = -theta4; // Panda joint 4 has a negative sign convention in standard configurations
        q_out[4] = theta5;
        q_out[5] = theta6;
        q_out[6] = theta7;

        // Joint limit verification for Franka Panda
        const std::array<double, 7> min_limits = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
        const std::array<double, 7> max_limits = { 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};

        for (size_t i = 0; i < 7; ++i) {
            if (q_out[i] < min_limits[i] || q_out[i] > max_limits[i]) {
                return false;
            }
        }

        return true;
    }
};

#endif // ANALYTICAL_IK_HPP