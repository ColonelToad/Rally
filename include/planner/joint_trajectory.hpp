#ifndef JOINT_TRAJECTORY_HPP
#define JOINT_TRAJECTORY_HPP

#include "quintic_spline.hpp"
#include <array>

class JointTrajectoryManager {
private:
    std::array<QuinticSpline, 7> splines;
    double current_time;
    double total_duration;
    bool active;

public:
    JointTrajectoryManager() : current_time(0.0), total_duration(0.0), active(false) {}

    void startTrajectory(const std::array<double, 7>& q0, const std::array<double, 7>& v0, const std::array<double, 7>& acc0,
                         const std::array<double, 7>& qf, const std::array<double, 7>& vf, const std::array<double, 7>& accf,
                         double duration) {
        total_duration = duration;
        current_time = 0.0;
        for (size_t i = 0; i < 7; ++i) {
            splines[i].generate(q0[i], v0[i], acc0[i], qf[i], vf[i], accf[i], duration);
        }
        active = true;
    }

    bool update(double dt, std::array<double, 7>& q_ref, std::array<double, 7>& v_ref, std::array<double, 7>& a_ref) {
        if (!active) return false;

        current_time += dt;
        if (current_time >= total_duration) {
            current_time = total_duration;
            active = false; // Trajectory complete, hold final position
        }

        for (size_t i = 0; i < 7; ++i) {
            TrajectoryPoint pt = splines[i].evaluate(current_time);
            q_ref[i] = pt.pos;
            v_ref[i] = pt.vel;
            a_ref[i] = pt.acc;
        }

        return active;
    }

    bool isActive() const { return active; }
};

#endif // JOINT_TRAJECTORY_HPP