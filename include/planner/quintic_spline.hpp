#ifndef QUINTIC_SPLINE_HPP
#define QUINTIC_SPLINE_HPP

#include <cmath>
#include <array>

struct TrajectoryPoint {
    double pos;
    double vel;
    double acc;
};

class QuinticSpline {
public:
    double a0, a1, a2, a3, a4, a5;
    double T;

    QuinticSpline() : a0(0), a1(0), a2(0), a3(0), a4(0), a5(0), T(1.0) {}

    /**
     * @brief Computes quintic polynomial coefficients for a 1D trajectory segment.
     * @param q0 Initial position
     * @param v0 Initial velocity
     * @param acc0 Initial acceleration
     * @param qf Final target position
     * @param vf Final target velocity
     * @param accf Final target acceleration
     * @param duration Time duration to reach target [s]
     */
    void generate(double q0, double v0, double acc0, 
                  double qf, double vf, double accf, double duration) {
        T = duration;
        if (T <= 0.0) T = 0.001; // Guard against zero duration

        a0 = q0;
        a1 = v0;
        a2 = 0.5 * acc0;

        double T2 = T * T;
        double T3 = T2 * T;
        double T4 = T3 * T;
        double T5 = T4 * T;

        double delta_q = qf - (a0 + a1 * T + a2 * T2);
        double delta_v = vf - (a1 + 2.0 * a2 * T);
        double delta_acc = accf - (2.0 * a2);

        // Analytical solution for 5th order polynomial coefficients
        a3 = (20.0 * delta_q - 8.0 * delta_v * T - 3.0 * delta_acc * T2) / (2.0 * T3);
        a4 = (-15.0 * delta_q + 7.0 * delta_v * T + 1.5 * delta_acc * T2) / (2.0 * T4);
        a5 = (6.0 * delta_q - 3.0 * delta_v * T - 0.5 * delta_acc * T2) / (2.0 * T5);
    }

    /**
     * @brief Evaluates position, velocity, and acceleration at time t.
     */
    TrajectoryPoint evaluate(double t) const {
        if (t < 0.0) t = 0.0;
        if (t > T) t = T;

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        TrajectoryPoint pt;
        pt.pos = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
        pt.vel = a1 + 2.0 * a2 * t + 3.0 * a3 * t2 + 4.0 * a4 * t3 + 5.0 * a5 * t4;
        pt.acc = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t2 + 20.0 * a5 * t3;

        return pt;
    }
};

#endif // QUINTIC_SPLINE_HPP