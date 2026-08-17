#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace rally {
namespace core {

#ifdef FIXED_POINT
// Simple 32.32 fixed-point representation wrapper for Eigen compatibility
class FixedPointScalar {
public:
    constexpr FixedPointScalar() : raw_(0) {}
    constexpr FixedPointScalar(double val) : raw_(static_cast<int64_t>(val * 4294967296.0)) {} // 32.32 format

    double to_double() const { return static_cast<double>(raw_) / 4294967296.0; }

    FixedPointScalar operator+(const FixedPointScalar& o) const { FixedPointScalar r; r.raw_ = raw_ + o.raw_; return r; }
    FixedPointScalar operator-(const FixedPointScalar& o) const { FixedPointScalar r; r.raw_ = raw_ - o.raw_; return r; }
    FixedPointScalar operator-() const { FixedPointScalar r; r.raw_ = -raw_; return r; }
    FixedPointScalar operator*(const FixedPointScalar& o) const {
        FixedPointScalar r;
        r.raw_ = mul64(raw_, o.raw_);
        return r;
    }
    FixedPointScalar operator/(const FixedPointScalar& o) const {
        FixedPointScalar r;
        r.raw_ = div64(raw_, o.raw_);
        return r;
    }

    FixedPointScalar& operator+=(const FixedPointScalar& o) { raw_ += o.raw_; return *this; }
    FixedPointScalar& operator-=(const FixedPointScalar& o) { raw_ -= o.raw_; return *this; }
    FixedPointScalar& operator*=(const FixedPointScalar& o) { raw_ = mul64(raw_, o.raw_); return *this; }
    FixedPointScalar& operator/=(const FixedPointScalar& o) { raw_ = div64(raw_, o.raw_); return *this; }

    bool operator==(const FixedPointScalar& o) const { return raw_ == o.raw_; }
    bool operator!=(const FixedPointScalar& o) const { return raw_ != o.raw_; }
    bool operator>(double val) const { return to_double() > val; }
    bool operator<(double val) const { return to_double() < val; }
    bool operator>=(double val) const { return to_double() >= val; }

    friend FixedPointScalar sqrt(FixedPointScalar val) { return FixedPointScalar(std::sqrt(val.to_double())); }
    friend std::ostream& operator<<(std::ostream& os, const FixedPointScalar& v) { return os << v.to_double(); }

private:
    // 64x64->128-bit widening multiply/divide without __int128 (an ISO
    // C++ extension that -Wpedantic -Werror rejects). Splits each 64-bit
    // operand into high/low 32-bit halves and combines via standard
    // 64-bit arithmetic, which is portable across compilers/targets —
    // notably including the embedded ARM toolchains this scalar type
    // exists for in the first place (docs/EMBEDDED_NOTES.md).
    static int64_t mul64(int64_t a, int64_t b) {
        bool neg = (a < 0) != (b < 0);
        uint64_t ua = a < 0 ? static_cast<uint64_t>(-a) : static_cast<uint64_t>(a);
        uint64_t ub = b < 0 ? static_cast<uint64_t>(-b) : static_cast<uint64_t>(b);

        uint64_t a_hi = ua >> 32, a_lo = ua & 0xFFFFFFFFu;
        uint64_t b_hi = ub >> 32, b_lo = ub & 0xFFFFFFFFu;

        uint64_t lo_lo = a_lo * b_lo;
        uint64_t hi_lo = a_hi * b_lo;
        uint64_t lo_hi = a_lo * b_hi;
        uint64_t hi_hi = a_hi * b_hi;

        // Full 128-bit product = (hi_hi << 64) + (hi_lo + lo_hi) << 32 + lo_lo.
        // We only need bits [32, 96) of that product (>>32 of a 128-bit
        // value, truncated to 64 bits), which is what the >>32 in the
        // original 32.32 fixed-point multiply extracts.
        uint64_t mid = hi_lo + lo_hi;
        uint64_t result = hi_hi * (uint64_t(1) << 32) + mid + (lo_lo >> 32);
        int64_t signed_result = static_cast<int64_t>(result);
        return neg ? -signed_result : signed_result;
    }

    static int64_t div64(int64_t a, int64_t b) {
        // Q32.32 divide via double-precision intermediate. Loses some
        // precision relative to a true 128-bit integer divide, but avoids
        // __int128 entirely; acceptable for this scalar's purpose (an EKF
        // running comfortably inside double's mantissa range).
        double da = static_cast<double>(a) / 4294967296.0;
        double db = static_cast<double>(b) / 4294967296.0;
        return static_cast<int64_t>((da / db) * 4294967296.0);
    }

    int64_t raw_;
};
using RealScalar = FixedPointScalar;
#else
using RealScalar = double;
#endif

class EkfBallPredictor {
public:
    EkfBallPredictor(double dt = 0.002, double drag_coeff = 0.01)
        : dt_(dt), cd_(drag_coeff) {
        x_ = Eigen::Matrix<RealScalar, 6, 1>::Zero();
        P_ = Eigen::Matrix<RealScalar, 6, 6>::Identity() * RealScalar(1.0);
        Q_ = Eigen::Matrix<RealScalar, 6, 6>::Identity() * RealScalar(0.001); // Process noise
        R_ = Eigen::Matrix<RealScalar, 3, 3>::Identity() * RealScalar(0.005); // Measurement noise
    }

    void initialize_state(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) {
        x_(0) = pos(0); x_(1) = pos(1); x_(2) = pos(2);
        x_(3) = vel(0); x_(4) = vel(1); x_(5) = vel(2);
    }

    void predict() {
        double px = get_double(x_(0));
        double py = get_double(x_(1));
        double pz = get_double(x_(2));
        double vx = get_double(x_(3));
        double vy = get_double(x_(4));
        double vz = get_double(x_(5));

        double v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);
        double safe_v_norm = (v_norm < 1e-5) ? 1e-5 : v_norm;

        // 1. Nonlinear state propagation
        double ax = -cd_ * safe_v_norm * vx;
        double ay = -cd_ * safe_v_norm * vy;
        double az = -9.81 - (cd_ * safe_v_norm * vz);

        x_(0) = RealScalar(px + vx * dt_);
        x_(1) = RealScalar(py + vy * dt_);
        x_(2) = RealScalar(pz + vz * dt_);
        x_(3) = RealScalar(vx + ax * dt_);
        x_(4) = RealScalar(vy + ay * dt_);
        x_(5) = RealScalar(vz + az * dt_);

        // 2. Compute State Transition Jacobian F (6x6)
        Eigen::Matrix<RealScalar, 6, 6> F = Eigen::Matrix<RealScalar, 6, 6>::Identity();
        
        F(0, 3) = RealScalar(dt_);
        F(1, 4) = RealScalar(dt_);
        F(2, 5) = RealScalar(dt_);

        double term = -cd_ / safe_v_norm;
        F(3, 3) = RealScalar(1.0 + term * (vx * vx + safe_v_norm * safe_v_norm) * dt_);
        F(3, 4) = RealScalar(term * (vx * vy) * dt_);
        F(3, 5) = RealScalar(term * (vx * vz) * dt_);

        F(4, 3) = RealScalar(term * (vy * vx) * dt_);
        F(4, 4) = RealScalar(1.0 + term * (vy * vy + safe_v_norm * safe_v_norm) * dt_);
        F(4, 5) = RealScalar(term * (vy * vz) * dt_);

        F(5, 3) = RealScalar(term * (vz * vx) * dt_);
        F(5, 4) = RealScalar(term * (vz * vy) * dt_);
        F(5, 5) = RealScalar(1.0 + term * (vz * vz + safe_v_norm * safe_v_norm) * dt_);

        // 3. Covariance prediction
        P_ = F * P_ * F.transpose() + Q_;
    }

    void update(const Eigen::Vector3d& measured_pos) {
        Eigen::Matrix<RealScalar, 3, 6> H = Eigen::Matrix<RealScalar, 3, 6>::Zero();
        H(0, 0) = RealScalar(1.0);
        H(1, 1) = RealScalar(1.0);
        H(2, 2) = RealScalar(1.0);

        Eigen::Matrix<RealScalar, 3, 1> z;
        z(0) = RealScalar(measured_pos(0));
        z(1) = RealScalar(measured_pos(1));
        z(2) = RealScalar(measured_pos(2));

        Eigen::Matrix<RealScalar, 3, 1> z_pred = H * x_;
        Eigen::Matrix<RealScalar, 3, 1> y = z - z_pred;

        Eigen::Matrix<RealScalar, 3, 3> S = H * P_ * H.transpose() + R_;
        Eigen::Matrix<RealScalar, 6, 3> K = P_ * H.transpose() * S.inverse();

        x_ = x_ + K * y;

        Eigen::Matrix<RealScalar, 6, 6> I = Eigen::Matrix<RealScalar, 6, 6>::Identity();
        P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R_ * K.transpose();

        last_innovation_norm_ = std::sqrt(get_double(y(0)) * get_double(y(0)) +
                                           get_double(y(1)) * get_double(y(1)) +
                                           get_double(y(2)) * get_double(y(2)));
    }

    // For observability (PHILOSOPHY.md Principle 6): the measurement
    // residual magnitude from the most recent update() call.
    double last_innovation_norm() const { return last_innovation_norm_; }

    // Trace of the state covariance — a scalar proxy for "how confident
    // is the filter right now," logged alongside innovation.
    double covariance_trace() const {
        double trace = 0.0;
        for (int i = 0; i < 6; ++i) trace += get_double(P_(i, i));
        return trace;
    }

    Eigen::Vector2d predict_landing_point() const {
        double px = get_double(x_(0));
        double py = get_double(x_(1));
        double pz = get_double(x_(2));
        double vx = get_double(x_(3));
        double vy = get_double(x_(4));
        double vz = get_double(x_(5));

        double sim_dt = 0.005;
        while (pz > 0.0 && pz < 100.0) {
            double v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);
            px += vx * sim_dt;
            py += vy * sim_dt;
            pz += vz * sim_dt;
            vx += (-cd_ * v_norm * vx) * sim_dt;
            vy += (-cd_ * v_norm * vy) * sim_dt;
            vz += (-9.81 - (cd_ * v_norm * vz)) * sim_dt;
        }

        return Eigen::Vector2d(px, py);
    }

    // Forward-simulates the filtered state (same drag-aware integration as
    // predict_landing_point) until the ball's X position crosses target_x,
    // returning (y, z, time_to_cross) at that plane — the query an arm
    // needs for interception, distinct from "where does it land."
    // valid is false if the ball is moving away from target_x or the plane
    // is never crossed within a reasonable time horizon.
    struct PlaneCrossing {
        double y;
        double z;
        double time_to_cross;
        bool valid;
    };

    PlaneCrossing predict_plane_crossing(double target_x) const {
        double px = get_double(x_(0));
        double py = get_double(x_(1));
        double pz = get_double(x_(2));
        double vx = get_double(x_(3));
        double vy = get_double(x_(4));
        double vz = get_double(x_(5));

        bool approaching = (target_x > px) ? (vx > 0.0) : (vx < 0.0);
        if (!approaching) {
            return {0.0, 0.0, 0.0, false};
        }

        double sim_dt = 0.002;
        double elapsed = 0.0;
        const double kMaxHorizon = 1.5;
        bool started_below = px < target_x;

        while (elapsed < kMaxHorizon) {
            double v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);
            px += vx * sim_dt;
            py += vy * sim_dt;
            pz += vz * sim_dt;
            vx += (-cd_ * v_norm * vx) * sim_dt;
            vy += (-cd_ * v_norm * vy) * sim_dt;
            vz += (-9.81 - (cd_ * v_norm * vz)) * sim_dt;
            elapsed += sim_dt;

            bool now_below = px < target_x;
            if (now_below != started_below) {
                return {py, pz, elapsed, true};
            }
        }

        return {0.0, 0.0, 0.0, false};
    }

private:
    double get_double(RealScalar val) const {
#ifdef FIXED_POINT
        return val.to_double();
#else
        return val;
#endif
    }

    double dt_;
    double cd_;
    Eigen::Matrix<RealScalar, 6, 1> x_;
    Eigen::Matrix<RealScalar, 6, 6> P_;
    Eigen::Matrix<RealScalar, 6, 6> Q_;
    Eigen::Matrix<RealScalar, 3, 3> R_;
    double last_innovation_norm_ = 0.0;
};

} // namespace core
} // namespace rally