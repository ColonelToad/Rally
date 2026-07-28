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
    FixedPointScalar operator*(const FixedPointScalar& o) const { 
        FixedPointScalar r; 
        r.raw_ = static_cast<int64_t>((static_cast<__int128>(raw_) * o.raw_) >> 32); 
        return r; 
    }
    FixedPointScalar operator/(const FixedPointScalar& o) const { 
        FixedPointScalar r; 
        r.raw_ = static_cast<int64_t>((static_cast<__int128>(raw_) << 32) / o.raw_); 
        return r; 
    }

    bool operator>(double val) const { return to_double() > val; }
    bool operator<(double val) const { return to_double() < val; }
    bool operator>=(double val) const { return to_double() >= val; }

    friend FixedPointScalar sqrt(FixedPointScalar val) { return FixedPointScalar(std::sqrt(val.to_double())); }

private:
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
};

} // namespace core
} // namespace rally