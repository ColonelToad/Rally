#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm> // for std::max/min
#include <Eigen/Dense>
#include <mujoco/mujoco.h>

using namespace Eigen;
using namespace std;

// 1. Franka Physical Constants (Lengths in meters)
const double d1 = 0.333;
const double d3 = 0.316;
const double d5 = 0.384;
const double df = 0.107; // Flange offset

// 2. The Analytical IK Solver
// Returns a vector of 7 joint angles.
vector<double> compute_analytical_ik(Matrix4d T_target) {
    vector<double> q(7, 0.0);
    
    // ---------------------------------------------------------
    // Step 1: Find the Wrist Center (P_wc)
    // ---------------------------------------------------------
    Vector3d p_end = T_target.block<3,1>(0,3);
    Vector3d z_end = T_target.block<3,1>(0,2);
    Vector3d p_wc = p_end - df * z_end; 

    // ---------------------------------------------------------
    // Step 2: Solve q4 (The Elbow) using the Law of Cosines
    // ---------------------------------------------------------
    // Distance from shoulder frame (z = d1) to wrist center
    double x_w = p_wc(0);
    double y_w = p_wc(1);
    double z_w = p_wc(2) - d1; 
    
    // Hypotenuse squared of the shoulder-to-wrist triangle
    double R_sq = x_w * x_w + y_w * y_w + z_w * z_w;
    
    // Law of cosines: c^2 = a^2 + b^2 - 2ab*cos(C)
    double D = (R_sq - d3 * d3 - d5 * d5) / (2 * d3 * d5);
    
    // Clamp D to [-1, 1] to avoid NaN if the target is slightly out of reach
    D = max(-1.0, min(1.0, D));
    
    // Franka elbow typically bends in the negative direction (elbow down)
    q[3] = -acos(D); 

    // ---------------------------------------------------------
    // Step 3: Solve q1, q2, q3 (The Shoulder)
    // ---------------------------------------------------------
    // Point the base at the wrist center
    q[0] = atan2(y_w, x_w);
    
    // Calculate pitch (q2). 
    // r is the planar distance, alpha is the elevation angle to the wrist.
    double r = sqrt(x_w * x_w + y_w * y_w);
    double alpha = atan2(z_w, r);
    
    // beta is the inner angle of the triangle at the shoulder
    double beta = acos((R_sq + d3 * d3 - d5 * d5) / (2 * sqrt(R_sq) * d3));
    
    // Franka's q2 zero-pose is straight up. 
    q[1] = (M_PI / 2.0) - (alpha + beta); 
    
    // Lock the redundant arm angle to 0 for this analytical solution
    q[2] = 0.0; 

    // ---------------------------------------------------------
    // Step 4: Solve q5, q6, q7 (The Wrist Orientation)
    // ---------------------------------------------------------
    // Compute Forward Kinematics from Base to Link 4 (the elbow)
    // Approximate standard rotation axes: q1(Z), q2(Y), q3(Z), q4(-Y)
    Matrix3d R_0_4 = AngleAxisd(q[0], Vector3d::UnitZ()).toRotationMatrix() *
                     AngleAxisd(q[1], Vector3d::UnitY()).toRotationMatrix() *
                     AngleAxisd(q[2], Vector3d::UnitZ()).toRotationMatrix() *
                     AngleAxisd(q[3], -Vector3d::UnitY()).toRotationMatrix();

    // The desired wrist rotation is the remaining rotation needed from Link 4
    Matrix3d R_target = T_target.block<3,3>(0,0);
    Matrix3d R_4_7 = R_0_4.transpose() * R_target;

    // Franka's wrist acts roughly as a Z-Y-Z Euler sequence
    Vector3d euler = R_4_7.eulerAngles(2, 1, 2);
    
    q[4] = euler[0];
    q[5] = euler[1];
    q[6] = euler[2];

    return q;
}

// 3. Simple test main
int main() {
    // Define a simple target pose: identity rotation, some position
    Matrix4d T_target = Matrix4d::Identity();
    T_target(0, 3) = 0.5;  // x
    T_target(1, 3) = 0.0;  // y
    T_target(2, 3) = 0.5;  // z

    vector<double> q = compute_analytical_ik(T_target);

    cout << "Computed joint angles:" << endl;
    for (size_t i = 0; i < q.size(); ++i) {
        cout << "q" << (i + 1) << " = " << q[i] << endl;
    }

    return 0;
}