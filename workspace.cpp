#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

// 1. The DH Matrix Function
Matrix4d compute_dh_matrix(double a, double alpha, double d, double theta) {
    Matrix4d T;
    T << cos(theta),             -sin(theta),              0,           a,
         sin(theta)*cos(alpha),   cos(theta)*cos(alpha),  -sin(alpha), -d * sin(alpha),
         sin(theta)*sin(alpha),   cos(theta)*sin(alpha),   cos(alpha),  d * cos(alpha),
         0,                       0,                       0,           1;
    return T;
}

int main() {
    // 2. Franka Panda Joint Limits (in radians)
    // q1, q2, q3, q4, q5, q6, q7
    double q_min[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    double q_max[7] = { 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};

    // 3. DH Parameters
    double a[7]        = {0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088};
    double alpha[7]    = {0.0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2};
    double d_offset[7] = {0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0};

    // 4. Setup CSV Output
    ofstream csv_file("workspace.csv");
    csv_file << "x,y,z\n";

    int steps = 6; // Number of angles to test per joint. 6^7 = ~280k points.
    
    cout << "Calculating workspace point cloud... " << endl;

    // 5. The Combinatorics Loop (7 nested loops for 7 joints)
    for(int i0 = 0; i0 < steps; i0++) {
        double q1 = q_min[0] + i0 * (q_max[0] - q_min[0]) / (steps - 1);
        Matrix4d T1 = compute_dh_matrix(a[0], alpha[0], d_offset[0], q1);

        for(int i1 = 0; i1 < steps; i1++) {
            double q2 = q_min[1] + i1 * (q_max[1] - q_min[1]) / (steps - 1);
            Matrix4d T2 = T1 * compute_dh_matrix(a[1], alpha[1], d_offset[1], q2);

            for(int i2 = 0; i2 < steps; i2++) {
                double q3 = q_min[2] + i2 * (q_max[2] - q_min[2]) / (steps - 1);
                Matrix4d T3 = T2 * compute_dh_matrix(a[2], alpha[2], d_offset[2], q3);

                for(int i3 = 0; i3 < steps; i3++) {
                    double q4 = q_min[3] + i3 * (q_max[3] - q_min[3]) / (steps - 1);
                    Matrix4d T4 = T3 * compute_dh_matrix(a[3], alpha[3], d_offset[3], q4);

                    for(int i4 = 0; i4 < steps; i4++) {
                        double q5 = q_min[4] + i4 * (q_max[4] - q_min[4]) / (steps - 1);
                        Matrix4d T5 = T4 * compute_dh_matrix(a[4], alpha[4], d_offset[4], q5);

                        for(int i5 = 0; i5 < steps; i5++) {
                            double q6 = q_min[5] + i5 * (q_max[5] - q_min[5]) / (steps - 1);
                            Matrix4d T6 = T5 * compute_dh_matrix(a[5], alpha[5], d_offset[5], q6);

                            for(int i6 = 0; i6 < steps; i6++) {
                                double q7 = q_min[6] + i6 * (q_max[6] - q_min[6]) / (steps - 1);
                                Matrix4d T7 = T6 * compute_dh_matrix(a[6], alpha[6], d_offset[6], q7);

                                // Add the flange offset
                                Matrix4d T_flange = Matrix4d::Identity();
                                T_flange(2, 3) = 0.107;
                                Matrix4d T_final = T7 * T_flange;

                                // Write to CSV
                                csv_file << T_final(0,3) << "," << T_final(1,3) << "," << T_final(2,3) << "\n";
                            }
                        }
                    }
                }
            }
        }
    }

    csv_file.close();
    cout << "Done! Saved to workspace.csv" << endl;
    
    return 0;
}