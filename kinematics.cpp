#include <iostream>
#include <vector>
#include <cmath>

// MuJoCo and Eigen Headers
#include <mujoco/mujoco.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

// Helper function to create a 4x4 transformation matrix from DH parameters
Matrix4d compute_dh_matrix(double a, double alpha, double d, double theta) {
    Matrix4d T;
    T << cos(theta),             -sin(theta),              0,           a,
         sin(theta)*cos(alpha),   cos(theta)*cos(alpha),  -sin(alpha), -d * sin(alpha),
         sin(theta)*sin(alpha),   cos(theta)*sin(alpha),   cos(alpha),  d * cos(alpha),
         0,                       0,                       0,           1;
    return T;
}

int main() {
    // ---------------------------------------------------------
    // 1. MUJOCO SETUP
    // ---------------------------------------------------------
    // Point this to your Menagerie scene
    const char* xml_path = "../mujoco_menagerie/franka_emika_panda/scene.xml";
    
    char error[1000] = "Could not load XML model";
    mjModel* m = mj_loadXML(xml_path, 0, error, 1000);
    if (!m) {
        cerr << "Load model error: " << error << endl;
        return 1;
    }
    
    mjData* d = mj_makeData(m);

    // ---------------------------------------------------------
    // 2. SET A TEST CONFIGURATION
    // ---------------------------------------------------------
    // Franka has 7 joints. Let's set them to a non-zero pose to test.
    double test_angles[7] = {0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4};
    
    // In the raw Menagerie XML, joint names are usually "joint1", "joint2", etc.
    for (int i = 0; i < 7; ++i) {
        string joint_name = "joint" + to_string(i + 1);
        int jnt_id = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
        if (jnt_id != -1) {
            d->qpos[m->jnt_qposadr[jnt_id]] = test_angles[i];
        }
    }

    // Run forward kinematics in MuJoCo based on the angles we just set
    mj_kinematics(m, d);

    // ---------------------------------------------------------
    // 3. YOUR EIGEN FORWARD KINEMATICS
    // ---------------------------------------------------------
    double a[7]     = {0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088};
    double alpha[7] = {0.0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2};
    double d_offset[7]     = {0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0};

    Matrix4d T_0_7 = Matrix4d::Identity();

    for (int i = 0; i < 7; ++i) {
        Matrix4d T_i = compute_dh_matrix(a[i], alpha[i], d_offset[i], test_angles[i]);
        T_0_7 = T_0_7 * T_i;
    }

    Matrix4d T_flange = Matrix4d::Identity();
    T_flange(2, 3) = 0.107; 

    Matrix4d T_final = T_0_7 * T_flange;

    // ---------------------------------------------------------
    // 4. VERIFICATION
    // ---------------------------------------------------------
    cout << "--- Forward Kinematics Verification ---" << endl;
    
    // Look up the flange body (link8) instead of the missing gripper site
    int ee_body_id = mj_name2id(m, mjOBJ_BODY, "link7"); 
    
    if (ee_body_id != -1) {
        cout << "MuJoCo EE Position (x,y,z): " 
             << d->xpos[3*ee_body_id] << ", "
             << d->xpos[3*ee_body_id + 1] << ", "
             << d->xpos[3*ee_body_id + 2] << endl;
    } else {
        cout << "Warning: Could not find end effector body in MuJoCo." << endl;
    }

    cout << "Eigen  EE Position (x,y,z): " 
         << T_final(0,3) << ", " 
         << T_final(1,3) << ", " 
         << T_final(2,3) << endl;

    // Cleanup
    mj_deleteData(d);
    mj_deleteModel(m);
    
    return 0;
}