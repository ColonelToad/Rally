#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#include <mujoco/mujoco.h>
#pragma GCC diagnostic pop

#include <zmq.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <array>
#include <algorithm>

#include "planner/analytical_ik.hpp"
#include "planner/joint_trajectory.hpp"
#include "control/impedance_controller.hpp"
#include "hal/mujoco_emulator.hpp"

mjModel* m = nullptr;
mjData* d = nullptr;
std::atomic<bool> simulation_running(true);

void physics_control_loop(void* zmq_pub) {
    const double dt = 0.002; // 500 Hz
    const int nq = m->nq;
    std::vector<double> qpos_buffer(nq);

    // --- INITIALIZE HAL ---
    hal::MuJoCoEmulator hardware(m, d);
    hal::Sensor* sensor = &hardware;
    hal::Actuator* actuator = &hardware;

    JointTrajectoryManager traj_manager;
    ImpedanceController impedance_ctrl;
    bool trajectory_triggered = false;
    double last_throw_time = -4.0; 

    // Environment variables (Used only for the batting cage hack)
    int ball_qpos_adr = -1;
    int ball_qvel_adr = -1;
    int ball_joint_id = mj_name2id(m, mjOBJ_JOINT, "ball_joint");
    if (ball_joint_id >= 0) {
        ball_qpos_adr = m->jnt_qposadr[ball_joint_id];
        ball_qvel_adr = m->jnt_dofadr[ball_joint_id];
    }

    while (simulation_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        // --- ENVIRONMENT: THE BATTING CAGE ---
        if (ball_qpos_adr >= 0 && (d->time - last_throw_time) > 4.0) {
            d->qpos[ball_qpos_adr] = 2.0; d->qpos[ball_qpos_adr + 1] = 0.0; d->qpos[ball_qpos_adr + 2] = 0.5;
            d->qvel[ball_qvel_adr] = -3.0; d->qvel[ball_qvel_adr + 1] = 0.0; d->qvel[ball_qvel_adr + 2] = 3.5;
            last_throw_time = d->time;
            trajectory_triggered = false;
        }

        // =================================================================
        // --- EMBEDDED FIRMWARE HOT PATH (Zero MuJoCo dependencies) ---
        // =================================================================

        std::array<double, 7> q_curr, dq_curr;
        sensor->readJoints(q_curr, dq_curr);

        std::array<double, 3> ball_pos, ball_vel;
        sensor->readBallState(ball_pos, ball_vel);

        // 1. Prediction & Planning
        if (!trajectory_triggered && ball_vel[0] < -0.1 && ball_pos[0] > 0.5) { 
            double t_intercept = (0.5 - ball_pos[0]) / ball_vel[0];

            if (t_intercept < 1.0 && t_intercept > 0.1) {
                double intercept_y = ball_pos[1] + (ball_vel[1] * t_intercept);
                double intercept_z = ball_pos[2] + (ball_vel[2] * t_intercept) - (0.5 * 9.81 * t_intercept * t_intercept);

                std::array<double, 7> v_zero = {0, 0, 0, 0, 0, 0, 0};
                std::array<double, 7> q_target;

                if (AnalyticalIK::computeIK(0.5, intercept_y, intercept_z, 0.0, q_target)) {
                    traj_manager.startTrajectory(q_curr, v_zero, v_zero, q_target, v_zero, v_zero, t_intercept);
                    trajectory_triggered = true;
                }
            }
        }

        // 2. Trajectory Evaluation
        std::array<double, 7> q_ref = q_curr; 
        std::array<double, 7> v_ref = {0, 0, 0, 0, 0, 0, 0};
        std::array<double, 7> a_ref = {0, 0, 0, 0, 0, 0, 0};

        if (traj_manager.isActive()) {
            traj_manager.update(dt, q_ref, v_ref, a_ref);
        }
        
        // 3. Impedance Control & Actuation
        std::array<double, 7> commanded_torques = impedance_ctrl.computeTorques(q_curr, dq_curr, q_ref, v_ref);
        actuator->writeTorques(commanded_torques);

        // =================================================================
        // --- END EMBEDDED FIRMWARE HOT PATH ---
        // =================================================================

        // Step Physics & Publish Telemetry
        mj_step(m, d);
        std::copy(d->qpos, d->qpos + nq, qpos_buffer.begin());
        zmq_send(zmq_pub, qpos_buffer.data(), nq * sizeof(double), ZMQ_DONTWAIT);

        // Enforce 500Hz Timing
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        if (elapsed.count() < dt) {
            std::this_thread::sleep_for(std::chrono::duration<double>(dt - elapsed.count()));
        }
    }
}

int main(int /*argc*/, char** /*argv*/) {
    char error[1000] = "Could not load XML model";
    m = mj_loadXML("panda_hit_scene.xml", nullptr, error, 1000);
    if (!m) {
        std::cerr << "MuJoCo Load Error: " << error << std::endl;
        return 1;
    }
    d = mj_makeData(m);

    void* context = zmq_ctx_new();
    void* publisher = zmq_socket(context, ZMQ_PUB);
    zmq_bind(publisher, "tcp://*:5556");

    std::cout << "[Bridge] Starting 500Hz Physics Publisher with HAL..." << std::endl;
    std::thread physics_thread(physics_control_loop, publisher);

    std::cout << "[Bridge] Press Enter to stop." << std::endl;
    std::cin.get();

    simulation_running.store(false);
    physics_thread.join();
    zmq_close(publisher);
    zmq_ctx_term(context);
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}