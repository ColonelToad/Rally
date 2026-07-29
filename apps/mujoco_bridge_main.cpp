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

mjModel* m = nullptr;
mjData* d = nullptr;
std::atomic<bool> simulation_running(true);

void physics_control_loop(void* zmq_pub) {
    const double dt = 0.002; // 500 Hz
    const int nq = m->nq;
    std::vector<double> qpos_buffer(nq);

    JointTrajectoryManager traj_manager;
    ImpedanceController impedance_ctrl;
    bool trajectory_triggered = false;

    // SAFELY Find the ball's memory addresses in MuJoCo
    int ball_joint_id = mj_name2id(m, mjOBJ_JOINT, "ball_joint");
    int ball_qpos_adr = -1;
    int ball_qvel_adr = -1;
    
    if (ball_joint_id >= 0) {
        ball_qpos_adr = m->jnt_qposadr[ball_joint_id];
        ball_qvel_adr = m->jnt_dofadr[ball_joint_id];
    } else {
        std::cerr << "[Warning] 'ball_joint' not found in physics loop!" << std::endl;
    }

    // Add a timer variable right before the loop
    double last_throw_time = -4.0; // Start negative so it throws immediately at t=0

    while (simulation_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        // --- THE BATTING CAGE: THROW EVERY 4 SECONDS ---
        if (ball_qpos_adr >= 0 && (d->time - last_throw_time) > 4.0) {
            // Reset position to the starting point
            d->qpos[ball_qpos_adr]     = 2.0; // X
            d->qpos[ball_qpos_adr + 1] = 0.0; // Y
            d->qpos[ball_qpos_adr + 2] = 0.5; // Z
            
            // Reset velocity (The Pitch)
            d->qvel[ball_qvel_adr]     = -3.0; // vx
            d->qvel[ball_qvel_adr + 1] = 0.0;  // vy
            d->qvel[ball_qvel_adr + 2] = 3.5;  // vz
            
            last_throw_time = d->time;
            trajectory_triggered = false; // Reset the arm so it can swing again
            std::cout << "[Bridge] Pitching ball!" << std::endl;
        }

        // --- THE BRAIN: PREDICT & SWING ---
        if (!trajectory_triggered && ball_qpos_adr >= 0) {
            double bx = d->qpos[ball_qpos_adr];
            double by = d->qpos[ball_qpos_adr + 1];
            double bz = d->qpos[ball_qpos_adr + 2];
            
            double vx = d->qvel[ball_qvel_adr];
            double vy = d->qvel[ball_qvel_adr + 1];
            double vz = d->qvel[ball_qvel_adr + 2];

            double intercept_x = 0.5;

            if (vx < -0.1 && bx > intercept_x) { 
                double t_intercept = (intercept_x - bx) / vx;

                if (t_intercept < 1.0 && t_intercept > 0.1) {
                    double intercept_y = by + (vy * t_intercept);
                    double intercept_z = bz + (vz * t_intercept) - (0.5 * 9.81 * t_intercept * t_intercept);

                    std::array<double, 7> q_current = {d->qpos[0], d->qpos[1], d->qpos[2], d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]};
                    std::array<double, 7> v_zero = {0, 0, 0, 0, 0, 0, 0};
                    std::array<double, 7> q_target;

                    if (AnalyticalIK::computeIK(intercept_x, intercept_y, intercept_z, 0.0, q_target)) {
                        traj_manager.startTrajectory(q_current, v_zero, v_zero, q_target, v_zero, v_zero, t_intercept);
                        trajectory_triggered = true;
                        std::cout << "[Bridge] Intercepting at Z: " << intercept_z << " in " << t_intercept << "s!" << std::endl;
                    }
                }
            }
        }

        // --- TRAJECTORY EVALUATION ---
        std::array<double, 7> q_ref, v_ref, a_ref;
        for(int i=0; i<7; ++i) { q_ref[i] = d->qpos[i]; v_ref[i] = 0.0; a_ref[i] = 0.0; } 

        if (traj_manager.isActive()) {
            traj_manager.update(dt, q_ref, v_ref, a_ref);
        }
        
        // --- IMPEDANCE CONTROL ---
        impedance_ctrl.computeTorques(m, d, q_ref, v_ref);

        // --- STEP PHYSICS & PUBLISH ---
        mj_step(m, d);
        std::copy(d->qpos, d->qpos + nq, qpos_buffer.begin());
        zmq_send(zmq_pub, qpos_buffer.data(), nq * sizeof(double), ZMQ_DONTWAIT);

        // --- TIMING ---
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        if (elapsed.count() < dt) {
            std::this_thread::sleep_for(std::chrono::duration<double>(dt - elapsed.count()));
        }
    }
}

int main(int argc, char** argv) {
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

    std::cout << "[Bridge] Starting 500Hz Physics Publisher..." << std::endl;
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