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

#include "planner/ownership_arbiter.hpp"
#include "planner/analytical_ik.hpp"
#include "planner/joint_trajectory.hpp"
#include "control/impedance_controller.hpp"
#include "hal/mujoco_emulator.hpp"
#include "ipc/lock_free_state_buffer.hpp"

struct AtomicTorqueArray {
    std::array<std::atomic<double>, 7> tau;
    
    AtomicTorqueArray() {
        for (int i = 0; i < 7; ++i) tau[i].store(0.0);
    }
    
    void write(const std::array<double, 7>& commanded) {
        for (int i = 0; i < 7; ++i) tau[i].store(commanded[i], std::memory_order_relaxed);
    }
    
    std::array<double, 7> read() const {
        std::array<double, 7> current;
        for (int i = 0; i < 7; ++i) current[i] = tau[i].load(std::memory_order_relaxed);
        return current;
    }
};

struct AtomicJointState {
    std::array<std::atomic<double>, 7> q;
    std::array<std::atomic<double>, 7> dq;
    
    AtomicJointState() {
        // Initialize with default home positions to prevent start-up jumps
        std::array<double, 7> q_home = {0.0, -0.785, 0.0, -2.356, 0.0, 1.578, 0.785};
        for (int i = 0; i < 7; ++i) { 
            q[i].store(q_home[i]); 
            dq[i].store(0.0); 
        }
    }
    
    void write(const std::array<double, 7>& current_q, const std::array<double, 7>& current_dq) {
        for (int i = 0; i < 7; ++i) {
            q[i].store(current_q[i], std::memory_order_relaxed);
            dq[i].store(current_dq[i], std::memory_order_relaxed);
        }
    }
    
    void read(std::array<double, 7>& out_q, std::array<double, 7>& out_dq) const {
        for (int i = 0; i < 7; ++i) {
            out_q[i] = q[i].load(std::memory_order_relaxed);
            out_dq[i] = dq[i].load(std::memory_order_relaxed);
        }
    }
};

// Global state buffers for the joints
AtomicJointState left_arm_state;
AtomicJointState right_arm_state;

// Global Architecture State
mjModel* m = nullptr;
mjData* d = nullptr;
std::atomic<bool> simulation_running(true);

LockFreeStateBuffer<16> shared_ball_buffer;
AtomicTorqueArray left_arm_torques;
AtomicTorqueArray right_arm_torques;

// =================================================================
// THREAD 1 & 2: INDEPENDENT ARM CONTROLLERS (500 Hz)
// =================================================================
void arm_control_loop(const std::string& arm_name, AtomicTorqueArray& torque_out) {
    const double dt = 0.002; // 500Hz
    ImpedanceController impedance_ctrl;
    AnalyticalIK analytical_ik; 
    
    // Instantiate a lock-free, thread-local arbiter with a 5cm deadband
    OwnershipArbiter arbiter(0.05); 
    
    std::cout << "[Controller] " << arm_name << " thread started." << std::endl;
    int tick_counter = 0;

    std::array<double, 7> q_home = {0.0, -0.785, 0.0, -2.356, 0.0, 1.578, 0.785};
    bool is_left_arm = (arm_name.find("Left") != std::string::npos);

    while (simulation_running.load(std::memory_order_acquire)) {
        auto start_time = std::chrono::steady_clock::now();

        BallState ball = shared_ball_buffer.getLatest();
        
        std::array<double, 7> q_ref = q_home;
        std::array<double, 7> v_ref = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        if (ball.valid) {
            
            // 1. Pass ball state into the arbiter
            ArmOwner owner = arbiter.arbitrate(arm_name, ball.pos, ball.vel, ball.timestamp);
            
            // 2. Determine if this specific thread owns the ball
            bool am_i_owner = (is_left_arm && owner == ArmOwner::LEFT_ARM) || 
                              (!is_left_arm && owner == ArmOwner::RIGHT_ARM);

            if (am_i_owner) {
                // --- ACTIVE INTERCEPTION LOGIC ---
                double target_x = is_left_arm ? -0.5 : 0.5;
                bool moving_towards_us = is_left_arm ? (ball.vel[0] < 0.0) : (ball.vel[0] > 0.0);

                if (moving_towards_us && std::abs(ball.vel[0]) > 0.01) {
                    double time_to_impact = (target_x - ball.pos[0]) / ball.vel[0];

                    if (time_to_impact > 0.0 && time_to_impact < 1.5) { 
                        const double g = -9.81;
                        double target_y = ball.pos[1] + (ball.vel[1] * time_to_impact);
                        double target_z = ball.pos[2] + (ball.vel[2] * time_to_impact) + (0.5 * g * time_to_impact * time_to_impact);

                        target_y = std::clamp(target_y, -0.6, 0.6);
                        target_z = std::clamp(target_z, 0.1, 1.2);

                        std::array<double, 7> ik_solution;
                        double redundant_q3 = 0.0; 

                        if (analytical_ik.computeIK(target_x, target_y, target_z, redundant_q3, ik_solution)) {
                            q_ref = ik_solution; 
                        }
                    }
                }
            } else {
                // --- TACTICAL IDLE REPOSITIONING ---
                // Instead of freezing in place, the non-owning arm actively prepares 
                // for the return shot by tracking the ball's Y-axis with its base joint.
                q_ref = q_home;
                
                // Track ball Y softly (scaled down to prevent aggressive swinging)
                double y_tracking_offset = ball.pos[1] * 0.3;
                q_ref[0] = q_home[0] + y_tracking_offset; 
            }
        }

        // 5. Read TRUE current joint states from the atomic buffer
        std::array<double, 7> q_curr; 
        std::array<double, 7> dq_curr;
        if (is_left_arm) {
            left_arm_state.read(q_curr, dq_curr);
        } else {
            right_arm_state.read(q_curr, dq_curr);
        }

        // 6. Compute Torques via Impedance Controller
        std::array<double, 7> tau = impedance_ctrl.computeTorques(q_curr, dq_curr, q_ref, v_ref);
        
        torque_out.write(tau);
        tick_counter++;

        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        if (elapsed.count() < dt) {
            std::this_thread::sleep_for(std::chrono::duration<double>(dt - elapsed.count()));
        }
    }
}

// =================================================================
// THREAD 0: HAL SENSOR / ACTUATOR & PHYSICS ENGINE (1000 Hz)
// =================================================================
void physics_hal_loop(void* zmq_pub, void* zmq_cmd_sub) {
    const double dt = 0.001; // 1kHz Physics & Sensor polling
    const int nq = m->nq;
    std::vector<double> qpos_buffer(nq);

    hal::MuJoCoEmulator hardware(m, d);
    hal::Sensor* sensor = &hardware;
    
    std::cout << "[HAL] Physics thread started at 1kHz." << std::endl;

    while (simulation_running.load(std::memory_order_acquire)) {
        auto start_time = std::chrono::steady_clock::now();

        char cmd_buffer[256] = {0};
        if (zmq_recv(zmq_cmd_sub, cmd_buffer, 255, ZMQ_DONTWAIT) > 0) {
            std::string cmd(cmd_buffer);
            if (cmd.find("SERVE") != std::string::npos) {
                int ball_joint_id = mj_name2id(m, mjOBJ_JOINT, "ball_joint");
                if (ball_joint_id >= 0) {
                    int qpos_adr = m->jnt_qposadr[ball_joint_id];
                    int dof_adr = m->jnt_dofadr[ball_joint_id];
                    
                    d->qpos[qpos_adr]     =  1.2;  
                    d->qpos[qpos_adr + 1] =  0.0;  
                    d->qpos[qpos_adr + 2] =  1.2;  
                    d->qvel[dof_adr]     = -3.5;  
                    d->qvel[dof_adr + 1] =  0.2;  
                    d->qvel[dof_adr + 2] =  1.0;
                    std::cout << "[HAL] Manual serve triggered via ZMQ!" << std::endl;
                }
            }
        }

        // 1. Read Sensors and push to IPC Buffer
        std::array<double, 3> ball_pos, ball_vel;
        sensor->readBallState(ball_pos, ball_vel);
        
        BallState current_state;
        current_state.pos = ball_pos;
        current_state.vel = ball_vel;
        // Grab the exact simulation time for the readers to verify
        current_state.timestamp = d->time; 
        current_state.valid = true;
        
        shared_ball_buffer.push(current_state);
        // 1b. Read Real Joint States and push to Atomic Buffers
        std::array<double, 7> q_l, dq_l, q_r, dq_r;
        hardware.readJoints(q_l, dq_l, q_r, dq_r); // Ensure your MuJoCoEmulator has this method!
        
        left_arm_state.write(q_l, dq_l);
        right_arm_state.write(q_r, dq_r);

        // 2. Read calculated torques from Control Threads
        std::array<double, 7> tau_left = left_arm_torques.read();
        std::array<double, 7> tau_right = right_arm_torques.read();

        // 3. Write torques to hardware (MuJoCo)
        // Note: You will need to update writeTorques to accept both arms soon!
        hardware.writeTorques(tau_left, tau_right); 

        // 4. Step Simulation & Publish Telemetry
        int ball_joint_id = mj_name2id(m, mjOBJ_JOINT, "ball_joint");
        if (ball_joint_id >= 0) {
            int qpos_adr = m->jnt_qposadr[ball_joint_id];
            int dof_adr = m->jnt_dofadr[ball_joint_id];
            
            double ball_x = d->qpos[qpos_adr];
            //double ball_y = d->qpos[qpos_adr + 1];
            double ball_z = d->qpos[qpos_adr + 2];
            double ball_vx = d->qvel[dof_adr];
            double ball_vy = d->qvel[dof_adr + 1];

            // If ball falls off table or slows to a stop, re-serve towards the left arm
            if (ball_z < 0.2 || (std::abs(ball_vx) < 0.05 && std::abs(ball_vy) < 0.05 && ball_x < 0.2)) {
                d->qpos[qpos_adr]     =  1.2;  // X position (Right side)
                d->qpos[qpos_adr + 1] =  0.0;  // Y position (Center)
                d->qpos[qpos_adr + 2] =  1.2;  // Z position (Height)
                
                d->qvel[dof_adr]     = -3.5;  // Velocity towards Left Arm A
                d->qvel[dof_adr + 1] =  0.2;  
                d->qvel[dof_adr + 2] =  1.0;
                
                std::cout << "[HAL] Ball automatically re-served across the table!" << std::endl;
            }
        }
        mj_step(m, d);
        std::copy(d->qpos, d->qpos + nq, qpos_buffer.begin());
        zmq_send(zmq_pub, qpos_buffer.data(), nq * sizeof(double), ZMQ_DONTWAIT);

        // Enforce 1kHz Timing
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
    // --- Configure Camera View for Ping Pong Table ---
    mjvCamera cam;
    mjv_defaultCamera(&cam);
    cam.azimuth = 90.0;      // Rotate to view from the side (horizontal)
    cam.elevation = -20.0;   // Look slightly down
    cam.distance = 4.5;      // Pull back to see both arms
    cam.lookat[0] = 0.0;     // Center of table X
    cam.lookat[1] = 0.0;     // Center of table Y
    cam.lookat[2] = 0.76;    // Table height Z

    void* context = zmq_ctx_new();
    void* publisher = zmq_socket(context, ZMQ_PUB);
    zmq_bind(publisher, "tcp://*:5556");
    void* cmd_subscriber = zmq_socket(context, ZMQ_PULL);
    zmq_bind(cmd_subscriber, "tcp://*:5557");

    // Spin up the Control Threads
    std::thread left_controller(arm_control_loop, "Arm A (Left)", std::ref(left_arm_torques));
    std::thread right_controller(arm_control_loop, "Arm B (Right)", std::ref(right_arm_torques));

    // Spin up the Physics / HAL Thread
    std::thread physics_thread(physics_hal_loop, publisher, cmd_subscriber);

    std::cout << "[System] Press Enter to safely shutdown." << std::endl;
    std::cin.get();

    // Clean teardown
    simulation_running.store(false, std::memory_order_release);
    
    left_controller.join();
    right_controller.join();
    physics_thread.join();
    
    zmq_close(publisher);
    zmq_ctx_term(context);
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}