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
#include <algorithm>

// MuJoCo data structures
mjModel* m = nullptr;
mjData* d = nullptr;

std::atomic<bool> simulation_running(true);

/**
 * @brief Hard real-time control loop running at 500Hz, publishing qpos via ZMQ.
 */
void physics_control_loop(void* zmq_pub) {
    const double dt = 0.002; // 500 Hz
    const int nq = m->nq;
    std::vector<double> qpos_buffer(nq);

    while (simulation_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        // 1. TODO: Run EKF Prediction (writing into d as needed)
        // 2. TODO: Run Quintic Spline Path Planner
        // 3. TODO: Run Impedance Controller (calculate torques into d->ctrl)

        // 4. Step MuJoCo physics: updates d->qpos, d->qvel, etc.
        mj_step(m, d);

        // 5. Copy qpos into a contiguous buffer and publish
        std::copy(d->qpos, d->qpos + nq, qpos_buffer.begin());

        int rc = zmq_send(zmq_pub,
                          qpos_buffer.data(),
                          nq * sizeof(double),
                          ZMQ_DONTWAIT);
        if (rc == -1) {
            // Optional: log errno or ignore; EAGAIN is common if no subscriber yet
            // std::cerr << "ZMQ send failed: " << zmq_strerror(zmq_errno()) << std::endl;
        }

        // Enforce 500Hz strict timing
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        if (elapsed.count() < dt) {
            std::this_thread::sleep_for(std::chrono::duration<double>(dt - elapsed.count()));
        }
    }
}

int main(int argc, char** argv) {
    // 1. Load MuJoCo Model
    char error[1000] = "Could not load XML model";
    m = mj_loadXML("panda_hit_scene.xml", nullptr, error, 1000);
    if (!m) {
        std::cerr << "MuJoCo Load Error: " << error << std::endl;
        return 1;
    }
    d = mj_makeData(m);

    // 2. Initialize ZeroMQ
    void* context = zmq_ctx_new();
    void* publisher = zmq_socket(context, ZMQ_PUB);

    // Bind to localhost:5556 (Python will connect)
    int rc = zmq_bind(publisher, "tcp://*:5556");
    if (rc != 0) {
        std::cerr << "Failed to bind ZMQ PUB socket: "
                  << zmq_strerror(zmq_errno()) << std::endl;
        mj_deleteData(d);
        mj_deleteModel(m);
        zmq_close(publisher);
        zmq_ctx_term(context);
        return 1;
    }

    std::cout << "[Bridge] Starting 500Hz Physics Publisher..." << std::endl;
    std::thread physics_thread(physics_control_loop, publisher);

    // 3. Simple blocking wait so the program doesn't exit
    std::cout << "[Bridge] Press Enter to stop." << std::endl;
    std::cin.get();

    // 4. Cleanup
    simulation_running.store(false);
    physics_thread.join();

    zmq_close(publisher);
    zmq_ctx_term(context);

    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}