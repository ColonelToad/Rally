#include "rally/ipc/zmq_context.hpp"
#include "rally/messages/arm_status.hpp"
#include <iostream>
#include <chrono>
#include <thread>

using namespace rally::ipc;

const std::string ARM_A_STATUS_PULL_URL = "ipc:///tmp/rally/arm_a_status.sock";
const double LOOP_RATE_HZ = 100.0;
const double CYCLE_TIME_MS = 1000.0 / LOOP_RATE_HZ;

int main() {
    std::cout << "[Coordination Bus] Starting up..." << std::endl;
    ZmqContext ctx;

    // Bind PULL socket to receive statuses from Arm A
    ZmqSocket status_pull(ctx, SocketType::PULL);
    status_pull.bind(ARM_A_STATUS_PULL_URL);

    ArmStatus incoming_status{};

    std::cout << "[Coordination Bus] Alive. Polling for arm status at 100Hz...\n";

    while (true) {
        // Non-blocking read (ZMQ_DONTWAIT). The HLC must not get stuck waiting on an LLC.
        while (status_pull.receive(incoming_status, ZMQ_DONTWAIT)) {
            // Drain the queue and process the latest status
        }

        if (incoming_status.timestamp_us > 0) {
            // Just prove we got it
            // In reality, this will update the ownership arbiter state
        }

        // Sleep to enforce 100Hz HLC rate
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(CYCLE_TIME_MS)));
    }

    return 0;
}