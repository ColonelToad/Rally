#include "rally/ipc/zmq_context.hpp"
#include "rally/messages/sensor_packet.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

using namespace rally::ipc;

const std::string SENSOR_PUB_URL = "ipc:///tmp/rally/sensor_pub.sock";
const double LOOP_RATE_HZ = 500.0;
const double CYCLE_TIME_MS = 1000.0 / LOOP_RATE_HZ;

int main() {
    std::cout << "[MuJoCo Bridge] Starting up..." << std::endl;
    ZmqContext ctx;
    
    // Bind the publisher
    ZmqSocket sensor_pub(ctx, SocketType::PUB);
    sensor_pub.bind(SENSOR_PUB_URL);

    // Pre-allocate the struct on the stack (Principle 3: No heap in hot path)
    SensorPacket packet{};
    packet.sequence_number = 0;

    std::cout << "[MuJoCo Bridge] Alive. Publishing 1Hz sine wave at 500Hz...\n";
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(now - start_time).count();

        // Populate header
        packet.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();
        packet.sequence_number++;

        // Inject 1Hz sine wave into joint 0
        packet.joint_positions[0] = std::sin(2.0 * M_PI * 1.0 * elapsed_sec);
        packet.joint_velocities[0] = 2.0 * M_PI * std::cos(2.0 * M_PI * 1.0 * elapsed_sec);

        // Zero-allocation send
        sensor_pub.send(packet);

        // Naive sleep to approximate 500Hz (will be replaced by Week 4 Task Executor)
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(CYCLE_TIME_MS)));
    }

    return 0;
}