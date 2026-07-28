#include "rally/ipc/zmq_context.hpp"
#include "rally/messages/sensor_packet.hpp"
#include "rally/messages/arm_status.hpp"
#include <iostream>
#include <iomanip>

using namespace rally::ipc;

const std::string SENSOR_PUB_URL = "ipc:///tmp/rally/sensor_pub.sock";
const std::string ARM_STATUS_PUSH_URL = "ipc:///tmp/rally/arm_a_status.sock";

int main() {
    std::cout << "[Arm A Controller] Starting up..." << std::endl;
    ZmqContext ctx;

    // Connect to Bridge (Sensors)
    ZmqSocket sensor_sub(ctx, SocketType::SUB);
    sensor_sub.connect(SENSOR_PUB_URL);
    sensor_sub.subscribe(); // Subscribe to all topics

    // Connect to Coordination Bus (Status)
    ZmqSocket status_push(ctx, SocketType::PUSH);
    status_push.connect(ARM_STATUS_PUSH_URL);

    // Stack-allocated buffers
    SensorPacket incoming_sensor{};
    ArmStatus outgoing_status{};
    outgoing_status.arm_id = 0; // Arm A

    std::cout << "[Arm A Controller] Listening for sine wave...\n";

    while (true) {
        // Zero-allocation receive. Blocks until data arrives.
        if (sensor_sub.receive(incoming_sensor)) {
            
            // Print every 100th packet (every ~200ms at 500Hz) to keep stdout clean
            if (incoming_sensor.sequence_number % 100 == 0) {
                std::cout << "[Arm A] Seq: " << incoming_sensor.sequence_number 
                          << " | Joint 0 Pos: " << std::fixed << std::setprecision(4) 
                          << incoming_sensor.joint_positions[0] << std::endl;
            }

            // Pretend we computed a control step, update and send status to the Bus
            outgoing_status.timestamp_us = incoming_sensor.timestamp_us;
            outgoing_status.current_state = 1; // 1 = Moving
            status_push.send(outgoing_status, ZMQ_DONTWAIT);
        }
    }

    return 0;
}