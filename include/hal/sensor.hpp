#ifndef HAL_SENSOR_HPP
#define HAL_SENSOR_HPP

#include <array>

namespace hal {

class Sensor {
public:
    virtual ~Sensor() = default;

    /**
     * @brief Reads joint positions and velocities from hardware.
     */
    virtual void readJoints(std::array<double, 7>& q_left, std::array<double, 7>& dq_left,
                        std::array<double, 7>& q_right, std::array<double, 7>& dq_right) = 0;

    /**
     * @brief Reads external environmental data (e.g., ball tracking).
     */
    virtual void readBallState(std::array<double, 3>& pos, std::array<double, 3>& vel) = 0;
    
    /**
     * @brief Returns current hardware timestamp in seconds.
     */
    virtual double getTime() = 0;
};

} // namespace hal

#endif // HAL_SENSOR_HPP