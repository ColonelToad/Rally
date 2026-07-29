#ifndef HAL_ACTUATOR_HPP
#define HAL_ACTUATOR_HPP

#include <array>

namespace hal {

class Actuator {
public:
    virtual ~Actuator() = default;

    /**
     * @brief Writes torque commands to hardware motors.
     */
    virtual void writeTorques(const std::array<double, 7>& torques) = 0;
};

} // namespace hal

#endif // HAL_ACTUATOR_HPP