#ifndef ENGINE_STEERING_ANGLE_HPP
#define ENGINE_STEERING_ANGLE_HPP

#include <cmath>
#include <limits>

namespace osrm::engine
{

/**
 * Represents steering angle in degrees
 * Used for vehicle dynamics constraints in routing and map matching
 * Positive angles represent right turn, negative represent left turn
 */
struct SteeringAngle
{
    static constexpr double INVALID_STEERING_ANGLE = std::numeric_limits<double>::max();
    static constexpr double MIN_STEERING_ANGLE = -45.0; // degrees
    static constexpr double MAX_STEERING_ANGLE = 45.0;  // degrees

    double angle; // degrees

    SteeringAngle() : angle(INVALID_STEERING_ANGLE) {}
    explicit SteeringAngle(double angle_) : angle(angle_) {}

    bool IsValid() const
    {
        return angle != INVALID_STEERING_ANGLE && 
               angle >= MIN_STEERING_ANGLE && 
               angle <= MAX_STEERING_ANGLE;
    }

    bool operator==(const SteeringAngle& other) const
    {
        return std::abs(angle - other.angle) < 1e-6;
    }

    bool operator!=(const SteeringAngle& other) const
    {
        return !(*this == other);
    }

    // Convert to curvature (1/radius) for path planning
    double ToCurvature(double wheelbase) const
    {
        if (!IsValid() || wheelbase <= 0) return 0.0;
        return std::tan(angle * M_PI / 180.0) / wheelbase;
    }
};

} // namespace osrm::engine

#endif // ENGINE_STEERING_ANGLE_HPP