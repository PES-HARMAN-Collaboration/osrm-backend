#ifndef ENGINE_YAW_RATE_HPP
#define ENGINE_YAW_RATE_HPP

#include <cmath>
#include <limits>

namespace osrm::engine
{

/**
 * Represents yaw rate (angular velocity around vertical axis) in degrees per second
 * Used for vehicle dynamics constraints in routing and map matching
 */
struct YawRate
{
    static constexpr double INVALID_YAW_RATE = std::numeric_limits<double>::max();
    static constexpr double MIN_YAW_RATE = -180.0; // deg/s
    static constexpr double MAX_YAW_RATE = 180.0;  // deg/s

    double rate; // degrees per second

    YawRate() : rate(INVALID_YAW_RATE) {}
    explicit YawRate(double rate_) : rate(rate_) {}

    bool IsValid() const
    {
        return rate != INVALID_YAW_RATE && 
               rate >= MIN_YAW_RATE && 
               rate <= MAX_YAW_RATE;
    }

    bool operator==(const YawRate& other) const
    {
        return std::abs(rate - other.rate) < 1e-6;
    }

    bool operator!=(const YawRate& other) const
    {
        return !(*this == other);
    }
};

} // namespace osrm::engine

#endif // ENGINE_YAW_RATE_HPP