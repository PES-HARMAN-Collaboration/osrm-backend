#ifndef OSRM_ENGINE_YAW_RATE_HPP
#define OSRM_ENGINE_YAW_RATE_HPP

#include <cmath>

namespace osrm::engine
{

struct YawRate
{
    double value; // Degrees per second (can be negative)

    // Typical sanity range: -90°/s to +90°/s for passenger vehicles
    bool IsValid() const { return value >= -90.0 && value <= 90.0; }

    // Optional: Classification helper
    bool IndicatesLaneChange() const { return std::abs(value) > 10.0; }  // adjustable threshold
};

inline std::vector<std::optional<YawRate>> parseListOfOptionalYawRates(const std::vector<double>& values)
{
    std::vector<std::optional<YawRate>> result;
    result.reserve(values.size());
    for (double v : values)
    {
        result.emplace_back(YawRate{v});
    }
    return result;
}

inline bool operator==(const YawRate lhs, const YawRate rhs)
{
    return lhs.value == rhs.value;
}

inline bool operator!=(const YawRate lhs, const YawRate rhs)
{
    return !(lhs == rhs);
}

} // namespace osrm::engine

#endif