#ifndef ENGINE_MAP_MATCHING_VEHICLE_DYNAMICS_TRANSITION_HPP
#define ENGINE_MAP_MATCHING_VEHICLE_DYNAMICS_TRANSITION_HPP

#include <cmath>
#include <optional>

namespace osrm::engine::map_matching
{

struct VehicleDynamicsTransitionProbability
{
    static constexpr double DEFAULT_BETA = 10.0;
    static constexpr double SIGMA = 0.6;
    static constexpr double PI = 3.14159265358979323846;

    double beta;
    double log_beta;

    VehicleDynamicsTransitionProbability(const double beta_ = DEFAULT_BETA)
        : beta(beta_), log_beta(std::log(beta_)) {}

    // d_t: distance difference, prev_coord/curr_coord: GNSS points,
    // curr_yaw_rate: optional yaw rate at current point,
    // time_interval: seconds between points
    double operator()(const double d_t,
                      const util::Coordinate &prev_coord,
                      const util::Coordinate &curr_coord,
                      const std::optional<YawRate> &curr_yaw_rate = std::nullopt,
                      const double time_interval = 1.0) const
    {
        double base_transition_pr = -log_beta - d_t / beta;
        if (!curr_yaw_rate || !curr_yaw_rate->IsValid())
            return base_transition_pr;
        double penalty = calculateYawRatePenalty(prev_coord, curr_coord, *curr_yaw_rate, time_interval);
        return base_transition_pr - penalty;
    }

    double operator()(const double d_t) const
    {
        return -log_beta - d_t / beta;
    }

private:
    double calculateYawRatePenalty(const util::Coordinate &prev_coord,
                                   const util::Coordinate &curr_coord,
                                   const YawRate &yaw_rate,
                                   const double time_interval) const
    {
        if (time_interval <= 0.0) return 0.0;
        double gps_heading_change = calculateGPSHeadingChange(prev_coord, curr_coord);
        double expected_heading_change = yaw_rate.rate * time_interval;
        double heading_diff = std::abs(expected_heading_change - gps_heading_change);
        double radians = std::abs((heading_diff * PI) / 180.0);
        return (radians * radians) / (2 * (SIGMA * SIGMA));
    }

    double calculateGPSHeadingChange(const util::Coordinate &prev_coord,
                                     const util::Coordinate &curr_coord) const
    {
        double lat1 = static_cast<double>(toFloating(prev_coord.lat)) * PI / 180.0;
        double lat2 = static_cast<double>(toFloating(curr_coord.lat)) * PI / 180.0;
        double dlon = static_cast<double>(toFloating(curr_coord.lon) - toFloating(prev_coord.lon)) * PI / 180.0;
        double y = std::sin(dlon) * std::cos(lat2);
        double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
        double heading = std::atan2(y, x) * 180.0 / PI;
        if (heading < 0) heading += 360.0;
        return heading;
    }
};

} // namespace osrm::engine::map_matching

#endif // ENGINE_MAP_MATCHING_VEHICLE_DYNAMICS_TRANSITION_HPP