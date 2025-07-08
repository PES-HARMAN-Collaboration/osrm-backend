#ifndef ENGINE_MAP_MATCHING_VEHICLE_DYNAMICS_TRANSITION_HPP
#define ENGINE_MAP_MATCHING_VEHICLE_DYNAMICS_TRANSITION_HPP

#include "engine/yaw_rate.hpp"
#include "engine/steering_angle.hpp"
#include "util/coordinate_calculation.hpp"

#include <cmath>
#include <optional>
#include <vector>
#include <type_traits>

namespace osrm::engine::map_matching
{

/**
 * Enhanced transition probability calculator that incorporates vehicle dynamics
 * to improve map matching accuracy by considering yaw rate and steering angle constraints.
 */
struct VehicleDynamicsTransitionProbability
{
    static constexpr double DEFAULT_BETA = 10.0;
    static constexpr double VEHICLE_DYNAMICS_WEIGHT = 0.3; // Weight for vehicle dynamics penalty
    static constexpr double MIN_VEHICLE_SPEED = 4.0; // m/s, minimum vehicle speed for dynamics
    static constexpr double MAX_VEHICLE_SPEED = 50.0; // m/s, maximum vehicle speed for dynamics
    static constexpr double DEFAULT_WHEELBASE = 2.7; // meters, typical passenger car wheelbase

    double beta;
    double log_beta;
    double vehicle_dynamics_weight;
    double wheelbase;

    VehicleDynamicsTransitionProbability(const double beta_ = DEFAULT_BETA,
                                         const double vehicle_dynamics_weight_ = VEHICLE_DYNAMICS_WEIGHT,
                                         const double wheelbase_ = DEFAULT_WHEELBASE)
        : beta(beta_), 
          log_beta(std::log(beta_)),
          vehicle_dynamics_weight(vehicle_dynamics_weight_),
          wheelbase(wheelbase_)
    {
    }

    /**
     * Calculate transition probability considering vehicle dynamics
     * @param d_t Distance difference between network and GPS trace
     * @param prev_coord Previous GPS coordinate
     * @param curr_coord Current GPS coordinate
     * @param prev_yaw_rate Optional yaw rate at previous point
     * @param curr_yaw_rate Optional yaw rate at current point
     * @param prev_steering_angle Optional steering angle at previous point
     * @param curr_steering_angle Optional steering angle at current point
     * @param time_interval Time interval between points in seconds
     * @return Log probability of the transition
     */
    double operator()(const double d_t,
                      const util::Coordinate &prev_coord,
                      const util::Coordinate &curr_coord,
                      const std::optional<YawRate> &prev_yaw_rate = std::nullopt,
                      const std::optional<YawRate> &curr_yaw_rate = std::nullopt,
                      const std::optional<SteeringAngle> &prev_steering_angle = std::nullopt,
                      const std::optional<SteeringAngle> &curr_steering_angle = std::nullopt,
                      const double time_interval = 1.0) const
    {
        // Base transition probability (original HMM approach)
        double base_transition_pr = -log_beta - d_t / beta;

        // If no vehicle dynamics data available, return base probability
        if (!prev_yaw_rate && !curr_yaw_rate && !prev_steering_angle && !curr_steering_angle)
        {
            return base_transition_pr;
        }

        // Calculate vehicle dynamics penalty
        double dynamics_penalty = calculateVehicleDynamicsPenalty(
            prev_coord, curr_coord, prev_yaw_rate, curr_yaw_rate, 
            prev_steering_angle, curr_steering_angle, time_interval);

        // Combine base probability with vehicle dynamics penalty
        return base_transition_pr + vehicle_dynamics_weight * dynamics_penalty;
    }

    /**
     * Fallback operator for backward compatibility (when no vehicle dynamics data)
     */
    double operator()(const double d_t) const
    {
        return -log_beta - d_t / beta;
    }

private:
    /**
     * Calculate penalty based on vehicle dynamics constraints
     */
    double calculateVehicleDynamicsPenalty(const util::Coordinate &prev_coord,
                                          const util::Coordinate &curr_coord,
                                          const std::optional<YawRate> &prev_yaw_rate,
                                          const std::optional<YawRate> &curr_yaw_rate,
                                          const std::optional<SteeringAngle> &prev_steering_angle,
                                          const std::optional<SteeringAngle> &curr_steering_angle,
                                          const double time_interval) const
    {
        double penalty = 0.0;

        // Calculate GPS-based heading change
        const double gps_heading_change = calculateGPSHeadingChange(prev_coord, curr_coord);
        const double gps_distance = util::coordinate_calculation::greatCircleDistance(prev_coord, curr_coord);
        
        // Estimate vehicle speed from GPS (if time interval is available)
        double estimated_speed = MIN_VEHICLE_SPEED;
        if (time_interval > 0.0)
        {
            estimated_speed = std::max(MIN_VEHICLE_SPEED, 
                                     std::min(MAX_VEHICLE_SPEED, gps_distance / time_interval));
        }

        // Check yaw rate consistency
        if (prev_yaw_rate && prev_yaw_rate->IsValid())
        {
            penalty += checkYawRateConsistency(*prev_yaw_rate, gps_heading_change, time_interval);
        }

        if (curr_yaw_rate && curr_yaw_rate->IsValid())
        {
            penalty += checkYawRateConsistency(*curr_yaw_rate, gps_heading_change, time_interval);
        }

        // Check steering angle consistency
        if (prev_steering_angle && prev_steering_angle->IsValid())
        {
            penalty += checkSteeringAngleConsistency(*prev_steering_angle, gps_heading_change, 
                                                   estimated_speed, time_interval);
        }

        if (curr_steering_angle && curr_steering_angle->IsValid())
        {
            penalty += checkSteeringAngleConsistency(*curr_steering_angle, gps_heading_change, 
                                                   estimated_speed, time_interval);
        }

        // Check temporal consistency of vehicle dynamics
        if (prev_yaw_rate && curr_yaw_rate && 
            prev_yaw_rate->IsValid() && curr_yaw_rate->IsValid())
        {
            penalty += checkTemporalConsistency(*prev_yaw_rate, *curr_yaw_rate, time_interval);
        }

        if (prev_steering_angle && curr_steering_angle && 
            prev_steering_angle->IsValid() && curr_steering_angle->IsValid())
        {
            penalty += checkTemporalConsistency(*prev_steering_angle, *curr_steering_angle, time_interval);
        }

        return penalty;
    }

    /**
     * Calculate heading change from GPS coordinates
     */
    double calculateGPSHeadingChange(const util::Coordinate &prev_coord,
                                    const util::Coordinate &curr_coord) const
    {
        const double lat1 = static_cast<double>(toFloating(prev_coord.lat)) * M_PI / 180.0;
        const double lat2 = static_cast<double>(toFloating(curr_coord.lat)) * M_PI / 180.0;
        const double dlon = static_cast<double>(toFloating(curr_coord.lon) - toFloating(prev_coord.lon)) * M_PI / 180.0;

        const double y = std::sin(dlon) * std::cos(lat2);
        const double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
        
        double heading = std::atan2(y, x) * 180.0 / M_PI;
        if (heading < 0) heading += 360.0;
        
        return heading;
    }

    /**
     * Check if yaw rate is consistent with GPS heading change
     */
    double checkYawRateConsistency(const YawRate &yaw_rate,
                                  const double gps_heading_change,
                                  const double time_interval) const
    {
        if (time_interval <= 0.0) return 0.0;

        // Calculate expected heading change from yaw rate
        const double expected_heading_change = yaw_rate.rate * time_interval;
        
        // Calculate difference between expected and observed heading change
        const double heading_diff = std::abs(expected_heading_change - gps_heading_change);
        
        // Normalize to 0-1 range and convert to penalty (higher difference = higher penalty)
        const double normalized_diff = std::min(1.0, heading_diff / 180.0);
        return -std::log(1.0 - normalized_diff + 1e-6);
    }

    /**
     * Check if steering angle is consistent with GPS heading change
     */
    double checkSteeringAngleConsistency(const SteeringAngle &steering_angle,
                                        const double gps_heading_change,
                                        const double speed,
                                        const double time_interval) const
    {
        if (time_interval <= 0.0 || speed <= 0.0) return 0.0;

        // Calculate expected heading change from steering angle and speed
        const double curvature = steering_angle.ToCurvature(wheelbase);
        const double expected_heading_change = curvature * speed * time_interval * 180.0 / M_PI;
        
        // Calculate difference between expected and observed heading change
        const double heading_diff = std::abs(expected_heading_change - gps_heading_change);
        
        // Normalize to 0-1 range and convert to penalty
        const double normalized_diff = std::min(1.0, heading_diff / 180.0);
        return -std::log(1.0 - normalized_diff + 1e-6);
    }

    /**
     * Check temporal consistency of vehicle dynamics measurements
     */
    template<typename DynamicsType>
    double checkTemporalConsistency(const DynamicsType &prev_dynamics,
                                   const DynamicsType &curr_dynamics,
                                   const double time_interval) const
    {
        if (time_interval <= 0.0) return 0.0;

        // Calculate rate of change - handle both YawRate and SteeringAngle types
        double rate_of_change;
        if constexpr (std::is_same_v<DynamicsType, YawRate>)
        {
            rate_of_change = std::abs(curr_dynamics.rate - prev_dynamics.rate) / time_interval;
        }
        else if constexpr (std::is_same_v<DynamicsType, SteeringAngle>)
        {
            rate_of_change = std::abs(curr_dynamics.angle - prev_dynamics.angle) / time_interval;
        }
        else
        {
            return 0.0; // Unknown type
        }
        
        // Penalize unrealistic rate of change (e.g., sudden steering changes)
        const double max_reasonable_rate = 100.0; // deg/s^2 for yaw rate, deg/s for steering
        const double normalized_rate = std::min(1.0, rate_of_change / max_reasonable_rate);
        
        return -std::log(1.0 - normalized_rate + 1e-6);
    }
};

} // namespace osrm::engine::map_matching

#endif // ENGINE_MAP_MATCHING_VEHICLE_DYNAMICS_TRANSITION_HPP 