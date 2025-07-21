/*

Copyright (c) 2017, Project OSRM contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ENGINE_API_MATCH_PARAMETERS_HPP
#define ENGINE_API_MATCH_PARAMETERS_HPP

#include "engine/api/route_parameters.hpp"
#include "engine/yaw_rate.hpp"
#include "engine/steering_angle.hpp"

#include <vector>
#include <optional>

namespace osrm::engine::api
{

/**
 * Parameters specific to the Match service.
 *
 * Holds member attributes:
 *  - timestamps: timestamp(s) for the corresponding input coordinate(s)
 *  - yaw_rate: yaw rate measurements for vehicle dynamics constraints during map matching
 *  - steering_angle: steering angle measurements for vehicle dynamics constraints during map matching
 *  - gaps: how to handle gaps in the GPS trace
 *  - tidy: whether to return a simplified geometry
 *
 * The yaw_rate and steering_angle are used to improve map matching accuracy by:
 * - Filtering candidate road segments based on vehicle kinematic feasibility
 * - Rejecting GPS points that would require impossible vehicle maneuvers
 * - Improving temporal consistency of the matched path
 *
 * \see OSRM, Coordinate, Hint, Bearing, YawRate, SteeringAngle, RouteParameters, 
 *      TableParameters, NearestParameters, TripParameters, and TileParameters
 */
struct MatchParameters : public RouteParameters
{
    enum class GapsType
    {
        Split,  // Split the trace at gaps
        Ignore  // Ignore gaps and continue matching
    };

    MatchParameters()
        : RouteParameters(false,
                          false,
                          false,
                          RouteParameters::GeometriesType::Polyline,
                          RouteParameters::OverviewType::Simplified,
                          {}),
          gaps(GapsType::Split), tidy(false)
    {
    }

    template <typename... Args>
    MatchParameters(const std::vector<unsigned> &timestamps_,
                    GapsType gaps_,
                    bool tidy_,
                    Args &&...args_)
        : MatchParameters(timestamps_, gaps_, tidy_, {}, {}, {}, std::forward<Args>(args_)...)
    {
    }

    template <typename... Args>
    MatchParameters(const std::vector<unsigned> &timestamps_,
                    GapsType gaps_,
                    bool tidy_,
                    const std::vector<std::optional<YawRate>> &yaw_rates_,
                    const std::vector<std::optional<SteeringAngle>> &steering_angles_,
                    Args &&...args_)
        : MatchParameters(timestamps_, gaps_, tidy_, yaw_rates_, steering_angles_, {}, std::forward<Args>(args_)...)
    {
    }

    template <typename... Args>
    MatchParameters(std::vector<unsigned> timestamps_,
                    GapsType gaps_,
                    bool tidy_,
                    std::vector<std::optional<YawRate>> yaw_rates_,
                    std::vector<std::optional<SteeringAngle>> steering_angles_,
                    const std::vector<std::size_t> &waypoints_,
                    Args &&...args_)
        : RouteParameters{std::forward<Args>(args_)..., waypoints_}, 
          timestamps{std::move(timestamps_)},
          yaw_rate{std::move(yaw_rates_)},
          steering_angle{std::move(steering_angles_)},
          gaps(gaps_), tidy(tidy_)
    {
    }

    // Core match parameters
    std::vector<unsigned> timestamps;
    std::vector<std::optional<YawRate>> yaw_rate;
    std::vector<std::optional<SteeringAngle>> steering_angle;
    GapsType gaps;
    bool tidy;

    // Optional: velocity (m/s) for each trace point, same length as trace_coordinates if provided
    std::vector<std::optional<double>> trace_velocities;

    bool IsValid() const
    {
        // Validate base route parameters first
        if (!RouteParameters::IsValid()) {
            return false;
        }

        // Validate size consistency
        bool size_valid = (timestamps.empty() || timestamps.size() == coordinates.size()) &&
                         (yaw_rate.empty() || yaw_rate.size() == coordinates.size()) &&
                         (steering_angle.empty() || steering_angle.size() == coordinates.size());

        if (!size_valid) {
            return false;
        }

        // Validate yaw rates using the structured type
        bool yaw_rates_valid = std::all_of(yaw_rate.begin(),
                                          yaw_rate.end(),
                                          [](const std::optional<YawRate> &yaw_rate)
                                          {
                                              return !yaw_rate || yaw_rate->IsValid();
                                          });

        // Validate steering angles using the structured type
        bool steering_angles_valid = std::all_of(steering_angle.begin(),
                                                steering_angle.end(),
                                                [](const std::optional<SteeringAngle> &steering_angle)
                                                {
                                                    return !steering_angle || steering_angle->IsValid();
                                                });

        // Validate temporal consistency (timestamps should be monotonic if provided)
        bool temporal_valid = true;
        if (!timestamps.empty()) {
            for (size_t i = 1; i < timestamps.size(); ++i) {
                if (timestamps[i] < timestamps[i-1]) {
                    temporal_valid = false;
                    break;
                }
            }
        }

        return yaw_rates_valid && steering_angles_valid && temporal_valid;
    }

    // Helper method to check if vehicle dynamics data is available
    bool HasVehicleDynamics() const
    {
        return !yaw_rate.empty() || !steering_angle.empty();
    }

    // Helper method to get the time interval between consecutive points
    std::vector<double> GetTimeIntervals() const
    {
        std::vector<double> intervals;
        if (timestamps.size() < 2) return intervals;
        
        intervals.reserve(timestamps.size() - 1);
        for (size_t i = 1; i < timestamps.size(); ++i) {
            intervals.push_back(static_cast<double>(timestamps[i] - timestamps[i-1]));
        }
        return intervals;
    }
};

} // namespace osrm::engine::api

#endif // ENGINE_API_MATCH_PARAMETERS_HPP