#pragma once

namespace vehicle_bridge {

/// Represents the current kinematic state of a vehicle.
struct VehicleState {
    double x{0.0};               ///< X position in meters (world frame)
    double y{0.0};               ///< Y position in meters (world frame)
    double velocity{0.0};        ///< Longitudinal velocity in m/s
    double heading{0.0};         ///< Heading angle in radians (0 = east, CCW positive)
    double steering_angle{0.0};  ///< Front wheel steering angle in radians

    /// Returns true if all fields are at their default (zero) values.
    bool is_zero() const {
        return x == 0.0 && y == 0.0 && velocity == 0.0 &&
               heading == 0.0 && steering_angle == 0.0;
    }
};

}  // namespace vehicle_bridge
