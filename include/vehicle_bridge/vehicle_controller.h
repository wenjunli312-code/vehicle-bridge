#pragma once

#include "vehicle_state.h"
#include "can_message.h"

namespace vehicle_bridge {

/// Callback type invoked when the controller emits a CAN message.
using CanSendCallback = void(*)(const CanMessage&, void* user_data);

/// Sends throttle, brake, and steering commands over a CAN bus abstraction.
class VehicleController {
public:
    VehicleController() = default;

    /// Register a callback that will be called each time a CAN frame is sent.
    void set_send_callback(CanSendCallback cb, void* user_data = nullptr);

    /// Send a throttle command.
    /// @param value  Normalised [0.0, 1.0] — 0 = no throttle, 1 = full throttle.
    void send_throttle(double value);

    /// Send a brake command.
    /// @param value  Normalised [0.0, 1.0] — 0 = no braking, 1 = full brake.
    void send_brake(double value);

    /// Send a steering command.
    /// @param angle_rad  Desired front wheel angle in radians.
    ///                   Positive = left turn, negative = right turn.
    void send_steering(double angle_rad);

    /// Convenience: send throttle, brake, and steering in one call.
    void send_command(double throttle, double brake, double steering_rad);

    /// Returns the last known vehicle state (updated externally via update_state).
    const VehicleState& state() const { return state_; }

    /// Update the internal vehicle state (e.g. from a state-estimation module).
    void update_state(const VehicleState& s) { state_ = s; }

private:
    VehicleState    state_{};
    CanSendCallback send_cb_{nullptr};
    void*           send_cb_user_data_{nullptr};

    void emit(const CanMessage& msg);
};

}  // namespace vehicle_bridge
