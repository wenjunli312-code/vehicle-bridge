#include "vehicle_bridge/vehicle_controller.h"

#include <algorithm>
#include <cstring>

namespace vehicle_bridge {

// CAN arbitration IDs used by this controller (project-specific, demo values)
static constexpr uint32_t CAN_ID_THROTTLE  = 0x100;
static constexpr uint32_t CAN_ID_BRAKE     = 0x101;
static constexpr uint32_t CAN_ID_STEERING  = 0x102;

// Encode a normalised double [0,1] into a uint16_t [0, 65535]
static uint16_t encode_normalised(double value) {
    const double clamped = std::max(0.0, std::min(1.0, value));
    return static_cast<uint16_t>(clamped * 65535.0);
}

// Encode a steering angle (radians) into an int16_t
// Maps [-max_angle, +max_angle] → [-32768, 32767]
static int16_t encode_steering(double angle_rad) {
    static constexpr double MAX_ANGLE_RAD = 0.6109;  // ~35 degrees
    const double clamped = std::max(-MAX_ANGLE_RAD, std::min(MAX_ANGLE_RAD, angle_rad));
    return static_cast<int16_t>((clamped / MAX_ANGLE_RAD) * 32767.0);
}

void VehicleController::set_send_callback(CanSendCallback cb, void* user_data) {
    send_cb_           = cb;
    send_cb_user_data_ = user_data;
}

void VehicleController::send_throttle(double value) {
    const uint16_t encoded = encode_normalised(value);
    CanMessage msg;
    msg.id  = CAN_ID_THROTTLE;
    msg.dlc = 2;
    msg.data[0] = static_cast<uint8_t>(encoded & 0xFF);
    msg.data[1] = static_cast<uint8_t>((encoded >> 8) & 0xFF);
    emit(msg);
}

void VehicleController::send_brake(double value) {
    const uint16_t encoded = encode_normalised(value);
    CanMessage msg;
    msg.id  = CAN_ID_BRAKE;
    msg.dlc = 2;
    msg.data[0] = static_cast<uint8_t>(encoded & 0xFF);
    msg.data[1] = static_cast<uint8_t>((encoded >> 8) & 0xFF);
    emit(msg);
}

void VehicleController::send_steering(double angle_rad) {
    const int16_t encoded = encode_steering(angle_rad);
    CanMessage msg;
    msg.id  = CAN_ID_STEERING;
    msg.dlc = 2;
    msg.data[0] = static_cast<uint8_t>(static_cast<uint16_t>(encoded) & 0xFF);
    msg.data[1] = static_cast<uint8_t>((static_cast<uint16_t>(encoded) >> 8) & 0xFF);
    emit(msg);
}

void VehicleController::send_command(double throttle, double brake, double steering_rad) {
    send_throttle(throttle);
    send_brake(brake);
    send_steering(steering_rad);
}

void VehicleController::emit(const CanMessage& msg) {
    if (send_cb_) {
        send_cb_(msg, send_cb_user_data_);
    }
}

}  // namespace vehicle_bridge
