#include "vehicle_bridge/vehicle_controller.h"
#include <cassert>
#include <vector>

struct CapturedMsg {
    vehicle_bridge::CanMessage msg;
};

static std::vector<CapturedMsg> g_captured;

static void capture_cb(const vehicle_bridge::CanMessage& msg, void* /*user_data*/) {
    g_captured.push_back({msg});
}

static void test_send_throttle_emits_one_message() {
    g_captured.clear();
    vehicle_bridge::VehicleController ctrl;
    ctrl.set_send_callback(capture_cb);
    ctrl.send_throttle(0.5);
    assert(g_captured.size() == 1);
    assert(g_captured[0].msg.id == 0x100);
    assert(g_captured[0].msg.dlc == 2);
}

static void test_send_command_emits_three_messages() {
    g_captured.clear();
    vehicle_bridge::VehicleController ctrl;
    ctrl.set_send_callback(capture_cb);
    ctrl.send_command(0.3, 0.0, 0.1);
    assert(g_captured.size() == 3);
}

static void test_no_callback_does_not_crash() {
    vehicle_bridge::VehicleController ctrl;
    ctrl.send_throttle(1.0);  // no callback set — should not crash
}

static void test_update_state() {
    vehicle_bridge::VehicleController ctrl;
    vehicle_bridge::VehicleState s;
    s.velocity = 20.0;
    s.heading  = 1.57;
    ctrl.update_state(s);
    assert(ctrl.state().velocity == 20.0);
    assert(ctrl.state().heading  == 1.57);
}

int main() {
    test_send_throttle_emits_one_message();
    test_send_command_emits_three_messages();
    test_no_callback_does_not_crash();
    test_update_state();
    return 0;
}
