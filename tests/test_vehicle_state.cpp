#include "vehicle_bridge/vehicle_state.h"
#include <cassert>

static void test_default_is_zero() {
    vehicle_bridge::VehicleState s;
    assert(s.is_zero());
}

static void test_not_zero_after_assignment() {
    vehicle_bridge::VehicleState s;
    s.velocity = 10.0;
    assert(!s.is_zero());
}

int main() {
    test_default_is_zero();
    test_not_zero_after_assignment();
    return 0;
}
