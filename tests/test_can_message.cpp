#include "vehicle_bridge/can_message.h"
#include <cassert>
#include <cstring>

static void test_parse_valid() {
    // id=0x00000042 (LE), dlc=3, data=[0xAA, 0xBB, 0xCC]
    uint8_t buf[] = {0x42, 0x00, 0x00, 0x00, 0x03, 0xAA, 0xBB, 0xCC};
    vehicle_bridge::CanMessage msg;
    assert(vehicle_bridge::CanMessage::parse(buf, sizeof(buf), msg));
    assert(msg.id == 0x42);
    assert(msg.dlc == 3);
    assert(msg.data[0] == 0xAA);
    assert(msg.data[1] == 0xBB);
    assert(msg.data[2] == 0xCC);
}

static void test_parse_too_short() {
    uint8_t buf[] = {0x01, 0x00, 0x00};
    vehicle_bridge::CanMessage msg;
    assert(!vehicle_bridge::CanMessage::parse(buf, sizeof(buf), msg));
}

static void test_parse_invalid_dlc() {
    uint8_t buf[] = {0x01, 0x00, 0x00, 0x00, 0x09};  // dlc=9 > 8
    vehicle_bridge::CanMessage msg;
    assert(!vehicle_bridge::CanMessage::parse(buf, sizeof(buf), msg));
}

static void test_roundtrip() {
    uint8_t src[] = {0x23, 0x01, 0x00, 0x00, 0x02, 0xDE, 0xAD};
    vehicle_bridge::CanMessage msg;
    assert(vehicle_bridge::CanMessage::parse(src, sizeof(src), msg));

    uint8_t dst[16]{};
    const std::size_t written = msg.serialize(dst, sizeof(dst));
    assert(written == 7);
    assert(std::memcmp(src, dst, written) == 0);
}

static void test_to_string_not_empty() {
    vehicle_bridge::CanMessage msg;
    msg.id  = 0x123;
    msg.dlc = 2;
    msg.data[0] = 0xAB;
    msg.data[1] = 0xCD;
    assert(!msg.to_string().empty());
}

int main() {
    test_parse_valid();
    test_parse_too_short();
    test_parse_invalid_dlc();
    test_roundtrip();
    test_to_string_not_empty();
    return 0;
}
