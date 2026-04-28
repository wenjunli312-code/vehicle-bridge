#include "vehicle_bridge/can_message.h"

#include <cstring>
#include <sstream>
#include <iomanip>

namespace vehicle_bridge {

bool CanMessage::parse(const uint8_t* buf, std::size_t len, CanMessage& out) {
    // Minimum frame: 4 bytes (id) + 1 byte (dlc)
    if (!buf || len < 5) {
        return false;
    }

    uint32_t id = 0;
    std::memcpy(&id, buf, 4);  // little-endian
    const uint8_t dlc = buf[4];

    if (dlc > CAN_MAX_DATA_LEN) {
        return false;
    }
    if (len < static_cast<std::size_t>(5 + dlc)) {
        return false;
    }

    out.id  = id;
    out.dlc = dlc;
    out.data.fill(0);
    std::memcpy(out.data.data(), buf + 5, dlc);
    return true;
}

std::size_t CanMessage::serialize(uint8_t* buf, std::size_t buf_len) const {
    if (!buf || dlc > CAN_MAX_DATA_LEN) {
        return 0;
    }
    const std::size_t needed = 5 + dlc;
    if (buf_len < needed) {
        return 0;
    }

    std::memcpy(buf, &id, 4);
    buf[4] = dlc;
    std::memcpy(buf + 5, data.data(), dlc);
    return needed;
}

std::string CanMessage::to_string() const {
    std::ostringstream oss;
    oss << "ID=0x" << std::hex << std::uppercase << std::setw(3)
        << std::setfill('0') << id
        << " DLC=" << std::dec << static_cast<int>(dlc) << " [";
    for (uint8_t i = 0; i < dlc; ++i) {
        if (i > 0) oss << ' ';
        oss << std::hex << std::uppercase << std::setw(2)
            << std::setfill('0') << static_cast<int>(data[i]);
    }
    oss << ']';
    return oss.str();
}

}  // namespace vehicle_bridge
