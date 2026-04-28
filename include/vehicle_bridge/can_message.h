#pragma once

#include <cstdint>
#include <array>
#include <string>

namespace vehicle_bridge {

/// Maximum number of data bytes in a standard CAN frame.
static constexpr std::size_t CAN_MAX_DATA_LEN = 8;

/// Simplified CAN bus message representation.
struct CanMessage {
    uint32_t id{0};                                  ///< CAN arbitration ID
    uint8_t  dlc{0};                                 ///< Data length code (0-8)
    std::array<uint8_t, CAN_MAX_DATA_LEN> data{};   ///< Payload bytes

    /// Parse a raw byte buffer into a CanMessage.
    /// Layout: [id(4 bytes LE)] [dlc(1 byte)] [data(dlc bytes)]
    /// Returns false if the buffer is too short or dlc > CAN_MAX_DATA_LEN.
    static bool parse(const uint8_t* buf, std::size_t len, CanMessage& out);

    /// Serialize this message into a byte buffer (same layout as parse).
    /// Returns the number of bytes written, or 0 on error.
    std::size_t serialize(uint8_t* buf, std::size_t buf_len) const;

    /// Human-readable representation, e.g. "ID=0x123 DLC=3 [AA BB CC]"
    std::string to_string() const;
};

}  // namespace vehicle_bridge
