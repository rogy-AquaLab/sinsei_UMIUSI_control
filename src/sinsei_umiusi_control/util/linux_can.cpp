#include "sinsei_umiusi_control/util/linux_can.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <rcpputils/tl_expected/expected.hpp>

namespace suc_util = sinsei_umiusi_control::util;

suc_util::LinuxCan::LinuxCan() : sock(-1) {}

auto suc_util::LinuxCan::close() -> tl::expected<void, std::string> {
    if (this->sock < 0) {
        return tl::make_unexpected("CAN socket is not initialized");
    }
    auto res = ::close(this->sock);
    if (res < 0) {
        return tl::make_unexpected("Failed to close CAN socket: " + std::string(strerror(errno)));
    }
    this->sock = -1;
    return {};
}

auto suc_util::LinuxCan::init(const std::string ifname) -> tl::expected<void, std::string> {
    // Create a socket
    this->sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->sock < 0) {
        return tl::make_unexpected("Failed to create CAN socket: " + std::string(strerror(errno)));
    }

    // Interface request (name -> if_index mapping)
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    auto res = ::ioctl(this->sock, SIOCGIFINDEX, &ifr);
    if (res < 0) {
        auto res = this->close();
        if (!res) {
            return tl::make_unexpected(
                "Failed to close CAN socket after ioctl failure: " + res.error());
        }
        return tl::make_unexpected(
            "Failed to get interface index: " + std::string(strerror(errno)));
    }

    // Set up the socket address structure
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind the socket to the CAN interface
    res = ::bind(this->sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    if (res < 0) {
        auto res = this->close();
        if (!res) {
            return tl::make_unexpected(
                "Failed to close CAN socket after bind failure: " + res.error());
        }
        return tl::make_unexpected("Failed to bind CAN socket: " + std::string(strerror(errno)));
    }

    return {};
}

auto suc_util::LinuxCan::send_frame(
    uint32_t id, const uint8_t * data, size_t length,
    bool is_extended) -> tl::expected<void, std::string> {
    if (this->sock < 0) {
        return tl::make_unexpected("CAN socket is not initialized");
    }

    if (length > CAN_MAX_DLEN) {
        return tl::make_unexpected("DLC exceeds maximum allowed CAN data length");
    }

    // Prepare a CAN frame to send
    can_frame frame{};
    if (is_extended) {
        frame.can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;  // Extended frame ID
    } else {
        frame.can_id = id & CAN_SFF_MASK;  // Standard frame ID
    }
    frame.can_dlc = static_cast<uint8_t>(length);  // always safe (bc. length <= CAN_MAX_DLEN = 8)
    const auto data_begin = data;
    const auto data_end = data + length;
    std::copy(data_begin, data_end, frame.data);

    // Send the CAN frame
    const auto bytes_to_write = sizeof(frame);
    const auto bytes_written = ::write(this->sock, &frame, bytes_to_write);
    if (bytes_written < 0) {
        return tl::make_unexpected("Failed to write CAN frame: " + std::string(strerror(errno)));
    }

    if (static_cast<size_t>(bytes_written) != bytes_to_write) {
        return tl::make_unexpected(
            "Incomplete CAN frame written: expected " + std::to_string(bytes_to_write) +
            " bytes, got " + std::to_string(bytes_written) + " bytes");
    }

    return {};
}

auto suc_util::LinuxCan::send_frame_std(uint32_t id, const uint8_t * data, size_t length)
    -> tl::expected<void, std::string> {
    return this->send_frame(id, data, length, false);
}

auto suc_util::LinuxCan::send_frame_ext(uint32_t id, const uint8_t * data, size_t length)
    -> tl::expected<void, std::string> {
    return this->send_frame(id, data, length, true);
}

auto suc_util::LinuxCan::recv_frame() -> tl::expected<CanFrame, std::string> {
    if (this->sock < 0) {
        return tl::make_unexpected("CAN socket is not initialized");
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);             // Clear the set
    FD_SET(this->sock, &read_fds);  // Add the socket to the set

    // Set a timeout for select (1 sec)
    constexpr int TIMEOUT_MS = 1000;
    struct timeval timeout = {
        TIMEOUT_MS / 1000,                                 // seconds
        (static_cast<int64_t>(TIMEOUT_MS % 1000)) * 1000,  // microseconds
    };

    // Wait for data to be available on the socket
    const auto nfds = this->sock + 1;  // highest file descriptor + 1
    auto res = ::select(nfds, &read_fds, nullptr, nullptr, &timeout);
    if (res < 0) {
        return tl::make_unexpected("select() failed: " + std::string(strerror(errno)));
    } else if (res == 0) {
        return tl::make_unexpected("No data available on CAN socket within timeout period");
    }

    // Read the CAN frame from the socket
    struct can_frame frame {};
    const auto bytes_to_read = sizeof(frame);
    const auto bytes_read = ::read(this->sock, &frame, bytes_to_read);
    if (bytes_read < 0) {
        return tl::make_unexpected("read() failed: " + std::string(strerror(errno)));
    }
    if (static_cast<size_t>(bytes_read) != bytes_to_read) {
        return tl::make_unexpected(
            "Incomplete CAN frame read: expected " + std::to_string(bytes_to_read) +
            " bytes, got " + std::to_string(bytes_read) + " bytes");
    }

    // Prepare the CanFrame result
    CanFrame result{};
    result.id = frame.can_id;
    result.dlc = frame.can_dlc;
    const auto data_length = std::min<uint8_t>(frame.can_dlc, CAN_MAX_DLEN);
    const auto data_begin = frame.data;
    const auto data_end = frame.data + data_length;
    std::copy(data_begin, data_end, result.data.begin());

    return result;
}