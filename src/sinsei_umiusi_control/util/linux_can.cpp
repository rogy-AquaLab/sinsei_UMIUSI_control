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
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>

namespace suc_util = sinsei_umiusi_control::util;

namespace {

// TODO: use optional<int> instead of int
auto _close(std::optional<suc_util::Socket> & sock) -> tl::expected<void, std::string> {
    if (!sock) {
        return tl::make_unexpected("Socket is not initialized");
    }
    auto res = ::close(sock.value());
    if (res < 0) {
        return tl::make_unexpected("Failed to close socket: " + std::string(strerror(errno)));
    }
    sock.reset();
    return {};
}

auto _init(std::optional<suc_util::Socket> & sock, const std::string & ifname)
    -> tl::expected<void, std::string> {
    // Create a socket
    sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        sock.reset();
        return tl::make_unexpected("Failed to create CAN socket: " + std::string(strerror(errno)));
    }

    // Interface request (name -> if_index mapping)
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    auto res = ::ioctl(sock.value(), SIOCGIFINDEX, &ifr);
    if (res < 0) {
        _close(sock);  // Reset the socket descriptor on error
        return tl::make_unexpected(
            "Failed to get interface index: " + std::string(strerror(errno)));
    }

    // Set up the socket address structure
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind the socket to the CAN interface
    res = ::bind(sock.value(), reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    if (res < 0) {
        _close(sock);  // Reset the socket descriptor on error
        return tl::make_unexpected("Failed to bind CAN socket: " + std::string(strerror(errno)));
    }

    return {};
}

auto _to_linux_can_frame(suc_util::CanFrame && frame) -> can_frame {
    can_frame linux_can_frame{};
    if (frame.is_extended) {
        linux_can_frame.can_id = (frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG;  // Extended frame ID
    } else {
        linux_can_frame.can_id = frame.id & CAN_SFF_MASK;  // Standard frame ID
    }
    linux_can_frame.len = frame.dlc;
    const auto first = frame.data.begin();
    const auto last = first + frame.dlc;
    std::copy(first, last, std::begin(linux_can_frame.data));
    return linux_can_frame;
}

auto _from_linux_can_frame(const can_frame & linux_can_frame) -> suc_util::CanFrame {
    suc_util::CanFrame frame{};
    frame.is_extended = (linux_can_frame.can_id & CAN_EFF_FLAG) != 0;
    if (frame.is_extended) {
        frame.id = linux_can_frame.can_id & CAN_EFF_MASK;  // Extended frame ID
    } else {
        frame.id = linux_can_frame.can_id & CAN_SFF_MASK;  // Standard frame ID
    }
    frame.dlc = linux_can_frame.len;
    const auto first = std::begin(linux_can_frame.data);
    const auto last = first + linux_can_frame.len;
    std::copy(first, last, frame.data.begin());
    return frame;
}

}  // namespace

suc_util::LinuxCan::LinuxCan() : sock_tx(std::nullopt), sock_rx(std::nullopt) {}

auto suc_util::LinuxCan::close() -> tl::expected<void, std::string> {
    auto res_tx = _close(this->sock_tx);
    auto res_rx = _close(this->sock_rx);

    if (!res_tx || !res_rx) {
        return tl::make_unexpected(
            "Failed to close sockets: \n    TX: " + res_tx.error() + "\n    RX: " + res_rx.error());
    }

    return {};
}

auto suc_util::LinuxCan::init(const std::string ifname) -> tl::expected<void, std::string> {
    auto res_tx = _init(this->sock_tx, ifname);
    auto res_rx = _init(this->sock_rx, ifname);

    if (!res_tx || !res_rx) {
        return tl::make_unexpected(
            "Failed to initialize sockets: \n    TX: " + res_tx.error() +
            "\n    RX: " + res_rx.error());
    }

    return {};
}

auto suc_util::LinuxCan::send_linux_can_frame(can_frame && frame)
    -> tl::expected<void, std::string> {
    if (!this->sock_tx) {
        return tl::make_unexpected("CAN socket is not initialized");
    }

    if (frame.len > CAN_MAX_DLEN) {
        return tl::make_unexpected("DLC exceeds maximum allowed CAN data length");
    }

    // Send the CAN frame
    const auto bytes_to_write = sizeof(frame);
    const auto bytes_written = ::write(this->sock_tx.value(), &frame, bytes_to_write);
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

auto suc_util::LinuxCan::recv_linux_can_frame() -> tl::expected<can_frame, std::string> {
    if (!this->sock_rx) {
        return tl::make_unexpected("CAN socket is not initialized");
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);                        // Clear the set
    FD_SET(this->sock_rx.value(), &read_fds);  // Add the socket to the set

    // Set a timeout for select (5 ms)
    constexpr int TIMEOUT_MS = 5;
    struct timeval timeout = {
        TIMEOUT_MS / 1000,                                 // seconds
        (static_cast<int64_t>(TIMEOUT_MS % 1000)) * 1000,  // microseconds
    };

    // Wait for data to be available on the socket
    const auto nfds = this->sock_rx.value() + 1;  // highest file descriptor + 1
    auto res = ::select(nfds, &read_fds, nullptr, nullptr, &timeout);
    if (res < 0) {
        return tl::make_unexpected("select() failed: " + std::string(strerror(errno)));
    } else if (res == 0) {
        return tl::make_unexpected("No data available on CAN socket within timeout period");
    }

    // Read the CAN frame from the socket
    struct can_frame frame {};
    const auto bytes_to_read = sizeof(frame);
    const auto bytes_read = ::read(this->sock_rx.value(), &frame, bytes_to_read);
    if (bytes_read < 0) {
        return tl::make_unexpected("read() failed: " + std::string(strerror(errno)));
    }
    if (static_cast<size_t>(bytes_read) != bytes_to_read) {
        return tl::make_unexpected(
            "Incomplete CAN frame read: expected " + std::to_string(bytes_to_read) +
            " bytes, got " + std::to_string(bytes_read) + " bytes");
    }

    return frame;
}

auto suc_util::LinuxCan::send_frame(suc_util::CanFrame && frame)
    -> tl::expected<void, std::string> {
    auto linux_can_frame = _to_linux_can_frame(std::move(frame));
    return this->send_linux_can_frame(std::move(linux_can_frame));
}

auto suc_util::LinuxCan::recv_frame() -> tl::expected<CanFrame, std::string> {
    auto linux_can_frame_result = this->recv_linux_can_frame();
    if (!linux_can_frame_result) {
        return tl::make_unexpected(
            "Failed to receive CAN frame: " + linux_can_frame_result.error());
    }
    return _from_linux_can_frame(std::move(linux_can_frame_result.value()));
}