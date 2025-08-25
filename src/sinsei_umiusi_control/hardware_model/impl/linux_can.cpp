#include "sinsei_umiusi_control/hardware_model/impl/linux_can.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstddef>
#include <cstring>
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>

namespace suchm = sinsei_umiusi_control::hardware_model;

namespace {

auto _close(std::optional<suchm::impl::FileDescriptor> & sock) -> tl::expected<void, std::string> {
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

auto _init(std::optional<suchm::impl::FileDescriptor> & sock, const std::string & ifname)
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

auto _to_linux_can_frame(suchm::interface::CanFrame && frame) -> can_frame {
    can_frame linux_can_frame{};
    if (frame.is_extended) {
        linux_can_frame.can_id = (frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG;  // Extended frame ID
    } else {
        linux_can_frame.can_id = frame.id & CAN_SFF_MASK;  // Standard frame ID
    }
    linux_can_frame.len = frame.len;
    for (size_t i = 0; i < frame.len; ++i) {
        linux_can_frame.data[i] = std::to_integer<uint8_t>(frame.data[i]);
    }
    return linux_can_frame;
}

auto _from_linux_can_frame(const can_frame & linux_can_frame) -> suchm::interface::CanFrame {
    suchm::interface::CanFrame frame{};
    frame.is_extended = (linux_can_frame.can_id & CAN_EFF_FLAG) != 0;
    if (frame.is_extended) {
        frame.id = linux_can_frame.can_id & CAN_EFF_MASK;  // Extended frame ID
    } else {
        frame.id = linux_can_frame.can_id & CAN_SFF_MASK;  // Standard frame ID
    }
    frame.len = linux_can_frame.len;
    for (size_t i = 0; i < frame.len; ++i) {
        frame.data[i] = std::byte(linux_can_frame.data[i]);
    }
    return frame;
}

}  // namespace

suchm::impl::LinuxCan::LinuxCan() : sock_tx(std::nullopt), sock_rx(std::nullopt) {}

auto suchm::impl::LinuxCan::close() -> tl::expected<void, std::string> {
    auto res_tx = _close(this->sock_tx);
    auto res_rx = _close(this->sock_rx);

    if (!res_tx || !res_rx) {
        return tl::make_unexpected(
            "Failed to close sockets: \n    TX: " + res_tx.error() + "\n    RX: " + res_rx.error());
    }

    return {};
}

auto suchm::impl::LinuxCan::init(const std::string ifname) -> tl::expected<void, std::string> {
    auto res_tx = _init(this->sock_tx, ifname);
    auto res_rx = _init(this->sock_rx, ifname);

    if (!res_tx || !res_rx) {
        return tl::make_unexpected(
            "Failed to initialize sockets: \n    TX: " + res_tx.error() +
            "\n    RX: " + res_rx.error());
    }

    return {};
}

auto suchm::impl::LinuxCan::send_linux_can_frame(can_frame && frame)
    -> tl::expected<void, std::string> {
    if (!this->sock_tx) {
        return tl::make_unexpected("CAN socket is not initialized");
    }

    if (frame.len > CAN_MAX_DLEN) {
        return tl::make_unexpected("len exceeds maximum allowed CAN data length");
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

auto suchm::impl::LinuxCan::recv_linux_can_frame() -> tl::expected<can_frame, std::string> {
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

auto suchm::impl::LinuxCan::send_frame(suchm::interface::CanFrame && frame)
    -> tl::expected<void, std::string> {
    auto linux_can_frame = _to_linux_can_frame(std::move(frame));
    return this->send_linux_can_frame(std::move(linux_can_frame))
        .and_then([this, &linux_can_frame]() {
            this->id_last_sent = linux_can_frame.can_id;
            return tl::expected<void, std::string>{};
        });
}

auto suchm::impl::LinuxCan::recv_frame() -> tl::expected<std::optional<CanFrame>, std::string> {
    return this->recv_linux_can_frame().map([this](can_frame && linux_can_frame) {
        const auto frame = _from_linux_can_frame(std::move(linux_can_frame));
        return std::optional<CanFrame>{
            // 直前に送信したフレームと同じIDなら無視する
            (frame.id == this->id_last_sent) ? std::nullopt : std::make_optional(frame)};
    });
}