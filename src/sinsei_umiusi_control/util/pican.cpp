#include "sinsei_umiusi_control/util/pican.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstddef>
#include <cstring>

namespace suc_util = sinsei_umiusi_control::util;

suc_util::Pican::~Pican() {
    if (this->sock >= 0) {
        close(this->sock);
    }
}

auto suc_util::Pican::init(const std::string ifname) -> tl::expected<void, std::string> {
    this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->sock < 0) {
        return tl::unexpected<std::string>(
            "Failed to create CAN socket: " + std::string(strerror(errno)));
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);

    if (ioctl(this->sock, SIOCGIFINDEX, &ifr) < 0) {
        close(this->sock);
        this->sock = -1;
        return tl::unexpected<std::string>(
            "Failed to get interface index: " + std::string(strerror(errno)));
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int loopback = 0;
    setsockopt(this->sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    setsockopt(this->sock, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);

    if (bind(this->sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        close(this->sock);
        this->sock = -1;
        return tl::unexpected<std::string>(
            "Failed to bind CAN socket: " + std::string(strerror(errno)));
    }

    return {};
}

auto suc_util::Pican::send_frame(uint32_t id, const uint8_t * data, size_t length, bool is_extended)
    -> tl::expected<void, std::string> {
    if (this->sock < 0) {
        return tl::unexpected<std::string>("CAN socket is not initialized");
    }

    if (length > CAN_MAX_DLEN) {
        return tl::unexpected<std::string>("DLC exceeds maximum allowed CAN data length");
    }

    can_frame frame{};
    frame.can_id = is_extended ? (id | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
    frame.can_dlc = static_cast<__u8>(length);
    std::memcpy(frame.data, data, length);

    ssize_t nbytes = write(this->sock, &frame, sizeof(can_frame));
    if (nbytes < 0) {
        return tl::unexpected<std::string>(
            "Failed to write CAN frame: " + std::string(strerror(errno)));
    }

    if (static_cast<size_t>(nbytes) < sizeof(can_frame)) {
        return tl::unexpected<std::string>("Partial CAN frame written");
    }

    return {};
}

auto suc_util::Pican::send_stdframe(uint32_t id, const uint8_t * data, size_t length)
    -> tl::expected<void, std::string> {
    return send_frame(id, data, length, false);
}

auto suc_util::Pican::send_extframe(uint32_t id, const uint8_t * data, size_t length)
    -> tl::expected<void, std::string> {
    return send_frame(id, data, length, true);
}

auto suc_util::Pican::receive_frame() -> tl::expected<void, std::string> {
    if (this->sock < 0) {
        return tl::unexpected<std::string>("CAN socket is not initialized");
    }

    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(this->sock, &rdfs);

    int timeout_ms = 1000;
    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (static_cast<__suseconds_t>(timeout_ms % 1000)) * 1000;

    int ret = select(this->sock + 1, &rdfs, nullptr, nullptr, &timeout);
    if (ret < 0) {
        return tl::unexpected<std::string>("select() failed: " + std::string(strerror(errno)));
    } else if (ret == 0) {
        return tl::unexpected<std::string>("select() timeout");
    }

    struct can_frame frame {};
    ssize_t nbytes = read(this->sock, &frame, sizeof(frame));
    if (nbytes < 0) {
        return tl::unexpected<std::string>("read() failed: " + std::string(strerror(errno)));
    }
    if (static_cast<size_t>(nbytes) != sizeof(struct can_frame)) {
        return tl::unexpected<std::string>("Incomplete CAN frame read");
    }

    CanFrame result;
    result.id = frame.can_id;
    result.dlc = frame.can_dlc;
    std::memcpy(result.data.data(), frame.data, CAN_MAX_DLEN);

    return {};
}