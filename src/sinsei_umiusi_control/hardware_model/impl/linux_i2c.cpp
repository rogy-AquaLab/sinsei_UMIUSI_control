#include "sinsei_umiusi_control/hardware_model/impl/linux_i2c.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <vector>

using namespace sinsei_umiusi_control::hardware_model;

namespace {

auto _to_linux_i2c_msg(const interface::I2cMessage & msg) -> i2c_msg {
    i2c_msg linux_msg{};
    linux_msg.addr = msg.address.value;
    linux_msg.flags = (msg.direction == interface::I2cDirection::Read) ? I2C_M_RD : 0;
    linux_msg.flags |= msg.flags;
    linux_msg.len = static_cast<uint16_t>(msg.buffer.length);
    // std::byte* から __u8* (uint8_t*) への変換
    linux_msg.buf = reinterpret_cast<uint8_t *>(const_cast<std::byte *>(msg.buffer.data));

    return linux_msg;
}

}  // namespace

namespace sinsei_umiusi_control::hardware_model::impl {

LinuxI2c::LinuxI2c(std::string device_path)
: fd(std::nullopt), device_path(std::move(device_path)) {}

LinuxI2c::~LinuxI2c() { (void)this->close(); }

auto LinuxI2c::open() -> tl::expected<void, std::string> {
    if (this->fd) {
        return tl::make_unexpected("I2C bus is already open");
    }

    this->fd = ::open(this->device_path.c_str(), O_RDWR);
    if (!this->fd) {
        this->fd.reset();
        return tl::make_unexpected(
            "Failed to open I2C device " + this->device_path + ": " + std::string(strerror(errno)));
    }

    return {};
}

auto LinuxI2c::close() -> tl::expected<void, std::string> {
    if (!this->fd) {
        return tl::make_unexpected("I2C bus is not open");
    }
    auto res = ::close(this->fd.value());
    if (res < 0) {
        return tl::make_unexpected(
            "Failed to close I2C device " + this->device_path + ": " +
            std::string(strerror(errno)));
    }
    this->fd.reset();
    return {};
}

auto LinuxI2c::transfer(const interface::I2cMessage * msgs, std::size_t size)
    -> tl::expected<void, std::string> {
    if (!this->fd) {
        return tl::make_unexpected("I2C bus is not open");
    }

    std::vector<i2c_msg> linux_msgs;
    linux_msgs.reserve(size);
    for (std::size_t i = 0; i < size; ++i) {
        linux_msgs.push_back(_to_linux_i2c_msg(msgs[i]));
    }

    i2c_rdwr_ioctl_data ioctl_data{};
    ioctl_data.msgs = linux_msgs.data();
    ioctl_data.nmsgs = static_cast<uint32_t>(size);

    auto res = ::ioctl(this->fd.value(), I2C_RDWR, &ioctl_data);
    if (res < 0) {
        return tl::make_unexpected("I2C transfer failed: " + std::string(strerror(errno)));
    }

    return {};
}

}  // namespace sinsei_umiusi_control::hardware_model::impl
