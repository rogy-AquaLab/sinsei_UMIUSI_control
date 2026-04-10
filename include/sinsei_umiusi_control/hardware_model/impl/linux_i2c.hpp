#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_LINUX_I2C_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_LINUX_I2C_HPP

#include <optional>
#include <string>

#include "sinsei_umiusi_control/hardware_model/interface/i2c.hpp"

namespace sinsei_umiusi_control::hardware_model::impl {

using FileDescriptor = int;

class LinuxI2c : public interface::I2c {
  private:
    std::optional<FileDescriptor> fd;
    std::string device_path;

  public:
    LinuxI2c(std::string device_path);
    ~LinuxI2c() override;

    auto open() -> tl::expected<void, std::string> override;
    auto close() -> tl::expected<void, std::string> override;

    auto transfer(const interface::I2cMessage * msgs, std::size_t size)
        -> tl::expected<void, std::string> override;
};

}  // namespace sinsei_umiusi_control::hardware_model::impl

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_LINUX_I2C_HPP
