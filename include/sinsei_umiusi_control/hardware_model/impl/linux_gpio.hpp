#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_LINUX_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_LINUX_GPIO_HPP

#include <gpiod.hpp>
#include <string>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace sinsei_umiusi_control::hardware_model::impl {

inline constexpr auto DEFAULT_CONSUMER = "sinsei_umiusi_control";

class LinuxGpioLineRequest : public interface::GpioLineRequest {
  private:
    using GpioValue = interface::GpioValue;

    gpiod::line_bulk bulk;

    auto set_gpiod_values(const std::vector<GpioValue> & values) -> tl::expected<void, std::string>;

  public:
    explicit LinuxGpioLineRequest(gpiod::line_bulk bulk);
    ~LinuxGpioLineRequest() override;

    auto set_values(const std::vector<GpioValue> & values)
        -> tl::expected<void, std::string> override;
    auto size() const noexcept -> std::size_t override;
};

class LinuxGpioChip : public interface::GpioChip {
  private:
    using GpioOutputRequest = interface::GpioOutputRequest;

    std::string chip_path;

    auto request_gpiod_lines(const GpioOutputRequest & request)
        -> tl::expected<gpiod::line_bulk, std::string>;

  public:
    explicit LinuxGpioChip(std::string chip_path);
    ~LinuxGpioChip() override;

    auto request_outputs(GpioOutputRequest request)
        -> tl::expected<std::unique_ptr<interface::GpioLineRequest>, std::string> override;
};

}  // namespace sinsei_umiusi_control::hardware_model::impl

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_LINUX_GPIO_HPP
