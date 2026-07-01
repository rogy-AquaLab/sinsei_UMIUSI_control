#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP

#include <cstdint>
#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <vector>

namespace sinsei_umiusi_control::hardware_model::interface {

using GpioOffset = uint32_t;

enum class GpioValue : uint8_t {
    Inactive = 0,
    Active = 1,
};

struct GpioOutputRequest {
    std::vector<GpioOffset> offsets;
    std::vector<GpioValue> initial_values;
    std::string consumer;
};

class GpioLineRequest {
  public:
    GpioLineRequest() = default;
    virtual ~GpioLineRequest() = default;

    virtual auto set_values(const std::vector<GpioValue> & values)
        -> tl::expected<void, std::string> = 0;
};

class GpioChip {
  public:
    GpioChip() = default;
    virtual ~GpioChip() = default;

    virtual auto request_outputs(GpioOutputRequest request)
        -> tl::expected<std::unique_ptr<GpioLineRequest>, std::string> = 0;
};

constexpr auto to_gpio_value(const bool value) noexcept -> GpioValue {
    return value ? GpioValue::Active : GpioValue::Inactive;
}

}  // namespace sinsei_umiusi_control::hardware_model::interface

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP
