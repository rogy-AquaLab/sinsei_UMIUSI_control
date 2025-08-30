#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_LOGIC_INTERFACE_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_LOGIC_INTERFACE_HPP

#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::controller::logic {

enum class ControlMode {
    FeedForward,
    FeedBack,
};

inline auto control_mode_to_str(const ControlMode & mode) -> std::string {
    switch (mode) {
        case ControlMode::FeedForward:
            return "ff";
        case ControlMode::FeedBack:
            return "fb";
        default:
            return "unknown";  // unreachable
    }
}

inline auto get_mode_from_str(const std::string_view & str)
    -> tl::expected<ControlMode, std::string> {
    if (str == "ff") {
        return ControlMode::FeedForward;
    }
    if (str == "fb") {
        return ControlMode::FeedBack;
    }
    return tl::make_unexpected("Invalid control mode: " + std::string(str));
}

template <typename Input, typename Output>
class LogicInterface {
  public:
    LogicInterface() = default;
    virtual ~LogicInterface() = default;

    virtual auto control_mode() const -> ControlMode = 0;
    virtual auto init(double time, const Input & input, const Output & output) -> Output = 0;
    virtual auto update(double time, double duration, const Input & input) -> Output = 0;
};

}  // namespace sinsei_umiusi_control::controller::logic

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_LOGIC_INTERFACE_HPP
