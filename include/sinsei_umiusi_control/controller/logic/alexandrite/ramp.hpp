#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ALEXANDRITE_RAMP_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ALEXANDRITE_RAMP_HPP

#include <algorithm>

namespace sinsei_umiusi_control::controller::logic::alexandrite {

// currentからtargetへ、max_step以内だけ近づける（UMIUSIのLinearAccelerationと同じ線形ランプ）
inline auto ramp(const double current, const double target, const double max_step) -> double {
    const auto step = std::max(max_step, 0.0);
    return std::clamp(target, current - step, current + step);
}

}  // namespace sinsei_umiusi_control::controller::logic::alexandrite

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ALEXANDRITE_RAMP_HPP
