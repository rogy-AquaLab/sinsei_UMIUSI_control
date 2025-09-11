#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_HEADLIGHTS_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_HEADLIGHTS_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <rclcpp/macros.hpp>

#include "sinsei_umiusi_control/hardware_model/headlights_model.hpp"

namespace sinsei_umiusi_control::hardware {

class Headlights : public hardware_interface::SystemInterface {
  private:
    std::optional<hardware_model::HeadlightsModel> model;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Headlights)

    Headlights() = default;

    auto on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
        -> hardware_interface::CallbackReturn override;
    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
    auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_HEADLIGHTS_HPP
