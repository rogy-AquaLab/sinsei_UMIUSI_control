#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_BMS_CONTROLLER_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_BMS_CONTROLLER_HPP

#include <array>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <string>

#include "sinsei_umiusi_control/state/bms.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_msgs/msg/bms_state.hpp"

namespace sinsei_umiusi_control::controller {

// Harmony16互換BMSの状態をtopicへ出す暫定コントローラ
// UMIUSIではGateControllerが出しているが、AlexandriteではGateControllerを使わないため分けている
class BmsController : public controller_interface::ControllerInterface {
  private:
    struct State {
        state::bms::Health health;
        state::bms::Voltages voltages;
        state::bms::Currents currents;
        state::bms::CapacityState capacity;
        state::bms::CellVoltageRange cell_voltage_range;
        state::bms::ChargeStatus charge_status;
        state::bms::Status status;
        state::bms::CellCount cell_count;
        std::array<state::bms::Cell, state::bms::CELL_COUNT> cells;
        state::bms::Temperature balance_ic_temperature;
        state::bms::Temperature mosfet_temperature;
        state::bms::Temperature ambient_temperature;
        std::array<state::bms::Temperature, state::bms::ADDITIONAL_TEMPERATURE_COUNT>
            additional_temperatures;
    };

    State state{};
    util::interface_accessor::InterfaceDataContainer state_interface_data;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher;
    rclcpp::Publisher<sinsei_umiusi_msgs::msg::BmsState>::SharedPtr bms_state_publisher;

    std::string state_topic_prefix{"state/"};

  public:
    BmsController() = default;

    auto command_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto state_interface_configuration() const
        -> controller_interface::InterfaceConfiguration override;
    auto on_init() -> CallbackReturn override;
    auto on_configure(const rclcpp_lifecycle::State & previous_state) -> CallbackReturn override;
    auto update(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> controller_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::controller

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_BMS_CONTROLLER_HPP
