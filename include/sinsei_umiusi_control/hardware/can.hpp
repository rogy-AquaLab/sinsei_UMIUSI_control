#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_CAN_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_CAN_HPP

#include <cstddef>
#include <cstdint>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <optional>
#include <rclcpp/macros.hpp>
#include <string>
#include <vector>

#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

namespace sinsei_umiusi_control::hardware {

class Can : public hardware_interface::SystemInterface {
  private:
    struct Actuator {
        std::string name;
        hardware_model::CanModel::MotorType motor_type;
        bool has_servo;
    };

    static constexpr std::size_t MAX_CYCLES_SINCE_NODE_UPDATE = 50;
    static constexpr std::size_t MAX_CYCLES_SINCE_ANY_NODE_UPDATE = 50;

    std::optional<hardware_model::CanModel> model;
    // CanModelへ渡したアクチュエータ設定と同じ順序で保持する
    std::vector<Actuator> actuators;

    // 最後の状態更新から経過した周期数
    std::size_t cycles_since_any_node_update = 0;
    std::size_t cycles_since_bms_update = 0;
    // `actuators`と同じ順序で保持する
    std::vector<std::size_t> cycles_since_vesc_update;

    auto find_actuator_index(const std::string & actuator_name) const
        -> std::optional<std::size_t>;
    auto set_vesc_health(std::size_t index, bool is_ok) -> void;

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Can)

    Can() = default;
    ~Can() override;

    auto on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
        -> hardware_interface::CallbackReturn override;
    auto on_configure(const rclcpp_lifecycle::State & previous_state)
        -> hardware_interface::CallbackReturn override;
    auto read(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
    auto write(const rclcpp::Time & time, const rclcpp::Duration & period)
        -> hardware_interface::return_type override;
};

}  // namespace sinsei_umiusi_control::hardware

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_CAN_HPP
