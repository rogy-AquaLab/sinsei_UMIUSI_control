#ifndef SINSEI_UMIUSI_CONTROL_INTERFACE_ACCESS_HELPER_HPP
#define SINSEI_UMIUSI_CONTROL_INTERFACE_ACCESS_HELPER_HPP

#include <cstddef>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <optional>
#include <string>
#include <vector>
namespace sinsei_umiusi_control {

template <typename NodeLike, std::size_t CmdSize, std::size_t StateSize>
class InterfaceAccessHelper {
  public:
    InterfaceAccessHelper(
        NodeLike * node, std::vector<hardware_interface::LoanedCommandInterface> & cmd_interfaces,
        const char * const (&cmd_names)[CmdSize],
        std::vector<hardware_interface::LoanedStateInterface> & state_interfaces,
        const char * const (&state_names)[StateSize])
    : node_(node),
      cmd_interfaces_(cmd_interfaces),
      cmd_names_(cmd_names),
      state_interfaces_(state_interfaces),
      state_names_(state_names) {}

    template <typename T>
    auto set_cmd_value(const std::size_t & index, const T & value) -> void {
        if (index >= cmd_interfaces_.size()) {
            const std::string name = cmd_names_[index];
            RCLCPP_ERROR(
                node_->get_logger(), "Index out of range for command interfaces: %s", name.c_str());
            return;
        }
        auto & interface = cmd_interfaces_[index];
        if (!interface.set_value(*reinterpret_cast<const double *>(&value))) {
            RCLCPP_WARN(
                node_->get_logger(), "Failed to set value for: %s", interface.get_name().c_str());
        }
    }

    template <typename T>
    auto get_state_value(const std::size_t & index, T & target) -> void {
        if (index >= state_interfaces_.size()) {
            const std::string name = state_names_[index];
            RCLCPP_ERROR(
                node_->get_logger(), "Index out of range for state interfaces: %s", name.c_str());
            return;
        }
        auto & interface = state_interfaces_[index];
        auto opt = interface.get_optional();
        if (!opt.has_value()) {
            RCLCPP_ERROR(
                node_->get_logger(), "State interface not ready: %s", interface.get_name().c_str());
            return;
        }
        target = *reinterpret_cast<T *>(&opt.value());
    }

  private:
    NodeLike * node_;
    std::vector<hardware_interface::LoanedCommandInterface> & cmd_interfaces_;
    const char * const (&cmd_names_)[CmdSize];
    std::vector<hardware_interface::LoanedStateInterface> & state_interfaces_;
    const char * const (&state_names_)[StateSize];
};

}  // namespace sinsei_umiusi_control

#endif  // SINSEI_UMIUSI_CONTROL_INTERFACE_ACCESS_HELPER_HPP