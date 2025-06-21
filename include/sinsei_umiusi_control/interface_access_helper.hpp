#ifndef SINSEI_UMIUSI_CONTROL_INTERFACE_ACCESS_HELPER_HPP
#define SINSEI_UMIUSI_CONTROL_INTERFACE_ACCESS_HELPER_HPP

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <optional>
#include <string>
#include <vector>
namespace sinsei_umiusi_control {

template <typename NodeLike>
class InterfaceAccessHelper {
  public:
    InterfaceAccessHelper(
        NodeLike * node, std::vector<hardware_interface::LoanedCommandInterface> & cmd_interfaces,
        std::vector<std::string> & cmd_names,
        std::vector<hardware_interface::LoanedStateInterface> & state_interfaces,
        std::vector<std::string> & state_names)
    : node_(node),
      cmd_interfaces_(cmd_interfaces),
      cmd_names_(cmd_names),
      state_interfaces_(state_interfaces),
      state_names_(state_names) {}

    auto get_index(const std::string & name, const std::vector<std::string> & names)
        -> std::optional<size_t> {
        auto it = std::find(names.begin(), names.end(), name);
        if (it != names.end()) {
            auto index = std::distance(names.begin(), it);
            return index;
        }
        return std::nullopt;
    }

    auto set_cmd_value(const std::string & name, const double & value) -> void {
        auto index_opt = this->get_index(name, cmd_names_);
        if (!index_opt.has_value()) {
            RCLCPP_ERROR(node_->get_logger(), "Command interface not found: %s", name.c_str());
            return;
        }
        size_t index = index_opt.value();
        if (index >= cmd_interfaces_.size()) {
            RCLCPP_ERROR(
                node_->get_logger(), "Index out of range for command interfaces: %s", name.c_str());
            return;
        }
        auto & interface = cmd_interfaces_[index];
        if (!interface.set_value(value)) {
            RCLCPP_WARN(
                node_->get_logger(), "Failed to set value for: %s", interface.get_name().c_str());
        }
    }

    template <typename T>
    auto get_state_value(const std::string & name, T & target) -> void {
        auto index_opt = this->get_index(name, state_names_);
        if (!index_opt.has_value()) {
            RCLCPP_ERROR(node_->get_logger(), "State interface not found: %s", name.c_str());
            return;
        }
        size_t index = index_opt.value();
        if (index >= state_interfaces_.size()) {
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
    std::vector<std::string> & cmd_names_;
    std::vector<hardware_interface::LoanedStateInterface> & state_interfaces_;
    std::vector<std::string> & state_names_;
};

}  // namespace sinsei_umiusi_control

#endif  // SINSEI_UMIUSI_CONTROL_INTERFACE_ACCESS_HELPER_HPP