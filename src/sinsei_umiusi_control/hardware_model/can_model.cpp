#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

#include "sinsei_umiusi_control/state/thruster.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::CanModel::CanModel(std::shared_ptr<suc::util::CanInterface> can, util::ThrusterMode mode)
: can(std::move(can)), mode(mode) {}

auto suchm::CanModel::on_read()
    -> tl::expected<
        std::tuple<
            std::array<suc::state::thruster::ServoCurrent, 4>,
            std::array<suc::state::thruster::Rpm, 4>, suc::state::main_power::BatteryCurrent,
            suc::state::main_power::BatteryVoltage, suc::state::main_power::Temperature,
            suc::state::main_power::WaterLeaked>,
        std::string> {
    // std::array<suc::state::thruster::ServoCurrent, 4> servo_current;
    std::array<suc::state::thruster::Rpm, 4> rpm;
    // suc::state::main_power::BatteryCurrent battery_current;
    // suc::state::main_power::BatteryVoltage battery_voltage;
    // suc::state::main_power::Temperature temperature;

    for (size_t i = 0; i < 4; ++i) {
        auto rpm_res = this->vesc_models[i].get_rpm();
        if (!rpm_res) {
            return tl::make_unexpected(
                "Failed to get RPM for thruster " + std::to_string(i + 1) + ": " + rpm_res.error());
        }
        rpm[i] = suc::state::thruster::Rpm{rpm_res.value()};
    }

    // FIXME: 仮の値を返している
    return std::make_tuple(
        std::array<suc::state::thruster::ServoCurrent, 4>{}, rpm,
        suc::state::main_power::BatteryCurrent{0.0}, suc::state::main_power::BatteryVoltage{0.0},
        suc::state::main_power::Temperature{0}, suc::state::main_power::WaterLeaked{false});
}

auto suchm::CanModel::on_write(
    std::array<suc::cmd::thruster::EscEnabled, 4> /*thruster_esc_enabled*/,
    std::array<suc::cmd::thruster::ServoEnabled, 4> /*thruster_servo_enabled*/,
    std::array<suc::cmd::thruster::Angle, 4> thruster_angle,
    std::array<suc::cmd::thruster::Thrust, 4> thruster_thrust,
    suc::cmd::main_power::Enabled main_power_enabled,
    suc::cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string> {
    for (size_t i = 0; i < 4; ++i) {
        // TODO: `thruster_esc_enabled`と`thruster_servo_enabled`の処理を実装する

        auto thrust = thruster_thrust[i].value;
        // TODO: dutyとrpmどちらを指定するか検討する
        auto thrust_res = this->vesc_models[i].set_rpm(thrust);
        if (!thrust_res) {
            return tl::make_unexpected(
                "Failed to set thrust for thruster " + std::to_string(i + 1) + ": " +
                thrust_res.error());
        }

        auto angle = thruster_angle[i].value;
        auto angle_res = this->vesc_models[i].set_servo_angle(angle);
        if (!angle_res) {
            return tl::make_unexpected(
                "Failed to set servo angle for thruster " + std::to_string(i + 1) + ": " +
                angle_res.error());
        }
    }

    auto write_res = this->on_write(main_power_enabled, led_tape_color);
    return write_res;
}

auto suchm::CanModel::on_write(
    suc::cmd::main_power::Enabled /*main_power_enabled*/,
    suc::cmd::led_tape::Color /*led_tape_color*/) -> tl::expected<void, std::string> {
    // TODO: `main_power_enabled`と`led_tape_color`の処理を実装する
    //return tl::make_unexpected("Not implemented");
    return {};
}