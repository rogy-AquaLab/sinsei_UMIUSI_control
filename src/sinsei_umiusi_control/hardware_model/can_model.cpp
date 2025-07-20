#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

#include <string>

#include "sinsei_umiusi_control/state/thruster.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::CanModel::CanModel(std::shared_ptr<suc::util::CanInterface> can) : can(std::move(can)) {}

auto suchm::CanModel::on_init() -> tl::expected<void, std::string> {
    auto res = this->can->init("can0");
    if (!res) {
        return tl::make_unexpected("Failed to initialize CAN interface: " + res.error());
    }
    return {};
}

auto suchm::CanModel::on_destroy() -> tl::expected<void, std::string> {
    // TODO: ここで念のためスラスターを停止しておく

    auto res = this->can->close();
    if (!res) {
        return tl::make_unexpected("Failed to close CAN interface: " + res.error());
    }
    return {};
}

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

    auto frame = this->can->recv_frame();
    if (!frame) {
        return tl::make_unexpected("Failed to receive CAN frame: " + frame.error());
    }

    auto success = false;
    std::string error_message;

    for (size_t i = 0; i < 4; ++i) {
        if (!success) {
            auto res = this->vesc_models[i].handle_frame(frame.value());
            if (!res) {
                error_message += "    VESC " + std::to_string(i + 1) + ": " + res.error() + "\n";
                continue;
            }
            rpm[i] = res.value();
            success = true;
        }
    }

    if (!success) {
        return tl::make_unexpected(
            "Failed to handle CAN frame \"" + std::to_string(frame.value().id) +
            "\" in all models: \n" + error_message);
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
        auto duty_frame = this->vesc_models[i].make_duty_frame(thrust);
        if (!duty_frame) {
            return tl::make_unexpected(
                "Failed to create duty frame for thruster " + std::to_string(i + 1) + ": " +
                duty_frame.error());
        }

        auto duty_send_res = this->can->send_frame(std::move(duty_frame.value()));
        if (!duty_send_res) {
            return tl::make_unexpected(
                "Failed to send duty frame for thruster " + std::to_string(i + 1) + ": " +
                duty_send_res.error());
        }

        auto angle = thruster_angle[i].value;
        auto angle_frame = this->vesc_models[i].make_servo_angle_frame(angle);
        if (!angle_frame) {
            return tl::make_unexpected(
                "Failed to create servo angle frame for thruster " + std::to_string(i + 1) + ": " +
                angle_frame.error());
        }

        auto angle_send_res = this->can->send_frame(std::move(angle_frame.value()));
        if (!angle_send_res) {
            return tl::make_unexpected(
                "Failed to send servo angle frame for thruster " + std::to_string(i + 1) + ": " +
                angle_send_res.error());
        }
    }

    auto write_res = this->on_write(main_power_enabled, led_tape_color);
    if (!write_res) {
        return tl::make_unexpected("Failed to write main power and LED tape: " + write_res.error());
    }
    return {};
}

auto suchm::CanModel::on_write(
    suc::cmd::main_power::Enabled /*main_power_enabled*/,
    suc::cmd::led_tape::Color /*led_tape_color*/) -> tl::expected<void, std::string> {
    // TODO: `main_power_enabled`と`led_tape_color`の処理を実装する
    //return tl::make_unexpected("Not implemented");
    return {};
}