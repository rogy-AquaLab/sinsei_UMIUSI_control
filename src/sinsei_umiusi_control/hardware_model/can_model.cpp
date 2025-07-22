#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

#include <string>

#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/state/thruster.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::CanModel::CanModel(std::shared_ptr<suc::util::CanInterface> can, std::array<int, 4> vesc_ids)
: can(std::move(can)),
  vesc_models{{
      can::VescModel(vesc_ids[0]),
      can::VescModel(vesc_ids[1]),
      can::VescModel(vesc_ids[2]),
      can::VescModel(vesc_ids[3]),
  }} {}

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
            std::array<suc::state::thruster::Rpm, 4>, suc::state::main_power::BatteryCurrent,
            suc::state::main_power::BatteryVoltage, suc::state::main_power::Temperature,
            suc::state::main_power::WaterLeaked>,
        std::string> {
    std::array<suc::state::thruster::Rpm, 4> rpm;
    // suc::state::main_power::BatteryCurrent battery_current;
    // suc::state::main_power::BatteryVoltage battery_voltage;
    // suc::state::main_power::Temperature temperature;

    auto frame = this->can->recv_frame();
    if (!frame) {
        return tl::make_unexpected("Failed to receive CAN frame: " + frame.error());
    }

    // フレームを各モデルに渡していく

    auto success = false;
    std::string error_message;

    // TODO: この位置に`can::MainPowerModel`の処理を追加する

    for (size_t i = 0; i < 4; ++i) {
        if (success) {
            // すでに成功したモデルがある場合はfor文ごとスキップ
            break;
        }
        auto rpm_res = this->vesc_models[i].get_rpm(frame.value());
        if (!rpm_res) {
            error_message += "    VESC " + std::to_string(i + 1) + ": " + rpm_res.error() + "\n";
            continue;
        }
        auto rpm_opt = rpm_res.value();
        if (!rpm_opt) {
            // フレームにRPMの情報が含まれていない場合はスキップ
            continue;
        }
        rpm[i] = rpm_opt.value();
        success = true;
    }

    // すべてのモデルでフレームを処理できなかった場合はエラー
    if (!success) {
        return tl::make_unexpected(
            "Failed to handle CAN frame \"" + std::to_string(frame.value().id) +
            "\" in all models: \n" + error_message);
    }

    // FIXME: 仮の値を返している
    return std::make_tuple(
        rpm, suc::state::main_power::BatteryCurrent{0.0},
        suc::state::main_power::BatteryVoltage{0.0}, suc::state::main_power::Temperature{0},
        suc::state::main_power::WaterLeaked{false});
}

auto suchm::CanModel::on_write(
    std::array<suc::cmd::thruster::EscEnabled, 4> /*thruster_esc_enabled*/,
    std::array<suc::cmd::thruster::ServoEnabled, 4> /*thruster_servo_enabled*/,
    std::array<suc::cmd::thruster::Angle, 4> thruster_angle,
    std::array<suc::cmd::thruster::DutyCycle, 4> thruster_duty_cycle,
    suc::cmd::main_power::Enabled main_power_enabled,
    suc::cmd::led_tape::Color led_tape_color) -> tl::expected<void, std::string> {
    for (size_t i = 0; i < 4; ++i) {
        // TODO: `thruster_esc_enabled`と`thruster_servo_enabled`の処理を実装する

        auto duty = thruster_duty_cycle[i].value;
        auto duty_frame = this->vesc_models[i].make_duty_frame(duty);
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