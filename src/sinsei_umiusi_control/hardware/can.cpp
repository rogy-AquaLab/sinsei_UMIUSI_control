#include "sinsei_umiusi_control/hardware/can.hpp"

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include "sinsei_umiusi_control/hardware_model/impl/linux_can.hpp"
#include "sinsei_umiusi_control/state/bms.hpp"
#include "sinsei_umiusi_control/state/can.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"
#include "sinsei_umiusi_control/util/string.hpp"

using namespace sinsei_umiusi_control::hardware;

Can::~Can() {
    if (!this->model) {
        RCLCPP_ERROR(this->get_logger(), "Can model is not initialized.");
        return;
    }

    auto res = this->model->on_destroy();
    if (!res) {
        RCLCPP_ERROR(
            this->get_logger(), "\n  Failed to destroy Can model: %s", res.error().c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Can model destroyed successfully.");
    }
}

auto Can::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
    -> hardware_interface::CallbackReturn {
    this->hardware_interface::SystemInterface::on_init(params);

    // FIXME: URDF側での名前付きGPIO設定を導入するまでは、既存のハードウェアパラメータを維持する。
    // CanModel自体は、この固定長の表現には依存していない
    constexpr size_t LEGACY_THRUSTER_COUNT = 4;
    auto thruster_configs = std::vector<hardware_model::CanModel::ThrusterConfig>{};
    auto thruster_names = std::vector<std::string>{};
    thruster_configs.reserve(LEGACY_THRUSTER_COUNT);
    thruster_names.reserve(LEGACY_THRUSTER_COUNT);
    for (size_t i = 0; i < LEGACY_THRUSTER_COUNT; ++i) {
        auto vesc_id_key = "vesc" + std::to_string(i + 1) + "_id";
        auto vesc_id_str = util::find_param(params.hardware_info.hardware_parameters, vesc_id_key);
        if (!vesc_id_str) {
            RCLCPP_ERROR(
                this->get_logger(), "Parameter '%s' not found in hardware parameters.",
                vesc_id_key.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        const auto vesc_id_res = util::from_chars_expected<unsigned int>(vesc_id_str.value());
        if (!vesc_id_res || vesc_id_res.value() > UINT8_MAX) {
            RCLCPP_ERROR(
                this->get_logger(), "Invalid VESC ID '%s' for parameter '%s'",
                vesc_id_str.value().c_str(), vesc_id_key.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        const auto thruster_name = "thruster" + std::to_string(i + 1);
        thruster_configs.push_back(hardware_model::CanModel::ThrusterConfig{
            thruster_name,
            static_cast<hardware_model::can::VescModel::Id>(vesc_id_res.value()),
        });
        thruster_names.push_back(thruster_name);
    }

    this->thruster_names = std::move(thruster_names);
    this->cycles_without_bms_updates = 50;
    this->cycles_without_esc_updates.fill(50);

    auto harmony_bms_id_str =
        util::find_param(params.hardware_info.hardware_parameters, "harmony_bms_id");
    if (!harmony_bms_id_str) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'harmony_bms_id' not found.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto harmony_bms_id_res =
        util::from_chars_expected<unsigned int>(harmony_bms_id_str.value());
    if (!harmony_bms_id_res || harmony_bms_id_res.value() > UINT8_MAX) {
        RCLCPP_ERROR(
            this->get_logger(), "Invalid Harmony BMS ID '%s'", harmony_bms_id_str.value().c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    this->model.emplace(
        std::make_shared<hardware_model::impl::LinuxCan>(), std::move(thruster_configs),
        static_cast<hardware_model::can::HarmonyBmsModel::Id>(harmony_bms_id_res.value()));

    auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(this->get_logger(), "\n  Failed to initialize Can: %s", res.error().c_str());
        // CANの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

auto Can::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hardware_interface::return_type {
    if (!this->model) {
        this->set_state("can/health", util::to_interface_data(state::can::Health{false}));
        this->set_state("bms/health", util::to_interface_data(state::bms::Boolean{false}));
        for (const auto & name : this->thruster_names) {
            this->set_state(
                name + "/esc/health", util::to_interface_data(state::thruster::esc::Health{false}));
        }

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Can model is not initialized");
        return hardware_interface::return_type::OK;
    }

    auto res = this->model->on_read();
    if (!res) {
        this->set_state("can/health", util::to_interface_data(state::can::Health{false}));
        this->set_state("bms/health", util::to_interface_data(state::bms::Boolean{false}));
        for (const auto & name : this->thruster_names) {
            this->set_state(
                name + "/esc/health", util::to_interface_data(state::thruster::esc::Health{false}));
        }

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to read CAN data: %s",
            res.error().c_str());
        return hardware_interface::return_type::OK;
    }

    const auto & read_batch = res.value();
    auto esc_updated = std::array<bool, 4>{};
    auto bms_updated = false;
    for (const auto & state : read_batch.states) {
        switch (state.index()) {
            case 0: {  // Rpm
                const auto & [thruster_name, rpm] = std::get<0>(state);
                this->set_state(thruster_name + "/esc/rpm", util::to_interface_data(rpm));
                const auto it = std::find(
                    this->thruster_names.begin(), this->thruster_names.end(), thruster_name);
                if (it != this->thruster_names.end()) {
                    esc_updated[static_cast<std::size_t>(it - this->thruster_names.begin())] = true;
                }
                break;
            }
            case 1: {  // ESC Voltage
                const auto & [thruster_name, voltage] = std::get<1>(state);
                this->set_state(thruster_name + "/esc/voltage", util::to_interface_data(voltage));
                const auto it = std::find(
                    this->thruster_names.begin(), this->thruster_names.end(), thruster_name);
                if (it != this->thruster_names.end()) {
                    esc_updated[static_cast<std::size_t>(it - this->thruster_names.begin())] = true;
                }
                break;
            }
            case 2: {  // ESC WaterLeaked
                const auto & [thruster_name, water_leaked] = std::get<2>(state);
                this->set_state(
                    thruster_name + "/esc/water_leaked", util::to_interface_data(water_leaked));
                const auto it = std::find(
                    this->thruster_names.begin(), this->thruster_names.end(), thruster_name);
                if (it != this->thruster_names.end()) {
                    esc_updated[static_cast<std::size_t>(it - this->thruster_names.begin())] = true;
                }
                break;
            }
            case 3: {  // ESC heartbeat
                const auto & [thruster_name, _health] = std::get<3>(state);
                const auto it = std::find(
                    this->thruster_names.begin(), this->thruster_names.end(), thruster_name);
                if (it != this->thruster_names.end()) {
                    esc_updated[static_cast<std::size_t>(it - this->thruster_names.begin())] = true;
                }
                break;
            }
            case 4: {  // Harmony BMS
                const auto & bms = std::get<hardware_model::can::HarmonyBmsModel::State>(state);
                const auto set_scalar = [this](const std::string & name, double value) {
                    this->set_state(name, util::to_interface_data(state::bms::Scalar{value}));
                };
                const auto set_bool = [this](const std::string & name, bool value) {
                    this->set_state(name, util::to_interface_data(state::bms::Boolean{value}));
                };

                set_scalar("bms/pack_voltage", bms.pack_voltage);
                set_scalar("bms/charger_voltage", bms.charger_voltage);
                set_scalar("bms/input_current", bms.input_current);
                set_scalar("bms/measured_current", bms.measured_current);
                set_scalar("bms/net_consumed_charge", bms.net_consumed_charge);
                set_scalar("bms/net_consumed_energy", bms.net_consumed_energy);
                set_scalar("bms/state_of_charge", bms.state_of_charge);
                set_scalar("bms/state_of_health", bms.state_of_health);
                set_scalar("bms/cell_voltage_min", bms.cell_voltage_min);
                set_scalar("bms/cell_voltage_max", bms.cell_voltage_max);
                set_scalar("bms/cell_temperature_max", bms.cell_temperature_max);
                set_bool("bms/charging", bms.charging);
                set_bool("bms/balancing", bms.balancing);
                set_bool("bms/charge_allowed", bms.charge_allowed);
                set_scalar("bms/humidity_sensor_temperature", bms.humidity_sensor_temperature);
                set_scalar("bms/relative_humidity", bms.relative_humidity);
                set_scalar("bms/balance_ic_temperature", bms.balance_ic_temperature);
                set_scalar("bms/total_charged_charge", bms.total_charged_charge);
                set_scalar("bms/total_charged_energy", bms.total_charged_energy);
                set_scalar("bms/total_discharged_charge", bms.total_discharged_charge);
                set_scalar("bms/total_discharged_energy", bms.total_discharged_energy);
                this->set_state(
                    "bms/cell_count", util::to_interface_data(state::bms::Count{bms.cell_count}));
                this->set_state(
                    "bms/temperature_count",
                    util::to_interface_data(state::bms::Count{bms.temperature_count}));
                this->set_state(
                    "bms/data_version",
                    util::to_interface_data(state::bms::Count{bms.data_version}));
                this->set_state(
                    "bms/power_switch_state", util::to_interface_data(state::bms::PowerSwitchState{
                                                  static_cast<uint8_t>(bms.power_switch_state)}));
                this->set_state(
                    "bms/fault_flags",
                    util::to_interface_data(state::bms::FaultFlags{bms.fault_flags}));
                for (std::size_t i = 0; i < bms.cell_voltages.size(); ++i) {
                    set_scalar("bms/cell_voltage_" + std::to_string(i), bms.cell_voltages[i]);
                    set_bool("bms/cell_balancing_" + std::to_string(i), bms.cell_balancing[i]);
                }
                for (std::size_t i = 0; i < bms.temperatures.size(); ++i) {
                    set_scalar("bms/temperature_" + std::to_string(i), bms.temperatures[i]);
                }
                for (std::size_t i = 0; i < 5; ++i) {
                    auto chunk = state::bms::StatusChunk{};
                    std::copy_n(bms.status.begin() + i * 8, 8, chunk.value.begin());
                    this->set_state(
                        "bms/status_chunk_" + std::to_string(i),
                        util::to_interface_data(std::move(chunk)));
                }
                bms_updated = true;
                break;
            }
        }
    }

    constexpr std::size_t MAX_CYCLES_WITHOUT_NODE_UPDATES = 50;
    if (bms_updated) {
        this->cycles_without_bms_updates = 0;
    } else if (this->cycles_without_bms_updates < MAX_CYCLES_WITHOUT_NODE_UPDATES) {
        ++this->cycles_without_bms_updates;
    }
    this->set_state(
        "bms/health", util::to_interface_data(state::bms::Boolean{
                          this->cycles_without_bms_updates < MAX_CYCLES_WITHOUT_NODE_UPDATES}));

    for (std::size_t i = 0; i < this->thruster_names.size(); ++i) {
        if (esc_updated[i]) {
            this->cycles_without_esc_updates[i] = 0;
        } else if (this->cycles_without_esc_updates[i] < MAX_CYCLES_WITHOUT_NODE_UPDATES) {
            ++this->cycles_without_esc_updates[i];
        }
        this->set_state(
            this->thruster_names[i] + "/esc/health",
            util::to_interface_data(state::thruster::esc::Health{
                this->cycles_without_esc_updates[i] < MAX_CYCLES_WITHOUT_NODE_UPDATES}));
    }

    if (!read_batch.states.empty()) {
        this->cycles_without_updates = 0;
    }

    // この周期数だけ状態更新がなければCANを異常とみなす
    constexpr std::size_t MAX_CYCLES_WITHOUT_UPDATES = 50;
    if (!read_batch.error_message.empty()) {
        this->set_state("can/health", util::to_interface_data(state::can::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to read CAN data: %s",
            read_batch.error_message.c_str());
    } else if (read_batch.states.empty()) {
        if (this->cycles_without_updates < MAX_CYCLES_WITHOUT_UPDATES) {
            ++this->cycles_without_updates;
        }
        if (this->cycles_without_updates >= MAX_CYCLES_WITHOUT_UPDATES) {
            this->set_state("can/health", util::to_interface_data(state::can::Health{false}));
        }
    } else {
        this->set_state("can/health", util::to_interface_data(state::can::Health{true}));
    }

    return hardware_interface::return_type::OK;
}

auto Can::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hardware_interface::return_type {
    if (!this->model) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Can model is not initialized");
        return hardware_interface::return_type::OK;
    }

    auto && power_distribution_enabled =
        util::from_interface_data<cmd::power_distribution::Enabled>(
            this->get_command("power_distribution/enabled"));
    auto && led_tape_color =
        util::from_interface_data<cmd::led_tape::Color>(this->get_command("led_tape/color"));

    auto thruster_commands = std::vector<hardware_model::CanModel::ThrusterCommand>{};
    thruster_commands.reserve(this->thruster_names.size());
    for (const auto & name : this->thruster_names) {
        thruster_commands.push_back(hardware_model::CanModel::ThrusterCommand{
            util::from_interface_data<cmd::thruster::esc::Allowed>(
                this->get_command(name + "/esc/allowed")),
            util::from_interface_data<cmd::thruster::esc::DutyCycle>(
                this->get_command(name + "/esc/duty_cycle")),
            util::from_interface_data<cmd::thruster::servo::Allowed>(
                this->get_command(name + "/servo/allowed")),
            util::from_interface_data<cmd::thruster::servo::Angle>(
                this->get_command(name + "/servo/angle")),
        });
    }

    const auto res =
        this->model->on_write(power_distribution_enabled, thruster_commands, led_tape_color);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to write Can: %s",
            res.error().c_str());
        return hardware_interface::return_type::OK;
    }

    return hardware_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sinsei_umiusi_control::hardware::Can, hardware_interface::SystemInterface)
