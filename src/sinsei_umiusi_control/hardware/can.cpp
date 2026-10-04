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
    this->cycles_since_bms_update = MAX_CYCLES_SINCE_NODE_UPDATE;
    this->cycles_since_esc_update.fill(MAX_CYCLES_SINCE_NODE_UPDATE);

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

auto Can::find_thruster_index(const std::string & thruster_name) const
    -> std::optional<std::size_t> {
    const auto it =
        std::find(this->thruster_names.begin(), this->thruster_names.end(), thruster_name);
    if (it == this->thruster_names.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(it - this->thruster_names.begin());
}

auto Can::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hardware_interface::return_type {
    if (!this->model) {
        this->set_state("can/health", util::to_interface_data(state::can::Health{false}));
        this->set_state("bms/health", util::to_interface_data(state::bms::Health{false}));
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
        this->set_state("bms/health", util::to_interface_data(state::bms::Health{false}));
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
    auto esc_updated = std::array<bool, LEGACY_THRUSTER_COUNT>{};
    auto bms_updated = false;
    for (const auto & state : read_batch.states) {
        switch (state.index()) {
            case 0: {  // Rpm
                const auto & [thruster_name, rpm] = std::get<0>(state);
                this->set_state(thruster_name + "/esc/rpm", util::to_interface_data(rpm));
                if (const auto index = this->find_thruster_index(thruster_name)) {
                    esc_updated[index.value()] = true;
                }
                break;
            }
            case 1: {  // ESC Voltage
                const auto & [thruster_name, voltage] = std::get<1>(state);
                this->set_state(thruster_name + "/esc/voltage", util::to_interface_data(voltage));
                if (const auto index = this->find_thruster_index(thruster_name)) {
                    esc_updated[index.value()] = true;
                }
                break;
            }
            case 2: {  // ESC WaterLeaked
                const auto & [thruster_name, water_leaked] = std::get<2>(state);
                this->set_state(
                    thruster_name + "/esc/water_leaked", util::to_interface_data(water_leaked));
                if (const auto index = this->find_thruster_index(thruster_name)) {
                    esc_updated[index.value()] = true;
                }
                break;
            }
            case 3: {  // ESC heartbeat
                const auto & [thruster_name, _health] = std::get<3>(state);
                if (const auto index = this->find_thruster_index(thruster_name)) {
                    esc_updated[index.value()] = true;
                }
                break;
            }
            case 4: {  // Harmony BMS
                const auto & bms = std::get<hardware_model::can::HarmonyBmsModel::State>(state);
                this->set_state("bms/voltages.pack", bms.voltages.pack);
                this->set_state("bms/voltages.charger", bms.voltages.charger);
                this->set_state("bms/currents.input", bms.currents.input);
                this->set_state("bms/currents.measured", bms.currents.measured);
                this->set_state("bms/capacity_state.state_of_charge", bms.capacity.state_of_charge);
                this->set_state("bms/capacity_state.state_of_health", bms.capacity.state_of_health);
                this->set_state("bms/cell_voltage_range.min", bms.cell_voltage_range.min);
                this->set_state("bms/cell_voltage_range.max", bms.cell_voltage_range.max);
                this->set_state("bms/status", util::to_interface_data(bms.status));
                this->set_state("bms/cell_count", util::to_interface_data(bms.cell_count));
                for (std::size_t i = 0; i < bms.cells.size(); ++i) {
                    const auto prefix = "bms/cell_" + std::to_string(i);
                    this->set_state(prefix + ".voltage", bms.cells[i].voltage);
                    this->set_state(
                        prefix + ".balancing", util::to_interface_data(bms.cells[i].balancing));
                }
                this->set_state(
                    "bms/balance_ic_temperature",
                    util::to_interface_data(bms.balance_ic_temperature));
                this->set_state(
                    "bms/mosfet_temperature", util::to_interface_data(bms.mosfet_temperature));
                this->set_state(
                    "bms/ambient_temperature", util::to_interface_data(bms.ambient_temperature));
                for (std::size_t i = 0; i < bms.additional_temperatures.size(); ++i) {
                    this->set_state(
                        "bms/additional_temperature_" + std::to_string(i),
                        util::to_interface_data(bms.additional_temperatures[i]));
                }

                if (bms.status_updated) {
                    constexpr auto DURATION = 3000;  // ms
                    if (util::has_bms_fault(bms.status.faults)) {
                        RCLCPP_ERROR_THROTTLE(
                            this->get_logger(), *this->get_clock(), DURATION,
                            "\n  Harmony BMS fault: %s", bms.status_text.c_str());
                    } else if (
                        bms.status.power_switch_state == util::BmsPowerSwitchState::Unknown) {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(), *this->get_clock(), DURATION,
                            "\n  Unknown Harmony BMS status: %s", bms.status_text.c_str());
                    }
                }
                bms_updated = true;
                break;
            }
        }
    }

    if (bms_updated) {
        this->cycles_since_bms_update = 0;
    } else if (this->cycles_since_bms_update < MAX_CYCLES_SINCE_NODE_UPDATE) {
        ++this->cycles_since_bms_update;
    }
    this->set_state(
        "bms/health", util::to_interface_data(state::bms::Health{
                          this->cycles_since_bms_update < MAX_CYCLES_SINCE_NODE_UPDATE}));

    for (std::size_t i = 0; i < this->thruster_names.size(); ++i) {
        if (esc_updated[i]) {
            this->cycles_since_esc_update[i] = 0;
        } else if (this->cycles_since_esc_update[i] < MAX_CYCLES_SINCE_NODE_UPDATE) {
            ++this->cycles_since_esc_update[i];
        }
        this->set_state(
            this->thruster_names[i] + "/esc/health",
            util::to_interface_data(state::thruster::esc::Health{
                this->cycles_since_esc_update[i] < MAX_CYCLES_SINCE_NODE_UPDATE}));
    }

    if (!read_batch.states.empty()) {
        this->cycles_since_any_node_update = 0;
    }

    // この周期数だけ状態更新がなければCANを異常とみなす
    if (!read_batch.error_message.empty()) {
        this->set_state("can/health", util::to_interface_data(state::can::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to read CAN data: %s",
            read_batch.error_message.c_str());
    } else if (read_batch.states.empty()) {
        if (this->cycles_since_any_node_update < MAX_CYCLES_SINCE_ANY_NODE_UPDATE) {
            ++this->cycles_since_any_node_update;
        }
        if (this->cycles_since_any_node_update >= MAX_CYCLES_SINCE_ANY_NODE_UPDATE) {
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
