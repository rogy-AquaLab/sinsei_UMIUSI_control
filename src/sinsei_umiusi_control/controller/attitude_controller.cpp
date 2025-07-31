#include "sinsei_umiusi_control/controller/attitude_controller.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <controller_interface/controller_interface_base.hpp>
#include <cstddef>
#include <rclcpp/logging.hpp>
#include <string>

#include "sinsei_umiusi_control/cmd/thruster.hpp"
#include "sinsei_umiusi_control/controller/attitude_controller/controller_logic.hpp"
#include "sinsei_umiusi_control/controller/attitude_controller/data_conversion.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

using namespace sinsei_umiusi_control::controller;

namespace {

class LogicFeedForward : public attitude_controller::LogicInterface<
                             AttitudeController::Input, AttitudeController::Output> {
  public:
    auto control_mode() const -> attitude_controller::ControlMode override {
        return attitude_controller::ControlMode::FeedForward;
    }

    auto init(
        double /*time*/, const AttitudeController::Input & /*input*/,
        const AttitudeController::Output & output) -> AttitudeController::Output override {
        return output;
    }

    auto update(double /*time*/, double /*duration*/, const AttitudeController::Input & input)
        -> AttitudeController::Output override {
        // TODO: Implement feed-forward logic
        const auto target_orientation =
            attitude_controller::to_eigen_vector(input.cmd.target_orientation);
        const auto target_velocity =
            attitude_controller::to_eigen_vector(input.cmd.target_velocity);

        // θx, θy, θz, vx, vy, vz
        //  -> f1h, f1v, f2h, f2v, f3h, f3v, f4h, f4v (h: horizontal, v: vertical)
        // z軸まわりに半時計周りをfの番号順とhorizontalの正の方向とする

        // 変換前の行列
        const auto u = Eigen::Vector<double, 6>{
            target_orientation[0],  // 目標姿勢ベクトル
            target_orientation[1],  //
            target_orientation[2],  //
            target_velocity[0],     // 目標速度ベクトル
            target_velocity[1],     //
            target_velocity[2],     //
        };
        // 係数行列
        const auto a = Eigen::Matrix<double, 8, 6>{
            {0.0, 0.0, 1.0, -sqrt(2.0), sqrt(2.0), 0.0},   // スラスタ1 (:lf) 水平出力
            {1.0, -1.0, 0.0, 0.0, 0.0, 1.0},               // スラスタ1 (:lf) 垂直出力
            {0.0, 0.0, 1.0, -sqrt(2.0), -sqrt(2.0), 0.0},  // スラスタ2 (:lb) 水平出力
            {1.0, 1.0, 0.0, 0.0, 0.0, 1.0},                // スラスタ2 (:lb) 垂直出力
            {0.0, 0.0, 1.0, sqrt(2.0), -sqrt(2.0), 0.0},   // スラスタ3 (:rb) 水平出力
            {-1.0, 1.0, 0.0, 0.0, 0.0, 1.0},               // スラスタ3 (:rb) 垂直出力
            {0.0, 0.0, 1.0, sqrt(2.0), sqrt(2.0), 0.0},    // スラスタ4 (:rf) 水平出力
            {-1.0, -1.0, 0.0, 0.0, 0.0, 1.0},              // スラスタ4 (:rf) 垂直出力
        };
        const auto y = a * u;

        auto output = AttitudeController::Output{};
        constexpr auto ATAN_OR_ZERO = [](const double & x, const double & y) -> double {
            return (x == 0.0 && y == 0.0) ? 0.0 : std::atan(y / x);
        };
        output.cmd.thruster_angles = {
            sinsei_umiusi_control::cmd::thruster::Angle{ATAN_OR_ZERO(y[0], y[1])},
            sinsei_umiusi_control::cmd::thruster::Angle{ATAN_OR_ZERO(y[2], y[3])},
            sinsei_umiusi_control::cmd::thruster::Angle{ATAN_OR_ZERO(y[4], y[5])},
            sinsei_umiusi_control::cmd::thruster::Angle{ATAN_OR_ZERO(y[6], y[7])},
        };
        // Magnitude
        constexpr auto MGN = [](const double & x, const double & y) -> double {
            return std::sqrt(x * x + y * y);
        };
        // Sign (phi/2 < φ < 3π/2 -> negative)
        constexpr auto SGN = [](const double & x, const double & y) -> double {
            constexpr auto half_pi = boost::math::constants::half_pi<double>();
            const auto phi = std::atan2(y, x);
            return (phi > half_pi && phi < 3.0 * half_pi) ? -1.0 : 1.0;
        };
        const auto duty_sgns = std::array<double, 4>{
            SGN(y[0], y[1]), SGN(y[2], y[3]), SGN(y[4], y[5]), SGN(y[6], y[7])};
        const auto duty_abss = std::array<double, 4>{
            MGN(y[0], y[1]), MGN(y[2], y[3]), MGN(y[4], y[5]), MGN(y[6], y[7])};

        // 絶対値の最大値が1になるように正規化。ただし、0除算を避けるために最大値が0のときは0にする。
        // Norm
        constexpr auto NRM = [](const double & sgn, const double & abs,
                                const double & max_abs) -> double {
            return max_abs == 0.0 ? 0.0 : (sgn * abs / max_abs);
        };
        const auto max_duty = *std::max_element(duty_abss.begin(), duty_abss.end());
        output.cmd.thruster_duty_cycles = {
            sinsei_umiusi_control::cmd::thruster::DutyCycle{
                NRM(duty_sgns[0], duty_abss[0], max_duty)},
            sinsei_umiusi_control::cmd::thruster::DutyCycle{
                NRM(duty_sgns[1], duty_abss[1], max_duty)},
            sinsei_umiusi_control::cmd::thruster::DutyCycle{
                NRM(duty_sgns[2], duty_abss[2], max_duty)},
            sinsei_umiusi_control::cmd::thruster::DutyCycle{
                NRM(duty_sgns[3], duty_abss[3], max_duty)},
        };

        return output;
    }
};

}  // namespace

auto AttitudeController::command_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    auto cmd_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->command_interface_data) {
        cmd_names.push_back(name);
    }

    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL,
        cmd_names,
    };
}

auto AttitudeController::state_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    auto state_names = std::vector<std::string>{};
    for (const auto & [name, _] : this->state_interface_data) {
        state_names.push_back(name);
    }

    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto AttitudeController::on_init() -> controller_interface::CallbackReturn {
    this->get_node()->declare_parameter("thruster_mode", "unknown");
    this->get_node()->declare_parameter("control_mode", "ff");

    this->input = AttitudeController::Input{};
    this->output = AttitudeController::Output{};

    return controller_interface::CallbackReturn::SUCCESS;
}

auto AttitudeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    // スラスタモードを取得
    const auto thruster_mode_str = this->get_node()->get_parameter("thruster_mode").as_string();
    const auto thruster_mode_res = util::get_mode_from_str(thruster_mode_str);
    if (!thruster_mode_res) {
        RCLCPP_ERROR(
            this->get_node()->get_logger(), "Invalid thruster mode: %s", thruster_mode_str.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    this->thruster_mode = thruster_mode_res.value();
    RCLCPP_INFO(this->get_node()->get_logger(), "Thruster mode: %s", thruster_mode_str.c_str());

    // コントロールモードを取得
    const auto control_mode_str = this->get_node()->get_parameter("control_mode").as_string();
    const auto control_mode_res = attitude_controller::get_mode_from_str(control_mode_str);
    if (!control_mode_res) {
        RCLCPP_ERROR(
            this->get_node()->get_logger(), "Invalid control mode: %s", control_mode_str.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    switch (control_mode_res.value()) {
        case attitude_controller::ControlMode::FeedForward: {
            this->logic = std::make_unique<LogicFeedForward>();
            break;
        }
        case attitude_controller::ControlMode::FeedBack: {
            // TODO: Implement feedback logic
            RCLCPP_ERROR(
                this->get_node()->get_logger(), "Feedback control mode is not implemented yet");
            return controller_interface::CallbackReturn::ERROR;
            break;
        }
        default: {
            return controller_interface::CallbackReturn::ERROR;  // unreachable
        }
    }
    RCLCPP_INFO(this->get_node()->get_logger(), "Control mode: %s", control_mode_str.c_str());

    // Command / State Interfaceの設定
    constexpr std::string_view THRUSTER_SUFFIX[4] = {"_lf", "_lb", "_rb", "_rf"};

    for (size_t i = 0; i < 4; ++i) {
        const auto prefix = "thruster_controller" + std::string(THRUSTER_SUFFIX[i]) + "/";
        this->command_interface_data.emplace_back(
            prefix + "angle", util::to_interface_data_ptr(this->output.cmd.thruster_angles[i]));
        this->command_interface_data.emplace_back(
            prefix + "duty_cycle",
            util::to_interface_data_ptr(this->output.cmd.thruster_duty_cycles[i]));
    }

    if (this->thruster_mode == util::ThrusterMode::Can) {
        // `can`モードのときは、RPMを取得するためのインターフェースを追加する。
        for (size_t i = 0; i < 4; ++i) {
            const auto prefix =
                "thruster_controller" + std::string(THRUSTER_SUFFIX[i]) + "/thruster/";
            this->state_interface_data.emplace_back(
                prefix + "esc/rpm",
                util::to_interface_data_ptr(this->input.state.thruster_rpms[i]));
        }
    }
    this->state_interface_data.emplace_back(
        "imu/quaternion.x", util::to_interface_data_ptr(this->input.state.imu_quaternion.x));
    this->state_interface_data.emplace_back(
        "imu/quaternion.y", util::to_interface_data_ptr(this->input.state.imu_quaternion.y));
    this->state_interface_data.emplace_back(
        "imu/quaternion.z", util::to_interface_data_ptr(this->input.state.imu_quaternion.z));
    this->state_interface_data.emplace_back(
        "imu/quaternion.w", util::to_interface_data_ptr(this->input.state.imu_quaternion.w));
    this->state_interface_data.emplace_back(
        "imu/velocity.x", util::to_interface_data_ptr(this->input.state.imu_velocity.x));
    this->state_interface_data.emplace_back(
        "imu/velocity.y", util::to_interface_data_ptr(this->input.state.imu_velocity.y));
    this->state_interface_data.emplace_back(
        "imu/velocity.z", util::to_interface_data_ptr(this->input.state.imu_velocity.z));

    this->ref_interface_data.emplace_back(
        "target_orientation.x", util::to_interface_data_ptr(this->input.cmd.target_orientation.x));
    this->ref_interface_data.emplace_back(
        "target_orientation.y", util::to_interface_data_ptr(this->input.cmd.target_orientation.y));
    this->ref_interface_data.emplace_back(
        "target_orientation.z", util::to_interface_data_ptr(this->input.cmd.target_orientation.z));
    this->ref_interface_data.emplace_back(
        "target_velocity.x", util::to_interface_data_ptr(this->input.cmd.target_velocity.x));
    this->ref_interface_data.emplace_back(
        "target_velocity.y", util::to_interface_data_ptr(this->input.cmd.target_velocity.y));
    this->ref_interface_data.emplace_back(
        "target_velocity.z", util::to_interface_data_ptr(this->input.cmd.target_velocity.z));

    return controller_interface::CallbackReturn::SUCCESS;
}

auto AttitudeController::on_export_reference_interfaces()
    -> std::vector<hardware_interface::CommandInterface> {
    // To avoid bug in ros2 control. `reference_interfaces_` is actually not used.
    this->reference_interfaces_.resize(this->ref_interface_data.size());

    auto interfaces = std::vector<hardware_interface::CommandInterface>{};
    for (auto & [name, data] : this->ref_interface_data) {
        interfaces.emplace_back(
            hardware_interface::CommandInterface(this->get_node()->get_name(), name, data));
    }
    return interfaces;
}

auto AttitudeController::on_export_state_interfaces()
    -> std::vector<hardware_interface::StateInterface> {
    auto interfaces = std::vector<hardware_interface::StateInterface>{};
    for (auto & [name, data] : this->state_interface_data) {
        interfaces.emplace_back(
            hardware_interface::StateInterface(this->get_node()->get_name(), name, data));
    }
    return interfaces;
}

auto AttitudeController::on_set_chained_mode(bool /*chained_mode*/) -> bool { return true; };

auto AttitudeController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) -> controller_interface::return_type {
    return controller_interface::return_type::OK;
}

auto AttitudeController::update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) -> controller_interface::return_type {
    // 状態を取得
    auto res = util::interface_accessor::get_states_from_loaned_interfaces(
        this->state_interfaces_, this->state_interface_data);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), DURATION,
            "Failed to get value of state interfaces");
    }

    // コントロールモード(フィードフォワード/フィードバック)を取得
    const auto control_mode_str = this->get_node()->get_parameter("control_mode").as_string();
    const auto control_mode_res = attitude_controller::get_mode_from_str(control_mode_str);
    if (!control_mode_res) {
        RCLCPP_ERROR(
            this->get_node()->get_logger(), "Invalid control mode: %s", control_mode_str.c_str());
    }
    const auto mode_changed = this->logic->control_mode() != control_mode_res.value();

    if (!mode_changed) {
        // 姿勢制御の関数を呼び出す
        this->output = this->logic->update(time.seconds(), period.seconds(), this->input);
    } else {
        RCLCPP_INFO(
            this->get_node()->get_logger(), "Control mode changed: %s -> %s",
            attitude_controller::control_mode_to_str(this->logic->control_mode()).c_str(),
            attitude_controller::control_mode_to_str(control_mode_res.value()).c_str());

        // モードが変わった場合はロジックを変更して初期化
        switch (control_mode_res.value()) {
            case attitude_controller::ControlMode::FeedForward: {
                this->logic = std::make_unique<LogicFeedForward>();
                break;
            }
            case attitude_controller::ControlMode::FeedBack: {
                // TODO: Implement feedback logic
                RCLCPP_ERROR(
                    this->get_node()->get_logger(), "Feedback control mode is not implemented yet");
                return controller_interface::return_type::ERROR;
            }
            default: {
                return controller_interface::return_type::ERROR;  // unreachable
            }
        }
        this->output = this->logic->init(time.seconds(), this->input, this->output);
    }

    RCLCPP_INFO(  // FIXME: Debug用。消す。
        this->get_node()->get_logger(), "\nangle: [%f, %f, %f, %f]\nduty : [%f, %f, %f, %f]\n",
        this->output.cmd.thruster_angles[0].value, this->output.cmd.thruster_angles[1].value,
        this->output.cmd.thruster_angles[2].value, this->output.cmd.thruster_angles[3].value,
        this->output.cmd.thruster_duty_cycles[0].value,
        this->output.cmd.thruster_duty_cycles[1].value,
        this->output.cmd.thruster_duty_cycles[2].value,
        this->output.cmd.thruster_duty_cycles[3].value);

    // コマンドを送信
    res = util::interface_accessor::set_commands_to_loaned_interfaces(
        this->command_interfaces_, this->command_interface_data);
    if (!res) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_node()->get_logger(), *this->get_node()->get_clock(), DURATION,
            "Failed to set value for command interfaces");
    }

    return controller_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::controller::AttitudeController,
    controller_interface::ChainableControllerInterface)
