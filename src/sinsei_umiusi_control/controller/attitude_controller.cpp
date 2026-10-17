#include "sinsei_umiusi_control/controller/attitude_controller.hpp"

#include <algorithm>
#include <array>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <controller_interface/controller_interface_base.hpp>
#include <cstddef>
#include <limits>
#include <optional>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>
#include <string>

#include "sinsei_umiusi_control/controller/logic/attitude/feed_back.hpp"
#include "sinsei_umiusi_control/controller/logic/attitude/feed_forward.hpp"
#include "sinsei_umiusi_control/controller/logic/logic_interface.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

using namespace sinsei_umiusi_control::controller;

namespace {

using rcl_interfaces::msg::FloatingPointRange;
using rcl_interfaces::msg::ParameterDescriptor;
using sinsei_umiusi_control::controller::logic::attitude::AttitudeFeedbackGains;
using sinsei_umiusi_control::controller::logic::attitude::FeedForwardGains;
using sinsei_umiusi_control::controller::logic::attitude::MixerParameters;

constexpr auto DEFAULT_SERVO_DIRECTION_DEADBAND_DEG = 5.0;
constexpr auto DEFAULT_SERVO_REVERSAL_DEADBAND_DEG = 10.0;
constexpr auto DEFAULT_SERVO_RETARGET_THRUST_ENTER = 0.10;
constexpr auto DEFAULT_SERVO_RETARGET_THRUST_EXIT = 0.06;
constexpr auto DEFAULT_ESC_THRUST_LIMIT = 1.0;
constexpr auto MAX_SERVO_DEADBAND_DEG = 90.0;

auto nonnegative_gain_descriptor(const std::string & description) -> ParameterDescriptor {
    return ParameterDescriptor{}
        .set__description(description)
        .set__type(rclcpp::PARAMETER_DOUBLE)
        .set__floating_point_range({FloatingPointRange{}.set__from_value(0.0).set__to_value(
            std::numeric_limits<double>::max())});
}

auto servo_deadband_descriptor(const std::string & description) -> ParameterDescriptor {
    return ParameterDescriptor{}
        .set__description(description)
        .set__type(rclcpp::PARAMETER_DOUBLE)
        .set__floating_point_range({FloatingPointRange{}
                                        .set__from_value(0.0)
                                        .set__to_value(MAX_SERVO_DEADBAND_DEG)});
}

auto normalized_thrust_descriptor(const std::string & description) -> ParameterDescriptor {
    return ParameterDescriptor{}
        .set__description(description)
        .set__type(rclcpp::PARAMETER_DOUBLE)
        .set__floating_point_range(
            {FloatingPointRange{}.set__from_value(0.0).set__to_value(1.0)});
}

auto read_mixer_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    -> std::optional<MixerParameters> {
    const auto direction_deadband_deg =
        node->get_parameter("mixer.servo_direction_deadband_deg").as_double();
    const auto reversal_deadband_deg =
        node->get_parameter("mixer.servo_reversal_deadband_deg").as_double();
    const auto retarget_thrust_enter =
        node->get_parameter("mixer.servo_retarget_thrust_enter").as_double();
    const auto retarget_thrust_exit =
        node->get_parameter("mixer.servo_retarget_thrust_exit").as_double();
    const auto esc_thrust_limit = node->get_parameter("mixer.esc_thrust_limit").as_double();
    const auto deadbands = std::array<double, 2>{direction_deadband_deg, reversal_deadband_deg};
    if (!std::all_of(deadbands.begin(), deadbands.end(), [](double value) {
            return std::isfinite(value) && value >= 0.0 && value <= MAX_SERVO_DEADBAND_DEG;
        })) {
        RCLCPP_ERROR(node->get_logger(), "Servo deadbands must be finite and between 0 and 90 deg");
        return std::nullopt;
    }
    if (!std::isfinite(retarget_thrust_enter) || retarget_thrust_enter < 0.0 ||
        retarget_thrust_enter > 1.0 || !std::isfinite(retarget_thrust_exit) ||
        retarget_thrust_exit < 0.0 || retarget_thrust_exit > retarget_thrust_enter) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Servo retarget thrust thresholds must satisfy 0 <= exit <= enter <= 1");
        return std::nullopt;
    }
    if (!std::isfinite(esc_thrust_limit) || esc_thrust_limit <= 0.0 || esc_thrust_limit > 1.0) {
        RCLCPP_ERROR(node->get_logger(), "ESC thrust limit must satisfy 0 < limit <= 1");
        return std::nullopt;
    }

    constexpr auto DEG_TO_RAD = boost::math::constants::pi<double>() / 180.0;
    return MixerParameters{
        direction_deadband_deg * DEG_TO_RAD,
        reversal_deadband_deg * DEG_TO_RAD,
        retarget_thrust_enter,
        retarget_thrust_exit,
        esc_thrust_limit,
    };
}

auto read_feedback_gains(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    -> std::optional<AttitudeFeedbackGains> {
    auto gains = AttitudeFeedbackGains{
        node->get_parameter("feedback.kp_roll").as_double(),
        node->get_parameter("feedback.kp_pitch").as_double(),
        node->get_parameter("feedback.kd_roll").as_double(),
        node->get_parameter("feedback.kd_pitch").as_double(),
        node->get_parameter("feedback.kp_yaw_rate").as_double(),
        node->get_parameter("feedback.ki_roll").as_double(),
        node->get_parameter("feedback.ki_pitch").as_double(),
        node->get_parameter("feedback.i_max").as_double(),
    };
    const auto values = std::array<double, 8>{
        gains.kp_roll,     gains.kp_pitch, gains.kd_roll,  gains.kd_pitch,
        gains.kp_yaw_rate, gains.ki_roll,  gains.ki_pitch, gains.i_max,
    };
    if (!std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value) && value >= 0.0;
        })) {
        RCLCPP_ERROR(node->get_logger(), "Feedback gains must be finite and non-negative");
        return std::nullopt;
    }
    return gains;
}

auto read_feed_forward_gains(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
    -> std::optional<FeedForwardGains> {
    auto gains = FeedForwardGains{
        node->get_parameter("feed_forward.k_attitude").as_double(),
        node->get_parameter("feed_forward.k_yaw_rate").as_double(),
    };
    const auto values = std::array<double, 2>{gains.k_attitude, gains.k_yaw_rate};
    if (!std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value) && value >= 0.0;
        })) {
        RCLCPP_ERROR(node->get_logger(), "Feed-forward gains must be finite and non-negative");
        return std::nullopt;
    }
    return gains;
}

}  // namespace

auto AttitudeController::command_interface_configuration() const
    -> controller_interface::InterfaceConfiguration {
    auto cmd_names = std::vector<std::string>{};
    for (const auto & [name, _data, _size] : this->command_interface_data) {
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
    for (const auto & [name, _data, _size] : this->state_interface_data) {
        state_names.push_back(name);
    }

    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::INDIVIDUAL,
        state_names,
    };
}

auto AttitudeController::on_init() -> controller_interface::CallbackReturn {
    this->get_node()->declare_parameter("control_mode", "fb");
    this->get_node()->declare_parameter(
        "feed_forward.k_attitude", 2.0,
        nonnegative_gain_descriptor("Open-loop roll/pitch attitude gain"));
    this->get_node()->declare_parameter(
        "feed_forward.k_yaw_rate", 0.2, nonnegative_gain_descriptor("Open-loop yaw-rate gain"));
    this->get_node()->declare_parameter(
        "feedback.kp_roll", 1.0, nonnegative_gain_descriptor("Roll proportional gain"));
    this->get_node()->declare_parameter(
        "feedback.kp_pitch", 1.0, nonnegative_gain_descriptor("Pitch proportional gain"));
    this->get_node()->declare_parameter(
        "feedback.kd_roll", 0.35, nonnegative_gain_descriptor("Roll derivative gain"));
    this->get_node()->declare_parameter(
        "feedback.kd_pitch", 0.35, nonnegative_gain_descriptor("Pitch derivative gain"));
    this->get_node()->declare_parameter(
        "feedback.kp_yaw_rate", 1.0, nonnegative_gain_descriptor("Yaw-rate proportional gain"));
    this->get_node()->declare_parameter(
        "feedback.ki_roll", 0.0, nonnegative_gain_descriptor("Roll integral gain"));
    this->get_node()->declare_parameter(
        "feedback.ki_pitch", 0.0, nonnegative_gain_descriptor("Pitch integral gain"));
    this->get_node()->declare_parameter(
        "feedback.i_max", 0.2, nonnegative_gain_descriptor("Roll/pitch integral error clamp"));
    this->get_node()->declare_parameter(
        "mixer.servo_direction_deadband_deg", DEFAULT_SERVO_DIRECTION_DEADBAND_DEG,
        servo_deadband_descriptor("Servo direction deadband [deg]"));
    this->get_node()->declare_parameter(
        "mixer.servo_reversal_deadband_deg", DEFAULT_SERVO_REVERSAL_DEADBAND_DEG,
        servo_deadband_descriptor("Servo end-stop reversal deadband [deg]"));
    this->get_node()->declare_parameter(
        "mixer.servo_retarget_thrust_enter", DEFAULT_SERVO_RETARGET_THRUST_ENTER,
        normalized_thrust_descriptor("Normalized thrust to start servo direction tracking"));
    this->get_node()->declare_parameter(
        "mixer.servo_retarget_thrust_exit", DEFAULT_SERVO_RETARGET_THRUST_EXIT,
        normalized_thrust_descriptor("Normalized thrust to stop servo direction tracking"));
    this->get_node()->declare_parameter(
        "mixer.esc_thrust_limit", DEFAULT_ESC_THRUST_LIMIT,
        normalized_thrust_descriptor(
            "Normalized ESC thrust limit (match max_duty / duty_per_thrust)"));

    this->input = AttitudeController::Input{};
    this->input.cmd.target_attitude.w = 1.0;
    this->output = AttitudeController::Output{};
    this->servo_estimated_angle_interfaces.fill(
        state::thruster::servo::EstimatedAngle{std::numeric_limits<double>::quiet_NaN()});

    return controller_interface::CallbackReturn::SUCCESS;
}

auto AttitudeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    // コントロールモードを取得
    const auto control_mode_str = this->get_node()->get_parameter("control_mode").as_string();
    const auto control_mode_res = logic::get_mode_from_str(control_mode_str);
    if (!control_mode_res) {
        RCLCPP_ERROR(
            this->get_node()->get_logger(), "Invalid control mode: %s", control_mode_str.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    const auto mixer_parameters = read_mixer_parameters(this->get_node());
    if (!mixer_parameters) {
        return controller_interface::CallbackReturn::ERROR;
    }
    switch (control_mode_res.value()) {
        case logic::ControlMode::FeedForward: {
            const auto gains = read_feed_forward_gains(this->get_node());
            if (!gains) {
                return controller_interface::CallbackReturn::ERROR;
            }
            this->logic =
                std::make_unique<logic::attitude::FeedForward>(*gains, *mixer_parameters);
            break;
        }
        case logic::ControlMode::FeedBack: {
            const auto gains = read_feedback_gains(this->get_node());
            if (!gains) {
                return controller_interface::CallbackReturn::ERROR;
            }
            this->logic = std::make_unique<logic::attitude::FeedBack>(*gains, *mixer_parameters);
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
        const auto controller_prefix =
            "thruster_controller" + std::string(THRUSTER_SUFFIX[i]) + "/";
        this->command_interface_data.push_back(std::make_tuple(
            controller_prefix + "esc/duty_cycle",
            util::to_interface_data_ptr(this->output.cmd.esc_thrusts[i]),
            sizeof(this->output.cmd.esc_thrusts[i])));
        this->command_interface_data.push_back(std::make_tuple(
            controller_prefix + "servo/angle",
            util::to_interface_data_ptr(this->output.cmd.servo_angles[i]),
            sizeof(this->output.cmd.servo_angles[i])));

        // arm 状態。thruster_controller が自分の名前で出している state interface で、
        // gate_controller も同じものを読んでいる
        this->state_interface_data.push_back(std::make_tuple(
            controller_prefix + "esc/mode",
            util::to_interface_data_ptr(this->input.state.esc_modes[i]),
            sizeof(this->input.state.esc_modes[i])));

        const auto thruster_prefix = controller_prefix + "thruster/";
        this->state_interface_data.push_back(std::make_tuple(
            thruster_prefix + "esc/rpm", util::to_interface_data_ptr(this->input.state.esc_rpms[i]),
            sizeof(this->input.state.esc_rpms[i])));
        this->state_interface_data.push_back(std::make_tuple(
            controller_prefix + "servo/commanded_angle",
            util::to_interface_data_ptr(this->input.state.servo_commanded_angles[i]),
            sizeof(this->input.state.servo_commanded_angles[i])));
        this->state_interface_data.push_back(std::make_tuple(
            controller_prefix + "servo/estimated_angle",
            util::to_interface_data_ptr(this->servo_estimated_angle_interfaces[i]),
            sizeof(this->servo_estimated_angle_interfaces[i])));
        this->state_interface_data.push_back(std::make_tuple(
            controller_prefix + "servo/max_angular_velocity",
            util::to_interface_data_ptr(this->input.state.servo_max_angular_velocities[i]),
            sizeof(this->input.state.servo_max_angular_velocities[i])));
    }
    this->state_interface_data.push_back(std::make_tuple(
        "imu/quaternion.x", util::to_interface_data_ptr(this->input.state.imu_quaternion.x),
        sizeof(this->input.state.imu_quaternion.x)));
    this->state_interface_data.push_back(std::make_tuple(
        "imu/quaternion.y", util::to_interface_data_ptr(this->input.state.imu_quaternion.y),
        sizeof(this->input.state.imu_quaternion.y)));
    this->state_interface_data.push_back(std::make_tuple(
        "imu/quaternion.z", util::to_interface_data_ptr(this->input.state.imu_quaternion.z),
        sizeof(this->input.state.imu_quaternion.z)));
    this->state_interface_data.push_back(std::make_tuple(
        "imu/quaternion.w", util::to_interface_data_ptr(this->input.state.imu_quaternion.w),
        sizeof(this->input.state.imu_quaternion.w)));
    this->state_interface_data.push_back(std::make_tuple(
        "imu/acceleration.x", util::to_interface_data_ptr(this->input.state.imu_acceleration.x),
        sizeof(this->input.state.imu_acceleration.x)));
    this->state_interface_data.push_back(std::make_tuple(
        "imu/acceleration.y", util::to_interface_data_ptr(this->input.state.imu_acceleration.y),
        sizeof(this->input.state.imu_acceleration.y)));
    this->state_interface_data.push_back(std::make_tuple(
        "imu/acceleration.z", util::to_interface_data_ptr(this->input.state.imu_acceleration.z),
        sizeof(this->input.state.imu_acceleration.z)));
    this->state_interface_data.push_back(std::make_tuple(
        "imu/angular_velocity.x",
        util::to_interface_data_ptr(this->input.state.imu_angular_velocity.x),
        sizeof(this->input.state.imu_angular_velocity.x)));
    this->state_interface_data.push_back(std::make_tuple(
        "imu/angular_velocity.y",
        util::to_interface_data_ptr(this->input.state.imu_angular_velocity.y),
        sizeof(this->input.state.imu_angular_velocity.y)));
    this->state_interface_data.push_back(std::make_tuple(
        "imu/angular_velocity.z",
        util::to_interface_data_ptr(this->input.state.imu_angular_velocity.z),
        sizeof(this->input.state.imu_angular_velocity.z)));

    this->ref_interface_data.push_back(std::make_tuple(
        "target_attitude.x", util::to_interface_data_ptr(this->input.cmd.target_attitude.x),
        sizeof(this->input.cmd.target_attitude.x)));
    this->ref_interface_data.push_back(std::make_tuple(
        "target_attitude.y", util::to_interface_data_ptr(this->input.cmd.target_attitude.y),
        sizeof(this->input.cmd.target_attitude.y)));
    this->ref_interface_data.push_back(std::make_tuple(
        "target_attitude.z", util::to_interface_data_ptr(this->input.cmd.target_attitude.z),
        sizeof(this->input.cmd.target_attitude.z)));
    this->ref_interface_data.push_back(std::make_tuple(
        "target_attitude.w", util::to_interface_data_ptr(this->input.cmd.target_attitude.w),
        sizeof(this->input.cmd.target_attitude.w)));
    this->ref_interface_data.push_back(std::make_tuple(
        "target_attitude.yaw_rate",
        util::to_interface_data_ptr(this->input.cmd.target_attitude.yaw_rate),
        sizeof(this->input.cmd.target_attitude.yaw_rate)));
    this->ref_interface_data.push_back(std::make_tuple(
        "target_attitude.hold_yaw",
        util::to_interface_data_ptr(this->input.cmd.target_attitude.hold_yaw),
        sizeof(this->input.cmd.target_attitude.hold_yaw)));
    this->ref_interface_data.push_back(std::make_tuple(
        "target_velocity.x", util::to_interface_data_ptr(this->input.cmd.target_velocity.x),
        sizeof(this->input.cmd.target_velocity.x)));
    this->ref_interface_data.push_back(std::make_tuple(
        "target_velocity.y", util::to_interface_data_ptr(this->input.cmd.target_velocity.y),
        sizeof(this->input.cmd.target_velocity.y)));
    this->ref_interface_data.push_back(std::make_tuple(
        "target_velocity.z", util::to_interface_data_ptr(this->input.cmd.target_velocity.z),
        sizeof(this->input.cmd.target_velocity.z)));

    return controller_interface::CallbackReturn::SUCCESS;
}

auto AttitudeController::on_export_reference_interfaces()
    -> std::vector<hardware_interface::CommandInterface> {
    // To avoid bug in ros2 control. `reference_interfaces_` is actually not used.
    this->reference_interfaces_.resize(this->ref_interface_data.size());

    auto interfaces = std::vector<hardware_interface::CommandInterface>{};
    for (auto & [name, data, _] : this->ref_interface_data) {
        interfaces.emplace_back(
            hardware_interface::CommandInterface(this->get_node()->get_name(), name, data));
    }
    return interfaces;
}

auto AttitudeController::on_export_state_interfaces()
    -> std::vector<hardware_interface::StateInterface> {
    auto interfaces = std::vector<hardware_interface::StateInterface>{};
    for (auto & [name, data, _] : this->state_interface_data) {
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
    for (size_t i = 0; i < this->servo_estimated_angle_interfaces.size(); ++i) {
        const auto estimated_angle = this->servo_estimated_angle_interfaces[i];
        if (std::isfinite(estimated_angle.value)) {
            this->input.state.servo_estimated_angles[i] = estimated_angle;
        } else {
            this->input.state.servo_estimated_angles[i] = std::nullopt;
        }
    }

    // コントロールモード(フィードフォワード/フィードバック)を取得
    const auto control_mode_str = this->get_node()->get_parameter("control_mode").as_string();
    const auto control_mode_res = logic::get_mode_from_str(control_mode_str);
    if (!control_mode_res) {
        RCLCPP_ERROR(
            this->get_node()->get_logger(), "Invalid control mode: %s", control_mode_str.c_str());
    }
    const auto mode_changed = this->logic->control_mode() != control_mode_res.value();

    // disarm 中は logic を走らせず初期状態に戻し続ける。on_activate だけでは足りない:
    // 実運用の arm/disarm は `/cmd/thruster_runnable_all` (ThrusterMode) で行われ、
    // attitude_controller は active のままなので deactivate を通らない。
    // rl はレンチモードの積分器を持っていて、disarm 中も回し続けると誤差が閉じないまま
    // ±1 のレールに張り付き、arm した瞬間に max_duty のキックが出る (実機で確認)。
    const auto armed = std::any_of(
        this->input.state.esc_modes.begin(), this->input.state.esc_modes.end(),
        [](const auto & m) { return m.value == util::ThrusterMode::Runnable; });

    if (!mode_changed && !armed) {
        this->output = this->logic->init(time.seconds(), this->input, this->output);
    } else if (!mode_changed) {
        // 姿勢制御の関数を呼び出す
        this->output = this->logic->update(time.seconds(), period.seconds(), this->input);
    } else {
        RCLCPP_INFO(
            this->get_node()->get_logger(), "Control mode changed: %s -> %s",
            logic::control_mode_to_str(this->logic->control_mode()).data(),
            logic::control_mode_to_str(control_mode_res.value()).data());

        // モードが変わった場合はロジックを変更して初期化
        const auto mixer_parameters = read_mixer_parameters(this->get_node());
        if (!mixer_parameters) {
            return controller_interface::return_type::ERROR;
        }
        switch (control_mode_res.value()) {
            case logic::ControlMode::FeedForward: {
                const auto gains = read_feed_forward_gains(this->get_node());
                if (!gains) {
                    return controller_interface::return_type::ERROR;
                }
                this->logic =
                    std::make_unique<logic::attitude::FeedForward>(*gains, *mixer_parameters);
                break;
            }
            case logic::ControlMode::FeedBack: {
                const auto gains = read_feedback_gains(this->get_node());
                if (!gains) {
                    return controller_interface::return_type::ERROR;
                }
                this->logic =
                    std::make_unique<logic::attitude::FeedBack>(*gains, *mixer_parameters);
                break;
            }
            default: {
                return controller_interface::return_type::ERROR;  // unreachable
            }
        }
        this->output = this->logic->init(time.seconds(), this->input, this->output);
    }

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
