#include "sinsei_umiusi_control/hardware/imu.hpp"

#include <array>
#include <memory>
#include <string>

#include "sinsei_umiusi_control/hardware_model/impl/linux_i2c.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"
#include "sinsei_umiusi_control/util/params.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

using namespace sinsei_umiusi_control::hardware;

Imu::~Imu() {
    if (!this->model) {
        RCLCPP_ERROR(this->get_logger(), "Imu model is not initialized.");
        return;
    }

    auto res = this->model->on_destroy();
    if (!res) {
        RCLCPP_ERROR(
            this->get_logger(), "\n  Failed to destroy Imu model: %s", res.error().c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Imu model destroyed successfully.");
    }
}

auto Imu::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
    -> hardware_interface::CallbackReturn {
    this->hardware_interface::SensorInterface::on_init(params);

    const auto device_path =
        util::find_param(params.hardware_info.hardware_parameters, "i2c_device");
    if (!device_path) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter 'i2c_device' not found in hardware parameters.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 化けサンプルの判定。パラメータ名と既定値は autonomy の umiusi_common/imu_sanity.py と
    // 揃えてある (両スタックの run を突き合わせるため)。既定は検出のみで値は通す
    auto sanity_opt = util::ImuSanity::Options{};
    const auto & hw_params = params.hardware_info.hardware_parameters;
    if (const auto v = util::find_param(hw_params, "imu_max_gyro")) {
        sanity_opt.max_gyro = std::stod(*v);
    }
    if (const auto v = util::find_param(hw_params, "imu_max_step_deg")) {
        sanity_opt.max_step_deg = std::stod(*v);
    }
    if (const auto v = util::find_param(hw_params, "imu_quat_tol")) {
        sanity_opt.quat_tol = std::stod(*v);
    }
    if (const auto v = util::find_param(hw_params, "imu_stale_after")) {
        sanity_opt.stale_after = std::stoi(*v);
    }
    if (const auto v = util::find_param(hw_params, "imu_sanity_enforce")) {
        sanity_opt.enforce = (*v == "true" || *v == "True" || *v == "1");
    }
    this->sanity = util::ImuSanity{sanity_opt};

    this->model.emplace(
        std::make_unique<sinsei_umiusi_control::hardware_model::impl::LinuxI2c>(
            device_path.value()));

    auto res = this->model->on_init();
    if (!res) {
        RCLCPP_ERROR(this->get_logger(), "\n  Failed to initialize IMU: %s", res.error().c_str());
        // IMUの初期化に失敗した場合、モデルにnullを再代入する
        this->model.reset();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

auto Imu::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hardware_interface::return_type {
    if (!this->model) {
        this->set_state("imu/health", util::to_interface_data(state::imu::Health{false}));

        constexpr auto DURATION = 3000;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  IMU model is not initialized");
        return hardware_interface::return_type::OK;
    }

    const auto res = this->model->on_read();
    if (!res) {
        this->set_state("imu/health", util::to_interface_data(state::imu::Health{false}));

        constexpr auto DURATION = 3000;  // ms
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  Failed to read IMU data: %s",
            res.error().c_str());

        return hardware_interface::return_type::OK;
    }
    this->set_state("imu/health", util::to_interface_data(state::imu::Health{true}));

    const auto [quaternion, acceleration, angular_velocity, temperature] = res.value();

    // 化けサンプルを判定する。既定 (enforce=false) では検出をログに出すだけで値は通すので、
    // 生データを録った bag から閾値を決め直せる (known_issues A-1 の 2026-08-21 方針)。
    // 加速度は判定に使っていないので常に生値を出す
    const auto checked = this->sanity.update(
        {quaternion.w, quaternion.x, quaternion.y, quaternion.z},
        {angular_velocity.x, angular_velocity.y, angular_velocity.z});
    if (checked.reason != util::ImuSanity::Reason::None) {
        constexpr auto DURATION = 3000;  // ms
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), DURATION, "\n  %s",
            this->sanity.describe(checked).c_str());
    }
    // 棄却したときだけ直前の有効値に差し替える。検出しただけのときは生値を通す
    // (正規化した値を出すと bag から |q| の化けが見えなくなり、閾値を決め直せない)。
    // まだ 1 つも有効値が無いうちも生値 (捨てると初期化中に値が来なくなる)
    const auto held = checked.held && checked.sample;
    const auto q = held ? checked.sample->quat
                        : std::array<double, 4>{
                              quaternion.w, quaternion.x, quaternion.y, quaternion.z};
    const auto g = held ? checked.sample->gyro
                        : std::array<double, 3>{
                              angular_velocity.x, angular_velocity.y, angular_velocity.z};

    this->set_state("imu/quaternion.x", q[1]);
    this->set_state("imu/quaternion.y", q[2]);
    this->set_state("imu/quaternion.z", q[3]);
    this->set_state("imu/quaternion.w", q[0]);
    this->set_state("imu/acceleration.x", acceleration.x);
    this->set_state("imu/acceleration.y", acceleration.y);
    this->set_state("imu/acceleration.z", acceleration.z);
    this->set_state("imu/angular_velocity.x", g[0]);
    this->set_state("imu/angular_velocity.y", g[1]);
    this->set_state("imu/angular_velocity.z", g[2]);
    this->set_state("imu/temperature", util::to_interface_data(temperature));

    return hardware_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sinsei_umiusi_control::hardware::Imu, hardware_interface::SensorInterface)
