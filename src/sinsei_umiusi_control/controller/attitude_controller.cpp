#include "sinsei_umiusi_control/controller/attitude_controller.hpp"

#include <cmath>
#include <controller_interface/controller_interface_base.hpp>
#include <cstddef>
#include <rclcpp/logging.hpp>
#include <string>

#include "sinsei_umiusi_control/controller/logic/attitude/feed_forward.hpp"
#include "sinsei_umiusi_control/controller/logic/logic_interface.hpp"
#include "sinsei_umiusi_control/util/interface_accessor.hpp"
#include "sinsei_umiusi_control/util/serialization.hpp"

// RL logic は libtorch がある環境でだけ入る (CMakeLists の find_package(Torch QUIET))。
// 無ければ `control_mode:=rl` は on_configure で明示的に落とす。
#ifdef SINSEI_UMIUSI_CONTROL_WITH_TORCH
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include "sinsei_umiusi_control/controller/logic/attitude/rl.hpp"
#endif

using namespace sinsei_umiusi_control::controller;

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
    this->get_node()->declare_parameter("control_mode", "ff");

    // --- `control_mode:=rl` のときだけ使うパラメータ ---
    // 同梱バンドルの名前 (models/<name>/deploy.pt)。rl.model_path が空のときに使う
    this->get_node()->declare_parameter("rl.model_name", "av_mode13");
    // 同梱していないバンドルを使うときだけ、deploy.pt のフルパスを直接指定する
    this->get_node()->declare_parameter("rl.model_path", "");
    // 配備前検証に使う golden.pt。空なら model_path と同じディレクトリの golden.pt を探し、
    // それも無ければ検証をスキップする
    this->get_node()->declare_parameter("rl.golden_path", "");
    // duty の絶対値上限。力は上限の 2 乗で効くので 0.2 -> 0.4 は倍ではなく 4 倍
    // (autonomy known_issues A-17)
    this->get_node()->declare_parameter("rl.max_duty", 0.25);
    // 方策の servo 出力 ±1 が何度に当たるか。sim の servo_range_deg と揃える
    this->get_node()->declare_parameter("rl.servo_range_deg", 90.0);
    // 指令のレート制限。sim のプラントが持っていた値と揃える (known_issues A-11)
    this->get_node()->declare_parameter("rl.servo_slew_deg_per_s", 250.0);
    this->get_node()->declare_parameter("rl.thrust_slew_per_s", 4.0);
    // false で yaw の保持だけ切る (roll/pitch のみ保持)
    this->get_node()->declare_parameter("rl.hold_yaw", true);

    this->input = AttitudeController::Input{};
    this->output = AttitudeController::Output{};

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
    switch (control_mode_res.value()) {
        case logic::ControlMode::FeedForward: {
            this->logic = std::make_unique<logic::attitude::FeedForward>();
            break;
        }
        case logic::ControlMode::FeedBack: {
            // TODO: Implement feedback logic
            RCLCPP_ERROR(
                this->get_node()->get_logger(), "Feedback control mode is not implemented yet");
            return controller_interface::CallbackReturn::ERROR;
            break;
        }
        case logic::ControlMode::Rl: {
#ifdef SINSEI_UMIUSI_CONTROL_WITH_TORCH
            auto opt = logic::attitude::Rl::Options{};
            opt.model_path = this->get_node()->get_parameter("rl.model_path").as_string();
            opt.golden_path = this->get_node()->get_parameter("rl.golden_path").as_string();
            opt.max_duty = std::abs(this->get_node()->get_parameter("rl.max_duty").as_double());
            opt.servo_range_deg = this->get_node()->get_parameter("rl.servo_range_deg").as_double();
            opt.servo_slew_deg_per_s =
                this->get_node()->get_parameter("rl.servo_slew_deg_per_s").as_double();
            opt.thrust_slew_per_s =
                this->get_node()->get_parameter("rl.thrust_slew_per_s").as_double();
            opt.hold_yaw = this->get_node()->get_parameter("rl.hold_yaw").as_bool();
            // 学習時の control_rate_hz と照合する (合わなければ report に警告が出る)
            opt.control_hz = static_cast<double>(this->get_update_rate());
            if (opt.model_path.empty()) {
                // 既定は同梱バンドル。これで control 単体で rl が立ち上がる
                // (配備物の作り方と由来は models/README.md)
                const auto name = this->get_node()->get_parameter("rl.model_name").as_string();
                if (name.empty()) {
                    RCLCPP_ERROR(
                        this->get_node()->get_logger(),
                        "control_mode:=rl には rl.model_name か rl.model_path が要ります");
                    return controller_interface::CallbackReturn::ERROR;
                }
                const auto share = std::filesystem::path(
                    ament_index_cpp::get_package_share_directory("sinsei_umiusi_control"));
                opt.model_path = (share / "models" / name / "deploy.pt").string();
                if (!std::filesystem::exists(opt.model_path)) {
                    RCLCPP_ERROR(
                        this->get_node()->get_logger(),
                        "同梱バンドル '%s' がありません (%s)", name.c_str(),
                        opt.model_path.c_str());
                    return controller_interface::CallbackReturn::ERROR;
                }
            }
            if (opt.golden_path.empty()) {
                // 既定の置き場所を探す。見つからなければ空のまま (検証はスキップ)
                const auto beside =
                    std::filesystem::path(opt.model_path).parent_path() / "golden.pt";
                if (std::filesystem::exists(beside)) {
                    opt.golden_path = beside.string();
                }
            }
            try {
                // バンドルの読み込みと配備前検証はここで済ませる。update() は制御周期で
                // 回るので、数秒かかる読み込みを持ち込まない
                auto rl = std::make_unique<logic::attitude::Rl>(opt);
                RCLCPP_INFO(this->get_node()->get_logger(), "%s", rl->report().c_str());
                this->logic = std::move(rl);
            } catch (const std::exception & e) {
                RCLCPP_ERROR(
                    this->get_node()->get_logger(), "RL ポリシーを読み込めません: %s", e.what());
                return controller_interface::CallbackReturn::ERROR;
            }
            break;
#else
            RCLCPP_ERROR(
                this->get_node()->get_logger(),
                "control_mode:=rl は libtorch 無しでビルドされたため使えません "
                "(CMAKE_PREFIX_PATH に libtorch を足して再ビルドしてください)");
            return controller_interface::CallbackReturn::ERROR;
#endif
        }
        default: {
            return controller_interface::CallbackReturn::ERROR;  // unreachable
        }
    }
    RCLCPP_INFO(this->get_node()->get_logger(), "Control mode: %s", control_mode_str.c_str());

    // Command / State Interfaceの設定
    // 再 configure されうる (deactivate -> cleanup -> configure、`control_mode` を
    // `rl` に変えて入り直す導線がこれ)。push_back の前に空にしないと同じ名前が
    // 二重に並び、activate でインタフェースを二重 claim して失敗する
    this->command_interface_data.clear();
    this->state_interface_data.clear();
    this->ref_interface_data.clear();

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

        const auto thruster_prefix = controller_prefix + "thruster/";
        this->state_interface_data.push_back(std::make_tuple(
            thruster_prefix + "esc/rpm",
            util::to_interface_data_ptr(this->input.state.esc_rpms[i]),
            sizeof(this->input.state.esc_rpms[i])));
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
        "target_orientation.x", util::to_interface_data_ptr(this->input.cmd.target_orientation.x),
        sizeof(this->input.cmd.target_orientation.x)));
    this->ref_interface_data.push_back(std::make_tuple(
        "target_orientation.y", util::to_interface_data_ptr(this->input.cmd.target_orientation.y),
        sizeof(this->input.cmd.target_orientation.y)));
    this->ref_interface_data.push_back(std::make_tuple(
        "target_orientation.z", util::to_interface_data_ptr(this->input.cmd.target_orientation.z),
        sizeof(this->input.cmd.target_orientation.z)));
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

auto AttitudeController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    -> controller_interface::CallbackReturn {
    // activate のたびに logic の状態を初期化する。deactivate は disarm の経路なので、
    // 前回の値を抱えたまま再開すると最初の tick で disarm 前の指令が出る。
    // rl は prev_action / モード積分器 / レート制限の 3 つを持っていて、いずれも
    // 「指令を出していない間の値」を残すと実際とずれる (rl.hpp 冒頭の不変条件)。
    if (this->logic) {
        this->output = this->logic->init(
            this->get_node()->now().seconds(), this->input, this->output);
    }
    return controller_interface::CallbackReturn::SUCCESS;
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
    const auto control_mode_res = logic::get_mode_from_str(control_mode_str);
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
            logic::control_mode_to_str(this->logic->control_mode()).data(),
            logic::control_mode_to_str(control_mode_res.value()).data());

        // モードが変わった場合はロジックを変更して初期化
        switch (control_mode_res.value()) {
            case logic::ControlMode::FeedForward: {
                this->logic = std::make_unique<logic::attitude::FeedForward>();
                break;
            }
            case logic::ControlMode::FeedBack: {
                // TODO: Implement feedback logic
                RCLCPP_ERROR(
                    this->get_node()->get_logger(), "Feedback control mode is not implemented yet");
                return controller_interface::return_type::ERROR;
            }
            case logic::ControlMode::Rl: {
                // バンドルの読み込みと golden 検証に数秒かかる。制御周期の中でやると
                // その間スラスタへ指令が出ないので、rl へは configure でしか入れない
                RCLCPP_ERROR(
                    this->get_node()->get_logger(),
                    "rl への実行時切替は非対応です。control_mode:=rl で再 configure "
                    "してください");
                return controller_interface::return_type::ERROR;
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
