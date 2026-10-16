#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_STEPPING_MOTOR_DRIVER_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_STEPPING_MOTOR_DRIVER_MODEL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <variant>

#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"

// 直動機構2軸を動かすステッピングモータードライバ基板に対応
// ref: https://github.com/rogy-AquaLab/SteppingMotorDriver/blob/main/docs/can_protocol.md

namespace sinsei_umiusi_control::hardware_model::can {

class SteppingMotorDriverModel {
  public:
    static constexpr std::size_t AXIS_COUNT = 2;
    // 行程に対する割合の最大値（100.00 %）
    static constexpr uint16_t RATIO_SCALE = 10000;

    // すべて標準ID
    enum class PacketId : interface::CanFrame::Id {
        SetTargets = 0x310,
        SetTarget = 0x311,
        Calibrate = 0x312,
        Status = 0x318,
        MeasuredTravel = 0x319,
    };
    // ドライバ基板が受け付ける受信IDの範囲
    static constexpr interface::CanFrame::Id COMMAND_ID_MIN = 0x310;
    static constexpr interface::CanFrame::Id COMMAND_ID_MAX = 0x317;

    enum class AxisState : uint8_t {
        Uncalibrated = 0,
        ApproachingMinFast = 1,
        LeavingMin = 2,
        ApproachingMinSlow = 3,
        MeasuringTravel = 4,
        Ready = 5,
        Fault = 6,
    };

    struct AxisStatus {
        // 行程に対する割合（0.0〜1.0）。不明なときはNaN
        double position;
        uint8_t state;
        uint8_t fault_reason;
        uint8_t flags;
    };

    struct PacketStatus {
        static constexpr PacketId ID = PacketId::Status;

        std::array<AxisStatus, AXIS_COUNT> axes;
    };

    struct PacketMeasuredTravel {
        static constexpr PacketId ID = PacketId::MeasuredTravel;

        uint8_t axis;
        int32_t steps;
    };

    // std::monostateは、ドライバ基板宛てのコマンドなど、返す値が無いフレーム
    using AnyPacket = std::variant<std::monostate, PacketStatus, PacketMeasuredTravel>;

    // ドライバ基板のフレームでなければstd::nulloptを返す
    static auto decode(const interface::CanFrame & frame)
        -> tl::expected<std::optional<AnyPacket>, std::string>;

    // 1軸の目標（0.0〜1.0）を送るフレームを作る
    static auto make_target_frame(std::size_t axis, double ratio)
        -> tl::expected<interface::CanFrame, std::string>;

    // ゼロ点を取り直すフレームを作る。bit0 = M1、bit1 = M2
    static auto make_calibrate_frame(uint8_t axis_mask)
        -> tl::expected<interface::CanFrame, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::can

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_STEPPING_MOTOR_DRIVER_MODEL_HPP
