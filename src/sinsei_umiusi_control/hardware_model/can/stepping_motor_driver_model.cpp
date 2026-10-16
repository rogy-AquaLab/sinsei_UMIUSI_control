#include "sinsei_umiusi_control/hardware_model/can/stepping_motor_driver_model.hpp"

#include <cmath>
#include <limits>
#include <string>

#include "sinsei_umiusi_control/util/byte.hpp"

using namespace sinsei_umiusi_control::hardware_model;

namespace {

using Model = can::SteppingMotorDriverModel;

auto validate_frame_length(const interface::CanFrame & frame, std::size_t min_length)
    -> tl::expected<void, std::string> {
    if (frame.len < min_length) {
        return tl::make_unexpected(
            "Stepping motor driver frame " + std::to_string(frame.id) + " is too short: expected " +
            std::to_string(min_length) + " bytes, got " + std::to_string(frame.len));
    }
    return {};
}

auto decode_axis_status(const interface::CanFrame::Data & data, std::size_t axis)
    -> Model::AxisStatus {
    const auto raw_position = sinsei_umiusi_control::util::to_integer_be<int16_t>(data, axis * 2);
    const auto state_byte = std::to_integer<uint8_t>(data[4 + axis]);

    // 準備完了で行程が設定済みのときだけ0〜10000になり、それ以外は-1
    const auto position = raw_position && raw_position.value() >= 0
                              ? static_cast<double>(raw_position.value()) / Model::RATIO_SCALE
                              : std::numeric_limits<double>::quiet_NaN();
    return Model::AxisStatus{
        position,
        static_cast<uint8_t>(state_byte & 0x0F),
        static_cast<uint8_t>(state_byte >> 4),
        std::to_integer<uint8_t>(data[6 + axis]),
    };
}

}  // namespace

auto can::SteppingMotorDriverModel::decode(const interface::CanFrame & frame)
    -> tl::expected<std::optional<AnyPacket>, std::string> {
    if (frame.is_extended) {
        return std::nullopt;
    }

    if (frame.id >= COMMAND_ID_MIN && frame.id <= COMMAND_ID_MAX) {
        // ほかのノードがドライバ基板へ送ったコマンド
        return std::optional<AnyPacket>{std::monostate{}};
    }

    switch (static_cast<PacketId>(frame.id)) {
        case PacketStatus::ID: {
            const auto validation_res = validate_frame_length(frame, 8);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
            }
            auto packet = PacketStatus{};
            for (std::size_t axis = 0; axis < AXIS_COUNT; ++axis) {
                packet.axes[axis] = decode_axis_status(frame.data, axis);
            }
            return std::optional<AnyPacket>{packet};
        }
        case PacketMeasuredTravel::ID: {
            const auto validation_res = validate_frame_length(frame, 5);
            if (!validation_res) {
                return tl::make_unexpected(validation_res.error());
            }
            return std::optional<AnyPacket>{PacketMeasuredTravel{
                std::to_integer<uint8_t>(frame.data[0]),
                util::to_integer_be<int32_t>(frame.data, 1).value(),
            }};
        }
        default:
            return std::nullopt;
    }
}

auto can::SteppingMotorDriverModel::make_target_frame(std::size_t axis, double ratio)
    -> tl::expected<interface::CanFrame, std::string> {
    if (axis >= AXIS_COUNT) {
        return tl::make_unexpected("Invalid stepping motor driver axis: " + std::to_string(axis));
    }
    if (!std::isfinite(ratio) || ratio < 0.0 || ratio > 1.0) {
        return tl::make_unexpected(
            "Stepping motor driver target must be in [0, 1]: " + std::to_string(ratio));
    }

    const auto target = static_cast<uint16_t>(std::lround(ratio * RATIO_SCALE));
    const auto target_bytes = util::to_bytes_be(target);

    auto frame = interface::CanFrame{};
    frame.id = static_cast<interface::CanFrame::Id>(PacketId::SetTarget);
    frame.len = 3;
    frame.data[0] = std::byte{static_cast<uint8_t>(axis)};
    frame.data[1] = target_bytes[0];
    frame.data[2] = target_bytes[1];
    frame.is_extended = false;
    return frame;
}

auto can::SteppingMotorDriverModel::make_calibrate_frame(uint8_t axis_mask)
    -> tl::expected<interface::CanFrame, std::string> {
    constexpr uint8_t ALL_AXES = (1U << AXIS_COUNT) - 1;
    if (axis_mask == 0 || (axis_mask & ~ALL_AXES) != 0) {
        return tl::make_unexpected(
            "Invalid stepping motor driver axis mask: " + std::to_string(axis_mask));
    }

    auto frame = interface::CanFrame{};
    frame.id = static_cast<interface::CanFrame::Id>(PacketId::Calibrate);
    frame.len = 1;
    frame.data[0] = std::byte{axis_mask};
    frame.is_extended = false;
    return frame;
}
