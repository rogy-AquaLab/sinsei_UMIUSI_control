#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_VESC_PROTOCOL_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_VESC_PROTOCOL_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"
#include "sinsei_umiusi_control/util/byte.hpp"

// VESCのCANプロトコルのうち、VESC (モーター制御) とVESC BMSで共通の部分

namespace sinsei_umiusi_control::hardware_model::can::vesc_protocol {

// VESCのCANプロトコルのパケットID
// 名前と値は本家のファームウェアと同じ。説明は以下のドキュメントとソースを元にしている
// ref: https://github.com/vedderb/bldc/blob/4fd8279ea45a17c0d69357438ae2f7237a32514f/datatypes.h#L1156-L1227
// ref: https://github.com/vedderb/bldc/blob/4fd8279ea45a17c0d69357438ae2f7237a32514f/documentation/comm_can.md
// ref: https://github.com/vedderb/bldc/blob/4fd8279ea45a17c0d69357438ae2f7237a32514f/comm/comm_can.c
// ref: https://github.com/vedderb/bldc/blob/4fd8279ea45a17c0d69357438ae2f7237a32514f/bms.c
// CANのIDには8ビットで格納されるため、`uint8_t`とする
enum class PacketId : uint8_t {
    // デューティ比を指令する (-1.0〜1.0)
    CAN_PACKET_SET_DUTY = 0,
    // モーター電流を指令する [A]
    CAN_PACKET_SET_CURRENT = 1,
    // ブレーキ電流を指令する [A]
    CAN_PACKET_SET_CURRENT_BRAKE = 2,
    // 回転数を指令する [ERPM]
    CAN_PACKET_SET_RPM = 3,
    // 位置を指令する [deg]
    CAN_PACKET_SET_POS = 4,
    // 分割転送: 受信バッファにデータを書き込む (オフセット1バイト)
    CAN_PACKET_FILL_RX_BUFFER = 5,
    // 分割転送: 受信バッファにデータを書き込む (オフセット2バイト)
    CAN_PACKET_FILL_RX_BUFFER_LONG = 6,
    // 分割転送: 受信バッファのデータをCRCで確認して処理させる
    CAN_PACKET_PROCESS_RX_BUFFER = 7,
    // 1フレームに収まる短いデータを処理させる
    CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
    // ステータス: ERPM、電流、デューティ比
    CAN_PACKET_STATUS = 9,
    // 最大電流に対する割合でモーター電流を指令する (-1.0〜1.0)
    CAN_PACKET_SET_CURRENT_REL = 10,
    // 最大電流に対する割合でブレーキ電流を指令する (-1.0〜1.0)
    CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
    // ハンドブレーキ電流を指令する [A]
    CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
    // 最大電流に対する割合でハンドブレーキ電流を指令する (-1.0〜1.0)
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13,
    // ステータス: 消費した電荷量と回生した電荷量 [Ah]
    CAN_PACKET_STATUS_2 = 14,
    // ステータス: 消費した電力量と回生した電力量 [Wh]
    CAN_PACKET_STATUS_3 = 15,
    // ステータス: FET温度、モーター温度、入力電流、PID位置
    CAN_PACKET_STATUS_4 = 16,
    // 応答 (PONG) を要求する
    CAN_PACKET_PING = 17,
    // PINGへの応答 (応答元のIDとハードウェアの種類)
    CAN_PACKET_PONG = 18,
    // FOCのパラメータの検出と適用を実行させる
    CAN_PACKET_DETECT_APPLY_ALL_FOC = 19,
    // FOCのパラメータの検出と適用の結果
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES = 20,
    // モーター電流の上下限を設定する (保存しない)
    CAN_PACKET_CONF_CURRENT_LIMITS = 21,
    // モーター電流の上下限を設定し、保存する
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS = 22,
    // 入力電流の上下限を設定する (保存しない)
    CAN_PACKET_CONF_CURRENT_LIMITS_IN = 23,
    // 入力電流の上下限を設定し、保存する
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN = 24,
    // FOCのオープンループ回転数とセンサーレスに切り替える回転数を設定する (保存しない)
    CAN_PACKET_CONF_FOC_ERPMS = 25,
    // FOCのオープンループ回転数とセンサーレスに切り替える回転数を設定し、保存する
    CAN_PACKET_CONF_STORE_FOC_ERPMS = 26,
    // ステータス: タコメーター、入力電圧
    CAN_PACKET_STATUS_5 = 27,
    // TS5700N8501エンコーダーのステータスを要求する
    CAN_PACKET_POLL_TS5700N8501_STATUS = 28,
    // 電流の制限を始める・終える電池電圧を設定する (保存しない)
    CAN_PACKET_CONF_BATTERY_CUT = 29,
    // 電流の制限を始める・終える電池電圧を設定し、保存する
    CAN_PACKET_CONF_STORE_BATTERY_CUT = 30,
    // 電源を切らせる (対応するハードウェアのみ)
    CAN_PACKET_SHUTDOWN = 31,
    // IOボード: ADC 1〜4の電圧
    CAN_PACKET_IO_BOARD_ADC_1_TO_4 = 32,
    // IOボード: ADC 5〜8の電圧
    CAN_PACKET_IO_BOARD_ADC_5_TO_8 = 33,
    // IOボード: ADC 9〜12の電圧
    CAN_PACKET_IO_BOARD_ADC_9_TO_12 = 34,
    // IOボード: デジタル入力
    CAN_PACKET_IO_BOARD_DIGITAL_IN = 35,
    // IOボード: デジタル出力を設定する
    CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL = 36,
    // IOボード: PWM出力のデューティ比を設定する
    CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM = 37,
    // BMS: パック電圧、充電器の電圧
    CAN_PACKET_BMS_V_TOT = 38,
    // BMS: 入力電流、測定した電流
    CAN_PACKET_BMS_I = 39,
    // BMS: 積算した電荷量 [Ah] と電力量 [Wh]
    CAN_PACKET_BMS_AH_WH = 40,
    // BMS: セル電圧 (複数フレームに分割)
    CAN_PACKET_BMS_V_CELL = 41,
    // BMS: 各セルのバランシング状態
    CAN_PACKET_BMS_BAL = 42,
    // BMS: 温度 (複数フレームに分割)
    CAN_PACKET_BMS_TEMPS = 43,
    // BMS: 湿度センサーの温度と湿度、バランスICの温度、気圧
    CAN_PACKET_BMS_HUM = 44,
    // BMS: セル電圧の最小・最大、SOC、SOH、セルの最高温度、充電やバランシングの状態
    CAN_PACKET_BMS_SOC_SOH_TEMP_STAT = 45,
    // 電源スイッチ: 入出力の電圧、温度、出力・プリチャージ・放電のON/OFF
    CAN_PACKET_PSW_STAT = 46,
    // 電源スイッチ: ON/OFFを切り替える
    CAN_PACKET_PSW_SWITCH = 47,
    // BMS: ハードウェア固有のデータ (このファームウェアでは処理されていない)
    CAN_PACKET_BMS_HW_DATA_1 = 48,
    // BMS: ハードウェア固有のデータ (このファームウェアでは処理されていない)
    CAN_PACKET_BMS_HW_DATA_2 = 49,
    // BMS: ハードウェア固有のデータ (このファームウェアでは処理されていない)
    CAN_PACKET_BMS_HW_DATA_3 = 50,
    // BMS: ハードウェア固有のデータ (このファームウェアでは処理されていない)
    CAN_PACKET_BMS_HW_DATA_4 = 51,
    // BMS: ハードウェア固有のデータ (このファームウェアでは処理されていない)
    CAN_PACKET_BMS_HW_DATA_5 = 52,
    // BMS: 充電した電荷量 [Ah] と電力量 [Wh] の累計
    CAN_PACKET_BMS_AH_WH_CHG_TOTAL = 53,
    // BMS: 放電した電荷量 [Ah] と電力量 [Wh] の累計
    CAN_PACKET_BMS_AH_WH_DIS_TOTAL = 54,
    // PIDによる位置制御のオフセットを更新する
    CAN_PACKET_UPDATE_PID_POS_OFFSET = 55,
    // ロータの位置 (エンコーダーの角度) を要求する
    CAN_PACKET_POLL_ROTOR_POS = 56,
    // 起動の通知 (このファームウェアでは処理されていない)
    CAN_PACKET_NOTIFY_BOOT = 57,
    // ステータス: ADC1〜3の電圧、PPMの入力
    CAN_PACKET_STATUS_6 = 58,
    // GNSS: 時刻
    CAN_PACKET_GNSS_TIME = 59,
    // GNSS: 緯度
    CAN_PACKET_GNSS_LAT = 60,
    // GNSS: 経度
    CAN_PACKET_GNSS_LON = 61,
    // GNSS: 高度、速度、HDOP
    CAN_PACKET_GNSS_ALT_SPEED_HDOP = 62,
    // CANのボーレートを変更する
    CAN_PACKET_UPDATE_BAUD = 63,
    // BMS: ステータス文字列の1番目のチャンク (8バイトずつ5フレームに分割)
    CAN_PACKET_BMS_STATUS_1 = 64,
    // BMS: ステータス文字列の2番目のチャンク
    CAN_PACKET_BMS_STATUS_2 = 65,
    // BMS: ステータス文字列の3番目のチャンク
    CAN_PACKET_BMS_STATUS_3 = 66,
    // BMS: ステータス文字列の4番目のチャンク
    CAN_PACKET_BMS_STATUS_4 = 67,
    // BMS: ステータス文字列の5番目のチャンク
    CAN_PACKET_BMS_STATUS_5 = 68,
};

// 拡張フレームのCAN IDは、下位8ビットがノードID、その上の8ビットがパケットID
// ref: https://github.com/vedderb/bldc/blob/4fd8279ea45a17c0d69357438ae2f7237a32514f/comm/comm_can.c#L1586-L1587
constexpr interface::CanFrame::Id NODE_ID_MASK = 0xFF;
constexpr interface::CanFrame::Id PACKET_ID_MASK = 0xFF;
constexpr unsigned int PACKET_ID_SHIFT = 8;

inline auto extract_node_id(const interface::CanFrame & frame) -> uint8_t {
    return static_cast<uint8_t>(frame.id & NODE_ID_MASK);
}

inline auto extract_packet_id(const interface::CanFrame & frame) -> PacketId {
    return static_cast<PacketId>((frame.id >> PACKET_ID_SHIFT) & PACKET_ID_MASK);
}

inline auto make_id(PacketId packet_id, uint8_t node_id) -> interface::CanFrame::Id {
    return (static_cast<interface::CanFrame::Id>(packet_id) & PACKET_ID_MASK) << PACKET_ID_SHIFT |
           (static_cast<interface::CanFrame::Id>(node_id) & NODE_ID_MASK);
}

// 複数フレームに分割して送られるデータの転送用フレームか
inline auto is_transport_frame(const interface::CanFrame & frame) -> bool {
    if (!frame.is_extended) {
        return false;
    }

    switch (extract_packet_id(frame)) {
        case PacketId::CAN_PACKET_FILL_RX_BUFFER:
        case PacketId::CAN_PACKET_FILL_RX_BUFFER_LONG:
        case PacketId::CAN_PACKET_PROCESS_RX_BUFFER:
        case PacketId::CAN_PACKET_PROCESS_SHORT_BUFFER:
            return true;
        default:
            return false;
    }
}

// VESCの`buffer_append_float32_auto`形式で格納された4バイトをdoubleに変換する
// ref: https://github.com/vedderb/bldc/blob/4fd8279ea45a17c0d69357438ae2f7237a32514f/util/buffer.c#L225-L243
inline auto to_float32_auto_be(const std::array<std::byte, 8> & bytes, std::size_t offset = 0)
    -> std::optional<double> {
    const auto raw = util::to_uint_be<uint32_t>(bytes, offset);
    if (!raw) {
        return std::nullopt;
    }
    auto exponent = static_cast<int>((raw.value() >> 23) & 0xFFU);
    const auto significand_raw = raw.value() & 0x7FFFFFU;
    double significand = 0.0;
    if (exponent != 0 || significand_raw != 0) {
        significand = static_cast<double>(significand_raw) / (8388608.0 * 2.0) + 0.5;
        exponent -= 126;
    }
    if ((raw.value() & 0x80000000U) != 0U) {
        significand = -significand;
    }
    return std::ldexp(significand, exponent);
}

}  // namespace sinsei_umiusi_control::hardware_model::can::vesc_protocol

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_CAN_VESC_PROTOCOL_HPP
