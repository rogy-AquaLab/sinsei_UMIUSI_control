#ifndef SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP

#include <cstddef>
#include <cstdint>

#include "sinsei_umiusi_control/util/bms_status.hpp"

namespace sinsei_umiusi_control::state::bms {

constexpr std::size_t CELL_COUNT = 12;
// 互換基板に搭載した追加の温度センサーは5個
constexpr std::size_t ADDITIONAL_TEMPERATURE_COUNT = 5;

// バッテリー自体の状態ではなく、BMSとのCAN通信が続いているか
// control側では、BMSから送られてくるfault以外にバッテリーの異常検知は行っていない
struct Health {
    bool is_ok;
};

struct Voltages {
    double pack;
    double charger;
};

struct Currents {
    double input;
    double measured;
};

struct CapacityState {
    double state_of_charge;
    double state_of_health;
};

struct CellVoltageRange {
    double min;
    double max;
};

struct ChargeStatus {
    bool is_charging;
    bool is_balancing;
    bool is_charge_allowed;
};

struct Status {
    util::BmsFaults faults;
    util::BmsPowerSwitchState power_switch_state;
};

struct Cell {
    double voltage;
    bool is_balancing;
};

struct CellCount {
    uint8_t value;
};

struct Temperature {
    double value;
};

}  // namespace sinsei_umiusi_control::state::bms

#endif  // SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP
