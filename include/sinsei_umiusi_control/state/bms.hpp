#ifndef SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP

#include <cstddef>
#include <cstdint>

#include "sinsei_umiusi_control/util/bms_status.hpp"

namespace sinsei_umiusi_control::state::bms {

constexpr std::size_t CELL_COUNT = 12;
// 互換基板に搭載した追加の温度センサーは5個
constexpr std::size_t ADDITIONAL_TEMPERATURE_COUNT = 5;

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

struct Status {
    util::BmsFaults faults;
    util::BmsPowerSwitchState power_switch_state;
    bool charging;
    bool balancing;
    bool charge_allowed;
};

struct Cell {
    double voltage;
    bool balancing;
};

struct CellCount {
    uint8_t value;
};

struct Temperature {
    double value;
};

}  // namespace sinsei_umiusi_control::state::bms

#endif  // SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP
