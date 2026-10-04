#ifndef SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP

#include <cstdint>

namespace sinsei_umiusi_control::state::bms {

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
    uint32_t fault_flags;
    uint8_t power_switch_state;
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
