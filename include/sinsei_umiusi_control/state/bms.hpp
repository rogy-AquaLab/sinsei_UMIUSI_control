#ifndef SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP

#include <array>
#include <cstdint>

namespace sinsei_umiusi_control::state::bms {

struct Voltages {
    float pack;
    float charger;
};

struct Currents {
    float input;
    float measured;
};

struct CapacityState {
    float state_of_charge;
    float state_of_health;
};

struct CellVoltageRange {
    float min;
    float max;
};

struct Status {
    uint32_t fault_flags;
    uint8_t power_switch_state;
    bool charging;
    bool balancing;
    bool charge_allowed;
};

struct Cell {
    float voltage;
    bool balancing;
};

struct Temperatures {
    double balance_ic;
    double mosfet;
    double ambient;
    std::array<double, 5> additional;
};

}  // namespace sinsei_umiusi_control::state::bms

#endif  // SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP
