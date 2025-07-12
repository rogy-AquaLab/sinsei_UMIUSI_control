#ifndef SINSEI_UMIUSI_CONTROL_LED_TAPE_HPP
#define SINSEI_UMIUSI_CONTROL_LED_TAPE_HPP

#include <cstdint>

namespace sinsei_umiusi_control::cmd::led_tape {

struct Color {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

}  // namespace sinsei_umiusi_control::cmd::led_tape

#endif  // SINSEI_UMIUSI_CONTROL_LED_TAPE_HPP