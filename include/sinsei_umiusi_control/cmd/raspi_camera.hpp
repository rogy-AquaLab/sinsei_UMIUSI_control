#ifndef SINSEI_UMIUSI_CONTROL_CMD_RASPI_CAMERA_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_RASPI_CAMERA_HPP

namespace sinsei_umiusi_control::cmd::raspi_camera {

struct Config {
    int brightness;
    int exposure_value;
    int shutter_speed;
};

}  // namespace sinsei_umiusi_control::cmd::raspi_camera

#endif  // SINSEI_UMIUSI_CONTROL_CMD_RASPI_CAMERA_HPP