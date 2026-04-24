#ifndef SINSEI_UMIUSI_CONTROL_CAMERA_GST_CAMERA_NODE_HPP
#define SINSEI_UMIUSI_CONTROL_CAMERA_GST_CAMERA_NODE_HPP

#include <gst/gst.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace sinsei_umiusi_control {

class GstCameraNode final : public rclcpp::Node {
  private:
    static void initialize_gstreamer_once();
    void poll_bus();

    std::string pipeline_description;
    GstElement * pipeline = nullptr;
    GstBus * bus = nullptr;
    rclcpp::TimerBase::SharedPtr bus_poll_timer;

  public:
    GstCameraNode();
    ~GstCameraNode() override;
};

}  // namespace sinsei_umiusi_control

#endif  // SINSEI_UMIUSI_CONTROL_CAMERA_GST_CAMERA_NODE_HPP
