#ifndef SINSEI_UMIUSI_CONTROL_CAMERA_GST_CAMERA_NODE_HPP
#define SINSEI_UMIUSI_CONTROL_CAMERA_GST_CAMERA_NODE_HPP

#include <gstreamermm-1.0/gstreamermm.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace sinsei_umiusi_control {

class GstCameraNode : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer;

  Glib::RefPtr<Gst::Element> pipeline;
  Glib::RefPtr<Gst::Bus> bus;

  auto init_pipeline(const std::string &pipeline_description) -> void;
  auto stop_pipeline() noexcept -> void;
  auto handle_bus_message(const Glib::RefPtr<Gst::Message> &message) -> bool;
  auto timer_callback() -> void;

public:
  GstCameraNode();
  ~GstCameraNode() override;
};

} // namespace sinsei_umiusi_control

#endif // SINSEI_UMIUSI_CONTROL_CAMERA_GST_CAMERA_NODE_HPP
