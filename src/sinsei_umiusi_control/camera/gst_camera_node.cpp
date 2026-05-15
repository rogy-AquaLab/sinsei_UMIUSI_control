#include "sinsei_umiusi_control/camera/gst_camera_node.hpp"

#include <stdexcept>

using namespace sinsei_umiusi_control;

GstCameraNode::GstCameraNode() : Node("gst_camera_node") {
    this->declare_parameter<std::string>("pipeline", "");

    const auto pipeline_description = this->get_parameter("pipeline").as_string();
    if (pipeline_description.empty()) {
        throw std::runtime_error("Pipeline parameter must not be empty.");
    }
    RCLCPP_INFO(
        this->get_logger(), "Starting GStreamer pipeline: %s", pipeline_description.c_str());

    this->init_pipeline(pipeline_description);
    this->bus_thread = std::thread(&GstCameraNode::bus_loop, this);
}

GstCameraNode::~GstCameraNode() {
    this->stop_requested.store(true);
    if (this->bus) {
        this->bus->set_flushing(true);
    }
    if (this->bus_thread.joinable()) {
        this->bus_thread.join();
    }
    this->stop_pipeline();
}

auto GstCameraNode::init_pipeline(const std::string & pipeline_description) -> void {
    this->pipeline = Gst::Parse::launch(pipeline_description);
    if (!this->pipeline) {
        throw std::runtime_error("Failed to parse GStreamer pipeline.");
    }

    this->bus = this->pipeline->get_bus();
    if (!this->bus) {
        this->stop_pipeline();
        throw std::runtime_error("Failed to get GStreamer bus.");
    }

    const auto res = this->pipeline->set_state(Gst::STATE_PLAYING);
    if (res == Gst::STATE_CHANGE_FAILURE) {
        this->stop_pipeline();
        throw std::runtime_error("Failed to start GStreamer pipeline.");
    }
}

auto GstCameraNode::stop_pipeline() noexcept -> void {
    this->bus.reset();

    if (!this->pipeline) {
        return;
    }

    this->pipeline->set_state(Gst::STATE_NULL);
    this->pipeline.reset();
}

auto GstCameraNode::handle_bus_message(const Glib::RefPtr<Gst::Message> & message) -> bool {
    switch (message->get_message_type()) {
        case Gst::MESSAGE_ERROR: {
            const auto error_message =
                Gst::wrap_msg_derived<Gst::MessageError>(message->gobj(), true);
            const auto error = error_message->parse_error();
            const auto debug = error_message->parse_debug();
            RCLCPP_ERROR(
                this->get_logger(), "\n  GStreamer pipeline error: %s%s%s", error.what().c_str(),
                debug.empty() ? "" : "\n  debug: ", debug.c_str());
            return false;
        }
        case Gst::MESSAGE_EOS:
            RCLCPP_WARN(this->get_logger(), "GStreamer pipeline reached EOS.");
            return false;
        default:
            return true;
    }
}

auto GstCameraNode::bus_loop() -> void {
    const auto bus = this->bus;
    if (!bus) {
        return;
    }

    constexpr auto terminal_messages =
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS);

    while (!this->stop_requested.load()) {
        const auto message_raw =
            gst_bus_timed_pop_filtered(bus->gobj(), GST_CLOCK_TIME_NONE, terminal_messages);
        if (!message_raw) {
            continue;
        }

        const auto message = Glib::wrap(message_raw, false);
        if (!this->handle_bus_message(message)) {
            this->stop_requested.store(true);
            this->stop_pipeline();
            rclcpp::shutdown();
            return;
        }
    }
}

auto main(int argc, char ** argv) -> int {
    try {
        Gst::init(argc, argv);
        rclcpp::init(argc, argv);

        auto node = std::make_shared<sinsei_umiusi_control::GstCameraNode>();
        rclcpp::spin(node);
    } catch (const Glib::Error & error) {
        RCLCPP_FATAL(
            rclcpp::get_logger("gst_camera_node"), "GStreamer error: %s", error.what().c_str());
        rclcpp::shutdown();
        return 1;
    } catch (const std::exception & exception) {
        RCLCPP_FATAL(
            rclcpp::get_logger("gst_camera_node"), "Unhandled exception in gst_camera_node: %s",
            exception.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
