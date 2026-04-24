#include "sinsei_umiusi_control/camera/gst_camera_node.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>

namespace sinsei_umiusi_control {

GstCameraNode::GstCameraNode() : Node("gst_camera_node") {
    this->declare_parameter<std::string>("pipeline", "");

    this->initialize_gstreamer_once();

    this->pipeline_description = this->get_parameter("pipeline").as_string();
    if (this->pipeline_description.empty()) {
        throw std::invalid_argument("Pipeline parameter must not be empty.");
    }
    RCLCPP_INFO(
        this->get_logger(), "Starting GStreamer pipeline: %s", this->pipeline_description.c_str());

    GError * parse_error = nullptr;
    this->pipeline = gst_parse_launch(this->pipeline_description.c_str(), &parse_error);
    if (parse_error) {
        const auto error_message = std::string{parse_error->message};
        g_error_free(parse_error);
        if (this->pipeline) {
            gst_object_unref(this->pipeline);
            this->pipeline = nullptr;
        }
        throw std::runtime_error("Failed to parse GStreamer pipeline: " + error_message);
    }
    if (!this->pipeline) {
        throw std::runtime_error("Failed to parse GStreamer pipeline.");
    }

    this->bus = gst_element_get_bus(this->pipeline);
    if (!this->bus) {
        throw std::runtime_error("Failed to get GStreamer bus.");
    }

    const auto res = gst_element_set_state(this->pipeline, GST_STATE_PLAYING);
    if (res == GST_STATE_CHANGE_FAILURE) {
        throw std::runtime_error("Failed to start GStreamer pipeline.");
    }

    constexpr auto POLL_INTERVAL_MS = 100;
    this->bus_poll_timer = this->create_wall_timer(
        std::chrono::milliseconds(POLL_INTERVAL_MS), [this]() { this->poll_bus(); });
}

GstCameraNode::~GstCameraNode() {
    this->bus_poll_timer.reset();
    if (this->pipeline) {
        gst_element_set_state(this->pipeline, GST_STATE_NULL);
    }
    if (this->bus) {
        gst_object_unref(this->bus);
        this->bus = nullptr;
    }
    if (this->pipeline) {
        gst_object_unref(this->pipeline);
        this->pipeline = nullptr;
    }
}

void GstCameraNode::initialize_gstreamer_once() {
    static const bool initialized = []() {
        gst_init(nullptr, nullptr);
        return true;
    }();
    (void)initialized;
}

void GstCameraNode::poll_bus() {
    while (true) {
        auto * message = gst_bus_pop(this->bus);
        if (!message) {
            return;
        }

        switch (GST_MESSAGE_TYPE(message)) {
            case GST_MESSAGE_ERROR: {
                GError * error = nullptr;
                gchar * debug = nullptr;
                gst_message_parse_error(message, &error, &debug);
                RCLCPP_ERROR(
                    this->get_logger(), "\n  GStreamer pipeline error: %s%s%s",
                    error ? error->message : "Unknown error", debug ? "\n  debug: " : "",
                    debug ? debug : "");
                if (error) {
                    g_error_free(error);
                }
                if (debug) {
                    g_free(debug);
                }
                gst_message_unref(message);
                throw std::runtime_error("GStreamer pipeline reported an error.");
            }
            case GST_MESSAGE_EOS:
                RCLCPP_WARN(this->get_logger(), "GStreamer pipeline reached EOS.");
                gst_message_unref(message);
                return;
            default:
                gst_message_unref(message);
                break;
        }
    }
}

}  // namespace sinsei_umiusi_control

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<sinsei_umiusi_control::GstCameraNode>();
        rclcpp::spin(node);
    } catch (const std::exception & exception) {
        RCLCPP_FATAL(
            rclcpp::get_logger("gst_camera_node"), "Failed to start GStreamer camera node: %s",
            exception.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
