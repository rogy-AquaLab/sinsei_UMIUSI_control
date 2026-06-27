#include "sinsei_umiusi_control/hardware_model/impl/linux_gpio.hpp"

#include <exception>
#include <utility>
#include <vector>

#include "sinsei_umiusi_control/util/vector.hpp"

using namespace sinsei_umiusi_control::hardware_model;

impl::LinuxGpioLineRequest::LinuxGpioLineRequest(gpiod::line_bulk bulk) : bulk(std::move(bulk)) {}

impl::LinuxGpioLineRequest::~LinuxGpioLineRequest() {
    try {
        if (this->bulk) {
            this->bulk.release();
        }
    } catch (const std::exception &) {
        // Destructors must not throw.
    }
}

auto impl::LinuxGpioLineRequest::set_gpiod_values(const std::vector<GpioValue> & values)
    -> tl::expected<void, std::string> {
    const auto gpiod_values = util::cast_vector<int>(values);

    try {
        this->bulk.set_values(gpiod_values);
    } catch (const std::exception & e) {
        return tl::make_unexpected("Failed to set GPIO line values: " + std::string(e.what()));
    }

    return {};
}

auto impl::LinuxGpioLineRequest::set_values(const std::vector<GpioValue> & values)
    -> tl::expected<void, std::string> {
    if (values.size() != this->size()) {
        return tl::make_unexpected(
            "Invalid GPIO output values: expected " + std::to_string(this->size()) + ", got " +
            std::to_string(values.size()));
    }

    return this->set_gpiod_values(values);
}

auto impl::LinuxGpioLineRequest::size() const noexcept -> std::size_t { return this->bulk.size(); }

impl::LinuxGpioChip::LinuxGpioChip(std::string chip_path) : chip_path(std::move(chip_path)) {}

impl::LinuxGpioChip::~LinuxGpioChip() = default;

auto impl::LinuxGpioChip::request_gpiod_lines(const GpioOutputRequest & request)
    -> tl::expected<gpiod::line_bulk, std::string> {
    if (this->chip_path.empty()) {
        return tl::make_unexpected("GPIO chip path is empty");
    }

    const auto consumer = request.consumer.empty() ? DEFAULT_CONSUMER : request.consumer;

    try {
        auto chip = gpiod::chip(this->chip_path, gpiod::chip::OPEN_BY_PATH);
        auto bulk = chip.get_lines(request.offsets);
        const auto config = gpiod::line_request{consumer, gpiod::line_request::DIRECTION_OUTPUT, 0};

        const auto initial_values = util::cast_vector<int>(request.initial_values);

        bulk.request(config, initial_values);

        return bulk;
    } catch (const std::exception & e) {
        return tl::make_unexpected(
            "Failed to request GPIO output lines from '" + this->chip_path +
            "': " + std::string(e.what()));
    }
}

auto impl::LinuxGpioChip::request_outputs(GpioOutputRequest request)
    -> tl::expected<std::unique_ptr<interface::GpioLineRequest>, std::string> {
    if (request.offsets.empty()) {
        return tl::make_unexpected("GPIO output request must contain at least one line");
    }
    if (request.offsets.size() != request.initial_values.size()) {
        return tl::make_unexpected(
            "GPIO output request offsets and initial_values must have the same size");
    }

    const auto gpiod_lines_res = this->request_gpiod_lines(request);
    if (!gpiod_lines_res) {
        return tl::make_unexpected(gpiod_lines_res.error());
    }

    return std::make_unique<LinuxGpioLineRequest>(std::move(gpiod_lines_res.value()));
}
