#include "camera/camera_receiver.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace rmcs_energy_trace_v2::camera {
namespace {

cv::Mat to_bgr(const sensor_msgs::msg::Image& image) {
    try {
        return cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
    } catch (...) {
        return {};
    }
}

cv::Mat to_depth(const sensor_msgs::msg::Image& image) {
    if (image.encoding != sensor_msgs::image_encodings::TYPE_16UC1 && image.encoding != "Z16")
        return {};

    const auto rows = static_cast<std::size_t>(image.height);
    const auto cols = static_cast<std::size_t>(image.width);
    constexpr auto kBytesPerPixel = sizeof(std::uint16_t);
    if (rows == 0 || cols == 0 || image.step < cols * kBytesPerPixel
        || image.data.size() < static_cast<std::size_t>(image.step) * rows)
        return {};

    auto depth = cv::Mat(static_cast<int>(image.height), static_cast<int>(image.width), CV_16UC1);
    for (std::size_t row = 0; row < rows; ++row)
        std::memcpy(
            depth.ptr<std::uint16_t>(static_cast<int>(row)), image.data.data() + row * image.step,
            cols * kBytesPerPixel);
    return depth;
}

} // namespace

CameraReceiver::CameraReceiver(rclcpp::Node& node, const Config& config, Callback callback)
    : callback_{std::move(callback)} {
    color_sub_ = node.create_subscription<Image>(
        config.color_topic, rclcpp::SensorDataQoS{},
        [this](const Image::ConstSharedPtr& color) { on_color(color); });
    depth_sub_ = node.create_subscription<Image>(
        config.depth_topic, rclcpp::SensorDataQoS{},
        [this](const Image::ConstSharedPtr& depth) { on_depth(depth); });
}

void CameraReceiver::on_color(const Image::ConstSharedPtr& color) {
    auto depth = Image::ConstSharedPtr{};
    {
        const auto lock = std::scoped_lock{mutex_};
        depth = latest_depth_;
    }
    if (depth)
        emit(color, depth);
}

void CameraReceiver::on_depth(const Image::ConstSharedPtr& depth) {
    const auto lock = std::scoped_lock{mutex_};
    latest_depth_ = depth;
}

void CameraReceiver::emit(const Image::ConstSharedPtr& color, const Image::ConstSharedPtr& depth) {
    const auto bgr = to_bgr(*color);
    const auto aligned_depth = to_depth(*depth);
    if (bgr.empty() || aligned_depth.empty() || bgr.size() != aligned_depth.size())
        return;

    callback_(
        Frame{
            .bgr = bgr,
            .depth = aligned_depth,
            .stamp = rclcpp::Time{color->header.stamp},
            .frame_id = color->header.frame_id,
        });
}

} // namespace rmcs_energy_trace_v2::camera
