#include "camera/camera_receiver.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/logging.hpp>
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
    if (image.encoding != sensor_msgs::image_encodings::TYPE_16UC1 && image.encoding != "Z16") return {};

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

CameraReceiver::CameraReceiver(rclcpp::Node& node, Callback callback)
    : callback_{ std::move(callback) }
    , logger_{ node.get_logger() } {

    info_sub_ = node.create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera/color/camera_info", rclcpp::SensorDataQoS{},
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) { on_camera_info(info); });

    color_sub_ = node.create_subscription<Image>(
    "/camera/camera/color/image_raw", rclcpp::SensorDataQoS{},
    [this](const Image::ConstSharedPtr& color) { on_color(color); });

    depth_sub_ = node.create_subscription<Image>(
    "/camera/camera/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS{},
    [this](const Image::ConstSharedPtr& depth) { on_depth(depth); });
}

void CameraReceiver::on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
    // 只存储第一次获得的相机内参
    if (camera_.load()) return;

    if (info->width == 0 || info->height == 0 || !std::isfinite(info->k[0])
        || !std::isfinite(info->k[4]) || !std::isfinite(info->k[2]) || !std::isfinite(info->k[5])
        || info->k[0] <= 0. || info->k[4] <= 0.) {
        RCLCPP_ERROR_ONCE(logger_, "CameraInfo has invalid image dimensions or intrinsics");
        return;
    }

    const auto camera = std::make_shared<const geometry::Camera>(geometry::Camera{
        .width = static_cast<int>(info->width),
        .height = static_cast<int>(info->height),
        .fx = info->k[0],
        .fy = info->k[4],
        .cx = info->k[2],
        .cy = info->k[5],
        // 深度图单位统一换算为毫米
        .depth_scale = 0.001,
    });

    camera_.store(camera);

    RCLCPP_INFO(
        logger_, "CameraInfo: %dx%d, fx=%.6f fy=%.6f cx=%.6f cy=%.6f, depth_scale=%.3f m/unit",
        camera->width, camera->height, camera->fx, camera->fy, camera->cx, camera->cy,
        camera->depth_scale);
}

void CameraReceiver::on_color(const Image::ConstSharedPtr& color) {
    const auto camera = camera_.load();
    if (!camera) return;

    const auto depth = latest_depth_.load();
    if (depth) { emit(color, depth, *camera); }
}

void CameraReceiver::on_depth(const Image::ConstSharedPtr& depth) { latest_depth_.store(depth); }

void CameraReceiver::emit(
    const Image::ConstSharedPtr& color, const Image::ConstSharedPtr& depth,
    const geometry::Camera& camera) {
    // 检查图像是否符合相机内参
    if (color->width != static_cast<std::uint32_t>(camera.width)
        || color->height != static_cast<std::uint32_t>(camera.height)
        || depth->width != color->width || depth->height != color->height) {
        RCLCPP_ERROR_ONCE(
            logger_, "RGB-D image dimensions do not match CameraInfo; frame rejected");
        return;
    }

    const auto bgr = to_bgr(*color);
    const auto aligned_depth = to_depth(*depth);
    if (bgr.empty() || aligned_depth.empty()) return;

    callback_(
        Frame{
            .bgr = bgr,
            .depth = aligned_depth,
            .camera = camera,
            .stamp = rclcpp::Time{color->header.stamp},
            .frame_id = color->header.frame_id,
        });
}

} // namespace rmcs_energy_trace_v2::camera
