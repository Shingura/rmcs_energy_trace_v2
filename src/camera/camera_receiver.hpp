#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "geometry/camera.hpp"

namespace rmcs_energy_trace_v2::camera {

// 一帧 RGB-D 图像，深度图取最近收到的一帧
struct Frame {
    cv::Mat bgr;   // CV_8UC3
    cv::Mat depth; // CV_16UC1，单位 mm，0 为无效
    geometry::Camera camera;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::string frame_id;
};

// 保存首次有效的彩色 CameraInfo；收到内参后，用彩色图和最近的对齐深度组成帧。
// 采集配置在运行期间保持不变，更换配置后需重启。
class CameraReceiver {
public:
    using Callback = std::function<void(const Frame&)>;

    CameraReceiver(rclcpp::Node& node, Callback callback);

private:
    using Image = sensor_msgs::msg::Image;

    void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);
    void on_color(const Image::ConstSharedPtr& color);
    void on_depth(const Image::ConstSharedPtr& depth);
    void emit(
        const Image::ConstSharedPtr& color, const Image::ConstSharedPtr& depth,
        const geometry::Camera& camera);

    Callback callback_;
    rclcpp::Logger logger_;
    std::atomic<std::shared_ptr<const geometry::Camera>> camera_;
    std::atomic<Image::ConstSharedPtr> latest_depth_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<Image>::SharedPtr color_sub_;
    rclcpp::Subscription<Image>::SharedPtr depth_sub_;
};

} // namespace rmcs_energy_trace_v2::camera
