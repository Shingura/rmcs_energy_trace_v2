#pragma once

#include <functional>
#include <mutex>
#include <string>

#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rmcs_energy_trace_v2::camera {

// 一帧 RGB-D 图像，深度图取最近收到的一帧
struct Frame {
    cv::Mat bgr;   // CV_8UC3
    cv::Mat depth; // CV_16UC1，0 为无效
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::string frame_id;
};

// 订阅彩色图和已对齐的深度图，用最近收到的深度图组成帧后交给回调
class CameraReceiver {
public:
    using Callback = std::function<void(const Frame&)>;

    struct Config {
        std::string color_topic;
        std::string depth_topic;
    };

    CameraReceiver(rclcpp::Node& node, const Config& config, Callback callback);

private:
    using Image = sensor_msgs::msg::Image;

    void on_color(const Image::ConstSharedPtr& color);
    void on_depth(const Image::ConstSharedPtr& depth);
    void emit(const Image::ConstSharedPtr& color, const Image::ConstSharedPtr& depth);

    Callback callback_;
    std::mutex mutex_;
    Image::ConstSharedPtr latest_depth_;
    rclcpp::Subscription<Image>::SharedPtr color_sub_;
    rclcpp::Subscription<Image>::SharedPtr depth_sub_;
};

} // namespace rmcs_energy_trace_v2::camera
