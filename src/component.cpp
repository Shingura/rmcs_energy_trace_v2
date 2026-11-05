#include <algorithm>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <eigen3/Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "camera/camera_receiver.hpp"
#include "detection/detector.hpp"

namespace rmcs_energy_trace_v2 {

// 当前只接入相机和检测。位姿输出保持等待状态，不初始化位姿估计器或响应遥控器。
class EnergyTraceComponent final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    EnergyTraceComponent()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_output("/energy_trace/status", status_, 0);
        register_output("/energy_trace/is_pose_valid", is_pose_valid_, false);
        register_output("/energy_trace/pose", pose_, Eigen::Isometry3d::Identity());

        auto config = detection::Detector::Config{};
        config.network_path = get_parameter("network_path").as_string();
        config.infer_threads = get_parameter_or<int>("threads", 4);
        if (const auto initialized = detector_.initialize(config); !initialized)
            throw std::runtime_error{initialized.error()};

        detection_image_pub_ =
            create_publisher<sensor_msgs::msg::Image>("/energy_trace/detection_image", 1);
        auto receiver_config = camera::CameraReceiver::Config{};
        receiver_config.color_topic =
            get_parameter_or<std::string>("color_topic", "/camera/camera/color/image_raw");
        receiver_config.depth_topic = get_parameter_or<std::string>(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
        camera_receiver_ = std::make_unique<camera::CameraReceiver>(
            *this, receiver_config, [this](const camera::Frame& frame) { on_frame(frame); });
    }

    void update() override {}

private:
    void on_frame(const camera::Frame& frame) {
        const auto detections = detector_.detect(frame.bgr);
        if (detection_image_pub_->get_subscription_count() == 0)
            return;

        auto display = cv_bridge::CvImage{};
        display.header.stamp = frame.stamp;
        display.header.frame_id = frame.frame_id;
        display.encoding = sensor_msgs::image_encodings::BGR8;
        display.image = frame.bgr.clone();
        for (const auto& detection : detections) {
            if (!detection.mask.empty()) {
                auto tinted = display.image.clone();
                tinted.setTo(cv::Scalar{255, 255, 0}, detection.mask);
                cv::addWeighted(display.image, 0.5, tinted, 0.5, 0., display.image);
            }
            cv::rectangle(display.image, detection.box, {0, 255, 0}, 2);
            cv::putText(
                display.image, std::format("{:.3f}", detection.score),
                {static_cast<int>(detection.box.x),
                 std::max(static_cast<int>(detection.box.y) - 6, 12)},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
        }
        detection_image_pub_->publish(*display.toImageMsg());
    }

    OutputInterface<int> status_;
    OutputInterface<bool> is_pose_valid_;
    OutputInterface<Eigen::Isometry3d> pose_;
    detection::Detector detector_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_image_pub_;
    std::unique_ptr<camera::CameraReceiver> camera_receiver_;
};

} // namespace rmcs_energy_trace_v2

PLUGINLIB_EXPORT_CLASS(rmcs_energy_trace_v2::EnergyTraceComponent, rmcs_executor::Component)
