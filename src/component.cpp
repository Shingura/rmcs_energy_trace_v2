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
        camera_receiver_ = std::make_unique<camera::CameraReceiver>(
            *this, [this](const camera::Frame& frame) { on_frame(frame); });
    }

    void update() override {}

private:
    // For debug：从彩色图中检测能量单元并绘制
    void on_frame(const camera::Frame& frame) {
        // 对彩色图进行检测
        const auto detections = detector_.detect(frame.bgr);
        // 没有订阅者时直接返回，节省绘制耗时
        if (detection_image_pub_->get_subscription_count() == 0)
            return;

        // 复制彩色图作为底图，保留时间戳和坐标系
        auto display = cv_bridge::CvImage{};
        display.header.stamp = frame.stamp;
        display.header.frame_id = frame.frame_id;
        display.encoding = sensor_msgs::image_encodings::BGR8;
        display.image = frame.bgr.clone();

        for (const auto& detection : detections) {
            // 有掩码时，将掩码区域涂为半透明青色
            if (!detection.mask.empty()) {
                auto tinted = display.image.clone();
                tinted.setTo(cv::Scalar{255, 255, 0}, detection.mask);
                cv::addWeighted(display.image, 0.5, tinted, 0.5, 0., display.image);
            }
            // 绘制绿色检测矩形框
            cv::rectangle(display.image, detection.box, {0, 255, 0}, 2);
            // 在框上方写上检测得分
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
