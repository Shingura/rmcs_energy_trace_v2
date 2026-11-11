#include <algorithm>
#include <chrono>
#include <filesystem>
#include <format>
#include <memory>
#include <optional>
#include <rclcpp/time.hpp>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
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
#include "estimation/initialization.hpp"
#include "estimation/target_estimator.hpp"

namespace rmcs_energy_trace_v2 {

// 接入相机、检测与目标估计器。估计器在首帧拿到真实内参后初始化，尚未逐帧推进；
// 位姿输出保持等待状态，不响应遥控器。
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

        network_path_ = get_parameter_or<std::string>("network_path", "");
        mesh_path_ = get_parameter_or<std::string>("mesh_path", "");
        model_dir_ = get_parameter_or<std::string>("model_dir", "");
        threads_ = get_parameter_or<int>("threads", 4);

        // 留空时用包安装后的 share 目录，路径随 install 位置变化
        const auto share = std::filesystem::path{
            ament_index_cpp::get_package_share_directory("rmcs_energy_trace_v2")};
        if (network_path_.empty())
            network_path_ = (share / "assets" / "yolo11n_seg_v1.xml").string();
        if (mesh_path_.empty())
            mesh_path_ = (share / "meshes" / "energy_unit.stl").string();
        if (model_dir_.empty())
            model_dir_ = (share / "assets").string();

        RCLCPP_INFO(
            get_logger(), "network='%s' mesh='%s' model_dir='%s'", network_path_.c_str(),
            mesh_path_.c_str(), model_dir_.c_str());

        auto config = detection::Detector::Config{};
        config.network_path = network_path_;
        config.infer_threads = threads_;

        if (const auto initialized = detector_.initialize(config); !initialized)
            throw std::runtime_error{initialized.error()};

        detection_image_pub_ =
            create_publisher<sensor_msgs::msg::Image>("/energy_trace/detection_image", 1);
        camera_receiver_ = std::make_unique<camera::CameraReceiver>(
            *this, [this](const camera::Frame& frame) { on_frame(frame); });
    }

    void update() override {}

private:
    using Clock = estimation::Initialization::Clock;

    // 从 ROS 时间戳转换为 estimator 单调时钟时间
    Clock::time_point to_estimator_time(const rclcpp::Time& ros_stamp) {
        // 初始化第一帧时间戳，作为计时起点
        if (!first_ros_stamp_) {
            first_ros_stamp_ = ros_stamp;
            first_estimator_stamp_ = Clock::now();
        }

        // 计算经过时间
        const auto delta =
            std::chrono::nanoseconds{ros_stamp.nanoseconds() - first_ros_stamp_->nanoseconds()};

        return first_estimator_stamp_ + std::chrono::duration_cast<Clock::duration>(delta);
    }

    // 初始化 estimator
    void initialize_estimator(const geometry::Camera& camera) {
        estimator_initialized_ = true;

        auto config = estimation::TargetEstimator::Config{};
        config.camera = camera;
        config.threads = threads_;
        config.detector.network_path = network_path_;
        config.tracker.model_dir = model_dir_;

        if (const auto initialized = estimator_.initialize(mesh_path_, config); !initialized)
            RCLCPP_ERROR(get_logger(), "Target estimator: %s", initialized.error().c_str());
        else
            RCLCPP_INFO(
                get_logger(), "Target estimator initialized, %dx%d", camera.width, camera.height);
    }

    // For debug：从彩色图中检测能量单元并绘制
    void on_frame(const camera::Frame& frame) {
        current_estimator_stamp_ = to_estimator_time(frame.stamp);

        if (!estimator_initialized_)
            initialize_estimator(frame.camera);

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

    std::string network_path_;
    std::string mesh_path_;
    std::string model_dir_;
    int threads_ = 4;

    detection::Detector detector_;
    estimation::TargetEstimator estimator_;
    bool estimator_initialized_ = false;
    std::optional<rclcpp::Time> first_ros_stamp_;
    Clock::time_point first_estimator_stamp_{};
    Clock::time_point current_estimator_stamp_{};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_image_pub_;
    std::unique_ptr<camera::CameraReceiver> camera_receiver_;
};

} // namespace rmcs_energy_trace_v2

PLUGINLIB_EXPORT_CLASS(rmcs_energy_trace_v2::EnergyTraceComponent, rmcs_executor::Component)
