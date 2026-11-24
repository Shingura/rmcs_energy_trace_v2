#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <format>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "camera/camera_receiver.hpp"
#include "detection/detector.hpp"
#include "estimation/initialization.hpp"
#include "estimation/target_estimator.hpp"
#include "geometry/mesh.hpp"

namespace rmcs_energy_trace_v2 {

// 接入相机、检测、目标估计器与遥控器。估计器在首帧拿到真实内参后初始化，
// 由工作线程逐帧推进，主线程响应遥控器双下解锁/复位、左上右下启动并输出最近估计结果。
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

        // 遥控器不是必须的：没有接遥控器时，跳过开关读取
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);

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

        const auto camera_translation = get_parameter_or<std::vector<double>>(
            "camera_translation", std::vector<double>{0., 0., 0.});
        const auto camera_rpy =
            get_parameter_or<std::vector<double>>("camera_rpy", std::vector<double>{0., 0., 0.});
        camera_to_base_ = make_camera_to_base(camera_translation, camera_rpy);

        RCLCPP_INFO(
            get_logger(), "camera_translation=[%.4f, %.4f, %.4f] camera_rpy=[%.4f, %.4f, %.4f]",
            camera_translation[0], camera_translation[1], camera_translation[2], camera_rpy[0],
            camera_rpy[1], camera_rpy[2]);

        auto config = detection::Detector::Config{};
        config.network_path = network_path_;
        config.infer_threads = threads_;

        if (const auto initialized = detector_.initialize(config); !initialized)
            throw std::runtime_error{initialized.error()};

        const auto mesh = geometry::load_binary_stl(mesh_path_);
        if (!mesh)
            throw std::runtime_error{mesh.error()};
        unit_marker_template_ = make_unit_marker_template(*mesh);

        detection_image_pub_ =
            create_publisher<sensor_msgs::msg::Image>("/energy_trace/detection_image", 1);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/energy_trace/pose", 1);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/energy_trace/marker", 1);
        camera_receiver_ = std::make_unique<camera::CameraReceiver>(
            *this, [this](const camera::Frame& frame) { on_frame(frame); });

        // 在独立工作线程中运行 work_loop()
        worker_ = std::thread{[this] { work_loop(); }};
    }

    ~EnergyTraceComponent() override {
        camera_receiver_.reset();
        // 组件析构时将 should_stop_ 置为 true，停止 worker_ 线程
        should_stop_.store(true);
        if (worker_.joinable())
            worker_.join();
    }

    void update() override {
        handle_remote();

        const auto lock = std::lock_guard{result_mutex_};
        *status_ = latest_estimation_result_.status;
        *is_pose_valid_ = latest_estimation_result_.is_pose_valid;
        *pose_ = latest_estimation_result_.pose;
    }

private:
    using Clock = estimation::Initialization::Clock;

    // 帧队列元素，同时包括帧本身与其时间戳
    struct QueuedFrame {
        camera::Frame frame;
        Clock::time_point stamp;
    };

    // 工作线程与主线程共享的最近估计结果
    struct LatestEstimationResult {
        int status = static_cast<int>(estimation::Initialization::Status::kIdle);
        bool is_pose_valid = false;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    };

    // 主线程发往工作线程的控制指令
    enum class Command : std::uint8_t { kNone, kStart, kReset };

    static constexpr const char* status_name(estimation::Initialization::Status status) noexcept {
        switch (status) {
        case estimation::Initialization::Status::kIdle: return "idle";
        case estimation::Initialization::Status::kInitializing: return "initializing";
        case estimation::Initialization::Status::kReady: return "ready";
        case estimation::Initialization::Status::kFailed: return "failed";
        }
        return "unknown";
    }

    // 在工作线程按状态变化或固定周期打印估计结果
    void log_estimation(
        estimation::Initialization::Status status, bool pose_valid,
        const Eigen::Isometry3d& pose) const {
        if (status == estimation::Initialization::Status::kReady || pose_valid) {
            const Eigen::Vector3d t = pose.translation();
            const Eigen::Vector3d z = pose.linear().col(2);
            const auto q = Eigen::Quaterniond{pose.linear()};
            RCLCPP_INFO(
                get_logger(),
                "status=%s valid=%d t=[%.4f, %.4f, %.4f] z=[%.3f, %.3f, %.3f] "
                "q=[%.3f, %.3f, %.3f, %.3f]",
                status_name(status), static_cast<int>(pose_valid), t.x(), t.y(), t.z(), z.x(),
                z.y(), z.z(), q.w(), q.x(), q.y(), q.z());
        } else {
            RCLCPP_INFO(
                get_logger(), "status=%s valid=%d", status_name(status),
                static_cast<int>(pose_valid));
        }
    }

    static geometry_msgs::msg::Pose to_pose_msg(const Eigen::Isometry3d& transform) {
        auto message = geometry_msgs::msg::Pose{};
        const Eigen::Vector3d t = transform.translation();
        const auto q = Eigen::Quaterniond{transform.linear()};
        message.position.x = t.x();
        message.position.y = t.y();
        message.position.z = t.z();
        message.orientation.w = q.w();
        message.orientation.x = q.x();
        message.orientation.y = q.y();
        message.orientation.z = q.z();
        return message;
    }

    // 预构造按原点居中的能量单元 TRIANGLE_LIST 网格模板
    static visualization_msgs::msg::Marker make_unit_marker_template(const geometry::Mesh& mesh) {
        auto marker = visualization_msgs::msg::Marker{};
        marker.header.frame_id = kBaseFrame;
        marker.ns = "energy_unit";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.;
        marker.scale.x = 1.;
        marker.scale.y = 1.;
        marker.scale.z = 1.;
        marker.points.reserve(mesh.triangles.size() * 3);
        for (const auto& triangle : mesh.triangles) {
            for (const auto& vertex : triangle) {
                auto point = geometry_msgs::msg::Point{};
                point.x = static_cast<double>(vertex.x());
                point.y = static_cast<double>(vertex.y());
                point.z = static_cast<double>(vertex.z());
                marker.points.push_back(point);
            }
        }
        return marker;
    }

    void publish_pose(const rclcpp::Time& stamp, const Eigen::Isometry3d& pose) const {
        if (pose_pub_->get_subscription_count() == 0)
            return;

        auto message = geometry_msgs::msg::PoseStamped{};
        message.header.stamp = stamp;
        message.header.frame_id = kBaseFrame;
        message.pose = to_pose_msg(pose);
        pose_pub_->publish(message);
    }

    void publish_marker(
        const rclcpp::Time& stamp, estimation::Initialization::Status status,
        const Eigen::Isometry3d& pose) {
        if (marker_pub_->get_subscription_count() == 0)
            return;

        unit_marker_template_.header.stamp = stamp;
        unit_marker_template_.pose = to_pose_msg(pose);
        if (status == estimation::Initialization::Status::kReady) {
            unit_marker_template_.color.r = 0.15f;
            unit_marker_template_.color.g = 0.85f;
            unit_marker_template_.color.b = 0.35f;
            unit_marker_template_.color.a = 0.85f;
        } else {
            unit_marker_template_.color.r = 0.95f;
            unit_marker_template_.color.g = 0.62f;
            unit_marker_template_.color.b = 0.12f;
            unit_marker_template_.color.a = 0.65f;
        }
        marker_pub_->publish(unit_marker_template_);
    }

    void delete_marker(const rclcpp::Time& stamp) const {
        if (marker_pub_->get_subscription_count() == 0)
            return;

        auto marker = visualization_msgs::msg::Marker{};
        marker.header.stamp = stamp;
        marker.header.frame_id = kBaseFrame;
        marker.ns = "energy_unit";
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_pub_->publish(marker);
    }

    // 按 ROS 相机系参数（外旋 XYZ / 内旋 ZYX 的 RPY）构造 OpenCV 光学系到底座的变换
    static Eigen::Isometry3d make_camera_to_base(
        const std::vector<double>& translation, const std::vector<double>& rpy) {
        if (translation.size() != 3)
            throw std::runtime_error{
                "Parameter 'camera_translation' must have 3 elements, got "
                + std::to_string(translation.size())};
        if (rpy.size() != 3)
            throw std::runtime_error{
                "Parameter 'camera_rpy' must have 3 elements, got " + std::to_string(rpy.size())};

        const auto rotation = Eigen::Quaterniond{
            Eigen::AngleAxisd{rpy[2], Eigen::Vector3d::UnitZ()}
            * Eigen::AngleAxisd{rpy[1], Eigen::Vector3d::UnitY()}
            * Eigen::AngleAxisd{rpy[0], Eigen::Vector3d::UnitX()}};
        const auto optical_to_ros = Eigen::Matrix3d{
            {0., 0., 1.},
            {-1., 0., 0.},
            {0., -1., 0.},
        };

        auto transform = Eigen::Isometry3d::Identity();
        transform.linear() = rotation.toRotationMatrix() * optical_to_ros;
        transform.translation() = Eigen::Vector3d{translation[0], translation[1], translation[2]};
        return transform;
    }

    // 读取遥控器左右拨杆：先双下解锁，之后双下或断联复位，左上右下触发初始化
    void handle_remote() {
        if (!switch_left_.ready() || !switch_right_.ready())
            return;

        const auto left = *switch_left_;
        const auto right = *switch_right_;

        if (!remote_unlocked_) {
            if (left == rmcs_msgs::Switch::DOWN && right == rmcs_msgs::Switch::DOWN)
                remote_unlocked_ = true;
            last_switch_left_ = left;
            last_switch_right_ = right;
            return;
        }

        const auto is_reset = (left == rmcs_msgs::Switch::DOWN && right == rmcs_msgs::Switch::DOWN)
                           || left == rmcs_msgs::Switch::UNKNOWN
                           || right == rmcs_msgs::Switch::UNKNOWN;
        const auto was_reset = (last_switch_left_ == rmcs_msgs::Switch::DOWN
                                && last_switch_right_ == rmcs_msgs::Switch::DOWN)
                            || last_switch_left_ == rmcs_msgs::Switch::UNKNOWN
                            || last_switch_right_ == rmcs_msgs::Switch::UNKNOWN;
        if (is_reset) {
            if (!was_reset)
                pending_command_.store(Command::kReset);
            last_switch_left_ = left;
            last_switch_right_ = right;
            return;
        }

        const auto is_start = left == rmcs_msgs::Switch::UP && right == rmcs_msgs::Switch::DOWN;
        const auto was_start = last_switch_left_ == rmcs_msgs::Switch::UP
                            && last_switch_right_ == rmcs_msgs::Switch::DOWN;
        if (is_start && !was_start)
            pending_command_.store(Command::kStart);

        last_switch_left_ = left;
        last_switch_right_ = right;
    }

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

        if (const auto initialized = estimator_.initialize(mesh_path_, config); !initialized) {
            RCLCPP_ERROR(get_logger(), "Target estimator: %s", initialized.error().c_str());
        } else {
            estimator_ready_.store(true);
            RCLCPP_INFO(
                get_logger(), "Target estimator initialized, %dx%d", camera.width, camera.height);
        }
    }

    // 在相机回调线程中写入帧队列
    void push_frame(const camera::Frame& frame, Clock::time_point stamp) {
        const auto lock = std::lock_guard{queue_mutex_};
        if (frame_queue_.size() >= kFrameQueueCapacity) {
            // 初始化完成后，丢弃最旧帧，以确保跟踪的实时性
            if (estimator_status_.load()
                == static_cast<int>(estimation::Initialization::Status::kReady))
                frame_queue_.pop_front();
            // 初始化完成前，丢弃最新帧
            else
                return;
        }
        frame_queue_.push_back(QueuedFrame{frame, stamp});
    }

    // 为工作线程从帧队列头部取出一帧
    std::optional<QueuedFrame> pop_frame() {
        const auto lock = std::lock_guard{queue_mutex_};
        if (frame_queue_.empty())
            return std::nullopt;
        auto frame = std::move(frame_queue_.front());
        frame_queue_.pop_front();
        return frame;
    }

    // 复位时清空帧队列
    void clear_frame_queue() {
        const auto lock = std::lock_guard{queue_mutex_};
        frame_queue_.clear();
    }

    // 在工作线程中循环处理指令、取帧并更新估计器状态
    void work_loop() {
        auto pending_start = false;
        auto last_status = estimation::Initialization::Status::kIdle;
        auto marker_visible = false;
        std::optional<Clock::time_point> last_log_time;
        std::optional<Clock::time_point> last_marker_time;

        while (!should_stop_.load()) {
            const auto command = pending_command_.exchange(Command::kNone);
            if (command == Command::kReset) {
                pending_start = false;
                clear_frame_queue();
                if (estimator_ready_.load())
                    estimator_.reset();
                estimator_status_.store(
                    static_cast<int>(estimation::Initialization::Status::kIdle));
                {
                    const auto lock = std::lock_guard{result_mutex_};
                    latest_estimation_result_ = LatestEstimationResult{};
                }
                if (marker_visible) {
                    delete_marker(this->now());
                    marker_visible = false;
                    last_marker_time.reset();
                }
                last_status = estimation::Initialization::Status::kIdle;
                last_log_time = Clock::now();
                RCLCPP_INFO(get_logger(), "Estimator reset by remote");
                continue;
            }
            if (command == Command::kStart) {
                if (estimator_ready_.load()) {
                    pending_start = true;
                    RCLCPP_INFO(get_logger(), "Estimator start requested by remote");
                } else {
                    RCLCPP_WARN(
                        get_logger(), "Estimator start ignored: waiting for first camera frame");
                }
            }

            const auto queued_frame = pop_frame();
            // 返回空值时，休眠 1ms 后进入下一轮循环
            if (!queued_frame) {
                std::this_thread::sleep_for(std::chrono::milliseconds{1});
                continue;
            }

            if (!estimator_ready_.load())
                continue;

            if (pending_start) {
                estimator_.start(queued_frame->stamp);
                pending_start = false;
            }

            // 取到帧时，传入 estimator_.update() 执行一次估计
            const auto output = estimator_.update(
                queued_frame->frame.bgr, queued_frame->frame.depth, camera_to_base_,
                queued_frame->stamp);
            estimator_status_.store(static_cast<int>(output.status));
            const auto output_pose = output.status == estimation::Initialization::Status::kReady
                                       ? output.reference_pose
                                       : output.pose;

            {
                const auto lock = std::lock_guard{result_mutex_};
                latest_estimation_result_.status = static_cast<int>(output.status);
                latest_estimation_result_.is_pose_valid = output.pose_valid;
                latest_estimation_result_.pose = output_pose;
            }

            const auto status_changed = output.status != last_status;
            const auto has_pose =
                output.status == estimation::Initialization::Status::kReady || output.pose_valid;
            const auto steady_now = Clock::now();

            if (status_changed || !last_log_time || steady_now - *last_log_time >= kLogPeriod) {
                log_estimation(output.status, output.pose_valid, output_pose);
                last_log_time = steady_now;
            }

            if (has_pose) {
                publish_pose(queued_frame->frame.stamp, output_pose);
                if (!marker_visible || status_changed || !last_marker_time
                    || steady_now - *last_marker_time >= kMarkerPeriod) {
                    publish_marker(queued_frame->frame.stamp, output.status, output_pose);
                    marker_visible = true;
                    last_marker_time = steady_now;
                }
            } else if (marker_visible) {
                delete_marker(queued_frame->frame.stamp);
                marker_visible = false;
                last_marker_time.reset();
            }

            last_status = output.status;
        }
    }

    // 相机订阅回调函数
    void on_frame(const camera::Frame& frame) {
        const auto stamp = to_estimator_time(frame.stamp);

        if (!estimator_initialized_)
            initialize_estimator(frame.camera);

        push_frame(frame, stamp);

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

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
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
    std::atomic<bool> estimator_ready_{false};

    // RViz 位姿与网格话题使用的机械臂底座坐标系名称
    static constexpr const char* kBaseFrame = "base_link";
    // 帧队列长度，默认为 30 帧
    static constexpr std::size_t kFrameQueueCapacity = 30;
    // 终端日志的周期输出间隔
    static constexpr auto kLogPeriod = std::chrono::seconds{1};
    // 网格标记在状态未变化时的刷新间隔
    static constexpr auto kMarkerPeriod = std::chrono::milliseconds{200};

    // 最近估计结果
    LatestEstimationResult latest_estimation_result_{};
    std::mutex result_mutex_;

    // 使用 std::deque 维护帧队列
    std::deque<QueuedFrame> frame_queue_;
    std::mutex queue_mutex_;

    // 估计器状态的最新副本
    std::atomic<int> estimator_status_{static_cast<int>(estimation::Initialization::Status::kIdle)};

    // 相机坐标系到基座坐标系的变换
    Eigen::Isometry3d camera_to_base_ = Eigen::Isometry3d::Identity();

    std::atomic<Command> pending_command_{Command::kNone};
    bool remote_unlocked_ = false;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    std::thread worker_;
    std::atomic<bool> should_stop_{false};

    std::optional<rclcpp::Time> first_ros_stamp_;
    Clock::time_point first_estimator_stamp_{};

    visualization_msgs::msg::Marker unit_marker_template_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::unique_ptr<camera::CameraReceiver> camera_receiver_;
};

} // namespace rmcs_energy_trace_v2

PLUGINLIB_EXPORT_CLASS(rmcs_energy_trace_v2::EnergyTraceComponent, rmcs_executor::Component)
