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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <eigen3/Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "camera/camera_receiver.hpp"
#include "detection/detector.hpp"
#include "estimation/initialization.hpp"
#include "estimation/target_estimator.hpp"

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

        auto config = detection::Detector::Config{};
        config.network_path = network_path_;
        config.infer_threads = threads_;

        if (const auto initialized = detector_.initialize(config); !initialized)
            throw std::runtime_error{initialized.error()};

        detection_image_pub_ =
            create_publisher<sensor_msgs::msg::Image>("/energy_trace/detection_image", 1);
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
                continue;
            }
            if (command == Command::kStart && estimator_ready_.load())
                pending_start = true;

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

            {
                const auto lock = std::lock_guard{result_mutex_};
                latest_estimation_result_.status = static_cast<int>(output.status);
                latest_estimation_result_.is_pose_valid = output.pose_valid;
                latest_estimation_result_.pose =
                    output.status == estimation::Initialization::Status::kReady
                        ? output.reference_pose
                        : output.pose;
            }
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

    // 帧队列长度，默认为 30 帧
    static constexpr std::size_t kFrameQueueCapacity = 30;

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

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_image_pub_;
    std::unique_ptr<camera::CameraReceiver> camera_receiver_;
};

} // namespace rmcs_energy_trace_v2

PLUGINLIB_EXPORT_CLASS(rmcs_energy_trace_v2::EnergyTraceComponent, rmcs_executor::Component)
