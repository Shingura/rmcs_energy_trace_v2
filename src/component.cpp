#include <string>

#include <eigen3/Eigen/Geometry>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "estimation/target_estimator.hpp"
#include "geometry/camera.hpp"

namespace rmcs_energy_trace_v2 {

// 能量单元位姿估计组件：开环取矿。遥控器双上开始初始化，双下强制停机。
// 相机图像、图像时间戳和相机到底座的位姿都还没有接入，所以估计器不逐帧推进，
// 输出保持上一次的结果。三者就位后 update 里改成按帧调用即可。
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

        // 遥控器不是必须的：没有接遥控器时，指令由上层组件给出
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);

        mesh_path_ = get_parameter_or<std::string>("mesh_path", "");
        // 相机参数没有默认值：缺少时构造失败，避免用错误的内参静默运行
        camera_ = geometry::Camera{
            .width = static_cast<int>(get_parameter("camera.width").as_int()),
            .height = static_cast<int>(get_parameter("camera.height").as_int()),
            .fx = get_parameter("camera.fx").as_double(),
            .fy = get_parameter("camera.fy").as_double(),
            .cx = get_parameter("camera.cx").as_double(),
            .cy = get_parameter("camera.cy").as_double(),
            .depth_scale = get_parameter("camera.depth_scale").as_double(),
        };

        auto config = estimation::TargetEstimator::Config{};
        config.camera = camera_;
        // 实机上还有机械臂控制等组件，视觉不占满所有核
        config.threads = get_parameter_or<int>("threads", 4);
        config.detector.network_path = get_parameter_or<std::string>("network_path", "");
        config.tracker.model_dir = get_parameter_or<std::string>("model_dir", "");

        if (const auto initialized = estimator_.initialize(mesh_path_, config)) {
            estimator_ready_ = true;
        } else {
            RCLCPP_ERROR(get_logger(), "%s", initialized.error().c_str());
        }
        RCLCPP_INFO(
            get_logger(), "energy trace component created, mesh_path='%s'", mesh_path_.c_str());
    }

    void update() override {
        handle_remote();

        // 相机尚未接入：没有图像就不推进估计器。接入后改为
        //     output_ = estimator_.update(color, depth, camera_to_base, stamp);
        *status_ = static_cast<int>(output_.status);
        *is_pose_valid_ = output_.pose_valid;
        *pose_ = output_.pose;
    }

private:
    void handle_remote() {
        if (!switch_left_.ready() || !switch_right_.ready())
            return;
        const auto left = *switch_left_;
        const auto right = *switch_right_;
        const auto both_up = left == rmcs_msgs::Switch::UP && right == rmcs_msgs::Switch::UP;
        const auto both_down = left == rmcs_msgs::Switch::DOWN && right == rmcs_msgs::Switch::DOWN;

        // 只在开关拨到位的那一帧动作，避免每帧重复触发
        if (estimator_ready_) {
            if (both_up && !both_up_last_)
                estimator_.start();
            else if (both_down && !both_down_last_)
                estimator_.reset();
        }
        both_up_last_ = both_up;
        both_down_last_ = both_down;
    }

    // 等待 0、初始化中 1、已定下参考位姿 2、初始化失败 3
    OutputInterface<int> status_;
    OutputInterface<bool> is_pose_valid_;
    OutputInterface<Eigen::Isometry3d> pose_;

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    std::string mesh_path_;
    geometry::Camera camera_;
    estimation::TargetEstimator estimator_;
    estimation::TargetEstimator::Output output_{};

    bool estimator_ready_ = false;
    bool both_up_last_ = false;
    bool both_down_last_ = false;
};

} // namespace rmcs_energy_trace_v2

PLUGINLIB_EXPORT_CLASS(rmcs_energy_trace_v2::EnergyTraceComponent, rmcs_executor::Component)
