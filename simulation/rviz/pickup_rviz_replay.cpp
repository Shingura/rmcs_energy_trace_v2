#include "rviz_replay.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "geometry/mesh.hpp"

namespace rmcs_energy_trace_v2::simulation {
namespace {

using Marker = visualization_msgs::msg::Marker;
using Markers = visualization_msgs::msg::MarkerArray;
using Color = std_msgs::msg::ColorRGBA;

Color color(float r, float g, float b, float a = 1.f) {
    return Color{}.set__r(r).set__g(g).set__b(b).set__a(a);
}

geometry_msgs::msg::Point point(const Eigen::Vector3d& value) {
    return geometry_msgs::msg::Point{}.set__x(value.x()).set__y(value.y()).set__z(value.z());
}

geometry_msgs::msg::Pose pose(const Eigen::Isometry3d& transform) {
    auto result = geometry_msgs::msg::Pose{};
    result.position = point(transform.translation());
    const auto q = Eigen::Quaterniond{transform.linear()};
    result.orientation.set__x(q.x()).set__y(q.y()).set__z(q.z()).set__w(q.w());
    return result;
}

Marker marker(const std::string& name, int id, int type, const Color& tint) {
    auto result = Marker{};
    result.header.frame_id = "pickup_world";
    result.ns = name;
    result.id = id;
    result.type = type;
    result.action = Marker::ADD;
    result.pose.orientation.w = 1.;
    result.scale.set__x(1.).set__y(1.).set__z(1.);
    result.color = tint;
    return result;
}

Marker
    box(const std::string& name, const Eigen::Vector3d& center, const Eigen::Vector3d& size,
        const Color& tint) {
    auto result = marker(name, 0, Marker::CUBE, tint);
    result.pose.position = point(center);
    result.scale.set__x(size.x()).set__y(size.y()).set__z(size.z());
    return result;
}

geometry_msgs::msg::TransformStamped transform(
    const std::string& parent, const std::string& child, const Eigen::Isometry3d& value,
    const rclcpp::Time& stamp) {
    auto message = geometry_msgs::msg::TransformStamped{};
    message.header.frame_id = parent;
    message.header.stamp = stamp;
    message.child_frame_id = child;
    const auto p = pose(value);
    message.transform.translation.set__x(p.position.x).set__y(p.position.y).set__z(p.position.z);
    message.transform.rotation = p.orientation;
    return message;
}

} // namespace

class PickupRvizReplayNode final : public rclcpp::Node {
public:
    PickupRvizReplayNode()
        : Node("pickup_rviz_replay") {
        const auto share = ament_index_cpp::get_package_share_directory("rmcs_energy_trace_v2");
        const auto plan_path = declare_parameter<std::string>("plan_path", "");
        const auto description = declare_parameter<std::string>("robot_description", "");
        const auto scene_path = declare_parameter("scene_path", share + "/simulation/scene.json");
        const auto mesh_path = declare_parameter("mesh_path", share + "/meshes/energy_unit.stl");
        speed_ = declare_parameter("speed", 1.);
        loop_ = declare_parameter("loop", true);
        playing_ = declare_parameter("autoplay", true);
        if (plan_path.empty() || description.empty())
            throw std::runtime_error{"plan_path and robot_description are required"};
        if (!std::isfinite(speed_) || speed_ <= 0.)
            throw std::runtime_error{"speed must be positive"};
        playback_ = std::make_unique<PickupRvizReplay>(
            load_pickup_plan(plan_path), description, load_hub_dimensions(scene_path));
        const auto mesh = geometry::load_binary_stl(mesh_path);
        if (!mesh)
            throw std::runtime_error{mesh.error()};
        build_scene(*mesh);

        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        marker_pub_ = create_publisher<Markers>("scene", rclcpp::QoS{1}.transient_local());
        static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        static_tf_->sendTransform(
            std::vector<geometry_msgs::msg::TransformStamped>{
                transform(
                    "pickup_world", "pickup/" + playback_->root_link(), playback_->plan().base,
                    now()),
                transform(
                    "pickup/" + playback_->plan().camera_link, "pickup/camera_optical",
                    playback_->plan().camera, now())});

        pause_service_ = create_service<std_srvs::srv::SetBool>(
            "set_playing", [this](
                               const std_srvs::srv::SetBool::Request::SharedPtr& request,
                               const std_srvs::srv::SetBool::Response::SharedPtr& response) {
                playing_ = request->data;
                response->success = true;
                response->message = playing_ ? "Playback started" : "Playback paused";
            });
        restart_service_ = create_service<std_srvs::srv::Trigger>(
            "restart", [this](
                           const std_srvs::srv::Trigger::Request::SharedPtr&,
                           const std_srvs::srv::Trigger::Response::SharedPtr& response) {
                frame_ = 0;
                publish_frame();
                response->success = true;
                response->message = "Playback reset to frame 0";
            });
        const auto period = std::chrono::duration<double>{1. / (playback_->plan().fps * speed_)};
        // 定时器每次推进一帧，关节角、目标单元和阶段标注使用同一个帧号。
        timer_ =
            create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] {
                publish_frame();
                if (playing_) {
                    if (frame_ + 1 < playback_->plan().frames.size())
                        ++frame_;
                    else if (loop_)
                        frame_ = 0;
                    else
                        playing_ = false;
                }
            });
        RCLCPP_INFO(
            get_logger(), "Loaded %zu frames at %.1f fps, target slot %zu; visualization only",
            playback_->plan().frames.size(), playback_->plan().fps, playback_->plan().target);
    }

private:
    void build_scene(const geometry::Mesh& mesh) {
        const auto& plan = playback_->plan();
        const auto& dimensions = playback_->dimensions();
        auto& markers = scene_.markers;
        markers.push_back(box("ground", {0., 0., -0.025}, {4., 4., 0.05}, color(.28f, .30f, .32f)));
        // 简化支架仅表示中心块离地位置，不参与规划或碰撞判断。
        markers.push_back(box(
            "hub_support", plan.hub * Eigen::Vector3d{0., .10, -plan.hub.translation().z() / 2.},
            {.045, .045, plan.hub.translation().z()}, color(.42f, .44f, .46f)));

        auto hub = marker("hub", 0, Marker::TRIANGLE_LIST, color(.12f, .14f, .17f));
        hub.pose = pose(plan.hub);
        const auto radius = dimensions.apothem / std::cos(std::numbers::pi / 6.);
        const auto half = dimensions.thickness / 2.;
        auto vertex = [&](int k, double y) {
            const auto angle = static_cast<double>(k) * std::numbers::pi / 3.;
            return Eigen::Vector3d{radius * std::cos(angle), y, radius * std::sin(angle)};
        };
        auto triangle = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                            const Eigen::Vector3d& c) {
            hub.points.push_back(point(a));
            hub.points.push_back(point(b));
            hub.points.push_back(point(c));
        };
        for (int k = 0; k < 6; ++k) {
            triangle({0., -half, 0.}, vertex(k, -half), vertex(k + 1, -half));
            triangle({0., half, 0.}, vertex(k + 1, half), vertex(k, half));
            triangle(vertex(k, -half), vertex(k, half), vertex(k + 1, half));
            triangle(vertex(k, -half), vertex(k + 1, half), vertex(k + 1, -half));
        }
        markers.push_back(std::move(hub));

        for (std::size_t slot = 0; slot < plan.slots.size(); ++slot) {
            auto pillar =
                marker("pillars", static_cast<int>(slot), Marker::CYLINDER, color(.6f, .62f, .65f));
            auto pillar_pose = playback_->unit_pose(slot);
            // 与 Blender 相同：柱中心距中心块中心 130 mm，长度 80 mm，直径 34 mm。
            pillar_pose.translation() -=
                pillar_pose.linear().col(2) * (dimensions.unit_offset - .13);
            pillar.pose = pose(pillar_pose);
            pillar.scale.set__x(.034).set__y(.034).set__z(.08);
            markers.push_back(std::move(pillar));
            if (!plan.slots[slot])
                continue;
            auto unit = marker(
                "units", static_cast<int>(slot), Marker::TRIANGLE_LIST,
                slot == plan.target ? color(.95f, .62f, .12f) : color(.25f, .30f, .36f));
            unit.pose = pose(playback_->unit_pose(slot));
            for (const auto& face : mesh.triangles)
                for (const auto& v : face)
                    unit.points.push_back(point(v.cast<double>()));
            if (slot == plan.target)
                target_marker_ = markers.size();
            markers.push_back(std::move(unit));
        }

        auto bracket = marker("camera_bracket", 0, Marker::LINE_STRIP, color(.16f, .18f, .20f));
        bracket.header.frame_id = "pickup/" + plan.camera_link;
        bracket.scale.x = .02;
        for (const auto& p : plan.bracket)
            bracket.points.push_back(point(p));
        if (!bracket.points.empty())
            markers.push_back(std::move(bracket));

        auto camera = marker("camera", 0, Marker::CUBE, color(.25f, .45f, .75f));
        camera.header.frame_id = "pickup/camera_optical";
        camera.pose.position.z = -.015;
        camera.scale.set__x(.09).set__y(.025).set__z(.025);
        markers.push_back(std::move(camera));
        auto frustum = marker("camera_frustum", 0, Marker::LINE_LIST, color(.25f, .65f, 1.f, .6f));
        frustum.header.frame_id = "pickup/camera_optical";
        frustum.scale.x = .002;
        const auto x = .35 * std::tan(plan.hfov / 2.);
        const auto y = x * static_cast<double>(plan.height) / static_cast<double>(plan.width);
        const auto corners = std::array<Eigen::Vector3d, 4>{
            Eigen::Vector3d{-x, -y, .35}, {x, -y, .35}, {x, y, .35}, {-x, y, .35}};
        for (std::size_t i = 0; i < corners.size(); ++i) {
            frustum.points.push_back(point(Eigen::Vector3d::Zero()));
            frustum.points.push_back(point(corners[i]));
            frustum.points.push_back(point(corners[i]));
            frustum.points.push_back(point(corners[(i + 1) % corners.size()]));
        }
        markers.push_back(std::move(frustum));
        text_marker_ = markers.size();
        auto text = marker("phase", 0, Marker::TEXT_VIEW_FACING, color(1.f, 1.f, 1.f));
        text.pose.position = point(plan.hub.translation() + Eigen::Vector3d{0., 0., .40});
        text.scale.set__x(0.).set__y(0.).set__z(.035);
        markers.push_back(std::move(text));
    }

    void publish_frame() {
        const auto stamp = now();
        const auto& plan = playback_->plan();
        const auto& step = plan.frames[frame_];
        auto joints = sensor_msgs::msg::JointState{};
        joints.header.stamp = stamp;
        for (std::size_t i = 0; i < step.joints.size(); ++i) {
            joints.name.push_back("joint_" + std::to_string(i + 1));
            joints.position.push_back(step.joints[i]);
        }
        joint_pub_->publish(joints);
        scene_.markers[target_marker_].pose = pose(playback_->at(frame_).target);
        scene_.markers[text_marker_].text = step.phase + " | " + std::to_string(frame_) + "/"
                                          + std::to_string(plan.frames.size() - 1)
                                          + (playing_ ? "" : " | paused");
        // 零时间戳让 RViz 取最新 TF，不要求 Marker 与 robot_state_publisher 的消息同时到达。
        marker_pub_->publish(scene_);
    }

    std::unique_ptr<PickupRvizReplay> playback_;
    Markers scene_;
    std::size_t target_marker_ = 0;
    std::size_t text_marker_ = 0;
    std::size_t frame_ = 0;
    double speed_ = 1.;
    bool loop_ = true;
    bool playing_ = true;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<Markers>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_service_;
};

} // namespace rmcs_energy_trace_v2::simulation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto code = 0;
    try {
        rclcpp::spin(std::make_shared<rmcs_energy_trace_v2::simulation::PickupRvizReplayNode>());
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("pickup_rviz_replay"), "%s", error.what());
        code = 1;
    }
    rclcpp::shutdown();
    return code;
}
