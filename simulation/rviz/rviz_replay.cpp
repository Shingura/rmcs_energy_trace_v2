#include "rviz_replay.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <opencv2/core.hpp>

namespace rmcs_energy_trace_v2::simulation {
namespace {

void require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error{message};
}

Eigen::Isometry3d read_pose(const cv::FileNode& node) {
    require(node.isSeq() && node.size() == 4, "Pose must contain four rows");
    auto pose = Eigen::Isometry3d::Identity();
    for (int row = 0; row < 4; ++row) {
        require(node[row].isSeq() && node[row].size() == 4, "Pose rows must contain four values");
        for (int col = 0; col < 4; ++col)
            pose.matrix()(row, col) = node[row][col].real();
    }
    require(pose.matrix().allFinite(), "Pose contains a non-finite value");
    return pose;
}

} // namespace

PickupPlan load_pickup_plan(const std::string& path) {
    const auto file = cv::FileStorage{path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON};
    require(file.isOpened(), "Cannot open pickup plan: " + path);
    auto plan = PickupPlan{};
    plan.fps = file["fps"].real();
    require(std::isfinite(plan.fps) && plan.fps > 0., "Plan fps must be positive");
    plan.base = read_pose(file["base"]);
    const auto hub = file["hub"];
    plan.hub = read_pose(hub["pose"]);
    require(hub["slots"].size() == plan.slots.size(), "Plan must contain six hub slots");
    for (std::size_t i = 0; i < plan.slots.size(); ++i)
        plan.slots[i] = hub["slots"][static_cast<int>(i)].real() != 0.;
    const auto target = static_cast<int>(hub["target"].real());
    require(target >= 0 && target < 6, "Target slot must be between 0 and 5");
    plan.target = static_cast<std::size_t>(target);
    require(plan.slots[plan.target], "Target slot is empty");

    const auto camera = file["camera"];
    plan.camera_link = camera["link"].string();
    plan.camera = read_pose(camera["pose"]);
    plan.width = static_cast<int>(camera["width"].real());
    plan.height = static_cast<int>(camera["height"].real());
    plan.hfov = camera["hfov_deg"].real() * std::numbers::pi / 180.;
    require(
        plan.width > 0 && plan.height > 0 && plan.hfov > 0. && plan.hfov < std::numbers::pi,
        "Invalid camera dimensions or field of view");
    for (const auto& point : camera["bracket"]) {
        require(point.size() == 3, "Camera bracket points must contain three values");
        plan.bracket.emplace_back(point[0].real(), point[1].real(), point[2].real());
    }

    for (const auto& node : file["frames"]) {
        auto frame = PickupFrame{};
        require(node["joints"].size() == frame.joints.size(), "Each frame must contain six joints");
        for (std::size_t i = 0; i < frame.joints.size(); ++i) {
            frame.joints[i] = node["joints"][static_cast<int>(i)].real();
            require(std::isfinite(frame.joints[i]), "Joint position is not finite");
        }
        frame.phase = node["phase"].string();
        frame.attached = node["attached"].real() != 0.;
        plan.frames.push_back(std::move(frame));
    }
    require(!plan.frames.empty(), "Plan has no frames");
    return plan;
}

HubDimensions load_hub_dimensions(const std::string& path) {
    const auto file = cv::FileStorage{path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON};
    require(file.isOpened(), "Cannot open scene dimensions: " + path);
    const auto hub = file["hub"];
    auto dimensions =
        HubDimensions{hub["apothem"].real(), hub["thickness"].real(), hub["unit_offset"].real()};
    require(
        dimensions.apothem > 0. && dimensions.thickness > 0. && dimensions.unit_offset > 0.,
        "Hub dimensions must be positive");
    return dimensions;
}

PickupRvizReplay::PickupRvizReplay(
    PickupPlan plan, const std::string& urdf, HubDimensions dimensions)
    : plan_{std::move(plan)}
    , dimensions_{dimensions} {
    require(kdl_parser::treeFromString(urdf, tree_), "Cannot parse arm URDF");
    root_link_ = tree_.getRootSegment()->first;
    require(tree_.getNrOfJoints() == 6, "Arm URDF must contain six movable joints");
    const auto& segments = tree_.getSegments();
    for (std::size_t i = 0; i < joint_indices_.size(); ++i) {
        const auto name = "joint_" + std::to_string(i + 1);
        const auto segment = std::find_if(segments.begin(), segments.end(), [&](const auto& entry) {
            return entry.second.segment.getJoint().getName() == name;
        });
        require(segment != segments.end(), "URDF is missing " + name);
        joint_indices_[i] = segment->second.q_nr;
    }
    require(segments.contains("link_6"), "URDF is missing gripper link_6");
    require(
        segments.contains(plan_.camera_link), "URDF is missing camera link " + plan_.camera_link);
    const auto attached = std::find_if(
        plan_.frames.begin(), plan_.frames.end(), [](const auto& frame) { return frame.attached; });
    if (attached != plan_.frames.end())
        gripper_from_target_ =
            (plan_.base * link_pose(*attached, "link_6")).inverse() * unit_pose(plan_.target);
}

Eigen::Isometry3d
    PickupRvizReplay::link_pose(const PickupFrame& frame, const std::string& link) const {
    auto joints = KDL::JntArray{tree_.getNrOfJoints()};
    for (std::size_t i = 0; i < joint_indices_.size(); ++i)
        joints(joint_indices_[i]) = frame.joints[i];
    auto solver = KDL::TreeFkSolverPos_recursive{tree_};
    auto result = KDL::Frame{};
    require(solver.JntToCart(joints, result, link) >= 0, "Cannot compute pose of " + link);
    auto pose = Eigen::Isometry3d::Identity();
    for (int row = 0; row < 3; ++row) {
        pose.translation()(row) = result.p(row);
        for (int col = 0; col < 3; ++col)
            pose.linear()(row, col) = result.M(row, col);
    }
    return pose;
}

Eigen::Isometry3d PickupRvizReplay::unit_pose(std::size_t slot) const {
    const auto angle = (30. + 60. * static_cast<double>(slot)) * std::numbers::pi / 180.;
    const auto axis = Eigen::Vector3d{std::cos(angle), 0., std::sin(angle)};
    const Eigen::Vector3d x = Eigen::Vector3d::UnitY().cross(axis).normalized();
    auto local = Eigen::Isometry3d::Identity();
    local.linear().col(0) = x;
    local.linear().col(1) = axis.cross(x);
    local.linear().col(2) = axis;
    local.translation() = dimensions_.unit_offset * axis;
    return plan_.hub * local;
}

PickupRvizReplay::State PickupRvizReplay::at(std::size_t frame) const {
    const auto& step = plan_.frames.at(frame);
    const Eigen::Isometry3d gripper = plan_.base * link_pose(step, "link_6");
    return {
        gripper, step.attached ? gripper * gripper_from_target_ : unit_pose(plan_.target),
        plan_.base * link_pose(step, plan_.camera_link) * plan_.camera};
}

} // namespace rmcs_energy_trace_v2::simulation
