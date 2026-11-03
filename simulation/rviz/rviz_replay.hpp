#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <kdl/tree.hpp>

namespace rmcs_energy_trace_v2::simulation {

struct PickupFrame {
    std::array<double, 6> joints{};
    std::string phase;
    bool attached = false;
};

struct PickupPlan {
    double fps = 30.;
    Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d hub = Eigen::Isometry3d::Identity();
    std::array<bool, 6> slots{};
    std::size_t target = 0;
    std::string camera_link;
    Eigen::Isometry3d camera = Eigen::Isometry3d::Identity();
    int width = 0;
    int height = 0;
    double hfov = 0.; // rad
    std::vector<Eigen::Vector3d> bracket;
    std::vector<PickupFrame> frames;
};

struct HubDimensions {
    double apothem = 0.;
    double thickness = 0.;
    double unit_offset = 0.;
};

// 文件或格式无效时抛出 std::runtime_error，由命令行入口报告。
PickupPlan load_pickup_plan(const std::string& path);
HubDimensions load_hub_dimensions(const std::string& path);

// 直接采样任意帧，不依赖前一帧。暂停、重播和循环不会改变夹持时的相对位姿。
class PickupRvizReplay {
public:
    struct State {
        Eigen::Isometry3d gripper;
        Eigen::Isometry3d target;
        Eigen::Isometry3d camera;
    };

    PickupRvizReplay(PickupPlan plan, const std::string& urdf, HubDimensions dimensions);

    const PickupPlan& plan() const { return plan_; }
    const HubDimensions& dimensions() const { return dimensions_; }
    const std::string& root_link() const { return root_link_; }
    Eigen::Isometry3d unit_pose(std::size_t slot) const;
    State at(std::size_t frame) const;

private:
    Eigen::Isometry3d link_pose(const PickupFrame& frame, const std::string& link) const;

    PickupPlan plan_;
    HubDimensions dimensions_;
    KDL::Tree tree_;
    std::string root_link_;
    std::array<unsigned int, 6> joint_indices_{};
    Eigen::Isometry3d gripper_from_target_ = Eigen::Isometry3d::Identity();
};

} // namespace rmcs_energy_trace_v2::simulation
