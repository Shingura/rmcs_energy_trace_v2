#include "rviz_replay.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>

#include <gtest/gtest.h>

namespace rmcs_energy_trace_v2::simulation {
namespace {

class PickupRvizReplayTest : public testing::Test {
protected:
    void SetUp() override {
        const auto root = std::filesystem::path{RMCS_RVIZ_REPLAY_SOURCE_DIR};
        dimensions = load_hub_dimensions((root / "simulation/scene.json").string());
        auto file = std::ifstream{root.parent_path() / "arm_description/urdf/arm_description.urdf"};
        ASSERT_TRUE(file);
        urdf = std::string{std::istreambuf_iterator<char>{file}, std::istreambuf_iterator<char>{}};
        plan.hub.translation() = Eigen::Vector3d{.2, .1, .7};
        plan.base.translation() = Eigen::Vector3d{.3, -.5, .4};
        plan.slots = {true, true, false, false, false, false};
        plan.target = 1;
        plan.camera_link = "link_3";
        plan.camera.translation() = Eigen::Vector3d{.13, .03, -.1};
        plan.frames = {
            {{0., -.1, -.5, 0., 1.2, 0.}, "hold", false},
            {{.1, -.1, -.5, .2, 1.2, 0.}, "approach", false},
            {{.1, -.1, -.5, .2, 1.2, 0.}, "grasp", true},
            {{.3, -.2, -.6, .2, 1.1, 0.}, "extract", true},
        };
    }

    PickupPlan plan;
    HubDimensions dimensions;
    std::string urdf;
};

TEST_F(PickupRvizReplayTest, UnitStartsOnTheSelectedHubSlot) {
    const auto playback = PickupRvizReplay{plan, urdf, dimensions};
    const Eigen::Vector3d expected = plan.hub.translation() + Eigen::Vector3d{0., 0., .15};
    EXPECT_TRUE(playback.unit_pose(1).translation().isApprox(expected, 1e-10));
    EXPECT_TRUE(playback.at(0).target.matrix().isApprox(playback.at(1).target.matrix(), 1e-12));
}

TEST_F(PickupRvizReplayTest, AttachmentIsContinuousAndRigid) {
    const auto playback = PickupRvizReplay{plan, urdf, dimensions};
    const auto grasp = playback.at(2);
    const auto extract = playback.at(3);
    EXPECT_TRUE(grasp.target.matrix().isApprox(playback.unit_pose(plan.target).matrix(), 1e-12));
    EXPECT_TRUE((grasp.gripper.inverse() * grasp.target)
                    .matrix()
                    .isApprox((extract.gripper.inverse() * extract.target).matrix(), 1e-12));
    EXPECT_GT((extract.target.translation() - grasp.target.translation()).norm(), .01);
}

TEST_F(PickupRvizReplayTest, RandomAccessAndLoopingDoNotChangeTheResult) {
    const auto playback = PickupRvizReplay{plan, urdf, dimensions};
    const auto end = playback.at(3);
    static_cast<void>(playback.at(0));
    EXPECT_TRUE(end.target.matrix().isApprox(playback.at(3).target.matrix(), 1e-12));
}

TEST_F(PickupRvizReplayTest, BaseAndHubTransformsAreAppliedInTheWorldFrame) {
    const auto original = PickupRvizReplay{plan, urdf, dimensions};
    auto moved_plan = plan;
    auto shift = Eigen::Isometry3d::Identity();
    shift.linear() = Eigen::AngleAxisd{.6, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
    shift.translation() = Eigen::Vector3d{1., 2., .1};
    moved_plan.base = shift * plan.base;
    moved_plan.hub = shift * plan.hub;
    const auto moved = PickupRvizReplay{moved_plan, urdf, dimensions};
    for (std::size_t i = 0; i < plan.frames.size(); ++i) {
        EXPECT_TRUE(
            moved.at(i).target.matrix().isApprox((shift * original.at(i).target).matrix(), 1e-12));
        EXPECT_TRUE(
            moved.at(i).camera.matrix().isApprox((shift * original.at(i).camera).matrix(), 1e-12));
    }
}

TEST_F(PickupRvizReplayTest, CameraFollowsItsConfiguredLink) {
    const auto original = PickupRvizReplay{plan, urdf, dimensions};
    auto moved_plan = plan;
    for (auto& frame : moved_plan.frames)
        frame.joints[5] += .5;
    const auto moved = PickupRvizReplay{moved_plan, urdf, dimensions};
    EXPECT_TRUE(moved.at(3).camera.matrix().isApprox(original.at(3).camera.matrix(), 1e-12));
    EXPECT_FALSE(moved.at(3).gripper.matrix().isApprox(original.at(3).gripper.matrix(), 1e-5));
}

TEST_F(PickupRvizReplayTest, MissingRequiredLinkIsReported) {
    plan.camera_link = "missing_link";
    EXPECT_THROW((PickupRvizReplay{plan, urdf, dimensions}), std::runtime_error);
}

TEST(PickupPlan, MissingFileIsReported) {
    EXPECT_THROW(load_pickup_plan("/nonexistent/rmcs-pickup-plan.json"), std::runtime_error);
}

} // namespace
} // namespace rmcs_energy_trace_v2::simulation
