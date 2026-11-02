#include <chrono>
#include <numbers>
#include <optional>
#include <thread>

#include <eigen3/Eigen/Geometry>
#include <gtest/gtest.h>

#include "estimation/initialization.hpp"

using rmcs_energy_trace_v2::estimation::Initialization;

namespace {

constexpr double kRadToDeg = 180. / std::numbers::pi;

using Pose = Eigen::Isometry3d;
using Status = Initialization::Status;

// 中心在 (x, 0, 0.5)，长轴（z 轴）绕 y 转 axis_deg
auto make_pose(double x, double axis_deg) -> Pose {
    auto pose = Pose::Identity();
    pose.translation() = Eigen::Vector3d{x, 0., 0.5};
    pose.linear() =
        Eigen::AngleAxisd{axis_deg / kRadToDeg, Eigen::Vector3d::UnitY()}.toRotationMatrix();
    return pose;
}

const Pose kReference = make_pose(0., 0.);

// 配准通过的那一帧：给出参考位姿，还没有跟踪结果
auto feed_reference(Initialization& init) -> Status {
    return init.update(kReference, std::nullopt);
}

auto advance(Initialization& init, int frames, const std::optional<Pose>& tracked) -> Status {
    auto status = init.status();
    for (int i = 0; i < frames; ++i)
        status = init.update(std::nullopt, tracked);
    return status;
}

auto reach_ready(Initialization& init) -> Status {
    feed_reference(init);
    return advance(init, 15, kReference);
}

} // namespace

TEST(Initialization, IdleIgnoresFrames) {
    auto init = Initialization{};
    EXPECT_EQ(init.status(), Status::kIdle);
    EXPECT_EQ(init.update(kReference, kReference), Status::kIdle);
    EXPECT_FALSE(init.reference_pose().has_value());
}

TEST(Initialization, RegistrationSetsReferenceAndKeepsInitializing) {
    auto init = Initialization{};
    init.start();
    EXPECT_EQ(init.status(), Status::kInitializing);
    EXPECT_EQ(feed_reference(init), Status::kInitializing);
    ASSERT_TRUE(init.reference_pose().has_value());
    EXPECT_TRUE(init.reference_pose()->isApprox(kReference));
}

TEST(Initialization, ConfirmationWindowReachesReady) {
    auto init = Initialization{};
    init.start();
    EXPECT_EQ(reach_ready(init), Status::kReady);
    EXPECT_TRUE(init.reference_pose()->isApprox(kReference));
}

TEST(Initialization, CenterDeviationBeyondThresholdRestartsWindow) {
    auto init = Initialization{};
    init.start();
    feed_reference(init);
    EXPECT_EQ(advance(init, 10, kReference), Status::kInitializing);
    // 中心偏 6 mm，超过 5 mm 阈值
    EXPECT_EQ(init.update(std::nullopt, make_pose(0.006, 0.)), Status::kInitializing);
    EXPECT_EQ(advance(init, 14, kReference), Status::kInitializing);
    EXPECT_EQ(advance(init, 1, kReference), Status::kReady);
}

TEST(Initialization, AxisDeviationBeyondThresholdRestartsWindow) {
    auto init = Initialization{};
    init.start();
    feed_reference(init);
    EXPECT_EQ(advance(init, 10, kReference), Status::kInitializing);
    // 长轴偏 7°，超过 6° 阈值
    EXPECT_EQ(init.update(std::nullopt, make_pose(0., 7.)), Status::kInitializing);
    EXPECT_EQ(advance(init, 14, kReference), Status::kInitializing);
    EXPECT_EQ(advance(init, 1, kReference), Status::kReady);
}

TEST(Initialization, DeviationWithinThresholdIsAccepted) {
    auto init = Initialization{};
    init.start();
    feed_reference(init);
    // 中心偏 4 mm、长轴偏 5°，都在阈值内
    EXPECT_EQ(advance(init, 15, make_pose(0.004, 5.)), Status::kReady);
}

TEST(Initialization, LaterRegistrationDoesNotReplaceReference) {
    auto init = Initialization{};
    init.start();
    feed_reference(init);
    EXPECT_EQ(init.update(make_pose(0.05, 0.), std::nullopt), Status::kInitializing);
    EXPECT_TRUE(init.reference_pose()->isApprox(kReference));
}

TEST(Initialization, TimeoutWithoutRegistrationFails) {
    auto init = Initialization{Initialization::Config{.timeout_s = 0.02}};
    init.start();
    EXPECT_EQ(advance(init, 10, std::nullopt), Status::kInitializing);
    std::this_thread::sleep_for(std::chrono::milliseconds{30});
    EXPECT_EQ(init.update(std::nullopt, std::nullopt), Status::kFailed);
    EXPECT_FALSE(init.reference_pose().has_value());
}

TEST(Initialization, TimeoutDuringConfirmationFails) {
    auto init = Initialization{Initialization::Config{.timeout_s = 0.02}};
    init.start();
    feed_reference(init);
    EXPECT_EQ(advance(init, 10, kReference), Status::kInitializing);
    std::this_thread::sleep_for(std::chrono::milliseconds{30});
    EXPECT_EQ(init.update(std::nullopt, kReference), Status::kFailed);
}

TEST(Initialization, ReadyIsTerminal) {
    auto init = Initialization{};
    init.start();
    EXPECT_EQ(reach_ready(init), Status::kReady);
    EXPECT_EQ(init.update(kReference, make_pose(0.05, 20.)), Status::kReady);
    EXPECT_TRUE(init.reference_pose()->isApprox(kReference));
}

TEST(Initialization, FailedIsTerminalUntilReset) {
    auto init = Initialization{Initialization::Config{.timeout_s = 0.02}};
    init.start();
    EXPECT_EQ(init.update(std::nullopt, std::nullopt), Status::kInitializing);
    std::this_thread::sleep_for(std::chrono::milliseconds{30});
    EXPECT_EQ(init.update(std::nullopt, std::nullopt), Status::kFailed);
    EXPECT_EQ(init.update(kReference, kReference), Status::kFailed);

    init.reset();
    EXPECT_EQ(init.status(), Status::kIdle);
    EXPECT_FALSE(init.reference_pose().has_value());
    // 复位后不重新开始就不处理帧
    EXPECT_EQ(feed_reference(init), Status::kIdle);
    EXPECT_FALSE(init.reference_pose().has_value());

    init.start();
    EXPECT_EQ(feed_reference(init), Status::kInitializing);
    EXPECT_TRUE(init.reference_pose()->isApprox(kReference));
}
