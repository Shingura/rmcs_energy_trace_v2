#include <chrono>
#include <numbers>
#include <optional>

#include <eigen3/Eigen/Geometry>
#include <gtest/gtest.h>

#include "estimation/initialization.hpp"

using rmcs_energy_trace_v2::estimation::Initialization;

namespace {

constexpr double kRadToDeg = 180. / std::numbers::pi;

using Pose = Eigen::Isometry3d;
using Status = Initialization::Status;
using Clock = Initialization::Clock;

// 30 fps 下的一帧，用于推进时间
constexpr auto kFrameTime = std::chrono::milliseconds{33};
const Clock::time_point kBegin = Clock::now();

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
auto feed_reference(Initialization& init, Clock::time_point now) -> Status {
    return init.update(kReference, std::nullopt, now);
}

auto advance(
    Initialization& init, int frames, const std::optional<Pose>& tracked, Clock::time_point now)
    -> Status {
    auto status = init.status();
    for (int i = 0; i < frames; ++i)
        status = init.update(std::nullopt, tracked, now + i * kFrameTime);
    return status;
}

auto reach_ready(Initialization& init, Clock::time_point now) -> Status {
    feed_reference(init, now);
    return advance(init, 15, kReference, now + kFrameTime);
}

} // namespace

TEST(Initialization, IdleIgnoresFrames) {
    auto init = Initialization{};
    EXPECT_EQ(init.status(), Status::kIdle);
    EXPECT_EQ(init.update(kReference, kReference, kBegin), Status::kIdle);
    EXPECT_FALSE(init.reference_pose().has_value());
}

TEST(Initialization, RegistrationSetsReferenceAndKeepsInitializing) {
    auto init = Initialization{};
    init.start(kBegin);
    EXPECT_EQ(init.status(), Status::kInitializing);
    EXPECT_EQ(feed_reference(init, kBegin), Status::kInitializing);
    ASSERT_TRUE(init.reference_pose().has_value());
    EXPECT_TRUE(init.reference_pose()->isApprox(kReference));
}

TEST(Initialization, ConfirmationWindowReachesReady) {
    auto init = Initialization{};
    init.start(kBegin);
    EXPECT_EQ(reach_ready(init, kBegin), Status::kReady);
    EXPECT_TRUE(init.reference_pose()->isApprox(kReference));
}

TEST(Initialization, CenterDeviationBeyondThresholdRestartsWindow) {
    auto init = Initialization{};
    init.start(kBegin);
    feed_reference(init, kBegin);
    EXPECT_EQ(advance(init, 10, kReference, kBegin + kFrameTime), Status::kInitializing);
    // 中心偏 6 mm，超过 5 mm 阈值
    EXPECT_EQ(
        init.update(std::nullopt, make_pose(0.006, 0.), kBegin + 11 * kFrameTime),
        Status::kInitializing);
    EXPECT_EQ(advance(init, 14, kReference, kBegin + 12 * kFrameTime), Status::kInitializing);
    EXPECT_EQ(init.update(std::nullopt, kReference, kBegin + 26 * kFrameTime), Status::kReady);
}

TEST(Initialization, AxisDeviationBeyondThresholdRestartsWindow) {
    auto init = Initialization{};
    init.start(kBegin);
    feed_reference(init, kBegin);
    EXPECT_EQ(advance(init, 10, kReference, kBegin + kFrameTime), Status::kInitializing);
    // 长轴偏 7°，超过 6° 阈值
    EXPECT_EQ(
        init.update(std::nullopt, make_pose(0., 7.), kBegin + 11 * kFrameTime),
        Status::kInitializing);
    EXPECT_EQ(advance(init, 14, kReference, kBegin + 12 * kFrameTime), Status::kInitializing);
    EXPECT_EQ(init.update(std::nullopt, kReference, kBegin + 26 * kFrameTime), Status::kReady);
}

TEST(Initialization, DeviationWithinThresholdIsAccepted) {
    auto init = Initialization{};
    init.start(kBegin);
    feed_reference(init, kBegin);
    // 中心偏 4 mm、长轴偏 5°，都在阈值内
    EXPECT_EQ(advance(init, 15, make_pose(0.004, 5.), kBegin + kFrameTime), Status::kReady);
}

TEST(Initialization, LaterRegistrationDoesNotReplaceReference) {
    auto init = Initialization{};
    init.start(kBegin);
    feed_reference(init, kBegin);
    EXPECT_EQ(
        init.update(make_pose(0.05, 0.), std::nullopt, kBegin + kFrameTime), Status::kInitializing);
    EXPECT_TRUE(init.reference_pose()->isApprox(kReference));
}

TEST(Initialization, TimeoutWithoutRegistrationFails) {
    auto init = Initialization{Initialization::Config{.timeout_s = 0.1}};
    init.start(kBegin);
    // 0、33、66 ms：都在 100 ms 的时限内
    EXPECT_EQ(advance(init, 3, std::nullopt, kBegin), Status::kInitializing);
    EXPECT_EQ(
        init.update(std::nullopt, std::nullopt, kBegin + std::chrono::milliseconds{200}),
        Status::kFailed);
    EXPECT_FALSE(init.reference_pose().has_value());
}

TEST(Initialization, TimeoutDuringConfirmationFails) {
    auto init = Initialization{Initialization::Config{.timeout_s = 0.1}};
    init.start(kBegin);
    feed_reference(init, kBegin);
    // 33、66 ms：都在 100 ms 的时限内，确认窗口还没有满
    EXPECT_EQ(advance(init, 2, kReference, kBegin + kFrameTime), Status::kInitializing);
    EXPECT_EQ(
        init.update(std::nullopt, kReference, kBegin + std::chrono::milliseconds{200}),
        Status::kFailed);
}

TEST(Initialization, ReadyIsTerminal) {
    auto init = Initialization{};
    init.start(kBegin);
    EXPECT_EQ(reach_ready(init, kBegin), Status::kReady);
    EXPECT_EQ(
        init.update(kReference, make_pose(0.05, 20.), kBegin + std::chrono::seconds{1}),
        Status::kReady);
    EXPECT_TRUE(init.reference_pose()->isApprox(kReference));
}

TEST(Initialization, FailedIsTerminalUntilReset) {
    auto init = Initialization{Initialization::Config{.timeout_s = 0.1}};
    init.start(kBegin);
    EXPECT_EQ(init.update(std::nullopt, std::nullopt, kBegin), Status::kInitializing);
    EXPECT_EQ(
        init.update(std::nullopt, std::nullopt, kBegin + std::chrono::milliseconds{200}),
        Status::kFailed);
    EXPECT_EQ(
        init.update(kReference, kReference, kBegin + std::chrono::milliseconds{300}),
        Status::kFailed);

    init.reset();
    EXPECT_EQ(init.status(), Status::kIdle);
    EXPECT_FALSE(init.reference_pose().has_value());
    // 复位后不重新开始就不处理帧
    EXPECT_EQ(feed_reference(init, kBegin), Status::kIdle);
    EXPECT_FALSE(init.reference_pose().has_value());

    init.start(kBegin);
    EXPECT_EQ(feed_reference(init, kBegin), Status::kInitializing);
    EXPECT_TRUE(init.reference_pose()->isApprox(kReference));
}
