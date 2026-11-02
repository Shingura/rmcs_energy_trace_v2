#include "estimation/initialization.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace rmcs_energy_trace_v2::estimation {

namespace {

// 两个位姿之差：中心距离和长轴夹角。长轴为 z 轴，见组件输出的位姿约定
struct Deviation {
    double center = 0.;
    double axis_deg = 0.;
};

auto deviation(const Eigen::Isometry3d& reference, const Eigen::Isometry3d& tracked) -> Deviation {
    const double dot = std::clamp(reference.linear().col(2).dot(tracked.linear().col(2)), -1., 1.);
    return Deviation{
        .center = (reference.translation() - tracked.translation()).norm(),
        .axis_deg = std::acos(dot) * 180. / std::numbers::pi};
}

} // namespace

Initialization::Initialization(const Config& config) noexcept
    : config_{config} {}

void Initialization::start() noexcept {
    status_ = Status::kInitializing;
    reference_pose_.reset();
    confirmation_count_ = 0;
    begin_time_ = std::chrono::steady_clock::now();
}

void Initialization::reset() noexcept {
    status_ = Status::kIdle;
    reference_pose_.reset();
    confirmation_count_ = 0;
    begin_time_ = {};
}

auto Initialization::update(
    const std::optional<Eigen::Isometry3d>& registered,
    const std::optional<Eigen::Isometry3d>& tracked) noexcept -> Status {
    if (status_ != Status::kInitializing)
        return status_;

    if (std::chrono::steady_clock::now() - begin_time_
        > std::chrono::duration<double>{config_.timeout_s}) {
        status_ = Status::kFailed;
        return status_;
    }

    if (!reference_pose_ && registered)
        reference_pose_ = registered;

    if (!reference_pose_)
        return status_;

    bool consistent = false;
    if (tracked) {
        const auto d = deviation(*reference_pose_, *tracked);
        consistent = d.center <= config_.max_center_deviation
                  && d.axis_deg <= config_.max_axis_deviation_deg;
    }
    confirmation_count_ = consistent ? confirmation_count_ + 1 : 0;
    if (confirmation_count_ >= config_.confirmation_frames)
        status_ = Status::kReady;

    return status_;
}

auto Initialization::status() const noexcept -> Status { return status_; }

auto Initialization::reference_pose() const noexcept -> const std::optional<Eigen::Isometry3d>& {
    return reference_pose_;
}

} // namespace rmcs_energy_trace_v2::estimation
