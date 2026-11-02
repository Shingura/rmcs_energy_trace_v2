#include "estimation/target_estimator.hpp"

#include <cmath>
#include <exception>
#include <memory>
#include <optional>
#include <string>

#include <omp.h>
#include <opencv2/core/types.hpp>

#include "geometry/mesh.hpp"

namespace rmcs_energy_trace_v2::estimation {

namespace {

auto to_roi(const cv::Rect2f& box, const geometry::Camera& camera) -> cv::Rect2i {
    const auto rect = cv::Rect2i{
        cv::Point2i{static_cast<int>(std::floor(box.x)), static_cast<int>(std::floor(box.y))},
        cv::Size{static_cast<int>(std::ceil(box.width)), static_cast<int>(std::ceil(box.height))}};
    return rect & cv::Rect2i{cv::Point2i{0, 0}, cv::Size{camera.width, camera.height}};
}

// 选离画面中心最近的检测。操作手按键前已用准心对准目标，每次按键都按同一条规则选
auto pick_nearest_center(
    const detection::Detector::Detections& detections, const geometry::Camera& camera)
    -> std::optional<detection::Detector::Detection> {
    const auto center = cv::Point2f{
        static_cast<float>(camera.width) / 2.f, static_cast<float>(camera.height) / 2.f};
    std::optional<detection::Detector::Detection> best;
    auto best_distance = 0.;
    for (const auto& detection : detections) {
        const auto box_center = cv::Point2f{
            detection.box.x + detection.box.width / 2.f,
            detection.box.y + detection.box.height / 2.f};
        const auto distance = cv::norm(box_center - center);
        if (!best || distance < best_distance) {
            best = detection;
            best_distance = distance;
        }
    }
    return best;
}

} // namespace

struct TargetEstimator::Impl {
    Config config;
    detection::Detector detector;
    registration::Registration registration;
    tracking::Tracker tracker;
    verification::DepthCheck depth_check;
    Initialization initialization;
    bool tracker_started = false;
    // 最近一次得到的位姿（底座系），本帧没有结果时沿用
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

    std::expected<void, std::string>
        initialize(const std::string& mesh_path, const Config& new_config) {
        config = new_config;
        auto tracker_config = config.tracker;
        tracker_config.camera = config.camera;
        if (config.threads > 0) {
            config.detector.infer_threads = config.threads;
            omp_set_num_threads(config.threads);
        }

        if (const auto result = detector.initialize(config.detector); !result)
            return std::unexpected{"Failed to initialize the detector: " + result.error()};
        if (const auto result = registration.initialize(mesh_path, config.registration); !result)
            return std::unexpected{"Failed to initialize the registration: " + result.error()};
        if (const auto result = tracker.initialize(tracker_config); !result)
            return std::unexpected{"Failed to initialize the tracker: " + result.error()};

        auto mesh = geometry::load_binary_stl(mesh_path);
        if (!mesh)
            return std::unexpected{"Failed to load the mesh: " + mesh.error()};
        depth_check = verification::DepthCheck{std::move(*mesh), config.depth_check};

        initialization = Initialization{config.initialization};
        return {};
    }

    // 跟踪结果的可信判定：按 v1 的监督器，只取深度检查相关的判据。
    // 位姿相对上一帧的跳变由确认窗口覆盖，检测框 IoU 需要每帧跑检测，两者都不在这里判断
    [[nodiscard]] bool is_trusted(const Eigen::Isometry3d& pose, const cv::Mat& depth) const {
        const auto box = depth_check.projected_box(config.camera, pose);
        if (box.empty())
            return false;
        const auto quality = depth_check.check(depth, config.camera, box, {}, pose);

        // 只看未被遮挡的像素：支持和空闲空间违例
        const auto unoccluded_ratio = quality.support_ratio + quality.free_space_ratio;
        const auto unoccluded = static_cast<double>(quality.valid_pixels) * unoccluded_ratio;
        if (unoccluded < static_cast<double>(config.trust.min_unoccluded_pixels))
            return false;
        if (unoccluded_ratio < config.trust.min_unoccluded_ratio)
            return false;
        if (quality.free_space_ratio / unoccluded_ratio > config.trust.max_free_space_ratio)
            return false;

        const auto closed = depth_check.end_observed_ratios(depth, config.camera, pose).closed;
        return closed >= config.trust.min_closed_end_observed_ratio;
    }

    void start(Initialization::Clock::time_point now) noexcept {
        initialization.start(now);
        tracker_started = false;
        pose = Eigen::Isometry3d::Identity();
    }

    void reset() noexcept {
        initialization.reset();
        tracker_started = false;
        pose = Eigen::Isometry3d::Identity();
    }

    auto update(
        const cv::Mat& bgr, const cv::Mat& depth, const Eigen::Isometry3d& camera_to_base,
        Initialization::Clock::time_point now) -> Output {
        const auto status = initialization.status();
        if (status != Status::kInitializing && status != Status::kReady)
            return Output{.status = status};

        std::optional<Eigen::Isometry3d> registered;
        std::optional<Eigen::Isometry3d> tracked;
        auto pose_valid = false;

        if (tracker_started) {
            if (const auto result = tracker.track(bgr, depth)) {
                tracked = camera_to_base * result->pose;
                pose_valid = is_trusted(result->pose, depth);
            }
        } else {
            // 每帧检测并配准，直到配准通过。通过后启动跟踪，进入确认窗口
            const auto detections = detector.detect(bgr);
            if (const auto detection = pick_nearest_center(detections, config.camera)) {
                const auto result = registration.register_depth(
                    depth, config.camera, to_roi(detection->box, config.camera), detection->mask);
                if (result) {
                    registered = camera_to_base * result->pose;
                    pose_valid = true;
                    // 启动失败时下一帧重新配准
                    if (tracker.start(result->pose, bgr, depth))
                        tracker_started = true;
                }
            }
        }

        if (tracked)
            pose = *tracked;
        else if (registered)
            pose = *registered;

        auto output = Output{
            .status = initialization.update(registered, tracked, now),
            .pose = pose,
            .pose_valid = pose_valid};
        if (const auto& reference = initialization.reference_pose())
            output.reference_pose = *reference;
        return output;
    }
};

auto TargetEstimator::initialize(const std::string& mesh_path, const Config& config) noexcept
    -> std::expected<void, std::string> try {
    return pimpl_->initialize(mesh_path, config);
} catch (const std::exception& e) {
    return std::unexpected{std::string{"Failed to initialize the target estimator: "} + e.what()};
}

void TargetEstimator::start(Clock::time_point now) noexcept { pimpl_->start(now); }

void TargetEstimator::reset() noexcept { pimpl_->reset(); }

auto TargetEstimator::update(
    const cv::Mat& bgr, const cv::Mat& depth, const Eigen::Isometry3d& camera_to_base,
    Clock::time_point now) noexcept -> Output try {
    return pimpl_->update(bgr, depth, camera_to_base, now);
} catch (const std::exception&) {
    return Output{.status = pimpl_->initialization.status()};
}

TargetEstimator::TargetEstimator() noexcept
    : pimpl_{std::make_unique<Impl>()} {}

TargetEstimator::~TargetEstimator() noexcept = default;

} // namespace rmcs_energy_trace_v2::estimation
