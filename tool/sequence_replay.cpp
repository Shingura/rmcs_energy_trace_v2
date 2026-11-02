// 离线回放：在渲染出的连续 RGB-D 序列上运行开环目标估计，
// 输出叠加画面的视频、逐帧 CSV 和相对真值的误差汇总

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <numbers>
#include <optional>
#include <print>
#include <random>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "estimation/target_estimator.hpp"
#include "geometry/mesh.hpp"
#include "verification/depth_check.hpp"

#if !defined(RMCS_ENERGY_UNIT_MESH) || !defined(RMCS_ENERGY_TRACE_ASSET_DIR)
# error "RMCS_ENERGY_UNIT_MESH and RMCS_ENERGY_TRACE_ASSET_DIR must be defined"
#endif

namespace fs = std::filesystem;
using rmcs_energy_trace_v2::estimation::TargetEstimator;
using rmcs_energy_trace_v2::geometry::Camera;
using Clock = TargetEstimator::Clock;

namespace {

constexpr double kRadToDeg = 180. / std::numbers::pi;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// 深度图随机置为无效的像素比例
constexpr double kDropoutRatio = 0.005;
constexpr std::uint32_t kNoiseSeed = 20261027;
// 错误接受：可信但误差超过该阈值
constexpr double kWrongCenterMm = 10.;
constexpr double kWrongAxisDeg = 5.;

struct Options {
    fs::path sequence_dir;
    fs::path output_dir;
    std::string network;
    double fps = 30.;
    double depth_noise = 1.5;
    int frames = 0;         // 0 表示全部
    bool video = true;
};

struct TruthUnit {
    int index;
    Eigen::Isometry3d pose; // 能量单元到相机
};

struct Sequence {
    Camera camera;
    int target = 0;
    int frames = 0;
    double fps = 30.;       // 序列的帧率，没有时按 30
};

// 一帧的记录，对应 frames.csv 的一行
struct Row {
    int frame = 0;
    bool processed = false;
    std::string status; // idle、initializing、ready、failed、skipped
    bool pose_valid = false;
    std::string phase;  // 序列的阶段（hold、approach、grasp、extract），没有时为空
    bool has_pose = false;
    int unit = -1;
    bool is_target = false;
    double center_mm = kNaN, axis_deg = kNaN;
    // 参考位姿的误差，ready 之后每帧相同
    double ref_center_mm = kNaN, ref_axis_deg = kNaN;
    double total_ms = kNaN;
    // 诊断：该帧位姿处的深度统计，由本工具自己算
    double support = kNaN, free_space = kNaN, occluded = kNaN;
    double closed_end_observed = kNaN, open_end_observed = kNaN;
};

auto median(std::vector<double> values) -> double {
    if (values.empty())
        return kNaN;
    std::ranges::sort(values);
    return values[values.size() / 2];
}

auto percentile(std::vector<double> values, double p) -> double {
    if (values.empty())
        return kNaN;
    std::ranges::sort(values);
    return values[static_cast<std::size_t>(
        std::lround(p * static_cast<double>(values.size() - 1)))];
}

auto max_of(const std::vector<double>& values) -> double {
    return values.empty() ? kNaN : *std::ranges::max_element(values);
}

auto frame_name(int frame) -> std::string { return std::format("{:06d}", frame); }

auto cell(double value) -> std::string {
    return std::isnan(value) ? "" : std::format("{:.3f}", value);
}

auto status_name(TargetEstimator::Status status) -> std::string {
    switch (status) {
    case TargetEstimator::Status::kIdle: return "idle";
    case TargetEstimator::Status::kInitializing: return "initializing";
    case TargetEstimator::Status::kReady: return "ready";
    case TargetEstimator::Status::kFailed: return "failed";
    }
    return {};
}

auto parse_pose(const cv::FileNode& node) -> Eigen::Isometry3d {
    auto pose = Eigen::Isometry3d::Identity();
    auto r = 0;
    for (const auto& row : node) {
        auto c = 0;
        for (const auto& value : row)
            pose.matrix()(r, c++) = static_cast<double>(value.real());
        r++;
    }
    return pose;
}

auto load_sequence(const fs::path& dir) -> std::optional<Sequence> {
    auto storage = cv::FileStorage{(dir / "sequence.json").string(), cv::FileStorage::READ};
    if (!storage.isOpened())
        return std::nullopt;
    auto sequence = Sequence{};
    const auto camera = storage["camera"];
    sequence.camera = {
        .width = static_cast<int>(camera["width"].real()),
        .height = static_cast<int>(camera["height"].real()),
        .fx = camera["fx"].real(),
        .fy = camera["fy"].real(),
        .cx = camera["cx"].real(),
        .cy = camera["cy"].real(),
        .depth_scale = camera["depth_scale"].real(),
    };
    sequence.target = static_cast<int>(storage["target"].real());
    sequence.frames = static_cast<int>(storage["frames"].real());
    if (const auto fps = storage["fps"]; !fps.empty() && fps.real() > 0.)
        sequence.fps = fps.real();
    return sequence;
}

auto load_truth(const fs::path& dir, int frame) -> std::vector<TruthUnit> {
    auto units = std::vector<TruthUnit>{};
    auto storage = cv::FileStorage{
        (dir / "meta" / (frame_name(frame) + ".json")).string(), cv::FileStorage::READ};
    if (!storage.isOpened())
        return units;
    for (const auto& unit : storage["units"])
        units.push_back({
            .index = static_cast<int>(unit["index"].real()),
            .pose = parse_pose(unit["pose"]),
        });
    return units;
}

// meta 里的相机（OpenCV 坐标系）到世界的位姿。世界系即底座系，没有时按单位阵处理
auto load_camera_pose(const fs::path& dir, int frame) -> std::optional<Eigen::Isometry3d> {
    auto storage = cv::FileStorage{
        (dir / "meta" / (frame_name(frame) + ".json")).string(), cv::FileStorage::READ};
    if (!storage.isOpened())
        return std::nullopt;
    const auto camera = storage["camera"];
    if (!camera.isSeq() || camera.size() != 4)
        return std::nullopt;
    return parse_pose(camera);
}

// 按规划渲染的序列，meta 里有阶段；内置轨迹的序列没有，返回空字符串
auto load_phase(const fs::path& dir, int frame) -> std::string {
    auto storage = cv::FileStorage{
        (dir / "meta" / (frame_name(frame) + ".json")).string(), cv::FileStorage::READ};
    if (!storage.isOpened())
        return {};
    const auto phase = storage["phase"];
    return phase.isString() ? phase.string() : std::string{};
}

// 深度噪声：标准差 = noise_mm * z^2（mm，z 单位 m），另有一部分像素置为无效。
// 种子由帧号决定，同一帧的噪声不随跳帧而变
auto add_depth_noise(const cv::Mat& depth, const Camera& camera, double noise_mm, int frame)
    -> cv::Mat {
    auto noisy = depth.clone();
    if (noise_mm <= 0.)
        return noisy;
    auto engine = std::mt19937{kNoiseSeed + static_cast<std::uint32_t>(frame)};
    auto normal = std::normal_distribution<double>{};
    auto dropout = std::bernoulli_distribution{kDropoutRatio};
    const auto to_unit = 1e-3 / camera.depth_scale; // mm -> 深度图单位
    for (int v = 0; v < noisy.rows; v++) {
        auto* row = noisy.ptr<std::uint16_t>(v);
        for (int u = 0; u < noisy.cols; u++) {
            if (row[u] == 0)
                continue;
            if (dropout(engine)) {
                row[u] = 0;
                continue;
            }
            const auto z = row[u] * camera.depth_scale;
            const auto value = row[u] + normal(engine) * noise_mm * z * z * to_unit;
            row[u] = static_cast<std::uint16_t>(std::clamp(std::lround(value), 1L, 65535L));
        }
    }
    return noisy;
}

auto axis_error_deg(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) -> double {
    return std::acos(std::clamp(a.linear().col(2).dot(b.linear().col(2)), -1., 1.)) * kRadToDeg;
}

// mesh 按位姿投影后所有顶点的凸包，作为轮廓
auto outline(
    const std::vector<Eigen::Vector3f>& vertices, const Camera& camera,
    const Eigen::Isometry3d& pose) -> std::vector<cv::Point> {
    const Eigen::Matrix3f rotation = pose.linear().cast<float>();
    const Eigen::Vector3f translation = pose.translation().cast<float>();
    auto points = std::vector<cv::Point2f>{};
    points.reserve(vertices.size());
    for (const auto& vertex : vertices) {
        const Eigen::Vector3f p = rotation * vertex + translation;
        if (p.z() <= 0.01f)
            continue;
        points.emplace_back(
            static_cast<float>(camera.fx) * p.x() / p.z() + static_cast<float>(camera.cx),
            static_cast<float>(camera.fy) * p.y() / p.z() + static_cast<float>(camera.cy));
    }
    auto hull = std::vector<cv::Point2f>{};
    if (!points.empty())
        cv::convexHull(points, hull);
    return {hull.begin(), hull.end()};
}

void put_text(cv::Mat& image, const std::string& text, int line) {
    const auto origin = cv::Point{6, 16 + 16 * line};
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 0}, 3, cv::LINE_AA);
    cv::putText(
        image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1, cv::LINE_AA);
}

// pose 和 truth 都是相机系
auto draw_overlay(
    const cv::Mat& color, const TargetEstimator::Output& output, const Row& row,
    const std::vector<Eigen::Vector3f>& vertices, const Camera& camera,
    const Eigen::Isometry3d& pose, const Eigen::Isometry3d& truth) -> cv::Mat {
    auto image = color.clone();
    cv::polylines(
        image, std::vector<std::vector<cv::Point>>{outline(vertices, camera, truth)}, true,
        {255, 255, 255}, 1, cv::LINE_AA);
    if (row.has_pose) {
        cv::polylines(
            image, std::vector<std::vector<cv::Point>>{outline(vertices, camera, pose)}, true,
            row.pose_valid ? cv::Scalar{0, 220, 0} : cv::Scalar{0, 255, 255}, 2, cv::LINE_AA);
    }
    put_text(
        image,
        std::format("#{} {} {}", row.frame, row.status, row.pose_valid ? "trusted" : "untrusted"),
        0);
    if (row.has_pose)
        put_text(
            image,
            std::format(
                "{} err {:.1f} mm {:.1f} deg", row.is_target ? "target" : "other unit",
                row.center_mm, row.axis_deg),
            1);
    put_text(image, std::format("{:.0f} ms", row.total_ms), 2);
    if (!row.phase.empty())
        put_text(image, std::format("phase {}", row.phase), 3);
    if (output.status == TargetEstimator::Status::kReady)
        put_text(
            image, std::format("ref {:.1f} mm {:.1f} deg", row.ref_center_mm, row.ref_axis_deg), 4);
    return image;
}

void write_csv(const fs::path& path, const std::vector<Row>& rows) {
    auto out = std::ofstream{path};
    out << "frame,processed,status,pose_valid,phase,unit,is_target,center_mm,axis_deg,"
           "ref_center_mm,ref_axis_deg,total_ms,support,free_space,occluded,closed_end_observed,"
           "open_end_observed\n";
    for (const auto& r : rows) {
        out << r.frame << ',' << r.processed << ',' << r.status << ',' << r.pose_valid << ','
            << r.phase << ',' << (r.unit >= 0 ? std::to_string(r.unit) : "") << ','
            << (r.has_pose ? r.is_target : 0) << ',' << cell(r.center_mm) << ',' << cell(r.axis_deg)
            << ',' << cell(r.ref_center_mm) << ',' << cell(r.ref_axis_deg) << ','
            << cell(r.total_ms) << ',' << cell(r.support) << ',' << cell(r.free_space) << ','
            << cell(r.occluded) << ',' << cell(r.closed_end_observed) << ','
            << cell(r.open_end_observed) << '\n';
    }
}

// 一个阶段内的统计
auto phase_summary(const std::string& phase, const std::vector<Row>& rows) -> std::string {
    auto processed = 0, trusted = 0, wrong = 0;
    auto centers = std::vector<double>{}, axes = std::vector<double>{};
    for (const auto& r : rows) {
        if (!r.processed || r.phase != phase)
            continue;
        processed++;
        trusted += r.pose_valid;
        if (r.pose_valid && r.has_pose) {
            centers.push_back(r.center_mm);
            axes.push_back(r.axis_deg);
            wrong += r.center_mm > kWrongCenterMm || r.axis_deg > kWrongAxisDeg;
        }
    }
    return std::format(
        "phase {}: processed {} trusted {} ({:.1f}%) | trusted center mm median {:.2f} "
        "max {:.2f} | axis deg median {:.2f} max {:.2f} | wrong accepts {}\n",
        phase, processed, trusted, processed > 0 ? 100. * trusted / processed : 0., median(centers),
        max_of(centers), median(axes), max_of(axes), wrong);
}

auto summarize(const Options& options, const Sequence& sequence, const std::vector<Row>& rows)
    -> std::string {
    auto processed = 0, skipped = 0;
    auto ready_frame = -1, motion_start = -1;
    auto phases = std::vector<std::string>{}; // 按出现顺序
    auto ref_center = kNaN, ref_axis = kNaN;
    auto trusted = 0, wrong = 0, trusted_other = 0;
    auto centers = std::vector<double>{}, axes = std::vector<double>{};
    auto total_ms = 0.;
    for (const auto& r : rows) {
        if (!r.phase.empty()) {
            if (std::ranges::find(phases, r.phase) == phases.end())
                phases.push_back(r.phase);
            if (r.phase != "hold" && motion_start < 0)
                motion_start = r.frame;
        }
        if (!r.processed) {
            skipped++;
            continue;
        }
        processed++;
        total_ms += r.total_ms;
        trusted += r.pose_valid;
        if (r.status == "ready") {
            if (ready_frame < 0)
                ready_frame = r.frame;
            ref_center = r.ref_center_mm;
            ref_axis = r.ref_axis_deg;
        }
        if (r.pose_valid && r.has_pose) {
            centers.push_back(r.center_mm);
            axes.push_back(r.axis_deg);
            wrong += r.center_mm > kWrongCenterMm || r.axis_deg > kWrongAxisDeg;
            trusted_other += !r.is_target;
        }
    }
    const auto n = std::max(processed, 1);
    auto text = std::format(
        "sequence {} | fps {} | depth noise {} mm/m^2\n", options.sequence_dir.string(),
        options.fps, options.depth_noise);
    text += std::format("frames processed {} skipped {}\n", processed, skipped);
    if (ready_frame >= 0) {
        text += std::format(
            "initialized at frame {} ({:.2f} s)\n", ready_frame, ready_frame / sequence.fps);
        text += std::format(
            "reference pose error: center {:.2f} mm, axis {:.2f} deg\n", ref_center, ref_axis);
    } else {
        const auto failed =
            std::ranges::any_of(rows, [](const Row& r) { return r.status == "failed"; });
        text += std::format("initialization {}\n", failed ? "failed" : "not finished");
    }
    if (motion_start >= 0 && ready_frame >= 0) {
        text += std::format(
            "initialized {} motion start by {:.2f} s (motion starts at frame {})\n",
            (motion_start - ready_frame) >= 0 ? "before" : "after",
            std::abs(static_cast<double>(motion_start - ready_frame)) / sequence.fps, motion_start);
    }
    text += std::format(
        "trusted {}/{} ({:.1f}%)\n", trusted, processed,
        processed > 0 ? 100. * trusted / processed : 0.);
    text += std::format(
        "trusted center error mm: median {:.2f} p90 {:.2f} max {:.2f}\n", median(centers),
        percentile(centers, 0.9), max_of(centers));
    text += std::format(
        "trusted axis error deg: median {:.2f} p90 {:.2f} max {:.2f}\n", median(axes),
        percentile(axes, 0.9), max_of(axes));
    text += std::format(
        "wrong accepts (trusted, center > {:.0f} mm or axis > {:.0f} deg) {}; trusted frames on a "
        "non-target unit {}\n",
        kWrongCenterMm, kWrongAxisDeg, wrong, trusted_other);
    text += std::format("mean ms per processed frame: total {:.1f}\n", total_ms / n);
    for (const auto& phase : phases)
        text += phase_summary(phase, rows);
    return text;
}

auto parse_options(int argc, char* argv[]) -> std::optional<Options> {
    auto options = Options{};
    auto positional = std::vector<std::string>{};
    for (int i = 1; i < argc; i++) {
        const auto arg = std::string{argv[i]};
        if (arg == "--network" && i + 1 < argc) {
            options.network = argv[++i];
        } else if (arg == "--fps" && i + 1 < argc) {
            options.fps = std::strtod(argv[++i], nullptr);
        } else if (arg == "--depth-noise" && i + 1 < argc) {
            options.depth_noise = std::strtod(argv[++i], nullptr);
        } else if (arg == "--frames" && i + 1 < argc) {
            options.frames = static_cast<int>(std::strtol(argv[++i], nullptr, 10));
        } else if (arg == "--no-video") {
            options.video = false;
        } else {
            positional.push_back(arg);
        }
    }
    if (positional.size() != 2 || options.network.empty())
        return std::nullopt;
    options.sequence_dir = positional[0];
    options.output_dir = positional[1];
    return options;
}

} // namespace

auto main(int argc, char* argv[]) -> int {
    const auto options = parse_options(argc, argv);
    if (!options) {
        std::println(
            "Usage: sequence_replay <sequence_dir> <output_dir> --network <network.xml> "
            "[--fps 30] [--depth-noise 1.5] [--frames N] [--no-video]");
        return EXIT_FAILURE;
    }
    const auto sequence = load_sequence(options->sequence_dir);
    if (!sequence) {
        std::println(stderr, "Cannot read {}/sequence.json", options->sequence_dir.string());
        return EXIT_FAILURE;
    }
    const auto total =
        options->frames > 0 ? std::min(options->frames, sequence->frames) : sequence->frames;

    auto config = TargetEstimator::Config{};
    config.camera = sequence->camera;
    config.detector.network_path = options->network;
    config.tracker.model_dir = RMCS_ENERGY_TRACE_ASSET_DIR;
    auto estimator = TargetEstimator{};
    if (auto initialized = estimator.initialize(RMCS_ENERGY_UNIT_MESH, config); !initialized) {
        std::println(stderr, "{}", initialized.error());
        return EXIT_FAILURE;
    }

    // 叠加画面和诊断用。诊断由本工具自己算，估计器只给出位姿和是否可信
    auto mesh = rmcs_energy_trace_v2::geometry::load_binary_stl(RMCS_ENERGY_UNIT_MESH);
    if (!mesh) {
        std::println(stderr, "{}", mesh.error());
        return EXIT_FAILURE;
    }
    const auto depth_check =
        rmcs_energy_trace_v2::verification::DepthCheck{*mesh, config.depth_check};
    auto vertices = std::vector<Eigen::Vector3f>{};
    for (const auto& triangle : mesh->triangles)
        vertices.insert(vertices.end(), triangle.begin(), triangle.end());

    fs::create_directories(options->output_dir);
    auto writer = cv::VideoWriter{};
    auto frames_dir = fs::path{};
    if (options->video) {
        const auto video_path = (options->output_dir / "replay.mp4").string();
        writer.open(
            video_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30.,
            {sequence->camera.width, sequence->camera.height});
        if (!writer.isOpened()) {
            frames_dir = options->output_dir / "frames";
            fs::create_directories(frames_dir);
            std::println(
                "Cannot open {} for writing, saving JPEGs to {}", video_path, frames_dir.string());
        }
    }

    // 序列时间：处理一帧的时间通常远长于相机出帧的间隔，超时按序列时间判断
    const auto begin_time = Clock::now();
    const auto frame_time = [&](int frame) {
        return begin_time
             + std::chrono::duration_cast<Clock::duration>(
                   std::chrono::duration<double>{frame / sequence->fps});
    };

    auto rows = std::vector<Row>{};
    for (int frame = 0; frame < total;) {
        auto row = Row{};
        row.frame = frame;
        const auto color =
            cv::imread((options->sequence_dir / "color" / (frame_name(frame) + ".jpg")).string());
        const auto depth = cv::imread(
            (options->sequence_dir / "depth" / (frame_name(frame) + ".png")).string(),
            cv::IMREAD_UNCHANGED);
        if (color.empty() || depth.empty()) {
            std::println(stderr, "Cannot read frame {}", frame);
            return EXIT_FAILURE;
        }
        const auto noisy = add_depth_noise(depth, sequence->camera, options->depth_noise, frame);
        const auto truth = load_truth(options->sequence_dir, frame);
        // 世界系即底座系，meta 里没有相机位姿时按单位阵处理
        const auto to_base =
            load_camera_pose(options->sequence_dir, frame).value_or(Eigen::Isometry3d::Identity());

        // 第 0 帧相当于操作手按键
        if (frame == 0)
            estimator.start(frame_time(frame));
        const auto begin = Clock::now();
        const auto output = estimator.update(color, noisy, to_base, frame_time(frame));
        const auto seconds = std::chrono::duration<double>{Clock::now() - begin}.count();

        row.processed = true;
        row.status = status_name(output.status);
        row.pose_valid = output.pose_valid;
        row.phase = load_phase(options->sequence_dir, frame);
        row.total_ms = seconds * 1e3;
        row.has_pose = output.status == TargetEstimator::Status::kInitializing
                    || output.status == TargetEstimator::Status::kReady;

        if (row.has_pose) {
            // 诊断：该帧位姿处的深度统计
            const auto in_camera = to_base.inverse() * output.pose;
            const auto box = depth_check.projected_box(sequence->camera, in_camera);
            const auto quality = depth_check.check(noisy, sequence->camera, box, {}, in_camera);
            row.support = quality.support_ratio;
            row.free_space = quality.free_space_ratio;
            row.occluded = quality.occluded_ratio;
            const auto observed =
                depth_check.end_observed_ratios(noisy, sequence->camera, in_camera);
            row.closed_end_observed = observed.closed;
            row.open_end_observed = observed.open;

            // 与真值比较：中心最近的真值单元
            if (!truth.empty()) {
                const auto nearest = std::min_element(
                    truth.begin(), truth.end(), [&](const TruthUnit& a, const TruthUnit& b) {
                        return (a.pose.translation() - in_camera.translation()).norm()
                             < (b.pose.translation() - in_camera.translation()).norm();
                    });
                row.unit = nearest->index;
                row.is_target = nearest->index == sequence->target;
                row.center_mm =
                    (nearest->pose.translation() - in_camera.translation()).norm() * 1e3;
                row.axis_deg = axis_error_deg(nearest->pose, in_camera);
            }
        }
        if (output.status == TargetEstimator::Status::kReady) {
            auto target_pose = std::optional<Eigen::Isometry3d>{};
            for (const auto& unit : truth) {
                if (unit.index == sequence->target)
                    target_pose = to_base * unit.pose;
            }
            if (target_pose) {
                row.ref_center_mm =
                    (target_pose->translation() - output.reference_pose.translation()).norm() * 1e3;
                row.ref_axis_deg = axis_error_deg(*target_pose, output.reference_pose);
            }
        }

        if (options->video) {
            const auto in_camera = to_base.inverse() * output.pose;
            auto target_pose = Eigen::Isometry3d::Identity();
            auto has_target = false;
            for (const auto& unit : truth) {
                if (unit.index == sequence->target) {
                    target_pose = unit.pose;
                    has_target = true;
                }
            }
            const auto image = draw_overlay(
                color, output, row, vertices, sequence->camera, in_camera,
                has_target ? target_pose : in_camera);
            if (writer.isOpened())
                writer.write(image);
            else
                cv::imwrite((frames_dir / (frame_name(frame) + ".jpg")).string(), image);
        }
        std::println(
            "#{:03} {:<12} {:<9} {}", frame, row.status, row.pose_valid ? "trusted" : "untrusted",
            row.has_pose ? std::format("{:6.1f} mm {:5.1f} deg", row.center_mm, row.axis_deg)
                         : std::string{"      no pose       "});
        rows.push_back(row);

        // 处理一帧用了 seconds 秒，相机在这段时间里继续出帧，下一帧取最新的一帧
        const auto step = options->fps > 0.
                            ? std::max(1, static_cast<int>(std::ceil(seconds * options->fps)))
                            : 1;
        for (int skip = 1; skip < step && frame + skip < total; skip++) {
            auto& skipped = rows.emplace_back();
            skipped.frame = frame + skip;
            skipped.status = "skipped";
            skipped.phase = load_phase(options->sequence_dir, frame + skip);
        }
        frame += step;
    }

    write_csv(options->output_dir / "frames.csv", rows);
    const auto summary = summarize(*options, *sequence, rows);
    std::ofstream{options->output_dir / "summary.txt"} << summary;
    std::print("\n{}", summary);
    return EXIT_SUCCESS;
}
