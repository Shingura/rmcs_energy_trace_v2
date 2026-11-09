#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include <eigen3/Eigen/Geometry>

namespace rmcs_energy_trace_v2::estimation {

// 开环取矿的初始化状态推进，维护状态和判据：等待指令 → 配准给出参考位姿 → 确认窗口 → 成功或失败。
class Initialization {
public:
    struct Config {
        // 确认窗口：与参考位姿一致的连续帧数达到该值即成功。30 fps 下 15 帧约 0.5 s
        int confirmation_frames = 15;
        // 开始初始化后的时长上限，超过即失败
        double timeout_s = 3.;
        // 确认窗口中跟踪结果与参考位姿之差的阈值。
        // 阈值来自单元静止时两次可信位姿之差的 p99：中心 3.3 mm、轴线 4.3°。
        // 跟踪本身的波动是 1～2 mm、1～2°，阈值按跟踪定，不按配准的精度定
        double max_center_deviation = 0.005; // m
        double max_axis_deviation_deg = 6.;  // °
    };

    enum class Status : std::uint8_t { kIdle, kInitializing, kReady, kFailed };

    Initialization() = default;
    explicit Initialization(const Config& config) noexcept;

    using Clock = std::chrono::steady_clock;

    // 收到启动指令。now 为这一帧的时刻
    void start(Clock::time_point now = Clock::now()) noexcept;
    // 回到等待状态，等下一次指令。参考位姿同时清空
    void reset() noexcept;

    // 逐帧推进一次。registered 为本帧配准通过时给出的位姿，tracked 为本帧跟踪结果，
    // 两者为空表示本帧没有结果。已有参考位姿时忽略 registered。
    // now 为这一帧的时刻：实机用图像的时间戳，回放按序列帧率换算。离线处理一帧的时间
    // 通常远长于相机出帧的间隔，所以不能用当前时刻。
    // 返回推进后的状态：成功和失败都是终态，保持到 reset
    Status update(
        const std::optional<Eigen::Isometry3d>& registered,
        const std::optional<Eigen::Isometry3d>& tracked,
        Clock::time_point now = Clock::now()) noexcept;

    [[nodiscard]] Status status() const noexcept;

    // 参考位姿：配准通过后有效，成功之后不再改变
    [[nodiscard]] const std::optional<Eigen::Isometry3d>& reference_pose() const noexcept;

private:
    Config config_{};
    Status status_ = Status::kIdle;
    std::optional<Eigen::Isometry3d> reference_pose_{};
    int confirmation_count_ = 0;
    Clock::time_point begin_time_{};
};

} // namespace rmcs_energy_trace_v2::estimation
