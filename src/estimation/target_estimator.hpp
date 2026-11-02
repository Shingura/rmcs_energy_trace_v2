#pragma once

#include <expected>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <opencv2/core/mat.hpp>

#include "detection/detector.hpp"
#include "estimation/initialization.hpp"
#include "geometry/camera.hpp"
#include "registration/registration.hpp"
#include "tracking/tracker.hpp"
#include "utility/pimpl.hpp"
#include "verification/depth_check.hpp"

namespace rmcs_energy_trace_v2::estimation {

// 开环取矿的目标估计：初始化时定下能量单元在底座坐标系下的位姿，机械臂按该位姿抓取。
// 初始化成功后本模块照常逐帧估计并输出，机械臂不再采用这些结果。
// 初始化失败时保持不动，由使用方复位后重新开始。
class TargetEstimator {
    RMCS_PIMPL_DEFINITION(TargetEstimator)

public:
    // 跟踪结果是否可信的判据，阈值沿用 v1 的监督器
    struct TrustConfig {
        // 未被遮挡的像素中，空闲空间违例（mesh 所在处看到了更远的背景）的比例上限。
        // 遮挡不算失败：接近时夹爪会挡住能量单元
        double max_free_space_ratio = 0.15;
        // 未被遮挡的有效像素少于该值时，这一帧无法判断：几乎全被挡住或已离开视野
        std::size_t min_unoccluded_pixels = 200;
        // 未被遮挡的像素占有效像素的比例下限。剩下的像素太少时看不出位姿是否偏了，
        // 只看像素数的话分辨率高的相机几乎不会触发
        double min_unoccluded_ratio = 0.25;
        // 封闭端被观测到的像素比例下限。轴向位置靠封闭端约束：封闭端被遮挡、没有实测深度
        // 或出了画面时，跟踪会沿轴线滑动，其余的深度统计看不出来。
        // 开口端套在柱子上，正确跟踪时也大多看不到，不用它判断
        double min_closed_end_observed_ratio = 0.8;
    };

    struct Config {
        // 彩色图和深度图共用这组参数（深度图已对齐到彩色图）。同时用于配准、跟踪和检测框裁剪
        geometry::Camera camera;
        // 线程数上限，同时用于推理和 OpenMP（配准、跟踪、深度检查）。0 表示不限制。
        // 实机上还有机械臂控制等组件，视觉不该占满所有核
        int threads = 0;

        detection::Detector::Config detector;
        registration::Registration::Config registration;
        // camera 取上面的值，这里的 camera 会被覆盖
        tracking::Tracker::Config tracker;
        // 逐帧深度检查的参数
        verification::DepthCheck::Config depth_check;
        TrustConfig trust;
        Initialization::Config initialization;
    };

    using Status = Initialization::Status;
    using Clock = Initialization::Clock;

    struct Output {
        Status status = Status::kIdle;
        // 参考位姿（底座系）：配准通过后有效，成功之后不再改变
        Eigen::Isometry3d reference_pose = Eigen::Isometry3d::Identity();
        // 本帧估计的位姿（底座系）与是否可信。成功之后机械臂不再采用它
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        bool pose_valid = false;
    };

    // mesh_path 供配准使用，检测网络和 M3T 模型目录见各自 Config 中的路径。
    // 三者缺一返回错误
    std::expected<void, std::string>
        initialize(const std::string& mesh_path, const Config& config) noexcept;

    // 收到启动指令，开始初始化。now 为该帧的时刻，见 Initialization::update
    void start(Clock::time_point now = Clock::now()) noexcept;
    // 回到等待状态：丢弃参考位姿和跟踪状态，等下一次指令
    void reset() noexcept;

    // 逐帧处理。camera_to_base 为该帧图像对应的相机到底座的位姿，now 为该帧的时刻。
    // 等待和失败状态下不做处理，只回报状态
    Output update(
        const cv::Mat& bgr, const cv::Mat& depth, const Eigen::Isometry3d& camera_to_base,
        Clock::time_point now = Clock::now()) noexcept;
};

} // namespace rmcs_energy_trace_v2::estimation
