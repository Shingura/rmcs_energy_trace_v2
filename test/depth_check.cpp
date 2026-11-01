#include <array>
#include <numbers>
#include <utility>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "geometry/camera.hpp"
#include "geometry/mesh.hpp"
#include "verification/depth_check.hpp"

using rmcs_energy_trace_v2::geometry::Camera;
using rmcs_energy_trace_v2::geometry::Mesh;
using rmcs_energy_trace_v2::verification::DepthCheck;

namespace {

// 长轴沿 z、长 0.15 m 的平板（x = 0 平面，y 方向宽 0.04 m），在 z = ±0.0375 处分段，
// 两端各一段、中段一段。端部按三角形重心划分：|z| > 半长 / 2
Mesh make_plate() {
    constexpr auto w = 0.02f;
    const auto quad = [&](float z0, float z1) {
        return std::array<Mesh::Triangle, 2>{
            Mesh::Triangle{
                Eigen::Vector3f{0.f, -w, z0}, Eigen::Vector3f{0.f, w, z0},
                Eigen::Vector3f{0.f, w, z1}},
            Mesh::Triangle{
                Eigen::Vector3f{0.f, -w, z0}, Eigen::Vector3f{0.f, w, z1},
                Eigen::Vector3f{0.f, -w, z1}}};
    };
    auto mesh = Mesh{};
    for (const auto& [z0, z1] :
         {std::pair{-0.075f, -0.0375f}, std::pair{-0.0375f, 0.0375f}, std::pair{0.0375f, 0.075f}}) {
        for (const auto& triangle : quad(z0, z1))
            mesh.triangles.push_back(triangle);
    }
    return mesh;
}

constexpr Camera kCamera{
    .width = 320, .height = 240, .fx = 300., .fy = 300., .cx = 160., .cy = 120.};

// mesh 的 z 轴转到相机的 x 方向（封闭端在 +x 侧），平板正对相机，沿 x 平移 tx，距相机 0.5 m。
// 真值位姿下平板投影为 u 115～205、v 108～132
Eigen::Isometry3d make_pose(double tx) {
    auto pose = Eigen::Isometry3d::Identity();
    pose.linear() =
        Eigen::AngleAxisd{std::numbers::pi / 2., Eigen::Vector3d::UnitY()}.toRotationMatrix();
    pose.translation() = Eigen::Vector3d{tx, 0., 0.5};
    return pose;
}

// 整幅画面的实测深度都是 millimeters。occlude_ends 时，真值位姿下两端所在的列
// （u < 137.5 和 u > 182.5）换成 0.4 m 处的前景，只露出中段
cv::Mat make_depth(int millimeters = 500, bool occlude_ends = false) {
    auto depth = cv::Mat{
        kCamera.height, kCamera.width, CV_16UC1, cv::Scalar{static_cast<double>(millimeters)}};
    if (occlude_ends) {
        depth.colRange(0, 138).setTo(400);
        depth.colRange(183, kCamera.width).setTo(400);
    }
    return depth;
}

const auto kImage = cv::Rect2i{0, 0, kCamera.width, kCamera.height};

} // namespace

// ---- check：逐像素分类 ----

// 实测表面正好在 mesh 处：全部支持
TEST(DepthCheck, TruePoseIsFullySupported) {
    const auto check = DepthCheck{make_plate(), {}};
    const auto quality = check.check(make_depth(), kCamera, kImage, {}, make_pose(0.));
    EXPECT_GT(quality.model_pixels, 1000U);
    EXPECT_EQ(quality.valid_pixels, quality.model_pixels);
    EXPECT_DOUBLE_EQ(quality.support_ratio, 1.);
    EXPECT_DOUBLE_EQ(quality.free_space_ratio, 0.);
    EXPECT_DOUBLE_EQ(quality.occluded_ratio, 0.);
    EXPECT_DOUBLE_EQ(quality.missing_ratio, 0.);
}

// 实测比 mesh 远 10 cm：mesh 悬在空中，全部是空闲空间违例
TEST(DepthCheck, MeshInFrontOfSurfaceIsFreeSpace) {
    const auto check = DepthCheck{make_plate(), {}};
    const auto quality = check.check(make_depth(600), kCamera, kImage, {}, make_pose(0.));
    EXPECT_DOUBLE_EQ(quality.free_space_ratio, 1.);
    EXPECT_DOUBLE_EQ(quality.support_ratio, 0.);
}

// 实测比 mesh 近 10 cm：前面有东西，全部是遮挡，遮挡计入分母、支持率为 0
TEST(DepthCheck, SurfaceInFrontOfMeshIsOccluded) {
    const auto check = DepthCheck{make_plate(), {}};
    const auto quality = check.check(make_depth(400), kCamera, kImage, {}, make_pose(0.));
    EXPECT_DOUBLE_EQ(quality.occluded_ratio, 1.);
    EXPECT_DOUBLE_EQ(quality.support_ratio, 0.);
}

// 差 4 mm 仍在容差（5 mm）内，算支持
TEST(DepthCheck, ToleranceIsFiveMillimeters) {
    const auto check = DepthCheck{make_plate(), {}};
    EXPECT_DOUBLE_EQ(
        check.check(make_depth(504), kCamera, kImage, {}, make_pose(0.)).support_ratio, 1.);
    EXPECT_DOUBLE_EQ(
        check.check(make_depth(506), kCamera, kImage, {}, make_pose(0.)).free_space_ratio, 1.);
}

// 没有实测深度的像素不参与支持率的统计，只计入缺失率
TEST(DepthCheck, MissingDepthIsExcluded) {
    const auto check = DepthCheck{make_plate(), {}};
    auto depth = make_depth();
    depth.colRange(160, kCamera.width).setTo(0);
    const auto quality = check.check(depth, kCamera, kImage, {}, make_pose(0.));
    EXPECT_NEAR(quality.missing_ratio, 0.5, 0.05);
    EXPECT_LT(quality.valid_pixels, quality.model_pixels);
    EXPECT_DOUBLE_EQ(quality.support_ratio, 1.);
}

// 掩码只覆盖平板左半边。右半边（掩码外）更近时不统计；更远时仍记为违例
TEST(DepthCheck, OutsideMaskOnlyFreeSpaceCounts) {
    const auto check = DepthCheck{make_plate(), {}};
    auto mask = cv::Mat{kCamera.height, kCamera.width, CV_8UC1, cv::Scalar{0}};
    mask.colRange(0, 160).setTo(255);

    auto closer = make_depth();
    closer.colRange(160, kCamera.width).setTo(400);
    const auto near_quality = check.check(closer, kCamera, kImage, mask, make_pose(0.));
    EXPECT_DOUBLE_EQ(near_quality.support_ratio, 1.);
    EXPECT_DOUBLE_EQ(near_quality.occluded_ratio, 0.);
    EXPECT_EQ(near_quality.valid_pixels, near_quality.model_pixels);

    auto farther = make_depth();
    farther.colRange(160, kCamera.width).setTo(600);
    const auto far_quality = check.check(farther, kCamera, kImage, mask, make_pose(0.));
    EXPECT_NEAR(far_quality.free_space_ratio, 0.5, 0.05);
    EXPECT_GT(far_quality.valid_pixels, far_quality.model_pixels);
}

// 轮廓向内腐蚀：腐蚀越多，参与统计的像素越少
TEST(DepthCheck, ErosionShrinksCheckedPixels) {
    const auto pixels = [](int erosion) {
        const auto check = DepthCheck{make_plate(), {.silhouette_erosion_px = erosion}};
        return check.check(make_depth(), kCamera, kImage, {}, make_pose(0.)).model_pixels;
    };
    EXPECT_GT(pixels(0), pixels(2));
    EXPECT_GT(pixels(2), pixels(4));
}

// 输入不符时返回全零
TEST(DepthCheck, InvalidInputGivesZeroQuality) {
    const auto check = DepthCheck{make_plate(), {}};
    const auto pose = make_pose(0.);
    const auto wrong_type = cv::Mat{kCamera.height, kCamera.width, CV_32FC1, cv::Scalar{0.5}};
    EXPECT_EQ(check.check(wrong_type, kCamera, kImage, {}, pose).model_pixels, 0U);
    const auto wrong_mask = cv::Mat{10, 10, CV_8UC1, cv::Scalar{255}};
    EXPECT_EQ(check.check(make_depth(), kCamera, kImage, wrong_mask, pose).model_pixels, 0U);
    EXPECT_EQ(
        check.check(make_depth(), kCamera, cv::Rect2i{400, 0, 20, 20}, {}, pose).model_pixels, 0U);
}

// ---- projected_box ----

TEST(DepthCheck, ProjectedBoxIsClippedToImage) {
    const auto check = DepthCheck{make_plate(), {}};
    EXPECT_EQ(check.projected_box(kCamera, make_pose(0.)), (cv::Rect2i{115, 108, 91, 25}));
    // 平板右移到一半在画面外
    const auto box = check.projected_box(kCamera, make_pose(0.2));
    EXPECT_EQ(box.x + box.width, kCamera.width);
}

// ---- end_observed_ratios：两端被观测到的比例 ----

// 没有遮挡的真值位姿：两端都被观测到
TEST(DepthCheck, EndObservedWithoutOcclusion) {
    const auto check = DepthCheck{make_plate(), {}};
    const auto observed = check.end_observed_ratios(make_depth(), kCamera, make_pose(0.));
    EXPECT_GT(observed.closed, 0.95);
    EXPECT_GT(observed.open, 0.95);
}

// 两端被前景挡住、只露出中段：真值位姿和沿轴偏 15 mm 的位姿，两端都几乎观测不到。
// 沿轴偏 15 mm 时中段表面仍与实测深度吻合，这是 check 的盲区
TEST(DepthCheck, EndObservedWithEndsOccluded) {
    const auto check = DepthCheck{make_plate(), {}};
    const auto depth = make_depth(500, true);
    const auto truth = make_pose(0.);
    const auto shifted = make_pose(0.015);

    EXPECT_LT(check.end_observed_ratios(depth, kCamera, truth).closed, 0.1);
    const auto observed = check.end_observed_ratios(depth, kCamera, shifted);
    EXPECT_LT(observed.closed, 0.8);
    EXPECT_LT(observed.open, 0.8);

    // 中段仍与实测深度吻合
    const auto quality =
        check.check(depth, kCamera, check.projected_box(kCamera, shifted), {}, shifted);
    EXPECT_GT(quality.support_ratio, 0.1);
    EXPECT_DOUBLE_EQ(quality.free_space_ratio, 0.);
}

// 只有一端被挡住时，另一端不受影响
TEST(DepthCheck, EndObservedOnlyClosedEndOccluded) {
    const auto check = DepthCheck{make_plate(), {}};
    auto depth = make_depth();
    depth.colRange(183, kCamera.width).setTo(400);
    const auto observed = check.end_observed_ratios(depth, kCamera, make_pose(0.));
    EXPECT_LT(observed.closed, 0.1);
    EXPECT_GT(observed.open, 0.95);
}

// 没有实测深度的像素不算被观测到
TEST(DepthCheck, EndObservedNeedsDepth) {
    const auto check = DepthCheck{make_plate(), {}};
    auto depth = make_depth();
    depth.colRange(183, kCamera.width).setTo(0);
    const auto observed = check.end_observed_ratios(depth, kCamera, make_pose(0.));
    EXPECT_LT(observed.closed, 0.1);
    EXPECT_GT(observed.open, 0.95);
}

// 分母包括画面外的部分：封闭端有一半在画面外时约为 0.5。
// 封闭端的 u 范围为 160 + 600 tx + [22.5, 45]，tx = 0.2121 时中点在 u = 319.5
TEST(DepthCheck, EndObservedCountsPartOutsideImage) {
    const auto check = DepthCheck{make_plate(), {}};
    const auto observed = check.end_observed_ratios(make_depth(), kCamera, make_pose(0.2121));
    EXPECT_NEAR(observed.closed, 0.5, 0.05);
    EXPECT_GT(observed.open, 0.95);
}

TEST(DepthCheck, EndObservedBehindCamera) {
    const auto check = DepthCheck{make_plate(), {}};
    auto pose = make_pose(0.);
    pose.translation().z() = -0.5;
    const auto observed = check.end_observed_ratios(make_depth(), kCamera, pose);
    EXPECT_DOUBLE_EQ(observed.closed, 0.);
    EXPECT_DOUBLE_EQ(observed.open, 0.);
}
