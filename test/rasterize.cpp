#include <cmath>
#include <numbers>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "geometry/camera.hpp"
#include "geometry/mesh.hpp"
#include "geometry/rasterize.hpp"

using rmcs_energy_trace_v2::geometry::Camera;
using rmcs_energy_trace_v2::geometry::Mesh;
using rmcs_energy_trace_v2::geometry::projected_box;
using rmcs_energy_trace_v2::geometry::rasterize;

namespace {

constexpr Camera kCamera{
    .width = 320, .height = 240, .fx = 300., .fy = 300., .cx = 160., .cy = 120.};
const auto kImage = cv::Rect2i{0, 0, kCamera.width, kCamera.height};

// mesh 系 z = z0 平面上、以原点为中心、边长 2·half 的正方形，两个三角形
void add_square(Mesh& mesh, float half, float z0 = 0.f) {
    const Eigen::Vector3f a{-half, -half, z0}, b{half, -half, z0}, c{half, half, z0},
        d{-half, half, z0};
    mesh.triangles.push_back({a, b, c});
    mesh.triangles.push_back({a, c, d});
}

Eigen::Isometry3d translation(double x, double y, double z) {
    auto pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d{x, y, z};
    return pose;
}

} // namespace

// 正对相机、距离 0.5 m 的正方形：覆盖的像素深度都是 0.5 m，其余为 0 和 -1。
// 半边长 0.0505 m 投影为 30.3 px，边界落在像素之间，覆盖 u、v 都是 130～190
TEST(Rasterize, FrontalSquareHasConstantDepth) {
    auto mesh = Mesh{};
    add_square(mesh, 0.0505f);
    const auto result = rasterize(mesh, translation(0., 0., 0.5), kCamera, kImage);

    ASSERT_EQ(result.depth.type(), CV_32FC1);
    ASSERT_EQ(result.triangle.type(), CV_32SC1);
    EXPECT_EQ(cv::countNonZero(result.triangle >= 0), 61 * 61);
    for (int v = 0; v < kCamera.height; v++) {
        for (int u = 0; u < kCamera.width; u++) {
            const auto inside = u >= 130 && u <= 190 && v >= 90 && v <= 150;
            if (inside) {
                EXPECT_NEAR(result.depth.at<float>(v, u), 0.5f, 1e-6f);
                EXPECT_GE(result.triangle.at<int>(v, u), 0);
            } else {
                EXPECT_EQ(result.depth.at<float>(v, u), 0.f);
                EXPECT_EQ(result.triangle.at<int>(v, u), -1);
            }
        }
    }
}

// 倾斜 50° 的平面：每个像素的深度等于视线与平面交点的 z。
// 在画面上对 z 线性插值会差到毫米级，对 1/z 插值才精确
TEST(Rasterize, TiltedPlaneDepthIsPerspectiveCorrect) {
    auto mesh = Mesh{};
    add_square(mesh, 0.1f);
    auto pose = translation(0., 0., 0.5);
    pose.linear() = Eigen::AngleAxisd{50. * std::numbers::pi / 180., Eigen::Vector3d::UnitY()}
                        .toRotationMatrix();
    const auto result = rasterize(mesh, pose, kCamera, kImage);

    const Eigen::Vector3d normal = pose.linear().col(2);
    const auto offset = normal.dot(pose.translation());
    auto checked = 0;
    for (int v = 0; v < kCamera.height; v += 7) {
        for (int u = 0; u < kCamera.width; u += 7) {
            if (result.triangle.at<int>(v, u) < 0)
                continue;
            const Eigen::Vector3d ray{
                (u - kCamera.cx) / kCamera.fx, (v - kCamera.cy) / kCamera.fy, 1.};
            EXPECT_NEAR(result.depth.at<float>(v, u), offset / normal.dot(ray), 2e-6);
            checked++;
        }
    }
    EXPECT_GT(checked, 100);
}

// 两个正方形叠在一起：近的（0.4 m）挡住远的（0.5 m）。近的先画、远的后画，远的不能覆盖它
TEST(Rasterize, NearerSurfaceWins) {
    auto mesh = Mesh{};
    add_square(mesh, 0.03f, -0.1f); // 三角形 0、1，距相机 0.4 m
    add_square(mesh, 0.05f, 0.f);   // 三角形 2、3，距相机 0.5 m
    const auto result = rasterize(mesh, translation(0., 0., 0.5), kCamera, kImage);

    EXPECT_NEAR(result.depth.at<float>(120, 160), 0.4f, 1e-6f);
    EXPECT_LE(result.triangle.at<int>(120, 160), 1);
    // 近的正方形投影半边长 22.5 px，远的 30 px：中间的环只有远的
    EXPECT_NEAR(result.depth.at<float>(120, 186), 0.5f, 1e-6f);
    EXPECT_GE(result.triangle.at<int>(120, 186), 2);
}

// roi 可以超出画面：结果按 roi 存放，roi 内画面外的像素照样渲染
TEST(Rasterize, RoiCanExtendOutsideImage) {
    auto mesh = Mesh{};
    add_square(mesh, 0.0505f);
    // 正方形中心投影到像素 (0, 0)
    const auto pose = translation(-160. / 300. * 0.5, -120. / 300. * 0.5, 0.5);
    const auto roi = cv::Rect2i{-20, -10, 100, 60};
    const auto result = rasterize(mesh, pose, kCamera, roi);

    ASSERT_EQ(result.roi, roi);
    ASSERT_EQ(result.depth.size(), roi.size());
    // 像素 (u, v) 在结果中的位置是 (v - roi.y, u - roi.x)
    EXPECT_NEAR(result.depth.at<float>(0 - roi.y, 0 - roi.x), 0.5f, 1e-6f);
    EXPECT_NEAR(result.depth.at<float>(-5 - roi.y, -15 - roi.x), 0.5f, 1e-6f);
    EXPECT_EQ(result.depth.at<float>(40 - roi.y, 40 - roi.x), 0.f);
}

TEST(Rasterize, EmptyRoiGivesEmptyResult) {
    auto mesh = Mesh{};
    add_square(mesh, 0.05f);
    const auto result = rasterize(mesh, translation(0., 0., 0.5), kCamera, {});
    EXPECT_TRUE(result.depth.empty());
    EXPECT_TRUE(result.triangle.empty());
}

// 在相机后方的 mesh 不画，也没有投影框
TEST(Rasterize, MeshBehindCameraIsSkipped) {
    auto mesh = Mesh{};
    add_square(mesh, 0.05f);
    const auto pose = translation(0., 0., -0.5);
    const auto result = rasterize(mesh, pose, kCamera, kImage);
    EXPECT_EQ(cv::countNonZero(result.triangle >= 0), 0);
    EXPECT_TRUE(projected_box(mesh, pose, kCamera).empty());
}

// 投影框不与画面求交：中心在像素 (0, 0) 的正方形，左上部分在画面外
TEST(ProjectedBox, CanExtendOutsideImage) {
    auto mesh = Mesh{};
    add_square(mesh, 0.0505f);
    const auto pose = translation(-160. / 300. * 0.5, -120. / 300. * 0.5, 0.5);
    EXPECT_EQ(projected_box(mesh, pose, kCamera), (cv::Rect2i{-31, -31, 63, 63}));
    EXPECT_TRUE(projected_box(Mesh{}, pose, kCamera).empty());
}
