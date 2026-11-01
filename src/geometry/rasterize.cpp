#include "geometry/rasterize.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace rmcs_energy_trace_v2::geometry {

namespace {

// 三角形 (a, b, c) 的有向面积的两倍；c 在 ab 左侧时为正
float edge(const Eigen::Vector2f& a, const Eigen::Vector2f& b, const Eigen::Vector2f& c) noexcept {
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}

} // namespace

Rasterized rasterize(
    const Mesh& mesh, const Eigen::Isometry3d& pose, const Camera& camera,
    const cv::Rect2i& roi) noexcept {
    auto result = Rasterized{};
    result.roi = roi;
    if (roi.empty())
        return result;
    // 光栅化时没有覆盖的像素深度为无穷远，任何表面都比它近；最后再换成 0
    result.depth = cv::Mat{
        roi.height, roi.width, CV_32FC1, cv::Scalar{std::numeric_limits<double>::infinity()}};
    result.triangle = cv::Mat{roi.height, roi.width, CV_32SC1, cv::Scalar{-1}};

    const Eigen::Matrix3f rotation = pose.linear().cast<float>();
    const Eigen::Vector3f translation = pose.translation().cast<float>();
    const auto fx = static_cast<float>(camera.fx);
    const auto fy = static_cast<float>(camera.fy);
    const auto cx = static_cast<float>(camera.cx);
    const auto cy = static_cast<float>(camera.cy);
    constexpr float kNearPlane = 0.01f;
    // 像素中心恰好落在两个三角形的公共边上时，两边都算覆盖，避免拼接处漏掉像素
    constexpr float kEdgeTolerance = 1e-4f;

    for (std::size_t index = 0; index < mesh.triangles.size(); index++) {
        const auto& triangle = mesh.triangles[index];
        auto vertices = std::array<Eigen::Vector3f, 3>{};
        auto projected = std::array<Eigen::Vector2f, 3>{};
        auto visible = true;
        for (std::size_t k = 0; k < 3; k++) {
            vertices[k] = rotation * triangle[k] + translation;
            if (vertices[k].z() < kNearPlane) {
                visible = false;
                break;
            }
            projected[k] = {
                fx * vertices[k].x() / vertices[k].z() + cx,
                fy * vertices[k].y() / vertices[k].z() + cy};
        }
        if (!visible)
            continue;

        // 投影后退化成线段的三角形（侧面正对视线）不覆盖任何像素
        const auto area = edge(projected[0], projected[1], projected[2]);
        if (std::abs(area) < 1e-9f)
            continue;

        // 只遍历三角形投影的包围框与 roi 的交集
        const auto lo_x = std::min({projected[0].x(), projected[1].x(), projected[2].x()});
        const auto hi_x = std::max({projected[0].x(), projected[1].x(), projected[2].x()});
        const auto lo_y = std::min({projected[0].y(), projected[1].y(), projected[2].y()});
        const auto hi_y = std::max({projected[0].y(), projected[1].y(), projected[2].y()});
        const auto x0 = std::max(roi.x, static_cast<int>(std::ceil(lo_x)));
        const auto x1 = std::min(roi.x + roi.width - 1, static_cast<int>(std::floor(hi_x)));
        const auto y0 = std::max(roi.y, static_cast<int>(std::ceil(lo_y)));
        const auto y1 = std::min(roi.y + roi.height - 1, static_cast<int>(std::floor(hi_y)));

        for (int v = y0; v <= y1; v++) {
            auto* depth_row = result.depth.ptr<float>(v - roi.y);
            auto* triangle_row = result.triangle.ptr<int>(v - roi.y);
            for (int u = x0; u <= x1; u++) {
                // 重心坐标：三个权重都不为负时，像素中心在三角形内
                const Eigen::Vector2f pixel{static_cast<float>(u), static_cast<float>(v)};
                const auto w0 = edge(projected[1], projected[2], pixel) / area;
                const auto w1 = edge(projected[2], projected[0], pixel) / area;
                const auto w2 = 1.f - w0 - w1;
                if (w0 < -kEdgeTolerance || w1 < -kEdgeTolerance || w2 < -kEdgeTolerance)
                    continue;
                // 透视投影下，深度的倒数 1/z 在画面上是线性的，所以对 1/z 插值
                const auto inverse_z =
                    w0 / vertices[0].z() + w1 / vertices[1].z() + w2 / vertices[2].z();
                const auto z = 1.f / inverse_z;
                // z-buffer：只保留离相机最近的表面
                if (z < depth_row[u - roi.x]) {
                    depth_row[u - roi.x] = z;
                    triangle_row[u - roi.x] = static_cast<int>(index);
                }
            }
        }
    }

    result.depth.setTo(0.f, result.triangle < 0);
    return result;
}

cv::Rect2i
    projected_box(const Mesh& mesh, const Eigen::Isometry3d& pose, const Camera& camera) noexcept {
    if (mesh.triangles.empty())
        return {};
    const Eigen::Matrix3f rotation = pose.linear().cast<float>();
    const Eigen::Vector3f translation = pose.translation().cast<float>();
    auto lower =
        Eigen::Vector2f{std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    auto upper =
        Eigen::Vector2f{std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};
    for (const auto& triangle : mesh.triangles) {
        for (const auto& vertex : triangle) {
            const Eigen::Vector3f p = rotation * vertex + translation;
            if (p.z() <= 0.f)
                return {};
            const Eigen::Vector2f pixel{
                static_cast<float>(camera.fx) * p.x() / p.z() + static_cast<float>(camera.cx),
                static_cast<float>(camera.fy) * p.y() / p.z() + static_cast<float>(camera.cy)};
            lower = lower.cwiseMin(pixel);
            upper = upper.cwiseMax(pixel);
        }
    }
    const auto left = static_cast<int>(std::floor(lower.x()));
    const auto top = static_cast<int>(std::floor(lower.y()));
    const auto right = static_cast<int>(std::ceil(upper.x()));
    const auto bottom = static_cast<int>(std::ceil(upper.y()));
    return cv::Rect2i{left, top, right - left + 1, bottom - top + 1};
}

} // namespace rmcs_energy_trace_v2::geometry
