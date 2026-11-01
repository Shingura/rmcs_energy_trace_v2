#pragma once

#include <eigen3/Eigen/Geometry>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "geometry/camera.hpp"
#include "geometry/mesh.hpp"

namespace rmcs_energy_trace_v2::geometry {

// mesh 按位姿渲染出的深度图，只覆盖 roi 内的像素
struct Rasterized {
    cv::Rect2i roi; // 像素坐标，可以超出画面
    // CV_32FC1，roi 大小：每个像素上离相机最近的 mesh 表面的深度 (m)，没有被覆盖的像素为 0
    cv::Mat depth;
    // CV_32SC1，roi 大小：上述最近表面所在三角形在 mesh.triangles 中的下标，没有被覆盖的像素为 -1
    cv::Mat triangle;
};

// 把 mesh 按位姿 (mesh -> camera) 光栅化到 roi 内（z-buffer）。
// 像素 (u, v) 的中心对应投影坐标 (u, v)，与 depth_to_cloud 一致。
// 有顶点离相机不到 1 cm（包括在相机后方）的三角形不画
Rasterized rasterize(
    const Mesh& mesh, const Eigen::Isometry3d& pose, const Camera& camera,
    const cv::Rect2i& roi) noexcept;

// mesh 按位姿投影后的像素包围框，不与画面求交，可以超出画面。
// mesh 为空或有顶点在相机后方时为空
cv::Rect2i
    projected_box(const Mesh& mesh, const Eigen::Isometry3d& pose, const Camera& camera) noexcept;

} // namespace rmcs_energy_trace_v2::geometry
