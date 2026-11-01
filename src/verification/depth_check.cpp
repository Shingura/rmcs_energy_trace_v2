#include "verification/depth_check.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "geometry/rasterize.hpp"

namespace rmcs_energy_trace_v2::verification {

namespace {

// 渲染轮廓：被 mesh 覆盖的像素，向内腐蚀
cv::Mat make_silhouette(const cv::Mat& model_depth, int erosion_px) noexcept {
    auto silhouette = cv::Mat{model_depth > 0.f};
    if (erosion_px > 0) {
        const auto size = 2 * erosion_px + 1;
        cv::erode(
            silhouette, silhouette,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size{size, size}));
    }
    return silhouette;
}

enum class PixelClass : std::uint8_t { kMissing, kSupported, kFreeSpace, kOccluded };

// 一个像素的“应该看到的深度”与实测深度比较
PixelClass classify_pixel(
    std::uint16_t measured, float model_depth, double depth_scale, float tolerance) noexcept {
    if (measured == 0)
        return PixelClass::kMissing;
    const auto difference = static_cast<float>(measured * depth_scale) - model_depth;
    if (difference > tolerance)
        return PixelClass::kFreeSpace;
    if (difference < -tolerance)
        return PixelClass::kOccluded;
    return PixelClass::kSupported;
}

} // namespace

DepthCheck::DepthCheck(geometry::Mesh mesh, const Config& config) noexcept
    : mesh_{std::move(mesh)}
    , config_{config} {
    auto lower = std::numeric_limits<float>::max();
    auto upper = std::numeric_limits<float>::lowest();
    for (const auto& triangle : mesh_.triangles) {
        for (const auto& vertex : triangle) {
            lower = std::min(lower, vertex.z());
            upper = std::max(upper, vertex.z());
        }
    }
    const auto half_length = mesh_.triangles.empty() ? 0.f : (upper - lower) / 2.f;

    triangle_end_.reserve(mesh_.triangles.size());
    for (const auto& triangle : mesh_.triangles) {
        const auto centroid_z = (triangle[0].z() + triangle[1].z() + triangle[2].z()) / 3.f;
        if (std::abs(centroid_z) <= half_length / 2.f)
            triangle_end_.push_back(End::kMiddle);
        else
            triangle_end_.push_back(centroid_z > 0.f ? End::kClosed : End::kOpen);
    }
}

DepthCheck::Quality DepthCheck::check(
    const cv::Mat& depth, const geometry::Camera& camera, const cv::Rect2i& roi,
    const cv::Mat& mask, const Eigen::Isometry3d& pose) const noexcept {
    auto quality = Quality{};
    const auto rect = roi & cv::Rect2i{0, 0, depth.cols, depth.rows};
    if (depth.type() != CV_16UC1 || rect.empty())
        return quality;
    if (!mask.empty() && (mask.type() != CV_8UC1 || mask.size() != depth.size()))
        return quality;

    const auto model = geometry::rasterize(mesh_, pose, camera, rect);
    const auto silhouette = make_silhouette(model.depth, config_.silhouette_erosion_px);

    // 掩码只决定一个像素能否算作支持或遮挡：掩码外是别的东西，深度相同或更近都说明不了位姿对错，
    // 不统计；但掩码外看到比 mesh 更远的背景，说明 mesh 所在处是空的，仍记为违例
    auto supported = std::size_t{}, free_space = std::size_t{}, occluded = std::size_t{},
         missing = std::size_t{};
    for (int v = 0; v < rect.height; v++) {
        const auto* model_row = model.depth.ptr<float>(v);
        const auto* silhouette_row = silhouette.ptr<std::uint8_t>(v);
        const auto* depth_row = depth.ptr<std::uint16_t>(v + rect.y);
        const auto* mask_row = mask.empty() ? nullptr : mask.ptr<std::uint8_t>(v + rect.y);
        for (int u = 0; u < rect.width; u++) {
            if (silhouette_row[u] == 0)
                continue;
            const auto in_mask = mask_row == nullptr || mask_row[u + rect.x] != 0;
            if (in_mask)
                quality.model_pixels++;
            switch (classify_pixel(
                depth_row[u + rect.x], model_row[u], camera.depth_scale, config_.depth_tolerance)) {
            case PixelClass::kMissing:
                if (in_mask)
                    missing++;
                continue;
            case PixelClass::kFreeSpace: free_space++; break;
            case PixelClass::kOccluded:
                if (!in_mask)
                    continue;
                occluded++;
                break;
            case PixelClass::kSupported:
                if (!in_mask)
                    continue;
                supported++;
                break;
            }
            quality.valid_pixels++;
        }
    }
    if (quality.model_pixels > 0) {
        quality.missing_ratio =
            static_cast<double>(missing) / static_cast<double>(quality.model_pixels);
    }
    if (quality.valid_pixels > 0) {
        const auto count = static_cast<double>(quality.valid_pixels);
        quality.support_ratio = static_cast<double>(supported) / count;
        quality.free_space_ratio = static_cast<double>(free_space) / count;
        quality.occluded_ratio = static_cast<double>(occluded) / count;
    }
    return quality;
}

cv::Rect2i DepthCheck::projected_box(
    const geometry::Camera& camera, const Eigen::Isometry3d& pose) const noexcept {
    return geometry::projected_box(mesh_, pose, camera)
         & cv::Rect2i{0, 0, camera.width, camera.height};
}

DepthCheck::EndObserved DepthCheck::end_observed_ratios(
    const cv::Mat& depth, const geometry::Camera& camera,
    const Eigen::Isometry3d& pose) const noexcept {
    auto result = EndObserved{};
    auto box = geometry::projected_box(mesh_, pose, camera);
    if (depth.type() != CV_16UC1 || box.empty())
        return result;
    // 渲染范围是完整投影（不与画面求交），四周留出腐蚀用的边，使画面外的轮廓也按同样的方式腐蚀
    const auto margin = config_.silhouette_erosion_px + 1;
    box =
        cv::Rect2i{box.x - margin, box.y - margin, box.width + 2 * margin, box.height + 2 * margin};

    const auto model = geometry::rasterize(mesh_, pose, camera, box);
    const auto silhouette = make_silhouette(model.depth, config_.silhouette_erosion_px);

    // 下标 0 为封闭端，1 为开口端
    auto total = std::array<std::size_t, 2>{};
    auto observed = std::array<std::size_t, 2>{};
    for (int v = 0; v < box.height; v++) {
        const auto* model_row = model.depth.ptr<float>(v);
        const auto* triangle_row = model.triangle.ptr<int>(v);
        const auto* silhouette_row = silhouette.ptr<std::uint8_t>(v);
        const auto image_v = v + box.y;
        for (int u = 0; u < box.width; u++) {
            if (silhouette_row[u] == 0)
                continue;
            const auto part = triangle_end_[static_cast<std::size_t>(triangle_row[u])];
            if (part == End::kMiddle)
                continue;
            const auto end = part == End::kClosed ? 0 : 1;
            total[end]++;
            const auto image_u = u + box.x;
            if (image_u < 0 || image_u >= depth.cols || image_v < 0 || image_v >= depth.rows)
                continue;
            const auto kind = classify_pixel(
                depth.at<std::uint16_t>(image_v, image_u), model_row[u], camera.depth_scale,
                config_.depth_tolerance);
            if (kind == PixelClass::kSupported || kind == PixelClass::kFreeSpace)
                observed[end]++;
        }
    }
    if (total[0] > 0)
        result.closed = static_cast<double>(observed[0]) / static_cast<double>(total[0]);
    if (total[1] > 0)
        result.open = static_cast<double>(observed[1]) / static_cast<double>(total[1]);
    return result;
}

} // namespace rmcs_energy_trace_v2::verification
