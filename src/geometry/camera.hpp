#pragma once

namespace rmcs_energy_trace_v2::geometry {

// 相机参数，坐标系为 OpenCV 约定：x 右、y 下、z 前，单位 m
struct Camera {
    int width = 640;
    int height = 480;
    // 针孔内参，单位为像素
    double fx = 600.;
    double fy = 600.;
    double cx = 320.;
    double cy = 240.;
    // 深度图（CV_16UC1，0 为无效）每个单位对应的米数，毫米深度图为 0.001
    double depth_scale = 0.001;
};

} // namespace rmcs_energy_trace_v2::geometry
