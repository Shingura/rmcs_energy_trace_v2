# rmcs_energy_trace_v2

RMCS 工程机器人的能量单元 6D 位姿估计。输入对齐到彩色图的 RGB-D 图像，输出底座坐标系下目标能量单元的位姿，以及这个位姿是否可信。

## 获取

本包是独立仓库，不是 RMCS 的子模块。在 RMCS 工作区的 `rmcs_ws/src/` 下克隆：

```zsh
cd rmcs_ws/src
git clone https://github.com/Shingura/rmcs_energy_trace_v2.git
```

## 目录

| 目录 | 内容 |
| --- | --- |
| `src/geometry/` | 相机参数、STL 读取与表面采样、深度图反投影为点云 |
| `src/detection/` | YOLO11 检测与分割网络的推理（OpenVINO）和输出解码 |
| `src/registration/` | 配准用的 point-to-plane 求解器 |
| `src/tracking/` | M3T 跟踪需要的 OBJ 导出和模型文件约定 |
| `src/synthetic/` | 把 mesh 渲染成合成深度图，供测试使用 |
| `src/component.cpp` | RMCS 组件 |
| `third_party/m3t/` | DLR 的 M3T 跟踪器，修改见其中的 README |
| `tool/` | `energy_unit_detect`（离线检测）、`m3t_generate`（生成 M3T 模型） |
| `test/` | 各模块的离线测试 |
| `scripts/` | 运行所需文件的下载脚本 |

## 运行所需文件

M3T 的 OBJ 和区域、深度模型不放进仓库。运行下面的脚本，下载到包内的 `assets/` 并校验 SHA256：

```zsh
scripts/download_assets.sh
```

文件清单和校验值在 `scripts/assets.sha256`。改了 mesh 或 M3T 生成参数后，用 `m3t_generate` 重新生成，并更新清单中的校验值。

## 构建与测试

在 RMCS 工作区中：

```zsh
build-rmcs --packages-select rmcs_energy_trace_v2
colcon test --packages-select rmcs_energy_trace_v2
```

生成 M3T 模型的工具需要 OpenGL 和显示器，默认不构建。需要时加 `--cmake-args -DRMCS_ENERGY_TRACE_V2_BUILD_M3T_TOOLS=ON`。

开发约定见 `AGENTS.md`。
