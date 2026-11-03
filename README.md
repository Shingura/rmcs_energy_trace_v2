# rmcs_energy_trace_v2

RMCS 工程机器人的能量单元 6D 位姿估计。输入对齐到彩色图的 RGB-D 图像，输出底座坐标系下目标能量单元的位姿，以及这个位姿是否可信。

## 获取

在 RMCS 工作区的 `rmcs_ws/src/` 下克隆：

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
| `scripts/` | 开发依赖安装与运行所需文件的脚本 |
| `simulation/` | 仿真：Blender 渲染（`blender/`）、检测网络训练（`training/`）、MoveIt 取矿规划（`planning/`），中心块尺寸在 `scene.json` |
| `dataset/` | 本地数据，需自行下载：`inputs/` 外部获取的文件，`detector/<版本>/` 检测网络的训练流程，`scenarios/<名称>/` 规划和渲染序列 |

## 下载所需文件

dataset 文件夹下的检测网络（YOLO11n-seg，OpenVINO 格式）和 M3T 的 OBJ、区域模型、深度模型未放入仓库。运行下面的脚本，从 GitHub Release `assets-v2` 下载到包内的 `assets/`，并校验 SHA256：

```zsh
scripts/download_assets.sh
```

## 开发依赖

首次配置或重建 RMCS 开发容器后，在本包根目录执行：

```bash
bash scripts/setup_container.sh
```

脚本安装机械臂和视觉开发工具的额外依赖，包括 MoveIt、可视化工具、Servo、示例资源、RViz，以及 OpenGL 和 X11 开发包。执行时需要网络和 sudo 权限。

## 构建与测试

在 RMCS 工作区中：

```zsh
build-rmcs --packages-select rmcs_energy_trace_v2
colcon test --packages-select rmcs_energy_trace_v2
```

生成 M3T 模型的工具需要 OpenGL 和显示器，默认不构建。需要时加 `--cmake-args -DRMCS_ENERGY_TRACE_V2_BUILD_M3T_TOOLS=ON`。

开发约定见 `AGENTS.md`。
