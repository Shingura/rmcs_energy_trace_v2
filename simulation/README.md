# 生成训练集与回放序列

## 环境与目录约定

本文使用 Linux x86_64 宿主机和 RMCS 开发容器。宿主机负责 Blender 渲染、标签转换和网络训练，容器负责 MoveIt 规划和离线回放。

我们假设宿主机使用 NVIDIA 显卡，并已安装了驱动。

回放序列依赖完整的 [RMCS 工作区](https://github.com/Shingura/RMCS/tree/dev/auto_mining)，请执行以下命令 clone 对应的分支

```bash
mkdir -p "$HOME/workspaces"
git clone --branch dev/auto_mining --recurse-submodules \
    https://github.com/Shingura/RMCS.git "$HOME/workspaces/RMCS"
```

请确保 `rmcs_ws/src/` 中存在下面三个文件夹：

```text
RMCS/rmcs_ws/src/
├── rmcs_energy_trace_v2/
├── arm_description/        # 用于导入机械臂模型文件进行仿真
└── arm_moveit_config/      # 用于使用 moveit 规划机械臂运动
```

本文假设宿主机上的 RMCS 位于 `$HOME/workspaces/RMCS`。容器内同一目录挂载为 `/workspaces/RMCS`。请按实际位置修改宿主机路径，不要把宿主机绝对路径直接用于容器命令。

## 准备宿主机

在宿主机终端执行：

```bash
cd "$HOME/workspaces/RMCS/rmcs_ws/src/rmcs_energy_trace_v2"

sudo apt-get update
sudo apt-get install -y curl xz-utils python3-venv

bash simulation/setup_host.sh
bash simulation/blender/download_hdri.sh

BLENDER="$HOME/tools/blender-4.5.15-linux-x64/blender"
PYTHON="$HOME/venvs/yolo/bin/python"
```

[setup_host.sh](setup_host.sh) 会安装 Blender 4.5.15，下载 YOLO11n-seg 预训练权重，并创建训练与导出网络用的 Python 虚拟环境。

脚本固定安装 PyTorch 2.11.0（CUDA 12.8）、torchvision 0.26.0、Ultralytics 8.3.251 和 OpenVINO 2026.4.1。PyTorch 使用的 CUDA 运行库由软件包安装，无需另装完整的 CUDA Toolkit；宿主机驱动仍须兼容 CUDA 12.8。

[download_hdri.sh](blender/download_hdri.sh) 下载 13 张室内 HDRI（高动态范围环境贴图），用于随机背景和环境光。素材来自 [Poly Haven](https://polyhaven.com/hdris)，采用 CC0 许可。目录中没有 `.hdr` 文件时，渲染器使用深色背景。

首次安装需要下载 Blender、PyTorch、CUDA 运行库和环境贴图，请预留下载与解压空间。脚本失败后可以重新执行。默认位置如下：

| 内容 | 位置 |
| --- | --- |
| Blender | `~/tools/blender-4.5.15-linux-x64/` |
| Python 虚拟环境 | `~/venvs/yolo/` |
| 预训练权重 | `dataset/inputs/pretrained/yolo11n-seg.pt` |
| 环境贴图 | `dataset/inputs/hdri/` |

可用 `setup_host.sh --tools-dir <目录> --venv <目录>` 修改工具位置。修改后，同步调整 `BLENDER`、`PYTHON`，并在训练时传入 `--venv`。

下文的宿主机命令都在本包根目录执行，并使用上述变量。

## 生成训练集

### 渲染图片与编号图

在宿主机执行：

```bash
DATASET="dataset/detector/local_v1"

"$BLENDER" --background --factory-startup \
    --python simulation/blender/render_dataset.py -- \
    --out "$DATASET/render" \
    --count 2000 --start 0 --seed 0 \
    --width 1280 --height 720 --samples 32 --device OPTIX
```

首次使用时，可把 `--count 2000` 改为 `--count 10`，先尝试渲染 10 帧，检查输出后再改回 2000 进行完整渲染。渲染器按 `meta/` 中的文件跳过已完成的帧，因此相同命令可以继续未完成的训练集渲染。

每帧重新随机生成场景、材质、光照、相机位置和图像效果。输出目录中的文件按六位帧号对应：

| 目录 | 内容 |
| --- | --- |
| `render/images/` | 彩色 JPG 图片 |
| `render/index/` | 单元编号 PNG，像素值表示可见的单元编号，0 表示其他物体 |
| `render/full/` | 移除其他物体后渲染的单元编号 PNG，用于计算可见比例 |
| `render/meta/` | 相机内参、单元位姿、随机种子和图像效果 |

训练图片不包含深度图。需要 RGB-D 数据时，使用下文的回放序列流程。

`--start` 是起始帧号，`--count` 是本次覆盖的帧数。例如，`--start 2000 --count 1000` 会生成第 2000 至第 2999 帧。保持种子不变即可继续扩充同一组数据。

更改种子、分辨率、场景或渲染配置时，使用新的输出目录，避免跳过旧帧后混用两种配置。本文用 `local_v1` 作为示例名称。

### 转换为 YOLO 实例分割数据集

渲染完成后，在宿主机执行：

```bash
"$PYTHON" -I simulation/training/make_yolo_dataset.py \
    "$DATASET/render" "$DATASET/yolo" \
    --val 0.1 --min-pixels 300 --min-visible 0.2 --preview 20
```

[make_yolo_dataset.py](training/make_yolo_dataset.py) 按单元编号提取可见轮廓，生成类别为 `energy_unit` 的分割多边形。可见像素少于 300，或可见比例低于 20% 的单元将不会被标注。

`--val 0.1` 按帧号散列划分约 10% 的验证图片，其余用于训练。输出结构如下：

```text
local_v1/yolo/
├── data.yaml
├── images/
│   ├── train/
│   └── val/
├── labels/
│   ├── train/
│   └── val/
└── preview/
```

训练完成后，终端会输出训练帧数、验证帧数和标注数量，可打开 `preview/` 中的图片检查轮廓。

转换脚本可以在补渲染后重新执行。若要减少帧数或更换划分规则，请使用新的 YOLO 输出目录，避免留下旧文件。

### 训练与导出网络（可选）

只需要训练集时，到标签转换这一步即可。需要训练网络时，在宿主机继续执行：

```bash
bash simulation/training/train_yolo.sh \
    "$DATASET/yolo" "$DATASET/train"

"$PYTHON" -I simulation/training/export_yolo.py \
    "$DATASET/train/weights/best.pt" "$DATASET/network"
```

[train_yolo.sh](training/train_yolo.sh) 默认训练 100 轮，输入尺寸为 640，batch 为 16。连续 30 轮没有提升时将会提前停止。显存不足时，可更改 `--batch 8`。

训练权重位于 `train/weights/best.pt`，日志位于 `train/train.log`。

[export_yolo.py](training/export_yolo.py) 将网络导出到 `network/best_openvino_model/`，其中的 `best.xml` 和 `best.bin` 供 C++ 检测器使用。导出参数与当前解码器一致，不要自行启用半精度或内置 NMS（非极大值抑制）。

训练脚本会复用指定的输出目录，导出脚本会替换同名导出目录。保留已有结果时，请换用新的目录。

## 生成回放序列

回放序列按机械臂轨迹逐帧生成。先在容器中规划观察、接近、夹持和拉出动作，再由宿主机上的 Blender 渲染彩色图、深度图和位姿真值。

### 在容器中安装依赖并构建

在 RMCS 开发容器的 Bash 终端执行：

```bash
cd /workspaces/RMCS/rmcs_ws/src/rmcs_energy_trace_v2
bash scripts/setup_container.sh

cd /workspaces/RMCS/rmcs_ws
source /opt/ros/jazzy/setup.bash
colcon build --merge-install \
    --packages-up-to rmcs_energy_trace_v2 arm_description arm_moveit_config \
    --cmake-args -DRMCS_ENERGY_TRACE_V2_BUILD_M3T_TOOLS=OFF
source install/setup.bash
```

这里构建了本包及其工作区依赖，同时安装机械臂描述和 MoveIt 配置。`m3t_generate` 与本流程无关，因此关闭其构建。生成训练集和规划文件都不需要下载已有的检测网络或 M3T 文件。

如果构建输出包含 `MoveIt not found, skip pickup_plan`，表明规划工具未生成，请在确认依赖安装成功后重新构建。（如果此前已构建过，你可能还需要清理 CMake 缓存并重新编译）

### 生成取矿规划

在同一个容器终端执行：

```bash
ros2 launch rmcs_energy_trace_v2 pickup_plan.launch.py \
    use_rviz:=false \
    output_dir:=/workspaces/RMCS/rmcs_ws/src/rmcs_energy_trace_v2/dataset/scenarios/local_v1
```

默认不启动 RViz，也不连接或控制真实机械臂。规划完成后，整个 launch 会退出。检查输出目录中是否有下面三份文件：

```text
dataset/scenarios/local_v1/
├── plan_slot1_seed0/plan.json
├── plan_slot0_seed1/plan.json
└── plan_slot4_seed2/plan.json
```

规划参数在 [pickup_plan.yaml](planning/pickup_plan.yaml) 中。`seeds` 和 `target_slots` 按位置配对，长度必须相同。默认配置生成上述三组场景，帧率为 30 fps，相机分辨率为 1280 × 720。`camera.*` 描述仿真相机的安装和成像参数，实机使用时需另行配置。

修改 YAML 后，重新构建本包，再运行规划。[scene.json](scene.json) 保存规划与渲染共用的中心块尺寸，数值单位为米。

规划器含随机搜索和限时求解，相同种子不保证得到相同轨迹。需要复用同一动作时，保留生成的 `plan.json`。重新规划时换用新的输出目录，避免覆盖已有规划后继续使用旧序列。

### 在 RViz 中回放规划（可选）

生成 `plan.json` 后，可以直接在 RViz 中查看取矿动作，无需先渲染 Blender 序列。`pickup_rviz_replay` 读取规划和机械臂 URDF，显示地面、中心块、安装柱、能量单元、机械臂和相机视场。

容器需要访问宿主机显示器。先在宿主机终端授权当前用户：

```bash
xhost +SI:localuser:$(whoami)
```

在容器的 Bash 终端执行：

```bash
cd /workspaces/RMCS/rmcs_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

PKG="/workspaces/RMCS/rmcs_ws/src/rmcs_energy_trace_v2"
env -u WAYLAND_DISPLAY QT_QPA_PLATFORM=xcb DISPLAY=:0 \
    ros2 launch rmcs_energy_trace_v2 pickup_rviz_replay.launch.py \
    plan:="$PKG/dataset/scenarios/local_v1/plan_slot1_seed0/plan.json"
```

示例使用宿主机的 X11 显示器 `:0`。若宿主机的 `DISPLAY` 不同，请替换该值。

回放默认以原速循环播放。橙色单元为目标，夹持前固定在中心块上，夹持后随夹爪拉出。底座始终使用规划中的固定位置。界面显示当前动作阶段和帧号。

[启动文件](rviz/pickup_rviz_replay.launch.py) 支持以下参数。在启动命令末尾用 `参数名:=值` 指定：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `plan` | 必填 | 回放使用的 `plan.json` 路径 |
| `urdf` | `arm_description` 包中的 `urdf/arm_description.urdf` | 指定机械臂 URDF 文件 |
| `scene` | 包内的 `simulation/scene.json` | 中心块尺寸，需与生成规划时一致 |
| `speed` | `1.0` | 播放速度，`0.5` 表示半速，必须大于 0 |
| `loop` | `true` | 是否循环，`false` 表示播放完后停在最后一帧 |
| `autoplay` | `true` | 是否立即播放，`false` 表示停在第一帧 |
| `use_rviz` | `true` | 是否启动 RViz 窗口 |

例如，指定现有 URDF、半速播放一次：

```bash
env -u WAYLAND_DISPLAY QT_QPA_PLATFORM=xcb DISPLAY=:0 \
    ros2 launch rmcs_energy_trace_v2 pickup_rviz_replay.launch.py \
    plan:="$PKG/dataset/scenarios/local_v1/plan_slot1_seed0/plan.json" \
    urdf:="/workspaces/RMCS/rmcs_ws/src/arm_description/urdf/arm_description.urdf" \
    speed:=0.5 loop:=false
```

需要暂停、继续或从头播放时，在另一个已加载 ROS 环境的容器终端执行：

```bash
# 暂停。
ros2 service call /pickup/set_playing std_srvs/srv/SetBool '{data: false}'

# 回到第一帧，保持当前播放或暂停状态。
ros2 service call /pickup/restart std_srvs/srv/Trigger '{}'

# 继续播放。
ros2 service call /pickup/set_playing std_srvs/srv/SetBool '{data: true}'
```

在启动回放的终端按 `Ctrl+C` 即可关闭节点和 RViz。

### 在宿主机渲染 RGB-D 序列

切回准备过 `BLENDER` 变量的宿主机终端，确认当前目录为本包根目录，然后执行：

```bash
SCENARIOS="dataset/scenarios/local_v1"

for scenario in plan_slot1_seed0 plan_slot0_seed1 plan_slot4_seed2; do
    "$BLENDER" --background --factory-startup \
        --python simulation/blender/render_sequence.py -- \
        --plan "$SCENARIOS/$scenario/plan.json" \
        --out "$SCENARIOS/$scenario/sequence" \
        --seed "${scenario##*seed}" --samples 32 --device OPTIX
done
```

宿主机直接读取工作区中的规划文件，无需复制容器内的绝对路径。[render_sequence.py](blender/render_sequence.py) 从 `plan.json` 读取相机参数、每帧关节角和夹持状态。渲染用的 `--seed` 决定材质、光照和背景，不负责生成机械臂轨迹。

每个场景的输出结构如下：

```text
plan_slot1_seed0/
├── plan.json
└── sequence/
    ├── sequence.json
    ├── color/
    ├── depth/
    ├── index/
    └── meta/
```

`color/` 保存 JPG 彩色图，`depth/` 保存 16 位无符号 PNG 深度图，单位为毫米，0 表示无效深度。`index/` 保存单元编号图，`meta/` 保存每帧的位姿真值、相机到世界的变换、关节角、动作阶段和夹持状态。

`sequence.json` 记录帧数、帧率、相机内参和目标编号。帧号从 `000000` 开始，四个目录中同一帧的文件名对应。确认文件数量与记录的帧数一致，并查看彩色图和深度图是否能读取。

当前序列渲染器会跳过已有帧，但不会恢复这些帧的夹持状态。连续序列中断后，请换用空的输出目录完整重渲染。更换规划、种子或场景配置后，也需使用空目录，避免混用旧帧。

## 运行 RGB-D 序列回放（可选）

生成序列后，可以用现有网络检查数据能否读入位姿估计流程，无需先完成训练。在容器的 Bash 终端执行：

```bash
cd /workspaces/RMCS/rmcs_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

PKG="/workspaces/RMCS/rmcs_ws/src/rmcs_energy_trace_v2"
bash "$PKG/scripts/download_assets.sh"

ros2 run rmcs_energy_trace_v2 sequence_replay \
    "$PKG/dataset/scenarios/local_v1/plan_slot1_seed0/sequence" \
    "$PKG/dataset/replay/local_v1/plan_slot1_seed0" \
    --network "$PKG/assets/yolo11n_seg_v1.xml" \
    --threads 4 --fps 0
```
回放目录中，将会生成 `frames.csv`、`summary.txt` 和叠加画面 `replay.mp4`。

[download_assets.sh](../scripts/download_assets.sh) 下载已有检测网络与 M3T 文件。若要使用自己训练的网络，将 `--network` 改为导出的 `best.xml` 路径，并保留同目录下的 `best.bin`。


