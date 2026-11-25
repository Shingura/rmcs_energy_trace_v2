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

脚本先用 `rosdep` 安装 `package.xml` 声明的系统和 ROS 依赖，再安装 MoveIt、RViz2、仿真回放消息与 TF 依赖、RealSense D435i 驱动，以及编译 `m3t_generate` 需要的 OpenGL 和 X11 开发包。执行时需要网络和 sudo 权限。

## 构建与测试

在 RMCS 工作区中：

```zsh
build-rmcs --packages-select rmcs_energy_trace_v2
colcon test --packages-select rmcs_energy_trace_v2
```

生成 M3T 模型的工具需要 OpenGL 和显示器，默认不构建。需要时加 `--cmake-args -DRMCS_ENERGY_TRACE_V2_BUILD_M3T_TOOLS=ON`。

开发约定见 `AGENTS.md`。

## 实车部署

在本机编译并同步产物后，后续操作都在 `ssh-remote` 打开的远程终端中执行。

### 1. 连接测试车并同步产物

1. 在本机连接测试车并同步编译产物：

```bash
build-rmcs --packages-up-to rmcs_core rmcs_bringup rmcs_energy_trace_v2
set-remote alliance-engineer.local
sync-remote
```

等待 `sync-remote` 输出绿色的 `Nothing to do` 后，继续下一步。

2. 另开一个本机终端登录测试车：

```bash
ssh-remote
```

### 2. 检查并安装测试车运行依赖

1. 在远程终端检查动态库、RealSense 驱动和相机 USB 连接：

```bash
ldd /rmcs_install/lib/librmcs_core.so /rmcs_install/lib/librmcs_energy_trace_v2.so | grep "not found" || echo "all libs ok"
ros2 pkg prefix realsense2_camera
lsusb | grep -i intel || echo "no Intel USB device"
```

依赖正常时，`ldd` 输出 `all libs ok`（没有 `not found` 行），`ros2 pkg prefix` 输出 `/opt/ros/jazzy`，`lsusb` 列出 Intel RealSense 设备。

2. 若提示缺库或找不到 `realsense2_camera`，在远程终端补装运行依赖：

```bash
apt-get update && apt-get install -y \
  libopencv-contrib406t64 \
  ros-jazzy-realsense2-camera \
  ros-jazzy-librealsense2 \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-moveit-task-constructor-msgs
```

安装完成后，再运行一次第 1 步的检查命令。新装 `ros-jazzy-compressed-image-transport` 后，需要重启一次 `service rmcs` 才会启用相机压缩话题。

### 3. 启动程序并查看终端日志

在远程终端记录当前配置名（方便测完恢复），切换到视觉取矿测试配置，然后重启服务并附着输出：

```bash
grep RMCS_ROBOT_TYPE ~/env_setup.bash
set-robot lunar_rover_energy_trace
service rmcs restart && service rmcs attach
```

需要退出查看但不停机时，先按 `Ctrl+A`，松开后再按 `D`。需要停止服务时，在远程终端运行 `service rmcs stop`。

启动正常时，终端输出以下内容（示例省略日志中的时间戳片段）：

- `rmcs_executor` 正常加载各组件，没有缺少输入接口的报错。未加载兑矿组件时，`arm_controller` 每秒打印一次 `pillar_pose not ready` 警告，属于正常情况。
- `energy_trace` 打印资产路径和外参配置，收到相机首帧后打印内参（数值随具体设备略有差异）与初始化完成消息，随后每秒打印一次空闲状态：

```text
[INFO] [energy_trace]: network='/rmcs_install/share/rmcs_energy_trace_v2/assets/yolo11n_seg_v1.xml' mesh='/rmcs_install/share/rmcs_energy_trace_v2/meshes/energy_unit.stl' model_dir='/rmcs_install/share/rmcs_energy_trace_v2/assets'
[INFO] [energy_trace]: camera_translation=[0.0000, 0.0000, 0.0000] camera_rpy=[0.0000, 0.0000, 0.0000]
[INFO] [energy_trace]: CameraInfo: 640x480, fx=606.295471 fy=605.189941 cx=317.442383 cy=255.065521, depth_scale=0.001 m/unit
[INFO] [energy_trace]: Target estimator initialized, 640x480
[INFO] [energy_trace]: status=idle valid=0
```

未加载兑矿组件时，`arm_controller` 每秒打印一次 `pillar_pose not ready` 占满屏幕。另开一个远程终端过滤 `/rosout`，可以只看 `energy_trace` 的日志：

```bash
ros2 topic echo /rosout --filter "m.name == 'energy_trace'" --full-length
```

### 4. 查看相机画面、检测叠加图和三维位姿

车上容器默认设置 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，开发电脑默认收不到车上的 ROS 2 话题。相机默认以 640×480、30 Hz 输出彩色图和对齐深度图。

**方式一：使用 Foxglove Studio 查看图像和三维场景**

1. 在远程终端启动检测图压缩节点（把 `/energy_trace/detection_image` 转成 JPEG 压缩话题）：

```bash
ros2 run image_transport republish --ros-args \
  -p in_transport:=raw \
  -p out_transport:=compressed \
  -r in:=/energy_trace/detection_image \
  -r out/compressed:=/energy_trace/detection_image/compressed
```

2. 另开一个远程终端启动 `foxglove_bridge`（限制发送缓冲以降低延迟，并关闭参数订阅避免 MoveIt 未初始化参数报错）：

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:=8765 \
  max_qos_depth:=1 \
  send_buffer_limit:=1000000 \
  capabilities:="[clientPublish,services,connectionGraph,assets]"
```

3. 在开发电脑打开 Foxglove Studio，连接地址填写 `ws://<robot_ip>:8765`。
4. 在对应面板订阅下表中的话题（3D 面板把 Fixed Frame 设为 `base_link`，并开启 TF 与 `/robot_description` 显示）：

| 面板类型 | 话题 | 预期画面 |
| --- | --- | --- |
| Image | `/camera/camera/color/image_raw/compressed` | 压缩彩色画面 |
| Image | `/camera/camera/color/image_raw` | 原始彩色画面（带宽占用较大） |
| Image | `/camera/camera/aligned_depth_to_color/image_raw` | 对齐深度图，物体边缘与彩色画面重合（带宽占用较大） |
| Image | `/energy_trace/detection_image/compressed` | 压缩检测叠加图，检出目标覆盖半透明青色掩码和绿色矩形框，框上方标有 3 位小数得分，无目标时显示原图 |
| Image | `/energy_trace/detection_image` | 原始检测叠加图，带宽占用较大 |
| 3D | `/energy_trace/pose` | `base_link` 坐标系下的目标位姿坐标轴，跟踪有效或就绪时显示 |
| 3D | `/energy_trace/marker` | `arm_6dof` 机械臂旁的能量单元 mesh，初始化有效时呈琥珀色半透明，进入 `ready` 后变为绿色并固定位置，复位或失败时消失 |

**方式二：用命令行检查话题输出**

不方便使用图形界面时，在另一个远程终端检查话题列表与帧率：

```bash
ros2 topic list | grep -E 'camera|energy_trace'
ros2 topic hz /camera/camera/color/image_raw /camera/camera/aligned_depth_to_color/image_raw
```

### 5. 测试整车控制与遥控器开关

操作前确认遥控器摇杆居中，测试车处于空旷地面或已架空。

1. 把遥控器拨到双下（`DOWN && DOWN`）完成上电解锁，此时整车保持失能复位。
2. 保持摇杆居中，把左拨杆拨到上，进入左上右下（`UP && DOWN`）触发视觉估计。

测试底盘、腿部和机械臂的整车控制时，各拨杆组合对应的模式和摇杆操作见下表：

| 左拨杆 | 右拨杆 | 模式与摇杆操作 |
| --- | --- | --- |
| `DOWN` | `DOWN` | 上电解锁，解锁后再次拨到双下（或遥控器断联）整车失能并复位视觉 |
| `MIDDLE` | `MIDDLE` | 底盘跟随（高速档）+ 腿部四轮模式。左摇杆平移，右摇杆 y 轴控大 yaw |
| `MIDDLE` | `DOWN` | 底盘跟随（高速档）+ 腿部六轮模式。左摇杆平移，右摇杆 y 轴控大 yaw |
| `MIDDLE` | `UP` | 底盘小陀螺（高速档）+ 腿部四轮模式。左摇杆平移，右摇杆 y 轴控大 yaw |
| `UP` | `UP` | 机械臂位置控制（`joint_1`～`joint_3`），底盘失能。左 y 控 `joint_1`，右 x 控 `joint_2`，左 x 控 `joint_3` |
| `UP` | `MIDDLE` | 机械臂姿态控制（`joint_4`～`joint_6`），底盘失能。右 y 控 `joint_4`，左 x 控 `joint_5`，左 y 控 `joint_6` |
| `DOWN` | `MIDDLE` | 夹爪控制，底盘失能。右摇杆 `y > 0.8` 打开，`y < -0.8` 闭合 |
| `DOWN` | `UP` | 键鼠与拨轮模式 |
| `UP` | `DOWN` | 触发视觉估计，底盘失能 |

**安全提示**：左拨杆从 `DOWN` 拨到 `UP` 会短暂经过 `MIDDLE && DOWN`。底盘和腿部控制器在该档位会使能，切换前必须确认摇杆居中。

触发后，在日志终端（`service rmcs attach` 或 `/rosout` 过滤终端）可以看到以下日志：

- 相机已出图时，先打印启动请求和初始化状态，配准和跟踪有效时附带坐标与姿态：

```text
[INFO] [energy_trace]: Estimator start requested by remote
[INFO] [energy_trace]: status=initializing valid=0
[INFO] [energy_trace]: status=initializing valid=1 t=[0.5123, -0.0234, 0.3456] z=[0.012, -0.005, 0.999] q=[0.707, 0.000, 0.707, 0.000]
```

- 视野内存在能量单元且连续 15 帧跟踪稳定后，状态变为就绪并锁定参考位姿：

```text
[INFO] [energy_trace]: status=ready valid=1 t=[0.5123, -0.0234, 0.3456] z=[0.012, -0.005, 0.999] q=[0.707, 0.000, 0.707, 0.000]
```

其中 `t` 是能量单元中心在 `base_link` 下的坐标（单位 m），`z` 是能量单元封闭端朝向的单位向量，`q` 是姿态四元数 `[w, x, y, z]`。进入 `ready` 后，即使后续目标被遮挡导致 `valid=0`，输出中的 `t`、`z` 和 `q` 仍保持锁定不变。

- 视野内没有能量单元，或者 3 s 内没有完成 15 帧确认时，状态变为失败：

```text
[INFO] [energy_trace]: status=failed valid=0
```

- 相机尚未出图时拨到 `UP && DOWN`，终端打印警告：

```text
[WARN] [energy_trace]: Estimator start ignored: waiting for first camera frame
```

3. 拨回双下（`DOWN && DOWN`）或关闭遥控器，复位视觉估计器。终端立即打印复位日志并回到空闲状态：

```text
[INFO] [energy_trace]: Estimator reset by remote
[INFO] [energy_trace]: status=idle valid=0
```

测试完成后，在远程终端恢复原有车型配置：

```bash
set-robot lunar_rover_engineer && service rmcs restart
```
