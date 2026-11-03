#!/usr/bin/env bash
# 在 RMCS 开发容器中安装机械臂和视觉开发工具的额外依赖。容器重建后需重新运行。
#   - 机械臂：MoveIt、可视化工具、Servo、示例资源、RViz，以及机械臂配置包的依赖
#   - 取矿规划 pickup_plan（simulation/planning/）：MoveIt 规划接口
#   - M3T 模型生成工具 m3t_generate：从源码编译 GLFW 需要的 X11 和 OpenGL 开发包
#
# 用法：bash scripts/setup_container.sh（在本包目录运行，需要网络和 sudo 权限）

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
workspace_src=$(dirname "$(dirname "${script_dir}")")

if [[ -z "${ROS_DISTRO:-}" ]]; then
    # ROS 环境脚本会读取未设置的变量，加载期间暂时关闭 nounset。
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
fi

sudo apt-get update
rosdep update
# warehouse_ros_mongo 没有 rosdep 键；它只用于保存规划场景，规划工具用不到
rosdep install --from-paths "${workspace_src}/arm_description" "${workspace_src}/arm_moveit_config" --ignore-src -y \
    --skip-keys warehouse_ros_mongo
sudo apt-get install -y \
    "ros-${ROS_DISTRO}-moveit" \
    "ros-${ROS_DISTRO}-moveit-visual-tools" \
    "ros-${ROS_DISTRO}-moveit-servo" \
    "ros-${ROS_DISTRO}-moveit-resources" \
    "ros-${ROS_DISTRO}-rviz2" \
    "ros-${ROS_DISTRO}-moveit-ros-planning-interface" \
    libgl-dev libglew-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
