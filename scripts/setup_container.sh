#!/usr/bin/env bash
# 在 RMCS 开发容器中安装本包的开发工具需要、而 RMCS 镜像里没有的依赖。容器重建后要重新运行。
#   - 取矿规划 pickup_plan（simulation/planning/）：MoveIt，以及 arm_moveit_config 启动 move_group 需要的包
#   - M3T 模型生成工具 m3t_generate：从源码编译 GLFW 需要的 X11 和 OpenGL 开发包
# 机器人上运行的组件不需要这些依赖。
#
# 用法：scripts/setup_container.sh（在容器内运行，会调用 sudo apt-get）

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
workspace_src=$(dirname "$(dirname "${script_dir}")")

if [[ -z "${ROS_DISTRO:-}" ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

sudo apt-get update
rosdep update
# warehouse_ros_mongo 没有 rosdep 键；它只用于保存规划场景，规划工具用不到
rosdep install --from-paths "${workspace_src}/arm_description" "${workspace_src}/arm_moveit_config" --ignore-src -y \
    --skip-keys warehouse_ros_mongo
sudo apt-get install -y \
    "ros-${ROS_DISTRO}-moveit-ros-planning-interface" \
    libgl-dev libglew-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
