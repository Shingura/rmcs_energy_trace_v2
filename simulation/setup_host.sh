#!/usr/bin/env bash
# 在宿主机（有 NVIDIA 显卡、不需要 ROS）上准备仿真的渲染和训练环境。已完成的步骤会跳过，所以可以重复运行。
#   1. Blender 4.5.15 LTS 便携版：下载到 <工具目录>/blender-4.5.15-linux-x64/，校验 SHA256
#   2. 训练和导出检测网络用的 Python 虚拟环境：PyTorch（CUDA 12.8）、Ultralytics、OpenVINO，版本固定，
#      并关闭 Ultralytics 的使用统计上传
#   3. YOLO11n-seg 的官方预训练权重：下载到包内的 dataset/inputs/pretrained/，校验 SHA256
#
# 用法：simulation/setup_host.sh [--tools-dir <目录>] [--venv <目录>]
#   --tools-dir  Blender 放在哪里，默认 ~/tools
#   --venv       虚拟环境的目录，默认 ~/venvs/yolo
# 需要 python3 的 venv 模块（Ubuntu 上是 python3-venv 包）。任何一步失败时立即退出，退出码非 0。

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
package_dir=$(dirname "${script_dir}")
tools_dir="${HOME}/tools"
venv_dir="${HOME}/venvs/yolo"

while (($# > 0)); do
    case "$1" in
    --tools-dir)
        tools_dir=$2
        shift 2
        ;;
    --venv)
        venv_dir=$2
        shift 2
        ;;
    -h | --help)
        sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
    *)
        echo "unknown option: $1" >&2
        exit 2
        ;;
    esac
done

blender_version=4.5.15
blender_name="blender-${blender_version}-linux-x64"
blender_url="https://download.blender.org/release/Blender4.5/${blender_name}.tar.xz"
blender_sha256=0d2b8bb2efe97e6c8eb57de6b7322f8800e1984e3125b60decca5f6771e4773e

pretrained_url=https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-seg.pt
pretrained_sha256=55ed65c56c91713d23e8402371c6c49a6fd84f257f7dce452e8d70e41dcbe152

# 下载到 <目标>.part，校验通过后再改名，中断或校验失败时不留下看似完整的文件
download() {
    local url=$1 sha256=$2 target=$3
    curl --fail --location --retry 3 --progress-bar --output "${target}.part" "${url}"
    if ! echo "${sha256}  ${target}.part" | sha256sum --check --status; then
        echo "checksum mismatch: ${url}" >&2
        rm -f "${target}.part"
        return 1
    fi
    mv "${target}.part" "${target}"
}

# ---- 1. Blender ----
if [[ -x "${tools_dir}/${blender_name}/blender" ]]; then
    echo "ok        Blender ${blender_version}"
else
    echo "download  Blender ${blender_version}"
    mkdir -p "${tools_dir}"
    archive="${tools_dir}/${blender_name}.tar.xz"
    download "${blender_url}" "${blender_sha256}" "${archive}"
    tar -xf "${archive}" -C "${tools_dir}"
    rm "${archive}"
fi

# ---- 2. 虚拟环境 ----
if [[ ! -x "${venv_dir}/bin/python" ]]; then
    echo "create    ${venv_dir}"
    python3 -m venv "${venv_dir}"
fi
echo "install   Python packages into ${venv_dir}"
"${venv_dir}/bin/pip" install --quiet torch==2.11.0 torchvision==0.26.0 \
    --index-url https://download.pytorch.org/whl/cu128
"${venv_dir}/bin/pip" install --quiet ultralytics==8.3.251 openvino==2026.4.1
"${venv_dir}/bin/yolo" settings sync=False >/dev/null

# ---- 3. 预训练权重 ----
pretrained="${package_dir}/dataset/inputs/pretrained/yolo11n-seg.pt"
if [[ -f "${pretrained}" ]] && echo "${pretrained_sha256}  ${pretrained}" | sha256sum --check --status; then
    echo "ok        yolo11n-seg.pt"
else
    echo "download  yolo11n-seg.pt"
    mkdir -p "$(dirname "${pretrained}")"
    download "${pretrained_url}" "${pretrained_sha256}" "${pretrained}"
fi

echo "blender:  ${tools_dir}/${blender_name}/blender"
echo "python:   ${venv_dir}/bin/python"
