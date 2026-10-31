#!/usr/bin/env bash
# 在宿主机的显卡上训练能量单元的 YOLO11n-seg 分割网络。
# 起点是官方预训练权重 dataset/inputs/pretrained/yolo11n-seg.pt（simulation/setup_host.sh 下载），
# 数据集由 make_yolo_dataset.py 生成。参数与 v1 网络相同：imgsz 640、100 轮、batch 16、
# 30 轮没有提升就提前停止、种子 0。训练完成后用 export_yolo.py 导出 <输出目录>/weights/best.pt。
#
# 用法：simulation/training/train_yolo.sh <数据集目录> <输出目录> [--venv <目录>] [--epochs 100] [--batch 16]
#   <数据集目录>  make_yolo_dataset.py 的输出，里面有 data.yaml，例如 dataset/detector/v2/yolo
#   <输出目录>    Ultralytics 的输出（权重、曲线、results.csv），例如 dataset/detector/v2/train；
#                 已存在时覆盖其中的同名文件。控制台输出另存为其中的 train.log
#   --venv        setup_host.sh 建的虚拟环境，默认 ~/venvs/yolo
#   --epochs、--batch  显存不够时把 batch 降到 8；改了这两个参数，结果就不再与 v1 可比
# 训练失败时退出码非 0。
# Ultralytics 开始训练前会检查混合精度：它下载 yolo11n.pt 到输出目录，并在 ~/runs/detect/ 下建一个空的
# predict* 目录。两者都与训练结果无关，可以删除。

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
package_dir=$(dirname "$(dirname "${script_dir}")")
venv_dir="${HOME}/venvs/yolo"
epochs=100
batch=16
positional=()

while (($# > 0)); do
    case "$1" in
    --venv)
        venv_dir=$2
        shift 2
        ;;
    --epochs)
        epochs=$2
        shift 2
        ;;
    --batch)
        batch=$2
        shift 2
        ;;
    -h | --help)
        sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
    -*)
        echo "unknown option: $1" >&2
        exit 2
        ;;
    *)
        positional+=("$1")
        shift
        ;;
    esac
done
if ((${#positional[@]} != 2)); then
    sed -n '7p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
    exit 2
fi

data="$(realpath "${positional[0]}")/data.yaml"
out_dir=$(realpath --canonicalize-missing "${positional[1]}")
pretrained="${package_dir}/dataset/inputs/pretrained/yolo11n-seg.pt"
yolo="${venv_dir}/bin/yolo"

for file in "${data}" "${pretrained}" "${yolo}"; do
    if [[ ! -e "${file}" ]]; then
        echo "not found: ${file}（预训练权重和虚拟环境由 simulation/setup_host.sh 准备）" >&2
        exit 1
    fi
done

mkdir -p "${out_dir}"
# Ultralytics 的输出目录是 project/name；project 必须是绝对路径，否则会放到 ~/runs/ 下。
# 在输出目录里运行，Ultralytics 运行时自动下载的文件也不会落到调用者的当前目录
cd "${out_dir}"
"${yolo}" segment train model="${pretrained}" data="${data}" \
    imgsz=640 epochs="${epochs}" batch="${batch}" patience=30 device=0 workers=8 seed=0 \
    project="$(dirname "${out_dir}")" name="$(basename "${out_dir}")" exist_ok=True \
    2>&1 | tee "${out_dir}/train.log"

echo "weights:  ${out_dir}/weights/best.pt"
