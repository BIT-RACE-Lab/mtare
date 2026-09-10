#!/usr/bin/env bash
# Install catkin tools and the bundled Livox SDK2 on Ubuntu 20.04.
# 在 Ubuntu 20.04 上安装 catkin 工具与随仓库附带的 Livox SDK2。
set -euo pipefail
# Resolve the M-TARE workspace root from this script's location.
# 依据脚本自身位置推导 M-TARE 工作区根目录。
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
# Default number of parallel jobs. / 默认并行任务数。
JOBS=4
# Print command line usage. / 打印命令行用法。
usage() {
    echo 'Usage: ./scripts/install_dependencies.sh [-j N | --jobs N]'
    echo 'Requires Ubuntu 20.04.'
    echo 'Installs python3-catkin-tools and builds/installs the bundled Livox SDK2.'
}
# Parse command line options. / 解析命令行选项。
while [[ $# -gt 0 ]]; do
    case "$1" in
        -j|--jobs) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; JOBS="$2"; shift 2 ;; # Separate form. / 分开书写形式。
        -j[0-9]*) JOBS="${1#-j}"; shift ;; # Joined form such as -j8. / 连写形式，例如 -j8。
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done
# Validate the number of parallel jobs. / 校验并行任务数是否合法。
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || { echo 'Jobs must be positive' >&2; exit 2; }
# This script only supports Ubuntu 20.04 (matching ROS noetic/foxy).
# 本脚本仅支持 Ubuntu 20.04（与 ROS noetic/foxy 对应）。
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 20.04 ]] || {
    echo 'Use Ubuntu 20.04 for this script.' >&2; exit 1;
}
# The bundled Livox SDK2 sources must be present.
# 必须存在仓库内附带的 Livox SDK2 源码。
[[ -f "$ROOT/Livox-SDK2/CMakeLists.txt" ]] || {
    echo "Missing: $ROOT/Livox-SDK2/CMakeLists.txt" >&2; exit 1;
}
# Use sudo only when the script is not already running as root.
# 仅在非 root 用户下才使用 sudo。
privilege=()
if [[ $EUID -ne 0 ]]; then
    command -v sudo >/dev/null || { echo 'Run as root or install sudo.' >&2; exit 1; }
    privilege=(sudo)
fi
# Other build dependencies are provided by the existing container image.
# 其余编译依赖由现有容器镜像提供。
"${privilege[@]}" apt-get update
"${privilege[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-catkin-tools
# Compile as the invoking user; only system installation needs privilege.
# 以调用者身份编译，仅系统级安装需要提权。
cmake -S "$ROOT/Livox-SDK2" -B "$ROOT/Livox-SDK2/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/Livox-SDK2/build" --parallel "$JOBS"
"${privilege[@]}" cmake --install "$ROOT/Livox-SDK2/build" --prefix /usr/local
"${privilege[@]}" ldconfig
# Done; point the user to the next build step.
# 安装完成，提示用户下一步执行编译脚本。
echo 'Dependencies installed. Next: ./scripts/build.sh -j '"$JOBS"
