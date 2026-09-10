#!/usr/bin/env bash
# Remove regenerable build outputs of the M-TARE workspace.
# 清理 M-TARE 工作区中可再生的编译产物。
set -euo pipefail

# Always use the parent of scripts/ as the workspace root, independent of the caller's cwd.
# 始终以 scripts 的上一级目录为工作区根目录，不依赖调用者的当前目录。
MTARE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"

# Print usage information. / 打印用法信息。
usage() {
    cat <<'EOF'
用法: ./scripts/clean_mtare_build.sh [--dry-run]
      bash mtare/scripts/clean_mtare_build.sh [--dry-run]

清除 M-TARE 已知工作区的可再生构建目录及构建日志。
--dry-run  仅列出待删除目录，不执行删除。
-h, --help 显示帮助。

保留源码、.catkin_tools 配置、运行日志和独立的 scout_driver。
不会卸载已安装到 /usr/local 等系统目录的依赖。
请先停止编译和相关节点，再执行清理。
EOF
}

dry_run=false
# First pass: validate every argument before acting on it.
# 第一遍：先校验全部参数，再执行任何操作。
for arg in "$@"; do
    case "$arg" in
        --dry-run) dry_run=true ;; # Preview only, do not delete. / 仅预览，不实际删除。
        -h|--help) ;; # Defer help output until all arguments are validated. / 校验全部参数后再显示帮助。
        *) echo "错误：未知参数 ${arg}" >&2; usage >&2; exit 2 ;;
    esac
done
# Second pass: show help only when the whole command line is valid.
# 第二遍：仅在整条命令行合法时才显示帮助。
for arg in "$@"; do
    case "$arg" in -h|--help) usage; exit 0 ;; esac
done

# Guard against running from an unexpected copy of the script.
# 防止从非预期的脚本副本位置执行。
if [[ ! -f "${MTARE_ROOT}/scripts/build.sh" || ! -d "${MTARE_ROOT}/tare_system/src" ]]; then
    echo "错误：脚本必须位于 M-TARE 的 scripts 目录：${MTARE_ROOT}" >&2
    exit 1
fi

# Keep an explicit artifact list instead of searching for same-named directories;
# .catkin_tools holds user configuration and must be kept.
# 仅维护明确的产物列表，不递归搜索同名目录；.catkin_tools 含用户配置。
readonly -a BUILD_ARTIFACTS=(
    "Livox-SDK2/build"
    "dcl_slam/ros1_ws/build"
    "dcl_slam/ros1_ws/devel"
    "dcl_slam/ros1_ws/install"
    "dcl_slam/ros1_ws/logs"
    "dcl_slam/ros2_ws/build"
    "dcl_slam/ros2_ws/install"
    "dcl_slam/ros2_ws/log"
    # Build outputs from before the DCL migration.
    # DCL 迁移前的构建产物。
    "dcl_slam/build"
    "dcl_slam/devel"
    "dcl_slam/install"
    "dcl_slam/logs"
    "tare_system/build"
    "tare_system/devel"
    "tare_system/install"
    "tare_system/ros2_tare_msgs/build"
    "tare_system/ros2_tare_msgs/install"
    "tare_system/ros2_tare_msgs/log"
    "autonomous_exploration_development_environment/build"
    "autonomous_exploration_development_environment/devel"
    "autonomous_exploration_development_environment/install"
    "ros1_bridge/build"
    "ros1_bridge/install"
    "ros1_bridge/log"
)

# Validate every target first, then delete; this avoids a half-finished cleanup.
# 先校验全部目标，再执行删除；遇到异常时避免只清理了一部分。
targets=()
for relative_path in "${BUILD_ARTIFACTS[@]}"; do
    target="${MTARE_ROOT}/${relative_path}"
    [[ -e "$target" || -L "$target" ]] || continue
    resolved="$(realpath -m -- "$target")"
    # Never follow symlinks and only delete plain directories.
    # 不跟随软链接，且只删除普通目录。
    if [[ "$resolved" != "$target" || -L "$target" || ! -d "$target" ]]; then
        echo "错误：清理目标不是普通目录或路径经过软链接：${target}" >&2
        exit 1
    fi
    targets+=("$target")
done

# Nothing to do when no build output exists. / 没有构建产物时无需处理。
if [[ ${#targets[@]} -eq 0 ]]; then
    echo "未找到需要清理的编译产物。"
    exit 0
fi

# Delete one target at a time and abort on the first failure.
# 逐个删除目标，遇到第一个失败即中止。
for target in "${targets[@]}"; do
    if "$dry_run"; then
        echo "将删除：${target}"
    else
        echo "删除：${target}"
        if ! rm -rf -- "$target"; then
            echo "错误：删除失败，请检查目录权限及占用情况：${target}" >&2
            exit 1
        fi
    fi
done

# Summarize the result and remind the user to rebuild.
# 输出结果摘要，并提醒用户需要重新编译。
if "$dry_run"; then
    echo "共 ${#targets[@]} 个目录；以上为预览，移除 --dry-run 后才会实际删除。"
else
    echo "M-TARE 编译产物清理完成，共 ${#targets[@]} 个目录。重新运行前需要编译。"
fi
