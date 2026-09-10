#!/usr/bin/env bash
# Run inside the ROS container. Dependencies (including Livox SDK2) must exist.
# 需在 ROS 容器内运行；依赖（包括 Livox SDK2）必须已安装。
set -eo pipefail
# Resolve the M-TARE workspace root from this script's location.
# 依据脚本自身位置推导 M-TARE 工作区根目录。
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Defaults: 4 parallel jobs, incremental bridge build.
# 默认值：4 个并行任务、ros1_bridge 增量编译。
JOBS=4
REBUILD_BRIDGE=false
# Print command line usage. / 打印命令行用法。
usage() {
    echo 'Usage: ./scripts/build.sh [-j N | --jobs N] [--rebuild-bridge]'
    echo 'Default: 4 parallel jobs and incremental bridge build.'
    echo '--rebuild-bridge: refresh bridge CMake cache and regenerate conversion sources.'
}
# Parse command line options. / 解析命令行选项。
while [[ $# -gt 0 ]]; do
    case "$1" in
        -j|--jobs)
            # Separate forms -j N and --jobs N need a value.
            # 分开书写的形式 -j N 和 --jobs N 需要提供参数值。
            [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
            JOBS="$2"
            shift 2
            ;;
        -j[0-9]*) JOBS="${1#-j}"; shift ;; # Joined form such as -j8. / 连写形式，例如 -j8。
        --rebuild-bridge) REBUILD_BRIDGE=true; shift ;; # Force conversion regeneration. / 强制重新生成桥接转换源码。
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done
# Validate the number of parallel jobs. / 校验并行任务数是否合法。
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || { echo 'Jobs must be positive' >&2; exit 2; }
# Fail fast when a ROS distribution or a required input is missing.
# 缺少 ROS 发行版或必要输入文件时立即报错退出。
for file in /opt/ros/noetic/setup.bash /opt/ros/foxy/setup.bash \
    "$ROOT/dcl_slam/scripts/build.sh" "$ROOT/tare_system/ros2_tare_msgs/package.xml" \
    "$ROOT/ros1_bridge/package.xml"; do
    [[ -f "$file" ]] || { echo "Missing: $file" >&2; exit 1; }
done
# Livox SDK2 is installed system-wide by install_dependencies.sh.
# Livox SDK2 由 install_dependencies.sh 安装到系统目录。
[[ -f /usr/local/lib/liblivox_lidar_sdk_static.a ]] || {
    echo "Missing Livox SDK2; run ./scripts/install_dependencies.sh first." >&2; exit 1;
}
# Clear ROS environment variables before switching between noetic and foxy.
# 在 noetic 与 foxy 之间切换前清理 ROS 相关的环境变量。
clean_ros() {
    unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ROOT
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
    unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
}
# Step 1: build the DCL ROS1 workspace and ROS2 workspace.
# 步骤 1：编译 DCL 的 ROS1 与 ROS2 工作空间。
echo '[1/5] DCL ROS1 and ROS2 messages'
ROS1_DISTRO=noetic ROS2_DISTRO=foxy bash "$ROOT/dcl_slam/scripts/build.sh" -j "$JOBS"
clean_ros
source /opt/ros/noetic/setup.bash
source "$ROOT/dcl_slam/ros1_ws/devel/setup.bash" --extend
# Step 2: build the TARE planner in the ROS1 overlay.
# 步骤 2：在 ROS1 overlay 中编译 TARE 规划器。
echo '[2/5] TARE'
cd "$ROOT/tare_system"
catkin_make -j "$JOBS" -DCMAKE_BUILD_TYPE=Release
source "$ROOT/tare_system/devel/setup.bash" --extend
# Step 3: build terrain analysis and the local planner.
# 步骤 3：编译地形分析与局部规划器。
echo '[3/5] Terrain analysis and local planner'
cd "$ROOT/autonomous_exploration_development_environment"
catkin_make -j "$JOBS" -DCMAKE_BUILD_TYPE=Release
clean_ros
source /opt/ros/foxy/setup.bash
# Step 4: build the TARE ROS2 message package.
# 步骤 4：编译 TARE 的 ROS2 消息包。
echo '[4/5] TARE ROS2 messages'
cd "$ROOT/tare_system/ros2_tare_msgs"
MAKEFLAGS="-j$JOBS" colcon build --base-paths . --symlink-install --packages-select tare_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
# Verify all message overlays before configuring the bridge.
# 配置桥接之前先校验所有消息 overlay 是否齐全。
for setup in "$ROOT/dcl_slam/ros1_ws/devel/setup.bash" \
    "$ROOT/tare_system/devel/setup.bash" \
    "$ROOT/tare_system/ros2_tare_msgs/install/local_setup.bash" \
    "$ROOT/dcl_slam/ros2_ws/install/local_setup.bash"; do
    [[ -f "$setup" ]] || { echo "Missing: $setup; run the full build first." >&2; exit 1; }
done
clean_ros
source /opt/ros/noetic/setup.bash
source "$ROOT/dcl_slam/ros1_ws/devel/setup.bash" --extend
source "$ROOT/tare_system/devel/setup.bash" --extend
# Check ROS1 before adding the same-named ROS2 tare_msgs Python package.
# 在引入同名的 ROS2 tare_msgs Python 包之前先校验 ROS1 消息。
for message in tare_msgs/Cell tare_msgs/Edge tare_msgs/ExplorationInfo \
    dcl_slam/global_descriptor dcl_slam/loop_info dcl_slam/neighbor_estimate; do
    rosmsg show "$message" >/dev/null || { echo "ROS1 message unavailable: $message" >&2; exit 1; }
done
unset ROS_DISTRO
source /opt/ros/foxy/setup.bash
source "$ROOT/tare_system/ros2_tare_msgs/install/local_setup.bash"
source "$ROOT/dcl_slam/ros2_ws/install/local_setup.bash"
# Then check the ROS2 equivalents. / 再校验对应的 ROS2 消息。
for message in tare_msgs/msg/Cell tare_msgs/msg/Edge tare_msgs/msg/ExplorationInfo \
    dcl_slam_msgs/msg/GlobalDescriptor dcl_slam_msgs/msg/LoopInfo dcl_slam_msgs/msg/NeighborEstimate; do
    ros2 interface show "$message" >/dev/null || { echo "ROS2 message unavailable: $message" >&2; exit 1; }
done
# Step 5: build the combined DCL + TARE ros1_bridge.
# 步骤 5：编译 DCL + TARE 合并的 ros1_bridge。
echo '[5/5] Combined DCL + TARE bridge'
cd "$ROOT/ros1_bridge"
# Factory generation does not track changes to the sourced message environment.
# Only explicitly requested rebuilds invalidate the conversion sources/cache.
# 工厂生成过程不会跟踪所 source 的消息环境变化，
# 只有显式请求重建时才会使转换源码与 CMake 缓存失效。
bridge_args=()
if "$REBUILD_BRIDGE"; then
    echo 'Forcing bridge conversion regeneration and CMake configuration.'
    # Keep the previous generated sources so they can be inspected or restored.
    # 保留上一次生成的转换源码，便于排查问题或回滚。
    if [[ -d build/ros1_bridge/generated ]]; then
        backup="$(mktemp -d "${TMPDIR:-/tmp}/mtare-bridge-generated.XXXXXX")"
        mv -- build/ros1_bridge/generated "$backup/generated"
        echo "Previous generated bridge sources preserved at: $backup/generated"
    fi
    bridge_args+=(--cmake-clean-cache --cmake-force-configure)
else
    echo 'Building bridge incrementally.'
fi
MAKEFLAGS="-j$JOBS" colcon build --symlink-install --packages-select ros1_bridge \
    "${bridge_args[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$ROOT/ros1_bridge/install/local_setup.bash"
# Final gate: every required conversion pair must exist in the bridge.
# 最终校验：桥接中必须包含所有必需的转换类型对。
PAIRS="$(ros2 run ros1_bridge dynamic_bridge --print-pairs)"
for expected in tare_msgs/msg/Cell tare_msgs/msg/Edge tare_msgs/msg/ExplorationInfo dcl_slam_msgs/msg/GlobalDescriptor \
    dcl_slam_msgs/msg/LoopInfo dcl_slam_msgs/msg/NeighborEstimate; do
    [[ "$PAIRS" == *"$expected"* ]] || {
        # Report the exact missing pair and how to regenerate it.
        # 指出缺失的具体类型对以及重新生成的方法。
        echo "Missing bridge conversion: $expected" >&2
        echo "If messages or overlays changed, rerun ./scripts/build.sh -j $JOBS --rebuild-bridge" >&2
        exit 1
    }
done
echo 'Build complete; DCL and TARE bridge conversions verified.'
