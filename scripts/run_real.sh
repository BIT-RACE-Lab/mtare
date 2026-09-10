#!/usr/bin/env bash
# Bring up the real two-robot stack on one robot (letter a/0 or b/1).
# 在单台机器人上启动真实双机系统（字母 a/0 或 b/1）。
set -eo pipefail
# Resolve the M-TARE workspace root from this script's location.
# 依据脚本自身位置推导 M-TARE 工作区根目录。
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Print command line usage. / 打印命令行用法。
usage() {
    echo 'Usage: ./scripts/run_real.sh a|b [real_robot_stack.launch name:=value ...]'
    echo 'Defaults: autonomy_mode:=false rviz:=false. ROS_IP may override the Wi-Fi IP.'
    echo 'Start on both robots; press Enter on each only after both bridges are running.'
}
# --help prints usage and exits without doing any work.
# --help 仅打印用法后直接退出，不执行任何实际操作。
[[ "${1:-}" != --help ]] || { usage; exit 0; }
# Map the robot letter to its prefix, robot id and default Wi-Fi IP.
# 将机器人字母映射为前缀、机器人编号和默认 Wi-Fi 地址。
case "${1:-}" in
    a) PREFIX=a; ROBOT_ID=0; DEFAULT_IP=192.168.31.11 ;;
    b) PREFIX=b; ROBOT_ID=1; DEFAULT_IP=192.168.31.12 ;;
    *) usage >&2; exit 2 ;;
esac
# Drop the robot letter so that the remaining arguments are launch arguments.
# 移除机器人字母参数，其余参数即为要转发的 launch 参数。
shift
# Validate the forwarded launch arguments.
# 校验要转发的 launch 参数。
for arg in "$@"; do
    # Every forwarded argument must use the name:=value form.
    # 所有转发的参数都必须采用 name:=value 形式。
    [[ "$arg" == *:=* ]] || { echo "Invalid launch argument: $arg" >&2; exit 2; }
    # Reject arguments that would conflict with the fixed two-robot layout.
    # 拒绝会与固定双机布局冲突的参数。
    case "$arg" in robot_prefix:=*|robot_id:=*|robot_num:=*|robot_prefixes:=*)
        echo 'This script uses the fixed a/0, b/1 two-robot configuration.' >&2; exit 2 ;;
    esac
done
# Verify that all required overlays exist before sourcing anything.
# source 之前先确认所有必需的 overlay 均已存在。
for file in /opt/ros/noetic/setup.bash /opt/ros/foxy/setup.bash \
    "$ROOT/dcl_slam/ros1_ws/devel/setup.bash" "$ROOT/tare_system/devel/setup.bash" \
    "$ROOT/autonomous_exploration_development_environment/devel/setup.bash" \
    "$ROOT/tare_system/ros2_tare_msgs/install/local_setup.bash" "$ROOT/dcl_slam/ros2_ws/install/local_setup.bash" \
    "$ROOT/ros1_bridge/install/local_setup.bash" "$ROOT/dcl_slam/config/fastdds_wifi.xml"; do
    [[ -f "$file" ]] || { echo "Missing: $file; run ./scripts/build.sh first." >&2; exit 1; }
done
# setsid is required to create a separate process group per child process.
# 需要 setsid 为每个子进程创建独立进程组，便于统一清理。
command -v setsid >/dev/null
# Start from a clean shell environment to avoid mixing noetic and foxy settings.
# 从干净的环境开始，避免 noetic 与 foxy 的环境变量互相污染。
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ROOT
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
source /opt/ros/noetic/setup.bash
# Layer the ROS1 overlays on top of noetic, in dependency order.
# 按依赖顺序在 noetic 之上叠加各 ROS1 overlay。
source "$ROOT/dcl_slam/ros1_ws/devel/setup.bash" --extend
source "$ROOT/tare_system/devel/setup.bash" --extend
source "$ROOT/autonomous_exploration_development_environment/devel/setup.bash" --extend
# Bind ROS1 to localhost and advertise this robot's Wi-Fi IP to the other robot.
# ROS1 主节点固定在本机，并把本机 Wi-Fi 地址通告给另一台机器人。
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP="${ROS_IP:-$DEFAULT_IP}"
unset ROS_HOSTNAME
# ROS2/DDS settings must match on both robots; profile enables Wi-Fi transport.
# ROS2/DDS 设置需在两台机器人上保持一致；该 profile 启用 Wi-Fi 通信。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}" ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/dcl_slam/config/fastdds_wifi.xml"
mkdir -p "$HOME/log"
# Separate process groups allow cleanup of ros2 run's child as well as roslaunch.
# 为每个子进程使用独立进程组，使 ros2 run 的子进程与 roslaunch 都能被统一清理。
pids=()
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    # Ask every process group to terminate gracefully first.
    # 先请求每个进程组优雅退出。
    for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
    # Wait up to about 5 seconds for them to exit.
    # 最多等待约 5 秒，让各进程完成退出。
    for ((i=0; i<50; i++)); do
        local alive=false
        for pid in "${pids[@]}"; do kill -0 -- "-$pid" 2>/dev/null && alive=true; done
        $alive || break
        sleep 0.1
    done
    # Force kill whatever is still alive, then reap the children.
    # 对仍未退出的进程强制终止，并回收子进程。
    for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Start a private roscore only when no ROS master is already reachable.
# 仅当当前没有可用的 ROS 主节点时才启动独立的 roscore。
if ! timeout 2 rosparam list >/dev/null 2>&1; then
    setsid roscore &
    pids+=("$!")
fi
# Wait up to about 6 seconds for the master to become available.
# 最多等待约 6 秒，直至主节点可用。
ready=false
for ((i=0; i<30; i++)); do
    if timeout 1 rosparam list >/dev/null 2>&1; then ready=true; break; fi
    sleep 0.2
done
$ready || { echo 'ROS master unavailable' >&2; exit 1; }
# Refuse to run when a previous stack is still alive on this robot.
# 若本机上仍有上一次的系统在运行，则拒绝继续启动。
nodes="$(rosnode list)"
if [[ "$nodes" == *'/ros_bridge'* || "$nodes" == *'/laserMapping'* || "$nodes" == *'/livox_lidar_publisher2'* ]]; then
    echo 'Stop existing bridge/DCL/Livox (including run_robot.sh) before this script.' >&2
    exit 1
fi
# Publish the real two-robot bridge configuration on the parameter server.
# 在参数服务器上加载真实双机场景的桥接配置。
roslaunch tare_planner param.launch bridge_config:=bridge_real_two_robots.yaml
# Run the bridge in a subshell so ROS2 sourcing cannot pollute the ROS1 shell.
# 在子 shell 中运行桥接，避免 ROS2 的环境变量污染 ROS1 侧。
(
    unset ROS_DISTRO
    source /opt/ros/foxy/setup.bash
    source "$ROOT/tare_system/ros2_tare_msgs/install/local_setup.bash"
    source "$ROOT/dcl_slam/ros2_ws/install/local_setup.bash"
    source "$ROOT/ros1_bridge/install/local_setup.bash"
    # Fail early when the bridge lacks the required DCL/TARE conversions.
    # 桥接缺少必要的 DCL/TARE 转换时提前失败。
    pairs="$(ros2 run ros1_bridge dynamic_bridge --print-pairs)"
    for expected in tare_msgs/msg/ExplorationInfo dcl_slam_msgs/msg/GlobalDescriptor \
        dcl_slam_msgs/msg/LoopInfo dcl_slam_msgs/msg/NeighborEstimate; do
        if [[ "$pairs" != *"$expected"* ]]; then
            echo "Missing bridge conversion: $expected" >&2
            echo "Run: cd $ROOT && ./scripts/build.sh -j 4 --rebuild-bridge" >&2
            exit 1
        fi
    done
    exec setsid ros2 run ros1_bridge parameter_bridge
) &
bridge_pid=$!
pids+=("$bridge_pid")
# Give the bridge a moment, then confirm it is still running.
# 等待片刻后确认桥接进程仍存活。
sleep 2
kill -0 "$bridge_pid" 2>/dev/null || { echo 'Bridge failed to start' >&2; exit 1; }
echo "Robot $PREFIX bridge started: ROS_IP=$ROS_IP DOMAIN=$ROS_DOMAIN_ID"
# Cross-robot barrier: both robots must reach this prompt before launching.
# 跨机同步点：必须等待两台机器人都到达此提示后再继续。
read -r -p 'After BOTH robots have reached this prompt, press Enter to launch SLAM/exploration: '
kill -0 "$bridge_pid" 2>/dev/null || { echo 'Bridge exited' >&2; exit 1; }
# Start SLAM and exploration, forwarding any extra launch arguments.
# 启动 SLAM 与探索流程，并转发其余 launch 参数。
setsid roslaunch tare_planner real_robot_stack.launch \
    robot_prefix:="$PREFIX" robot_id:="$ROBOT_ID" robot_num:=2 "$@" &
pids+=("$!")
echo 'Press Ctrl-C to stop all processes started here. An existing roscore is preserved.'
# Exit as soon as any managed process dies so the cleanup trap can run.
# 任一受管子进程退出即结束，以便触发清理逻辑。
status=0
wait -n "${pids[@]}" || status=$?
echo "A managed process exited ($status); stopping this stack."
exit "$status"
