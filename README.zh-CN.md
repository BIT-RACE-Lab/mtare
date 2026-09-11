# M-TARE

<p align="center">
  <a href="README.md">English</a> | <strong>简体中文</strong>
</p>

本仓库集成 M-TARE 多机器人探索、DCL-SLAM、Livox MID360、地形分析与局部规划，以及定制版 ROS1/ROS2 桥接。当前实车入口固定为两台机器人：`a/0` 和 `b/1`。

每台车运行独立的 ROS1 Master，跨车协作消息通过 ROS2/DDS 交换。

## 目录结构

| 路径 | 用途 |
| --- | --- |
| `scripts/` | 依赖安装、编译、实车启动和构建产物清理 |
| `docker/` | 实车容器配置，使用宿主机网络 |
| `Livox-SDK2/` | 随仓库附带的 Livox SDK2 源码 |
| `dcl_slam/` | Git 子模块，包含 ROS1 算法与 ROS2 消息工作区 |
| `tare_system/` | ROS1 探索规划器及 `ros2_tare_msgs/` 消息包 |
| `autonomous_exploration_development_environment/` | 地形分析、局部规划与仿真相关源码 |
| `ros1_bridge/` | Git 子模块，定制的 DCL + TARE 联合桥接 |


`autonomous_exploration_development_environment/src/vehicle_simulator/mesh/` 下的仿真场景资源不纳入 Git，实车启动不依赖这些资源。需要运行仿真时，请从上游探索开发环境单独获取。

## 1. 获取源码

### 1.1 探索部分

首次克隆时指定本地目录名为 `mtare`，与 Compose 的相对挂载路径保持一致：

```bash
git clone --recurse-submodules <仓库地址> mtare
cd mtare
```

已有仓库在更新主仓库后执行：

```bash
git submodule sync --recursive
git submodule update --init --recursive
git submodule status --recursive
```

`.gitmodules` 中的子模块使用 GitHub SSH 地址，需先配置相应访问权限。子模块按主仓库记录的提交检出；常规部署无需切换到远端最新分支。

当前实车配置面向 Livox MID360。依赖安装脚本从 `Livox-SDK2/` 构建 SDK；如果源码目录尚不存在，请在仓库根目录获取（已有目录时无需重复克隆）：

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
```

### 1.2 底盘部分

此代码只包含上层代码，不包含底盘驱动，需自行下载。本说明仅给出对松灵机器人Scout MINI底盘的适配过程

将底盘驱动下载到`mtare`文件夹的同级目录

```bash
cd ..

git clone https://github.com/BIT-RACE-Lab/scout_driver.git
```

## 2. 准备运行环境

当前配置使用 **Ubuntu 20.04、ROS Noetic、ROS Foxy 和 amd64 容器**。宿主机需提供 Docker 和 Docker Compose，容器内需已有 ROS、CMake、编译工具、colcon 及项目其余依赖。

[依赖安装脚本](scripts/install_dependencies.sh)仅安装 `python3-catkin-tools`，并编译、安装仓库内的 Livox SDK2 到 `/usr/local`；它不是从空白 Ubuntu 安装完整 ROS 环境的脚本。

### 启动容器

**进入宿主机的仓库根目录**并执行以下命令：

```bash
docker --version
docker compose version
docker image inspect foxy-noetic:amd64

# 如果实车是松灵机器人Scout系列，请使用
docker compose -f docker/compose.scout.yml up -d
# 否则使用
docker compose -f docker/compose.yml up -d
```

Compose 使用本地镜像 `foxy-noetic:amd64`，并设置 `pull_policy: never`。启动前需自行准备该镜像，容器镜像的下载请参考[foxy-noetic-docker-image](https://github.com/BIT-RACE-Lab/foxy-noetic-docker-image)

容器名称为 `mtare-amd64`，源码挂载到 `/root/mtare`。配置中的 `../../mtare` 相对于 `docker/compose.yml` 所在目录解析；若本地仓库目录不叫 `mtare`，需调整该挂载路径。挂载源码的修改会同步到宿主机，系统依赖则安装在容器中，重建容器后可能需要重新安装。

默认不启动 RViz。如需图形界面，宿主机需有可用的 X11 显示环境，并向容器用户开放访问。例如，在使用本地 X11 的宿主机上执行：

```bash
xhost +si:localuser:root
```

Compose 已传入 `DISPLAY` 并挂载 `/tmp/.X11-unix`。结束使用后可通过 `xhost -si:localuser:root` 撤销上述授权。

## 3. 配置两车网络与雷达

当前实车地址记录如下；请按实际设备核对：

| 机器人 | 前缀 / ID | 车间通信 IP | MID360 IP | 本机雷达网口 IP |
| --- | --- | --- | --- | --- |
| 一号车 | `a / 0` | `192.168.31.11` | `192.168.2.167` | `192.168.2.166` |
| 二号车 | `b / 1` | `192.168.31.12` | `192.168.2.195` | `192.168.2.22` |

在宿主机配置对应网卡地址，确保两车通信网络可互通，各车可访问自己的雷达。雷达网口与车间通信网口分别使用上表中的网段。

MID360 启动文件当前固定读取 [MID360_config.json](dcl_slam/ros1_ws/src/livox_ros_driver2/config/MID360_config.json)。每台车都应按实际雷达 IP 和本机接收 IP 修改其配置；传入 `a` 或 `b` 只选择机器人身份，**不会自动切换雷达 IP 配置**。

> 若 MID360/有线网卡 IP 与实机不符，请修改 `dcl_slam/ros1_ws/src/livox_ros_driver2/config/MID360_config.json` 中的 `ip`

[run_real.sh](scripts/run_real.sh)中的环境变量设置如下：

| 环境变量 | 默认值 / 行为 |
| --- | --- |
| `ROS_MASTER_URI` | 固定为 `http://127.0.0.1:11311`，连接本车 Master |
| `ROS_IP` | a 为 `192.168.31.11`，b 为 `192.168.31.12`，可通过环境变量覆盖 |
| `ROS_DOMAIN_ID` | 默认 `30`，两车必须一致 |
| `ROS_LOCALHOST_ONLY` | 固定为 `0` |
| `RMW_IMPLEMENTATION` | 固定为 `rmw_fastrtps_cpp` |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | 固定使用 `dcl_slam/config/fastdds_wifi.xml` |

[Fast DDS 配置](dcl_slam/config/fastdds_wifi.xml)通过网卡白名单和显式单播发现地址选择车间通信网络。**若改变车间 IP**，需要在两车上同步修改 `interfaceWhiteList` 和 `initialPeersList`，并在启动时指定对应地址，例如：

```bash
ROS_IP=192.168.31.21 ./scripts/run_real.sh a
```

仅修改 `ROS_IP` 不会更新 DDS 配置。容器使用 `network_mode: host`，两车应校时，并确保防火墙允许所需 DDS UDP 通信；`ping` 成功并不代表桥接消息已正常传输。

## 4. 安装依赖与编译

### 4.1 探索部分

首先进入容器中

```bash
docker exec -it mtare-amd64 /bin/bash

cd /root/mtare
```

通过脚本一键安装依赖

```bash
./scripts/install_dependencies.sh -j 4
```

也可以手动执行

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends python3-catkin-tools

cd ./Livox-SDK2/
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 4
sudo cmake --install build --prefix /usr/local
sudo ldconfig
cd /root/mtare
```

进行代码编译

```bash
./scripts/build.sh -j 4
```

两个脚本均支持 `-j 4`、`--jobs 4` 或 `-j4`，默认并行度为 4。编译按以下顺序进行：

1. DCL 的 ROS1 工作区与 ROS2 消息工作区。
2. TARE ROS1 规划器。
3. 地形分析与局部规划。
4. `tare_system/ros2_tare_msgs` 中的 TARE ROS2 消息。
5. 联合 `ros1_bridge`，最后检查 DCL 和 TARE 消息转换。

默认增量编译 bridge。修改消息定义、映射规则或消息环境，或者出现缺少转换的错误后，执行：

```bash
./scripts/build.sh -j 4 --rebuild-bridge
```

`--rebuild-bridge` 会刷新 bridge 的 CMake 缓存，并将旧 `generated` 转换源码移至临时备份目录后重新生成。最终检查包含 TARE 的 `Cell`、`Edge`、`ExplorationInfo`，以及 DCL 的 `GlobalDescriptor`、`LoopInfo`、`NeighborEstimate`。重新编译后，需重启运行中的 bridge。

DCL ROS1 消息与 `dcl_slam/ros2_ws/src/dcl_slam_msgs` 中的 ROS2 镜像应同步维护。TARE ROS2 消息的 `build/`、`install/` 和 `log/` 均位于 `tare_system/ros2_tare_msgs/` 内。

### 4.2 底盘部分

```bash
cd /root/scout_driver

sudo apt update
sudo apt install -y libasio-dev

./scripts/build.sh -j 8
```

## 5. 启动双车系统

### 5.1 启动探索代码

先停止已有的 DCL、Livox 和 bridge，包括此前通过 `dcl_slam/scripts/run_robot.sh` 启动的进程。每台车只运行一套实车启动脚本。

一号车容器内：

```bash
cd /root/mtare
./scripts/run_real.sh a
```

二号车容器内：

```bash
cd /root/mtare
./scripts/run_real.sh b
```

脚本先检查编译产物，在没有可用 ROS Master 时启动 `roscore`，加载双车桥接配置并启动联合 bridge。**两车都到达等待提示后，再分别按 Enter**，继续启动 Livox、DCL、地形分析、局部规划和探索节点。采集启动点云期间保持车辆静止。

Enter 提示用于人工同步，不检测 DDS 连通性。探索门控节点等待两车的 `alignment_ready` 均为 true 并持续约 2 秒，然后发布 `/start_exploration`。该门控只负责首次启动，不是运行期间的持续停车监控。

启动脚本不负责底盘驱动的环境加载和接口检查。局部规划器的 `/cmd_vel` 输出类型为 `geometry_msgs/Twist`。默认 `autonomy_mode:=false`，该参数控制自主模式，不应视为急停或所有速度输出的禁用开关。

**常用启动参数**

额外参数以 `name:=value` 形式传给 [real_robot_stack.launch](tare_system/src/tare_planner/launch/real_robot_stack.launch)：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `rviz` | `false` | 显示探索 RViz 界面 |
| `scenario` | `indoor` | 选择探索场景配置 |
| `start_livox` | `true` | 是否由 launch 启动 Livox 驱动 |
| `fast_lio_config` | DCL 包中的 `config/dcl_fast_lio_mid360.yaml` | FAST-LIO 配置文件 |
| `autonomy_mode` | `false` | 局部控制自主模式 |
| `max_speed` | `0.3` | 最大速度，单位 m/s |
| `autonomy_speed` | `0.3` | 自主模式速度，单位 m/s |
| `vehicle_length` | `0.6` | 车辆长度，单位 m |
| `vehicle_width` | `0.6` | 车辆宽度，单位 m |

例如：

```bash
./scripts/run_real.sh a rviz:=true scenario:=indoor max_speed:=0.3 \
  vehicle_length:=0.6 vehicle_width:=0.6
```

车辆尺寸、雷达外参和控制参数需按实际平台配置。完成控制接口与停车行为验证后，可添加 `autonomy_mode:=true` 启用自主模式。

脚本固定使用 `a/0`、`b/1`、机器人总数 `2`，拒绝覆盖 `robot_prefix`、`robot_id`、`robot_num` 和 `robot_prefixes`。可用参数以顶层 launch 声明为准，子 launch 的参数不一定能直接通过本脚本传入。

### 5.2 启动底盘驱动

宿主机连接适配器后配置 CAN

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

在容器中运行驱动（此启动脚本适用于`Scout mini`型号，其他型号请自行更改）

```bash
cd /root/scout_driver
export ROS_MASTER_URI=http://127.0.0.1:11311
./scripts/run.sh port_name:=can0
```

此驱动订阅 `/cmd_vel`（`geometry_msgs/Twist`）。请在独立终端运行；`run_real.sh` 不管理该驱动的生命周期，退出探索脚本时需另行停止驱动。

### 5.3 停止

在探索启动终端按 Ctrl-C，脚本会清理自己启动的进程组，保留启动前已存在的 ROS Master。独立启动的底盘驱动需在其终端中另行停止。需要停止容器时，在宿主机执行 `docker stop mtare-amd64`。

## 6. 通信架构与检查

```text
车 a：独立 ROS1 Master                 车 b：独立 ROS1 Master
Livox / DCL / TARE / 局部规划           Livox / DCL / TARE / 局部规划
             |                                     |
        ros1_bridge <--------- ROS2 DDS --------> ros1_bridge
```

原始点云、IMU、地形图、局部路径和速度指令留在本车 ROS1。联合桥接仅传输 [bridge_real_two_robots.yaml](tare_system/src/tare_planner/config/bridge_real_two_robots.yaml) 列出的消息，包括：

- M-TARE 的机器人位置、探索信息和结束状态。
- DCL 的全局描述子、回环信息及分布式优化状态和估计值。
- 两车的 `/a/dcl_slam/alignment_ready` 和 `/b/dcl_slam/alignment_ready`。

`ros1_bridge` 是项目定制子模块，包含自定义消息转换、参数化话题列表、持久化消息支持与双向桥接回环抑制。构建和运行应使用仓库内版本。

在新的容器终端中，先加载本车 ROS1 环境再检查：

```bash
cd /root/mtare
source /opt/ros/noetic/setup.bash
source dcl_slam/ros1_ws/devel/setup.bash --extend
source tare_system/devel/setup.bash --extend
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.31.11  # 二号车改为 192.168.31.12
unset ROS_HOSTNAME

rosnode list
rostopic echo -n 1 /a/dcl_slam/alignment_ready
rostopic echo -n 1 /b/dcl_slam/alignment_ready
rostopic echo -n 1 /start_exploration
rostopic info /cmd_vel
rostopic type /cmd_vel
```

`rostopic echo -n 1` 在没有消息时会等待，可按 Ctrl-C 结束。应在两台车分别确认能收到对方的消息，不能仅根据话题名称存在判断跨车通信正常。

| 现象 | 检查方向 |
| --- | --- |
| 镜像不存在、容器无法启动 | 确认本地存在 `foxy-noetic:amd64`，且架构与配置匹配 |
| `Missing: ...; run ./scripts/build.sh first.` | 检查源码挂载及各工作区编译是否完成 |
| `Missing Livox SDK2` | 在当前容器执行依赖安装脚本 |
| `Missing bridge conversion` | 同步消息定义，执行 `./scripts/build.sh -j 4 --rebuild-bridge` 并重启 |
| 提示停止已有 bridge/DCL/Livox | 停止旧启动流程，再运行本脚本 |
| 本车无雷达数据 | 检查雷达网卡地址、MID360 JSON 中的设备及接收地址，以及 Livox 节点日志 |
| 收不到另一台车的消息 | 检查两车 IP、DDS 白名单和单播发现地址、Domain ID、防火墙及两侧 bridge |
| 探索迟迟不开始 | 检查两车 `alignment_ready` 的实际消息值及 DCL 日志，确认已完成 Enter 同步 |
| RViz 无法显示 | 检查宿主机 `DISPLAY`、X11 socket 挂载和显示访问权限 |

## 7. 清理与维护

先停止相关节点和编译任务，预览清理范围：

```bash
./scripts/clean_mtare_build.sh --dry-run
```

确认列表后执行清理，再重新编译：

```bash
./scripts/clean_mtare_build.sh
./scripts/build.sh -j 4
```

清理脚本只删除预定义工作区的可再生构建目录与构建日志，保留源码、`.catkin_tools` 配置和运行日志，不卸载 `/usr/local` 中已经安装的依赖。

需要更新子模块版本时，先在子模块内完成修改与验证，再在主仓库提交对应的子模块引用。例如，已在独立 bridge 仓库发布并验证了某个提交后：

```bash
git -C ros1_bridge fetch origin
git -C ros1_bridge checkout <已验证的提交>
git add ros1_bridge
git commit -m "chore: update ros1_bridge submodule"
```

其他开发者拉取主仓库后，执行 `git submodule update --init --recursive` 即可对齐版本。子模块已登记在 `.gitmodules` 中，无需重新 `git init` 或 `git submodule add`。

## 引用与致谢

本仓库基于 [M-TARE](https://github.com/caochao39/mtare_planner) 与 [TARE Planner](https://github.com/caochao39/tare_planner) 构建。感谢 Chao Cao 及原项目贡献者提供多机器人探索规划算法、源码和部署示例。本仓库主要补充 DCL-SLAM 集成、实车双车部署、联合消息桥接与相关文档；原始算法成果归原作者所有。

同时感谢以下项目及其贡献者：

- [Autonomous Exploration Development Environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)：地形分析、局部规划和仿真环境。
- [DCL-SLAM](dcl_slam/README.zh-CN.md)：分布式协同定位与建图，以及相关上游算法和依赖。
- [Livox-SDK2](Livox-SDK2/) 与 Livox ROS Driver 2：MID360 设备接入。
- [ros1_bridge](ros1_bridge/README.md) 及 ROS 社区：ROS1/ROS2 消息转换与通信基础设施。
- [Google OR-Tools](tare_system/src/tare_planner/or-tools/README.md)：优化求解工具。

使用 M-TARE/TARE 算法开展研究时，请引用原作者的工作：

1. C. Cao, H. Zhu, Z. Ren, H. Choset, and J. Zhang. *Representation Granularity Enables Time-Efficient Autonomous Exploration in Large, Complex Worlds*. Science Robotics, vol. 8, no. 80, 2023.
2. C. Cao, H. Zhu, H. Choset, and J. Zhang. *TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments*. Robotics: Science and Systems (RSS), 2021.

TARE 的 BibTeX 与算法说明见 [TARE README](tare_system/README.md)，DCL-SLAM 的引用见 [DCL-SLAM 上游 README](dcl_slam/ros1_ws/src/DCL-SLAM/README.md)。仓库级归属说明见 [NOTICE](NOTICE)。

旧版 Docker 仿真的 `run_mtare.sh`、`stop.sh` 和 `docker-compose-network.yml` 不在当前仓库中，请勿将上游仿真命令直接作为本仓库的实车启动命令。

## 许可证

本仓库新增的集成脚本、配置与文档采用 [Apache License 2.0](LICENSE)。M-TARE/TARE、DCL-SLAM、Livox、ROS、OR-Tools 及其他随仓库分发的第三方代码保留各自的版权与许可条款；根目录 LICENSE 不替代或重新许可这些组件。请查阅 [NOTICE](NOTICE) 及各组件源码中的许可证、包元数据和版权声明。
