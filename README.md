# M-TARE

<p align="center">
  <strong>English</strong> | <a href="README.zh-CN.md">简体中文</a>
</p>

This repository integrates M-TARE multi-robot exploration, DCL-SLAM, Livox MID360, terrain analysis, local planning, and a customized ROS1/ROS2 bridge. The physical-robot entry point currently uses two fixed robot identities: `a/0` and `b/1`.

Each robot runs an independent ROS1 master. Cross-robot coordination messages are exchanged over ROS2/DDS.

## Repository Layout

| Path | Description |
| --- | --- |
| `scripts/` | Dependency installation, builds, physical-robot launch, and build cleanup |
| `docker/` | Physical-robot container configurations using host networking |
| `Livox-SDK2/` | Livox SDK2 sources used by the dependency installation script |
| `dcl_slam/` | Git submodule containing ROS1 algorithms and ROS2 message mirrors |
| `tare_system/` | ROS1 exploration planner and the `ros2_tare_msgs/` package |
| `autonomous_exploration_development_environment/` | Terrain analysis, local planning, and simulation sources |
| `ros1_bridge/` | Git submodule providing the customized combined DCL + TARE bridge |


Simulation scene assets under `autonomous_exploration_development_environment/src/vehicle_simulator/mesh/` are excluded from Git. They are not required by the physical-robot launch. Obtain them separately from the upstream exploration development environment if running simulations.

## 1. Get the Source

### 1.1 Exploration stack

Clone into a directory named `mtare` to match the relative Compose mount:

```bash
git clone --recurse-submodules <repository-url> mtare
cd mtare
```

For an existing checkout, run the following after updating the main repository:

```bash
git submodule sync --recursive
git submodule update --init --recursive
git submodule status --recursive
```

The submodules in `.gitmodules` use GitHub SSH URLs and require appropriate access. Deployment uses the commits recorded by the main repository; there is no need to switch submodules to their latest remote branches.

The current physical-robot configuration targets Livox MID360. The dependency installer builds the SDK from `Livox-SDK2/`. If that source directory is missing, obtain it from the repository root; skip this step if it already exists:

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
```

### 1.2 Optional chassis driver

The chassis driver is maintained separately. The example here uses a Scout Mini. Clone its workspace next to `mtare`:

```bash
cd ..
git clone https://github.com/BIT-RACE-Lab/scout_driver.git
```

## 2. Prepare the Runtime Environment

The current configuration uses **Ubuntu 20.04, ROS Noetic, ROS Foxy, and an amd64 container**. The host needs Docker and Docker Compose. The container must already provide ROS, CMake, compiler tools, colcon, and the remaining project dependencies.

The [dependency installation script](scripts/install_dependencies.sh) only installs `python3-catkin-tools` and builds and installs Livox SDK2 into `/usr/local`. It does not provision a complete ROS environment from a bare Ubuntu installation.

### Start the container

Run these commands from the repository root on the host:

```bash
docker --version
docker compose version
docker image inspect foxy-noetic:amd64

# Use this configuration for the separate Scout workspace:
docker compose -f docker/compose.scout.yml up -d
# Otherwise, use this configuration instead:
docker compose -f docker/compose.yml up -d
```

Compose uses the local image `foxy-noetic:amd64` with `pull_policy: never`. Prepare the image before starting the container; see the [image repository](https://github.com/BIT-RACE-Lab/foxy-noetic-docker-image).

The container is named `mtare-amd64`, with sources mounted at `/root/mtare`. The `../../mtare` mount is resolved relative to the Compose file's directory. Adjust it if your local checkout has a different directory name. Source edits are shared with the host; system dependencies are installed inside the container and may need reinstalling after container recreation.

RViz is disabled by default. To use it, the host needs an available X11 display and must grant the container user access. For a local X11 session, for example:

```bash
xhost +si:localuser:root
```

Compose forwards `DISPLAY` and mounts `/tmp/.X11-unix`. Revoke this grant afterward with `xhost -si:localuser:root`.

## 3. Configure the Two-Robot Network and LiDAR

The recorded deployment addresses are shown below. Check them against your hardware:

| Robot | Prefix / ID | Inter-robot IP | MID360 IP | Host LiDAR NIC IP |
| --- | --- | --- | --- | --- |
| Robot 1 | `a / 0` | `192.168.31.11` | `192.168.2.167` | `192.168.2.166` |
| Robot 2 | `b / 1` | `192.168.31.12` | `192.168.2.195` | `192.168.2.22` |

Configure the host interfaces so that the robot computers can reach each other and each computer can reach its own sensor. Use the separate LiDAR and inter-robot subnets shown above.

The MID360 launch file reads [MID360_config.json](dcl_slam/ros1_ws/src/livox_ros_driver2/config/MID360_config.json). Configure the sensor and host receiving IP addresses on each robot. Selecting `a` or `b` selects the robot identity; it **does not switch the LiDAR IP configuration automatically**.

[run_real.sh](scripts/run_real.sh) sets the following environment variables:

| Variable | Default / behavior |
| --- | --- |
| `ROS_MASTER_URI` | Fixed to `http://127.0.0.1:11311`, using the local master |
| `ROS_IP` | `192.168.31.11` for a, `192.168.31.12` for b; overridable through the environment |
| `ROS_DOMAIN_ID` | Defaults to `30`; must match on both robots |
| `ROS_LOCALHOST_ONLY` | Fixed to `0` |
| `RMW_IMPLEMENTATION` | Fixed to `rmw_fastrtps_cpp` |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | Uses `dcl_slam/config/fastdds_wifi.xml` |

The [Fast DDS profile](dcl_slam/config/fastdds_wifi.xml) selects the inter-robot network using an interface allowlist and explicit unicast discovery peers. When changing inter-robot IP addresses, update both `interfaceWhiteList` and `initialPeersList` on both robots, and supply the corresponding launch address:

```bash
ROS_IP=192.168.31.21 ./scripts/run_real.sh a
```

Changing `ROS_IP` alone does not update DDS. Containers use `network_mode: host`. Synchronize the robot clocks and allow the required DDS UDP traffic through the firewall. Successful `ping` responses do not establish that bridged messages are being delivered.

## 4. Install Dependencies and Build

### 4.1 Exploration stack

Enter the container:

```bash
docker exec -it mtare-amd64 /bin/bash
cd /root/mtare
```

Install dependencies with the script:

```bash
./scripts/install_dependencies.sh -j 4
```

Alternatively, perform the installation manually:

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

Build the stack:

```bash
./scripts/build.sh -j 4
```

Both scripts accept `-j 4`, `--jobs 4`, or `-j4`, and default to four parallel jobs. The build order is:

1. DCL ROS1 workspace and ROS2 message workspace.
2. TARE ROS1 planner.
3. Terrain analysis and local planning.
4. TARE ROS2 messages in `tare_system/ros2_tare_msgs`.
5. Combined `ros1_bridge`, followed by DCL and TARE conversion checks.

The bridge builds incrementally by default. After changing message definitions, mapping rules, or message environments, or when conversions are missing, run:

```bash
./scripts/build.sh -j 4 --rebuild-bridge
```

`--rebuild-bridge` refreshes the bridge CMake cache and moves the previous `generated` conversion sources into a temporary backup directory before regeneration. Final checks cover TARE `Cell`, `Edge`, and `ExplorationInfo`, and DCL `GlobalDescriptor`, `LoopInfo`, and `NeighborEstimate`. Restart any running bridge after rebuilding.

Keep DCL ROS1 definitions synchronized with the ROS2 mirrors in `dcl_slam/ros2_ws/src/dcl_slam_msgs`. The TARE ROS2 `build/`, `install/`, and `log/` directories are located inside `tare_system/ros2_tare_msgs/`.

### 4.2 Optional chassis driver

In a container using the Scout Compose configuration:

```bash
cd /root/scout_driver
sudo apt update
sudo apt install -y libasio-dev
./scripts/build.sh -j 8
```

## 5. Run the Two-Robot System

### 5.1 Start exploration

Stop existing DCL, Livox, and bridge processes, including those started by `dcl_slam/scripts/run_robot.sh`. Run only one instance of the physical-robot stack on each robot.

Inside robot a's container:

```bash
cd /root/mtare
./scripts/run_real.sh a
```

Inside robot b's container:

```bash
cd /root/mtare
./scripts/run_real.sh b
```

The script checks build outputs, starts `roscore` if no master is available, loads the two-robot bridge configuration, and starts the combined bridge. **Wait until both robots reach the prompt, then press Enter on each** to start Livox, DCL, terrain analysis, local planning, and exploration nodes. Keep the robots stationary while initial point clouds are collected.

The prompt is a manual synchronization point, not a DDS connectivity test. The exploration gate waits until both `alignment_ready` values remain true for approximately two seconds, then publishes `/start_exploration`. It only controls initial startup; it is not a continuous stop monitor.

The launch script does not load chassis-driver environments or check chassis interfaces. The local planner publishes `/cmd_vel` as `geometry_msgs/Twist`. The default `autonomy_mode:=false` controls autonomous mode; it is not an emergency stop or a guarantee that all velocity output is disabled.

### Common launch arguments

Extra arguments use `name:=value` syntax and are forwarded to [real_robot_stack.launch](tare_system/src/tare_planner/launch/real_robot_stack.launch):

| Argument | Default | Purpose |
| --- | --- | --- |
| `rviz` | `false` | Show exploration in RViz |
| `scenario` | `indoor` | Select exploration scenario configuration |
| `start_livox` | `true` | Start the Livox driver through this launch |
| `fast_lio_config` | `config/dcl_fast_lio_mid360.yaml` in the DCL package | FAST-LIO configuration |
| `autonomy_mode` | `false` | Autonomous local-control mode |
| `max_speed` | `0.3` | Maximum speed in m/s |
| `autonomy_speed` | `0.3` | Autonomous-mode speed in m/s |
| `vehicle_length` | `0.6` | Vehicle length in meters |
| `vehicle_width` | `0.6` | Vehicle width in meters |

Example:

```bash
./scripts/run_real.sh a rviz:=true scenario:=indoor max_speed:=0.3 \
  vehicle_length:=0.6 vehicle_width:=0.6
```

Configure vehicle dimensions, LiDAR extrinsics, and control parameters for the actual platform. Add `autonomy_mode:=true` after validating the control interface and stopping behavior.

The script fixes the layout to `a/0`, `b/1`, and two robots. It rejects overrides of `robot_prefix`, `robot_id`, `robot_num`, and `robot_prefixes`. Only arguments declared in the top-level launch can be forwarded; arguments from included launch files are not necessarily exposed.

### 5.2 Start the optional chassis driver

After connecting the adapter, configure CAN on the host:

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

Run the driver in a separate container terminal. This example targets Scout Mini; other models require the appropriate driver configuration:

```bash
cd /root/scout_driver
export ROS_MASTER_URI=http://127.0.0.1:11311
./scripts/run.sh port_name:=can0
```

The driver subscribes to `/cmd_vel` (`geometry_msgs/Twist`). `run_real.sh` does not manage this separately started driver; stop it separately when ending the run.

### 5.3 Stop

Press Ctrl-C in the exploration terminal to clean up the process groups started by the script. A ROS master that existed before launch is preserved. Stop any separately started chassis driver in its own terminal. To stop the container, run `docker stop mtare-amd64` on the host.

## 6. Communication Architecture and Checks

```text
Robot a: independent ROS1 master       Robot b: independent ROS1 master
Livox / DCL / TARE / local planner     Livox / DCL / TARE / local planner
             |                                     |
        ros1_bridge <--------- ROS2 DDS --------> ros1_bridge
```

Raw point clouds, IMU, terrain maps, local paths, and velocity commands remain in local ROS1. The combined bridge forwards only messages listed in [bridge_real_two_robots.yaml](tare_system/src/tare_planner/config/bridge_real_two_robots.yaml):

- M-TARE robot positions, exploration information, and completion states.
- DCL global descriptors, loop information, and distributed optimization states and estimates.
- `/a/dcl_slam/alignment_ready` and `/b/dcl_slam/alignment_ready`.

The customized `ros1_bridge` submodule provides custom conversions, parameterized topic lists, durable message handling, and bidirectional bridge-loop suppression. Use the repository version for building and running.

Load the local ROS1 environment in a new container terminal before checking:

```bash
cd /root/mtare
source /opt/ros/noetic/setup.bash
source dcl_slam/ros1_ws/devel/setup.bash --extend
source tare_system/devel/setup.bash --extend
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.31.11  # Use 192.168.31.12 on robot b.
unset ROS_HOSTNAME

rosnode list
rostopic echo -n 1 /a/dcl_slam/alignment_ready
rostopic echo -n 1 /b/dcl_slam/alignment_ready
rostopic echo -n 1 /start_exploration
rostopic info /cmd_vel
rostopic type /cmd_vel
```

`rostopic echo -n 1` waits if no message arrives; use Ctrl-C to stop waiting. Check on both robots that messages from the peer actually arrive. A listed topic alone does not demonstrate working inter-robot communication.

| Symptom | What to check |
| --- | --- |
| Missing image or container startup failure | Local `foxy-noetic:amd64` image and matching architecture |
| `Missing: ...; run ./scripts/build.sh first.` | Source mount and completed workspace builds |
| `Missing Livox SDK2` | Run the dependency installer in the current container |
| `Missing bridge conversion` | Synchronize messages, run `./scripts/build.sh -j 4 --rebuild-bridge`, and restart |
| Existing bridge/DCL/Livox warning | Stop the previous launch before starting again |
| No local LiDAR data | LiDAR NIC address, device/receiver addresses in MID360 JSON, and Livox logs |
| No peer messages | Robot IPs, DDS allowlist and discovery peers, domain ID, firewall, and both bridges |
| Exploration never starts | Actual values of both `alignment_ready` topics, DCL logs, and Enter synchronization |
| RViz cannot display | Host `DISPLAY`, X11 socket mount, and display access permissions |

## 7. Cleanup and Maintenance

Stop relevant nodes and builds before previewing cleanup:

```bash
./scripts/clean_mtare_build.sh --dry-run
```

After reviewing the list, clean and rebuild:

```bash
./scripts/clean_mtare_build.sh
./scripts/build.sh -j 4
```

The cleanup script removes only predefined regenerable workspace outputs and build logs. It preserves sources, `.catkin_tools` configuration, and runtime logs, and does not uninstall dependencies from `/usr/local`.

To update a submodule, complete and validate changes there before committing the reference in the main repository. For example, after publishing and validating a bridge commit:

```bash
git -C ros1_bridge fetch origin
git -C ros1_bridge checkout <validated-commit>
git add ros1_bridge
git commit -m "chore: update ros1_bridge submodule"
```

Other developers can align versions with `git submodule update --init --recursive` after pulling the main repository. Submodules are already registered in `.gitmodules`; do not initialize or add them again.

## Citation and Acknowledgements

This repository builds on [M-TARE](https://github.com/caochao39/mtare_planner) and [TARE Planner](https://github.com/caochao39/tare_planner). We thank Chao Cao and the upstream contributors for the exploration algorithms, source code, and deployment examples. This repository adds DCL-SLAM integration, physical two-robot deployment, combined message bridging, and documentation. Credit for the original algorithms belongs to their authors.

We also thank the following projects and their contributors:

- [Autonomous Exploration Development Environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment): terrain analysis, local planning, and simulation environments.
- [DCL-SLAM](dcl_slam/README.md): distributed collaborative localization and mapping, together with its upstream algorithms and dependencies.
- [Livox-SDK2](Livox-SDK2/) and Livox ROS Driver 2: MID360 integration.
- [ros1_bridge](ros1_bridge/README.md) and the ROS community: ROS1/ROS2 conversions and communication infrastructure.
- [Google OR-Tools](tare_system/src/tare_planner/or-tools/README.md): optimization tools.

Please cite the original work when using M-TARE/TARE algorithms in research:

1. C. Cao, H. Zhu, Z. Ren, H. Choset, and J. Zhang. *Representation Granularity Enables Time-Efficient Autonomous Exploration in Large, Complex Worlds*. Science Robotics, vol. 8, no. 80, 2023.
2. C. Cao, H. Zhu, H. Choset, and J. Zhang. *TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments*. Robotics: Science and Systems (RSS), 2021.

See the [TARE README](tare_system/README.md) for BibTeX and algorithm details, and the [upstream DCL-SLAM README](dcl_slam/ros1_ws/src/DCL-SLAM/README.md) for its citation. Repository-level attribution is recorded in [NOTICE](NOTICE).

The legacy Docker simulation scripts `run_mtare.sh`, `stop.sh`, and `docker-compose-network.yml` are not included here. Do not use those upstream simulation commands as this repository's physical-robot launch procedure.

## License

Repository-level integration scripts, configuration, and documentation added by this project are available under the [Apache License 2.0](LICENSE). M-TARE/TARE, DCL-SLAM, Livox, ROS, OR-Tools, and other bundled third-party code retain their original copyrights and license terms. The root LICENSE does not replace or relicense those components. See [NOTICE](NOTICE) and each component's license files, package metadata, and source notices.
