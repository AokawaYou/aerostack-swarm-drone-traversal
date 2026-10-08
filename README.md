> ⚠️ **本机（WSL / Gazebo Harmonic）的移植与修复记录见 [`challenge_multi_drone/README.md`](challenge_multi_drone/README.md)** —— 含 RMW 一致性、插件 Fortress→Harmonic 移植、gz 桥方向、柱子几何、退出复位等
> 改动说明、自查命令（附正常值）与「改文件铁律」。

# Drone Swarm Path Planning Projects with Aerostack2

<!-- DSH-FORK-NOTICE -->
## 关于本仓库：上游项目的分支（fork）

* **上游项目**：[`sanan222/aerostack-swarm-drone-traversal`](https://github.com/sanan222/aerostack-swarm-drone-traversal)（作者 Sanan Garayev，MIT 协议）
* **本仓库定位**：上游代码的**分支**。`main` 分支保持与上游一致（`357f312`），**本机全部改动都在 `dsh-scenario4-harmonic-port` 分支上**；`git diff main..dsh-scenario4-harmonic-port` 就是全部改动。
* **改动范围**：集中在"让 **Scenario 4（动态障碍避障）** 在本机跑起来"这一条线上——① Fortress→Harmonic 移植、② 规划与避障算法改进、③ 启动与运行环境修复。Scenario 1/2/3 的任务代码未改。
* **本机调试手册**（踩坑记录、自查命令附"正常值"、改文件铁律）：[`challenge_multi_drone/README.md`](challenge_multi_drone/README.md)

---

### 一、从 Gazebo Fortress 移植到 Gazebo Harmonic

上游面向 **Ignition Fortress**（gz-sim 6），本机是 **Gazebo Harmonic**（gz-sim 8）。两代插件**二进制接口不兼容**，所以上游的自制插件在 Harmonic 下"编译成功但从不加载"（gz 日志报 `The symbol [GzPluginHook] is missing` / `No plugins detected in library`）——**动态障碍从未真的动过**。

| 改了什么 | 具体做法 |
|---|---|
| 插件注册宏 | `IGNITION_ADD_PLUGIN` → `GZ_ADD_PLUGIN`。校验：`nm -D --defined-only X.so \| grep -i PluginHook`，应由 `IgnitionPluginHook` 变为 `GzPluginHook` |
| 构建系统 | `find_package(ignition-gazebo6)` → `gz-plugin2` / `gz-common5` / `gz-sim8`（目标名 `gz-sim8::gz-sim8`、`gz-plugin2::register`）。`ldd X.so \| grep ignition` 应为空 |
| 命名空间 | 源码 `ignition::msgs::` → `gz::msgs::`；`ignerr` / `ignmsg` → `gzerr` / `gzmsg` |
| 时间 API | gz-sim 8 的 `UpdateInfo::simTime` 是 `steady_clock::duration`（**不是 time_point**），插件限频判断按此改写 |
| 障碍位姿桥 | 桥声明由 `Pose_V` / `PoseArray` 改为 `ignition.msgs.Pose → geometry_msgs/msg/PoseStamped`。上游此处类型写错，消费端**一条数据都收不到** |
| 柱子几何 | 柱体半截陷地：在 `utils/sdf_templates.py` **模板**里给 collision/visual 加 `<pose>0 0 {height/2} 0 0 0</pose>`。注意 `world.yaml` 与柱子 SDF 每次启动都会被重新生成，**必须改模板**，改生成物是定时炸弹 |
| 仿真步调 | 柱子插件原本每个物理步（4 ms）都发一次 `/world/<w>/set_pose`（≈1250 次/秒）把 RTF 压到 ≈0.4；改为**限频 + 插件自积分**，轨迹不变 |

### 二、规划与避障算法改进

| 问题（上游 / 旧版表现） | 改动 | 效果 |
|---|---|---|
| 只会直飞，明明可行的路径不走 | 反应层方向锥锚点从"到终点的方位"改为**路径切向**；有路径时前进量按沿路径计算 | 该躲就躲，绕行系数下降 |
| 被柱子"推着后退" | 禁后退改成**显式硬约束**；净空双向对冲 | 后退占比 22.2% → 0.2% |
| 太慢：`go_to` action 本身就要几十秒 | 改为 **20 Hz 直发 `/motion_reference`** 替代 `go_to`，目标 = 编队槽位 + v×lead_time | 导航段 138.8 s → 75.9 s；控制效率 ≈92% |
| **【致命一行】** 预检循环漏写 `streamers[id]=st`，导致控制层"修复"从未生效、全程**静默**退化回 `go_to` | 补上登记 + 单拍发送硬时限 + 发送失败 3 次才降级 | 5/5 架真在直发，不再偷偷跑 `go_to` |
| 动态障碍只收到 3/5 根 → 编队一头撞进"看不见的柱子" | 独立节点 **best-effort 双路订阅**；起飞门槛改为"柱子凑齐才起飞" | 5/5 根稳定收齐 |
| 起飞时就被柱子撞（集结点最北与柱体活动区最南只差 1 cm） | **收缩柱子活动区**（`min_y` 1.0 → 2.5） | 起飞阶段不再被撞 |
| `safety_margin` 调大反而更差（**负结果**） | 参数回退到 plan4 那一组 | 已记录，别再盲目调大 |

### 附：启动与运行环境修复（为跑通 Scenario 4 所必需）

| 问题 | 修法 |
|---|---|
| 任务脚本所有服务调用全 TIMEOUT | `setup.bash` 强制 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（WSL 下 Fast-DDS 共享内存反复失败导致发现不稳定） |
| `setup.bash` 按 `$(pwd)/../../` 推工作区，在仓库根编译时报 No such file | 改为基于 `BASH_SOURCE` 的 `dirname/..` 推导 |
| 上一轮残留的 Fast-DDS 共享内存段让新会话发现失败 | `launch_as2.bash` 启动时 `rm -f /dev/shm/fastrtps_*` |
| Gazebo 能开但世界里空的（spawn 竞态） | 行为树 pane 换 echo + 加 sleep 等待 spawn 完成 |
| 每轮跑完留脏状态 → 下一轮 takeoff 被拒 | 退出时 `reset_all()` 复位 5 架机位姿 |

---

## 依赖

| 依赖 | 版本 / 说明 |
|---|---|
| 操作系统 | Ubuntu 22.04（本机跑在 **WSL2**，发行版名 `Ubuntu22.04D`） |
| ROS 2 | **Humble**（`/opt/ros/humble`） |
| Gazebo | **Harmonic**，`gz sim 8.15.0`。⚠️ **不要用 Fortress**；两套共存时注意 CMake 别链到 `ignition-gazebo6` |
| Python | 3.10.12（Ubuntu 22.04 自带） |
| 编译 | `colcon`（在**仓库根**编译） |
| 会话编排 | `tmux` + `tmuxinator`（`launch_as2.bash` 用它开 5 架机的会话） |
| gz ↔ ROS 桥 | `ros_gz_bridge`（`ros-humble-ros-gz`） |
| RMW | `rmw_cyclonedds_cpp`（本仓库 `setup.bash` 强制 export） |
| Aerostack2 | 仓库内**已 vendored**（`aerostack2/` 目录），随仓库一起编译，无需另装 |

编译（在仓库根执行一次即可）：

```bash
cd ~/projects/aerostack-swarm-drone-traversal
colcon build
```

## 启动流程

以下命令都在 `challenge_multi_drone/` 目录下执行。

```bash
cd ~/projects/aerostack-swarm-drone-traversal/challenge_multi_drone
source setup.bash          # 设置 WORKSPACE + rmw_cyclonedds_cpp
```

### 终端 1 — 启动仿真栈（Gazebo + 5 架机）

```bash
./launch_as2.bash -s scenarios/scenario1_stage4.yaml -w config_sim/config/world_swarm.yaml
```

> ⚠️ **不要加 `-m`**：`-m` 会走多旋翼仿真平台分支、**不启动 Gazebo**，而 Scenario 4 的动态障碍只由 Gazebo 插件产生。`-w` 也必须用 Gazebo 格式的 `config_sim/config/world_swarm.yaml`（`config_multicopter/` 那份是 `vehicle_initial_pose` 格式，会导致 0 架机 spawn）。

### 终端 2 — 跑 Scenario 4 任务

```bash
python3 mapf_scenario_4_6.py --scenario_file scenarios/scenario1_stage4.yaml
```

`mapf_scenario_4_6.py` 是**当前唯一推荐档**（5/5 根柱子可见 + 0.7 m/s + 全程直发参考）。被取代的旧版本已归档到 `out_of_date/`；对照时这样跑（**注意 `PYTHONPATH=.`**，因为 `mapf_utils.py` 留在原目录，而 Python 的 `sys.path[0]` 是脚本所在目录）：

```bash
PYTHONPATH=. python3 out_of_date/mapf_scenario_4_4.py --scenario_file scenarios/scenario1_stage4.yaml
```

### 终端 3 — 停止

```bash
./stop.bash
```

> 该脚本按 config 里的机名杀 tmux 会话，随后 `pkill` 掉 `gz` / `gazebo` / `ros_gz_bridge`——**它会连带关掉 Gazebo**。

任务跑完会自动复位 5 架机；逐拍航迹写在 `challenge_multi_drone/traces/`（文件名前缀固定为 `scenario4_4_`，与跑的是哪一档无关）。
<!-- /DSH-FORK-NOTICE -->


## Running Instructions

This project supports centralized Multi-Agent Path Finding (MAPF) for drone swarms in simulation. Each scenario is launched and controlled using separate terminal sessions.

### Scenario 1 – Circular Path with Formation Changes

**Terminal 1:**
```bash
cd challenge_multi_drone
colcon build
source install/setup.bash
cd src/challenge_multi_drone/
./launch_as2.bash -m -s scenarios/scenario1_stage1.yaml -w config_sim/config_multicopter/world_swarm.yaml
```

**Terminal 2:**
```bash
./launch_ground_station.bash -v -s scenarios/scenario1_stage1.yaml
```

**Terminal 3:**
```bash
python3 mapf_scenario_1.py --scenario_file scenarios/scenario1_stage1.yaml
```

### Scenario 2 – Window Traversal

**Terminal 1:**
```bash
./launch_as2.bash -m -s scenarios/scenario1_stage2.yaml -w config_sim/config_multicopter/world_swarm.yaml
```

**Terminal 2:**
```bash
./launch_ground_station.bash -v -s scenarios/scenario1_stage2.yaml
```

**Terminal 3:**
```bash
python3 mapf_scenario_2.py --scenario_file scenarios/scenario1_stage2.yaml
```

### Scenario 3 – Forest Navigation

**Terminal 1:**
```bash
./launch_as2.bash -m -s scenarios/scenario1_stage3.yaml -w config_sim/config_multicopter/world_swarm.yaml
```

**Terminal 2:**
```bash
./launch_ground_station.bash -v -s scenarios/scenario1_stage3.yaml
```

**Terminal 3:**
```bash
python3 mapf_scenario_3.py --scenario_file scenarios/scenario1_stage3.yaml
```

### Scenario 4 – Dynamic Obstacle Avoidance

**Terminal 1:**
```bash
./launch_as2.bash -m -s scenarios/scenario1_stage4.yaml -w config_sim/config_multicopter/world_swarm.yaml
```

**Terminal 2:**
```bash
./launch_ground_station.bash -v -s scenarios/scenario1_stage4.yaml
```

**Terminal 3:**
```bash
python3 mapf_scenario_4.py --scenario_file scenarios/scenario1_stage4.yaml
```

---

## MAPF Methodology Overview

The Multi-Agent Path Finding (MAPF) implementation provides centralized control over drone swarms. It ensures safe, collision-free trajectories while preserving formation structure using path planning techniques such as Conflict-Based Search (CBS), Dijkstra’s algorithm, and hybrid local avoidance.

### ▸ Scenario 1: Coordinated Formation Transitions on a Circular Trajectory
- Circular path (radius = 2.0m, altitude = 1.5m) is segmented, each with a unique formation (Line, V, Grid, etc.).
- CBS computes safe, layered altitude assignments for formation transitions.
- Orientation is realigned tangentially to the path for visual consistency.
- Altitude separation and CBS ensure conflict-free motion during realignment.

### ▸ Scenario 2: Window Traversal with Adaptive Compression
- Formation dynamically compresses to fit through narrow windows using YAML-defined waypoints.
- The compression radius is calculated by:
  ```math
  r_\text{compressed} = \frac{\text{window width} - 2 \times \text{safety margin}}{2}
  ```
- Compressed formation used only at critical waypoints; standard pentagon used otherwise.
- Movement synchronization is achieved using delayed non-blocking commands for smooth transitions.

### ▸ Scenario 3: Obstacle-Aware Navigation through Forest Environments
- Environment is represented as a graph; obstacles become nodes.
- Dijkstra’s algorithm finds shortest global path; local 3D A* plans for each drone.
- Formation may split/rejoin based on obstacle proximity.
- Safety ensured using 0.2m resolution and 0.3m clearance margins.

### ▸ Scenario 4: Hybrid Navigation with Dynamic Obstacle Prediction
- Combines global A* path generation and local potential field-based avoidance.
- Real-time obstacle tracking via ROS2 topic `/dynamic_obstacles/locations`.
- Predictive "no-fly zones" built from smoothed obstacle velocity estimations.
- Movement vector is the sum of attractive (goal-seeking) and repulsive (avoidance) forces.
- Formation compresses or expands depending on proximity to moving obstacles.
- Motion is velocity-normalized and capped to 0.5 m/s for smooth, stable transitions.

---

This MAPF-based centralized control system was developed by **Sanan Garayev** and validated in simulation across all four test scenarios using five drones within Aerostack2 and ROS2 environments.
