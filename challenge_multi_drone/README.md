# challenge_multi_drone — 本机移植与修复手册（Harmonic / AS2 Scenario 4）

> 面向"下次出问题怎么查"：每条都是「症状 → 根因 → 修法 → 验证命令」。
> 生成于 2026-10-07，环境与改动见下。**改动全部有备份**：`.bak-sanan222`（最早一批）、`.bak-dsh4 … .bak-dsh23`；
> 2026-10-08 起，所有备份文件与被取代的旧版本脚本都收进了 **`challenge_multi_drone/out_of_date/`**（保持原相对目录结构），
> 回滚单文件用 `cp out_of_date/<file>.bak-dshN <file>`。本仓库是 git 仓库，`git diff` 也能看全貌。

---

## 0. 环境快照（先确认这些，能排掉一半问题）

| 项 | 值 |
|---|---|
| 系统 | Ubuntu 22.04（WSL2 发行版 `Ubuntu22.04D`） |
| ROS | ROS 2 **Humble** |
| 仿真器 | **Gazebo Harmonic (`gz sim` 8.15)**；⚠️ 本机**同时装着 Ignition Fortress**（`ignition-fortress` → `libignition-gazebo6`） |
| RMW | **`rmw_cyclonedds_cpp`（必须，见 §1.1）** |
| Aerostack2 | 源码装于 `~/projects/aerostack2`（install 30 包） |
| 本仓库 | colcon 构建在**仓库根**：`~/projects/aerostack-swarm-drone-traversal`（`<repo>/install` 35 包） |
| 工作目录 | `~/projects/aerostack-swarm-drone-traversal/challenge_multi_drone` |
| 诊断脚本 | Windows 侧 `E:\AokawaYou\带学资料\papers\uav_swarm\DSH\wsl-files\`（WSL 里走 `/mnt/e/...`） |

---

## 1. 修复清单

### 1.1 任务脚本所有服务调用全 TIMEOUT（旧版表现为"永久挂死"）

* **症状**：`[fix] drone0 set_arming_state TIMEOUT after 8.0s (attempt 1/4)` 连续重试；加超时之前是卡在
  `[fix] arming drone0 ...` 永不返回。但用命令行 `ros2 service call /drone0/set_arming_state …` 却秒回。
* **根因**：脚本跑在 `rmw_fastrtps_cpp`（启动它的 shell 没 source 本目录的 `setup.bash`），而 AS2 栈的
  117 个进程全在 `rmw_cyclonedds_cpp`。两套 RMW 在 RTPS 层只是**部分互通**：`wait_for_service` 返回 True、
  请求发得出去，**响应送不到**；该进程的节点在别人的 ROS 图里也不会出现。AS2 的 `Client.call()` 没有超时
  （`rclpy/client.py:97` 的 `event.wait()`）把它放大成永久挂死。
* **判据**：
  ```bash
  grep -l librmw /proc/<pid>/maps            # fastrtps 多行、cyclonedds 0 行 = 不一致
  grep -a -c RMW_IMPLEMENTATION /proc/<pid>/environ   # 0 = 没设，会退回 fastrtps
  ```
* **修法**：`mapf_scenario_4.py` 顶部加 RMW 自对齐守卫（在 `import rclpy` **之前**判断并 `os.execv` 重启自己），
  启动后打印 `[fix5] active rmw implementation = …`（`apply_fixes5.py`）。
* **验证**：日志出现 `[fix5] RMW_IMPLEMENTATION=unset -> re-exec with rmw_cyclonedds_cpp`，随后 5 架 arm/offboard 一次成功。

### 1.2 动态障碍位姿一条也收不到

* **症状**：`/dynamic_obstacles/locations` 明明有 5 个发布者，`ros2 topic hz` 却没数据；脚本打印
  `Warning: No dynamic obstacle data received`。
* **根因**：gz 插件（`dynamic_object.cpp:98`）发布的是 `ignition.msgs.Pose`，而桥声明成
  `ignition.msgs.Pose_V → geometry_msgs/msg/PoseArray`；而所有消费端（`mapf_scenario_4.py`、`viz.py`、
  `scenario_runner/runner.py`）**一致按 `PoseStamped` 读**。
* **修法**：桥改 `ignition.msgs.Pose → geometry_msgs/msg/PoseStamped`（`apply_fixes6.py`）。
* **验证**：`ros2 topic info /dynamic_obstacles/locations` → `Type: geometry_msgs/msg/PoseStamped` 且有 Hz。

### 1.3 障碍话题频率爆炸（gz 侧 ~5k msg/s、ROS 侧 ~930 Hz）

* **症状**：`timeout 6 gz topic -e -t /dynamic_obstacles/locations | grep -c frame_id` → **25000+**。
* **根因（两处叠加）**：
  1. **桥方向写成双向**：launch 里拼成 `f'{gz_topic}@{ros_type}@{gz_type}'`，而 `@` = 双向
     （`[` = GZ→ROS，`]` = ROS→GZ，**方向符必须放在 ROS 类型之后**）；且这个 launch **每架机起一份**（5 份）
     → gz→ROS→gz **自激回环**。
  2. 插件发布门限 `current_time += dt` 失效，几乎每个仿真步都发。
* **修法**：障碍桥改 `[` 单向 + 新增 `obstacle_bridge` 参数**只由 base_launch（drone0）起一份**；LED 桥改 `]`；
  插件门限改按 `_info.simTime`（`apply_fixes9/10/13.py`）。
* **验证**：gz 侧回落到 **~38 条/5s**、ROS 侧 **~8 Hz**。
* **坑**：方向符位置写错会被**静默忽略**——不报错、也不建桥，只留一行 timeout 的 SIGTERM。

### 1.4 自制插件"编译成功但从不工作"

* **症状**：gz 日志 `… does not export any plugins. The symbol [GzPluginHook] is missing … No plugins detected in library`。
* **根因**：插件按 Ignition Fortress 写的（`IGNITION_ADD_PLUGIN` 导出 `IgnitionPluginHook`、CMake
  `find_package(ignition-gazebo6)`），而运行的是 Harmonic（只认 `GZ_ADD_PLUGIN` 导出的 `GzPluginHook`）。
* **判据**：`nm -D --defined-only X.so | grep -i PluginHook`；`ldd X.so | grep ignition`（链 Fortress 就是没移植）。
* **修法**：宏改 `GZ_ADD_PLUGIN` / `GZ_ADD_PLUGIN_ALIAS`；CMake 换 `gz-plugin2 / gz-common5 / gz-sim8`
  （目标 `gz-sim8::gz-sim8`、`gz-plugin2::register`）；源码 `ignition::msgs::` → `gz::msgs::`、
  `ignerr/ignmsg` → `gzerr/gzmsg`（补 `#include <gz/common/Console.hh>`）（`apply_fixes7/8.py`）。
* **坑**：gz-sim 8 的 `UpdateInfo::simTime` 是 `steady_clock::duration`（**不是** time_point）。

### 1.5 重建插件后"新代码不生效"

* **根因（两条）**：①运行中的进程对已 dlopen 的库**按路径缓存**（必须换文件名）；
  ②**gz 删除模型时不会销毁插件实例**（泄漏——实测把柱子全删掉、模型数 0 时，话题仍在灌）。
* **结论**：**插件代码改动只有重启 gz 才可靠生效**；"换名 + respawn 模型"只在旧实例是坏的（不发布）时才看起来有效。
* **验证**：`grep -i libDynamicMoving /proc/<gzserver>/maps` 的 inode 与 `ls -li` 对照；`gz topic -i -t …` 的发布者数量。

### 1.6 柱子半截陷进地面（用户报："刚出现时贴地、马上半陷"）

* **根因（两处）**：①插件每步 `set_pose` 只填 `position.x/y`，**z 默认 0**、姿态默认单位阵 → 把柱子按到地面；
  ②SDF 的 box `0.5×0.5×5.0` **以 link 原点为中心**，spawn 在 z=0 → 实际只露 0~2.5 m。
* **修法**：插件补 `set_z()` + 保留姿态；把 `<pose>0 0 2.5 0 0 0</pose>` 烘进 5 个 `dynamic_obstacle_*.sdf`
  的 collision/visual（`apply_fixes9/12.py`）→ 柱子 0~5 m 立在地面。
* ⚠️ **`world.yaml` 与 `models/dynamic_obstacle_*/` 都是启动时生成的**：`launch_as2.bash` 第 82/100 行
  **每次启动**都跑 `utils/generate_world_from_scenario.py`，它会用 `utils/sdf_templates.py` 的
  `SDF_DYNAMIC_OBJECT_TEMPLATE` 重新渲染柱子 SDF（x/y 随机、`initial_angle` 随机、z 恒为 0.0）。
  → 柱子几何的修复**必须写进模板**（`apply_fixes15.py` 已给 collision/visual 加上
  `<pose>0 0 {{ height / 2 }} 0 0 0</pose>`），只改生成物等于埋了颗定时炸弹：
  下次启动重新生成 SDF，柱子又会半埋。（第一轮 fix12 正是这么做的，本轮补上的。）
* **验证**：`gz topic -e -t … -n 1` 里出现 `z:` 字段（=新代码）；视觉上柱子立于地面、顶端 5 m。
  另外可用 `python3 utils/generate_world_from_scenario.py scenarios/scenario1_stage4.yaml -o /tmp/wtest -f world.yaml`
  单独跑一遍生成器，再 `grep -A1 'collision name' /tmp/wtest/models/dynamic_obstacle_0/dynamic_obstacle_0.sdf`
  确认渲染出来的 SDF 里带 `<pose>0 0 2.5 …`。

### 1.7 实时避障永远不触发

* **症状**：日志一路 `No obstacles, following A* path`，一条直线飞到终点。
* **根因**：巡航高度硬编码 `mapf_scenario_4.py:272  cruise_alt = 3.5`（注释却写 4.5 m，是过期注释），而柱子只露到 2.5 m
  → 编队从柱顶**上方**掠过，几何上不可能相遇。
* **修法**：柱子立到 0~5 m（§1.6）+ 增加自检打印
  `Cruise altitude 3.5 m vs obstacle top 5.0 m -> BELOW top (avoidance testable)`。
* **验证**（本机实测日志）：
  ```
  Received data for 5 dynamic obstacles. Starting hybrid navigation.
  Planning path … with A*, obstacles: 25            # 5 柱 ×（当前位置+预测位置）
  Obstacle getting close (0.83m) - using potential fields for avoidance
  Adjusting formation scale to 0.68 (obstacle at 0.83m)
  ```
  且航迹出现横向让位（x 让到 ~1.8 m），不再是直线。

### 1.8 每轮跑完留下脏状态 → 下一轮 takeoff 被拒

* **症状**：`[WARN] … Goal Rejected`；`platform/info` 显示 `offboard: true, armed: false, state: 0`。
* **修法**：`SwarmConductor.reset_all()`——关 offboard → disarm → 用 `gz service /world/empty/set_pose`
  把每架机搬回 `world.yaml` 的初始 xyz；挂在 `finally` 里、rclpy 关闭之前（`apply_fixes11.py`）。
* **验证**：退出时 5 架全 `ok`，位姿回 `(-1,0,0.2)/(1,0,0.2)/(0,0,0.2)/(0,1,0.2)/(0,-1,0.2)`。
* **补充**：从 `TAKING_OFF` 状态调 `platform_land` 会被拒（`success=False`），恢复要重启那一架的 platform 节点。

### 1.9 诊断设施（本次加的，出问题先用它们）

* `mapf_scenario_4.py`：`faulthandler` —— `kill -USR1 <pid>` 会把**全线程 Python 栈**写到 `/tmp/dsh_stack.log`；
  "停滞看门狗"（任何 `print` 都算心跳，45 s 无心跳自动 dump，**只打印不杀进程**）。
* ⚠️ 卡在 `rclpy Client.call()` 的进程 **SIGTERM / Ctrl-C 都杀不掉**（rclpy 装了信号处理器，主线程阻塞在 C 层
  `pthread_cond_wait`）→ **只能 `kill -9`**。

### 1.10 避障行为差：被柱子"推着后退"、该躲不躲（第二轮，2026-10-07）

* **症状**（用户原话）："集群可以稍微变换一下形状就快速通过，但实际表现出来的是整体向后退，迟迟不前进"；
  "几乎要撞上柱子的时候集群还不躲避（如果我没看错就是有一架撞上柱子了）"；
  "躲避行为倾向于后退而不是往侧面躲避"。
* **根因（对着代码逐条定位，四个结构性缺陷）**：
  1. **A* 对柱子完全失明**：A* 把柱子当"半径 0.55 m 的球、球心在 z≈0"，而编队中心在 z=3.5
     → `is_collision_free()` 的 3D 距离恒 > 0.55 m → `has_line_of_sight()` 永远 True
     → **A* 永远只输出一条直线**：所谓"混合导航"里的"规划"这一层其实一直没起作用。
  2. **正面挡路时整体后退**：一旦 `closest_obstacle_dist < safety_radius*2.5`，代码**直接丢弃 A* 路径**，
     改用"指向终点的引力 + 径向斥力"；柱子正面挡路时径向方向与目标方向近乎反向 → 合力把编队往后推。
     而那段"保证前进"的 `min_forward_component` 是**死代码**：只在合力模长 <0.25 时执行，
     可斥力上限是 8.0、5 根柱子叠加，永远不触发。
  3. **安全半径只对编队中心算**：没计入编队自身外接半径（紧编队 = 0.5·scale）。
     日志报"最近 0.83 m 安全"时，最外侧那架机到柱面可能只剩 ~0.2 m
     —— 这就是"快撞上了还不躲 / 像撞了一下"。
  4. **可以规划到赛场外**：`planning_bounds = min/max ± 10 m`（y 能到 22.5 m），
     而类里那句 `self.boundaries = [-10, 10, …]` 从来没被使用过
     → 实测航迹跑到 x = −5.0 贴着赛场边绕行（对比图左panel）。
* **修法**：**不覆盖原文件**，复制出 `challenge_multi_drone/mapf_scenario_4_1.py`
  （由 `apply_fixes15.py` 从 `mapf_scenario_4.py` 生成，可反复重跑；原文件一行未动），
  Phase 2 的导航层整段替换，新增纯几何类 `ArenaAvoidancePlanner`：
  * 柱子按**竖直柱体**建模（含高度判定；横截面是 0.5×0.5 方盒，取外接圆半径 0.354 m），
    安全距离 `safe_axis_dist = 柱外接半径 + 编队外接半径 + 0.30 m` —— **保护的是最外侧那架机**；
  * 只在该**赛场矩形**内规划：bounds 取自 scenario 的 `stage_center/stage_size`，与柱子 SDF 的
    `boundary_min/max` 完全一致（x[−5,5]、y[1,11]）；**横向严格不出赛场**（禁止绕到场地外躲开所有障碍），
    y 只放宽到容纳起飞点/终点；z 恒为巡航高度（禁止爬高绕过）；
  * A* 改为**按到达时刻判定**：把时间维并进扩展代价（`eta = g / v_plan`），柱子扫过去之后格子恢复通行。
    （用"2 秒扫掠管"会把整条走廊**瞬时**封死 → A* 全线失败 → 退化成纯局部反应 → 被两根柱子夹住穿模。）
  * 反应层在"目标方向 ±60°"内采样（规划不出路时放宽到 ±90°，含**停车档**），**字典序**选择：
    ① 最外侧机净空 ≥ 0.30 m 的候选里取分最高 → ② ≥ 0.12 m → ③ 都保不住时**不再看前进量，全向取净空最大**
    （同等净空优先"不后退/侧移"）；只有预测已经贴上才允许 ≤0.25 m/s 的退避，并打 `[ESC-BACKOFF]` 日志。
  * 净空按**双向对冲**算：对轨迹上每点同时查"标称到达时刻"与"滞后 1.43 倍后的到达时刻"，取更小者
    （柱子迎面而来时走得慢反而更危险）。
* **验证**（离线仿真 `sim_compare_avoidance.py`：20 组随机柱子、同一套真实反弹轨迹、同一滞后模型）：

  | 指标 | 旧版决策层 | 新版 `_1` |
  |---|---|---|
  | 到达终点 | 20/20 | 20/20 |
  | 平均用时 | 42.9 s | **25.3 s** |
  | 最外侧机到柱面最小净空 | −0.041 m | **+0.123 m** |
  | 净空 <0（几何穿模 = 撞上） | 1/20 | **0/20** |
  | 后退时间占比 | 22.2 % | **0.2 %** |
  | 后退总位移（均值） | 3.90 m | **0.02 m** |
  | 被夹在赛场边界上的次数 | 58 | **0** |

  * 编队跟踪滞后压力测试（tau = 0.3 s）：穿模 **0/20**（旧版 1/20）、最小净空 **+0.145 m**、后退 0.9 %、用时 39.6 s（旧版 44.2 s）。
  * Part 1 用**原版未被改动的** `astar_plan_path()` 直接证明失明：柱子放在起点正前方 1.0 m 处（z=0）时
    它仍打印 `Direct line of sight to goal found` 并返回 2 点直线；把球心挪到 z=3.5 作对照才会报
    `A* search failed after 10000 iterations`。
  * 复现命令：`bash /mnt/e/…/DSH/wsl-files/run_sim_compare.sh`（不需要 Gazebo、不需要话题，
    只要先 `source setup.bash`）；对比图 `DSH/wsl-files/scenario4_1_compare.png`。
* **本轮踩的两个坑（都是"验证台错了"，记下来免得再犯）**：
  1. 柱子撞到赛场边界会**反弹、速度符号翻转**（插件每步越界就 `v = -v`）。验证脚本一开始喂的是 SDF 里的
     **初始**角速度，于是每次反弹后规划器收到的速度方向都是反的 → 预测彻底跑偏
     （实测：预判"中心轨迹段到柱面最近 0.69 m"，真值其实只有 0.13 m）。真机靠差分
     `/dynamic_obstacles/locations` 得到**当前**速度，所以不会犯这个错；已在 `update_obstacles()` 里加了
     "贴近墙面时速度不可能朝墙外"的兜底翻转，抵消指数平滑在反弹瞬间的滞后。
  2. **别把"危险度"做成软代价**：第一版用 `-w_risk·risk²` 打分，前进量的权重能压过它，
     实测比旧版还差（8/20 穿模）。**安全必须是约束（字典序），不是可以用前进量交换的代价。**
* **调试用的独立验证台**：`check_planner_sim.py` —— 对某一拍的状态，把 `_simulate()` 给的最坏情况净空
  与 0.001 s 步长的暴力枚举对照，要求 `A ≤ C ≤ B`（PASS/FAIL 直接打印）。改预测逻辑后先跑它。

### 1.11 太慢：一趟一分多钟（`_1` 实测 171 s / 207 s）—— 根因在**控制层**，不在规划层

* **症状**（用户）："现在无论是单架无人机还是集群整体都太慢了，一次要一分多钟"。
* **实测诊断**（用 `analyze_trace.py` 读 `_1` 自己写出的两条航迹，这是硬证据）：
  | 指标 | 两次实跑 |
  |---|---|
  | nav 循环时长 / 中心路程 | 206.8 s / 13.93 m，171.5 s / 12.46 m |
  | **指令速度**（规划器输出 \|v\|） | 中位 **0.500 m/s** |
  | **实际速度**（相邻两拍的位移/Δt） | **0.067 / 0.073 m/s**（跟踪效率 **13~15%**） |
  | 循环实际周期 | **1.06 ~ 2.33 s**（代码里设计的是 0.3 s） |
  | 其中 `TRACK` 档（**没有避障介入**） | 50~73% 的时间，速度也只有 7 cm/s |
  → 结论：**不是规划/避障拖慢的，是每拍下发指令的方式拖慢的**。
* **根因（三层，逐层都实测过）**：
  1. `as2_python_api/behavior_actions/behavior_handler.py` 的 `start()`：
     ```python
     send_goal_future = self.__action_client.send_goal_async(goal_msg, feedback_callback=...)
     while not send_goal_future.done():
         sleep(0.1)          # 每次 go_to 都要阻塞等目标被接受，0.1s 粒度
     ```
     而且 `modify()` 就是 `raise NotImplementedError` —— **没有"更新目标"这回事**，
     每次重发目标都是"取消 + 重启整个 GoTo 行为"。5 架串行 → 每拍 1~2 s。
  2. 循环里 `dt = max(0.05, min(0.6, dt))` 把真实 Δt（1.9 s）夹到 0.6 s，
     于是**每拍指令位移 = v·0.6**、而这一拍实际花了 1.9 s
     → 等效指令速度只有 0.5×0.6/1.9 ≈ 0.16 m/s，**循环速率成了编队的速度上限**。
  3. 目标是**相对实测位置**生成的（`next_center = center + v*dt`）：跟踪器每拍只走完一步的一部分，
     下一步又从这个"只走了一点"的新位置起算 → 自己把自己锁在低速上
     （离线仿真里复现了同一现象：决策周期越小，这个模型算出来的稳态速度越低）。
* **修法（`_2` = `mapf_scenario_4_1_2.py`，只换控制层与节奏，避障/规划逻辑一行未动）**：
  1. **控制层换成 20 Hz 直发 `/motion_reference`**（AS2 官方 teleop 接口
     `as2_motion_reference_handlers.position_motion.PositionMotion`，不经过任何 action）：
     循环周期 ~0.05 s，不再有"等目标被接受"的阻塞。
     内部只在**控制模式变化时**做一次阻塞服务调用（`controller/set_control_mode`），
     因此加了保护：首拍验证 + 单次 >0.5 s 记账，连续 3 次或首拍失败就把那架机（或全部）降级回 `go_to`。
  2. **目标改为"槽位 + 前视"**：`setpoint = 编队槽位 + v * lead_time`。
     位置环是 P 控制（`kp=1.0`，见 `config_sim/config/pid_speed_controller.yaml`），
     跟随匀速目标时有 `v/kp` 的稳态滞后，取 `lead_time = 1/kp = 1.0 s` 正好抵消
     → **实际速度 ≈ 指令速度**；落后时误差变大、自动加速追赶（自补偿）。
     接近终点时 `lead_time` 自动收缩到"剩余距离/速度"，不会冲过头。
  3. **巡航速度 0.5 → 1.0 m/s**（`S4_SPEED["v_max"]`），前视跟踪点 0.9 → 1.2 m。
  4. **节奏类固定开销**：起飞速度 0.8→1.2、落地 0.5→0.8、起飞后等待 2.0→0.5 s、
     编队过渡速度 0.7/0.8→1.2、过渡轮询 0.5→0.2 s、障碍数据采集 2.0→1.0 s。
* **验证**：
  * 离线三方对比（`sim_compare_speed.py`，20 组随机柱子、同一套反弹轨迹、同一世界系滞后模型）：

    | 指标 | 旧版决策层 | `_1` | **`_2`** |
    |---|---|---|---|
    | 用时 均值 (s) | 42.9 | 25.2 | **12.8** |
    | 用时 最慢 (s) | 112.5 | 30.0 | **13.9** |
    | 最外侧机最小净空 (m) | −0.041 | 0.136 | **0.292** |
    | 净空 <0（穿模） | 1/20 | 0/20 | **0/20** |
    | 后退时间占比 | 22.2 % | 0.2 % | **0.0 %** |
    | 编队滞后 tau=0.3s 时的用时/净空 | 43.4 / −0.121 | 25.6 / 0.148 | **13.0 / 0.335** |

    即：**速度翻倍的同时安全反而更好**（净空从 0.136 提到 0.292，因为巡航更快、每拍决策更密，
    避障是提前让行而不是最后一刻）。
  * 接口烟测（`check_stream_api.py`，用 `ROS_DOMAIN_ID=99` 与在跑的栈隔离，不影响 Gazebo）：
    `PositionMotion` 可构造（0.18 s）；话题解析为 `/drone0/motion_reference/{pose,twist,trajectory}` ✓。
  * **待真机确认的部分**：仿真只建模"编队中心按指令速度前进"，测不出控制接口的改进。
    `_2` 的预期是 nav 段 **15~25 s**（对照 `_1` 实测 171/207 s）。
    跑起来每秒会打印一行 `cmd=… ach=… m/s (…%)`；若 `ach` 明显低于 `cmd`，
    日志会直接给出排查提示（改 `_MOTION_MODE="go_to"` 对照、看 `controller/info` 的 `control_mode`、
    看 `/drone0/motion_reference/pose` 是否有 20 Hz）。
  * 结束时会打印**分段耗时**（settle_formation / collect_obstacles / navigation / final_formation），
    下一次再慢就能一眼看出是那一段。

### 1.12 `_2` 在 1.0 m/s 下两次撞柱失控 → 真因是**控制链路能把循环卡死几十秒**（`_3` 安全档）

* **症状**（用户原话）："这次好像有点过于激进了，两次测试都有无人机撞上柱子然后失控。"
  另外两次跑完 **`traces/` 里连一个 csv 都没有** —— 因为 `_2` 是在导航循环结束后才写文件的，
  撞机被中断就什么都留不下（这本身也是 `_3` 要修的）。
* **首要根因（读源码定位，不是猜）**：`as2_motion_reference_handlers/basic_motion_references.py`
  ```python
  def send_pose_command(self):
      if not self.check_mode(): return False          # 模式不符 -> __set_mode()
      ...
  def __set_mode(self, mode):
      cli.wait_for_service(timeout_sec=3)             # ① 先等 3s
      resp = set_control_mode_cli_.call(req)          # ② 阻塞服务调用
      while self.current_mode_.control_mode != mode.control_mode:
          if 超时 5s: return False                    # ③ 再轮询最多 5s（0.1s 粒度）
          time.sleep(0.1)
  ```
  而 `_2` 的降级门槛是"**同一架机累计 3 次 >0.5 s**"才降级 → 一旦模式切不过去：
  每架每拍阻塞 ≈6 s、5 架串行 ≈ **25 s/拍**；这 25 s 里无人机仍按**上一条参考**全速飞
  → 盲飞撞柱 → 碰撞后被弹开/姿态失控。**与避障算法无关，换任何速度都危险。**
* **次要根因**：① `_2` 把 Phase 1（起飞→编队）过渡速度提到 1.2 m/s，而 **Phase 1 完全没有避障逻辑**；
  ② 安全半径用的是"假定五边形"`0.5*scale`，没用**实测**队形外接半径；
  ③ 单机偏差没有任何纠正手段（所有机共用"中心速度 + 槽位"）。
* **修法**（`_3` = `mapf_scenario_4_3.py`；避障/规划逻辑与 `_1` 同一套，只改控制与安全参数）：
  1. **控制链路预检**：起飞后每架发一条"原地保持"参考，用**线程 + 1.0 s 硬超时**兜住；
     超时/失败该机直接走 `go_to`；全部失败则整体走 `go_to`。
  2. **单拍发送硬性时限**：任何一拍发送总耗时 >0.5 s → **立刻**整体降级为 `go_to`
     （把"连续 3 次"改成"一次就降"），且每拍都查。这是对首要根因的正面修复。
  3. **默认速度回到 0.5 m/s**（用户已验证过的值）。提速交给控制层：
     `_1` 实测只有 0.067 m/s，把它修到 ≈0.5 本身就是 7 倍以上，不需要靠"飞得快"。
     参数仍是一处可调：`S4_SPEED_SAFE["v_max"]`（离线看 0.7 也安全）。
  4. 安全半径改**实测**：`step(drone_positions=…)` → `max(名义, max_i|p_i−center|) + 0.20 m 误差预算`；
     余量 0.30→**0.40 m** 并随速度放大（+0.20·v）；预测的滞后对冲 `eta_slack` 0.70→**0.55**。
  5. **单机护栏**：某机自身净空 <0.45 m 时，只给它加一个背离最近柱子的偏移（≤0.25 m）。
  6. 参考点改**世界系积分 + 间隙上限 0.5 m**，去掉 `_2` 的"位置前视"（避免把目标点推到计划点之前）。
  7. Phase 1 过渡速度**不提高**（只缩短等待：起飞后 2.0→0.5 s、过渡轮询 0.5→0.2 s、落地 0.5→0.8 m/s）。
  8. **航迹边跑边落盘**（`traces/scenario4_3_*.csv`，新增 `reff`/`send_ms` 两列）→ 撞机/中断也留证据。
  9. **检测到几何重叠（最危险那架机净空 <0.05 m）立即中止**编队并进入降落（`_ABORT_ON_CONTACT`）。
* **验证**（逐机仿真 `sim_drone_level.py`：每架机各自 kp 差异 0.8~1.2 + 速度饱和 + 平台速度响应，
  净空取 **5 架机里最危险的那架**，20 组随机柱子）：

  | 场景 | 用时均值 | 最危险净空 | 净空<0 | <0.15m |
  |---|---|---|---|---|
  | `_2` 1.0 m/s（plan2 无额外裕度） | 13.4 s | 0.312 m | 0/20 | 0/20 |
  | `_2` 0.5 m/s（同口径） | 26.0 s | **0.102 m** | 0/20 | 2/20 |
  | **`_3` 0.5 m/s（安全档）** | 29.9 s | **0.431 m** | **0/20** | **0/20** |
  | `_3` 0.7 m/s | 22.0 s | 0.500 m | 0/20 | 0/20 |
  | `_3` 1.0 m/s | 14.6 s | 0.427 m | 0/20 | 0/20 |

  安全档把最危险净空抬高 **4 倍**（0.102→0.431 m），代价只有 15% 时间。
  ⚠ **但仿真测不到"控制链路阻塞"**（真机撞柱的主因），那部分只能靠 ① 预检 ② 单拍时限
  ③ 真机日志里的 `ctl=`/`send=` 字段确认。
* **本轮教训**（已并入 §4 铁律）：给"可能阻塞数秒的第三方调用"设降级阈值时，
  **"连续 N 次才降级"是错误设计**——一次就够降了；并且必须给**整个控制周期**设硬时限，
  否则一次阻塞就等于让无人机盲飞好几秒。

---

### 1.13 「倾向于直飞、明明可行的路径却不去执行」——反应层方向锥锚错了地方（第四轮，2026-10-08）

* **症状**：用户报"集群还是倾向于直飞而不是向障碍物躲避，导致很多明明可行的路径却不去执行"。
  同期 `_3` 真机航迹（1293 拍）：`path_pts>2`（A\* 确实规划出绕行）占 **60%**，
  但反应层模式里 `ESC-BACKOFF` 占 17.6%、`AVOID-TIGHT` 占 20% —— 计划在执行层被"吃掉"了。
* **根因**：规划器 `step()` 里候选方向锥的锚点是 `base_ang = atan2(到终点)`，
  只在 **目标方位 ±max_angle(60°)** 内采样；打分是
  `w_progress(1.0)×前进量(按到终点的投影) + w_align(0.35)×贴合路径`。
  锥边缘朝终点一侧的 `prog` 增益（≈1.4×(1−cos60°)≈0.7）**远大于** `align` 惩罚（≈0.25），
  所以"最高分"永远是锥内最靠终点的那个方向 —— A\* 绕出来的路径只是打分里的软项，
  永远被"直奔终点"压住。**表现就是"计划了但没执行"、"看起来在直飞"。**
* **修法**（`new_planner_class_2.py`，由 `make_planner_2.py` 从 `_1` 生成）：
  1. 有路径时 `base_ang = 路径切向 pu`（没路径才用终点方位）；
  2. 有路径时"前进量"改按**沿路径切向**推进算；
  3. "不许朝终点反方向"从"靠锥宽保证"改成**显式硬约束**
     `if vx*gu[0]+vy*gu[1] < 0: continue`。
* **验证**：离线 `python3 check_plan4.py`（B 项锥跟随路径、C 项 100 组随机场景不后退）；
  真机 `_4`：`scale 分布 1.00:100%`、`planner stats: emergency 0 / backoff 0`、
  模式分布 `TRACK 62% / AVOID 33% / FALLBACK 5%`（**一个 ESC-\* 都没有**）。

### 1.14 【致命一行】`streamers` 从未登记 → 全程**静默**退化成 go_to（"速度又变慢了"的真主因）

* **症状**：无论怎么调 `v_max`、怎么改规划器，一趟都要一分多钟；航迹里 `ach/cmd ≈ 0.3`。
* **根因**：`navigate_through_dynamic_obstacles()` 的控制链路预检循环里，
  **只有失败分支**写了 `streamers[id] = None`，**成功分支漏了 `streamers[id] = st`**
  → `n_ok` 恒为 0 → 每次启动都打这句：
  `[plan4] 没有任何一架通过控制链路预检 -> 整体走 go_to`。
  真机实测最刺眼的一次：**5 架预检全部 OK，紧接着就是这句**。
  也就是说：`_3` 那三趟真机数据、以及 `_4` 的前两趟，**"控制层修复"从来没有被执行过**，
  全部跑在 `_1` 时代那条慢速 go_to 上（每 1.0s 才重发一次目标）。
* **修法**：`PositionMotion` 构造成功后立即登记（`new_nav_method_4.py` 的 N9a）。
* **验证**：启动日志出现 `控制层 = 直发 /motion_reference 20Hz，5/5 架可用`；
  每拍第二行结尾是 `直发 5/5 架`；收尾是 `结束档位 stream`、`控制层降级次数 0`。
  **以后任何"变慢了"的排查，第一步就看这三处。**

### 1.15 控制链路预检阈值 1.0s 太紧 → 健康的无人机被判超时

* **症状**：预检随机把几架机判成"超时"，然后它们整趟走慢速回退路。
* **根因**：HOVER→POSITION 的**第一次**模式切换本身就要 0.8~1.4s。
  实测样本：0.657 / 0.736 / 0.753 / 0.797 / 0.819 / 0.826 / 0.902 / 0.958 / 0.971 /
  1.018 / 1.020 / 1.091 / 1.176 / 1.278 / 1.368 / 1.409 s —— **1.0s 正落在这条分布的正中间**。
* **修法**（N6a/N6b）：阈值 3.0s；**5 架并行探测**（原来串行，5 架各 ~1s，看着像卡死）；
  首次超时**重试一次**（此时模式往往已经切过去、第二次秒回）；两次都超时才降级
  —— 真正的病态情形是单次 ~6s（`wait_for_service(3s)` + 轮询 5s），3.0×2=6.0s 仍拦得住。
* **验证**：启动日志应有 `控制链路预检 OK (x.xxxs, 第 1 次)` ×5。

### 1.16 起飞门槛：1.0s 收不齐 5 根柱子 → 编队一头撞进"看不见的柱子"

* **症状**：`received data for 3 dynamic obstacles`；随后
  `clear_drone 0.08 -> 0.03`、`检测到与柱子几何重叠 ... @ t=51.3s` 中止。
* **根因**：实测（`gz topic -e` 与 rclpy 逐 id 计数对比）——
  **gz 侧**每根柱子 18 条/5s = 3.6Hz，但 **ROS 侧**每根只剩 7 条/5s ≈ **1.4Hz**（桥丢了大半），
  5 个 id 都能到、但都很稀。1.0s 的收集窗口按 coupon collector 只能看到 ≈3.8 根。
  那一趟只认出 3 根（`object_35/39/68`），A\* 与净空判据都看不到另外两根 → 撞上去。
* **修法**（N9b）：收集改为"**凑齐 5 根** 或 6s 无新增才继续"；不足时**大声警告**并打印已见 id；
  期望根数可用环境变量 `S4_EXPECT_OBSTACLES` 覆盖。
* **验证**：正常应打印 `柱子凑齐 5/5，用时 x.xs`；不足时是
  `!! 6s 内无新增，只收到 N/5 根柱子 ([...])：**场地里有看不见的柱子**`。
  **看到 3/5 就别信这一趟的碰撞结论**（第四轮第 2 趟就是这么撞的）。
* **残留**：即使这样，`_4` 的两趟真机仍只收到 3/5（同一批 id），
  而**同时**用独立 rclpy 探针测同一个话题能看到 5/5 各 1Hz —— 说明丢的是
  **任务脚本自己那个订阅**（挂在 drone0 节点上），不是桥。已加警告，**真因待查**。

### 1.17 为什么"慢"里有一大半其实是**仿真**慢：RTF≈0.4（附加速方案，需重启栈）

* **症状**：指令 0.7 m/s，墙钟算出来的实际速度只有 0.23 m/s。
* **根因**（实测）：
  * `gz topic -e -t /world/empty/stats` → `step_size = 4ms`、`real_time_factor ≈ 0.48~0.65`，
    `sim_time/real_time = 425/887 = 0.479`；
  * `ps` → `gz sim server` 长期 **0.95~1.0 核**（16 核机器，单核跑满）；
  * gdb 采样 server 主线程 → 热点全在 `libgz-sim-physics-system.so` 的
    `Physics::Update → PhysicsPrivate::ChangedLinks`，其余线程在 `Barrier::Wait()`；
  * 插件 `dynamic_object.cpp` 的 `PreUpdate` **每个物理步**都发一次
    `/world/<w>/set_pose`：4ms 步长 → 250 次/s × 5 根柱子 = **1250 次/s** 的位姿写入。
  → 结论：**所有墙钟时间天然 ×2.5**。0.7 m/s 的仿真时间速度，在墙钟上就是 0.27 m/s。
* **修法（需要重启仿真栈才生效；当前 server 仍用旧 .so）**：
  * `python3 patch_obstacle_plugin.py` + `colcon build --packages-select dynamic_moving_objects`
    → 把 set_pose 限频到 50Hz（位置改由插件自己逐步积分，**轨迹逐字不变**）；
  * 可选：`max_step_size` 4ms → 8ms（physics 250Hz→125Hz），理论上 RTF 再翻倍。
* **验证**：重启前后各跑一次 `timeout 6 gz topic -e -t /world/empty/stats -n 2`，
  对比 `real_time_factor`；目标 ≥0.8。
* **注意**：本项**尚未在重启后实测**，属于"有依据的推断 + 已落盘的补丁"，不是已验证结论。

### 1.18 每拍耗时账（分段计时已经把时间花在哪算清了）

* 真机 `_4` 实测（`traces/scenario4_4_*.csv` 的 `t_pos_ms/t_plan_ms/t_step_ms` 列）：
  读位姿 **0.1 ms**、A\* 重规划 **8.3 ms**（最大 313 ms）、反应层 **14.9 ms**、
  发送 **30.9 ms**（p95 55.9、最大 98.5，预算 500ms，**0 次降级**）
  → 墙钟环频 15.3Hz，换算成**仿真时间** 40Hz（目标 20Hz，已经够用）。
* **根因（两处纯浪费）**：
  * `_simulate` 对 44 个候选各算 8 步 × 2 个时刻 × 5 根柱 ≈ **1.8 万次** `_pred_xy`，
    而 `t` 在候选之间**完全重复**（就是 `k*sim_dt` 与 `t/eta_slack`）；
  * `_dist_now_grid()`（4132 格 × 5 柱）**与 scale 无关**，但 `best_scale` 对每个候选缩放
    都调一次 `plan()` → 一次重规划最多算 5 遍。
* **修法**：`_pred_xy` 加每拍缓存（key=`(柱子id,t)`，结果完全等价）；
  `_dist_now_grid` 每拍只算一次；`best_scale` 并列时改为优先**更大**的编队
  （原来是更小 —— 那等于鼓励"缩到最小从缝里挤过去"，与"绕开柱子"的意图相反）。
* **验证**：`python3 check_plan4.py`（A 项计时：`best_scale` 180.7→90.0 ms、
  `step` 11.6→8.1 ms、合计 29.7→17.1 ms/拍）。

### 1.19 `_4` 与 `_3` 的真机对照（2026-10-08 12:09，同一套仿真栈、柱子随机位置不同）

| 指标 | `_3`（0.5 m/s，实为 go_to） | `_4`（0.7 m/s，真 stream） |
|---|---|---|
| nav 段墙钟 | **138.8 s** | **55.6 s** |
| 仿真时间 | 55.5 s | 21.2 s（RTF 0.384） |
| 控制层 | go_to（预检被误判） | **直发 20Hz，5/5 架** |
| 控制效率（仿真时间口径） | 66% | **89%** |
| 仿真时间平均速度 | 0.46 m/s（指令的 66%） | **0.574 m/s（82%）** |
| 最危险那架机净空 | 0.198 m | **0.803 m** |
| `emergency` / `backoff` | — / 227 拍 | **0 / 0** |
| 编队 scale | 降到 0.60 | **全程 1.00** |
| 横向绕行幅度 | 4.20 m（来回摆） | 0.65 m（只在要绕的地方绕） |
| 落盘证据 | 只有编队中心 | 逐机 (x,y)/目标偏差/自身净空 + RTF + 分段耗时 |

**结论**：两个用户观测（"倾向直飞"、"又变慢"）各有独立根因，均已定位并修掉；
`_4` 在"更快（2.5×）+ 更安全（净空 4×）"上同时改善。
剩下最大的墙钟开销是 RTF 0.38（§1.17），那一项需要重启仿真栈才能验证。

### 1.20 「只收到 3/5 根柱子」的真根因与修法（第五轮，2026-10-08）——**已解决**

* **症状**：`received data for 3 dynamic obstacles`，随后编队一头撞进"看不见的柱子"
  （`clear_drone 0.08 -> 0.03`、`检测到与柱子几何重叠 @ t=51.3s` 中止）。
  规划器看不见的柱子，A\* 与净空判据全都无能为力 —— 这很可能同时是
  "看着像直飞不躲避"和"躲完容易失控"的来源。
* **取证**（plan5 加的逐 id 计数）：
  * 挂在 **drone0 节点**上的那路订阅：`{object_38: 21, object_92: 8, object_121: 3}`
    —— **只有 3 个 id**；
  * 同一时刻用**独立进程**探针订阅同一话题：**5 个 id 各 ~1 Hz 全都在**；
  * gz 侧 `gz topic -e` 也是 5 个 id 各 3.6Hz。
  → 丢数据的既不是插件也不是桥，**是任务脚本自己那个订阅**（节点/QoS 环境问题）。
* **修法**（plan6，`new_nav_method_6.py` M2）：另起一个**干净的独立 rclpy 节点**，
  用 `rclpy.qos.qos_profile_sensor_data`（best-effort）再订一份，两路写同一个 dict（取并集）；
  **必须在"凑齐柱子"的收集窗口之前创建**（plan5 建晚了，收集时它的计数恒为 `{}`，白建）。
* **验证**：真机 plan6 日志 `柱子凑齐 5/5，用时 0.0s`（窗口开启前就齐了）、
  `received data for 5 dynamic obstacles`。**这是后面所有避障结论的前提**。

### 1.21 负结果：把 `safety_margin` 调大反而更差（第五轮）

* **动机**：用户报"前面有障碍物仍倾向直飞、不侧向躲"，想用"更早更宽"的绕行来治 →
  plan5 把 `safety_margin` 0.50→0.60、`pure_pursuit_lookahead` 1.10→1.90、
  `predict_horizon` 2.0→2.5（速度不动）。
* **实测（plan5，真机）**：`plan_fail 239/353`、`no_feasible_path 47`、`emergency 111`、
  `backoff 67`、编队被迫缩到 **scale 0.5**、最危险净空一度掉到 **0.20 m**、
  nav **84.1 s**（plan4 是 55.6 s）。**比改之前更差。**
* **根因**：`safety_margin` 是 **A\* 的硬可行性门槛**（`safe_axis_dist = r_pillar + R_f + margin`），
  不是软代价。调大 → 走廊在参数上直接不可行 → A\* 找不到路 → 掉进应急/逃逸层（ESC-\*）
  → 行为更差、更慢、更贴柱。`lookahead` 1.9 又让编队朝路径远处切内角。
* **修法**：plan6 参数**回退到 plan4 那一组**（margin 0.50 / lookahead 1.10 / predict_horizon 2.0）。
  想"更早绕"就该动**代价权重**（`line_weight`/`near_weight`）或路径跟随，**不要动硬门槛**。
* **教训**：调参必须带"是否更差"的判据（`plan_fail / emergency / backoff / scale 分布 /
  逐机净空 / nav 时长`六个数一起看），否则会把负结果当进展。

### 1.22 柱子活动区收缩（治"起飞就被柱子撞"）——**需重启栈**

* **症状**（用户原话）："集群初始位置离障碍物场地过近，起飞的时候容易被柱子撞到，调整一下。"
* **根因（实测数字）**：柱子活动区 == 赛场本身：`boundary_min_x=-5.0 max_x=5.0
  min_y=1.0 max_y=11.0`（`utils/generate_world_from_scenario.py` 用 `center ± size/2`）。
  柱体是 0.5×0.5 方柱、高 5 m → 柱子停在 y=1.0 时**柱体最南到 y=0.75**；
  而起飞后五边形集结点在 (0,0)、半径 0.8，最北机位 **y=+0.76** → 差 1 cm，
  **柱子一停边界就压在编队上**。柱高 5 m 意味着垂直起飞也要穿过它的 0~5 m 体段，
  **所以"升高度"解决不了，只能收缩活动区**。
* **修法**：`python3 patch_pillar_bounds.py` —— 在生成器的 `boundaries` 里加收边，
  默认 `min_y` 收 1.5 m（→ 柱体最南 2.25 m，离集结点 1.49 m）；
  可用 scenario 的 `stage4.obstacle_boundary_inset_y_min` / `obstacle_boundary_inset` 覆盖。
* **生效条件**：柱子 SDF 每次启动都重新生成 → **必须重启整套仿真栈**（可顺带一起应用
  §1.17 的 set_pose 限频补丁）。回滚：`cp out_of_date/utils/generate_world_from_scenario.py.bak-dsh21 …`。
* **验证**：重启后 `grep boundary_min_y config_sim/world/models/dynamic_obstacle_0/*.sdf`
  应看到 2.5；plan6 的"开局自检净空"应从 1.89 m 升到 ≥2 m。

### 1.23 plan6 真机结果（5/5 首次看齐后的第一个数据点，2026-10-08 12:36）

| 指标 | 数值 | 解读 |
|---|---|---|
| nav 段 | 75.9 s（仿真 30.4 s，RTF 0.41） | 比 plan4 的 55.6 s 慢 |
| 控制层 | 5/5 架直发、效率 **92%**、仿真时间环频 27.6 Hz、**0 次降级** | 控制层没问题 |
| 中心路径 | 16.66 m / 直线 12.04 m（绕行系数 **1.384**） | 确实在绕，不是直飞 |
| 编队 scale | **0.50 占 69%**、0.60 占 23%、1.00 占 8% | 5 根柱子下宽编队不可行 → 一直在"挤" |
| 反应层 | AVOID 63% / AVOID-TIGHT 19% / ESC-BACKOFF 7% | `emergency 69 / backoff 59` |
| **逐机最危险净空** | **+0.040 m**（d1），15 个"机·拍" <0.15 m | **安全余量被吃掉了** |
| 编队中心口径净空 | +0.394 m | 只看中心会严重高估安全 |
| A\* 单次最大 | 1393 ms | 多缩放回退时的卡顿 |

**结论**：5/5 看齐后，plan4 那组参数（0.7 m/s + margin 0.50）在 5 根柱子的真实场景里
**偏紧**——表现为长期缩编队 + 应急逃逸 + 逐机净空 4 cm。
下一步的候选（**尚未实测**）：①`v_max` 0.7→0.5（牺牲时间换净空）；
②`per_drone_guard` 0.55→0.70 / `per_drone_push` 0.35→0.45（单机护栏更早介入，
直接针对本例 4 cm 的"某一架"）；③把 `scales` 下限从 0.5 抬到 0.7 并**接受更长绕行**
（不再靠缩编队过缝）。**判据**用 §1.21 那六个数一起看。

## 2. 日常操作

### 2.1 启动

```bash
cd ~/projects/aerostack-swarm-drone-traversal/challenge_multi_drone
./launch_as2.bash        # tmux 5 会话 + Gazebo + 5 架机（base_launch=drone0）
```

pane 拓扑（单独重启组件时要知道打哪）：

| 组件 | pane |
|---|---|
| Gazebo server+gui（`as2_gazebo_assets launch_simulation.py`） | `drone0:0.0` |
| platform 节点 | `drone0:0.1` / `droneN:0.0` |
| 障碍桥（`plugin_gz_bridges.launch.py`） | `drone0:0.2` / `droneN:0.1` |

### 2.2 跑任务

```bash
# ★ 当前唯一推荐档：5/5 根柱子（双路订阅）+ 0.7 m/s + 全程直发参考
python3 mapf_scenario_4_6.py --scenario_file scenarios/scenario1_stage4.yaml
```

被取代的旧版本已移到 `out_of_date/`，要对照时从那里跑。**注意两点**：
①`mapf_utils.py` 留在原位，所以必须加 `PYTHONPATH=.`（否则 `from mapf_utils import *` 失败）；
②这些脚本按 `__file__` 定位航迹目录，航迹会写进 `out_of_date/traces/`。

```bash
PYTHONPATH=. python3 out_of_date/mapf_scenario_4.py     --scenario_file scenarios/scenario1_stage4.yaml  # 原版：避障失明（§1.10）
PYTHONPATH=. python3 out_of_date/mapf_scenario_4_1.py   --scenario_file scenarios/scenario1_stage4.yaml  # [plan1] 重写避障，0.5 m/s，仍走 go_to
PYTHONPATH=. python3 out_of_date/mapf_scenario_4_1_2.py --scenario_file scenarios/scenario1_stage4.yaml  # [plan2] 1.0 m/s 速度版，**已知撞柱，不要再跑**
PYTHONPATH=. python3 out_of_date/mapf_scenario_4_2.py   --scenario_file scenarios/scenario1_stage4.yaml  # 同 _1_2 的副本
PYTHONPATH=. python3 out_of_date/mapf_scenario_4_3.py   --scenario_file scenarios/scenario1_stage4.yaml  # [plan3] 安全档（直发参考其实从未生效，§1.14）
PYTHONPATH=. python3 out_of_date/mapf_scenario_4_4.py   --scenario_file scenarios/scenario1_stage4.yaml  # [plan4] 当时只看得见 3/5 根柱子
PYTHONPATH=. python3 out_of_date/mapf_scenario_4_5.py   --scenario_file scenarios/scenario1_stage4.yaml  # [plan5] margin 0.60，真机更差
```

真正绕障后约 **270 s**（无遮挡时 137 s）；退出会自动复位 5 架机。

`_1` / `_2` 启动后先打印一段几何自检（**先看这段，数字不对就别飞**）：

```
[plan2] arena (stage_center±stage_size/2) = x[-5.00,5.00] y[1.00,11.00]
[plan2] nav bounds (横向不外扩)          = x[-5.00,5.00] y[0.30,12.70]
[plan2] pillar: 截面 0.50x0.50 方柱 -> 外接半径 0.354 m; 高 5.00 m -> z[0, 5.00]
[plan2] cruise 3.50 m 在柱子高度区间内 -> 实时避障可以被触发
[plan2] speed profile: v_max=1.00 m/s  speed_fracs=(1.0, 0.7, 0.45, 0.0)  scales=(1.0, 0.85, 0.7, 0.6, 0.5)
[plan2] control: stream @ 20 Hz  lead_time=1.00s (kp=1.0 的稳态滞后补偿)
```

每拍进度行形如（`cmd`=指令速度、`ach`=实测速度、括号里是跟踪效率）：

```
[plan2] t=   8.2s center=(  0.13,  5.42) d_goal= 7.08 scale=0.85 path=  3 clear_drone= 1.02 v=TRACK   side=  +0.0deg | cmd=1.00 ach=0.97 m/s ( 97%)
[plan2] t=  13.1s center=( -1.24,  8.10) d_goal= 4.41 scale=0.70 path=  4 clear_drone= 0.34 v=AVOID   side= -30.0deg | cmd=0.65 ach=0.61 m/s ( 94%)
```

结束时另存一份逐拍航迹 `challenge_multi_drone/traces/scenario4_4_<时间戳>.csv`
（列：`t,center_x,center_y,vx,vy,d_goal,clear_drone,reff,scale,path_pts,mode,cmd_speed,ach_speed,send_ms,sim_t,rtf,…`），
并打印 `planner stats = {...}` 与**分段耗时**表。
用 `analyze_trace.py` 回看任意一条航迹（时长/实际速度/分档位/里程碑/最小净空一次看全）。

> `traces/` 已于 2026-10-08 清空（§6）；重跑任意档都会重新生成，文件名前缀仍是 `scenario4_4_`。

### 2.3 自查命令（附"正常值"）

| 想看什么 | 命令 | 正常 |
|---|---|---|
| 障碍话题频率(ROS) | `ros2 topic hz /dynamic_obstacles/locations` | 几 ~ 十几 Hz |
| 障碍话题频率(gz) | `timeout 6 gz topic -e -t /dynamic_obstacles/locations` 再接 `grep -c frame_id` | ~40 条/5s |
| 话题类型 | `ros2 topic info /dynamic_obstacles/locations` | `geometry_msgs/msg/PoseStamped` |
| 单机状态 | `ros2 topic echo --once /droneN/platform/info` | `armed/offboard/state`（1=LANDED,2=TAKING_OFF,3=FLYING,0=DISARMED） |
| 插件是否加载 | gz 终端里找 `Publisher initialised` / `Failed to load` | 应只有前者 |
| 桥方向 | 桥 pane 里的 `Creating GZ->ROS Bridge` / `ROS->GZ` | 障碍桥只应是 GZ->ROS |
| 环境对不对 | 任务日志首行 `[fix5] active rmw implementation` | `rmw_cyclonedds_cpp` |
| **新版几何对不对** | 新版日志的 `[plan1] … 几何自检` 段 | `arena x[-5.00,5.00] y[1.00,11.00]`；`nav bounds` 的 x 也是 `[-5.00,5.00]`（横向不外扩）；`cruise 3.50 m 在柱子高度区间内` |
| **新版是否真在规划** | 新版日志 | 出现 `path=` 非 0、`side=` 非 0°；跑完 `plan_ok` > 0 且 `plan_fail/plan_ok` 不接近 1 |
| **新版是否还在后退** | 新版日志 `v=` 列 + `planner stats` | 绝大多数是 `TRACK/AVOID/AVOID-TIGHT`；`backoff` 应为 0 或极小（出现就说明它真的被逼到墙角了） |
| 航迹落盘 | `ls -t challenge_multi_drone/traces/ \| head` | 每次跑生成一个 `scenario4_*.csv`，末列 mode、`clear_drone` 列是最外侧机的真实净空 |
| **速度跟不跟得上** | `_2` 日志每行的 `cmd=… ach=… (…%)` | `ach/cmd` 应 >85%；<35% 会自动打出排查提示 |
| **控制模式对不对** | `ros2 topic echo --once /drone0/controller/info` | `control_mode` 应为 POSITION（=2），yaw_mode = YAW_ANGLE |
| **直发参考有没有在发** | `ros2 topic hz /drone0/motion_reference/pose` | `_2` 的 stream 模式下 ~20 Hz；`_1` 下这个话题是 go_to 行为在发（也很快，但那不是我们的循环） |
| **各段耗时** | `_2` 结束时的「分段耗时」表 | navigation 应是主体；`collect_obstacles`≈1 s、`settle_formation` 数秒 |
| 回看任一条航迹 | `python3 DSH/wsl-files/analyze_trace.py` | 打印时长/实际速度/分档位统计/d_goal 里程碑/最小净空 |
| **`_3` 控制链路是否通过预检** | 启动后 `[plan3] … 控制链路预检 OK/超时/失败` | 5 架都应 `OK`；任何一架超时/失败就说明模式切换有问题，此时它会自动走 go_to（仍能跑） |
| **`_3` 有没有发生降级/盲飞** | 日志 `!! 本拍发送耗时 …ms 超过预算` 与结束时的「控制层降级次数」 | 应为 **0 次**；出现就说明控制链路阻塞过，需要查 `controller/set_control_mode` |
| `_3` 单机净空与队形半径 | 每秒行里的 `clear_drone=` / `Reff=` | `clear_drone` 应 >0.15 m；`Reff` 若明显大于名义值(0.5·scale)说明队形被跟踪误差撑开 |
| `_3` 逐拍发送耗时 | `traces/scenario4_3_*.csv` 的 `send_ms` 列 | 正常 <50 ms；接近 500 就会被降级 |

### 2.4 单独重启某个组件（**不碰 Gazebo**）

```bash
tmux respawn-pane -k -t <session:win.pane> '<命令>'
```

**pane 索引别猜**：从目标子进程沿 `/proc/<pid>/stat` 的 ppid 上溯到 `pane_pid`，
再用 `tmux list-panes -a -F '#{pane_pid} #{session_name}:#{window_index}.#{pane_index}'` 反查。
现成实现见 `DSH\wsl-files\restart_bridges_and_drone0.sh`、`apply13_restart_bridges_measure.sh`、`restart_drone0_platform.sh`。

### 2.5 重建障碍物

```bash
python3 /mnt/e/AokawaYou/带学资料/papers/uav_swarm/DSH/wsl-files/respawn_obstacles.py
```

删+建 5 个柱子（x/y 读 `world.yaml`）。副作用：gz 泄漏旧插件实例 → 发布者翻倍（回环已断，无害；重启 gz 后干净）。

### 2.6 编译插件

```bash
cd ~/projects/aerostack-swarm-drone-traversal
MAKEFLAGS=-j4 colcon build --packages-select dynamic_moving_objects led_ring_plugin
```

改完**必须重启 gz**（§1.5）。

### 2.7 离线对比/回归（不碰 Gazebo，改避障逻辑后先跑这个）

```bash
bash /mnt/e/AokawaYou/带学资料/papers/uav_swarm/DSH/wsl-files/run_sim_compare.sh 20
```

做的事：① 用**原版未改动的** A* 证明它对柱子失明；② 20 组随机柱子下，旧版决策层 vs 新版 `[plan1]` 的
闭环对比（含一阶跟踪滞后压力测试），打印指标表与逐组净空；③ 出对比图 `scenario4_1_compare.png`；
④ 若新版出现负净空，自动 dump 首次变负前后各若干拍（含"预测净空 vs 实际净空"）。

单项验证预测逻辑：

```bash
cd ~/projects/aerostack-swarm-drone-traversal/challenge_multi_drone && source setup.bash
python3 /mnt/e/…/DSH/wsl-files/check_planner_sim.py 17 18.3   # 参数：seed、时刻
```

判定必须是 `判据 A <= C <= B : PASS`（A=`_simulate` 给的净空，B=按时刻对齐的暴力真值，
C=任意时刻扫掠的暴力真值）。

速度三方对比（旧版 / `_1` / `_2`，同样不需要 Gazebo）：

```bash
bash /mnt/e/AokawaYou/带学资料/papers/uav_swarm/DSH/wsl-files/run_sim_speed.sh 20
```

⚠ 口径：仿真只建模"编队中心按指令速度前进"（世界系积分参考 + 可选一阶滞后 tau），
**测不出控制接口的改进**，因此它比的是"规划层在更高巡航速度、更密决策周期下的表现"。
`_2` 的速度收益主要来自控制层，只能在真机上验证（真实基线见 §1.11 的实测表）。

接口烟测（与在跑的栈隔离，不会影响 Gazebo）：

```bash
python3 /mnt/e/…/DSH/wsl-files/check_stream_api.py drone0     # 内部用 ROS_DOMAIN_ID=99
python3 /mnt/e/…/DSH/wsl-files/check_variant_2.py             # 两个脚本能否 import + 片段就位
```

---

## 3. 已知残留 / 可选项

1. `led_ring_plugin` 在 Harmonic 下编译不过（gz-msgs10 里 `Float_V` 与 `FloatV` 都不存在，待查正确向量消息名）
   → **纯装饰**，LED 颜色不显示，不影响任务。
2. 插件实例泄漏（§2.5）。
3. **旧版**避障（`out_of_date/mapf_scenario_4.py`）的已知缺陷**不再修**：四个结构性缺陷见 §1.10。
   现在只维护 `mapf_scenario_4_6.py` 这一份；旧档在 `out_of_date/` 里留作对照（跑法见 §2.2）。
4. 可调参数集中在 `mapf_scenario_4_6.py` 顶部的 `S4_SPEED` / `S4_SPEED_SAFE` / `S4_SPEED_FAST`：
   `v_max`、`control_hz`、`lead_time`、`safety_margin`、`scales`、`sim_horizon` 等。
   改完先跑 §2.7 的离线对比；真机上先看 `ach/cmd` 比。
   控制方式用 `_MOTION_MODE` 切换（`"stream"` / `"go_to"`）。
5. **`out_of_date/` 里各档的实测结论**：`_1`/`_3`/`_4`/`_5` 只作对照
   （`_3`/`_4` 都在"只看得见 3 根柱子"的前提下跑的，结论不可直接套用）；`_2` 已知会撞柱。
6. `_2`（1.0 m/s）**已知会撞柱**（§1.12），不要再跑；留作对照。
7. 障碍桥已去重为 1 份；tmuxinator 的改动要**下次重启栈**才在 pane 里体现（本轮用 respawn-pane 即时生效）。
8. 柱子的起始位置与 `initial_angle` **每次启动都是随机的**（生成器 `random`），
   所以两次启动的避障难度不一样；要复现某一次的场景得先把 `world.yaml` 与
   `models/dynamic_obstacle_*/` 一起备份（它们是一致的一对）。

---

## 4. 改文件的铁律（本机踩过的坑）

1. **先分清"手写文件"与"生成物"**：`config_sim/world/world.yaml` 与 `config_sim/world/models/*/`（含 5 个柱子 SDF）
   都是 `launch_as2.bash` **每次启动**调用 `utils/generate_world_from_scenario.py` 生成的
   → 要改就改**模板**（`utils/sdf_templates.py`）或**生成器**，改生成物等于埋定时炸弹（§1.6/§1.10 都栽过）。
2. 改前备份（`.bak-dshN`），并尽量写成**幂等脚本**（`DRY_RUN=1` 可预览）。
   备份与被取代的旧文件统一放 `challenge_multi_drone/out_of_date/`，根目录只留**在用的那一份**。
3. 改完要给出**验证命令 + 正常值**，不要只说"修好了"。
4. **用户描述的现象是重要线索**："先是对的、随后变坏" ⇒ spawn 时正确、之后被别的组件覆盖。
5. **离线验证台的输入必须和真机同源**：本轮的验证脚本喂错了柱子速度（用初始角速度而非当前速度），
   让"现象"完全失真，白追了好几轮。**在改被测对象之前，先证明验证台本身是对的**
   （`check_planner_sim.py` 那种"被测实现 vs 暴力枚举"的对照就是为此）。
6. 别用 `strings` 查编译产物里的注释（注释不进二进制）；别按 `top` 的 comm 列判断进程身份
   （`gz sim` 的 CLI 本体是 Ruby 脚本，显示为 `ruby`）。
7. WSL 下 `wsl.exe` 会**剥掉命令里的双引号**，含 `(` `<` `|` 的命令也会被改写 → 复杂命令一律写成 `.sh`/`.py` 文件再执行。
8. **"慢"必须量出来，不要凭日志推断**：航迹 CSV 里就有"实际位移/Δt"。
   一次 `analyze_trace.py` 就能分清是「指令太小」「循环太慢」还是「跟踪跟不上」——
   本轮正是靠它把矛头从"规划/避障"直接转到"控制层"（指令 0.5 m/s 而实际 0.07 m/s）。
9. **被控对象的接口能力要读源码确认**：同一个"飞到这个点"，
   AS2 的 `go_to` 走 action（`start()` 里 `while not future.done(): sleep(0.1)` 阻塞、
   `modify()` 未实现所以每次重发＝取消+重启），
   而 `MotionReferenceHandlerModule` / `PositionMotion` 是**直发话题**（20 Hz、无阻塞）。
   两套接口在本项目里差了 7 倍速度（§1.11）。用之前先看实现，别假设 API 是异步的。
10. **命令要发给"轨迹"而不是"当前位置"**：`target = 实测位置 + v·dt` 这种写法会让跟踪器的
   单步响应反过来限制编队速度（离线能复现：决策周期越小算出来的稳态速度越低）。
   正确做法是给世界系里**连续前进的参考点**（必要时加前视 `v/kp` 抵消 P 控制稳态滞后）。
11. **给"可能阻塞数秒的第三方调用"设降级阈值：一次就降，不要"连续 N 次"**。
   `_2` 用"同一架机累计 3 次 >0.5s"才降级，而单次阻塞可达 6 s、5 架串行 25 s/拍 ——
   无人机在这 25 s 里按旧参考盲飞，直接撞柱（§1.12）。除了一次就降，还要给
   **整个控制周期**设硬时限（`_TICK_SEND_BUDGET`），并在启动时做一次**预检**（线程 + 硬超时）。
12. **关键过程数据要边跑边落盘**：`_2` 两次撞机后 `traces/` 是空的（文件写在循环之后），
   等于丢了全部证据，只能靠读源码反推。诊断类脚本/任务一律**逐拍 flush**。

---

## 5. 改动脚本清单（`DSH\wsl-files\`，WSL 内路径 `/mnt/e/AokawaYou/带学资料/papers/uav_swarm/DSH/wsl-files/`）

| 脚本 | 作用 |
|---|---|
| `apply_fixes.py` / `2` / `3` | 最早一批：RMW 导出、清 /dev/shm、pane 延迟、takeoff/land 签名 |
| `apply_fixes4.py` | 有界+重试的 `arm()/offboard()` |
| `apply_fixes5.py` | **RMW 自对齐守卫 + re-exec** |
| `apply_fixes6.py` | 障碍桥类型 `Pose → PoseStamped`、看门狗心跳、tmux echo 语法 |
| `apply_fixes7.py` / `8.py` | 插件移植 Harmonic（宏 + CMake + 命名空间） |
| `apply_fixes9.py` / `fix10_rebuild_apply.sh` | 插件发布门限改 simTime + `set_pose` 补 z/姿态 |
| `apply_fixes11.py` | `manual()/disarm()` + `reset_all()` 退出复位 + cruise 自检 |
| `apply_fixes12.py` | 柱子抬高烘进模型 SDF |
| `apply_fixes13.py` | 桥方向单向化 + 障碍桥去重（`obstacle_bridge` 参数） |
| **`apply_fixes15.py`** | **柱子几何烘进模板 `sdf_templates.py`**（防启动时重新生成覆盖）+ 生成 `_1` 脚本（幂等，可重跑；`DRY_RUN=1` 预览） |
| **`apply_fixes17.py`** | 重生成 `_1` + 生成 `_2`（`mapf_scenario_4_1_2.py`，并复制一份 `mapf_scenario_4_2.py`） |
| **`apply_fixes18.py`** | 生成 `_3` 安全档（`mapf_scenario_4_3.py`）+ 重生成 `_1`/`_2`（规划器多了 3 个默认关闭的可选参数） |
| **`new_planner_class_1.py`** / **`new_nav_method_1.py`** / **`new_nav_method_2.py`** / **`new_nav_method_3.py`** | 新版避障与三版导航层的源码片段（`apply_fixes15/17/18.py` 把它们拼成 `_1`/`_2`/`_3`；改逻辑改片段再重跑对应脚本） |
| **`sim_drone_level.py`** | **逐机**仿真（逐机 kp 差异 + 速度饱和 + 平台速度响应），净空取 5 架机里最危险那架 —— 用来评估"队形散开"造成的风险 |
| **`sim_compare_avoidance.py`** / **`run_sim_compare.sh`** | 旧版决策层 vs `_1` 离线闭环对比 + 对比图 + 失明证明（不需要 Gazebo） |
| **`sim_compare_speed.py`** / **`run_sim_speed.sh`** | 旧版 / `_1` / `_2` 三方**速度**对比（含 tau 滞后压力测试）+ 对比图 |
| **`check_planner_sim.py`** | 单独验证 `_simulate()` 的净空预测（与暴力枚举对照，PASS/FAIL） |
| **`check_stream_api.py`** | 直发运动参考接口烟测（`ROS_DOMAIN_ID=99` 隔离，不碰在跑的栈） |
| **`check_variant_2.py`** / `check_variant_1.py` | `_1`/`_2` 能否 import、关键片段是否就位 |
| **`analyze_trace.py`** | 解析 `traces/*.csv`：实际速度 / 分档位 / d_goal 里程碑 / 最小净空 —— 判断"时间花在哪" |
| `respawn_obstacles.py` | 重建 5 个柱子 |
| `restart_*` / `enable_ported_plugin.sh` / `rebuild_plugins*.sh` | 单组件重启与插件重建 |
| `check_*.sh` / `test_*.sh` / `dump_hung.sh` / `run_in_ws.sh` | 各种自检（频率、节点、线程栈、环境） |
| `probe_*.sh` / `show_obstacle_params.sh` / `grab_tmux_scrollback.sh` | 只读探查（场景配置、柱子参数、AS2 运动接口、pane 回滚缓冲） |

---

## 6. 目录整理与注释精简（2026-10-08）

调试告一段落，`challenge_multi_drone/` 做过一次收敛，**根目录只留在用的文件**：

| 动作 | 内容 |
|---|---|
| 旧版本脚本归档 | `mapf_scenario_4.py`、`_4_1`、`_4_1_2`、`_4_2`、`_4_3`、`_4_4`、`_4_5` → `out_of_date/` |
| 备份归档 | 全部 `.bak*`（48 个，含子目录里的）→ `out_of_date/`，保持原相对目录结构 |
| 清空航迹 | `traces/*.csv` 23 个全部删除（重跑即重新生成） |
| 注释精简 | `mapf_scenario_4_6.py` 的注释收敛为「作者原注释 + 必要注释」，见下 |

**留下的（根目录）**：`mapf_scenario_4_6.py`（唯一在用的任务脚本）、`mapf_utils.py`（被 `_4_6` import，
**不能动**）、作者原版 `mapf_scenario_1/2/3.py`、`approach1.py`、`track_drone_paths.py`，以及 `launch_as2.bash` /
`setup.bash` / `stop.bash` / `README.md` 等。

**回滚**：`out_of_date/_manifest_1.txt` 是逐条 `原路径|现路径` 清单，`out_of_date/_rollback_1.sh` 可按清单搬回原位。
`traces/*.csv` 已删，无法恢复（不需要恢复，重跑就有）。

### 6.1 `mapf_scenario_4_6.py` 注释精简口径

- **保留**：作者原注释（英文/土耳其文，逐条核对过，**一条未删**）；单位/量纲、安全硬约束、
  "不看会踩坑"的短注释（如「必须凑齐 5 根柱子才起飞」「构造函数后必须登记 streamer」）。
- **删除**：`[planN]`/`[fixN]` 标记、调试过程叙述、历史沿革、实测数字、与本文档重复的内容、`# noqa: BLE001`。
- **docstring**：只改写了 6 处（类/方法级），删掉轮次对照叙述、保留设计要点。
- 结果：2775 → 2660 行；注释行 417 → 321。**代码 token 逐个比对完全一致**（只动了注释与 docstring）。
- 备份：`out_of_date/mapf_scenario_4_6.py.bak-dsh23`。
- 校验方式（可复跑）：`python3 -m py_compile mapf_scenario_4_6.py`；
  去注释后与 `.bak-dsh23` 做 token 级 diff，应只剩 6 处 docstring。

---



*本文件由 DSH 会话生成于 2026-10-07，随 `challenge_multi_drone/` 一起使用；如需更新，请保持「症状/根因/修法/验证」四段式。*

*2026-10-08 整理更新：目录收敛（旧版本脚本与 48 个备份 → `out_of_date/`、清空 `traces/`）+ `mapf_scenario_4_6.py` 注释精简，见 §6。*

*2026-10-08 再更新：补 §1.20~§1.23（第五轮：5/5 根柱子真根因与修法、`safety_margin` 调大的负结果、柱子活动区收缩、plan6 首个 5/5 数据点），当前推荐档 = `mapf_scenario_4_6.py`（柱子活动区收缩需重启栈才生效）。*

*2026-10-08 更新：补 §1.13~§1.19（第四轮：[plan4] 路径跟随 + 直发参考真正生效 + 柱子凑齐门槛 + RTF 实测 + 每拍耗时账），当前推荐档改为 `mapf_scenario_4_4.py`。*
