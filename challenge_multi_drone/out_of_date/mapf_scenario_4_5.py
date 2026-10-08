#!/usr/bin/env python3

# [fix5] RMW self-alignment ------------------------------------------------------
# AS2 栈全部运行在 rmw_cyclonedds_cpp 上。若本脚本在没有 export
# RMW_IMPLEMENTATION 的 shell 里启动，就会退回默认的 rmw_fastrtps_cpp，而两套 RMW
# 在 RTPS 层只有"部分互通"：服务会被 wait_for_service 认成可用，但请求/响应送不到
# （表现：[fix] 日志里 set_arming_state / set_offboard_mode 全部 TIMEOUT、却没有
# 一条 NOT AVAILABLE），并且本进程的节点在别人的 ROS 图里永远不出现。
# 这里在 import rclpy 之前强制对齐，必要时原样重启自己一次。
import os as _os
import sys as _sys

_RMW_WANTED = "rmw_cyclonedds_cpp"
_rmw_old = _os.environ.get("RMW_IMPLEMENTATION")
if _rmw_old != _RMW_WANTED:
    _os.environ["RMW_IMPLEMENTATION"] = _RMW_WANTED
    if not _os.environ.get("_DSH_RMW_REEXEC"):
        _os.environ["_DSH_RMW_REEXEC"] = "1"
        print(f"[fix5] RMW_IMPLEMENTATION={_rmw_old or 'unset'} -> re-exec with {_RMW_WANTED}",
              flush=True)
        _os.execv(_sys.executable, [_sys.executable] + _sys.argv)
# -------------------------------------------------------------------------------

import argparse
import sys
import time
import random
import threading
import math
from math import radians, cos, sin, atan2
from typing import List
import copy

import rclpy
from rclpy.executors import MultiThreadedExecutor
from as2_msgs.msg import YawMode, BehaviorStatus
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA  # Add this import for LED control
import yaml  # Senaryo dosyasını okumak için
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D
from mapf_utils import *


# =====================================================================================
# [plan1] Scenario 4 避障决策层（2026-10-07 新增）
#   * 纯几何：只依赖 math / heapq，不 import 任何 ROS 包，
#     因此可以用 challenge_multi_drone/sim_compare_avoidance.py 离线跑，不需要 Gazebo。
#   * 为什么重写：见 navigate_through_dynamic_obstacles() 的 docstring，
#     以及仓库 README 的 1.10 节（旧版四个结构性缺陷）。
# =====================================================================================
import heapq


def _point_seg_dist2(px, py, a, b):
    """点 (px,py) 到线段 a-b 的距离平方。"""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return (px - ax) ** 2 + (py - ay) ** 2
    t = ((px - ax) * dx + (py - ay) * dy) / l2
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    cx, cy = ax + t * dx, ay + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


class ArenaAvoidancePlanner:
    """[plan1] Scenario 4 的避障决策层。

    与旧版 mapf_scenario_4.py 的三处根本差异
    ----------------------------------------
    1) **柱子是竖直柱体，不是可以"从下面/上面绕过去"的球。**
       旧版 A* 把柱子当半径 0.55m 的球、球心在 z≈0；编队中心在 z=3.5，
       于是 `is_collision_free()` 里的 3D 距离恒 > 0.55m，
       `has_line_of_sight()` 永远返回 True → A* 永远只输出一条直线，
       **规划器对柱子完全失明**。本类只判 2D 平面距离，
       并且把**编队自身外接半径 R_f(scale)** 正式计入安全距离
       （`safe_axis_dist`）——保护的是最外侧那架机，而不是编队中心。
       旧版只护中心：中心报"最近 0.83m 安全"时，外侧机到柱面可能只剩 0.2m，
       看起来就是"快撞上了还不躲"。

    2) **只在该赛场内规划，高度带钉在柱顶之下。**
       旧版 `planning_bounds` 是 min/max±10m（y 能到 22.5），
       而且类里那句 `self.boundaries = [-10, 10, -10, 10, 0, 5]` 从来没被用过。
       本类的 bounds 直接来自 scenario 的 stage_center/stage_size
       （与柱子 SDF 的 boundary_min/max 完全一致：x[-5,5], y[1,11]），
       横向（x）严格不出赛场；纵向只放宽到能容纳起飞点/终点。
       z 恒定在巡航高度，不允许用"爬高"绕过柱子。

    3) **采样预测 + 硬禁后退。**
       旧版是径向斥力：柱子正面挡路时斥力方向与目标方向近乎反向，
       合力把编队整体往后推（这正是实测现象）；
       而那段"保证前进"的 `min_forward_component` 代码是**死代码**
       ——它只在合力模长 < 0.25 时才执行，可斥力上限是 8.0、5 根柱子叠加，永不触发。
       本类在"目标方向 ±`max_angle`"内采样若干速度、各自向前仿真 `sim_horizon` 秒，
       按「推进量 + 贴合规划路径 − 危险度」打分取最优；
       沿目标方向的速度分量恒 ≥ v_max·cos(max_angle) = 0.25 m/s，
       **结构上不可能后退**。只有"预测到所有候选都会撞"的极少数情况才允许
       纯侧移/停车，仍不允许后退；真的已经贴上了（净空 < 硬下限-0.15m）
       才允许 ≤`emergency_backoff` 的退避，并且会打 `[plan1][ESC]` 日志。

    离线单测：`python3 sim_compare_avoidance.py`（不需要 ROS/Gazebo）
    """

    def __init__(self,
                 bounds,
                 r_pillar,
                 formation_base_radius,
                 obstacle_bounds=None,
                 obstacle_height=5.0,
                 cruise_alt=3.5,
                 obstacle_z_is_base=True,
                 safety_margin=0.30,
                 grid_res=0.25,
                 grid_cushion=0.15,
                 predict_horizon=2.0,
                 line_weight=0.25,
                 near_weight=0.8,
                 v_max=0.5,
                 v_min=0.15,
                 sim_horizon=2.0,
                 sim_dt=0.25,
                 eta_slack=0.70,
                 alert_pad=0.7,
                 w_progress=1.0,
                 w_align=0.35,
                 w_risk=0.9,
                 speed_fracs=(1.0, 0.65, 0.4, 0.0),
                 max_angle=60.0,
                 tight_margin=0.12,
                 scale_candidates=(1.0, 0.85, 0.7, 0.6, 0.5),
                 pure_pursuit_lookahead=0.9,
                 emergency_backoff=0.25,
                 verbose=True):
        self.xmin, self.xmax, self.ymin, self.ymax = [float(v) for v in bounds]
        self.r_pillar = float(r_pillar)
        self.formation_base_radius = float(formation_base_radius)
        if obstacle_bounds is None:
            obstacle_bounds = (self.xmin, self.xmax, self.ymin, self.ymax)
        self.obstacle_bounds = [float(v) for v in obstacle_bounds]
        self.obstacle_height = float(obstacle_height)
        self.cruise_alt = float(cruise_alt)
        self.obstacle_z_is_base = bool(obstacle_z_is_base)
        self.safety_margin = float(safety_margin)
        self.res = float(grid_res)
        self.grid_cushion = float(grid_cushion)
        self.predict_horizon = float(predict_horizon)
        self.line_weight = float(line_weight)
        self.near_weight = float(near_weight)
        self.v_max = float(v_max)
        # 规划 ETA 用的估计速度：故意取得比 v_max 小（柱子预测位置偏后 -> 更保守）
        self.v_plan = 0.7 * self.v_max
        # 反应层"双向对冲"用的滞后系数：实际到达时刻最多按 t/eta_slack 估（越小越保守）
        self.eta_slack = float(eta_slack)
        self._last_cum = []
        # [迟滞] 上一轮路径在"起点->终点"直线哪一侧（符号）；用于抑制重规划抖动
        self.side_prev = 0.0
        self.w_side = 0.6
        # [迟滞] 上一拍速度；用于抑制决策抖动（抖动会让预测保证失效）
        self._v_prev = (0.0, 0.0)
        self.w_smooth = 0.30
        self.v_min = float(v_min)
        self.sim_horizon = float(sim_horizon)
        self.sim_dt = float(sim_dt)
        self.alert_pad = float(alert_pad)
        self.w_progress = float(w_progress)
        self.w_align = float(w_align)
        self.w_risk = float(w_risk)
        self.max_angle = float(max_angle)
        self.speed_fracs = tuple(float(s) for s in speed_fracs)
        self.tight_margin = float(tight_margin)
        self.scale_candidates = tuple(float(s) for s in scale_candidates)
        self.pure_pursuit_lookahead = float(pure_pursuit_lookahead)
        self.emergency_backoff = float(emergency_backoff)
        self.verbose = bool(verbose)

        # 采样角度：以目标方向为 0，正负对称。角度的上限就是"禁后退"的实现方式。
        self.angles = tuple(float(a) for a in
                            (-max_angle, -0.75 * max_angle, -0.5 * max_angle,
                             -max_angle / 3.0, -max_angle / 6.0, 0.0,
                             max_angle / 6.0, max_angle / 3.0,
                             0.5 * max_angle, 0.75 * max_angle, max_angle))
        # 规划不出路（走廊被瞬时封死）时放宽到 ±75°/±90°：允许纯侧移让行，
        # 但任何一个档位都不允许"沿目标方向分量为负"，所以仍然不会后退。
        self.angles_wide = tuple(sorted(set(self.angles + (75.0, 90.0, -75.0, -90.0))))
        # 紧急逃逸用的全向方向（绝对值，度）：允许往任意方向让，含侧移与停车
        self.escape_angles = tuple(float(a) for a in
                                   (-180, -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150))

        self.nx = int(math.floor((self.xmax - self.xmin) / self.res)) + 1
        self.ny = int(math.floor((self.ymax - self.ymin) / self.res)) + 1

        self.obstacles = []          # [{'id','p':(x,y),'v':(vx,vy)}, ...]
        self._swept = []             # 预测扫掠点（每次 plan()/step() 前刷新）
        # [plan4] 每拍缓存 `(柱子id, t) -> (x,y)`：t 在几十个候选之间完全重复，
        # 缓存后 _simulate 的成本从 ~1.8 万次预测降到约 200 次。随 update_obstacles 作废。
        self._pred_cache = {}
        # [plan4] 距离场缓存：`_dist_now_grid()` 与 scale 无关，但 plan() 每个候选缩放
        # 都会重算一遍（4132 格 × 5 柱，实测 ~20ms/次 × 最多 5 个缩放 = 100ms，
        # 这就是真机控制环只有 9.3Hz 的主因）。改成每拍只算一次。
        self._dist_grid_cache = None

        self.stats = {
            "plan_calls": 0,
            "plan_ok": 0,
            "plan_fail": 0,
            "emergency": 0,
            "backoff": 0,
            "brake": 0,
            "scale_hist": {},
        }

    # ------------------------------------------------------------------ 几何/工具
    def clamp_xy(self, x, y):
        return (min(self.xmax, max(self.xmin, x)),
                min(self.ymax, max(self.ymin, y)))

    def in_bounds(self, x, y, eps=0.0):
        return (self.xmin - eps <= x <= self.xmax + eps and
                self.ymin - eps <= y <= self.ymax + eps)

    def formation_radius(self, scale):
        return self.formation_base_radius * scale

    def safe_axis_dist(self, scale):
        """编队中心到柱子轴线的安全距离：柱半径 + 编队外接半径 + 余量。"""
        return self.r_pillar + self.formation_radius(scale) + self.safety_margin

    def hard_axis_dist(self, scale):
        """硬下限（**轴线距离**）：再近就真的可能撞上（不再留余量）。"""
        return self.r_pillar + self.formation_radius(scale)

    def hard_surface_clearance(self, scale):
        """硬下限（**柱面净空**，与 surface_clearance() 同单位）。

        编队中心到柱面的净空必须 >= 编队外接半径 + 0.05m 的离散化余量，
        否则最外侧那架机就会碰到柱面。
        注意：`surface_clearance()` 返回的是柱面净空，而 `hard_axis_dist()` 是轴线距离，
        两者差一个 r_pillar，**不能混着比**（本类第一版就踩了这个坑，
        导致紧急分支被误触发 392 次）。
        """
        return self.formation_radius(scale) + 0.05

    def safe_surface_clearance(self, scale):
        """安全下限（柱面净空）：最外侧那架机还要留 safety_margin。"""
        return self.formation_radius(scale) + self.safety_margin

    def alert_radius(self, scale):
        return self.safe_axis_dist(scale) + self.alert_pad

    def _index(self, x, y):
        ix = int(round((x - self.xmin) / self.res))
        iy = int(round((y - self.ymin) / self.res))
        ix = max(0, min(self.nx - 1, ix))
        iy = max(0, min(self.ny - 1, iy))
        return ix, iy

    def _xy(self, ix, iy):
        return (self.xmin + ix * self.res, self.ymin + iy * self.res)

    # ------------------------------------------------------------------ 障碍物
    def update_obstacles(self, positions, velocities=None, speed_hint=None):
        """接收订阅到的柱子位姿，筛掉与巡航高度不相交的。

        positions: {id: [x, y, z]}
        velocities: {id: [vx, vy, vz]}  （可选）
        speed_hint: 已知的名义速度（scenario 的 obstacle_velocity）；
                    给了就把测得速度**重新归一化**到该速度，抑制差分噪声。
        """
        velocities = velocities or {}
        kept = []
        for oid, p in positions.items():
            px, py, pz = float(p[0]), float(p[1]), float(p[2])
            if self.obstacle_z_is_base:
                z0, z1 = pz, pz + self.obstacle_height
            else:
                z0, z1 = pz - self.obstacle_height / 2.0, pz + self.obstacle_height / 2.0
            if not (z0 - 1e-6 <= self.cruise_alt <= z1 + 1e-6):
                continue                      # 高度不相交 -> 不构成障碍
            v = velocities.get(oid, [0.0, 0.0, 0.0])
            vx, vy = float(v[0]), float(v[1])
            if speed_hint:
                sp = math.hypot(vx, vy)
                if sp > 0.15 * float(speed_hint):
                    k = float(speed_hint) / sp
                    vx, vy = vx * k, vy * k
                elif sp > 1e-6:
                    vx, vy = 0.0, 0.0         # 太慢，认为还没测准
            # [fix] 刚撞完墙的那一小段时间，差分出来的速度可能还指着墙外（指数平滑的滞后），
            # 而真实速度不可能朝墙外 -> 就地翻转该分量。否则预测会把柱子送向墙里再弹回来，
            # 位置误差能到 1m 量级。
            bx0, bx1, by0, by1 = self.obstacle_bounds
            if px > bx1 - 0.30 and vx > 0.0:
                vx = -vx
            elif px < bx0 + 0.30 and vx < 0.0:
                vx = -vx
            if py > by1 - 0.30 and vy > 0.0:
                vy = -vy
            elif py < by0 + 0.30 and vy < 0.0:
                vy = -vy
            kept.append({"id": oid, "p": (px, py), "v": (vx, vy)})
        self.obstacles = kept
        self._pred_cache = {}                     # [plan4] 柱子状态变了 -> 缓存作废
        self._dist_grid_cache = None              # [plan4] 距离场也作废
        return kept

    @staticmethod
    def _fold(p0, v, lo, hi, t):
        """一维"撞墙反弹"的解析位置：与 gz 插件（越界就翻速度符号并 clamp）等价。

        插件每步做 new = p + v*dt，越界则 v = -v 且 clamp 到边界；
        其轨迹正好是周期为 2*(hi-lo) 的三角波，所以可以直接精确预测整条轨迹，
        不用像旧版那样只做 1.0s 的线性外推。
        """
        span = hi - lo
        if span <= 1e-9:
            return lo
        s = (p0 + v * t - lo) % (2.0 * span)
        return lo + (s if s <= span else 2.0 * span - s)

    def _pred_xy(self, obs, t):
        # [plan4] 加一层每拍缓存。key 用 (柱子id, t)：_simulate 里 t 只有
        # k*sim_dt 与 k*sim_dt/eta_slack 两组取值，几十个候选共享同一批 t，
        # 命中率接近 100%；命中时结果与直接计算**完全一致**。
        key = (obs["id"], t)
        hit = self._pred_cache.get(key)
        if hit is not None:
            return hit
        bx0, bx1, by0, by1 = self.obstacle_bounds
        val = (self._fold(obs["p"][0], obs["v"][0], bx0, bx1, t),
               self._fold(obs["p"][1], obs["v"][1], by0, by1, t))
        self._pred_cache[key] = val
        return val

    def _refresh_swept(self):
        pts = []
        n = max(2, int(round(self.predict_horizon / 0.25)) + 1)
        for obs in self.obstacles:
            for k in range(n):
                t = k * self.predict_horizon / (n - 1)
                pts.append(self._pred_xy(obs, t))
        self._swept = pts
        return pts

    def surface_clearance(self, x, y, t=0.0):
        """编队中心到"最近柱面"的距离（米，可负）。没有柱子时返回 +inf。"""
        best = float("inf")
        for obs in self.obstacles:
            ox, oy = self._pred_xy(obs, t)
            d = math.hypot(x - ox, y - oy) - self.r_pillar
            if d < best:
                best = d
        return best

    def min_drone_clearance(self, drone_positions, t=0.0):
        """实际最危险的那架机到柱面的距离——这才是"有没有快撞上"的真指标。"""
        best = float("inf")
        for p in drone_positions:
            for obs in self.obstacles:
                ox, oy = self._pred_xy(obs, t)
                d = math.hypot(float(p[0]) - ox, float(p[1]) - oy) - self.r_pillar
                if d < best:
                    best = d
        return best

    # ------------------------------------------------------------------ A* 规划
    # 【关键设计】按"到达时刻的柱子位置"判定，而不是"整个预测时段的扫掠区"。
    # 本类第一版用扫掠管（任何时刻扫过就算堵），5 根柱子在 10m 走廊里很容易
    # 把整条走廊**瞬时**封死 -> A* 全线失败 -> 退化成纯局部反应 -> 被两根柱子夹住穿模。
    # 正确做法：格子 (x,y) 只要在"我预计到达它的时刻 eta"柱子不在那儿，就是通的
    #（柱子扫过去之后那个格子自然恢复）。于是编队会**从柱子后面绕过去**，
    # 而不是原地等待或硬挤。严格说是把时间维并进了 A* 的扩展代价 g（eta = g / v_plan）。
    def _dist_now_grid(self):
        """当前时刻到各柱子轴线的距离场：只用于代价里的"贴柱惩罚"，不是安全判据。

        [plan4] 结果只依赖柱子当前位置，与 scale/起点/终点都无关，
        但原来 plan() 每调用一次（best_scale 会对 5 个缩放各调一次）就重算一遍。
        缓存后每拍只算一次，省下约 80% 的重规划时间。
        """
        if self._dist_grid_cache is not None:
            return self._dist_grid_cache
        dist = [[9.9] * self.nx for _ in range(self.ny)]
        for iy in range(self.ny):
            y = self.ymin + iy * self.res
            row = dist[iy]
            for ix in range(self.nx):
                x = self.xmin + ix * self.res
                dmin = 9.9
                for obs in self.obstacles:
                    dx = x - obs["p"][0]
                    dy = y - obs["p"][1]
                    d = math.sqrt(dx * dx + dy * dy)
                    if d < dmin:
                        dmin = d
                row[ix] = dmin
        self._dist_grid_cache = dist            # [plan4] 每拍只算一次
        return dist

    def _blocked_at_eta(self, x, y, eta, thr2):
        """位置 (x,y) 在"到达时刻 eta"是否与柱子冲突。"""
        for obs in self.obstacles:
            ox, oy = self._pred_xy(obs, eta)
            dx = x - ox
            dy = y - oy
            if dx * dx + dy * dy < thr2:
                return True
        return False

    def _nearest_free_at(self, gx, gy, eta, thr2, rmax=14):
        for r in range(0, rmax + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    ix, iy = gx + dx, gy + dy
                    if not (0 <= ix < self.nx and 0 <= iy < self.ny):
                        continue
                    px, py = self._xy(ix, iy)
                    if not self._blocked_at_eta(px, py, eta, thr2):
                        return ix, iy
        return None, None

    def _astar(self, dist_now, start_xy, goal_xy, scale):
        thr = self.safe_axis_dist(scale) + self.grid_cushion
        thr2 = thr * thr
        sx, sy = self._index(*start_xy)
        gx, gy = self._index(*goal_xy)
        # 目标的到达时刻按直线距离估（保守：偏低 -> 柱子预测更靠后）
        eta_goal = math.hypot(goal_xy[0] - start_xy[0], goal_xy[1] - start_xy[1]) / self.v_plan
        if self._blocked_at_eta(*self._xy(gx, gy), eta_goal, thr2):
            gx, gy = self._nearest_free_at(gx, gy, eta_goal, thr2)
            if gx is None:
                return None
        start, goalidx = (sx, sy), (gx, gy)
        if start == goalidx:
            return [self._xy(*start)]

        moves = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.41421), (1, -1, 1.41421),
                 (-1, 1, 1.41421), (-1, -1, 1.41421))

        ax, ay = start_xy
        segdx, segdy = goal_xy[0] - ax, goal_xy[1] - ay
        segl2 = segdx * segdx + segdy * segdy

        heap = [(0.0, start)]
        gscore = {start: 0.0}
        came = {}
        closed = set()
        guard = 0
        max_pop = 10 * self.nx * self.ny
        while heap and guard < max_pop:
            guard += 1
            _, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            closed.add(cur)
            if cur == goalidx:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                path.reverse()
                return [self._xy(ix, iy) for (ix, iy) in path]
            cx, cy = cur
            cg = gscore[cur]
            for dx, dy, step in moves:
                nx_, ny_ = cx + dx, cy + dy
                if nx_ < 0 or ny_ < 0 or nx_ >= self.nx or ny_ >= self.ny:
                    continue
                nb = (nx_, ny_)
                if nb in closed:
                    continue
                px, py = self.xmin + nx_ * self.res, self.ymin + ny_ * self.res
                near = max(0.0, 1.0 - dist_now[ny_][nx_] / max(0.05, thr * 2.0))
                if segl2 > 1e-9:
                    tt = ((px - ax) * segdx + (py - ay) * segdy) / segl2
                    tt = 0.0 if tt < 0.0 else (1.0 if tt > 1.0 else tt)
                    lx, ly = ax + tt * segdx, ay + tt * segdy
                    # 有向横向偏移：符号表示走直线的哪一侧
                    off = ((px - lx) * (-segdy) + (py - ly) * segdx) / max(1e-9, math.sqrt(segl2))
                else:
                    off = 0.0
                # [迟滞] 惩罚"换到上一轮路径的另一侧"：防止每次重规划都左右跳，
                # 抖动会让 2 秒预测的保证失效（实测抖动段预测 0.39m 而实际只剩 0.04m）。
                side_pen = 0.0
                if self.side_prev != 0.0 and self.side_prev * off < 0.0:
                    side_pen = self.w_side * abs(off)
                step_cost = step * self.res * (1.0 + self.near_weight * near +
                                               self.line_weight * abs(off) + side_pen)
                ng = cg + step_cost
                # 时间维：到达该格的预计时刻 = 沿路径累计代价 / 规划速度
                if self._blocked_at_eta(px, py, ng / self.v_plan, thr2):
                    continue
                if ng < gscore.get(nb, 1e18):
                    gscore[nb] = ng
                    came[nb] = cur
                    h = math.hypot(px - goal_xy[0], py - goal_xy[1])
                    heapq.heappush(heap, (ng + h, nb))
        return None

    def _seg_blocked(self, a, b, scale, s0=0.0):
        """线段 a-b 是否在"到达时刻"与柱子冲突。s0 = 从起点沿路径走到 a 的距离。"""
        thr2 = (self.safe_axis_dist(scale) + self.grid_cushion) ** 2
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(L / 0.1))
        for k in range(n + 1):
            frac = k / n
            px = a[0] + (b[0] - a[0]) * frac
            py = a[1] + (b[1] - a[1]) * frac
            if self._blocked_at_eta(px, py, (s0 + L * frac) / self.v_plan, thr2):
                return True
        return False

    def _shortcut(self, pts, cum, scale):
        """拉直：贪心地用最远的可见点连直线，得到少量长直段（便于纯跟踪）。"""
        if len(pts) <= 2:
            return list(pts)
        out = [pts[0]]
        out_cum = [cum[0]]
        i, n = 0, len(pts)
        while i < n - 1:
            j = n - 1
            while j > i + 1 and self._seg_blocked(pts[i], pts[j], scale, cum[i]):
                j -= 1
            out.append(pts[j])
            out_cum.append(cum[j])
            i = j
        self._last_cum = out_cum
        return out

    def plan(self, start, goal, scale):
        """在赛场内规划 (x,y) 路径；不可行返回 None。"""
        self.stats["plan_calls"] += 1
        if not self.obstacles:
            return [tuple(start), tuple(goal)]        # 没有柱子：直线
        dist_now = self._dist_now_grid()
        raw = self._astar(dist_now, start, goal, scale)
        if raw is None:
            self.stats["plan_fail"] += 1
            return None
        pts = [tuple(start)]
        for p in raw:
            if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-6:
                pts.append(tuple(p))
        if math.hypot(goal[0] - pts[-1][0], goal[1] - pts[-1][1]) > 1e-6:
            pts.append((float(goal[0]), float(goal[1])))
        cum = [0.0]
        for k in range(1, len(pts)):
            cum.append(cum[-1] + math.hypot(pts[k][0] - pts[k - 1][0],
                                            pts[k][1] - pts[k - 1][1]))
        path = self._shortcut(pts, cum, scale)
        # 记录这一轮路径偏向直线的哪一侧（迟滞用）
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        segl = math.hypot(dx, dy)
        if segl > 1e-6 and len(path) > 1:
            offs = []
            for (qx, qy) in path:
                tt = ((qx - start[0]) * dx + (qy - start[1]) * dy) / (segl * segl)
                tt = max(0.0, min(1.0, tt))
                lx, ly = start[0] + tt * dx, start[1] + tt * dy
                offs.append(((qx - lx) * (-dy) + (qy - ly) * dx) / segl)
            mean_off = sum(offs) / len(offs)
            self.side_prev = mean_off if abs(mean_off) > 0.15 else 0.0
        self.stats["plan_ok"] += 1
        return path

    def best_scale(self, start, goal, current_scale=1.0):
        """在候选缩放里挑"能过且最大"的那个——这就是"变个形状就过去"的正式实现。

        [plan3] 并列时优先**更小**的缩放（原来 tie-break 取 -s = 更大的）：
        小一点的编队外接半径更小、净空更大，速度高时这一点很重要。
        """
        # [plan4] 并列时优先**更大**的编队（原来是更小）。用户要的是"绕开柱子"，
        # 不是"把编队缩到最小、从缝里挤过去"；缩小仍保留为最后手段。
        order = sorted(self.scale_candidates,
                       key=lambda s: (abs(s - current_scale), -s))
        for s in order:
            p = self.plan(start, goal, s)
            if p:
                self.stats["scale_hist"][s] = self.stats["scale_hist"].get(s, 0) + 1
                return s, p
        return None, None

    # ------------------------------------------------------------------ 反应层
    def _pursuit_point(self, center, path):
        if not path:
            return None
        for p in path[1:]:
            if math.hypot(p[0] - center[0], p[1] - center[1]) >= self.pure_pursuit_lookahead:
                return p
        return path[-1]

    def _simulate(self, center, vx, vy, scale):
        """返回 (是否不出界, 最小柱面净空, 终点)。

        净空按**双向对冲**计算：对轨迹上的每个点，同时查"标称到达时刻 t"与
        "滞后到达时刻 t / eta_slack (eta_slack=0.7)"两处的柱子位置，取更小者。
          * 柱子迎面而来：编队实际比指令慢 -> 到达更晚 -> 柱子更近，用 t/0.7 才安全；
          * 柱子正在远离：到达越早越危险 -> 用标称 t 才安全。
        只用其一都会偏（本类试过两个极端：只用 t 时实测偏乐观 0.3m、每拍都判安全；
        只用"任意时刻扫掠"又过度悲观，预测 -0.34m 而真实还有 0.53m，
        导致"取最大净空"的逃逸判断失去分辨力）。
        """
        n = max(1, int(round(self.sim_horizon / self.sim_dt)))
        x, y = center
        min_clear = float("inf")
        for k in range(1, n + 1):
            t_nom = k * self.sim_dt
            x += vx * self.sim_dt
            y += vy * self.sim_dt
            if not self.in_bounds(x, y):
                return False, float("-inf"), (x, y)
            d1 = self.surface_clearance(x, y, t_nom)
            d2 = self.surface_clearance(x, y, t_nom / self.eta_slack)
            d = d1 if d1 < d2 else d2
            if d < min_clear:
                min_clear = d
        return True, min_clear, (x, y)

    def step(self, center, goal, path, scale, drone_positions=None,
             extra_radius=0.0, extra_margin=0.0):
        """返回 (vx, vy, info)。

        [plan3] 新增三个可选入参（默认 0/None，因此 _1/_2 的行为完全不变）：
          * `drone_positions`：实测的 5 架机位置。给了就用**实测队形外接半径**
            `max(名义 R_f, max_i|p_i-center|)`，而不是假定的 0.5*scale
            —— 之前"计划护中心、实际外侧机已经更贴柱"就是这么漏掉的。
          * `extra_radius`：额外安全半径（在线跟踪误差预算）。
          * `extra_margin`：额外安全余量（可随速度放大）。

        选择规则是**字典序**的 —— 安全是硬约束，不是可以拿"前进量"去换的软代价。
        （本类第一版把危险度做成软代价 `-w_risk*risk^2`，前进量的权重能压过它，
        在 20 组随机柱子里 8 组出现穿模；第二版改成"约束档"后仍有一个致命细节：
        降级档里继续按前进量打分，于是编队贴着硬下限"擦着柱子走"。这条教训写在 README 1.10 节。）

        净空一律换算成 **outer = 最外侧那架机到柱面的净空** = 中心柱面净空 - 队形外接半径。

          ① outer >= safety_margin+extra_margin      -> 正常避障/巡航，在档内取分最高
          ② outer >= tight_margin+extra_margin       -> AVOID-TIGHT，仍在档内取分最高
          ③ ①②都空                                  -> **不再看前进量，取 outer 最大者**；
              outer < 0（已经贴上）才允许 <= emergency_backoff 的退避，并打日志

        候选速度的偏角在"目标方向 ±max_angle"内（无路径时放宽到 ±90°）、
        速度档含 0（停车等待，此时仍按障碍物继续逼近来算净空），
        所以沿目标方向分量恒 >= 0 —— **永不后退**（唯一例外是 ③ 的 ESC-BACKOFF）。
        """
        if scale is None:                     # best_scale() 全不可行时不要传 None 进来
            scale = min(self.scale_candidates)
        gx, gy = goal[0] - center[0], goal[1] - center[1]
        gd = math.hypot(gx, gy)
        if gd < 1e-6:
            return 0.0, 0.0, {"mode": "ARRIVED", "clear": float("inf"),
                              "angle": 0.0, "pred": float("inf")}
        gu = (gx / gd, gy / gd)

        pursuit = self._pursuit_point(center, path)
        if pursuit is None:
            pursuit = goal
        px, py = pursuit[0] - center[0], pursuit[1] - center[1]
        pd = math.hypot(px, py)
        pu = (px / pd, py / pd) if pd > 1e-6 else gu
        # [plan4] 方向锥的锚点：**有规划路径时锚在路径切向上**。
        # plan3 把它锚在"直奔终点"方向，于是 A* 绕出来的路径只是打分里的一个软项
        # （w_align=0.35），被"前进量"（w_progress=1.0，且按到终点的投影算）压住，
        # 候选永远偏向锥内最靠终点的那个 -> 计划好的绕行不执行、看起来就是直飞。
        if path:
            base_ang = math.atan2(pu[1], pu[0])
        else:
            base_ang = math.atan2(gu[1], gu[0])

        # [plan3] 队形外接半径：取名义值与**实测值**的较大者，再加在线误差预算
        R_f = self.formation_radius(scale)
        reff = R_f
        if drone_positions:
            try:
                meas = max(math.hypot(float(q[0]) - center[0], float(q[1]) - center[1])
                           for q in drone_positions)
                if meas > reff:
                    reff = meas
            except Exception:                 # noqa: BLE001
                pass
        R_f = reff + float(extra_radius)
        safe_outer = self.safety_margin + float(extra_margin)
        tight_outer = self.tight_margin + float(extra_margin)
        alert_outer = safe_outer + self.alert_pad
        thr_norm = max(0.1, R_f + alert_outer)
        horizon_gain = self.v_max * self.sim_horizon
        # 有规划路径时按 ±max_angle 采样；规划不出路（走廊被瞬时封死）时放宽到 ±90°，
        # 允许纯侧移让行/停车等待，但**任何档位都不允许后退**。
        angles = self.angles if path else self.angles_wide

        # ---- 候选：速度档 x 偏角档 ----
        # 净空统一换算成"最外侧那架机到柱面的净空" outer = 中心净空 - 编队半径。
        cands = []
        for v_frac in self.speed_fracs:
            for a in angles:
                ang = base_ang + math.radians(a)
                vx = self.v_max * v_frac * math.cos(ang)
                vy = self.v_max * v_frac * math.sin(ang)
                # [plan4] 硬约束：任何候选都不许"朝终点反方向"。
                # plan3 靠"锥只开在目标方向 ±max_angle 内"来保证不后退；锥改锚路径后
                # 必须显式加这一条，否则路径切向接近横向时可能采到往回飞的方向。
                if vx * gu[0] + vy * gu[1] < 0.0:
                    continue
                ok, cmin, end = self._simulate(center, vx, vy, scale)
                if not ok:
                    continue                      # 出赛场：直接丢弃（硬约束）
                outer = cmin - R_f
                # [plan4] 前进量：有路径时按**沿路径切向**推进算，没有路径才算到终点的投影。
                # 否则"直奔终点"这一项会把候选拉到锥边缘（数学上：锥边缘朝终点一侧的
                # prog 增益 ~0.7 远大于 align 惩罚 ~0.25），计划好的绕行又被吃掉。
                if path:
                    prog = ((end[0] - center[0]) * pu[0]
                            + (end[1] - center[1]) * pu[1])
                else:
                    prog = ((end[0] - center[0]) * gu[0]
                            + (end[1] - center[1]) * gu[1])
                align = (vx * pu[0] + vy * pu[1]) / self.v_max
                risk = max(0.0, 1.0 - (R_f + outer) / thr_norm)
                dv = math.hypot(vx - self._v_prev[0], vy - self._v_prev[1])
                score = (self.w_progress * prog
                         + self.w_align * align * horizon_gain
                         - self.w_risk * horizon_gain * risk * risk
                         - self.w_smooth * dv * self.sim_horizon)
                cands.append({"vx": vx, "vy": vy, "clear": cmin, "outer": outer,
                              "angle": a, "speed_frac": v_frac, "score": score})

        if not cands:
            # 连"停在原地都算出界"（已经贴边）-> 朝目标微速，保证有输出
            self.stats["brake"] += 1
            here = self.surface_clearance(center[0], center[1])
            return (self.v_min * gu[0], self.v_min * gu[1],
                    {"mode": "FALLBACK", "clear": here, "outer": here - R_f,
                     "angle": 0.0})

        lvl1 = [c for c in cands if c["outer"] >= safe_outer]
        lvl2 = [c for c in cands if c["outer"] >= tight_outer]
        if lvl1:
            best = max(lvl1, key=lambda c: c["score"])
            best["mode"] = "TRACK" if best["outer"] >= alert_outer else "AVOID"
        elif lvl2:
            best = max(lvl2, key=lambda c: c["score"])
            best["mode"] = "AVOID-TIGHT"
        else:
            # ③ 安全余量和 0.12m 余量都保不住 —— **不再看前进量，改为最大化净空**。
            #    这一条是第一版最致命的缺陷：原来在降级档里仍然按"推进量+贴合路径"打分，
            #    于是编队会贴着硬下限"擦着柱子走"，实测 8/20 组穿模。
            #    逃逸方向从"只能正后方退"放宽为**全向**（含停车），
            #    同等净空优先"不后退、侧移" —— 否则会退化成一直被柱子推着后退。
            self.stats["emergency"] += 1
            esc = None
            for v_frac in (0.5, 0.25, 0.0):
                for a in self.escape_angles:
                    ang = math.radians(a)
                    vx = self.v_max * v_frac * math.cos(ang)
                    vy = self.v_max * v_frac * math.sin(ang)
                    ok, cmin, end = self._simulate(center, vx, vy, scale)
                    if not ok:
                        continue
                    outer = cmin - R_f
                    proj = vx * gu[0] + vy * gu[1]
                    key = (round(outer / 0.05), 1 if proj >= 0.0 else 0, round(proj, 3))
                    if esc is None or key > esc["key"]:
                        esc = {"key": key, "vx": vx, "vy": vy, "clear": cmin,
                               "outer": outer, "angle": a, "speed_frac": v_frac,
                               "proj": proj, "score": 0.0}
            if esc is None:
                # 全向都出界（理论上只在贴边时出现）
                self.stats["brake"] += 1
                here = self.surface_clearance(center[0], center[1])
                return (self.v_min * gu[0], self.v_min * gu[1],
                        {"mode": "FALLBACK", "clear": here, "outer": here - R_f,
                         "angle": 0.0})
            best = esc
            best["mode"] = "ESC-BACKOFF" if best["proj"] < 0.0 else "ESC-SIDE"
            if best["mode"] == "ESC-BACKOFF":
                self.stats["backoff"] += 1

        self._v_prev = (best["vx"], best["vy"])
        info = {"mode": best["mode"], "clear": best["clear"], "outer": best["outer"],
                "angle": best["angle"], "speed_frac": best["speed_frac"],
                "n_cand": len(cands), "n_lvl1": len(lvl1), "n_lvl2": len(lvl2),
                "proj_min": 0.0}
        return best["vx"], best["vy"], info


# =====================================================================================
# Scenario 4 运行参数（[plan2] 起集中在这里：只调速度/节奏就改这里，不动上面的逻辑）
#   nav 方法用 build_arena_planner() 建规划器；离线对比 sim_compare_avoidance.py 用的是
#   同一个函数，所以"离线测过的参数"就是"线上跑的参数"。
# =====================================================================================
S4_SPEED = dict(
    # --- 速度 ---
    v_max=1.00,                         # 巡航速度上限 m/s（_1 曾是 0.5）
    v_min=0.15,
    speed_fracs=(1.0, 0.70, 0.45, 0.0),  # 速度档（含停车档）
    # --- 控制/决策节奏 ---
    control_hz=20.0,                    # 直发 /motion_reference 的频率（替代 go_to action）
    update_interval=0.05,               # = 1 / control_hz
    replan_period=0.5,                  # A* 重规划周期
    max_track_speed=1.60,               # 回退用 go_to 时的跟踪速度上限
    lead_time=1.00,                     # 前视时间：抵消位置环 P 控制的稳态滞后
    position_kp=1.0,                    # pid_speed_controller 的 position_control kp（见 config_sim/config/pid_speed_controller.yaml）
    # --- 安全 ---
    safety_margin=0.30,
    tight_margin=0.12,
    sim_horizon=2.0,
    predict_horizon=2.0,
    scales=(1.0, 0.85, 0.7, 0.6, 0.5),
    pure_pursuit_lookahead=1.20,        # 速度提高后前视也要跟着变长，否则会切内角
    eta_slack=0.70,                     # 净空双向对冲的滞后系数（越小越保守）
)

# _1 用的那套参数（保留以便离线做 A/B 对照与随时回退）
S4_SPEED_LEGACY = dict(S4_SPEED, v_max=0.50, control_hz=3.33, update_interval=0.30,
                       replan_period=0.8, max_track_speed=0.85,
                       pure_pursuit_lookahead=0.90)

# [plan3] 安全档：**默认速度回到用户验证过的 0.5 m/s**，提速交给控制层（20Hz 直发参考）。
#   1.0 m/s 实测两次撞柱（见 README 1.12），所以先不靠"飞得快"来提速：
#   把 _1 实测的 0.067 m/s（跟踪效率 13%）修到 ≈0.5 m/s，本身就快 7 倍以上。
S4_SPEED_SAFE = dict(S4_SPEED,
                     v_max=0.50,
                     safety_margin=0.40,          # 0.30 -> 0.40
                     tight_margin=0.15,
                     eta_slack=0.55,              # 预测更保守（0.70 -> 0.55）
                     err_budget=0.20,             # 在线跟踪误差预算（并入队形外接半径）
                     speed_margin_gain=0.20,      # 余量随速度放大：margin += gain*v_max
                     ref_gap_cap=0.50,            # plan3：参考点相对实测中心的间隙上限(m)
                     per_drone_guard=0.45,        # 单机净空低于此值 -> 该机单独加避让偏移
                     per_drone_push=0.25,         # 单机避让偏移的上限(m)
                     pure_pursuit_lookahead=0.90)


S4_SPEED_FAST = dict(S4_SPEED,
                     # --- [plan4] 速度：用户在 0.5 m/s 上验证过安全；控制层已修好
                     #（_3 实测 send_ms p95 12ms / 最大 42ms、零降级），所以提到 0.7。
                     v_max=0.70,
                     # --- 安全：余量整体上调，因为速度上去了、且方向锥改锚路径后
                     # 编队会真的沿绕行路径走（会主动靠近柱子的"外侧"）
                     safety_margin=0.50,
                     tight_margin=0.18,
                     eta_slack=0.55,
                     err_budget=0.25,
                     speed_margin_gain=0.20,
                     # --- 参考点允许领先更多（P 控制 kp=1.0 要 0.7m 偏差才能给出 0.7m/s）
                     ref_gap_cap=0.75,
                     # --- 单机护栏略放大
                     per_drone_guard=0.55,
                     per_drone_push=0.35,
                     # --- 重规划周期放宽一点，减少"每 0.5s 换一次主意"的抖动
                     replan_period=0.60,
                     pure_pursuit_lookahead=1.10,
                     # --- [plan4] 预测采样加密一倍（有缓存，几乎不花钱）
                     sim_dt=0.125)


S4_SPEED_5 = dict(S4_SPEED,
                   # 速度不动（用户已确认够快），只让"绕"更早更宽
                   v_max=0.70,
                   safety_margin=0.60,        # 0.50 -> 0.60：可行域外扩，路径更早离开柱前
                   tight_margin=0.18,
                   eta_slack=0.55,
                   err_budget=0.25,
                   speed_margin_gain=0.20,
                   ref_gap_cap=0.75,
                   per_drone_guard=0.55,
                   per_drone_push=0.35,
                   replan_period=0.60,
                   pure_pursuit_lookahead=1.90,   # 1.10 -> 1.90：更早朝路径远处转
                   predict_horizon=2.50,          # 2.00 -> 2.50：预测窗口拉长
                   sim_dt=0.125)


def build_arena_planner(bounds, r_pillar, formation_base_radius, obstacle_bounds,
                        obstacle_height, cruise_alt, speed=None, verbose=True):
    """按 speed 字典（默认 S4_SPEED）构造规划器 —— nav 方法与离线对比脚本共用。"""
    cfg = dict(S4_SPEED if speed is None else speed)
    return ArenaAvoidancePlanner(
        bounds=bounds,
        r_pillar=r_pillar,
        formation_base_radius=formation_base_radius,
        obstacle_bounds=obstacle_bounds,
        obstacle_height=obstacle_height,
        cruise_alt=cruise_alt,
        obstacle_z_is_base=True,
        safety_margin=cfg["safety_margin"],
        grid_res=0.25,
        v_max=cfg["v_max"],
        v_min=cfg["v_min"],
        sim_horizon=cfg["sim_horizon"],
        predict_horizon=cfg["predict_horizon"],
        eta_slack=cfg.get("eta_slack", 0.70),
        speed_fracs=cfg["speed_fracs"],
        tight_margin=cfg["tight_margin"],
        scale_candidates=cfg["scales"],
        pure_pursuit_lookahead=cfg["pure_pursuit_lookahead"],
        sim_dt=float(cfg.get("sim_dt", 0.25)),
        verbose=verbose)


class SwarmConductor:
    def __init__(self, drones_ns: List[str], verbose: bool = False,
                 use_sim_time: bool = False, scenario_file: str = None):
        self.num_drones = len(drones_ns)
        self.drones = {}
        for idx, ns in enumerate(drones_ns):
            self.drones[idx] = SimpleDrone(ns, verbose, use_sim_time)
        self.formation_length_first_last = 0.75
        self.formation_length = 0.35  # Reduced spacing between drones (was 0.8)
        self.obstacles = []  # [x, y, z, radius]
        self.boundaries = [-10, 10, -10, 10, 0, 5]
        self.scenario_data = None
        self.leader_index = 0  # Default leader is drone0
        
        print(f"Initializing SwarmConductor with {self.num_drones} drones")
        
        if scenario_file:
            try:
                print(f"Attempting to load scenario file: {scenario_file}")
                with open(scenario_file, "r") as f:
                    scenario_data = yaml.safe_load(f)
                self.scenario_data = scenario_data
                print(f"Loaded scenario data: {self.scenario_data.keys()}")
                
                if "obstacles" in scenario_data:
                    self.obstacles = scenario_data["obstacles"]
                    print(f"Loaded {len(self.obstacles)} obstacles")
                
                if "stage2" in scenario_data:
                    stage2 = scenario_data["stage2"]
                    if "windows" in stage2:
                        windows = stage2["windows"]
                        window_ids = sorted(windows.keys(), key=lambda k: int(k))
                        print(f"Loaded {len(window_ids)} windows from stage2 data")
                        for wid in window_ids:
                            print(f"  Window {wid}: {windows[wid]}")
                
                print(f"Scenario file loaded: {scenario_file}")
            except Exception as e:
                print(f"Error loading scenario file: {e}")
                import traceback
                traceback.print_exc()

    def reset_all(self):
        """[fix11] 把 5 架机复位到起飞前的初始状态与位置，便于下一轮测试。

        之前每跑完一轮都留下"offboard=true 但 disarmed"的脏状态，下一轮的
        takeoff 目标会被行为服务器拒收（Goal Rejected）；这里在退出前统一
        关 offboard -> disarm -> 把 gz 里的模型位姿搬回世界文件里的初始位置。
        """
        import os as _os
        import subprocess

        print("Resetting all drones to their initial state (offboard off, disarm, teleport home)...")
        for drone in self.drones.values():
            try:
                drone.manual()
            except Exception as exc:  # noqa: BLE001
                print(f"  {drone.drone_id} manual() failed: {exc}", flush=True)

        for drone in self.drones.values():
            try:
                drone.disarm()
            except Exception as exc:  # noqa: BLE001
                print(f"  {drone.drone_id} disarm() failed: {exc}", flush=True)

        world_yaml = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                                   "config_sim/world/world.yaml")
        if not _os.path.exists(world_yaml):
            print(f"  {world_yaml} not found - skipping teleport back home")
            return

        with open(world_yaml, "r") as handle:
            world = yaml.safe_load(handle)
        world_name = world.get("world_name", "empty")
        spawn = {entry["model_name"]: entry["xyz"] for entry in world.get("drones", [])}

        for drone in self.drones.values():
            xyz = spawn.get(drone.drone_id)
            if xyz is None:
                continue
            x, y, z = xyz
            request = (f'name: "{drone.drone_id}", '
                       f'position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1.0}}')
            command = ["gz", "service", "-s", f"/world/{world_name}/set_pose",
                       "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                       "--timeout", "5000", "--req", request]
            try:
                result = subprocess.run(command, capture_output=True, text=True, timeout=15)
                ok = "true" in (result.stdout + result.stderr)
            except Exception as exc:  # noqa: BLE001
                ok = False
                print(f"  {drone.drone_id} teleport error: {exc}", flush=True)
            print(f"  {drone.drone_id} -> {xyz} : {'ok' if ok else 'FAILED'}", flush=True)

    def takeoff_all(self):
        # First arm and set to offboard mode for all drones
        for drone in self.drones.values():
            beat(f"arming {drone.drone_id}")
            print(f"[fix] arming {drone.drone_id} ...", flush=True)
            ok_arm = drone.arm()
            print(f"[fix] {drone.drone_id} arm -> {ok_arm}", flush=True)
            print(f"[fix] offboard {drone.drone_id} ...", flush=True)
            ok_off = drone.offboard()
            print(f"[fix] {drone.drone_id} offboard -> {ok_off}", flush=True)

        # Set different LED colors for different drones to distinguish them
        colors = [
            (255, 0, 0),     # Red
            (0, 255, 0),     # Green
            (0, 0, 255),     # Blue
            (255, 255, 0),   # Yellow
            (255, 0, 255)    # Purple
        ]
        
        for i, drone in enumerate(self.drones.values()):
            color_idx = i % len(colors)
            drone.change_led_colour(colors[color_idx])
        
        # Send takeoff commands to all drones without waiting (non-blocking)
        print("Sending takeoff commands to all drones simultaneously...")
        for drone in self.drones.values():
            # Use non-blocking takeoff (False parameter)
            drone.takeoff(2.0, 1.0, False)
        
        # Wait for all drones to complete takeoff
        print("Waiting for all drones to reach takeoff altitude...")
        all_reached_altitude = False
        max_wait_time = 5  # Maximum wait time in seconds
        start_time = time.time()
        
        while not all_reached_altitude and time.time() - start_time < max_wait_time:
            all_reached_altitude = True
            for drone in self.drones.values():
                try:
                    # Check if drone has reached target altitude
                    current_altitude = drone.position[2]
                    if current_altitude < 1.9:  # Adjusted from 1.4 to 1.9 for higher altitude
                        all_reached_altitude = False
                        break
                except Exception:
                    all_reached_altitude = False
                    break
            
            if not all_reached_altitude:
                time.sleep(0.2)  # Short sleep before checking again
        
        print("All drones have taken off.")
        
        # Give an additional delay for stability
        time.sleep(0.5)   # [plan5] 2.0 -> 0.5
        
        # After takeoff, change formation to pentagon shape
        print("Changing formation to pentagon shape after takeoff...")
        self.change_to_pentagon_formation()
        
    def change_to_pentagon_formation(self):
        """
        Change the formation to a pentagon shape after takeoff
        """
        print("Setting up pentagon formation...")
        
        # Set an orientation for the formation (we'll use this throughout the mission)
        formation_orientation = 90.0  # Exactly forward-facing (positive Y direction)
        self.formation_orientation = formation_orientation
        
        # Get all drones
        all_drones = list(self.drones.values())
        
        # Calculate the current center of the formation
        current_positions = [drone.get_current_position() for drone in all_drones]
        formation_center = [
            sum(pos[0] for pos in current_positions) / len(current_positions),
            sum(pos[1] for pos in current_positions) / len(current_positions),
            sum(pos[2] for pos in current_positions) / len(current_positions)
        ]
        
        # Generate pentagon formation positions
        pentagon_positions = self.generate_formation_positions(formation_center, formation_orientation)
        
        # Move all drones to the pentagon formation positions
        print("Moving drones to pentagon formation positions...")
        self.transition_to_formation(all_drones, pentagon_positions, formation_orientation)
        
        # Set different LED colors for different drones to distinguish them
        colors = [
            (255, 0, 0),     # Red
            (0, 255, 0),     # Green
            (0, 0, 255),     # Blue
            (255, 255, 0),   # Yellow
            (255, 0, 255)    # Purple
        ]
        
        for i, drone in enumerate(all_drones):
            color_idx = i % len(colors)
            drone.change_led_colour(colors[color_idx])
            
        print("Pentagon formation complete.")

    def land_all(self):
        """Land all drones simultaneously but at their current horizontal positions"""
        # Set LED color to red for all drones
        for drone in self.drones.values():
            drone.change_led_colour((255, 0, 0))  # Red for landing
        
        # Send landing commands to all drones without waiting (non-blocking)
        # Each drone will land at its current horizontal position
        print("Sending landing commands to all drones simultaneously...")
        for drone in self.drones.values():
            # Use non-blocking land (False parameter)
            drone.land(0.8, False)
        
        # Wait for all drones to reach the ground
        print("Waiting for all drones to land...")
        all_landed = False
        max_wait_time = 15  # Maximum wait time in seconds
        start_time = time.time()
        
        while not all_landed and time.time() - start_time < max_wait_time:
            all_landed = True
            for drone in self.drones.values():
                try:
                    # Check if drone has reached ground
                    current_altitude = drone.position[2]
                    if current_altitude > 0.1:  # If altitude is still significantly above ground
                        all_landed = False
                        break
                except Exception:
                    all_landed = False
                    break
        
            if not all_landed:
                time.sleep(0.2)  # Short sleep before checking again
        
        
    def shutdown(self):
        """Shutdown all drone interfaces"""
        for drone in self.drones.values():
            try:
                drone.destroy_node()
            except Exception as e:
                print(f"Error shutting down drone: {e}")

    def execute_scenario1_mission(self):
        """Execute scenario 1 mission (placeholder)"""
        print("Scenario 1 mission not implemented yet")

    def execute_scenario2_mission(self):
        """Execute scenario 2 mission (placeholder)"""
        print("Scenario 2 mission not implemented yet")

    def execute_scenario3_mission(self):
        """Execute scenario 3 mission (placeholder)"""
        print("Scenario 3 mission not implemented yet")

    def execute_scenario4_mission(self):
        """
        Scenario 4: Dynamic obstacle avoidance
         - Reads stage4 from YAML
         - Subscribes to dynamic obstacle positions
         - Plans and executes path from start to end while avoiding moving obstacles
        """
        # 1) Load and validate stage4 data
        if not (self.scenario_data and "stage4" in self.scenario_data):
            print("No stage4 data loaded.")
            return
        stage4 = self.scenario_data["stage4"]
        x0, y0 = stage4["stage_center"]     # (x0, y0)
        sx_rel, sy_rel = stage4["start_point"]
        ex_rel, ey_rel = stage4["end_point"]
        obs_h = stage4["obstacle_height"]
        obs_d = stage4["obstacle_diameter"]
        

        # 2) Compute global origin, start and end
        # [fix11] 巡航高度必须低于柱顶，否则整条航路都在柱子之上、实时避障永远
        # 触发不了。柱子高 obs_h(=5.0) 且现立在 z=2.5 -> 顶端 5.0m；下面的自检
        # 会在高度改错时立刻喊出来（原注释写 4.5m 但值是 3.5m，是过期注释）。
        cruise_alt = 3.5
        print(f"Cruise altitude {cruise_alt} m vs obstacle top {obs_h} m -> "
              f"{'BELOW top (avoidance testable)' if cruise_alt < obs_h else 'ABOVE the top - avoidance CANNOT trigger!'}")
        origin_wp = [0.0, 0.0, cruise_alt]
        start_wp  = [x0 - sy_rel, y0 - sx_rel, cruise_alt]
        start_wp[1] -= 1.5
        end_wp    = [x0 - ey_rel, y0 - ex_rel, cruise_alt]
        end_wp[1] += 2.5
        # Create final waypoint 1.5m further in negative Y direction from end point
        # final_wp  = [end_wp[0], end_wp[1] - 1.5, end_wp[2]]
        
        print("Origin:", origin_wp)
        print("Start :", start_wp)
        print("End   :", end_wp)
        # print("Final :", final_wp)

        # 3) Subscribe to dynamic obstacle positions
        self.dynamic_obstacles = {}  # Dictionary to store latest obstacle positions
        self.obstacle_rx = {}        # [plan5] 逐 id 到达计数（诊断"只收到 3/5"）
        self.setup_dynamic_obstacle_subscriber()

        start_time = time.time()
        
        # 4) FIRST PHASE: Go from origin to start in pentagon formation
        print("\n--- PHASE 1: Moving from origin to start point in pentagon formation ---")
        self.using_wide_formation = False
        
        # Set an orientation for the formation (looking forward)
        forward_orientation = 90.0  # Exactly forward-facing (positive Y direction)
        self.formation_orientation = forward_orientation
        
        # Get all drones
        all_drones = list(self.drones.values())
        
        # Define the current center (origin) and target (start point)
        current_positions = [drone.get_current_position() for drone in all_drones]
        formation_center = [
            sum(pos[0] for pos in current_positions) / len(current_positions),
            sum(pos[1] for pos in current_positions) / len(current_positions),
            sum(pos[2] for pos in current_positions) / len(current_positions)
        ]
        
        # Generate pentagon formation positions at the origin
        origin_pentagon_positions = self.generate_formation_positions(origin_wp, forward_orientation, scale=0.8)
        
        # Move all drones to the pentagon formation at origin
        print("Moving to pentagon formation at origin...")
        self.transition_to_formation(all_drones, origin_pentagon_positions, forward_orientation)
        
        # Brief pause to stabilize formation
        time.sleep(0.3)   # [plan5]
        
        # Generate pentagon formation positions at the start point
        start_pentagon_positions = self.generate_formation_positions(start_wp, forward_orientation, scale=0.8)
        
        # # Move the formation to the start point while maintaining the pentagon shape
        # print("Moving pentagon formation to start point...")
        # self.move_formation_to_waypoint(start_pentagon_positions, forward_orientation)
        
        # print("Formation has reached start point in pentagon formation.")
        
        # 5) SECOND PHASE: Navigate through dynamic obstacles to end point
        print("\n--- PHASE 2: Navigating through dynamic obstacles to end point ---")
        self.using_wide_formation = False  # Switch to tight formation
        
        # Execute dynamic obstacle avoidance
        self.navigate_through_dynamic_obstacles(start_wp, end_wp)
        
        print("Scenario 4 complete with dynamic obstacle avoidance.")

        end_time = time.time()
        print(f"Mission Completion Time: {end_time - start_time:.2f} seconds")

    def setup_dynamic_obstacle_subscriber(self):
        """
        Set up a subscriber to receive dynamic obstacle positions
        """
        # Need to use a node for subscription
        try:
            # Use one of the existing drones as a node for subscription
            leader = self.drones[self.leader_index]
            
            # Create a callback to handle incoming dynamic obstacle positions
            def obstacle_position_callback(msg):
                # Extract obstacle ID from frame_id (format: "object_X")
                try:
                    obstacle_id = msg.header.frame_id
                    position = [
                        msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z
                    ]
                    # Store latest position for this obstacle
                    self.dynamic_obstacles[obstacle_id] = position
                    self.obstacle_rx[obstacle_id] = \
                        self.obstacle_rx.get(obstacle_id, 0) + 1   # [plan5]
                    # print(f"Received position for {obstacle_id}: {position}")
                    
                except Exception as e:
                    print(f"Error processing obstacle position: {e}")
            
            # Subscribe to the dynamic obstacles topic
            self.obstacle_sub = leader.create_subscription(
                PoseStamped,
                "/dynamic_obstacles/locations",
                obstacle_position_callback,
                10
            )
            print("Subscribed to dynamic obstacle positions")
        except Exception as e:
            print(f"Failed to set up dynamic obstacle subscriber: {e}")
            # Initialize with empty obstacles as fallback
            self.dynamic_obstacles = {}

    def execute_waypoint_movement(self, waypoints):
        """
        Execute movement through a series of waypoints for all drones
        Modified to treat all drones equally in a pentagon formation
        """
        all_drones = list(self.drones.values())
        # Always use forward orientation (90 degrees)
        fixed_yaw = 90.0
        self.formation_orientation = fixed_yaw
        
        for idx, wp in enumerate(waypoints):
            print(f"\nMoving to waypoint {idx+1}/{len(waypoints)}: {wp}")
            
            # All drones move to their respective path points
            for drone in all_drones:
                if idx < len(drone.path):
                    tx, ty, tz = drone.path[idx]
                    drone.go_to(tx, ty, tz,
                              0.8, YawMode.FIXED_YAW, fixed_yaw,
                              "earth", False)
                    drone.is_moving = True
            
            # Wait until all have reached this index
            self.wait_for_all_drones_to_reach_waypoint(all_drones, idx)

    def wait_for_all_drones_to_reach_waypoint(self, drones, idx):
        """
        Wait for all drones to reach the specified waypoint index
        """
        active = drones.copy()
        start_t = time.time()
        while active and time.time() - start_t < 60:
            for d in active[:]:
                arrived = False
                # 1) behavior status
                try:
                    st = super(SimpleDrone, d).get_behavior_status()
                    if st is None or st.status == BehaviorStatus.IDLE:
                        arrived = True
                except:
                    pass
                # 2) position fallback
                if not arrived:
                    try:
                        cur = d.position
                        target = d.path[idx]
                        if math.dist(cur, target) < 0.3:
                            arrived = True
                    except Exception as e:
                        print(f"Error checking position for {d.drone_id}: {e}")
                
                if arrived:
                    print(f"  {d.drone_id} reached waypoint {idx+1}")
                    d.is_moving = False
                    d.change_led_colour((0, 0, 255))  # Blue for all drones
                    active.remove(d)
            if active:
                time.sleep(0.5)

    def _formation_base_radius(self):
        """generate_formation_positions() 用的五边形基准半径（宽编队 0.8 / 紧编队 0.5）。"""
        return 0.8 if getattr(self, "using_wide_formation", True) else 0.5

    def navigate_through_dynamic_obstacles(self, start_wp, end_wp):
        """[plan5] Scenario 4 Phase 2：**安全优先**的速度版（_3）。

        _2（1.0 m/s + 20Hz 直发参考）真机两次撞柱后，逐项定位到三个真因并逐条修掉：

        真因①（最致命）**控制链路能把循环卡死几十秒，期间无人机在盲飞**。
          `as2_motion_reference_handlers` 的 `send_pose_command()` → `check_mode()` →
          模式不匹配时进 `__set_mode()`：`wait_for_service(3s)` + `call()` +
          `while 当前模式!=目标: sleep(0.1)`（最多再 5s）。而 _2 的降级门槛是
          "同一架机累计 3 次 >0.5s"才降级 → 模式切不过去时每架每拍阻塞 ~6s、
          5 架串行 ~25s/拍；这 25s 里无人机仍按**上一条参考**全速飞 → 撞柱 → 碰撞后姿态失控。
          修法：起飞后先做**控制链路预检**（每架一次"原地保持"探测，线程 + 1.0s 硬超时，
          超时/失败直接该机降级走 go_to）；运行中**只要有一拍的发送总耗时 >0.5s 就立刻整体降级**
          （不再"连续 3 次"），并且每拍都检查。
        真因②**安全半径用的是"假定五边形"，不是实测队形**。改用实测外接半径
          `max(名义 R_f, max_i|p_i-center|)` + 在线误差预算 0.20m，余量 0.30→0.40m，
          并把预测的滞后对冲从 0.70 收紧到 0.55。
        真因③**单机偏差没有任何纠正手段**（所有机共用"中心速度+槽位"）。
          新增单机护栏：某机自身净空 < 0.45m 时，只给它加一个背离最近柱子的偏移（≤0.25m）。

        另外：默认速度回到**用户已验证过的 0.5 m/s**（提速交给控制层：
        _1 实测只有 0.067 m/s，把它修到 ≈0.5 就是 7 倍以上，不需要靠飞得快）；
        参考点改成**世界系积分 + 间隙上限**（避免 _2 的"位置前视"把目标点推到计划点之前）；
        航迹**边跑边落盘**（撞机中断也不会丢证据）；检测到几何重叠立刻中止编队并进入降落。
        """
        _MOTION_MODE = "stream"          # "stream"（直发参考）| "go_to"（保底/对照）
        # [plan5] 1.0 -> 3.0：真机实测**健康**的第一次模式切换就要 0.797 / 0.958 / 1.018 s
        # （HOVER->POSITION 的服务调用 + 轮询），1.0s 的阈值会把好链路判死。
        _PREFLIGHT_TIMEOUT = 3.0         # 控制链路预检的单机硬超时（秒）
        _TICK_SEND_BUDGET = 0.5          # 单拍发送总耗时上限，超了立刻整体降级
        _GOTO_RESEND = 1.0               # go_to 模式下的最小重发间隔（秒）
        _ABORT_ON_CONTACT = True         # 检测到几何重叠 -> 立即中止编队（随后降落）

        cfg = dict(S4_SPEED_5)             # [plan5] 0.7 m/s + 更早更宽的绕行
        fixed_yaw = 90.0
        self.formation_orientation = fixed_yaw

        stage4 = self.scenario_data["stage4"]
        obstacle_height = float(stage4["obstacle_height"])
        obstacle_diameter = float(stage4["obstacle_diameter"])
        obstacle_speed = float(stage4.get("obstacle_velocity", 0.5))
        stage_center = [float(v) for v in stage4["stage_center"]]
        stage_size = [float(v) for v in self.scenario_data.get("stage_size", [10.0, 10.0])]
        cruise_alt = float(start_wp[2])

        all_drones = list(self.drones.values())
        formation_base_radius = self._formation_base_radius()
        phase_t = {}

        arena = (stage_center[0] - stage_size[0] / 2.0,
                 stage_center[0] + stage_size[0] / 2.0,
                 stage_center[1] - stage_size[1] / 2.0,
                 stage_center[1] + stage_size[1] / 2.0)
        r_pillar = (obstacle_diameter / 2.0) * math.sqrt(2.0)
        nav_bounds = (arena[0], arena[1],
                      min(arena[2], float(start_wp[1])) - 0.2,
                      max(arena[3], float(end_wp[1])) + 0.2)

        extra_radius = float(cfg["err_budget"])
        extra_margin = float(cfg["speed_margin_gain"]) * float(cfg["v_max"])

        print("\n[plan5] ================= Scenario 4 安全档自检 =================")
        print(f"[plan5] arena x[{arena[0]:.2f},{arena[1]:.2f}] y[{arena[2]:.2f},{arena[3]:.2f}]  "
              f"nav x[{nav_bounds[0]:.2f},{nav_bounds[1]:.2f}] y[{nav_bounds[2]:.2f},{nav_bounds[3]:.2f}]")
        print(f"[plan5] 柱外接半径 {r_pillar:.3f} m；巡航 {cruise_alt:.2f} m / 柱高 "
              f"{obstacle_height:.2f} m ({'避障可触发' if cruise_alt <= obstacle_height else '避障不可触发!'})")
        print(f"[plan5] 速度档 v_max={cfg['v_max']:.2f} m/s  speed_fracs={cfg['speed_fracs']}  "
              f"scales={cfg['scales']}")
        print(f"[plan5] 安全：余量 {cfg['safety_margin']:.2f} + {extra_margin:.2f}(随速度) m；"
              f"队形半径额外预算 {extra_radius:.2f} m；预测滞后对冲 eta_slack={cfg['eta_slack']}")
        print(f"[plan5] 中心到柱轴所需净距 ≈ {r_pillar + 0.5 + extra_radius + cfg['safety_margin'] + extra_margin:.2f} m "
              f"(scale=1.0 时；_2 同口径只有 {r_pillar + 0.5 + 0.30:.2f} m)")
        print(f"[plan5] 控制：{_MOTION_MODE} @ {cfg['control_hz']:.0f}Hz，参考=世界系积分+间隙上限 "
              f"{cfg['ref_gap_cap']:.2f} m；单机护栏 {cfg['per_drone_guard']:.2f} m")
        print("[plan5] ======================================================\n")

        # ---- 编队先收成五边形 ----
        _t0 = time.time()
        current_positions = [drone.get_current_position() for drone in all_drones]
        formation_center = [
            sum(p[0] for p in current_positions) / len(current_positions),
            sum(p[1] for p in current_positions) / len(current_positions),
            sum(p[2] for p in current_positions) / len(current_positions),
        ]
        pentagon_positions = self.generate_formation_positions(formation_center, fixed_yaw)
        self.transition_to_formation(all_drones, pentagon_positions, fixed_yaw)
        time.sleep(0.3)
        phase_t["settle_formation"] = time.time() - _t0

        # ---- 等障碍物数据 ----
        _t0 = time.time()
        wait_start = time.time()
        while not self.dynamic_obstacles and time.time() - wait_start < 10.0:
            print("[plan5] Waiting for dynamic obstacle data...")
            time.sleep(0.5)
        if not self.dynamic_obstacles:
            print("[plan5] WARNING: 收不到 /dynamic_obstacles/locations，退化为直线飞向终点。")
            for wp in [formation_center, end_wp]:
                self.move_formation_to_waypoint(
                    self.generate_formation_positions(wp, fixed_yaw), fixed_yaw)
            return

        obstacle_prev_positions = {}
        obstacle_velocities = {}
        last_velocity_update = 0.0
        _VEL_DT = 0.3

        def update_obs_velocities(now):
            nonlocal last_velocity_update
            if now - last_velocity_update <= _VEL_DT:
                return
            dtv = max(1e-3, now - last_velocity_update)
            for oid, pos in list(self.dynamic_obstacles.items()):
                pos = list(pos)
                prev = obstacle_prev_positions.get(oid)
                if prev is not None:
                    vv = [(pos[i] - prev[i]) / dtv for i in range(3)]
                    old = obstacle_velocities.get(oid)
                    if old is None:
                        obstacle_velocities[oid] = vv
                    else:
                        obstacle_velocities[oid] = [0.7 * vv[i] + 0.3 * old[i] for i in range(3)]
                else:
                    obstacle_velocities.setdefault(oid, [0.0, 0.0, 0.0])
                obstacle_prev_positions[oid] = pos
            last_velocity_update = now

        # [plan5] 收集窗口 1.0s -> "凑齐或长时间无进展才走"，**必须凑齐 5 根才起飞**。
        # 真机实测（2026-10-08 12:0x）：gz 侧每根柱子 18 条/5s = 3.6Hz，
        # 但 ROS 侧只有 7 条/5s（5 个 frame_id 各 1.4Hz）—— 桥丢了大半。
        # 1.0s 的窗口按 coupon collector 只能看到 ~3.8 根：第二趟就只认出 3 根，
        # 编队随后一头撞进"看不见的那两根"里（clear_drone 0.08 -> 0.03，t=51.3s 中止）。
        _expect_obs = int(_os.environ.get("S4_EXPECT_OBSTACLES", "5"))
        print(f"[plan5] collecting initial dynamic obstacle data "
              f"(需要 {_expect_obs} 根；ROS 侧约 1.4Hz/根，凑齐通常要 2~4s)...")
        _collect_start = time.time()
        _last_progress = _collect_start
        _prev_n = 0
        while True:
            update_obs_velocities(time.time())
            _n_seen = len(self.dynamic_obstacles)
            if _n_seen > _prev_n:
                _prev_n = _n_seen
                _last_progress = time.time()
            if _n_seen >= _expect_obs:
                break
            if time.time() - _last_progress > 6.0:
                break
            time.sleep(0.05)
        update_obs_velocities(time.time() + _VEL_DT)
        _seen = sorted(self.dynamic_obstacles.keys())
        if len(_seen) < _expect_obs:
            _rx_drone = getattr(self, "obstacle_rx", {})
            print(f"[plan5] !! 6s 内无新增，只收到 {len(_seen)}/{_expect_obs} 根柱子 "
                  f"({_seen})：**场地里有看不见的柱子**，本趟不保证安全。")
            print(f"[plan5]    取证 · drone0 那路订阅计数: {_rx_drone}")
            print(f"[plan5]    取证 · 本脚本第二路(best-effort)计数: {_obs_rx if '_obs_rx' in dir() else {}}")
            print(f"[plan5]    对照命令: `ros2 topic hz /dynamic_obstacles/locations`"
                  f"（正常 >5Hz）；`gz topic -e -t /dynamic_obstacles/locations` 应看到 5 个 object_*")
        else:
            print(f"[plan5] 柱子凑齐 {len(_seen)}/{_expect_obs}，用时 "
                  f"{time.time() - _collect_start:.1f}s")
        phase_t["collect_obstacles"] = time.time() - _t0
        print(f"[plan5] received data for {len(self.dynamic_obstacles)} dynamic obstacles "
              f"(obstacle_velocity={obstacle_speed} m/s)")

        planner = build_arena_planner(nav_bounds, r_pillar, formation_base_radius, arena,
                                      obstacle_height, cruise_alt, speed=cfg, verbose=True)

        # ---- 开局净空自检：如果一开始就贴着柱子，先喊出来 ----
        planner.update_obstacles(self.dynamic_obstacles, obstacle_velocities,
                                 speed_hint=obstacle_speed)
        start_clear = planner.min_drone_clearance(current_positions)
        start_reff = 0.0
        for q in current_positions:
            start_reff = max(start_reff, math.hypot(q[0] - formation_center[0],
                                                    q[1] - formation_center[1]))
        print(f"[plan5] 开局自检：最危险那架机到柱面净空 {start_clear:.2f} m；"
              f"实测队形外接半径 {start_reff:.2f} m（名义 {formation_base_radius:.2f}）")
        if start_clear < 1.0:
            print(f"[plan5] !! 开局净空只有 {start_clear:.2f} m —— 柱子就压在集结点/起飞点附近。"
                  f"根因：柱子活动区 = 赛场本身（x[-5,5] y[1,11]），柱体 0.5 宽 -> 最南到 y=0.75，"
                  f"而五边形集结点最北机位 y=+0.76（半径 0.8）→ 柱子停在边界就压上来。"
                  f"修法：`python3 patch_pillar_bounds.py` 把柱子活动区收缩 1.5m（**需重启栈**）。")

        # ================= [plan5] 控制链路预检 =================
        motion_mode = _MOTION_MODE
        streamers = {}
        if motion_mode == "stream":
            try:
                from as2_motion_reference_handlers.position_motion import PositionMotion
            except Exception as exc:  # noqa: BLE001
                print(f"[plan5] PositionMotion 不可用 ({exc}) -> 整体退回 go_to")
                motion_mode = "go_to"
        if motion_mode == "stream":
            # [plan5] 三处修正（都是实测踩出来的）：
            #  ① 5 架**并行**探测，不再串行（串行时 5 架各 ~1s，而且是"卡在最后一架"的假象）；
            #  ② 超时阈值 1.0 -> 3.0 s：HOVER->POSITION 的第一次切换本来就要 0.8~1.0 s
            #     （真机日志：drone1 0.797s / drone3 0.958s / drone4 1.018s），
            #     1.0s 会把**健康的**链路判死 —— 上一轮 5 架全被判超时，整场退化成 go_to；
            #  ③ 首次超时**重试一次**（此时模式往往已经切过去了，第二次通常秒回）；
            #     两次都超时才降级。真正的病态情形是单次 ~6s（wait_for_service 3s + 轮询 5s），
            #     3.0 x 2 = 6.0s 仍然拦得住。
            probes = {}
            for d in all_drones:
                try:
                    st = PositionMotion(d)
                except Exception as exc:  # noqa: BLE001
                    print(f"[plan5] {d.drone_id} PositionMotion 构造失败 ({exc})")
                    streamers[d.drone_id] = None
                    continue
                # [plan5] **致命一行**：构造函数成功后必须把 streamer 登记进 streamers。
                # plan3 漏了这一行 -> `n_ok` 恒为 0 -> 整场**静默**退化成 go_to
                # （真机实测：5 架预检全部 OK，却紧跟着打印"没有任何一架通过控制链路预检"）。
                # 这就是"速度又变慢了"的主因：所谓"控制层修复"从来没被执行过。
                streamers[d.drone_id] = st
                pos = d.get_current_position()
                box = {"ok": False, "t": -1.0, "try": 0}

                def _probe(_st=st, _x=pos[0], _y=pos[1], _z=pos[2], _box=box):
                    while _box["try"] < 2:
                        _box["try"] += 1
                        t0 = time.time()
                        try:
                            _box["ok"] = _st.send_position_command_with_yaw_angle(
                                [_x, _y, _z], twist_limit=cfg["v_max"], pose_frame_id="earth",
                                twist_frame_id="earth", yaw_angle=math.radians(fixed_yaw))
                        except Exception as exc:  # noqa: BLE001
                            _box["ok"] = False
                            _box["exc"] = str(exc)
                        _box["t"] = time.time() - t0
                        if _box["ok"]:
                            return

                th = threading.Thread(target=_probe, daemon=True)
                th.start()
                probes[d.drone_id] = (d, th, box)

            # 并行等待：所有线程共享同一个 deadline
            _deadline = time.time() + 2.0 * _PREFLIGHT_TIMEOUT
            for did, (d, th, box) in probes.items():
                th.join(timeout=max(0.1, _deadline - time.time()))
            for did, (d, th, box) in probes.items():
                if th.is_alive():
                    print(f"[plan5] {d.drone_id} 控制链路预检超时 "
                          f"(2 次 x {_PREFLIGHT_TIMEOUT:.1f}s 都没回) -> 该机走 go_to")
                    streamers[d.drone_id] = None
                elif not box.get("ok", False):
                    print(f"[plan5] {d.drone_id} 控制链路预检失败 "
                          f"(ok=False, {box.get('t', -1):.2f}s{', ' + box['exc'] if 'exc' in box else ''})"
                          f" -> 该机走 go_to")
                    streamers[d.drone_id] = None
                else:
                    print(f"[plan5] {d.drone_id} 控制链路预检 OK "
                          f"({box.get('t', -1):.3f}s, 第 {box.get('try', 1)} 次)"
                          f" -> POSITION 直发参考")
            n_ok = sum(1 for v in streamers.values() if v is not None)
            if n_ok == 0:
                print("[plan5] 没有任何一架通过控制链路预检 -> 整体走 go_to")
                motion_mode = "go_to"
            else:
                print(f"[plan5] 控制层 = 直发 /motion_reference {cfg['control_hz']:.0f}Hz，"
                      f"{n_ok}/{len(all_drones)} 架可用"
                      f"{'（其余走 go_to）' if n_ok < len(all_drones) else ''}")
        if motion_mode == "go_to":
            print(f"[plan5] 控制层 = go_to action（前视 {cfg['ref_gap_cap']:.1f}s 的点 + "
                  f"线程并发下发 + 最慢每 {_GOTO_RESEND:.1f}s 重发一次）")

        goal = [float(end_wp[0]), float(end_wp[1]), cruise_alt]
        reached_end = False
        aborted = False
        last_update = time.time()
        start_navigation_time = last_update
        max_navigation_time = 300.0
        last_plan_time = 0.0
        last_goto_time = 0.0
        path = None
        scale = 1.0
        prev_mode = None
        tick = 0
        period = 1.0 / max(1.0, cfg["control_hz"])
        log_every = max(1, int(round(1.0 / period)))
        prev_center = None
        prev_tick_time = None
        ach_hist = []
        cmd_hist = []
        warn_done = False
        slow_total = 0
        ref = list(formation_center[:2])          # 世界系积分参考
        tick_over_budget = 0

        # [plan5] 航迹**边跑边落盘**：撞机/中断也不会丢证据
        import csv as _csv
        trace_dir = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "traces")
        _os.makedirs(trace_dir, exist_ok=True)
        trace_file = _os.path.join(trace_dir,
                                   "scenario4_4_" + time.strftime("%Y%m%d_%H%M%S") + ".csv")
        trace_fh = open(trace_file, "w", newline="")
        trace_w = _csv.writer(trace_fh)
        _hdr_extra = []
        for _hi in range(len(all_drones)):
            _hdr_extra += [f"d{_hi}_x", f"d{_hi}_y", f"d{_hi}_z", f"d{_hi}_v",
                           f"d{_hi}_lag", f"d{_hi}_clr"]
        trace_w.writerow(["t", "center_x", "center_y", "vx", "vy", "d_goal",
                          "clear_drone", "reff", "scale", "path_pts", "mode",
                          "cmd_speed", "ach_speed", "send_ms",
                          "sim_t", "rtf", "t_pos_ms", "t_plan_ms", "t_step_ms",
                          "n_cand", "n_lvl1", "n_lvl2", "n_demote"] + _hdr_extra)
        print(f"[plan5] trace -> {trace_file} (逐拍写入，中断也保留)")

        # ---- [plan5] 仿真时钟探针：/clock -> 实时率 RTF ----
        # _3 的航迹里 cmd=0.42 / ach=0.143（比值 0.34），单看这个数会误判成
        # "控制层跟不上"；Gazebo 侧 real_time_factor 实测 ≈0.5，即 0.5 m/s 的
        # **仿真时间**速度在墙钟上本来就只对应 ~0.25 m/s。没有这一列，
        # 就没法区分"飞得慢"和"仿得慢"——而这正是本轮要回答的第一个问题。
        _sim_t = {"t": None, "t0": None, "w0": None, "rtf": 0.0, "n": 0}
        _clk_node = None
        _clk_ex = None
        try:
            import rclpy as _rclpy
            from rosgraph_msgs.msg import Clock as _Clock
            from rclpy.executors import SingleThreadedExecutor as _STE

            _clk_node = _rclpy.create_node("plan4_clock_probe")

            def _on_clock(_msg, _st=_sim_t):
                _tt = _msg.clock.sec + _msg.clock.nanosec * 1e-9
                _st["t"] = _tt
                _st["n"] += 1
                if _st["t0"] is None:
                    _st["t0"], _st["w0"] = _tt, time.time()
                else:
                    _dw = time.time() - _st["w0"]
                    if _dw >= 1.0:
                        _st["rtf"] = (_tt - _st["t0"]) / _dw
                        _st["t0"], _st["w0"] = _tt, time.time()

            _clk_node.create_subscription(_Clock, "/clock", _on_clock, 10)
            _clk_ex = _STE()
            _clk_ex.add_node(_clk_node)
            threading.Thread(target=_clk_ex.spin, daemon=True).start()
            print("[plan5] /clock 探针已启动（rtf / sim_t 列）")

            # ---- [plan5] 第二路障碍订阅（独立节点 + best-effort）----
            # 目的：_4 两趟真机里 drone0 节点上那路订阅只收到 3/5 根柱子，
            # 而独立探针能看到 5/5。这里另起一个干净节点、用 best-effort QoS 再订一份，
            # 两路写同一个 dict（取并集），并分别计数以便定位"丢消息 vs 键冲突"。
            try:
                from rclpy.qos import qos_profile_sensor_data as _qos_sd
                from geometry_msgs.msg import PoseStamped as _PoseStamped
                _obs_node = _rclpy.create_node("plan5_obstacle_best_effort")
                _obs_rx = {}

                def _on_obs(_m, _st=self.dynamic_obstacles, _rx=_obs_rx):
                    _id = _m.header.frame_id or "<empty>"
                    _st[_id] = [_m.pose.position.x, _m.pose.position.y,
                                _m.pose.position.z]
                    _rx[_id] = _rx.get(_id, 0) + 1

                _obs_node.create_subscription(_PoseStamped,
                                              "/dynamic_obstacles/locations",
                                              _on_obs, _qos_sd)
                _obs_ex = _STE()
                _obs_ex.add_node(_obs_node)
                threading.Thread(target=_obs_ex.spin, daemon=True).start()
                print("[plan5] 第二路障碍订阅已启动（best-effort，独立节点）")
            except Exception as _exc:                    # noqa: BLE001
                _obs_node = None
                _obs_ex = None
                _obs_rx = {}
                print(f"[plan5] 第二路障碍订阅启动失败 ({_exc})")
        except Exception as _exc:                       # noqa: BLE001
            print(f"[plan5] /clock 探针启动失败 ({_exc}) -> rtf 列恒为 0")

        # ---- [plan5] 分段计时：每拍时间到底花在 读位姿 / A* / 反应层 哪一段 ----
        _tm = {"pos": 0.0, "plan": 0.0, "step": 0.0}
        for _d in all_drones:
            _orig_get = _d.get_current_position

            def _timed_get(_o=_orig_get):
                _t0 = time.time()
                _r = _o()
                _tm["pos"] += time.time() - _t0
                return _r

            _d.get_current_position = _timed_get
        _orig_best_scale = planner.best_scale

        def _timed_best_scale(*_a, **_kw):
            _t0 = time.time()
            _r = _orig_best_scale(*_a, **_kw)
            _tm["plan"] += time.time() - _t0
            return _r

        planner.best_scale = _timed_best_scale
        _orig_step = planner.step

        def _timed_step(*_a, **_kw):
            _t0 = time.time()
            _r = _orig_step(*_a, **_kw)
            _tm["step"] += time.time() - _t0
            return _r

        planner.step = _timed_step
        sent_sp = {}
        _fail = {}                       # [plan5] 每机连续发送失败计数
        n_demote = 0                     # [plan5] 实际降级到 go_to 的机·次
        prev_drone_pos = None

        while not reached_end and (time.time() - start_navigation_time < max_navigation_time):
            now = time.time()
            dt = now - last_update
            if dt < period:
                time.sleep(min(0.02, max(0.0, period - dt)))
                continue
            dt = max(0.02, min(0.4, dt))
            last_update = now
            tick += 1
            for _k in _tm:
                _tm[_k] = 0.0
            sent_sp = {}

            update_obs_velocities(now)
            planner.update_obstacles(self.dynamic_obstacles, obstacle_velocities,
                                     speed_hint=obstacle_speed)

            current_positions = [drone.get_current_position() for drone in all_drones]
            center = [
                sum(p[0] for p in current_positions) / len(current_positions),
                sum(p[1] for p in current_positions) / len(current_positions),
                sum(p[2] for p in current_positions) / len(current_positions),
            ]
            dist_to_goal = math.hypot(center[0] - goal[0], center[1] - goal[1])

            ach = 0.0
            if prev_center is not None and prev_tick_time is not None:
                dtt = max(1e-6, now - prev_tick_time)
                ach = math.hypot(center[0] - prev_center[0], center[1] - prev_center[1]) / dtt
            prev_center = list(center)
            prev_tick_time = now

            # 实测队形外接半径（在线估计，用于安全半径）
            reff_meas = 0.0
            for q in current_positions:
                reff_meas = max(reff_meas, math.hypot(q[0] - center[0], q[1] - center[1]))

            if dist_to_goal < 0.5:
                print(f"[plan5] formation has reached the end point (dist {dist_to_goal:.2f} m)")
                reached_end = True
                break

            if (now - last_plan_time) >= cfg["replan_period"] or not path:
                new_scale, new_path = planner.best_scale(center[:2], goal[:2], scale)
                last_plan_time = now
                if new_path is not None:
                    scale, path = new_scale, new_path
                else:
                    path = None
                    planner.stats["no_feasible_path"] = planner.stats.get("no_feasible_path", 0) + 1
                    last_plan_time = now + cfg["replan_period"]

            vx, vy, info = planner.step(center[:2], goal[:2], path, scale,
                                        drone_positions=current_positions,
                                        extra_radius=extra_radius,
                                        extra_margin=extra_margin)
            cmd_speed = math.hypot(vx, vy)

            # ---- 参考点：世界系积分 + 与实测中心的间隙上限（不要"位置前视"）----
            ref[0] += vx * dt
            ref[1] += vy * dt
            dx, dy = ref[0] - center[0], ref[1] - center[1]
            gap = math.hypot(dx, dy)
            if gap > cfg["ref_gap_cap"]:
                k = cfg["ref_gap_cap"] / gap
                ref = [center[0] + dx * k, center[1] + dy * k]
            ref[0] = min(nav_bounds[1], max(nav_bounds[0], ref[0]))
            ref[1] = min(nav_bounds[3], max(nav_bounds[2], ref[1]))
            new_positions = self.generate_formation_positions(
                [ref[0], ref[1], cruise_alt], fixed_yaw, scale=scale)

            # ---- 单机护栏：某机自身净空过小 -> 只给它加背离最近柱子的偏移 ----
            pushes = []
            for i, drone in enumerate(all_drones):
                pi = current_positions[i]
                best_d, best_u = float("inf"), (0.0, 0.0)
                for oid, pos in self.dynamic_obstacles.items():
                    dd = math.hypot(pi[0] - pos[0], pi[1] - pos[1]) - r_pillar
                    if dd < best_d:
                        best_d = dd
                        ux, uy = pi[0] - pos[0], pi[1] - pos[1]
                        m = math.hypot(ux, uy)
                        best_u = (ux / m, uy / m) if m > 1e-6 else (0.0, 0.0)
                if best_d < cfg["per_drone_guard"]:
                    mag = min(cfg["per_drone_push"], (cfg["per_drone_guard"] - best_d) * 1.5)
                    pushes.append((best_u[0] * mag, best_u[1] * mag))
                else:
                    pushes.append((0.0, 0.0))
            if any(abs(p[0]) + abs(p[1]) > 1e-9 for p in pushes):
                planner.stats["per_drone_push"] = planner.stats.get("per_drone_push", 0) + 1

            # ---- 下发 ----
            send_ms = 0.0
            if not (motion_mode == "go_to" and (now - last_goto_time) < _GOTO_RESEND):
                threads = []
                send_t0 = time.time()
                for i, drone in enumerate(all_drones):
                    if i >= len(new_positions):
                        continue
                    tgt = new_positions[i]
                    spx = min(nav_bounds[1], max(nav_bounds[0], tgt[0] + pushes[i][0]))
                    spy = min(nav_bounds[3], max(nav_bounds[2], tgt[1] + pushes[i][1]))
                    sp = [spx, spy, tgt[2]]
                    sent_sp[i] = (spx, spy)          # [plan5] 记录本拍真正下发的目标
                    lag = math.hypot(sp[0] - current_positions[i][0],
                                     sp[1] - current_positions[i][1])
                    speed_cmd = min(cfg["max_track_speed"], max(cfg["v_max"], lag / max(0.1, dt)))
                    streamer = streamers.get(drone.drone_id)
                    if motion_mode == "stream" and streamer is not None:
                        try:
                            ok_send = streamer.send_position_command_with_yaw_angle(
                                sp, twist_limit=cfg["v_max"], pose_frame_id="earth",
                                twist_frame_id="earth", yaw_angle=math.radians(fixed_yaw))
                        except Exception as exc:  # noqa: BLE001
                            ok_send = False
                            print(f"[plan5] {drone.drone_id} 直发参考异常 ({exc})")
                        if not ok_send:
                            # [plan5] 一次失败不降级：原来一次失败就永久切 go_to，
                            # 等于飞行中途换控制模式，很可能就是"躲完容易失控"。
                            # 改成连续 3 次失败才降级（并记进 stats/trace）。
                            _fail[d.drone_id] = _fail.get(d.drone_id, 0) + 1
                            if _fail[d.drone_id] >= 3:
                                print(f"[plan5] {drone.drone_id} 直发参考连续 3 次失败 "
                                      f"-> 该机改走 go_to")
                                streamers[drone.drone_id] = None
                                n_demote += 1
                            else:
                                print(f"[plan5] {drone.drone_id} 直发参考失败 "
                                      f"({_fail[d.drone_id]}/3，继续直发)")
                            th = threading.Thread(
                                target=drone.go_to,
                                args=(sp[0], sp[1], sp[2], speed_cmd, YawMode.FIXED_YAW,
                                      fixed_yaw, "earth", False), daemon=True)
                            th.start()
                            threads.append(th)
                        else:
                            drone.is_moving = True
                    else:
                        th = threading.Thread(
                            target=drone.go_to,
                            args=(sp[0], sp[1], sp[2], speed_cmd, YawMode.FIXED_YAW,
                                  fixed_yaw, "earth", False), daemon=True)
                        th.start()
                        threads.append(th)
                        drone.is_moving = True
                if motion_mode == "go_to":
                    last_goto_time = now
                send_ms = 1000.0 * (time.time() - send_t0)
                for th in threads:
                    th.join(timeout=2.0)
                # [plan5] 硬性时限：一拍的发送总耗时超预算 -> 立刻整体降级，绝不允许盲飞
                if send_ms > 1000.0 * _TICK_SEND_BUDGET and motion_mode == "stream":
                    slow_total += 1
                    print(f"[plan5] !! 本拍发送耗时 {send_ms:.0f} ms 超过预算 "
                          f"{1000 * _TICK_SEND_BUDGET:.0f} ms -> 立刻整体降级为 go_to"
                          f"（防止控制链路阻塞期间无人机盲飞）")
                    motion_mode = "go_to"
                    for kk in list(streamers.keys()):
                        streamers[kk] = None

            clear_center = planner.surface_clearance(center[0], center[1])
            clear_drone = planner.min_drone_clearance(current_positions)
            _row_extra = []
            for _i, _p in enumerate(current_positions):
                _sp = sent_sp.get(_i, (_p[0], _p[1]))
                _v = 0.0
                if prev_drone_pos is not None and _i < len(prev_drone_pos):
                    _q = prev_drone_pos[_i]
                    _v = math.sqrt((_p[0] - _q[0]) ** 2 + (_p[1] - _q[1]) ** 2
                                   + (_p[2] - _q[2]) ** 2) / max(1e-6, dt)
                _row_extra += [round(_p[0], 3), round(_p[1], 3), round(_p[2], 3), round(_v, 3),
                               round(math.hypot(_sp[0] - _p[0], _sp[1] - _p[1]), 3),
                               round(planner.surface_clearance(_p[0], _p[1]) - r_pillar, 3)]
            prev_drone_pos = [tuple(_p) for _p in current_positions]
            trace_w.writerow([round(now - start_navigation_time, 3), round(center[0], 3),
                              round(center[1], 3), round(vx, 3), round(vy, 3),
                              round(dist_to_goal, 3),
                              None if clear_drone == float("inf") else round(clear_drone, 3),
                              round(reff_meas, 3), round(scale, 2),
                              len(path) if path else 0, info["mode"],
                              round(cmd_speed, 3), round(ach, 3), round(send_ms, 1),
                              round(_sim_t.get("t") or 0.0, 3),
                              round(_sim_t.get("rtf", 0.0), 3),
                              round(_tm["pos"] * 1000.0, 1),
                              round(_tm["plan"] * 1000.0, 1),
                              round(_tm["step"] * 1000.0, 1),
                              info.get("n_cand", 0), info.get("n_lvl1", 0),
                              info.get("n_lvl2", 0), n_demote] + _row_extra)
            if tick % log_every == 0:
                trace_fh.flush()
            cmd_hist.append(cmd_speed)
            ach_hist.append(ach)
            if len(cmd_hist) > log_every:
                cmd_hist.pop(0)
                ach_hist.pop(0)

            # [plan5] 撞上/几何重叠 -> 立即中止编队
            if _ABORT_ON_CONTACT and clear_drone < 0.05:
                print(f"[plan5] !! 检测到与柱子几何重叠 (最危险那架机净空 {clear_drone:.2f} m) "
                      f"@ t={now - start_navigation_time:.1f}s -> 立即中止编队飞行，进入降落流程")
                aborted = True
                break

            if info["mode"] != prev_mode or tick % log_every == 0:
                n_path = len(path) if path else 0
                mean_cmd = sum(cmd_hist) / max(1, len(cmd_hist))
                mean_ach = sum(ach_hist) / max(1, len(ach_hist))
                eff = 100.0 * mean_ach / mean_cmd if mean_cmd > 1e-6 else 0.0
                print(f"[plan5] t={now - start_navigation_time:6.1f}s "
                      f"center=({center[0]:6.2f},{center[1]:6.2f}) d_goal={dist_to_goal:5.2f} "
                      f"scale={scale:.2f} path={n_path:3d} clear_drone={clear_drone:5.2f} "
                      f"Reff={reff_meas:.2f} v={info['mode']:<11s} side={info['angle']:+6.1f}deg | "
                      f"cmd={mean_cmd:.2f} ach={mean_ach:.2f} ({eff:3.0f}%) send={send_ms:.0f}ms "
                      f"ctl={motion_mode}")
                _rtf = _sim_t.get("rtf", 0.0)
                _n_stream = sum(1 for _v in streamers.values() if _v is not None)
                print(f"[plan5]     RTF={_rtf:.2f}  ach_sim={mean_ach / max(0.05, _rtf):.2f} m/s(仿真时间)"
                      f"  | 每拍 pos={_tm['pos'] * 1000:.0f} plan={_tm['plan'] * 1000:.0f}"
                      f" step={_tm['step'] * 1000:.0f} ms"
                      f"  | 候选 n={info.get('n_cand', 0)}"
                      f" lvl1={info.get('n_lvl1', 0)} lvl2={info.get('n_lvl2', 0)}"
                      f"  | sim_t={_sim_t.get('t') or 0.0:.1f}s"
                      f"  | 直发 {_n_stream}/{len(all_drones)} 架"
                      + ("" if _n_stream == len(all_drones)
                         else "  <- 有无人机在走慢速 go_to 回退路！"))
                prev_mode = info["mode"]
                if (not warn_done) and (now - start_navigation_time) > 6.0 and \
                        mean_cmd > 0.3 * cfg["v_max"] and eff < 35.0:
                    warn_done = True
                    print(f"[plan5] !! 实际速度只有指令的 {eff:.0f}%：控制层没跟上。"
                          f"先把 _MOTION_MODE 改成 \"go_to\" 跑一次对比；"
                          f"并查 `ros2 topic echo --once /drone0/controller/info` 的 control_mode "
                          f"与 `ros2 topic hz /drone0/motion_reference/pose`。")

        phase_t["navigation"] = time.time() - start_navigation_time
        try:
            trace_fh.flush()
            trace_fh.close()
            print(f"[plan5] trace 已写入 {trace_file}（中断也保留）")
            print(f"[plan5] planner stats = {planner.stats}")
            if _clk_ex is not None:                  # [plan5] 收掉 /clock 探针，
                try:                                 # 否则退出时会在后台线程里
                    _clk_ex.shutdown()               # 抛 ExternalShutdownException
                except Exception:                    # noqa: BLE001
                    pass
        except Exception as exc:  # noqa: BLE001
            print(f"[plan5] trace close failed: {exc}")

        _t1 = time.time()
        if not reached_end and not aborted:
            print("[plan5] navigation timeout reached, moving directly to end point.")
            self.move_formation_to_waypoint(
                self.generate_formation_positions(end_wp, fixed_yaw), fixed_yaw)
        if aborted:
            print("[plan5] 因检测到几何重叠而中止：跳过终点编队，直接交给降落流程。")
            phase_t["final_formation"] = 0.0
        else:
            print("[plan5] All drones have reached the end point successfully.")
            print("\n--- FORMATION TRANSITION: Switching to wide formation at end point ---")
            self.using_wide_formation = True
            current_positions = [drone.get_current_position() for drone in all_drones]
            formation_center = [
                sum(p[0] for p in current_positions) / len(current_positions),
                sum(p[1] for p in current_positions) / len(current_positions),
                sum(p[2] for p in current_positions) / len(current_positions),
            ]
            wide_formation_positions = self.generate_formation_positions(formation_center, fixed_yaw)
            self.move_formation_to_waypoint(wide_formation_positions, fixed_yaw)
            phase_t["final_formation"] = time.time() - _t1

        print("[plan5] ================= 分段耗时 =================")
        total = 0.0
        for k, v in phase_t.items():
            total += v
            print(f"[plan5]   {k:<20s} {v:7.1f} s")
        print(f"[plan5]   {'(nav 之外合计)':<20s} {total - phase_t.get('navigation', 0.0):7.1f} s")
        print(f"[plan5]   控制层降级次数 {slow_total}；结束档位 {motion_mode}")
        print("[plan5] ============================================")
    def transition_to_formation(self, drones, formation_positions, fixed_yaw=None):
        """
        Transition drones to specified formation positions
        
        Args:
            drones: List of drone objects to move
            formation_positions: List of positions for each drone
            fixed_yaw: The fixed yaw angle for the drones
        """
        # Ensure formation is facing forward if yaw not specified
        if fixed_yaw is None:
            fixed_yaw = 90.0
            self.formation_orientation = fixed_yaw
        
        active_drones = []
        for i, drone in enumerate(drones):
            if i < len(formation_positions):
                pos = formation_positions[i]
                print(f"Moving {drone.drone_id} to formation position: {pos}")
                drone.is_moving = True
                drone.go_to(pos[0], pos[1], pos[2], 0.7, YawMode.FIXED_YAW, fixed_yaw, "earth", False)
                active_drones.append(drone)
        
        # Wait for all drones to reach their positions
        if active_drones:
            print(f"Waiting for {len(active_drones)} drones to reach formation positions...")
            start_time = time.time()
            max_wait = 30  # Maximum wait time in seconds (reduced from 60)
            
            while time.time() - start_time < max_wait and active_drones:
                for drone in active_drones[:]:
                    reached_position = False
                    try:
                        status = super(SimpleDrone, drone).get_behavior_status()
                        if status is None or status.status == BehaviorStatus.IDLE:
                            reached_position = True
                    except Exception:
                        try:
                            current_pos = drone.position
                            idx = drones.index(drone)
                            if idx < len(formation_positions):
                                target_pos = formation_positions[idx]
                                dist = math.sqrt((current_pos[0]-target_pos[0])**2 + 
                                            (current_pos[1]-target_pos[1])**2 + 
                                            (current_pos[2]-target_pos[2])**2)
                                if dist < 0.3:  # Within 30cm
                                    reached_position = True
                        except Exception as e:
                            print(f"Error checking position for {drone.drone_id}: {e}")
                    
                    if reached_position:
                        print(f"{drone.drone_id} reached formation position.")
                        drone.is_moving = False
                        drone.change_led_colour((0, 0, 255))  # Blue for all drones in formation
                        active_drones.remove(drone)
                
                if active_drones:
                    time.sleep(0.2)   # [plan5]
                    
    def move_formation_to_waypoint(self, formation_positions, fixed_yaw=None):
        """
        Move all drones to the specified formation positions
        
        Args:
            formation_positions: List of positions for each drone
            fixed_yaw: The fixed yaw angle for the drones
        """
        # Ensure formation is facing forward if yaw not specified
        if fixed_yaw is None:
            fixed_yaw = 90.0
            self.formation_orientation = fixed_yaw
            
        all_drones = list(self.drones.values())
        active_drones = []
        
        # Send all drones to their target positions
        for i, drone in enumerate(all_drones):
            if i < len(formation_positions):
                pos = formation_positions[i]
                print(f"Moving {drone.drone_id} to waypoint formation position: {pos}")
                drone.is_moving = True
                drone.go_to(pos[0], pos[1], pos[2], 0.8, YawMode.FIXED_YAW, fixed_yaw, "earth", False)
                active_drones.append(drone)
        
        # Wait for all drones to reach their positions
        if active_drones:
            print(f"Waiting for {len(active_drones)} drones to reach waypoint...")
            start_time = time.time()
            max_wait = 30  # Maximum wait time in seconds (reduced from 60)
            
            while time.time() - start_time < max_wait and active_drones:
                for drone in active_drones[:]:
                    reached_position = False
                    try:
                        status = super(SimpleDrone, drone).get_behavior_status()
                        if status is None or status.status == BehaviorStatus.IDLE:
                            reached_position = True
                    except Exception:
                        try:
                            current_pos = drone.position
                            idx = all_drones.index(drone)
                            if idx < len(formation_positions):
                                target_pos = formation_positions[idx]
                                dist = math.sqrt((current_pos[0]-target_pos[0])**2 + 
                                            (current_pos[1]-target_pos[1])**2 + 
                                            (current_pos[2]-target_pos[2])**2)
                                if dist < 0.3:  # Within 30cm
                                    reached_position = True
                        except Exception as e:
                            print(f"Error checking position for {drone.drone_id}: {e}")
                    
                    if reached_position:
                        print(f"{drone.drone_id} reached waypoint.")
                        drone.is_moving = False
                        drone.change_led_colour((0, 0, 255))  # Blue for all drones
                        active_drones.remove(drone)
                
                if active_drones:
                    time.sleep(0.2)   # [plan5]

    def generate_formation_positions(self, center_position, orientation, scale=1.0):
        """
        Generate pentagon formation positions for all drones based on a center position
        
        Args:
            center_position: The center point of the formation [x, y, z]
            orientation: The orientation of the formation in degrees
            scale: Scale factor to adjust formation size (1.0 = normal size)
            
        Returns:
            List of positions for each drone in the formation
        """
        center_xy = center_position[:2]  # Get only x, y coordinates
        z = center_position[2]  # Get z coordinate
        
        # Base radius for the pentagon
        if getattr(self, "using_wide_formation", True):
            # Wider formation for initial and final positioning
            base_radius = 0.8  # Reduced from 1.2 to make a smaller pentagon
        else:
            # Tighter formation while navigating
            base_radius = 0.5  # Reduced from 0.6 to make a smaller pentagon
            
        # Apply scale factor
        radius = base_radius * scale
        
        # Generate pentagon formation in 2D
        pentagon_positions_2d = []
        
        # Calculate the angle between each point in the pentagon
        angle_step = 2 * math.pi / min(self.num_drones, 5)
        
        # Generate positions for each drone
        for i in range(min(self.num_drones, 5)):
            # Calculate angle for this point, starting from the top (90 degrees)
            angle = i * angle_step - math.pi/2 + math.radians(orientation)
            
            # Calculate position
            x = center_xy[0] + radius * math.cos(angle)
            y = center_xy[1] + radius * math.sin(angle)
            
            pentagon_positions_2d.append([x, y])
        
        # Add z-coordinate to each position
        formation_positions = [pos + [z] for pos in pentagon_positions_2d]
        
        return formation_positions

    def generate_paths_for_all_drones(self, waypoints: List[List[float]]):
        """
        Generate paths for all drones based on waypoints for the formation center
        
        Args:
            waypoints: List of waypoints for the formation center
        """
        # Set paths for all drones
        all_drones = list(self.drones.values())
        
        # Clear existing paths
        for drone in all_drones:
            drone.path = []
        
        # Use the fixed formation orientation established during initial formation
        orientation_deg = getattr(self, "formation_orientation", 90.0)
        
        # For each waypoint, calculate the pentagon formation positions
        for i, wp in enumerate(waypoints):
            # Generate pentagon formation positions for this waypoint
            formation_positions = self.generate_formation_positions(wp, orientation_deg)
            
            # Assign positions to each drone's path
            for j, drone in enumerate(all_drones):
                if j < len(formation_positions):
                    drone.path.append(formation_positions[j])
                else:
                    # If we have more drones than positions, use the last position
                    drone.path.append(formation_positions[-1])
        
        # Print path information
        print(f"Generated paths for {len(all_drones)} drones with {len(waypoints)} waypoints each")
        for i, drone in enumerate(all_drones):
            print(f"Drone {i} path: {len(drone.path)} waypoints")
            
        print("Pentagon formation paths generated successfully.")

    def astar_plan_path(self, start, goal, obstacles, planning_distance=1.5, resolution=0.02, 
                       safety_margin=0.3, bounds=None):
        """A* algorithm for 3D path planning with limited planning distance
        
        Args:
            start: Start position [x, y, z]
            goal: Goal position [x, y, z]
            obstacles: List of obstacles as [x, y, z, radius]
            planning_distance: Maximum planning distance (in meters)
            resolution: Grid resolution for neighbor search
            safety_margin: Safety margin to add around obstacles (in meters)
            bounds: Environment bounds as [xmin, xmax, ymin, ymax, zmin, zmax]
            
        Returns:
            List of waypoints from start to goal, or empty list if no path found
        """
        class Node:
            """Node class for A* algorithm"""
            def __init__(self, position, parent=None):
                self.position = position  # [x, y, z]
                self.parent = parent
                
                self.g = 0  # Cost from start to current node
                self.h = 0  # Heuristic (estimated cost from current to goal)
                self.f = 0  # Total cost (g + h)
            
            def __eq__(self, other):
                """Check if two nodes have the same position"""
                return (abs(self.position[0] - other.position[0]) < 1e-6 and
                        abs(self.position[1] - other.position[1]) < 1e-6 and
                        abs(self.position[2] - other.position[2]) < 1e-6)

            def __hash__(self):
                """Hash function for use in sets and dictionaries"""
                return hash((round(self.position[0], 3), 
                            round(self.position[1], 3), 
                            round(self.position[2], 3)))
        
        # If no bounds are provided, set default bounds
        if bounds is None:
            # Set bounds based on start and goal with some margin
            max_distance = max(planning_distance, math.dist(start, goal))
            bounds = [
                min(start[0], goal[0]) - max_distance,
                max(start[0], goal[0]) + max_distance,
                min(start[1], goal[1]) - max_distance,
                max(start[1], goal[1]) + max_distance,
                min(start[2], goal[2]) - max_distance,
                max(start[2], goal[2]) + max_distance
            ]
        
        # Create simpler movement directions (6-connectivity) for better performance
        movements = []
        
        # 6 basic directions
        movements.append([resolution, 0, 0])   # Right
        movements.append([-resolution, 0, 0])  # Left
        movements.append([0, resolution, 0])   # Forward
        movements.append([0, -resolution, 0])  # Backward
        movements.append([0, 0, resolution])   # Up
        movements.append([0, 0, -resolution])  # Down
        
        # Add 8 diagonal directions in the horizontal plane for better paths
        movements.append([resolution, resolution, 0])    # Right-Forward
        movements.append([resolution, -resolution, 0])   # Right-Backward
        movements.append([-resolution, resolution, 0])   # Left-Forward
        movements.append([-resolution, -resolution, 0])  # Left-Backward
        
        # Function to check if a position is within bounds
        def is_within_bounds(position):
            return (bounds[0] <= position[0] <= bounds[1] and
                    bounds[2] <= position[1] <= bounds[3] and
                    bounds[4] <= position[2] <= bounds[5])
        
        # Function to check if a position collides with any obstacle
        def is_collision_free(position):
            for obs in obstacles:
                ox, oy, oz, radius = obs
                # Check if position is inside obstacle sphere with safety margin
                dist = math.sqrt(
                    (position[0] - ox) ** 2 +
                    (position[1] - oy) ** 2 +
                    (position[2] - oz) ** 2
                )
                if dist <= radius + safety_margin:
                    return False
            return True
        
        # Function to calculate heuristic (Euclidean distance)
        def calculate_h(position):
            return math.sqrt(
                (position[0] - goal[0]) ** 2 +
                (position[1] - goal[1]) ** 2 +
                (position[2] - goal[2]) ** 2
            )
        
        # Check if distance from start to goal is greater than planning_distance
        distance_to_goal = math.sqrt(
            (start[0] - goal[0]) ** 2 +
            (start[1] - goal[1]) ** 2 +
            (start[2] - goal[2]) ** 2
        )
        
        # If goal is within planning distance, plan directly to it
        if distance_to_goal <= planning_distance:
            actual_goal = goal
        else:
            # Create an intermediate goal in the direction of the final goal
            # at distance planning_distance from start
            direction = [
                (goal[0] - start[0]) / distance_to_goal,
                (goal[1] - start[1]) / distance_to_goal,
                (goal[2] - start[2]) / distance_to_goal
            ]
            
            actual_goal = [
                start[0] + direction[0] * planning_distance,
                start[1] + direction[1] * planning_distance,
                start[2] + direction[2] * planning_distance
            ]
            
            print(f"Goal too far. Planning to intermediate point at {actual_goal}")
        
        # Check if we can see the goal directly (line of sight)
        def has_line_of_sight(from_pos, to_pos):
            # Number of checks based on distance
            dist = math.sqrt(
                (from_pos[0] - to_pos[0]) ** 2 +
                (from_pos[1] - to_pos[1]) ** 2 +
                (from_pos[2] - to_pos[2]) ** 2
            )
            
            num_checks = max(10, int(dist / (resolution * 2)))
            
            for i in range(1, num_checks):
                t = i / num_checks
                check_point = [
                    from_pos[0] + t * (to_pos[0] - from_pos[0]),
                    from_pos[1] + t * (to_pos[1] - from_pos[1]),
                    from_pos[2] + t * (to_pos[2] - from_pos[2])
                ]
                
                if not is_collision_free(check_point):
                    return False
            
            return True
        
        # If we have direct line of sight, return direct path
        if has_line_of_sight(start, actual_goal):
            print("Direct line of sight to goal found")
            return [start, actual_goal]
        
        # Create start and goal nodes
        start_node = Node(start)
        goal_node = Node(actual_goal)
        
        # Initialize open and closed sets
        open_set = {}
        closed_set = set()
        
        # Add start node to open set
        start_node_key = hash(start_node)
        open_set[start_node_key] = start_node
        
        # Set initial values for start node
        start_node.g = 0
        start_node.h = calculate_h(start)
        start_node.f = start_node.g + start_node.h
        
        # Main A* loop
        max_iterations = 10000  # Set a reasonable limit
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            # Find node with lowest f value in open set
            current_key = min(open_set, key=lambda k: open_set[k].f)
            current_node = open_set[current_key]
            
            # Check if we've reached the goal (within resolution distance)
            if (math.sqrt((current_node.position[0] - actual_goal[0]) ** 2 +
                         (current_node.position[1] - actual_goal[1]) ** 2 +
                         (current_node.position[2] - actual_goal[2]) ** 2) <= resolution * 2):
                
                # Reconstruct path
                path = []
                while current_node:
                    path.append(current_node.position)
                    current_node = current_node.parent
                
                # Return reversed path (start to goal)
                return path[::-1]
            
            # Move current node from open to closed set
            del open_set[current_key]
            closed_set.add(current_key)
            
            # Every 50 iterations, check if we can see the goal from current node
            if iterations % 50 == 0:
                if has_line_of_sight(current_node.position, actual_goal):
                    # Create a direct path from here
                    path = []
                    while current_node:
                        path.append(current_node.position)
                        current_node = current_node.parent
                    
                    # Add goal node to path and return reversed path
                    path.append(actual_goal)
                    return path[::-1]
            
            # Check all neighboring nodes
            for movement in movements:
                # Calculate new position
                new_position = [
                    current_node.position[0] + movement[0],
                    current_node.position[1] + movement[1],
                    current_node.position[2] + movement[2]
                ]
                
                # Check if new position is valid
                if not is_within_bounds(new_position) or not is_collision_free(new_position):
                    continue
                
                # Create neighbor node
                neighbor = Node(new_position, current_node)
                neighbor_key = hash(neighbor)
                
                # Skip if in closed set
                if neighbor_key in closed_set:
                    continue
                
                # Calculate g score (cost from start to neighbor)
                # Use Euclidean distance as cost
                tentative_g = current_node.g + math.sqrt(
                    movement[0] ** 2 + movement[1] ** 2 + movement[2] ** 2
                )
                
                # Check if we've found a better path to this neighbor
                if neighbor_key in open_set and tentative_g >= open_set[neighbor_key].g:
                    continue
                
                # This is the best path so far, save it
                neighbor.g = tentative_g
                neighbor.h = calculate_h(new_position)
                neighbor.f = neighbor.g + neighbor.h
                
                # Add to open set
                open_set[neighbor_key] = neighbor
        
        print(f"A* search failed after {iterations} iterations")
        
        # Return at least a simple straight-line path toward goal
        mid_point = [
            start[0] + (actual_goal[0] - start[0]) * 0.5,
            start[1] + (actual_goal[1] - start[1]) * 0.5,
            start[2] + (actual_goal[2] - start[2]) * 0.5,
        ]
        
        # Try to move the mid point vertically to avoid obstacles
        for vert_offset in [0.3, 0.5, 0.8, 1.0]:
            adjusted_mid = [mid_point[0], mid_point[1], mid_point[2] + vert_offset]
            if is_collision_free(adjusted_mid):
                mid_point = adjusted_mid
                break
        
        return [start, mid_point, actual_goal]

    # Helper method to check line of sight between two points
    def has_line_of_sight(self, from_pos, to_pos, obstacles, resolution=0.1):
        """Check if there is a clear line of sight between two positions"""
        # Calculate distance between points
        dist = math.sqrt(
            (from_pos[0] - to_pos[0]) ** 2 +
            (from_pos[1] - to_pos[1]) ** 2 +
            (from_pos[2] - to_pos[2]) ** 2
        )
        
        # Number of checks based on distance
        num_checks = max(10, int(dist / resolution))
        
        for i in range(1, num_checks):
            t = i / num_checks
            check_point = [
                from_pos[0] + t * (to_pos[0] - from_pos[0]),
                from_pos[1] + t * (to_pos[1] - from_pos[1]),
                from_pos[2] + t * (to_pos[2] - from_pos[2])
            ]
            
            # Check collision with any obstacle
            for obs in obstacles:
                ox, oy, oz, radius = obs
                # Check if position is inside obstacle sphere
                dist_to_obs = math.sqrt(
                    (check_point[0] - ox) ** 2 +
                    (check_point[1] - oy) ** 2 +
                    (check_point[2] - oz) ** 2
                )
                if dist_to_obs <= radius + 0.3:  # Using fixed safety margin
                    return False
        
        return True

##########################
# Main Fonksiyonu        #
##########################

# [fix4] faulthandler + stall watchdog -------------------------------------------
# 任何一次"卡住"都要能立刻拿到每个线程的 Python 栈，而不是靠猜。
#   * SIGUSR1 -> 把全线程栈追加写到 /tmp/dsh_stack.log（外部可直接 kill -USR1 <pid>）
#   * 看门狗：30s 没有进度就打一次全线程栈（只打印，不杀进程，不会误伤慢但正常的运行）
_PROGRESS = {"beat": 0.0, "where": "startup"}
_STACK_FILE = None


def beat(where: str) -> None:
    """Mark forward progress (used by the stall watchdog)."""
    import time as _time
    _PROGRESS["beat"] = _time.time()
    _PROGRESS["where"] = where


# [fix6] print-anywhere progress heartbeat -------------------------------------
# 看门狗不该靠人工埋点：把 builtins.print 包一层，mission 每打印一行就算一次进度，
# 并且把最后一行当成"当前在哪"的标记。
import builtins as _builtins
import time as _time_mod

if not getattr(_builtins, "_dsh_print_wrapped", False):
    _dsh_real_print = _builtins.print

    def _dsh_print(*args, **kwargs):
        try:
            _PROGRESS["beat"] = _time_mod.time()
            _PROGRESS["where"] = " ".join(str(a) for a in args)[:70]
        except Exception:  # noqa: BLE001
            pass
        return _dsh_real_print(*args, **kwargs)

    _builtins.print = _dsh_print
    _builtins._dsh_print_wrapped = True
# -------------------------------------------------------------------------------

def _install_stall_watchdog() -> None:
    import faulthandler
    import os as _os
    import signal as _signal
    import threading as _threading
    import time as _time

    global _STACK_FILE
    try:
        _STACK_FILE = open("/tmp/dsh_stack.log", "a", buffering=1)
        faulthandler.enable(file=_STACK_FILE, all_threads=True)
        faulthandler.register(_signal.SIGUSR1, file=_STACK_FILE, all_threads=True,
                              chain=False)
        print("[fix] faulthandler armed: kill -USR1 <pid> dumps stacks to /tmp/dsh_stack.log",
              flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"[fix] faulthandler setup failed: {exc}", flush=True)

    beat("startup")

    def watch():
        last_dump = 0.0
        while True:
            _time.sleep(5.0)
            idle = _time.time() - _PROGRESS["beat"]
            if idle > 45.0 and _time.time() - last_dump > 60.0:
                last_dump = _time.time()
                print(f"[fix] no progress heartbeat for {idle:.0f}s "
                      f"(last print: '{_PROGRESS['where']}') - dumping all thread "
                      f"stacks to /tmp/dsh_stack.log", flush=True)
                if _STACK_FILE is not None:
                    try:
                        faulthandler.dump_traceback(_STACK_FILE, all_threads=True)
                    except Exception:  # noqa: BLE001
                        pass

    _threading.Thread(target=watch, name="dsh-stall-watchdog", daemon=True).start()
# -------------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description='Multi-drone formation flight mission with centralized MAPF (Scenario 2)')
    parser.add_argument('-n','--namespaces', nargs='+',
                        default=['drone0','drone1','drone2','drone3','drone4'],
                        help='Drone namespace list')
    parser.add_argument('-s','--use_sim_time', action='store_true', default=True,
                        help='Use simulation time')
    parser.add_argument('--scenario_file', type=str, default='scenario1_stage2.yaml',
                        help='Path to scenario YAML file (for scenario 2)')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    args = parser.parse_args()

    rclpy.init()
    # [fix5] verify the active rmw implementation -------------------------------
    try:
        try:
            import rclpy as _rclpy_mod
            _rmw_now = _rclpy_mod.get_rmw_implementation_identifier()
        except AttributeError:
            from rclpy.utilities import get_rmw_implementation_identifier as _get_rmw
            _rmw_now = _get_rmw()
        print(f"[fix5] active rmw implementation = {_rmw_now}", flush=True)
        if _rmw_now != _RMW_WANTED:
            print(f"[fix5] !! WRONG RMW: this process uses {_rmw_now} while the AS2 stack "
                  f"runs {_RMW_WANTED}; the two only partially interoperate, so every "
                  f"service call will TIMEOUT. Run with "
                  f"RMW_IMPLEMENTATION={_RMW_WANTED} (or source challenge_multi_drone/setup.bash).",
                  flush=True)
    except Exception as _exc:  # noqa: BLE001
        print(f"[fix5] could not query rmw implementation: {_exc}", flush=True)
    # ---------------------------------------------------------------------------
    _install_stall_watchdog()
    swarm = SwarmConductor(args.namespaces, verbose=args.verbose, use_sim_time=args.use_sim_time,
                           scenario_file=args.scenario_file)

    # 2026-10-07 fix: DroneInterface 自带 spin 线程（SingleThreadedExecutor + __auto_spin），
    # 这里再建 MultiThreadedExecutor 去 spin 同一批节点 = 两个 executor 争抢回调，
    # 会让 ServiceHandler 的 Client.call()（无超时）永远收不到响应 -> 卡在 takeoff_all 的 arm()。
    # 因此保留对象但不再重复 spin（等价于 carmendrpr/flocking_behavior 的 mission_swarm.py 做法）。
    executor = MultiThreadedExecutor()
    spin_thread = threading.Thread(target=lambda: None, daemon=True)
    spin_thread.start()

    try:
        print("Starting takeoff sequence...")
        beat("takeoff_all")
        swarm.takeoff_all()
        
        # Determine which scenario to run based on the scenario filename
        scenario_file = args.scenario_file.lower()
        if "stage1" in scenario_file:
            print("Starting Scenario 1 mission...")
            swarm.execute_scenario1_mission()
        elif "stage2" in scenario_file:
            print("Starting Scenario 2 mission with centralized MAPF and pentagon formation...")
            swarm.execute_scenario2_mission()
        elif "stage3" in scenario_file:
            print("Starting Scenario 3 mission...")
            swarm.execute_scenario3_mission()
        elif "stage4" in scenario_file:
            print("Starting Scenario 4 mission...  [plan5 = 双路障碍订阅 + 更早绕行(mapf_scenario_4_5.py)：0.7m/s，safety_margin 0.60 / lookahead 1.90]")
            swarm.execute_scenario4_mission()
        else:
            print(f"Warning: Could not determine scenario type from filename '{args.scenario_file}'")
            print("Please specify a file containing 'stage1', 'stage2', 'stage3' or 'stage4' in the name.")
            print("Starting Scenario 2 mission by default...")
            swarm.execute_scenario2_mission()
        
        print("Landing all drones...")
        beat("land_all")
        swarm.land_all()
    except KeyboardInterrupt:
        print("Operation interrupted by user")
    finally:
        print("Cleaning up...")
        # [fix11] 复位无人机，避免下一轮 takeoff 因脏状态被拒
        beat("reset drones")
        try:
            swarm.reset_all()
        except Exception as e:
            print(f"Error during drone reset: {e}")
        # Close all matplotlib figures before shutdown
        plt.close('all')
        
        # First shutdown the executor (stop spinning)
        try:
            print("Shutting down executor...")
            executor.shutdown()
            print("Waiting for spin thread to complete...")
            spin_thread.join(timeout=2.0)
        except Exception as e:
            print(f"Error during executor shutdown: {e}")
        
        # Then shutdown the swarm interfaces
        try:
            print("Shutting down swarm interfaces...")
            swarm.shutdown()
            time.sleep(1.0)
        except Exception as e:
            print(f"Error during swarm shutdown: {e}")
        
        # Finally shutdown rclpy
        try:
            print("Shutting down rclpy...")
            rclpy.shutdown()
        except Exception as e:
            print(f"Error shutting down rclpy: {e}")
            
        print("ROS shutdown complete")
        sys.exit(0)


if __name__ == '__main__':
    main()






