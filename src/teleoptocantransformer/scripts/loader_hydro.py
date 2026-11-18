#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""装载机液压/关节简易模型（与仿真、CAN 下发同一套）。

倾角计 → 关节角（此处用线性行程–角度，实车再换成几何）。
控制量是比例阀开度：0 关闭；有效带 400~700，对应油缸线速度。
用关节角误差闭环，输出阀开度（不是直接控棱柱关节位置）。
"""

from __future__ import annotations

VALVE_DEAD = 400.0
VALVE_MAX = 700.0
VALVE_SPAN = VALVE_MAX - VALVE_DEAD  # 300

# 油缸行程 (m) 与关节角 (deg)
BOOM_STROKE = (0.05, 0.85)
BOOM_ANGLE_DEG = (5.0, 62.0)
BUCKET_STROKE = (0.05, 0.60)
BUCKET_ANGLE_DEG = (-18.0, 48.0)

# 阀=700 时的油缸线速度 (m/s)
BOOM_VMAX = 0.12
BUCKET_VMAX = 0.10

ANGLE_DEAD_DEG = 0.5
KP_PER_DEG = 0.12  # |误差|×Kp → 0~1，再映射到 400~700
TARGET_JOG_DEG_S = 22.0


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def unit_to_valve(unit_0_1: float) -> float:
    """归一化开度 (0,1] → 400~700；0 → 阀关。"""
    if unit_0_1 <= 0.0:
        return 0.0
    return VALVE_DEAD + VALVE_SPAN * clamp(unit_0_1, 0.0, 1.0)


def valve_to_speed(valve: float, vmax: float) -> float:
    """阀开度 → 油缸线速度。小于 400 不动。"""
    if valve < VALVE_DEAD:
        return 0.0
    u = (clamp(valve, VALVE_DEAD, VALVE_MAX) - VALVE_DEAD) / VALVE_SPAN
    return u * vmax


def lerp_angle(stroke: float, stroke_lim: tuple[float, float], ang_lim: tuple[float, float]) -> float:
    t = (stroke - stroke_lim[0]) / (stroke_lim[1] - stroke_lim[0])
    t = clamp(t, 0.0, 1.0)
    return ang_lim[0] + t * (ang_lim[1] - ang_lim[0])


def angle_to_stroke(angle: float, stroke_lim: tuple[float, float], ang_lim: tuple[float, float]) -> float:
    t = (angle - ang_lim[0]) / (ang_lim[1] - ang_lim[0])
    t = clamp(t, 0.0, 1.0)
    return stroke_lim[0] + t * (stroke_lim[1] - stroke_lim[0])


def angle_error_to_valve(error_deg: float) -> float:
    """关节角误差 → 阀开度 0 或 400~700。正误差=需要伸缸（升）。"""
    if abs(error_deg) < ANGLE_DEAD_DEG:
        return 0.0
    unit = clamp(abs(error_deg) * KP_PER_DEG, 0.0, 1.0)
    mag = unit_to_valve(unit)
    return mag if error_deg > 0.0 else -mag


class HydroPlant:
    """油缸积分 + 倾角/关节角。valve_up/down 为 0 或 400~700。"""

    def __init__(self) -> None:
        self.boom_stroke = angle_to_stroke(22.0, BOOM_STROKE, BOOM_ANGLE_DEG)
        self.bucket_stroke = angle_to_stroke(12.0, BUCKET_STROKE, BUCKET_ANGLE_DEG)
        self.boom_target_deg = 22.0
        self.bucket_target_deg = 12.0

    def boom_angle_deg(self) -> float:
        return lerp_angle(self.boom_stroke, BOOM_STROKE, BOOM_ANGLE_DEG)

    def bucket_angle_deg(self) -> float:
        return lerp_angle(self.bucket_stroke, BUCKET_STROKE, BUCKET_ANGLE_DEG)

    def step(
        self,
        dt: float,
        boom_jog: float,
        bucket_jog: float,
        estop: bool,
    ) -> dict:
        if estop:
            boom_valve = bucket_valve = 0.0
        else:
            self.boom_target_deg = clamp(
                self.boom_target_deg + boom_jog * TARGET_JOG_DEG_S * dt,
                BOOM_ANGLE_DEG[0],
                BOOM_ANGLE_DEG[1],
            )
            self.bucket_target_deg = clamp(
                self.bucket_target_deg + bucket_jog * TARGET_JOG_DEG_S * dt,
                BUCKET_ANGLE_DEG[0],
                BUCKET_ANGLE_DEG[1],
            )
            boom_valve = angle_error_to_valve(self.boom_target_deg - self.boom_angle_deg())
            bucket_valve = angle_error_to_valve(self.bucket_target_deg - self.bucket_angle_deg())

        boom_v = valve_to_speed(abs(boom_valve), BOOM_VMAX) * (1.0 if boom_valve >= 0 else -1.0)
        bucket_v = valve_to_speed(abs(bucket_valve), BUCKET_VMAX) * (1.0 if bucket_valve >= 0 else -1.0)
        self.boom_stroke = clamp(self.boom_stroke + boom_v * dt, *BOOM_STROKE)
        self.bucket_stroke = clamp(self.bucket_stroke + bucket_v * dt, *BUCKET_STROKE)

        boom_up = boom_valve if boom_valve > 0 else 0.0
        boom_down = -boom_valve if boom_valve < 0 else 0.0
        bucket_up = bucket_valve if bucket_valve > 0 else 0.0
        bucket_down = -bucket_valve if bucket_valve < 0 else 0.0
        return {
            'boom_valve_up': boom_up,
            'boom_valve_down': boom_down,
            'bucket_valve_up': bucket_up,
            'bucket_valve_down': bucket_down,
            'boom_stroke_m': self.boom_stroke,
            'bucket_stroke_m': self.bucket_stroke,
            'boom_angle_deg': self.boom_angle_deg(),
            'bucket_angle_deg': self.bucket_angle_deg(),
            'boom_target_deg': self.boom_target_deg,
            'bucket_target_deg': self.bucket_target_deg,
            'boom_cyl_vel': boom_v,
            'bucket_cyl_vel': bucket_v,
        }
