"""直线月牙摆焊：焊接平面、姿态过渡与摆动点位计算。"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

Pose = List[float]
PoseList = List[Pose]
CalcResult = Tuple[bool, PoseList]


def _as_pose(values: Sequence[float], size: int = 6) -> Pose:
    pose = [float(v) for v in values]
    if len(pose) < size:
        pose.extend([0.0] * (size - len(pose)))
    return pose[:size]


def is_nan_vector(values: Sequence[float]) -> bool:
    return any(math.isnan(v) for v in values)


def pose_distance(begin: Sequence[float], end: Sequence[float]) -> float:
    return math.sqrt(sum((end[i] - begin[i]) ** 2 for i in range(3)))


def get_number_of_periods(
    weave_begin: Sequence[float], weave_end: Sequence[float], step: float
) -> int:
    """焊缝长度按步进切分得到的周期点数。"""
    if step <= 0.0:
        return 0
    return int(pose_distance(weave_begin, weave_end) / step)


def _normalize(vec: Sequence[float]) -> List[float]:
    n = math.sqrt(sum(v * v for v in vec))
    if n < 1e-12:
        return [float("nan")] * len(vec)
    return [v / n for v in vec]


def _cross(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def rpy_to_rotation(rx: float, ry: float, rz: float) -> List[List[float]]:
    """RxRyRz，ZYX 欧拉角（与常见机器人姿态约定一致）。"""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ]


def get_welding_plane(
    weave_begin: Sequence[float], weave_end: Sequence[float], angle: float
) -> List[float]:
    """
    构造焊缝坐标系 4x4（行优先 16 元）：
    原点在起点，X 沿焊缝，Y 为摆动方向，Z 由 X×Y 得到。
    angle 为绕焊缝 X 的摆动面倾角（度）。
    """
    begin = _as_pose(weave_begin)
    end = _as_pose(weave_end)
    x_axis = _normalize([end[i] - begin[i] for i in range(3)])
    if is_nan_vector(x_axis):
        return [float("nan")] * 16

    ref = [0.0, 0.0, 1.0]
    if abs(_dot(x_axis, ref)) > 0.99:
        ref = [0.0, 1.0, 0.0]

    y_axis = _normalize(_cross(ref, x_axis))
    if is_nan_vector(y_axis):
        return [float("nan")] * 16
    z_axis = _normalize(_cross(x_axis, y_axis))
    y_axis = _normalize(_cross(z_axis, x_axis))

    rad = math.radians(angle)
    ca, sa = math.cos(rad), math.sin(rad)
    y_rot = [ca * y_axis[i] + sa * z_axis[i] for i in range(3)]
    z_rot = [-sa * y_axis[i] + ca * z_axis[i] for i in range(3)]

    origin = begin[:3]
    return [
        x_axis[0], y_rot[0], z_rot[0], origin[0],
        x_axis[1], y_rot[1], z_rot[1], origin[1],
        x_axis[2], y_rot[2], z_rot[2], origin[2],
        0.0, 0.0, 0.0, 1.0,
    ]


def calculate_weld_pose(welding_plan: Sequence[float], local_point: Sequence[float]) -> Pose:
    """焊缝平面局部点变换到基座标。"""
    px, py, pz = local_point[0], local_point[1], local_point[2]
    t = welding_plan
    world = [
        t[0] * px + t[1] * py + t[2] * pz + t[3],
        t[4] * px + t[5] * py + t[6] * pz + t[7],
        t[8] * px + t[9] * py + t[10] * pz + t[11],
    ]
    result = _as_pose(local_point)
    result[0], result[1], result[2] = world
    return result


def modify_posture(weave_begin: Sequence[float], point: Pose) -> None:
    """先把姿态对齐到焊缝起点。"""
    begin = _as_pose(weave_begin)
    point[3], point[4], point[5] = begin[3], begin[4], begin[5]


def attitude_assignment(
    weave_begin: Sequence[float],
    weave_end: Sequence[float],
    point: Pose,
    ratio: float,
) -> None:
    """沿焊缝进度线性过渡 Rx/Ry/Rz。"""
    begin = _as_pose(weave_begin)
    end = _as_pose(weave_end)
    ratio = max(0.0, min(1.0, float(ratio)))
    for i in range(3, 6):
        point[i] = begin[i] + (end[i] - begin[i]) * ratio


def _lerp_pose(begin: Sequence[float], end: Sequence[float], ratio: float) -> Pose:
    pose = _as_pose(begin)
    attitude_assignment(begin, end, pose, ratio)
    for i in range(3):
        pose[i] = begin[i] + (end[i] - begin[i]) * ratio
    return pose


def _rodrigues(axis: Sequence[float], angle: float, vec: Sequence[float]) -> List[float]:
    ax = _normalize(axis)
    if is_nan_vector(ax):
        return list(vec)
    c, s = math.cos(angle), math.sin(angle)
    cross = _cross(ax, vec)
    dot = _dot(ax, vec)
    return [
        vec[i] * c + cross[i] * s + ax[i] * dot * (1.0 - c) for i in range(3)
    ]


def _orthonormal_frame(x_axis: Sequence[float], y_hint: Sequence[float]) -> Tuple[List[float], List[float], List[float]]:
    x_axis = _normalize(x_axis)
    z_axis = _normalize(_cross(x_axis, y_hint))
    if is_nan_vector(z_axis):
        z_axis = _normalize(_cross(x_axis, [0.0, 0.0, 1.0]))
        if is_nan_vector(z_axis):
            z_axis = _normalize(_cross(x_axis, [0.0, 1.0, 0.0]))
    y_axis = _normalize(_cross(z_axis, x_axis))
    return x_axis, y_axis, z_axis


def plane_axes(weave_begin: Sequence[float], weave_end: Sequence[float], angle: float) -> Tuple[List[float], List[float], List[float]]:
    """取出焊缝坐标系的 X/Y/Z。"""
    t = get_welding_plane(weave_begin, weave_end, angle)
    return [t[0], t[4], t[8]], [t[1], t[5], t[9]], [t[2], t[6], t[10]]


def _iter_crescent_local(
    path_length: float,
    step: float,
    amplitude_m: float,
    depth_distance: float,
    direction: int,
) -> List[Tuple[float, float]]:
    """展开焊缝上的月牙局部点 (s, y)，含起点和终点。"""
    points: List[Tuple[float, float]] = [(0.0, 0.0)]
    if step <= 0.0 or path_length <= 0.0:
        points.append((path_length, 0.0))
        return points

    total_points = int(path_length / step) // 2
    for i in range(1, total_points):
        is_first_circle = (i % 2) != 0
        s1 = (2 * i - 1) * step
        pose_y = amplitude_m if is_first_circle else -amplitude_m
        y1 = pose_y if direction else -pose_y
        s3 = (2 * i + 1) * step
        y3 = -y1
        mid_s = 0.5 * (s1 + s3)
        mid_y = 0.5 * (y1 + y3)
        dx = s3 - s1
        if abs(dx) < 1e-12:
            continue
        slope = (y3 - y1) / dx
        if abs(slope) < 1e-12:
            s2, y2 = mid_s, mid_y + depth_distance
        else:
            slope_n = -1.0 / slope
            denom = math.sqrt(slope_n * slope_n + 1.0)
            s2 = mid_s + depth_distance / denom
            y2 = slope_n * s2 - slope_n * mid_s + mid_y
        if s1 < path_length:
            points.append((s1, y1))
        if s2 < path_length:
            points.append((s2, y2))
    points.append((path_length, 0.0))
    return points


class _BentPolyline:
    """两条直线过示教拐点；拐点附近裁掉伸进另一段焊缝的摆幅。"""

    def __init__(
        self,
        p0: Sequence[float],
        p1: Sequence[float],
        p2: Sequence[float],
        y1: Sequence[float],
        y2: Sequence[float],
        blend_m: float,
        clearance_m: float,
    ) -> None:
        self.p0 = _as_pose(p0)
        self.p1 = _as_pose(p1)
        self.p2 = _as_pose(p2)
        self.u1 = _normalize([self.p1[i] - self.p0[i] for i in range(3)])
        self.u2 = _normalize([self.p2[i] - self.p1[i] for i in range(3)])
        self.l1 = pose_distance(self.p0, self.p1)
        self.l2 = pose_distance(self.p1, self.p2)
        self.length = self.l1 + self.l2
        self.y1 = list(y1)
        self.y2 = list(y2)
        self.axis = _normalize(_cross(self.u1, self.u2))
        if is_nan_vector(self.axis):
            self.axis = [0.0, 0.0, 1.0]
            self.turn = 0.0
        else:
            self.turn = math.acos(max(-1.0, min(1.0, _dot(self.u1, self.u2))))
        max_blend = 0.4 * min(self.l1, self.l2)
        self.blend = max(0.0, min(blend_m, max_blend))
        self.clearance = max(0.0, clearance_m)

    def _y_axis(self, s: float) -> List[float]:
        if self.blend <= 1e-9 or self.turn < math.radians(1.0):
            return self.y1 if s <= self.l1 else self.y2
        t = (s - (self.l1 - self.blend)) / (2.0 * self.blend)
        t = max(0.0, min(1.0, t))
        return _rodrigues(self.axis, t * self.turn, self.y1)

    def _clip_y(
        self, s: float, origin: Sequence[float], y_axis: Sequence[float], y: float
    ) -> float:
        """拐点附近禁止摆进另一段焊缝，避免和上一段对应位置叠焊。"""
        along = (self.l1 - s) if s <= self.l1 else (s - self.l1)
        if along > self.clearance + abs(y):
            return y

        def offset_dot(yy: float, axis: Sequence[float], sign: float) -> float:
            rel = [
                origin[i] + y_axis[i] * yy - self.p1[i] for i in range(3)
            ]
            return sign * _dot(rel, axis)

        if s <= self.l1:
            pen_axis, sign = self.u2, 1.0
        else:
            pen_axis, sign = self.u1, -1.0
        pen = offset_dot(y, pen_axis, sign)
        if pen <= 1e-9:
            return y
        denom = sign * _dot(y_axis, pen_axis)
        if abs(denom) < 1e-12:
            return 0.0
        y_clip = y - pen / denom
        if y >= 0.0:
            return max(0.0, min(y, y_clip))
        return min(0.0, max(y, y_clip))

    def sample(self, s: float, y: float) -> Pose:
        s = max(0.0, min(self.length, s))
        if s <= self.l1:
            tangent = self.u1
            ratio = 0.0 if self.l1 < 1e-12 else s / self.l1
            pose = _lerp_pose(self.p0, self.p1, ratio)
            origin = [self.p0[i] + self.u1[i] * s for i in range(3)]
        else:
            s2 = s - self.l1
            tangent = self.u2
            ratio = 0.0 if self.l2 < 1e-12 else s2 / self.l2
            pose = _lerp_pose(self.p1, self.p2, ratio)
            origin = [self.p1[i] + self.u2[i] * s2 for i in range(3)]
        _, y_axis, _ = _orthonormal_frame(tangent, self._y_axis(s))
        y = self._clip_y(s, origin, y_axis, y)
        pose[0] = origin[0] + y_axis[0] * y
        pose[1] = origin[1] + y_axis[1] * y
        pose[2] = origin[2] + y_axis[2] * y
        return pose


def obtain_swing_waypoint_crescent_moon(
    weave_begin: Sequence[float],
    weave_end: Sequence[float],
    angle: float,
    welding_speed: float,
    frequency: float,
    hold_distance: float,
    amplitude: float,
    direction: int,
) -> CalcResult:
    """
    直线月牙摆焊点位。

    参数与 C++ LinearOscillation::obtainSwingWaypointCrescentMoon 对齐：
    - weave_begin / weave_end: 6 维位姿 [x, y, z, rx, ry, rz]，位置单位 m
    - angle: 摆动面倾角，度
    - welding_speed: 前进速度，mm/s
    - frequency: 摆动频率，Hz
    - hold_distance: 原接口保留，本算法未参与几何计算
    - amplitude: 摆幅（峰峰值），mm
    - direction: 非 0 为正摆，0 为反向

    返回 (成功标志, 路径点列表)。失败时为 (False, [])。
    """
    del hold_distance  # 与原函数签名一致，月牙几何未使用停留距离

    begin = _as_pose(weave_begin)
    end = _as_pose(weave_end)
    weave_path: PoseList = [begin[:]]

    welding_plan = get_welding_plane(begin, end, angle)
    if is_nan_vector(welding_plan):
        return False, []

    if frequency == 0.0:
        return False, []

    step = (welding_speed / frequency) / 1200.0
    arc_depth = 0.9
    amount_of_weave_period = get_number_of_periods(begin, end, step)
    amplitude_m = amplitude / 2.0 / 1000.0
    depth_distance = 0.8 * arc_depth * step
    total_points = amount_of_weave_period // 2

    # 月牙摆焊的点位数量是频次的一半
    for i in range(1, total_points):
        first_point = [0.0] * 6
        second_point = [0.0] * 6
        third_point = [0.0] * 6

        is_first_circle = (i % 2) != 0
        first_point[0] = (2 * i - 1) * step
        pose_y = amplitude_m if is_first_circle else -amplitude_m
        first_point[1] = pose_y if direction else -pose_y

        # 第三点仅用于求圆弧中点，不写入路径
        third_point[0] = (2 * i + 1) * step
        third_point[1] = -first_point[1]

        middle_x = 0.5 * (third_point[0] + first_point[0])
        middle_y = 0.5 * (third_point[1] + first_point[1])

        dx = third_point[0] - first_point[0]
        if abs(dx) < 1e-12:
            continue
        slope_first_and_third = (third_point[1] - first_point[1]) / dx
        if abs(slope_first_and_third) < 1e-12:
            slope_second_and_middle = float("inf")
            second_point[0] = middle_x
            second_point[1] = middle_y + depth_distance
        else:
            slope_second_and_middle = -1.0 / slope_first_and_third
            denom = math.sqrt(slope_second_and_middle ** 2 + 1.0)
            second_point[0] = middle_x + depth_distance / denom
            second_point[1] = (
                slope_second_and_middle * second_point[0]
                - slope_second_and_middle * middle_x
                + middle_y
            )

        first_point = calculate_weld_pose(welding_plan, first_point)
        modify_posture(begin, first_point)
        second_point = calculate_weld_pose(welding_plan, second_point)
        modify_posture(begin, second_point)

        ratio = float(i) / float(total_points)
        attitude_assignment(begin, end, first_point, ratio)
        attitude_assignment(begin, end, second_point, ratio)

        weave_path.append(first_point)
        weave_path.append(second_point)

    weave_path.append(end[:])
    return True, weave_path


def obtain_swing_waypoint_crescent_moon_bent(
    weave_begin: Sequence[float],
    weave_via: Sequence[float],
    weave_end: Sequence[float],
    angle: float,
    welding_speed: float,
    frequency: float,
    hold_distance: float,
    amplitude: float,
    direction: int,
    transition_radius: float | None = None,
) -> CalcResult:
    """
    三个示教点折成两条直线时的月牙摆焊。

    中心线走 P1→P2→P3，过示教拐点；摆动相位沿折线弧长连续。
    拐点附近会裁掉伸进另一段的摆幅，避免和上一段焊缝叠轨迹。
    transition_radius 是摆动平面过渡长度（mm）。
    默认用 hold_distance，若为 0 则用 1 mm。
    """
    p0 = _as_pose(weave_begin)
    p1 = _as_pose(weave_via)
    p2 = _as_pose(weave_end)

    if frequency == 0.0:
        return False, []
    _, y1, _ = plane_axes(p0, p1, angle)
    _, y2, _ = plane_axes(p1, p2, angle)
    if is_nan_vector(y1) or is_nan_vector(y2):
        return False, []

    if transition_radius is None:
        transition_radius = hold_distance if hold_distance > 0.0 else 1.0
    blend_m = transition_radius / 1000.0
    hold_m = max(0.0, hold_distance) / 1000.0
    step = (welding_speed / frequency) / 1200.0
    amplitude_m = amplitude / 2.0 / 1000.0
    clearance_m = max(2.0 * amplitude_m, blend_m, hold_m, 2.0 * step)
    path = _BentPolyline(p0, p1, p2, y1, y2, blend_m, clearance_m)
    depth_distance = 0.8 * 0.9 * step
    locals_xy = _iter_crescent_local(
        path.length, step, amplitude_m, depth_distance, direction
    )
    # 拐点必须采样，否则相邻摆动点的连线会抄近路，看起来离 P2 很远
    locals_xy.append((path.l1, 0.0))
    locals_xy = sorted(locals_xy, key=lambda item: item[0])
    deduped: List[Tuple[float, float]] = []
    for s, y in locals_xy:
        if deduped and abs(s - deduped[-1][0]) < 1e-9:
            deduped[-1] = (s, y)
        else:
            deduped.append((s, y))
    locals_xy = deduped

    weave_path: PoseList = []
    for s, y in locals_xy:
        weave_path.append(path.sample(s, y))
    if weave_path:
        weave_path[0] = p0[:]
        weave_path[-1] = p2[:]
    return True, weave_path


if __name__ == "__main__":
    print("=== 单段直线 ===")
    ok, points = obtain_swing_waypoint_crescent_moon(
        weave_begin=[0.0, 0.0, 0.30, math.pi, 0.0, 0.0],
        weave_end=[0.20, 0.0, 0.30, math.pi, 0.0, 0.1],
        angle=0.0,
        welding_speed=6.0,
        frequency=1.0,
        hold_distance=0.0,
        amplitude=4.0,
        direction=1,
    )
    print(f"ok={ok}, count={len(points)}")

    print("\n=== 三点两段折线（过拐点，摆动平面过渡） ===")
    ok2, points2 = obtain_swing_waypoint_crescent_moon_bent(
        weave_begin=[0.0, 0.0, 0.30, math.pi, 0.0, 0.0],
        weave_via=[0.15, 0.0, 0.30, math.pi, 0.0, 0.05],
        weave_end=[0.15, 0.12, 0.30, math.pi, 0.0, 0.10],
        angle=0.0,
        welding_speed=6.0,
        frequency=1.0,
        hold_distance=0.0,
        amplitude=4.0,
        direction=1,
        transition_radius=1.0,
    )
    print(f"ok={ok2}, count={len(points2)}")
    for idx, p in enumerate(points2):
        print(
            f"{idx:3d}  pos=({p[0]:8.5f}, {p[1]:8.5f}, {p[2]:8.5f})  "
            f"rpy=({p[3]:7.4f}, {p[4]:7.4f}, {p[5]:7.4f})"
        )
