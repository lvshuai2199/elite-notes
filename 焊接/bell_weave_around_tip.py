"""绕导电嘴尖点（TCP Z+ 上一点）的钟形摆动点位计算。

与 Welding/摆动/weave.html 一致：
- 摆动中心 O 在当前名义 TCP 沿工具 Z+ 距离 D 处（导电嘴尖点）
- 焊枪绕焊缝前进轴（过 O）左右摆角 θ
- 姿态与位置联动：工具 Z 始终穿过 O，O 沿焊缝中心线前进

D = 0 时 TCP 就是尖点，只改姿态（定点倾摆）。
D > 0 时 TCP 绕尖点走圆弧，尖点位置不变。

Elite 位姿为轴角 [x,y,z,rx,ry,rz]，位置 m，姿态 rad。
脚本格式与焊接任务一致：movep([x,y,z,rx,ry,rz], a, v, r)，end 后不再调用函数。
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Sequence, Tuple

Pose = List[float]


def _as_pose(values: Sequence[float], size: int = 6) -> Pose:
    pose = [float(v) for v in values]
    if len(pose) < size:
        pose.extend([0.0] * (size - len(pose)))
    return pose[:size]


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def _cross(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _norm(vec: Sequence[float]) -> float:
    return math.sqrt(sum(v * v for v in vec))


def _normalize(vec: Sequence[float]) -> List[float]:
    n = _norm(vec)
    if n < 1e-12:
        return [float("nan")] * len(vec)
    return [v / n for v in vec]


def _mul_mat_vec(r: Sequence[Sequence[float]], v: Sequence[float]) -> List[float]:
    return [sum(r[i][j] * v[j] for j in range(3)) for i in range(3)]


def _mul_mat(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> List[List[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def rotvec_to_matrix(rv: Sequence[float]) -> List[List[float]]:
    angle = _norm(rv)
    if angle < 1e-15:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    kx, ky, kz = rv[0] / angle, rv[1] / angle, rv[2] / angle
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return [
        [t * kx * kx + c, t * kx * ky - s * kz, t * kx * kz + s * ky],
        [t * ky * kx + s * kz, t * ky * ky + c, t * ky * kz - s * kx],
        [t * kz * kx - s * ky, t * kz * ky + s * kx, t * kz * kz + c],
    ]


def matrix_to_rotvec(r: Sequence[Sequence[float]]) -> List[float]:
    cos_theta = max(-1.0, min(1.0, 0.5 * (r[0][0] + r[1][1] + r[2][2] - 1.0)))
    theta = math.acos(cos_theta)
    if theta < 1e-12:
        return [0.0, 0.0, 0.0]
    if abs(theta - math.pi) < 1e-6:
        diag = [r[0][0], r[1][1], r[2][2]]
        axis = [math.sqrt(max(0.0, (diag[i] + 1.0) / 2.0)) for i in range(3)]
        if r[0][1] < 0:
            axis[1] = -axis[1]
        if r[0][2] < 0:
            axis[2] = -axis[2]
        n = _normalize(axis)
        return [n[i] * theta for i in range(3)]
    s = 2.0 * math.sin(theta)
    axis = [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s]
    return [axis[i] * theta for i in range(3)]


def _axis_angle_matrix(axis: Sequence[float], angle: float) -> List[List[float]]:
    return rotvec_to_matrix([axis[i] * angle for i in range(3)])


def _align_rotvec(prev: Sequence[float], curr: Sequence[float]) -> List[float]:
    """选与上一帧最近的轴角表示，避免 ±π 跳变导致 movep 反向。"""
    n = _norm(curr)
    if n < 1e-12:
        return [0.0, 0.0, 0.0]
    axis = [curr[i] / n for i in range(3)]
    best = list(curr)
    best_d = sum((best[i] - prev[i]) ** 2 for i in range(3))
    for k in (-1, 0, 1):
        cand = [axis[i] * (n + 2.0 * math.pi * k) for i in range(3)]
        dist = sum((cand[i] - prev[i]) ** 2 for i in range(3))
        if dist < best_d:
            best, best_d = cand, dist
    return best


def _slerp_rotvec(a: Sequence[float], b: Sequence[float], t: float) -> List[float]:
    if t <= 0.0:
        return list(a)
    if t >= 1.0:
        return list(b)
    ra = rotvec_to_matrix(a)
    rb = rotvec_to_matrix(b)
    rel = _mul_mat(
        [
            [ra[0][0], ra[1][0], ra[2][0]],
            [ra[0][1], ra[1][1], ra[2][1]],
            [ra[0][2], ra[1][2], ra[2][2]],
        ],
        rb,
    )
    rv = matrix_to_rotvec(rel)
    angle = _norm(rv)
    if angle < 1e-12:
        return list(a)
    return matrix_to_rotvec(_mul_mat(ra, _axis_angle_matrix(_normalize(rv), angle * t)))


def _tool_z(rotvec: Sequence[float]) -> List[float]:
    r = rotvec_to_matrix(rotvec)
    return [r[0][2], r[1][2], r[2][2]]


def _lerp_nominal(begin: Pose, end: Pose, ratio: float) -> Pose:
    ratio = max(0.0, min(1.0, float(ratio)))
    pose = [begin[i] + (end[i] - begin[i]) * ratio for i in range(3)]
    pose.extend(_slerp_rotvec(begin[3:], end[3:], ratio))
    return pose


def swing_angle(phase: float, theta_max: float, wave: str) -> float:
    """phase ∈ [0, 1) 对应一个摆动周期。中心为 0，左右为 ±theta_max。"""
    if wave == "triangle":
        # 0 → + → 0 → − → 0
        u = phase % 1.0
        if u < 0.25:
            return 4.0 * u * theta_max
        if u < 0.75:
            return theta_max - 4.0 * (u - 0.25) * theta_max
        return -theta_max + 4.0 * (u - 0.75) * theta_max
    return theta_max * math.sin(2.0 * math.pi * phase)


@dataclass
class BellWeaveSample:
    index: int
    s_m: float
    phase: float
    theta_rad: float
    pose: Pose
    pivot: List[float]


@dataclass
class BellWeaveResult:
    ok: bool
    samples: List[BellWeaveSample] = field(default_factory=list)
    weld_length_m: float = 0.0
    theta_max_rad: float = 0.0
    pivot_d_m: float = 0.0
    tcp_amplitude_pp_m: float = 0.0
    path_length_m: float = 0.0

    @property
    def poses(self) -> List[Pose]:
        return [s.pose for s in self.samples]


def obtain_swing_waypoint_around_tip(
    weave_begin: Sequence[float],
    weave_end: Sequence[float],
    pivot_d_mm: float,
    theta_max_deg: float,
    welding_speed: float,
    frequency: float,
    samples_per_cycle: int = 20,
    wave: str = "sine",
    amplitude_mm: float | None = None,
) -> BellWeaveResult:
    """
    计算绕 TCP Z+ 尖点摆动的 TCP 路点。

    :param weave_begin / weave_end: Elite 轴角位姿 [x,y,z,rx,ry,rz]，m / rad
    :param pivot_d_mm: 尖点相对 TCP 沿工具 Z+ 的距离 D（mm）。TCP 已在导电嘴尖则填 0
    :param theta_max_deg: 相对焊缝中心的最大摆角（单侧，度）
    :param welding_speed: 尖点沿焊缝前进速度 Vw（mm/s）
    :param frequency: 摆动频率 f（Hz），周期 T=1/f，每周期前进 Vw/f
    :param samples_per_cycle: 每周期采样点数，供 movep 交融
    :param wave: sine（钟摆）或 triangle（折线摆）
    :param amplitude_mm: 若给定且 D>0，用 A=2·D·sinθ 反推 θ_max（A 为 TCP 峰峰值，mm）
    """
    begin = _as_pose(weave_begin)
    end = _as_pose(weave_end)
    travel = [end[i] - begin[i] for i in range(3)]
    weld_len = _norm(travel)
    x_axis = _normalize(travel)
    if math.isnan(x_axis[0]) or frequency <= 0.0 or welding_speed <= 0.0 or samples_per_cycle < 2:
        return BellWeaveResult(ok=False)

    d_m = max(0.0, pivot_d_mm) / 1000.0
    if amplitude_mm is not None and d_m > 1e-9:
        half = abs(amplitude_mm) / 2000.0
        ratio = min(1.0, half / d_m)
        theta_max = math.asin(ratio)
    else:
        theta_max = math.radians(theta_max_deg)

    pitch_m = (welding_speed / 1000.0) / frequency
    if pitch_m <= 1e-9:
        return BellWeaveResult(ok=False)

    n_cycle = max(1, int(round(weld_len / pitch_m)))
    n_seg = n_cycle * samples_per_cycle
    result = BellWeaveResult(
        ok=True,
        weld_length_m=weld_len,
        theta_max_rad=theta_max,
        pivot_d_m=d_m,
        tcp_amplitude_pp_m=2.0 * d_m * math.sin(theta_max),
    )

    prev: Pose | None = None
    path = 0.0
    for i in range(n_seg + 1):
        s = weld_len * (i / n_seg)
        ratio = 0.0 if weld_len < 1e-12 else s / weld_len
        nominal = _lerp_nominal(begin, end, ratio)
        phase = (s / pitch_m) % 1.0
        theta = swing_angle(phase, theta_max, wave)
        r_nom = rotvec_to_matrix(nominal[3:])
        z_tool = [r_nom[0][2], r_nom[1][2], r_nom[2][2]]
        pivot = [nominal[j] + d_m * z_tool[j] for j in range(3)]
        r_swing = _axis_angle_matrix(x_axis, theta)
        rel = [nominal[j] - pivot[j] for j in range(3)]
        pos = [pivot[j] + _mul_mat_vec(r_swing, rel)[j] for j in range(3)]
        r_new = _mul_mat(r_swing, r_nom)
        rv = matrix_to_rotvec(r_new)
        if prev is not None:
            rv = _align_rotvec(prev[3:], rv)
        pose = pos + rv
        if prev is not None:
            path += math.sqrt(sum((pose[j] - prev[j]) ** 2 for j in range(3)))
        prev = pose
        result.samples.append(
            BellWeaveSample(i, s, phase, theta, pose, pivot)
        )
    result.path_length_m = path
    if result.samples:
        result.samples[0].pose[:3] = begin[:3]
        result.samples[0].pose[3:] = _align_rotvec(begin[3:], result.samples[0].pose[3:])
        result.samples[-1].pose[:3] = end[:3]
        result.samples[-1].pose[3:] = _align_rotvec(result.samples[-2].pose[3:], end[3:])
        result.samples[0].theta_rad = 0.0
    return result


def write_csv(path: str | Path, result: BellWeaveResult) -> Path:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "index",
                "s_m",
                "phase",
                "theta_deg",
                "x_m",
                "y_m",
                "z_m",
                "rx",
                "ry",
                "rz",
                "pivot_x_m",
                "pivot_y_m",
                "pivot_z_m",
            ]
        )
        for s in result.samples:
            p = s.pose
            o = s.pivot
            w.writerow(
                [
                    s.index,
                    f"{s.s_m:.6f}",
                    f"{s.phase:.4f}",
                    f"{math.degrees(s.theta_rad):.4f}",
                    f"{p[0]:.6f}",
                    f"{p[1]:.6f}",
                    f"{p[2]:.6f}",
                    f"{p[3]:.6f}",
                    f"{p[4]:.6f}",
                    f"{p[5]:.6f}",
                    f"{o[0]:.6f}",
                    f"{o[1]:.6f}",
                    f"{o[2]:.6f}",
                ]
            )
    return out


def _fmt_pose(pose: Sequence[float]) -> str:
    # Elite CS 与焊接任务脚本一致：六元数列表，不用 p[]
    return "[{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}]".format(*pose)


def write_movep_script(
    path: str | Path,
    result: BellWeaveResult,
    acceleration: float = 1.2,
    speed_m_s: float | None = None,
    blend_radius: float | None = None,
    func_name: str = "a",
) -> Path:
    """生成可直接下发的 Elite 脚本：movep([x,y,z,rx,ry,rz], a, v, r)。"""
    if not result.ok or len(result.samples) < 2:
        raise ValueError("摆动计算失败，无法生成脚本")
    if speed_m_s is None:
        speed_m_s = 0.008
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)

    poses = result.poses
    min_step = min(
        math.sqrt(sum((poses[i + 1][j] - poses[i][j]) ** 2 for j in range(3)))
        for i in range(len(poses) - 1)
    )
    if blend_radius is None:
        blend_radius = max(0.0, min(0.001, 0.35 * min_step))

    # 与 sample_movej / 焊接任务脚本一致：1 空格缩进，end 后不再调用函数
    lines = ["def {}():".format(func_name)]
    last = len(poses) - 1
    for i, pose in enumerate(poses):
        r = 0.0 if i == last else blend_radius
        lines.append(
            " movep({}, {:.3f}, {:.6f}, {:.6f})".format(
                _fmt_pose(pose), acceleration, speed_m_s, r
            )
        )
    lines.append("end")
    lines.append("")
    out.write_text("\n".join(lines), encoding="ascii")
    return out


def generate_bell_weave_files(
    weave_begin: Sequence[float],
    weave_end: Sequence[float],
    csv_path: str | Path,
    script_path: str | Path,
    pivot_d_mm: float = 15.0,
    theta_max_deg: float = 12.0,
    welding_speed: float = 8.0,
    frequency: float = 1.0,
    samples_per_cycle: int = 20,
    wave: str = "sine",
    amplitude_mm: float | None = None,
    acceleration: float = 1.2,
    blend_radius: float | None = None,
) -> Tuple[BellWeaveResult, Path, Path]:
    """一次算出点位并写出 csv、Elite movep 脚本。"""
    result = obtain_swing_waypoint_around_tip(
        weave_begin,
        weave_end,
        pivot_d_mm,
        theta_max_deg,
        welding_speed,
        frequency,
        samples_per_cycle,
        wave,
        amplitude_mm,
    )
    if not result.ok:
        raise ValueError("摆动计算失败")
    weld_time = result.weld_length_m / (welding_speed / 1000.0)
    speed_m_s = result.path_length_m / max(weld_time, 1e-9)
    csv_file = write_csv(csv_path, result)
    script_file = write_movep_script(
        script_path,
        result,
        acceleration=acceleration,
        speed_m_s=speed_m_s,
        blend_radius=blend_radius,
    )
    return result, csv_file, script_file


def _parse_pose(text: str) -> Pose:
    parts = [float(x.strip()) for x in text.split(",")]
    return _as_pose(parts)


def main() -> None:
    here = Path(__file__).resolve().parent
    out_dir = here / "out"
    parser = argparse.ArgumentParser(description="绕导电嘴尖点钟形摆动：生成 CSV 和 movep 脚本")
    parser.add_argument("--begin", default="0.5,-0.3,0.58,-0.072,0.139,-1.612", help="起点 p x,y,z,rx,ry,rz")
    parser.add_argument("--end", default="0.5,0.2,0.58,-0.072,0.139,-1.612", help="终点")
    parser.add_argument("--d-mm", type=float, default=15.0, help="尖点沿 TCP Z+ 的距离 mm")
    parser.add_argument("--theta-deg", type=float, default=12.0, help="单侧最大摆角 deg")
    parser.add_argument("--amp-mm", type=float, default=None, help="可选，TCP 峰峰值 A mm，覆盖 theta")
    parser.add_argument("--speed", type=float, default=8.0, help="焊缝前进速度 mm/s")
    parser.add_argument("--freq", type=float, default=1.0, help="摆动频率 Hz")
    parser.add_argument("--spc", type=int, default=20, help="每周期点数")
    parser.add_argument("--wave", choices=("sine", "triangle"), default="triangle")
    parser.add_argument("--out-dir", default=str(out_dir), help="csv 和 script 输出目录")
    parser.add_argument("--csv", default="bell_weave.csv", help="csv 文件名，或绝对路径")
    parser.add_argument("--script", default="bell_weave.script", help="script 文件名，或绝对路径")
    args = parser.parse_args()
    dest = Path(args.out_dir)
    dest.mkdir(parents=True, exist_ok=True)
    csv_path = Path(args.csv)
    script_path = Path(args.script)
    if not csv_path.is_absolute():
        csv_path = dest / csv_path
    if not script_path.is_absolute():
        script_path = dest / script_path

    result, csv_file, script_file = generate_bell_weave_files(
        _parse_pose(args.begin),
        _parse_pose(args.end),
        csv_path,
        script_path,
        pivot_d_mm=args.d_mm,
        theta_max_deg=args.theta_deg,
        welding_speed=args.speed,
        frequency=args.freq,
        samples_per_cycle=args.spc,
        wave=args.wave,
        amplitude_mm=args.amp_mm,
    )
    print(
        "ok points={} weld={:.1f}mm D={:.1f}mm theta={:.2f}deg A_tcp={:.2f}mm".format(
            len(result.samples),
            result.weld_length_m * 1000.0,
            result.pivot_d_m * 1000.0,
            math.degrees(result.theta_max_rad),
            result.tcp_amplitude_pp_m * 1000.0,
        )
    )
    print("csv", csv_file)
    print("script", script_file)


if __name__ == "__main__":
    main()
