"""校验钟形摆动的旋转中心 O 是否共线。

读 bell_weave.csv（pivot_x/y/z，以及 x/y/z + rx/ry/rz），
用首尾 O 连成直线，量每个 O 的垂距；再用 TCP 沿工具 Z+ 走 D，看是否回到 O。
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence

from bell_weave_around_tip import _norm, _normalize, _cross, _tool_z


@dataclass
class PivotLineCheck:
    line_length_mm: float
    max_dev_mm: float
    rms_dev_mm: float
    max_z_through_o_err_mm: float
    deviations_mm: List[float]
    z_through_o_err_mm: List[float]

    @property
    def colinear(self) -> bool:
        return self.max_dev_mm < 0.05


def _point_line_distance(point: Sequence[float], origin: Sequence[float], axis: Sequence[float]) -> float:
    rel = [point[i] - origin[i] for i in range(3)]
    return _norm(_cross(rel, axis))


def check_points(pivots: Sequence[Sequence[float]], poses: Sequence[Sequence[float]], d_mm: float) -> PivotLineCheck:
    empty = PivotLineCheck(0.0, float("nan"), float("nan"), float("nan"), [], [])
    if len(pivots) < 2:
        return empty
    p0 = pivots[0]
    p1 = pivots[-1]
    axis = _normalize([p1[i] - p0[i] for i in range(3)])
    line_len = _norm([p1[i] - p0[i] for i in range(3)])
    if math.isnan(axis[0]):
        return empty

    d_m = d_mm / 1000.0
    deviations = []
    z_errs = []
    for pivot, pose in zip(pivots, poses):
        deviations.append(_point_line_distance(pivot, p0, axis) * 1000.0)
        z_tool = _tool_z(pose[3:])
        o_from_tcp = [pose[j] + d_m * z_tool[j] for j in range(3)]
        z_errs.append(_norm([o_from_tcp[j] - pivot[j] for j in range(3)]) * 1000.0)

    n = len(deviations)
    rms = math.sqrt(sum(v * v for v in deviations) / n)
    return PivotLineCheck(
        line_length_mm=line_len * 1000.0,
        max_dev_mm=max(deviations),
        rms_dev_mm=rms,
        max_z_through_o_err_mm=max(z_errs),
        deviations_mm=deviations,
        z_through_o_err_mm=z_errs,
    )


def load_csv(path: Path) -> tuple[list[list[float]], list[list[float]]]:
    pivots: list[list[float]] = []
    poses: list[list[float]] = []
    with path.open(encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            pivots.append(
                [float(row["pivot_x_m"]), float(row["pivot_y_m"]), float(row["pivot_z_m"])]
            )
            poses.append(
                [
                    float(row["x_m"]),
                    float(row["y_m"]),
                    float(row["z_m"]),
                    float(row["rx"]),
                    float(row["ry"]),
                    float(row["rz"]),
                ]
            )
    return pivots, poses


def write_check_csv(path: Path, chk: PivotLineCheck) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["index", "pivot_line_dev_mm", "z_through_o_err_mm"])
        for i, (dev, zerr) in enumerate(zip(chk.deviations_mm, chk.z_through_o_err_mm)):
            w.writerow([i, f"{dev:.4f}", f"{zerr:.4f}"])


def write_pivot_script(
    path: Path,
    pivots: Sequence[Sequence[float]],
    poses: Sequence[Sequence[float]],
    acceleration: float = 1.2,
    speed_m_s: float = 0.05,
    blend_radius: float = 0.001,
) -> Path:
    """TCP 走到每个旋转中心 O，姿态用对应摆动姿态，用于看 O 是否走直线。"""
    if len(pivots) < 2:
        raise ValueError("点数不足，无法生成脚本")
    path.parent.mkdir(parents=True, exist_ok=True)
    last = len(pivots) - 1
    lines = ["def a():"]
    for i, (pivot, pose) in enumerate(zip(pivots, poses)):
        r = 0.0 if i == last else blend_radius
        p = [
            pivot[0],
            pivot[1],
            pivot[2],
            pose[3],
            pose[4],
            pose[5],
        ]
        lines.append(
            " movep([{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}], {:.3f}, {:.6f}, {:.6f})".format(
                *p, acceleration, speed_m_s, r
            )
        )
    lines.append("end")
    lines.append("")
    path.write_text("\n".join(lines), encoding="ascii")
    return path


def main() -> None:
    here = Path(__file__).resolve().parent
    out_dir = here / "out"
    parser = argparse.ArgumentParser(description="校验旋转中心 O 是否共线")
    parser.add_argument("--csv", default=str(out_dir / "bell_weave.csv"))
    parser.add_argument("--d-mm", type=float, default=15.0, help="尖点沿 TCP Z+ 的距离，与生成时一致")
    parser.add_argument("--out-dir", default=str(out_dir))
    parser.add_argument("--out", default="pivot_line_check.csv")
    parser.add_argument("--script", default="pivot_line_check.script")
    args = parser.parse_args()

    dest = Path(args.out_dir)
    dest.mkdir(parents=True, exist_ok=True)
    csv_out = Path(args.out)
    script_out = Path(args.script)
    if not csv_out.is_absolute():
        csv_out = dest / csv_out
    if not script_out.is_absolute():
        script_out = dest / script_out

    src = Path(args.csv)
    pivots, poses = load_csv(src)
    chk = check_points(pivots, poses, args.d_mm)
    write_check_csv(csv_out, chk)
    write_pivot_script(script_out, pivots, poses)
    print(
        "pivot line: length={:.2f}mm  max_dev={:.4f}mm  rms={:.4f}mm  colinear={}".format(
            chk.line_length_mm, chk.max_dev_mm, chk.rms_dev_mm, chk.colinear
        )
    )
    print("tool Z through O: max_err={:.4f}mm".format(chk.max_z_through_o_err_mm))
    print("csv", csv_out)
    print("script", script_out)


if __name__ == "__main__":
    main()
