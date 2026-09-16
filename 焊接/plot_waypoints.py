"""绘制三点折线月牙摆焊点位俯视图（仅用标准库，不依赖 matplotlib）。"""

from __future__ import annotations

import math
import struct
import zlib
from pathlib import Path

from linear_oscillation import obtain_swing_waypoint_crescent_moon_bent

WHITE = (248, 250, 252)
GRID = (226, 232, 240)
TEACH = (148, 163, 184)
PATH = (29, 78, 216)
DOT = (37, 99, 235)
NEAR = (180, 83, 9)
START = (21, 128, 61)
CORNER = (217, 119, 6)
END = (185, 28, 28)


def mm(values):
    return [v * 1000.0 for v in values]


def _save_png(path: Path, width: int, height: int, rgb: bytearray) -> None:
    def chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    raw = b"".join(
        b"\x00" + bytes(rgb[y * width * 3 : (y + 1) * width * 3]) for y in range(height)
    )
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )
    path.write_bytes(png)


def _put(rgb, w, h, x, y, color, clip=None) -> None:
    if clip is not None:
        x0, y0, x1, y1 = clip
        if not (x0 <= x < x1 and y0 <= y < y1):
            return
    if 0 <= x < w and 0 <= y < h:
        i = (y * w + x) * 3
        rgb[i], rgb[i + 1], rgb[i + 2] = color


def _fill_rect(rgb, w, h, box, color) -> None:
    x0, y0, x1, y1 = box
    for y in range(max(0, y0), min(h, y1)):
        for x in range(max(0, x0), min(w, x1)):
            _put(rgb, w, h, x, y, color)


def _cohen_sutherland(x0, y0, x1, y1, box):
    xmin, ymin, xmax, ymax = box[0], box[1], box[2] - 1, box[3] - 1

    def code(x, y):
        c = 0
        if x < xmin:
            c |= 1
        elif x > xmax:
            c |= 2
        if y < ymin:
            c |= 4
        elif y > ymax:
            c |= 8
        return c

    c0, c1 = code(x0, y0), code(x1, y1)
    while True:
        if c0 | c1 == 0:
            return True, int(x0), int(y0), int(x1), int(y1)
        if c0 & c1:
            return False, 0, 0, 0, 0
        c = c0 or c1
        if c & 1:
            y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0 + 1e-12)
            x = xmin
        elif c & 2:
            y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0 + 1e-12)
            x = xmax
        elif c & 4:
            x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0 + 1e-12)
            y = ymin
        else:
            x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0 + 1e-12)
            y = ymax
        if c == c0:
            x0, y0, c0 = x, y, code(x, y)
        else:
            x1, y1, c1 = x, y, code(x, y)


def _line(rgb, w, h, x0, y0, x1, y1, color, dash=0, clip=None) -> None:
    if clip is not None:
        ok, x0, y0, x1, y1 = _cohen_sutherland(x0, y0, x1, y1, clip)
        if not ok:
            return
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err, n = dx - dy, 0
    while True:
        if dash == 0 or (n // dash) % 2 == 0:
            _put(rgb, w, h, x0, y0, color, clip)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
        n += 1


def _disk(rgb, w, h, cx, cy, r, color, clip=None) -> None:
    for yy in range(-r, r + 1):
        for xx in range(-r, r + 1):
            if xx * xx + yy * yy <= r * r:
                _put(rgb, w, h, cx + xx, cy + yy, color, clip)


def _in_view(x, y, xlim, ylim, pad=0.5) -> bool:
    return xlim[0] - pad <= x <= xlim[1] + pad and ylim[0] - pad <= y <= ylim[1] + pad


def _draw_panel(
    rgb,
    w,
    h,
    box,
    xs,
    ys,
    teach,
    near,
    xlim,
    ylim,
) -> None:
    x0, y0, x1, y1 = box
    pw, ph = x1 - x0, y1 - y0
    _fill_rect(rgb, w, h, box, WHITE)

    def to_px(x: float, y: float) -> tuple[int, int]:
        px = x0 + int((x - xlim[0]) / (xlim[1] - xlim[0]) * (pw - 1))
        py = y0 + int((ylim[1] - y) / (ylim[1] - ylim[0]) * (ph - 1))
        return px, py

    for gx in range(int(xlim[0] // 20 * 20), int(xlim[1]) + 1, 20):
        a, b = to_px(gx, ylim[0])
        c, d = to_px(gx, ylim[1])
        _line(rgb, w, h, a, b, c, d, GRID, clip=box)
    for gy in range(int(ylim[0] // 20 * 20), int(ylim[1]) + 1, 20):
        a, b = to_px(xlim[0], gy)
        c, d = to_px(xlim[1], gy)
        _line(rgb, w, h, a, b, c, d, GRID, clip=box)

    tpx = [to_px(*p) for p in teach]
    _line(rgb, w, h, tpx[0][0], tpx[0][1], tpx[1][0], tpx[1][1], TEACH, dash=6, clip=box)
    _line(rgb, w, h, tpx[1][0], tpx[1][1], tpx[2][0], tpx[2][1], TEACH, dash=6, clip=box)

    pxs = [to_px(x, y) for x, y in zip(xs, ys)]
    for i in range(len(pxs) - 1):
        _line(rgb, w, h, pxs[i][0], pxs[i][1], pxs[i + 1][0], pxs[i + 1][1], PATH, clip=box)
    for i, ((x, y), (px, py)) in enumerate(zip(zip(xs, ys), pxs)):
        if not _in_view(x, y, xlim, ylim):
            continue
        if i == 0:
            _disk(rgb, w, h, px, py, 7, START, clip=box)
        elif i == len(pxs) - 1:
            _disk(rgb, w, h, px, py, 7, END, clip=box)
        elif i in near:
            _disk(rgb, w, h, px, py, 5, NEAR, clip=box)
        else:
            _disk(rgb, w, h, px, py, 4, DOT, clip=box)
    if _in_view(teach[1][0], teach[1][1], xlim, ylim):
        cx, cy = to_px(teach[1][0], teach[1][1])
        _disk(rgb, w, h, cx, cy, 6, CORNER, clip=box)


def main() -> None:
    begin = [0.0, 0.0, 0.30, math.pi, 0.0, 0.0]
    via = [0.15, 0.0, 0.30, math.pi, 0.0, 0.05]
    end = [0.15, 0.12, 0.30, math.pi, 0.0, 0.10]
    ok, points = obtain_swing_waypoint_crescent_moon_bent(
        begin,
        via,
        end,
        angle=0.0,
        welding_speed=6.0,
        frequency=1.0,
        hold_distance=0.0,
        amplitude=4.0,
        direction=1,
        transition_radius=1.0,
    )
    if not ok:
        raise RuntimeError("calculation failed")

    xs = mm([p[0] for p in points])
    ys = mm([p[1] for p in points])
    teach = [mm(begin[:2]), mm(via[:2]), mm(end[:2])]
    near = {
        i
        for i, (x, y) in enumerate(zip(xs, ys))
        if math.hypot(x - 150.0, y - 0.0) < 12.0 and i not in (0, len(xs) - 1)
    }

    w, h = 1400, 720
    gap = (236, 240, 244)
    rgb = bytearray(gap * (w * h))
    _draw_panel(rgb, w, h, (40, 40, 680, 680), xs, ys, teach, near, (-12, 195), (-22, 140))
    _draw_panel(rgb, w, h, (740, 40, 1360, 680), xs, ys, teach, near, (138, 162), (-12, 22))

    out = Path(__file__).with_name("月牙摆焊点位图.png")
    _save_png(out, w, h, rgb)
    print(f"saved {out}  points={len(points)}")


if __name__ == "__main__":
    main()
