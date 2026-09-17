#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function

import math
import struct
import unittest

from pathlib import Path

from state_listener import JOINT_STRIDE, RobotStateStream, format_pose, parse_robot_state


def _packet(subpackages):
    body = b""
    for sub_type, payload in subpackages:
        sub = struct.pack(">iB", 5 + len(payload), sub_type) + payload
        body += sub
    total = 5 + len(body)
    return struct.pack(">iB", total, 16) + body


class TestStateListener(unittest.TestCase):
    def test_parse_joint_and_cartesian(self):
        joints = [0.1, 0.2, 0.3, -0.4, 1.0, -1.2]
        joint_payload = b""
        for ang in joints:
            joint_payload += struct.pack(">d", ang) + b"\x00" * (JOINT_STRIDE - 8)
        cart = (0.12, -0.34, 0.56, 0.01, -0.02, 0.03)
        cart_payload = struct.pack(">6d", *cart) + struct.pack(">6d", *([0.0] * 6))
        packet = _packet([(1, joint_payload), (4, cart_payload)])
        parsed = parse_robot_state(packet)
        self.assertIsNotNone(parsed)
        for a, b in zip(parsed["joints_rad"], joints):
            self.assertAlmostEqual(a, b)
        for a, b in zip(parsed["cart_m_rad"], cart):
            self.assertAlmostEqual(a, b)
        shown = format_pose(parsed)
        self.assertAlmostEqual(shown["xyz_mm"][0], 120.0)
        self.assertAlmostEqual(shown["joints_deg"][0], math.degrees(0.1))

    def test_stream_splits_packets(self):
        cart = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
        payload = struct.pack(">6d", *cart) + struct.pack(">6d", *([0.0] * 6))
        packet = _packet([(4, payload)])
        stream = RobotStateStream()
        mid = len(packet) // 2
        self.assertEqual(stream.feed(packet[:mid]), [])
        poses = stream.feed(packet[mid:])
        self.assertEqual(len(poses), 1)
        self.assertAlmostEqual(poses[0]["cart_m_rad"][2], 3.0)


class TestTrajectory(unittest.TestCase):
    def test_write_csv_and_html(self, tmp=None):
        import tempfile
        from trajectory import pose_sample, write_csv, write_html

        parsed = {
            "joints_rad": (0.0, 0.1, 0.2, 0.3, 0.4, 0.5),
            "cart_m_rad": (0.01, 0.02, 0.03, 0.0, 0.0, 0.0),
        }
        samples = [pose_sample(parsed, 0.0, 0.1), pose_sample(parsed, 0.0, 0.2)]
        samples[1]["x_mm"] = 20.0
        samples[1]["y_mm"] = 30.0
        samples[1]["z_mm"] = 40.0
        folder = Path(tempfile.mkdtemp())
        csv_path = write_csv(folder / "t.csv", samples)
        html_path = write_html(folder / "t.html", samples)
        text = csv_path.read_text(encoding="utf-8-sig")
        self.assertIn("x_mm", text)
        self.assertIn("20.000", text)
        html = html_path.read_text(encoding="utf-8")
        self.assertIn("var pts =", html)
        self.assertIn('"x": 20.0', html)
        self.assertIn("O(0,0,0)", html)
        self.assertIn("保存HTML", html)
        self.assertIn("nameIn", html)
        self.assertIn("savedState", html)
        self.assertIn('id="chkMark"', html)
        self.assertIn('id="btnCancel"', html)
        self.assertNotIn('id="chkAxes" checked', html)
        self.assertNotIn('id="chkData" checked', html)


if __name__ == "__main__":
    unittest.main()
