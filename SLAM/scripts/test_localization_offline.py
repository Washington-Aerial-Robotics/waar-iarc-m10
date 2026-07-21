#!/usr/bin/env python3
"""Offline smoke test for pose fusion (no ESP32 required)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

SLAM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SLAM_DIR))

from localization.fusion import PoseFusion  # noqa: E402
from localization.imu import ImuAttitudeFilter  # noqa: E402


def test_imu_attitude():
    filt = ImuAttitudeFilter()
    t = 0.0
    for _ in range(50):
        t += 0.02
        euler = filt.update(
            np.array([0.0, 0.0, 9.81]),
            np.array([0.0, 0.0, 0.1]),
            t,
        )
    assert abs(euler[0]) > 0.01
    print("IMU attitude filter: OK")


def test_pose_fusion_motion():
    fusion = PoseFusion(
        launch_position=np.array([0.0, 0.0, 1.5]),
        launch_quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
        fx=700.0,
        fy=700.0,
    )
    t = 0.0
    for _ in range(30):
        t += 0.05
        fusion.update_imu(
            np.array([0.1, 0.0, 9.81]),
            np.array([0.0, 0.0, 0.05]),
            t,
        )
    fusion.apply_tag_correction(
        np.array([1.0, 0.0, 0.0]),
        np.array([1.5, 0.0, 0.0]),
        confidence=0.9,
    )
    snap = fusion.snapshot()
    assert snap["correction_count"] == 1
    print(f"Pose fusion snapshot: pos={snap['position']} corrections={snap['correction_count']}")
    print("Pose fusion: OK")


def test_state_packing():
    from localization.esp32_comms import pack_state, unpack_state

    pos = np.array([1.0, 2.0, 3.0])
    vel = np.array([0.1, 0.0, 0.0])
    euler = np.array([0.1, 0.2, 0.3])
    omega = np.array([0.0, 0.0, 0.5])
    packed = pack_state(pos, vel, euler, omega)
    assert len(packed) == 64
    state = unpack_state(packed)
    assert np.allclose(state.position, pos)
    print("State pack/unpack: OK")


if __name__ == "__main__":
    test_imu_attitude()
    test_pose_fusion_motion()
    test_state_packing()
    print("\nAll localization offline tests passed.")
