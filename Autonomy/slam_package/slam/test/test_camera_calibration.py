from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from slam.calibration import scaled_camera_info


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def calibration():
    return {
        'image_width': 1280,
        'image_height': 960,
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'data': [1, 2, 3, 4, 5]},
        'camera_matrix': {
            'data': [800, 0, 640, 0, 810, 480, 0, 0, 1],
        },
        'rectification_matrix': {
            'data': [1, 0, 0, 0, 1, 0, 0, 0, 1],
        },
        'projection_matrix': {
            'data': [750, 0, 620, -45, 0, 760, 470, 0, 0, 0, 1, 0],
        },
    }


def test_uniform_scaling_preserves_stereo_baseline_ratio():
    result = scaled_camera_info(calibration(), 0.5)
    assert result['width'] == 640
    assert result['height'] == 480
    assert result['k'] == pytest.approx([400, 0, 320, 0, 405, 240, 0, 0, 1])
    assert result['p'] == pytest.approx(
        [375, 0, 310, -22.5, 0, 380, 235, 0, 0, 0, 1, 0]
    )
    assert -result['p'][3] / result['p'][0] == pytest.approx(45 / 750)


@pytest.mark.parametrize('scale', [0.0, -1.0, 1.1])
def test_invalid_camera_scale_rejected(scale):
    with pytest.raises(ValueError):
        scaled_camera_info(calibration(), scale)


def test_urdf_baseline_matches_rectified_camera_calibration():
    with (PACKAGE_ROOT / 'config' / 'right.yaml').open(encoding='utf-8') as stream:
        right = yaml.safe_load(stream)
    projection = right['projection_matrix']['data']
    calibrated_baseline = abs(projection[3] / projection[0])

    root = ET.parse(PACKAGE_ROOT / 'urdf' / 'drone.urdf').getroot()
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    left_y = float(
        joints['base_to_camera_left'].find('origin').attrib['xyz'].split()[1]
    )
    right_y = float(
        joints['base_to_camera_right'].find('origin').attrib['xyz'].split()[1]
    )

    assert abs(left_y - right_y) == pytest.approx(calibrated_baseline, abs=1e-6)
    links = {link.attrib['name'] for link in root.findall('link')}
    assert {'camera_left_optical_frame', 'camera_right_optical_frame'} <= links
