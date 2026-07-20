import numpy as np
from scipy.spatial.transform import Rotation as R


def rotmat_to_euler_zyx_degrees(rotation: np.ndarray) -> tuple[float, float, float]:
    r20 = float(rotation[2, 0])
    r20 = max(-1.0, min(1.0, r20))
    pitch = np.arcsin(-r20)

    if abs(r20) > 0.9999:
        yaw = np.arctan2(-rotation[0, 1], rotation[1, 1])
        roll = 0.0
    else:
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])

    yaw_deg, pitch_deg, roll_deg = np.degrees([yaw, pitch, roll])
    return float(yaw_deg), float(pitch_deg), float(roll_deg)


def homogeneous_from_pose(position: np.ndarray, quaternion_xyzw: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = R.from_quat(quaternion_xyzw).as_matrix()
    transform[:3, 3] = position.reshape(3)
    return transform


def homogeneous_from_tag_pose(
    translation_camera: np.ndarray,
    rotation_camera: np.ndarray,
) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation_camera
    transform[:3, 3] = translation_camera.reshape(3)
    return transform


def tag_pose_to_world(
    translation_camera: np.ndarray,
    rotation_camera: np.ndarray,
    world_drone_transform: np.ndarray,
    drone_camera_transform: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute tag pose in world frame:

        T_world_tag = T_world_drone @ T_drone_camera @ T_camera_tag
    """
    camera_tag = homogeneous_from_tag_pose(translation_camera, rotation_camera)
    world_tag = world_drone_transform @ drone_camera_transform @ camera_tag
    world_position = world_tag[:3, 3].copy()
    world_rotation = world_tag[:3, :3].copy()
    return world_position, world_rotation
