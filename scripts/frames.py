"""Fixed geodetic tangent NED world; FRD IMU at CG; optical camera."""
import numpy as np
from scipy.spatial.transform import Rotation

FT = 0.3048


def ecef_to_ned(latitude, longitude):
    s, c = np.sin(latitude), np.cos(latitude)
    sl, cl = np.sin(longitude), np.cos(longitude)
    return np.array([[-s*cl, -s*sl, c], [-sl, cl, 0], [-c*cl, -c*sl, -s]])


def quat_wxyz(matrix):
    q = Rotation.from_matrix(matrix).as_quat(canonical=True)
    return np.roll(q, 1, axis=-1)


def matrix_wxyz(q):
    return Rotation.from_quat(np.roll(q, -1, axis=-1)).as_matrix()


def camera_transform(camera):
    # Optical [right, down, forward] -> camera FRD [forward, right, down].
    optical_to_frd = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    mount = Rotation.from_euler('y', camera['mount_pitch_deg'], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = mount @ optical_to_frd
    T[:3, 3] = camera['translation_body_m']
    return T


def kinematic_imu(velocity, rotation, dt, gravity):
    """Ideal flat-local VIO IMU, consistent with the exported fixed-world GT.

    velocity and rotation sampled at the full physics rate. CG is IMU origin.
    Earth rotation/transport are absorbed into this kinematic convention; this
    is not a model of an Earth-rate-sensitive physical IMU.
    """
    acceleration = np.gradient(velocity, dt, axis=0, edge_order=2)
    force = np.einsum('nji,nj->ni', rotation, acceleration - [0, 0, gravity])
    rdot = np.gradient(rotation, dt, axis=0, edge_order=2)
    skew = np.einsum('nji,njk->nik', rotation, rdot)
    gyro = np.stack([skew[:, 2, 1]-skew[:, 1, 2],
                     skew[:, 0, 2]-skew[:, 2, 0],
                     skew[:, 1, 0]-skew[:, 0, 1]], axis=1) / 2
    return gyro, force
