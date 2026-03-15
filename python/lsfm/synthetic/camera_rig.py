"""Camera rig generator for multi-view setups.

Creates arrays of ``Camera_f64`` instances for orbital, linear, or
custom placements.  Derived projectors (``CameraPluecker_f64``) are
computed lazily on first access.
"""

from __future__ import annotations

import math
from functools import cached_property
from typing import TYPE_CHECKING, Iterator

import numpy as np

if TYPE_CHECKING:
    import le_geometry


def _look_at(
    eye: np.ndarray,
    target: np.ndarray,
    up: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute rotation matrix and translation for a look-at camera.

    Uses the OpenCV / LE convention where the camera looks along +Z
    and the rotation + translation express the world-to-camera transform.

    :param eye: Camera position in world space ``(3,)``.
    :type eye: numpy.ndarray
    :param target: Point the camera looks at ``(3,)``.
    :type target: numpy.ndarray
    :param up: World up direction ``(3,)``.
    :type up: numpy.ndarray
    :return: ``(R, t)`` where ``R`` is a 3x3 rotation matrix and ``t``
        is the translation vector such that ``X_cam = R @ X_world + t``.
    :rtype: tuple[numpy.ndarray, numpy.ndarray]
    """
    forward = target - eye
    forward = forward / np.linalg.norm(forward)

    right = np.cross(forward, up)
    norm = np.linalg.norm(right)
    if norm < 1e-12:
        # Degenerate: forward is parallel to up — pick arbitrary right
        right = np.array([1.0, 0.0, 0.0])
    else:
        right = right / norm

    true_up = np.cross(right, forward)

    # Camera axes: x=right, y=-true_up (Y-down), z=forward
    # Rotation matrix (world-to-camera)
    rot = np.array(
        [
            right,
            -true_up,
            forward,
        ],
        dtype=np.float64,
    )

    # Translation: t = -R @ eye
    trans = -rot @ eye

    return rot, trans


def _rotation_matrix_to_rodrigues(rot: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a Rodrigues vector.

    :param rot: 3x3 rotation matrix.
    :type rot: numpy.ndarray
    :return: Rodrigues vector ``(3,)``.
    :rtype: numpy.ndarray
    """
    # Angle from trace: trace(R) = 1 + 2*cos(theta)
    trace = rot[0, 0] + rot[1, 1] + rot[2, 2]
    cos_theta = (trace - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = math.acos(cos_theta)

    if abs(theta) < 1e-12:
        return np.zeros(3, dtype=np.float64)

    if abs(theta - math.pi) < 1e-6:
        # Near 180 degrees: extract from diagonal
        diag = np.diag(rot)
        idx = int(np.argmax(diag))
        v = np.zeros(3)
        v[idx] = 1.0
        col = (rot[:, idx] + np.eye(3)[idx]) / (1.0 + diag[idx])
        axis = col / np.linalg.norm(col)
        return axis * theta

    # General case: axis from skew-symmetric part
    axis = np.array(
        [
            rot[2, 1] - rot[1, 2],
            rot[0, 2] - rot[2, 0],
            rot[1, 0] - rot[0, 1],
        ],
        dtype=np.float64,
    )
    axis = axis / (2.0 * math.sin(theta))
    return axis * theta


class CameraRig:
    """A collection of cameras sharing identical intrinsics.

    :param cameras: List of ``Camera_f64`` instances.
    :type cameras: list[le_geometry.Camera_f64]
    """

    def __init__(self, cameras: list[le_geometry.Camera_f64]) -> None:
        self._cameras = list(cameras)

    @property
    def cameras(self) -> list[le_geometry.Camera_f64]:
        """Return the list of cameras."""
        return list(self._cameras)

    def __len__(self) -> int:
        return len(self._cameras)

    @cached_property
    def projectors(self) -> list[le_geometry.CameraPluecker_f64]:
        """Pluecker projectors, one per camera (cached on first access).

        :return: List of ``CameraPluecker_f64`` constructed from cameras.
        :rtype: list[le_geometry.CameraPluecker_f64]
        """
        import le_geometry

        return [le_geometry.CameraPluecker_f64(cam) for cam in self._cameras]

    def camera_pair(
        self, i: int, j: int
    ) -> tuple[le_geometry.Camera_f64, le_geometry.Camera_f64]:
        """Return a camera pair by index.

        :param i: Index of the first camera.
        :type i: int
        :param j: Index of the second camera.
        :type j: int
        :return: ``(camera_i, camera_j)`` tuple.
        :rtype: tuple[le_geometry.Camera_f64, le_geometry.Camera_f64]
        """
        return self._cameras[i], self._cameras[j]

    def pairs(self) -> Iterator[tuple[int, int]]:
        """Yield index pairs for adjacent camera views.

        :return: Iterator of ``(i, i+1)`` tuples.
        :rtype: Iterator[tuple[int, int]]
        """
        for i in range(len(self._cameras) - 1):
            yield i, i + 1

    @classmethod
    def orbital(
        cls,
        n_cameras: int,
        target: tuple[float, float, float] = (0.0, 0.0, 0.0),
        radius: float = 10.0,
        height: float = 2.0,
        focal_length: float = 500.0,
        image_width: int = 640,
        image_height: int = 480,
    ) -> CameraRig:
        """Generate cameras equally spaced on a circle looking at a target.

        :param n_cameras: Number of cameras to generate.
        :type n_cameras: int
        :param target: World-space point all cameras look at.
        :type target: tuple[float, float, float]
        :param radius: Radius of the orbital circle.
        :type radius: float
        :param height: Y-coordinate of the camera orbit.
        :type height: float
        :param focal_length: Focal length in pixels (same for fx, fy).
        :type focal_length: float
        :param image_width: Image width in pixels.
        :type image_width: int
        :param image_height: Image height in pixels.
        :type image_height: int
        :return: A camera rig with ``n_cameras`` orbital cameras.
        :rtype: CameraRig
        """
        cameras = []
        target_arr = np.array(target, dtype=np.float64)
        up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        cx = image_width / 2.0
        cy = image_height / 2.0

        for i in range(n_cameras):
            angle = 2.0 * math.pi * i / n_cameras
            eye = np.array(
                [
                    target[0] + radius * math.cos(angle),
                    height,
                    target[2] + radius * math.sin(angle),
                ],
                dtype=np.float64,
            )
            cam = _make_camera(
                eye, target_arr, up, focal_length, cx, cy, image_width, image_height
            )
            cameras.append(cam)

        return cls(cameras)

    @classmethod
    def linear(
        cls,
        n_cameras: int,
        start: tuple[float, float, float] = (-2.0, 2.0, -10.0),
        end: tuple[float, float, float] = (2.0, 2.0, -10.0),
        target: tuple[float, float, float] = (0.0, 0.0, 0.0),
        focal_length: float = 500.0,
        image_width: int = 640,
        image_height: int = 480,
    ) -> CameraRig:
        """Generate cameras evenly spaced along a baseline.

        :param n_cameras: Number of cameras to generate.
        :type n_cameras: int
        :param start: Start position of the baseline.
        :type start: tuple[float, float, float]
        :param end: End position of the baseline.
        :type end: tuple[float, float, float]
        :param target: World-space point all cameras look at.
        :type target: tuple[float, float, float]
        :param focal_length: Focal length in pixels.
        :type focal_length: float
        :param image_width: Image width in pixels.
        :type image_width: int
        :param image_height: Image height in pixels.
        :type image_height: int
        :return: A camera rig with linearly spaced cameras.
        :rtype: CameraRig
        """
        cameras = []
        start_arr = np.array(start, dtype=np.float64)
        end_arr = np.array(end, dtype=np.float64)
        target_arr = np.array(target, dtype=np.float64)
        up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        cx = image_width / 2.0
        cy = image_height / 2.0

        for i in range(n_cameras):
            t = i / max(n_cameras - 1, 1)
            eye = start_arr + t * (end_arr - start_arr)
            cam = _make_camera(
                eye, target_arr, up, focal_length, cx, cy, image_width, image_height
            )
            cameras.append(cam)

        return cls(cameras)

    @classmethod
    def from_poses(
        cls,
        poses: list[le_geometry.Pose_f64],
        focal_length: float = 500.0,
        image_width: int = 640,
        image_height: int = 480,
    ) -> CameraRig:
        """Create a rig from explicit poses.

        :param poses: List of ``Pose_f64`` for each camera.
        :type poses: list[le_geometry.Pose_f64]
        :param focal_length: Focal length in pixels.
        :type focal_length: float
        :param image_width: Image width in pixels.
        :type image_width: int
        :param image_height: Image height in pixels.
        :type image_height: int
        :return: A camera rig with one camera per pose.
        :rtype: CameraRig
        """
        import le_geometry

        cameras = []
        cx = image_width / 2.0
        cy = image_height / 2.0

        for pose in poses:
            t = pose.origin
            r = pose.orientation
            cam = le_geometry.Camera_f64(
                focal_length,
                focal_length,
                cx,
                cy,
                float(image_width),
                float(image_height),
                float(t[0]),
                float(t[1]),
                float(t[2]),
                float(r[0]),
                float(r[1]),
                float(r[2]),
            )
            cameras.append(cam)

        return cls(cameras)


def _make_camera(
    eye: np.ndarray,
    target: np.ndarray,
    up: np.ndarray,
    focal_length: float,
    cx: float,
    cy: float,
    image_width: int,
    image_height: int,
) -> le_geometry.Camera_f64:
    """Create a Camera_f64 looking from eye toward target.

    :param eye: Camera position in world space.
    :type eye: numpy.ndarray
    :param target: Look-at target.
    :type target: numpy.ndarray
    :param up: World up vector.
    :type up: numpy.ndarray
    :param focal_length: Focal length.
    :type focal_length: float
    :param cx: Principal point X.
    :type cx: float
    :param cy: Principal point Y.
    :type cy: float
    :param image_width: Image width.
    :type image_width: int
    :param image_height: Image height.
    :type image_height: int
    :return: A configured Camera_f64 instance.
    :rtype: le_geometry.Camera_f64
    """
    import le_geometry

    rot_w2c, _ = _look_at(eye, target, up)
    rvec_w2c = _rotation_matrix_to_rodrigues(rot_w2c)

    # Pose stores camera-to-world: origin = eye, rotation = R_c2w.
    # R_c2w = R_w2c^T  =>  rodrigues(R_c2w) = -rodrigues(R_w2c).
    return le_geometry.Camera_f64(
        focal_length,
        focal_length,
        cx,
        cy,
        float(image_width),
        float(image_height),
        float(eye[0]),
        float(eye[1]),
        float(eye[2]),
        float(-rvec_w2c[0]),
        float(-rvec_w2c[1]),
        float(-rvec_w2c[2]),
    )


def make_stereo_camera(
    camera: le_geometry.Camera_f64,
    baseline: float = 0.5,
) -> le_geometry.Camera_f64:
    """Create a stereo partner by shifting a camera along its local X-axis.

    The returned camera has the same orientation and intrinsics as the
    input but its origin is displaced by *baseline* along the camera's
    right (local X) direction.  This produces a standard horizontal
    stereo pair.

    :param camera: Reference camera.
    :type camera: le_geometry.Camera_f64
    :param baseline: Horizontal offset in world units (positive = right).
    :type baseline: float
    :return: A new camera shifted along the local X-axis.
    :rtype: le_geometry.Camera_f64
    """
    import le_geometry

    # Extract c2w from hom_matrix
    hom = camera.hom_matrix()
    rot_c2w = np.array(
        [[hom[r][c] for c in range(3)] for r in range(3)], dtype=np.float64
    )
    origin = np.array([hom[0][3], hom[1][3], hom[2][3]], dtype=np.float64)

    # Camera local X-axis in world coords (first column of c2w rotation)
    right = rot_c2w[:, 0]

    new_origin = origin + baseline * right

    # Rodrigues for c2w rotation: negate the w2c Rodrigues
    rot_w2c = rot_c2w.T
    rvec_w2c = _rotation_matrix_to_rodrigues(rot_w2c)

    fx, fy = camera.focal
    cx, cy = camera.offset
    w = camera.width
    h = camera.height

    return le_geometry.Camera_f64(
        float(fx),
        float(fy),
        float(cx),
        float(cy),
        float(w),
        float(h),
        float(new_origin[0]),
        float(new_origin[1]),
        float(new_origin[2]),
        float(-rvec_w2c[0]),
        float(-rvec_w2c[1]),
        float(-rvec_w2c[2]),
    )
