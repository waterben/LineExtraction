"""3D geometric primitives with topology-geometry separation.

Each primitive exposes raw vertex arrays and index arrays (topology).
``LineSegment3_f64`` construction is deferred to the point of use via
:func:`~lsfm.synthetic.conversions.build_line_segments_3d`.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np
from numpy.typing import NDArray


@runtime_checkable
class Primitive(Protocol):
    """Protocol for 3D geometric primitives.

    Primitives provide vertices and index arrays that describe the
    topology. Geometry (world-space positions) is derived by applying
    a ``Pose_f64`` transform at the point of use.
    """

    @property
    def name(self) -> str:
        """Human-readable name of this primitive."""
        ...

    def vertices(self) -> NDArray[np.float64]:
        """Return local-space vertices as an ``(N, 3)`` float64 array."""
        ...

    def edge_indices(self) -> list[tuple[int, int]]:
        """Return unique edge index pairs ``[(i, j), ...]``."""
        ...

    def face_indices(self) -> list[list[int]]:
        """Return face vertex index lists ``[[i, j, k, ...], ...]``."""
        ...


class Box3D:
    """Axis-aligned box centered at the origin.

    :param width: Extent along X.
    :type width: float
    :param height: Extent along Y.
    :type height: float
    :param depth: Extent along Z.
    :type depth: float
    :param name: Optional display name.
    :type name: str
    """

    def __init__(
        self,
        width: float = 1.0,
        height: float = 1.0,
        depth: float = 1.0,
        name: str = "box",
    ) -> None:
        self._width = width
        self._height = height
        self._depth = depth
        self._name = name

    @property
    def name(self) -> str:
        """Human-readable name of this primitive."""
        return self._name

    def vertices(self) -> NDArray[np.float64]:
        """Return 8 vertices as ``(8, 3)`` float64 array.

        Vertex ordering (right-hand rule, Y-up):

        .. code-block:: text

            Bottom face (y = -h/2): 0-1-2-3
            Top face    (y = +h/2): 4-5-6-7
        """
        hw = self._width / 2.0
        hh = self._height / 2.0
        hd = self._depth / 2.0
        return np.array(
            [
                [-hw, -hh, -hd],
                [+hw, -hh, -hd],
                [+hw, -hh, +hd],
                [-hw, -hh, +hd],
                [-hw, +hh, -hd],
                [+hw, +hh, -hd],
                [+hw, +hh, +hd],
                [-hw, +hh, +hd],
            ],
            dtype=np.float64,
        )

    def edge_indices(self) -> list[tuple[int, int]]:
        """Return 12 unique edges of the box."""
        return [
            # Bottom face
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            # Top face
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            # Vertical edges
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ]

    def face_indices(self) -> list[list[int]]:
        """Return 6 quad faces with outward-pointing normals (CCW winding)."""
        return [
            [0, 1, 2, 3],  # bottom (-Y)
            [4, 7, 6, 5],  # top (+Y)
            [0, 4, 5, 1],  # front (-Z)
            [2, 6, 7, 3],  # back (+Z)
            [0, 3, 7, 4],  # left (-X)
            [1, 5, 6, 2],  # right (+X)
        ]


class Pyramid3D:
    """Square-base pyramid centered at the origin (base center).

    :param base_width: Width of the square base along X.
    :type base_width: float
    :param base_depth: Depth of the square base along Z.
    :type base_depth: float
    :param height: Height along Y (apex above base center).
    :type height: float
    :param name: Optional display name.
    :type name: str
    """

    def __init__(
        self,
        base_width: float = 1.0,
        base_depth: float = 1.0,
        height: float = 1.0,
        name: str = "pyramid",
    ) -> None:
        self._base_width = base_width
        self._base_depth = base_depth
        self._height = height
        self._name = name

    @property
    def name(self) -> str:
        """Human-readable name of this primitive."""
        return self._name

    def vertices(self) -> NDArray[np.float64]:
        """Return 5 vertices as ``(5, 3)`` float64 array.

        Vertex ordering:

        .. code-block:: text

            Base (y = 0): 0-1-2-3
            Apex (y = h): 4
        """
        hw = self._base_width / 2.0
        hd = self._base_depth / 2.0
        return np.array(
            [
                [-hw, 0.0, -hd],
                [+hw, 0.0, -hd],
                [+hw, 0.0, +hd],
                [-hw, 0.0, +hd],
                [0.0, self._height, 0.0],
            ],
            dtype=np.float64,
        )

    def edge_indices(self) -> list[tuple[int, int]]:
        """Return 8 unique edges of the pyramid."""
        return [
            # Base
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            # Apex edges
            (0, 4),
            (1, 4),
            (2, 4),
            (3, 4),
        ]

    def face_indices(self) -> list[list[int]]:
        """Return 5 faces with outward-pointing normals (CCW winding)."""
        return [
            [0, 1, 2, 3],  # base (-Y)
            [0, 4, 1],  # front (-Z, +Y)
            [1, 4, 2],  # right (+X, +Y)
            [2, 4, 3],  # back (+Z, +Y)
            [3, 4, 0],  # left (-X, +Y)
        ]
