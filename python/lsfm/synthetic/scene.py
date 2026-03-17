"""Scene composition with vertex welding for edge deduplication.

A :class:`Scene` holds a flat list of ``(Primitive, Pose_f64)`` entries.
Vertex welding merges coincident world-space vertices, ensuring that
shared edges between adjacent primitives are naturally deduplicated.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from lsfm.synthetic.conversions import _transform_vertices
from lsfm.synthetic.primitives import Primitive

if TYPE_CHECKING:
    import le_geometry


class Scene:
    """Composite 3D scene made of positioned primitives.

    :param tolerance: Vertex welding tolerance for merging coincident
        vertices.
    :type tolerance: float
    """

    def __init__(self, tolerance: float = 1e-6) -> None:
        self._entries: list[tuple[Primitive, le_geometry.Pose_f64]] = []
        self._tolerance = tolerance

    def add(self, primitive: Primitive, pose: le_geometry.Pose_f64) -> None:
        """Place a primitive in world space.

        :param primitive: A 3D primitive providing vertices and edges.
        :type primitive: Primitive
        :param pose: World-space pose for the primitive.
        :type pose: le_geometry.Pose_f64
        """
        self._entries.append((primitive, pose))

    @property
    def entries(self) -> list[tuple[Primitive, le_geometry.Pose_f64]]:
        """Return the list of ``(Primitive, Pose_f64)`` entries."""
        return list(self._entries)

    def ground_truth_lines(self) -> list[le_geometry.LineSegment3_f64]:
        """Return all unique 3D line segments after vertex welding.

        World-space vertices within :attr:`tolerance` are merged, then
        unique edges are derived from merged vertex indices.

        :return: Deduplicated 3D line segments in world space.
        :rtype: list[le_geometry.LineSegment3_f64]
        """
        import le_geometry

        if not self._entries:
            return []

        # Collect all world-space vertices and edges
        all_vertices: list[NDArray[np.float64]] = []
        all_edges: list[tuple[int, int]] = []
        offset = 0

        for prim, pose in self._entries:
            verts = _transform_vertices(prim.vertices(), pose)
            edges = prim.edge_indices()
            for i, j in edges:
                all_edges.append((i + offset, j + offset))
            all_vertices.append(verts)
            offset += len(verts)

        world_verts = np.vstack(all_vertices)

        # Vertex welding: merge vertices within tolerance
        n = len(world_verts)
        parent = list(range(n))

        def find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def union(a: int, b: int) -> None:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[rb] = ra

        for i in range(n):
            for j in range(i + 1, n):
                if np.linalg.norm(world_verts[i] - world_verts[j]) < self._tolerance:
                    union(i, j)

        # Canonical vertex positions (representative of each group)
        canonical: dict[int, NDArray[np.float64]] = {}
        for i in range(n):
            root = find(i)
            if root not in canonical:
                canonical[root] = world_verts[root]

        # Deduplicate edges using canonical vertex indices
        unique_edges: set[tuple[int, int]] = set()
        for i, j in all_edges:
            ri, rj = find(i), find(j)
            if ri == rj:
                continue
            edge = (min(ri, rj), max(ri, rj))
            unique_edges.add(edge)

        # Build LineSegment3_f64 from unique edges
        segments: list[le_geometry.LineSegment3_f64] = []
        for ri, rj in sorted(unique_edges):
            v0 = canonical[ri]
            v1 = canonical[rj]
            seg = le_geometry.LineSegment3_f64.from_endpoints(
                float(v0[0]),
                float(v0[1]),
                float(v0[2]),
                float(v1[0]),
                float(v1[1]),
                float(v1[2]),
            )
            segments.append(seg)
        return segments

    def bounding_box(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Return the axis-aligned bounding box of the scene.

        :return: ``(min_corner, max_corner)`` each as ``(3,)`` arrays.
        :rtype: tuple[numpy.ndarray, numpy.ndarray]
        """
        all_verts: list[NDArray[np.float64]] = []
        for prim, pose in self._entries:
            all_verts.append(_transform_vertices(prim.vertices(), pose))
        if not all_verts:
            return np.zeros(3), np.zeros(3)
        verts = np.vstack(all_verts)
        return verts.min(axis=0), verts.max(axis=0)

    def center(self) -> NDArray[np.float64]:
        """Return the center of the scene bounding box.

        :return: Center point as ``(3,)`` array.
        :rtype: numpy.ndarray
        """
        bb_min, bb_max = self.bounding_box()
        return (bb_min + bb_max) / 2.0


def build_church_scene() -> Scene:
    """Build a recognizable church scene from boxes and pyramids.

    The church consists of four flush-joined primitives:

    - **body** — wide box forming the nave
    - **roof** — wide shallow pyramid sitting on the body
    - **tower** — tall narrow box flush against the body's left side
    - **steeple** — pyramid capping the tower

    Adjacent primitives share coplanar faces (e.g. body top / roof
    base).  Back-face culling resolves visibility: for any viewpoint
    the two opposing normals ensure only the outward-facing surface is
    rendered while vertex welding deduplicates the shared edges.

    :return: A fully assembled church scene.
    :rtype: Scene
    """
    import le_geometry

    from lsfm.synthetic.primitives import Box3D, Pyramid3D

    scene = Scene()

    # Main body: wide box centered at (0, 1, 0).
    #   X: [-2, +2], Y: [0, 2], Z: [-1.5, +1.5]
    body = Box3D(width=4.0, height=2.0, depth=3.0, name="body")
    body_pose = le_geometry.Pose_f64(0.0, 1.0, 0.0)
    scene.add(body, body_pose)

    # Nave roof: pyramid base flush on body top (Y = 2), apex at Y = 3.
    roof = Pyramid3D(base_width=4.0, base_depth=3.0, height=1.0, name="roof")
    roof_pose = le_geometry.Pose_f64(0.0, 2.0, 0.0)
    scene.add(roof, roof_pose)

    # Tower: flush against body left face (X = -2).
    #   width = 1.5, half = 0.75
    #   center X = -2.0 - 0.75 = -2.75  →  X: [-3.5, -2.0]
    #   height = 4, center Y = 2  →  Y: [0, 4]
    tower = Box3D(width=1.5, height=4.0, depth=1.5, name="tower")
    tower_pose = le_geometry.Pose_f64(-2.75, 2.0, 0.0)
    scene.add(tower, tower_pose)

    # Steeple: pyramid base flush on tower top (Y = 4), apex at Y = 5.5.
    steeple = Pyramid3D(base_width=1.5, base_depth=1.5, height=1.5, name="steeple")
    steeple_pose = le_geometry.Pose_f64(-2.75, 4.0, 0.0)
    scene.add(steeple, steeple_pose)

    return scene
