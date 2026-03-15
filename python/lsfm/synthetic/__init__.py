"""lsfm.synthetic — Synthetic 3D multiview line reconstruction demo.

Provides tools for creating synthetic 3D scenes with known ground truth,
rendering wireframe and textured images, running line detection and
matching, and evaluating reconstruction accuracy at every pipeline stage.

Usage::

    from lsfm.synthetic import (
        Box3D, Pyramid3D,
        Scene, build_church_scene,
        CameraRig,
        SceneGroundTruth,
        render_wireframe, render_all_views,
        render_textured, render_all_textured_views,
        oracle_match, assign_detected_to_gt,
        evaluate_detection, evaluate_matching, evaluate_reconstruction,
    )
"""

from __future__ import annotations

from lsfm.synthetic.camera_rig import CameraRig, make_stereo_camera
from lsfm.synthetic.conversions import (
    build_line_segments_3d,
    segments3d_to_array,
    segments_to_array,
)
from lsfm.synthetic.evaluation import (
    DetectionMetrics,
    MatchMetrics,
    ReconstructionMetrics,
    evaluate_detection,
    evaluate_matching,
    evaluate_reconstruction,
    evaluation_summary,
)
from lsfm.synthetic.ground_truth import (
    SceneGroundTruth,
    clip_to_frustum,
    filter_behind_camera,
    filter_occluded,
    project_segments,
    render_depth_map,
)
from lsfm.synthetic.oracle import assign_detected_to_gt, oracle_match, oracle_tracks
from lsfm.synthetic.primitives import Box3D, Primitive, Pyramid3D
from lsfm.synthetic.renderer import (
    render_all_textured_views,
    render_all_views,
    render_textured,
    render_wireframe,
)
from lsfm.synthetic.scene import Scene, build_church_scene

__all__ = [
    # Primitives
    "Box3D",
    "Primitive",
    "Pyramid3D",
    # Scene
    "Scene",
    "build_church_scene",
    # Camera
    "CameraRig",
    "make_stereo_camera",
    # Ground truth
    "SceneGroundTruth",
    "clip_to_frustum",
    "filter_behind_camera",
    "filter_occluded",
    "project_segments",
    "render_depth_map",
    # Conversions
    "build_line_segments_3d",
    "segments3d_to_array",
    "segments_to_array",
    # Rendering
    "render_all_textured_views",
    "render_all_views",
    "render_textured",
    "render_wireframe",
    # Oracle
    "assign_detected_to_gt",
    "oracle_match",
    "oracle_tracks",
    # Evaluation
    "DetectionMetrics",
    "MatchMetrics",
    "ReconstructionMetrics",
    "evaluate_detection",
    "evaluate_matching",
    "evaluate_reconstruction",
    "evaluation_summary",
]
