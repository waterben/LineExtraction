#!/usr/bin/env python3
"""Rerun.io visualization demo for line segment detection (LSD).

Runs a line segment detector from the ``le_lsd`` bindings on an image and
streams the results — input image, detected segments, endpoints, and
auxiliary image data layers — to Rerun for interactive exploration.

Usage::

    # Via Bazel (uses bundled windmill.jpg by default)
    bazel run //examples/lsd/python:rerun_lsd_demo

    # With a custom image
    bazel run //examples/lsd/python:rerun_lsd_demo -- /path/to/image.jpg

    # Save to .rrd file instead of spawning viewer
    bazel run //examples/lsd/python:rerun_lsd_demo -- --save output.rrd

    # Directly (if le_lsd is importable)
    python rerun_lsd_demo.py [image_path]
"""

from __future__ import annotations

import argparse
import colorsys
import math
import sys
from typing import Any

import le_lsd
import numpy as np

try:
    import rerun_pth_fix as _  # noqa: F401  (Bazel .pth workaround — not available outside Bazel)
except ImportError:
    pass
import rerun as rr
from PIL import Image

# ============================================================================
# Available detectors (same catalogue as lsd_demo.py)
# ============================================================================

#: Detector classes keyed by display name.
DETECTORS: dict[str, dict[str, Any]] = {
    "CC": {"class": "LsdCC", "desc": "Connected component edge linking + split/fit"},
    "CP": {"class": "LsdCP", "desc": "CC with pattern-based linking"},
    "Burns": {"class": "LsdBurns", "desc": "Classic Burns algorithm"},
    "FBW": {"class": "LsdFBW", "desc": "Fast region growing from gradient seeds"},
    "FGioi": {"class": "LsdFGioi", "desc": "Probabilistic LSD (NFA validation)"},
    "EDLines": {"class": "LsdEDLZ", "desc": "Edge Drawing Lines algorithm"},
    "EL": {"class": "LsdEL", "desc": "Edge linking + NFA + sub-pixel"},
    "EP": {"class": "LsdEP", "desc": "Edge pattern detection + split/fit"},
    "HoughP": {"class": "LsdHoughP", "desc": "Probabilistic Hough Transform"},
}


# ============================================================================
# Helpers
# ============================================================================


def _resolve_default_image() -> str:
    """Try to find windmill.jpg via ``lsfm.data.TestImages``.

    :return: Path string to the default test image.
    :rtype: str
    :raises SystemExit: If the default image cannot be found.
    """
    try:
        from lsfm.data import TestImages

        return str(TestImages().windmill())
    except (ImportError, FileNotFoundError):
        print("Error: no image specified and default windmill.jpg not found.")
        print("Usage: rerun_lsd_demo.py [image_path]")
        sys.exit(1)


def _load_image(path: str) -> np.ndarray:
    """Load an image as grayscale uint8 via Pillow.

    :param path: Path to the image file.
    :type path: str
    :return: Grayscale image as 2-D uint8 array.
    :rtype: numpy.ndarray
    :raises SystemExit: If the image cannot be loaded.
    """
    try:
        pil_img = Image.open(path).convert("L")
        return np.array(pil_img, dtype=np.uint8)
    except Exception as exc:
        print(f"Error: could not load image '{path}': {exc}")
        sys.exit(1)


def _generate_colors(n: int) -> np.ndarray:
    """Generate *n* visually distinct RGB colors using HSV spacing.

    :param n: Number of colors to generate.
    :type n: int
    :return: Array of shape ``(n, 3)`` with uint8 RGB values.
    :rtype: numpy.ndarray
    """
    colors = []
    for i in range(max(n, 1)):
        hue = i / max(n, 1)
        r, g, b = colorsys.hsv_to_rgb(hue, 0.9, 0.9)
        colors.append([int(r * 255), int(g * 255), int(b * 255)])
    return np.array(colors, dtype=np.uint8)


def _numbered_menu(title: str, items: list[str], default: int = 0) -> int:
    """Print a numbered menu and return the user's selection index.

    :param title: Header text for the menu.
    :type title: str
    :param items: List of option labels.
    :type items: list[str]
    :param default: Default selection index (0-based).
    :type default: int
    :return: Selected index (0-based).
    :rtype: int
    """
    print(f"\n{'─' * 50}")
    print(f"  {title}")
    print(f"{'─' * 50}")
    for i, item in enumerate(items):
        marker = " *" if i == default else ""
        print(f"  [{i + 1}] {item}{marker}")
    print()

    while True:
        raw = input(f"  Choice [default={default + 1}]: ").strip()
        if not raw:
            return default
        try:
            choice = int(raw)
            if 1 <= choice <= len(items):
                return choice - 1
        except ValueError:
            pass
        print(f"  Please enter a number between 1 and {len(items)}.")


# ============================================================================
# Rerun logging helpers
# ============================================================================


def log_segments_to_rerun(
    img_u8: np.ndarray,
    segments: list[Any],
    detector_name: str,
    image_data_desc: list[Any] | None = None,
    image_data_layers: list[Any] | None = None,
) -> None:
    """Log line segment detection results to the active Rerun recording.

    Logs the following entities:

    * ``image/original`` — the input grayscale image.
    * ``image/segments`` — detected line segments as colored 2-D line strips.
    * ``image/endpoints`` — start/end points for each segment.
    * ``image/data/<layer>`` — auxiliary image data layers (gradient, edges, …).
    * ``logs`` — summary text messages.

    :param img_u8: Grayscale input image (H×W, uint8).
    :type img_u8: numpy.ndarray
    :param segments: List of detected ``LineSegment`` objects.
    :type segments: list
    :param detector_name: Name of the detector (for annotations).
    :type detector_name: str
    :param image_data_desc: Optional list of data descriptor entries.
    :type image_data_desc: list or None
    :param image_data_layers: Optional list of auxiliary image arrays.
    :type image_data_layers: list or None
    """
    # --- Input image -------------------------------------------------------
    rr.log("image/original", rr.Image(img_u8))
    rr.log(
        "logs",
        rr.TextLog(
            f"Image: {img_u8.shape[1]}x{img_u8.shape[0]} — Detector: {detector_name}"
        ),
    )

    num_segments = len(segments)

    if num_segments == 0:
        rr.log("logs", rr.TextLog("No segments detected.", level=rr.TextLogLevel.WARN))
        return

    # --- Line segments as 2-D line strips -----------------------------------
    strips: list[list[list[float]]] = []
    points: list[list[float]] = []
    seg_labels: list[str] = []
    ep_labels: list[str] = []

    for i, seg in enumerate(segments):
        ep = seg.end_points()  # (x1, y1, x2, y2)
        x1, y1, x2, y2 = float(ep[0]), float(ep[1]), float(ep[2]), float(ep[3])
        strips.append([[x1, y1], [x2, y2]])
        points.append([x1, y1])
        points.append([x2, y2])

        angle_deg = math.degrees(seg.angle)
        seg_labels.append(f"#{i}  L={seg.length:.1f}  θ={angle_deg:.1f}°")
        ep_labels.append(f"#{i} start ({x1:.0f}, {y1:.0f})")
        ep_labels.append(f"#{i} end ({x2:.0f}, {y2:.0f})")

    colors = _generate_colors(num_segments)
    rr.log(
        "image/segments",
        rr.LineStrips2D(
            strips,
            colors=colors,
            labels=seg_labels,
            show_labels=False,
            radii=1.5,
        ),
    )
    rr.log("logs", rr.TextLog(f"Found {num_segments} line segments"))

    # --- Endpoints as 2D points --------------------------------------------
    rr.log(
        "image/endpoints",
        rr.Points2D(
            points,
            colors=[255, 60, 60],
            labels=ep_labels,
            show_labels=False,
            radii=2.5,
        ),
    )

    # --- Segment statistics ------------------------------------------------
    lengths = [seg.length for seg in segments]
    rr.log(
        "logs",
        rr.TextLog(
            f"Length range: [{min(lengths):.1f}, {max(lengths):.1f}]  "
            f"Mean: {sum(lengths) / len(lengths):.1f}"
        ),
    )

    # --- Auxiliary image data layers ----------------------------------------
    if image_data_desc and image_data_layers:
        n_layers = min(len(image_data_desc), len(image_data_layers))
        for i in range(n_layers):
            layer_name = image_data_desc[i].name
            layer = image_data_layers[i]
            # Normalize float layers to uint8 for visualization
            layer_f = layer.astype(np.float64)
            if layer_f.max() > layer_f.min():
                layer_norm = (
                    (layer_f - layer_f.min()) / (layer_f.max() - layer_f.min()) * 255
                ).astype(np.uint8)
            else:
                layer_norm = np.zeros_like(layer, dtype=np.uint8)
            rr.log(f"image/data/{layer_name}", rr.Image(layer_norm))
            rr.log("logs", rr.TextLog(f"Logged auxiliary layer: {layer_name}"))


# ============================================================================
# Main workflow
# ============================================================================


def main() -> None:
    """Run the Rerun LSD visualization demo."""
    parser = argparse.ArgumentParser(
        description="Visualize LSD line segment detection results in Rerun."
    )
    parser.add_argument(
        "image",
        nargs="?",
        default=None,
        help="Path to input image (default: windmill.jpg).",
    )
    parser.add_argument(
        "--detector",
        type=str,
        default=None,
        choices=list(DETECTORS.keys()),
        help="Detector name (skip interactive menu). Default: interactive.",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Run ALL detectors and log each as a separate timeline step.",
    )
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        metavar="PATH",
        help="Save recording to an .rrd file instead of spawning the viewer.",
    )
    parser.add_argument(
        "--connect",
        type=str,
        default=None,
        metavar="ADDR",
        help="Connect to a running Rerun Viewer at the given gRPC address"
        " (e.g. rerun+http://127.0.0.1:9876).",
    )
    parser.add_argument(
        "--serve",
        action="store_true",
        help="Start a web-based Rerun Viewer (open in browser).  "
        "Ideal for WSL / headless environments.",
    )
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Load image
    # ------------------------------------------------------------------
    image_path: str = args.image if args.image else _resolve_default_image()
    img_u8 = _load_image(image_path)
    print(f"Loaded image: {image_path}  ({img_u8.shape[1]}x{img_u8.shape[0]})")

    # ------------------------------------------------------------------
    # Initialize Rerun
    # ------------------------------------------------------------------
    rr.init("rerun_lsd_demo", spawn=False)

    if args.save:
        rr.save(args.save)
        print(f"Recording will be saved to: {args.save}")
    elif args.connect:
        rr.connect_grpc(args.connect)
        print(f"Streaming to Rerun Viewer at {args.connect}")
    elif args.serve:
        server_uri = rr.serve_grpc()
        rr.serve_web_viewer(open_browser=False, connect_to=server_uri)
        # URL-encode '+' and replace 127.0.0.1 for WSL/remote access
        from urllib.parse import quote  # noqa: C0415

        viewer_url = quote(server_uri.replace("127.0.0.1", "localhost"), safe="/:?")
        print(f"Rerun web viewer serving at http://localhost:9090/?url={viewer_url}")
    else:
        rr.spawn()
        print("Spawned Rerun Viewer.")

    # ------------------------------------------------------------------
    # Select and run detector(s)
    # ------------------------------------------------------------------
    if args.all:
        detector_keys = list(DETECTORS.keys())
    elif args.detector:
        detector_keys = [args.detector]
    else:
        names = list(DETECTORS.keys())
        idx = _numbered_menu("Select line segment detector", names, default=0)
        detector_keys = [names[idx]]

    for step, det_key in enumerate(detector_keys):
        det_info = DETECTORS[det_key]
        cls_name = det_info["class"]
        cls = getattr(le_lsd, cls_name)
        det = cls()

        print(f"\n{'─' * 50}")
        print(f"  [{step + 1}/{len(detector_keys)}] {cls_name} — {det_info['desc']}")
        print(f"{'─' * 50}")

        # Use timeline to separate multiple detectors
        rr.set_time("detector_step", sequence=step)

        print("  Detecting... ", end="", flush=True)
        det.detect(img_u8)
        segments = det.line_segments()
        print(f"done. ({len(segments)} segments)")

        # Retrieve auxiliary image data
        try:
            desc = det.image_data_descriptor()
            data = det.image_data()
        except Exception:
            desc, data = None, None

        log_segments_to_rerun(
            img_u8,
            segments,
            detector_name=f"{cls_name} ({det_key})",
            image_data_desc=desc,
            image_data_layers=data,
        )

    print("\nDone. Explore the results in the Rerun Viewer!")

    if not args.save:
        try:
            input("\nPress Enter to exit...")
        except (EOFError, KeyboardInterrupt):
            pass


if __name__ == "__main__":
    main()
